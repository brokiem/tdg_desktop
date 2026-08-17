#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace tdg::direct {

struct AssemblerStats {
  std::uint32_t datagrams{};
  std::uint32_t frames{};
  std::uint32_t discarded{};
  std::uint32_t overflows{};
};

using JpegCallback = void (*)(const std::uint8_t*, std::size_t, void*);

class Wifi8kJpegAssembler {
 public:
  Wifi8kJpegAssembler(std::uint8_t* buffer, std::size_t capacity, JpegCallback callback, void* user)
      : buffer_(buffer), capacity_(capacity), callback_(callback), user_(user) {}

  void consume(const std::uint8_t* datagram, std::size_t datagram_size, std::uint64_t now_us) {
    ++stats_.datagrams;
    if (buffer_ == nullptr || capacity_ < 2 || datagram == nullptr ||
        datagram_size <= kHeaderSize || datagram_size > kMaxDatagram || datagram[1] > 1) {
      previous_payload_ended_ff_ = false;
      previous_payload_us_ = 0;
      return;
    }

    const auto* payload = datagram + kHeaderSize;
    const std::size_t payload_size = datagram_size - kHeaderSize;
    if (payload_size == 0) return;

    if (active_ && (now_us < started_us_ || now_us - started_us_ > kFrameTimeoutUs)) discard();

    // A fresh SOI always wins. This is the fastest recovery from packet loss and
    // prevents an old partial frame from delaying a newer complete frame.
    const bool split_soi = active_ && size_ != 0 && buffer_[size_ - 1] == 0xff && payload[0] == 0xd8;
    const auto soi = find_pair(payload, payload_size, 0xd8);
    if (split_soi) {
      discard();
      start_frame(now_us);
      const std::uint8_t marker[2] = {0xff, 0xd8};
      if (!append(marker, sizeof(marker))) return;
      if (payload_size > 1 && !append(payload + 1, payload_size - 1)) return;
    } else if (soi != npos) {
      if (active_) discard();
      start_frame(now_us);
      if (!append(payload + soi, payload_size - soi)) return;
    } else if (active_) {
      if (!append(payload, payload_size)) return;
    } else if (previous_payload_ended_ff_ && payload[0] == 0xd8 &&
               previous_payload_us_ != 0 && now_us >= previous_payload_us_ &&
               now_us - previous_payload_us_ <= kSplitMarkerWindowUs) {
      // Handle the rare case where SOI is split across adjacent UDP payloads
      // while idle, but do not bridge unrelated datagrams after a long gap.
      start_frame(now_us);
      const std::uint8_t marker[2] = {0xff, 0xd8};
      if (!append(marker, sizeof(marker))) return;
      if (payload_size > 1 && !append(payload + 1, payload_size - 1)) return;
    }

    previous_payload_ended_ff_ = payload[payload_size - 1] == 0xff;
    previous_payload_us_ = now_us;
    if (!active_) return;

    const auto end = parse_available();
    if (end == malformed) {
      discard();
      return;
    }
    if (end == npos) return;

    size_ = end;
    if (callback_) callback_(buffer_, size_, user_);
    ++stats_.frames;
    reset_frame();
  }

  const AssemblerStats& stats() const { return stats_; }

 private:
  static constexpr std::size_t kHeaderSize = 8;
  static constexpr std::size_t kMaxDatagram = 1472;
  static constexpr std::uint64_t kFrameTimeoutUs = 50000;
  static constexpr std::uint64_t kSplitMarkerWindowUs = 20000;
  static constexpr std::size_t npos = std::numeric_limits<std::size_t>::max();
  static constexpr std::size_t malformed = npos - 1;

  static std::size_t find_pair(const std::uint8_t* bytes, std::size_t size, std::uint8_t second) {
    if (bytes == nullptr || size < 2) return npos;
    const std::uint8_t* cursor = bytes;
    const std::uint8_t* const last = bytes + size - 1;
    while (cursor < last) {
      const auto remaining = static_cast<std::size_t>(last - cursor);
      const auto* ff = static_cast<const std::uint8_t*>(std::memchr(cursor, 0xff, remaining));
      if (!ff) return npos;
      if (ff[1] == second) return static_cast<std::size_t>(ff - bytes);
      cursor = ff + 1;
    }
    return npos;
  }

  // Incremental JPEG marker parser. parse_pos_ only moves forward, so each byte
  // is examined at most once instead of rescanning the complete JPEG after every
  // UDP datagram. It still validates segment lengths and entropy-coded markers,
  // so FF D9 inside metadata/stuffed scan data is not accepted as a false EOI.
  std::size_t parse_available() {
    if (size_ < 2) return npos;
    if (buffer_[0] != 0xff || buffer_[1] != 0xd8) return malformed;
    if (parse_pos_ < 2) parse_pos_ = 2;

    for (;;) {
      if (in_scan_) {
        while (parse_pos_ < size_) {
          if (buffer_[parse_pos_] != 0xff) {
            ++parse_pos_;
            continue;
          }

          const std::size_t marker_start = parse_pos_;
          while (parse_pos_ < size_ && buffer_[parse_pos_] == 0xff) ++parse_pos_;
          if (parse_pos_ == size_) {
            parse_pos_ = marker_start;
            return npos;
          }

          const auto marker = buffer_[parse_pos_++];
          if (marker == 0x00 || (marker >= 0xd0 && marker <= 0xd7)) continue;
          if (marker == 0xd9) return parse_pos_;

          // Progressive/multi-scan JPEGs may leave entropy data for another
          // marker segment. Reparse from the marker boundary in header mode.
          parse_pos_ = marker_start;
          in_scan_ = false;
          break;
        }
        if (in_scan_) return npos;
      }

      if (parse_pos_ >= size_) return npos;
      if (buffer_[parse_pos_] != 0xff) return malformed;

      const std::size_t marker_start = parse_pos_;
      while (parse_pos_ < size_ && buffer_[parse_pos_] == 0xff) ++parse_pos_;
      if (parse_pos_ == size_) {
        parse_pos_ = marker_start;
        return npos;
      }

      const auto marker = buffer_[parse_pos_++];
      if (marker == 0xd9) return parse_pos_;
      if (marker == 0xd8 || marker == 0x01 || (marker >= 0xd0 && marker <= 0xd7)) continue;

      if (size_ - parse_pos_ < 2) {
        parse_pos_ = marker_start;
        return npos;
      }
      const std::size_t segment = (static_cast<std::size_t>(buffer_[parse_pos_]) << 8) |
                                  buffer_[parse_pos_ + 1];
      if (segment < 2) return malformed;
      if (segment > size_ - parse_pos_) {
        parse_pos_ = marker_start;
        return npos;
      }

      parse_pos_ += segment;
      if (marker == 0xda) in_scan_ = true;
    }
  }

  void start_frame(std::uint64_t now_us) {
    active_ = true;
    size_ = 0;
    parse_pos_ = 0;
    in_scan_ = false;
    started_us_ = now_us;
  }

  bool append(const std::uint8_t* bytes, std::size_t count) {
    if (count > capacity_ - size_) {
      discard();
      ++stats_.overflows;
      return false;
    }
    std::memcpy(buffer_ + size_, bytes, count);
    size_ += count;
    return true;
  }

  void reset_frame() {
    active_ = false;
    size_ = 0;
    parse_pos_ = 0;
    in_scan_ = false;
  }

  void discard() {
    reset_frame();
    ++stats_.discarded;
  }

  std::uint8_t* buffer_{};
  std::size_t capacity_{};
  JpegCallback callback_{};
  void* user_{};
  std::size_t size_{};
  std::size_t parse_pos_{};
  std::uint64_t started_us_{};
  bool active_{};
  bool in_scan_{};
  bool previous_payload_ended_ff_{};
  std::uint64_t previous_payload_us_{};
  AssemblerStats stats_{};
};

}  // namespace tdg::direct
