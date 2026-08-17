#include "drone_session.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <jpeglib.h>
#include <netinet/ip.h>
#include <setjmp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <utility>

#include "wifi8k_jpeg_assembler.hpp"

namespace tdg::direct {
namespace {
constexpr std::size_t kCaptureCapacity = 96u * 1024u;
constexpr auto kVideoRestartInterval = std::chrono::milliseconds(350);
constexpr auto kVideoAliveWindow = std::chrono::milliseconds(1000);

std::uint64_t steady_us() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

void set_socket_int(int socket, int level, int option, int value) {
  if (socket >= 0) setsockopt(socket, level, option, &value, sizeof(value));
}

struct JpegError {
  jpeg_error_mgr base;
  jmp_buf jump;
};

void jpeg_error(j_common_ptr context) {
  longjmp(reinterpret_cast<JpegError*>(context->err)->jump, 1);
}

std::uint8_t escape_marker(std::uint8_t value) {
  return value == 0x66 || value == 0x99 ? static_cast<std::uint8_t>(value + 1) : value;
}
}  // namespace

DroneSession::DroneSession(std::string drone_ip) : drone_ip_(std::move(drone_ip)) {
  in_addr parsed{};
  if (inet_pton(AF_INET, drone_ip_.c_str(), &parsed) == 1) drone_addr_ = parsed.s_addr;

  control_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (control_socket_ >= 0) {
    // Keep commands out of a deep local send queue. IP_TOS is advisory but can
    // also map to a low-delay Wi-Fi access category on supporting systems/APs.
    set_socket_int(control_socket_, SOL_SOCKET, SO_SNDBUF, 4096);
    set_socket_int(control_socket_, IPPROTO_IP, IP_TOS, IPTOS_LOWDELAY);
  }

  latest_jpeg_.reserve(kCaptureCapacity);
  decode_thread_ = std::thread(&DroneSession::decode_loop, this);
  video_thread_ = std::thread(&DroneSession::video_loop, this);
}

DroneSession::~DroneSession() {
  running_.store(false, std::memory_order_release);
  jpeg_cv_.notify_all();
  if (video_thread_.joinable()) video_thread_.join();
  if (decode_thread_.joinable()) decode_thread_.join();
  if (control_socket_ >= 0) close(control_socket_);
}

void DroneSession::send_controls(std::uint8_t roll, std::uint8_t pitch, std::uint8_t throttle,
                                 std::uint8_t yaw, std::uint8_t flags) {
  if (control_socket_ < 0 || drone_addr_ == 0) return;

  const auto eroll = escape_marker(roll);
  const auto epitch = escape_marker(pitch);
  const auto ethrottle = escape_marker(throttle);
  const auto eyaw = escape_marker(yaw);
  const std::uint8_t packet[] = {
      0x66, eroll, epitch, ethrottle, eyaw, flags,
      escape_marker(static_cast<std::uint8_t>(eroll ^ epitch ^ ethrottle ^ eyaw ^ flags)),
      0x99};

  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(8090);
  destination.sin_addr.s_addr = drone_addr_;
  (void)sendto(control_socket_, packet, sizeof(packet), MSG_DONTWAIT,
               reinterpret_cast<sockaddr*>(&destination), sizeof(destination));
}

void DroneSession::command(CameraCommand command) {
  if (command == CameraCommand::Start) video_requested_.store(true, std::memory_order_release);
  if (command == CameraCommand::Stop) video_requested_.store(false, std::memory_order_release);
  pending_command_.store(static_cast<std::uint16_t>(command), std::memory_order_release);
}

bool DroneSession::take_latest(RgbFrame& frame) {
  std::scoped_lock guard(frame_lock_);
  if (!latest_ready_) return false;
  std::swap(frame, latest_);
  latest_ready_ = false;
  return true;
}

SessionStats DroneSession::stats() const {
  SessionStats result{};
  result.datagrams = datagrams_.load(std::memory_order_relaxed);
  result.frames = frames_.load(std::memory_order_relaxed);
  result.discarded = discarded_.load(std::memory_order_relaxed);
  result.decode_errors = decode_errors_.load(std::memory_order_relaxed);
  result.dropped_jpegs = dropped_jpegs_.load(std::memory_order_relaxed);
  result.video_restarts = video_restarts_.load(std::memory_order_relaxed);

  const auto last = last_frame_us_.load(std::memory_order_relaxed);
  const auto now = steady_us();
  const auto alive_us = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(kVideoAliveWindow).count());
  result.video_seen = last != 0 && now >= last && now - last <= alive_us;
  return result;
}

void DroneSession::set_video_enhancement(bool enabled, bool denoise, float contrast,
                                         float saturation) noexcept {
  // Keep user-provided values in a conservative range. Extreme settings are both
  // visually destructive and more likely to clip low-cost camera footage.
  enhance_enabled_.store(enabled, std::memory_order_relaxed);
  denoise_enabled_.store(denoise, std::memory_order_relaxed);
  enhance_contrast_.store(std::clamp(contrast, 0.85f, 1.30f), std::memory_order_relaxed);
  enhance_saturation_.store(std::clamp(saturation, 0.0f, 1.40f), std::memory_order_relaxed);
}

void DroneSession::on_jpeg(const std::uint8_t* jpeg, std::size_t size, void* user) {
  auto* session = static_cast<DroneSession*>(user);
  if (jpeg == nullptr || size < 4 || size > kCaptureCapacity) return;

  {
    std::scoped_lock guard(session->jpeg_lock_);
    if (session->jpeg_ready_) {
      session->dropped_jpegs_.fetch_add(1, std::memory_order_relaxed);
    }
    session->latest_jpeg_.assign(jpeg, jpeg + size);
    session->jpeg_ready_ = true;
  }
  session->last_jpeg_us_.store(steady_us(), std::memory_order_relaxed);
  session->jpeg_cv_.notify_one();
}

void DroneSession::decode_loop() {
  std::vector<std::uint8_t> jpeg_data;
  jpeg_data.reserve(kCaptureCapacity);
  std::vector<std::uint8_t> enhance_scratch;
  RgbFrame decoded;

  while (running_.load(std::memory_order_acquire)) {
    {
      std::unique_lock lock(jpeg_lock_);
      jpeg_cv_.wait(lock, [&] {
        return !running_.load(std::memory_order_acquire) || jpeg_ready_;
      });
      if (!running_.load(std::memory_order_acquire)) break;
      jpeg_data.swap(latest_jpeg_);
      jpeg_ready_ = false;
    }

    if (!decode_jpeg(jpeg_data.data(), jpeg_data.size(), decoded)) {
      decode_errors_.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    // Enhancement is deliberately kept off the render/control thread. The
    // scratch allocation is retained across frames, so steady-state processing
    // performs no heap allocation.
    if (enhance_enabled_.load(std::memory_order_relaxed)) {
      enhance_frame(decoded, enhance_scratch);
    }

    decoded.number = static_cast<std::uint64_t>(
        frames_.fetch_add(1, std::memory_order_relaxed)) + 1;
    {
      std::scoped_lock guard(frame_lock_);
      std::swap(decoded, latest_);  // stale RGB frame is dropped and its buffer recycled
      latest_ready_ = true;
    }
    last_frame_us_.store(steady_us(), std::memory_order_relaxed);
  }
}

bool DroneSession::decode_jpeg(const std::uint8_t* jpeg, std::size_t size, RgbFrame& frame) {
  if (jpeg == nullptr || size < 4) return false;

  jpeg_decompress_struct decoder{};
  JpegError error{};
  decoder.err = jpeg_std_error(&error.base);
  error.base.error_exit = jpeg_error;
  if (setjmp(error.jump)) {
    jpeg_destroy_decompress(&decoder);
    return false;
  }

  jpeg_create_decompress(&decoder);
  jpeg_mem_src(&decoder, jpeg, size);
  if (jpeg_read_header(&decoder, TRUE) != JPEG_HEADER_OK) {
    jpeg_destroy_decompress(&decoder);
    return false;
  }

  decoder.out_color_space = JCS_RGB;
  decoder.dct_method = JDCT_IFAST;
  decoder.do_fancy_upsampling = FALSE;
  decoder.do_block_smoothing = FALSE;
  if (!jpeg_start_decompress(&decoder)) {
    jpeg_destroy_decompress(&decoder);
    return false;
  }

  // Corrupt dimensions should never be allowed to trigger an enormous allocation.
  constexpr std::size_t kMaxDimension = 4096;
  if (decoder.output_width == 0 || decoder.output_height == 0 ||
      decoder.output_width > kMaxDimension || decoder.output_height > kMaxDimension ||
      decoder.output_components != 3) {
    jpeg_abort_decompress(&decoder);
    jpeg_destroy_decompress(&decoder);
    return false;
  }

  frame.width = static_cast<int>(decoder.output_width);
  frame.height = static_cast<int>(decoder.output_height);
  const std::size_t row_bytes = static_cast<std::size_t>(frame.width) * 3;
  frame.pixels.resize(row_bytes * static_cast<std::size_t>(frame.height));

  std::array<JSAMPROW, 16> rows{};
  while (decoder.output_scanline < decoder.output_height) {
    const auto remaining = decoder.output_height - decoder.output_scanline;
    const JDIMENSION batch = std::min<JDIMENSION>(remaining, rows.size());
    for (JDIMENSION i = 0; i < batch; ++i) {
      rows[i] = frame.pixels.data() +
                static_cast<std::size_t>(decoder.output_scanline + i) * row_bytes;
    }
    if (jpeg_read_scanlines(&decoder, rows.data(), batch) == 0) {
      jpeg_abort_decompress(&decoder);
      jpeg_destroy_decompress(&decoder);
      return false;
    }
  }

  if (!jpeg_finish_decompress(&decoder)) {
    jpeg_destroy_decompress(&decoder);
    return false;
  }
  jpeg_destroy_decompress(&decoder);
  return true;
}

void DroneSession::enhance_frame(RgbFrame& frame, std::vector<std::uint8_t>& scratch) const {
  if (frame.width < 3 || frame.height < 3 || frame.pixels.empty()) return;

  const int width = frame.width;
  const int height = frame.height;
  const std::size_t bytes = static_cast<std::size_t>(width) *
                            static_cast<std::size_t>(height) * 3u;
  if (frame.pixels.size() < bytes) return;

  auto* dst = frame.pixels.data();

  // A small edge-aware cross filter suppresses isolated sensor/JPEG chroma noise
  // without the blanket 3x3 Gaussian blur used previously. Neighbours across a
  // meaningful luminance edge are rejected, and the centre pixel retains most
  // of the weight even in perfectly flat areas.
  if (denoise_enabled_.load(std::memory_order_relaxed)) {
    scratch.resize(bytes);
    std::memcpy(scratch.data(), dst, bytes);
    const auto* src = scratch.data();
    constexpr int kEdgeThreshold = 18;

    const auto luma = [](const std::uint8_t* p) noexcept {
      return (77 * static_cast<int>(p[0]) + 150 * static_cast<int>(p[1]) +
              29 * static_cast<int>(p[2]) + 128) >> 8;
    };

    for (int y = 1; y < height - 1; ++y) {
      const std::size_t row = static_cast<std::size_t>(y) * width * 3u;
      for (int x = 1; x < width - 1; ++x) {
        const std::size_t i = row + static_cast<std::size_t>(x) * 3u;
        const auto* center = src + i;
        const int center_y = luma(center);

        const std::uint8_t* neighbours[4] = {
            center - 3,
            center + 3,
            center - static_cast<std::ptrdiff_t>(width) * 3,
            center + static_cast<std::ptrdiff_t>(width) * 3,
        };

        int sum_r = static_cast<int>(center[0]) * 2;
        int sum_g = static_cast<int>(center[1]) * 2;
        int sum_b = static_cast<int>(center[2]) * 2;
        int count = 2;
        for (const auto* neighbour : neighbours) {
          if (std::abs(luma(neighbour) - center_y) > kEdgeThreshold) continue;
          sum_r += neighbour[0];
          sum_g += neighbour[1];
          sum_b += neighbour[2];
          ++count;
        }

        // Blend the edge-aware local estimate back toward the original. In a
        // flat region this leaves roughly two thirds of the centre sample,
        // preserving texture while reducing one-pixel noise.
        const int avg_r = sum_r / count;
        const int avg_g = sum_g / count;
        const int avg_b = sum_b / count;
        int out_r = (static_cast<int>(center[0]) + avg_r + 1) >> 1;
        int out_g = (static_cast<int>(center[1]) + avg_g + 1) >> 1;
        int out_b = (static_cast<int>(center[2]) + avg_b + 1) >> 1;

        // Restore half of the centre pixel's luminance detail. Chroma noise is
        // still smoothed, but edges/fine monochrome texture remain much closer
        // to the original instead of acquiring a soft-focus look.
        const int filtered_y = (77 * out_r + 150 * out_g + 29 * out_b + 128) >> 8;
        const int detail = (center_y - filtered_y) / 2;
        out_r = std::clamp(out_r + detail, 0, 255);
        out_g = std::clamp(out_g + detail, 0, 255);
        out_b = std::clamp(out_b + detail, 0, 255);
        dst[i + 0] = static_cast<std::uint8_t>(out_r);
        dst[i + 1] = static_cast<std::uint8_t>(out_g);
        dst[i + 2] = static_cast<std::uint8_t>(out_b);
      }
    }
  }

  const float contrast = enhance_contrast_.load(std::memory_order_relaxed);
  const float saturation = enhance_saturation_.load(std::memory_order_relaxed);
  if (std::abs(contrast - 1.0f) < 0.001f && std::abs(saturation - 1.0f) < 0.001f) return;

  // Cache the tone curve per decode thread; it only needs rebuilding when the
  // enhancement strength changes, not for every frame.
  thread_local std::array<std::uint8_t, 256> tone{};
  thread_local float tone_contrast = -1.0f;
  if (std::abs(tone_contrast - contrast) > 0.0005f) {
    tone_contrast = contrast;
    for (int i = 0; i < 256; ++i) {
      const float x = static_cast<float>(i) / 255.0f;
      const float lifted = std::pow(x, 0.97f);
      const float centered = lifted - 0.5f;
      const float shape = std::max(0.0f, 1.0f - 4.0f * centered * centered);
      const float curved = lifted + (contrast - 1.0f) * 0.70f * centered * shape;
      tone[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(
          std::clamp(std::lround(curved * 255.0f), 0l, 255l));
    }
  }

  const int sat_q8 = static_cast<int>(std::lround(saturation * 256.0f));
  for (std::size_t i = 0; i < bytes; i += 3) {
    const int r = dst[i + 0];
    const int g = dst[i + 1];
    const int b = dst[i + 2];
    const int y = (77 * r + 150 * g + 29 * b + 128) >> 8;
    const int y2 = tone[static_cast<std::size_t>(y)];

    // Saturation is applied around luminance, then the tone delta is added to
    // all channels. This keeps perceived brightness stable and avoids the dark,
    // muddy result of independent RGB contrast scaling.
    const int rr = y2 + (((r - y) * sat_q8 + 128) >> 8);
    const int gg = y2 + (((g - y) * sat_q8 + 128) >> 8);
    const int bb = y2 + (((b - y) * sat_q8 + 128) >> 8);
    dst[i + 0] = static_cast<std::uint8_t>(std::clamp(rr, 0, 255));
    dst[i + 1] = static_cast<std::uint8_t>(std::clamp(gg, 0, 255));
    dst[i + 2] = static_cast<std::uint8_t>(std::clamp(bb, 0, 255));
  }
}

void DroneSession::send_video_command(int socket, CameraCommand command) {
  if (socket < 0 || drone_addr_ == 0) return;
  const std::uint8_t packet[] = {0x42, static_cast<std::uint8_t>(command)};
  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(8080);
  destination.sin_addr.s_addr = drone_addr_;
  (void)sendto(socket, packet, sizeof(packet), MSG_DONTWAIT,
               reinterpret_cast<sockaddr*>(&destination), sizeof(destination));
}

void DroneSession::video_loop() {
  const int video_socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (video_socket < 0) return;

  set_socket_int(video_socket, SOL_SOCKET, SO_REUSEADDR, 1);
  set_socket_int(video_socket, SOL_SOCKET, SO_RCVBUF, 256 * 1024);
  set_socket_int(video_socket, IPPROTO_IP, IP_TOS, IPTOS_LOWDELAY);

  timeval timeout{};
  timeout.tv_usec = 10000;  // wake quickly for commands/restart/shutdown
  setsockopt(video_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  sockaddr_in bind_address{};
  bind_address.sin_family = AF_INET;
  bind_address.sin_addr.s_addr = INADDR_ANY;
  bind_address.sin_port = 0;
  if (bind(video_socket, reinterpret_cast<sockaddr*>(&bind_address), sizeof(bind_address)) != 0) {
    close(video_socket);
    return;
  }

  std::array<std::uint8_t, kCaptureCapacity> capture{};
  Wifi8kJpegAssembler assembler(capture.data(), capture.size(), on_jpeg, this);
  std::array<std::uint8_t, 1472> datagram{};

  // Do not sleep through the first camera packets. The old startup sequence sent
  // seven commands with 80 ms sleeps, allowing more than half a second of stale
  // UDP data to accumulate before reception began.
  send_video_command(video_socket, CameraCommand::Start);
  video_restarts_.fetch_add(1, std::memory_order_relaxed);
  auto last_restart = std::chrono::steady_clock::now();

  while (running_.load(std::memory_order_acquire)) {
    const auto command = pending_command_.exchange(0, std::memory_order_acq_rel);
    if (command != 0) send_video_command(video_socket, static_cast<CameraCommand>(command));

    const int received = recv(video_socket, datagram.data(), datagram.size(), 0);
    const auto now = std::chrono::steady_clock::now();
    if (received > 0) {
      assembler.consume(datagram.data(), static_cast<std::size_t>(received), steady_us());
      const auto& source = assembler.stats();
      datagrams_.store(source.datagrams, std::memory_order_relaxed);
      discarded_.store(source.discarded + source.overflows, std::memory_order_relaxed);
    } else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      // Keep the session alive; transient Wi-Fi/socket errors often clear without
      // recreating application state. A Start command below also re-primes video.
    }

    const auto last_jpeg = last_jpeg_us_.load(std::memory_order_relaxed);
    const auto restart_us = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(kVideoRestartInterval).count());
    const auto now_us = steady_us();
    const bool jpeg_stale = last_jpeg == 0 || now_us < last_jpeg || now_us - last_jpeg > restart_us;
    if (video_requested_.load(std::memory_order_acquire) && jpeg_stale &&
        now - last_restart >= kVideoRestartInterval) {
      send_video_command(video_socket, CameraCommand::Start);
      video_restarts_.fetch_add(1, std::memory_order_relaxed);
      last_restart = now;
    }
  }

  close(video_socket);
}

}  // namespace tdg::direct
