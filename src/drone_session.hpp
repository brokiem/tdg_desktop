#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace tdg::direct {

struct RgbFrame {
  int width{};
  int height{};
  std::vector<std::uint8_t> pixels;
  std::uint64_t number{};
};

struct SessionStats {
  bool video_seen{};
  std::uint32_t datagrams{};
  std::uint32_t frames{};
  std::uint32_t discarded{};
  std::uint32_t decode_errors{};
  std::uint32_t dropped_jpegs{};
  std::uint32_t video_restarts{};
};

enum class CameraCommand : std::uint16_t {
  Start = 0x76,
  Stop = 0x77,
  Rotate = 0x78,
  Toggle = 0x79,
};

class DroneSession {
 public:
  explicit DroneSession(std::string drone_ip = "192.168.4.153");
  ~DroneSession();
  DroneSession(const DroneSession&) = delete;
  DroneSession& operator=(const DroneSession&) = delete;

  void send_controls(std::uint8_t roll, std::uint8_t pitch, std::uint8_t throttle,
                     std::uint8_t yaw, std::uint8_t flags);
  void command(CameraCommand command);
  bool take_latest(RgbFrame& frame);
  SessionStats stats() const;
  void set_video_enhancement(bool enabled, bool denoise, float contrast, float saturation) noexcept;

 private:
  static void on_jpeg(const std::uint8_t* jpeg, std::size_t size, void* user);
  void video_loop();
  void decode_loop();
  bool decode_jpeg(const std::uint8_t* jpeg, std::size_t size, RgbFrame& frame);
  void enhance_frame(RgbFrame& frame, std::vector<std::uint8_t>& scratch) const;
  void send_video_command(int socket, CameraCommand command);

  std::string drone_ip_;
  std::uint32_t drone_addr_{};
  int control_socket_{-1};
  std::atomic<bool> running_{true};
  std::atomic<bool> video_requested_{true};
  std::atomic<std::uint16_t> pending_command_{0};

  std::thread video_thread_;
  std::thread decode_thread_;

  // Receive/reassembly never waits on JPEG decode. The receive thread only copies
  // the newest complete JPEG into this single-slot mailbox; stale compressed
  // frames are overwritten rather than queued.
  std::mutex jpeg_lock_;
  std::condition_variable jpeg_cv_;
  std::vector<std::uint8_t> latest_jpeg_;
  bool jpeg_ready_{};

  // Decoded RGB mailbox. take_latest() swaps buffers so allocations are recycled
  // between the decoder and renderer instead of recreated every frame.
  mutable std::mutex frame_lock_;
  RgbFrame latest_;
  bool latest_ready_{};

  std::atomic<std::uint32_t> datagrams_{};
  std::atomic<std::uint32_t> frames_{};
  std::atomic<std::uint32_t> discarded_{};
  std::atomic<std::uint32_t> decode_errors_{};
  std::atomic<std::uint32_t> dropped_jpegs_{};
  std::atomic<std::uint32_t> video_restarts_{};
  std::atomic<std::uint64_t> last_frame_us_{};
  std::atomic<std::uint64_t> last_jpeg_us_{};

  // Optional post-processing runs on the decode worker so the UI/control thread
  // never pays for image cleanup. Atomics allow lock-free live tuning.
  std::atomic<bool> enhance_enabled_{true};
  std::atomic<bool> denoise_enabled_{true};
  std::atomic<float> enhance_contrast_{1.07f};
  std::atomic<float> enhance_saturation_{1.06f};
};

}  // namespace tdg::direct
