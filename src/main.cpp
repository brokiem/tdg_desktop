#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <deque>
#include <cstdarg>

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>

#include "drone_session.hpp"

namespace {

// ── Helpers ──────────────────────────────────────────────────────────

std::uint8_t axis_to_wire(Sint16 value, bool invert = false) {
  float n = static_cast<float>(value) / 32767.0f;
  if (invert) n = -n;
  if (n > -0.08f && n < 0.08f) n = 0.0f;
  return static_cast<std::uint8_t>(std::clamp(128.0f + n * 127.0f, 0.0f, 255.0f));
}

std::uint8_t throttle_to_wire(Sint16 value, bool invert = false) {
  float n = static_cast<float>(value) / 32767.0f;
  if (invert) n = -n;
  if (n > -0.08f && n < 0.08f) n = 0.0f;
  // Xbox gamepad springs to center. Center maps to 0. Top maps to 255.
  return static_cast<std::uint8_t>(std::clamp(n * 255.0f, 0.0f, 255.0f));
}

std::deque<std::string> ui_logs;

void AddLog(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  ui_logs.push_front(buf);
  if (ui_logs.size() > 14) ui_logs.pop_back();
}

int GetRealWifiRssi() {
  // /proc is local and cheap, but doing iostream/string parsing in the draw path
  // still creates avoidable jitter. Sample at 1 Hz using fixed buffers instead.
  static auto last_check = std::chrono::steady_clock::time_point{};
  static int cached_rssi = 0;

  const auto now = std::chrono::steady_clock::now();
  if (last_check.time_since_epoch().count() != 0 &&
      now - last_check < std::chrono::seconds(1)) {
    return cached_rssi;
  }
  last_check = now;

  FILE* file = std::fopen("/proc/net/wireless", "r");
  if (!file) return cached_rssi;

  char line[256];
  while (std::fgets(line, sizeof(line), file)) {
    char* colon = std::strchr(line, ':');
    if (!colon) continue;

    char status[32]{};
    float quality = 0.0f;
    float level = 0.0f;
    if (std::sscanf(colon + 1, "%31s %f %f", status, &quality, &level) != 3) continue;

    if (level < 0.0f) {
      cached_rssi = std::clamp(static_cast<int>((level + 90.0f) * (100.0f / 60.0f)), 0, 100);
    } else if (quality > 0.0f && quality <= 70.0f) {
      cached_rssi = std::clamp(static_cast<int>((quality / 70.0f) * 100.0f), 0, 100);
    }
    break;
  }
  std::fclose(file);
  return cached_rssi;
}

GLuint create_texture() {
  GLuint tex{};
  glGenTextures(1, &tex);
  glBindTexture(GL_TEXTURE_2D, tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  return tex;
}

GLuint create_scanline_texture() {
  // One repeating 1x4 texture replaces hundreds of AddLine() calls per frame.
  // Alpha is intentionally low; the goal is field texture, not a CRT mask.
  constexpr std::uint8_t pixels[4 * 4] = {
      0, 0, 0, 10,
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0,
  };
  GLuint tex{};
  glGenTextures(1, &tex);
  glBindTexture(GL_TEXTURE_2D, tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 4, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
  return tex;
}

void DrawVideoImage(ImDrawList* dl, GLuint texture,
                    ImVec2 p0, ImVec2 p1, ImVec2 p2, ImVec2 p3,
                    bool rotated, bool analog_enabled) {
  const ImVec2 uv0 = rotated ? ImVec2{0.0f, 1.0f} : ImVec2{0.0f, 0.0f};
  const ImVec2 uv1 = rotated ? ImVec2{0.0f, 0.0f} : ImVec2{1.0f, 0.0f};
  const ImVec2 uv2 = rotated ? ImVec2{1.0f, 0.0f} : ImVec2{1.0f, 1.0f};
  const ImVec2 uv3 = rotated ? ImVec2{1.0f, 1.0f} : ImVec2{0.0f, 1.0f};
  const ImTextureID id = static_cast<ImTextureID>(texture);

  if (!analog_enabled) {
    dl->AddImageQuad(id, p0, p1, p2, p3, uv0, uv1, uv2, uv3, IM_COL32_WHITE);
    return;
  }

  dl->PushClipRect(p0, p2, true);

  // 1. Warm Color Grade
  // Keep the base image bright, just a tiny hint of warmth
  dl->AddImageQuad(id, p0, p1, p2, p3, uv0, uv1, uv2, uv3, IM_COL32(255, 252, 248, 255));

  // 2. Chromatic Aberration & Lens Diffusion
  // Tints must be close to white so standard alpha blending doesn't crush the image brightness
  dl->AddImageQuad(id, 
      {p0.x - 1.5f, p0.y}, {p1.x - 1.5f, p1.y}, 
      {p2.x - 1.5f, p2.y}, {p3.x - 1.5f, p3.y}, 
      uv0, uv1, uv2, uv3, IM_COL32(200, 255, 255, 20)); // Cyan fringe

  dl->AddImageQuad(id, 
      {p0.x + 1.5f, p0.y}, {p1.x + 1.5f, p1.y}, 
      {p2.x + 1.5f, p2.y}, {p3.x + 1.5f, p3.y}, 
      uv0, uv1, uv2, uv3, IM_COL32(255, 200, 200, 20));  // Red fringe

  // 3. Halation / Bloom
  // Slightly scaled-up, highly transparent warm overlay to bleed highlights
  constexpr float bloom = 3.0f;
  dl->AddImageQuad(id, 
      {p0.x - bloom, p0.y - bloom}, {p1.x + bloom, p1.y - bloom}, 
      {p2.x + bloom, p2.y + bloom}, {p3.x - bloom, p3.y + bloom}, 
      uv0, uv1, uv2, uv3, IM_COL32(255, 220, 200, 18)); // Halation glow

  // 4. Lifted Blacks
  // A very faint bright wash over everything lifts true blacks without darkening whites
  dl->AddRectFilled(p0, p2, IM_COL32(230, 235, 255, 6));

  dl->PopClipRect();
}

std::uint64_t pack_control_state(std::uint8_t roll, std::uint8_t pitch,
                                 std::uint8_t throttle, std::uint8_t yaw,
                                 std::uint8_t flags) {
  return static_cast<std::uint64_t>(roll) |
         (static_cast<std::uint64_t>(pitch) << 8) |
         (static_cast<std::uint64_t>(throttle) << 16) |
         (static_cast<std::uint64_t>(yaw) << 24) |
         (static_cast<std::uint64_t>(flags) << 32);
}

bool ColoredButton(const char* label, const ImVec2& size, const ImVec4& col) {
  ImGui::PushStyleColor(ImGuiCol_Button, col);
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
      ImVec4(std::min(col.x + 0.12f, 1.0f), std::min(col.y + 0.12f, 1.0f),
             std::min(col.z + 0.12f, 1.0f), col.w));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive,
      ImVec4(col.x * 0.75f, col.y * 0.75f, col.z * 0.75f, col.w));
  bool pressed = ImGui::Button(label, size);
  ImGui::PopStyleColor(3);
  return pressed;
}

class AsyncRecorder {
 public:
  ~AsyncRecorder() { stop(); }
  AsyncRecorder(const AsyncRecorder&) = delete;
  AsyncRecorder& operator=(const AsyncRecorder&) = delete;
  AsyncRecorder() = default;

  bool start(int width, int height, const std::string& filename) {
    stop();
    if (width < 4 || height < 4 || filename.empty()) return false;

    width_ = width & ~1;
    height_ = height & ~1;
    const std::size_t bytes = static_cast<std::size_t>(width_) * height_ * 3u;
    pending_.assign(bytes, 0);
    worker_buffer_.assign(bytes, 0);

    char cmd[768];
    std::snprintf(cmd, sizeof(cmd),
        "ffmpeg -y -f rawvideo -pix_fmt rgb24 -s %dx%d -r 30 "
        "-i pipe:0 -vf vflip -c:v libx264 -preset ultrafast -crf 23 "
        "-pix_fmt yuv420p \"%s\" 2>/dev/null",
        width_, height_, filename.c_str());
    pipe_ = popen(cmd, "w");
    if (!pipe_) {
      pending_.clear();
      worker_buffer_.clear();
      return false;
    }

    dropped_.store(0, std::memory_order_relaxed);
    running_.store(true, std::memory_order_release);
    healthy_.store(true, std::memory_order_release);
    worker_ = std::thread(&AsyncRecorder::worker_loop, this);
    return true;
  }

  void stop() {
    running_.store(false, std::memory_order_release);
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    if (pipe_) {
      pclose(pipe_);
      pipe_ = nullptr;
    }
    {
      std::scoped_lock lock(lock_);
      pending_ready_ = false;
    }
    pending_.clear();
    worker_buffer_.clear();
    width_ = 0;
    height_ = 0;
  }

  bool submit(std::vector<std::uint8_t>& pixels) {
    if (!running_.load(std::memory_order_acquire) ||
        !healthy_.load(std::memory_order_acquire)) return false;

    const std::size_t expected = static_cast<std::size_t>(width_) * height_ * 3u;
    if (pixels.size() != expected) return false;

    {
      std::scoped_lock lock(lock_);
      if (pending_ready_) dropped_.fetch_add(1, std::memory_order_relaxed);
      std::swap(pixels, pending_);
      pending_ready_ = true;
    }
    cv_.notify_one();
    return true;
  }

  [[nodiscard]] bool healthy() const noexcept {
    return healthy_.load(std::memory_order_acquire);
  }

  [[nodiscard]] std::uint32_t dropped() const noexcept {
    return dropped_.load(std::memory_order_relaxed);
  }

 private:
  void worker_loop() {
    for (;;) {
      {
        std::unique_lock lock(lock_);
        cv_.wait(lock, [&] {
          return !running_.load(std::memory_order_acquire) || pending_ready_;
        });
        if (!pending_ready_ && !running_.load(std::memory_order_acquire)) break;
        if (!pending_ready_) continue;
        std::swap(worker_buffer_, pending_);
        pending_ready_ = false;
      }

      if (!pipe_) break;
      const std::size_t bytes = worker_buffer_.size();
      if (bytes == 0) continue;
      if (std::fwrite(worker_buffer_.data(), 1, bytes, pipe_) != bytes) {
        healthy_.store(false, std::memory_order_release);
        running_.store(false, std::memory_order_release);
        break;
      }
    }
  }

  FILE* pipe_{};
  int width_{};
  int height_{};
  std::mutex lock_;
  std::condition_variable cv_;
  std::thread worker_;
  std::vector<std::uint8_t> pending_;
  std::vector<std::uint8_t> worker_buffer_;
  bool pending_ready_{};
  std::atomic<bool> running_{false};
  std::atomic<bool> healthy_{true};
  std::atomic<std::uint32_t> dropped_{};
};

// ── Analog FPV presentation ─────────────────────────────────────────

void DrawAnalogFilter(ImDrawList* dl, GLuint scanline_texture,
                      ImVec2 vmin, ImVec2 vmax, bool enabled) {
  if (!enabled || scanline_texture == 0) return;

  const float w = vmax.x - vmin.x;
  const float h = vmax.y - vmin.y;
  if (w <= 2.0f || h <= 2.0f) return;

  dl->PushClipRect(vmin, vmax, true);

  // Sangat halus (very faint) texture to simulate organic analog grain, 
  // instead of harsh VHS scanlines.
  const float phase_px = std::fmod(static_cast<float>(ImGui::GetTime()) * 0.35f, 4.0f);
  const float v0 = phase_px * 0.25f;
  const float v1 = v0 + h * 0.25f;
  dl->AddImage(static_cast<ImTextureID>(scanline_texture), vmin, vmax,
               {0.0f, v0}, {1.0f, v1}, IM_COL32(255, 255, 255, 140));

  dl->PopClipRect();
}

// ── Analog Camera OSD Overlay ────────────────────────────────────────

void DrawOSD(ImDrawList* dl, ImFont* osd_font, ImVec2 vmin, ImVec2 vmax, float fps,
             const tdg::direct::SessionStats& stats, int tw, int th, int view_mode, float text_scale = 1.0f, bool is_recording = false) {
  // MAX7456-era FPV OSDs are predominantly white/light gray with a hard black
  // key, not saturated neon. Keep warning text only slightly warmer/brighter.
  const ImU32 osd   = IM_COL32(224, 226, 216, 235);
  const ImU32 dim   = IM_COL32(212, 214, 205, 125);
  const ImU32 warn  = IM_COL32(255, 226, 185, 245);
  const ImU32 key   = IM_COL32(0, 0, 0, 225);

  const float w = vmax.x - vmin.x;
  const float h = vmax.y - vmin.y;
  if (w <= 2.0f || h <= 2.0f || !osd_font) return;

  // Use a clear raster size and snap to a 2 px grid. Scaling the regular
  // UI font with arbitrary sub-pixel positions looks too clean for analog OSD.
  const float font_size = std::clamp(std::floor(h / 19.0f), 22.0f, 27.0f) * text_scale;
  const float cell = 2.0f;
  const float margin = std::max(6.0f, std::floor(std::min(w, h) * 0.018f));

  auto snap = [cell](float v) {
    return std::floor(v / cell) * cell;
  };

  auto text_size = [&](const char* text) {
    return osd_font->CalcTextSizeA(font_size, 10000.0f, 0.0f, text);
  };

  const float time = static_cast<float>(ImGui::GetTime());

  auto add_analog_osd_text = [&](ImVec2 pos, ImU32 color, const char* text, int elem_id) {
    // Very slow base rate (0.5 Hz - 1.5 Hz)
    float step_rate = 0.5f + static_cast<float>((elem_id * 7 + 11) % 5) * 0.25f;
    // Heavy time warping using chaotic sine waves for unpredictable jump intervals
    float chaotic_time = time 
                         + std::sin(time * 1.3f + elem_id) * 1.2f 
                         + std::sin(time * 0.7f - elem_id * 2.0f) * 0.8f;

    uint32_t step = static_cast<uint32_t>(chaotic_time * step_rate + static_cast<float>(elem_id * 17));

    // Integer RNG seed derived strictly from step + elem_id (stable across changing string numbers!)
    uint32_t rng = (step * 1103515245 + static_cast<uint32_t>(elem_id) * 2654435761u) & 0x7fffffff;

    // More random H-Sync Jitter (-3.0px to +3.0px), rests at 0 more often
    float text_jitter = 0.0f;
    uint32_t j_mod = (rng >> 3) % 23;
    if (j_mod == 1) text_jitter = -3.0f;
    else if (j_mod == 2) text_jitter = -2.0f;
    else if (j_mod == 3) text_jitter = -1.0f;
    else if (j_mod == 4) text_jitter = -0.5f;
    else if (j_mod == 5) text_jitter = 0.5f;
    else if (j_mod == 6) text_jitter = 1.0f;
    else if (j_mod == 7) text_jitter = 2.0f;
    else if (j_mod == 8) text_jitter = 3.0f;

    // Discrete RF Luma Flicker (opacity drops)
    float text_flicker = 1.0f;
    uint32_t f_mod = (rng >> 7) % 7;
    if (f_mod == 0) text_flicker = 0.72f;
    else if (f_mod == 1) text_flicker = 0.85f;
    else if (f_mod == 2) text_flicker = 0.93f;

    pos.x = snap(pos.x + text_jitter);
    pos.y = snap(pos.y);

    uint8_t r = static_cast<uint8_t>((color >> 0) & 0xFF);
    uint8_t g = static_cast<uint8_t>((color >> 8) & 0xFF);
    uint8_t b = static_cast<uint8_t>((color >> 16) & 0xFF);
    uint8_t a = static_cast<uint8_t>(((color >> 24) & 0xFF) * text_flicker);
    ImU32 flickered_color = IM_COL32(r, g, b, a);

    // Four keyed neighbours are enough for the low-resolution OSD font and
    // submit substantially less geometry than the previous 6-direction key.
    dl->AddText(osd_font, font_size, {pos.x - 1.0f, pos.y}, key, text);
    dl->AddText(osd_font, font_size, {pos.x + 1.0f, pos.y}, key, text);
    dl->AddText(osd_font, font_size, {pos.x, pos.y - 1.0f}, key, text);
    dl->AddText(osd_font, font_size, {pos.x, pos.y + 1.0f}, key, text);

    // One faint luma echo plus a single primary pass keeps the analog softness
    // without multiplying every OSD string into ten draw submissions.
    ImU32 soft_col = IM_COL32(r, g, b, static_cast<uint8_t>(a * 0.28f));
    dl->AddText(osd_font, font_size, {pos.x + 1.5f, pos.y}, soft_col, text);
    dl->AddText(osd_font, font_size, pos, flickered_color, text);
  };

  char buf[128];

  // Keep the overlay sparse and utilitarian, like a real flight OSD.
  std::snprintf(buf, sizeof(buf), "%.1f FPS", fps);
  add_analog_osd_text({vmin.x + margin, vmin.y + margin}, osd, buf, 0);

  if (tw > 0) {
    std::snprintf(buf, sizeof(buf), "%dx%d", tw, th);
    add_analog_osd_text({vmin.x + margin, vmin.y + margin + font_size + 3.0f},
                        dim, buf, 1);
  }

  const char* mode_name = (view_mode == 0) ? "NORM" :
                          (view_mode == 1) ? "WIDE" : "FULL";
  const auto mode_ts = text_size(mode_name);
  add_analog_osd_text({vmin.x + (w - mode_ts.x) * 0.5f, vmin.y + margin},
                      dim, mode_name, 2);

  std::snprintf(buf, sizeof(buf), "UDP %u", stats.datagrams);
  auto ts = text_size(buf);
  add_analog_osd_text({vmax.x - margin - ts.x, vmin.y + margin}, osd, buf, 3);

  // Real physical Wi-Fi RSSI signal strength from system wireless driver (/proc/net/wireless)
  int real_rssi = GetRealWifiRssi();
  int rssi_pct = 0;
  if (stats.video_seen) {
    if (real_rssi > 0) {
      rssi_pct = real_rssi;
    } else {
      float err_ratio = (stats.datagrams > 0) ?
          (static_cast<float>(stats.discarded + stats.decode_errors) / static_cast<float>(stats.datagrams)) : 0.0f;
      rssi_pct = std::clamp(static_cast<int>((1.0f - err_ratio) * 100.0f), 0, 100);
    }
  }

  char rssi_buf[32];
  std::snprintf(rssi_buf, sizeof(rssi_buf), "%d%%", rssi_pct);
  const auto rssi_ts = text_size(rssi_buf);
  ImVec2 rssi_pos = {vmax.x - margin - rssi_ts.x, vmin.y + margin + font_size + 3.0f};

  // Compute Slot 8 jitter & flicker for the signal icon + percentage
  const int elem_id = 8;
  float step_rate = 0.5f + static_cast<float>((elem_id * 7 + 11) % 5) * 0.25f;
  float chaotic_time = time 
                       + std::sin(time * 1.3f + elem_id) * 1.2f 
                       + std::sin(time * 0.7f - elem_id * 2.0f) * 0.8f;
  uint32_t step = static_cast<uint32_t>(chaotic_time * step_rate + static_cast<float>(elem_id * 17));
  uint32_t rng = (step * 1103515245 + static_cast<uint32_t>(elem_id) * 2654435761u) & 0x7fffffff;

  float icon_jitter = 0.0f;
  uint32_t j_mod = (rng >> 3) % 23;
  if (j_mod == 1) icon_jitter = -3.0f;
  else if (j_mod == 2) icon_jitter = -2.0f;
  else if (j_mod == 3) icon_jitter = -1.0f;
  else if (j_mod == 4) icon_jitter = -0.5f;
  else if (j_mod == 5) icon_jitter = 0.5f;
  else if (j_mod == 6) icon_jitter = 1.0f;
  else if (j_mod == 7) icon_jitter = 2.0f;
  else if (j_mod == 8) icon_jitter = 3.0f;

  float icon_flicker = 1.0f;
  uint32_t f_mod = (rng >> 7) % 7;
  if (f_mod == 0) icon_flicker = 0.72f;
  else if (f_mod == 1) icon_flicker = 0.85f;
  else if (f_mod == 2) icon_flicker = 0.93f;

  // Draw 4-bar signal icon with full analog OSD effects (bold black keying, luma dispersion, jitter & flicker)
  const float bar_w = 3.0f * text_scale;
  const float bar_gap = 2.0f * text_scale;
  const float max_bar_h = font_size * 0.75f;
  const ImVec2 bar_base = ImVec2{snap(rssi_pos.x - 22.0f * text_scale + icon_jitter), snap(rssi_pos.y + font_size * 0.85f)};

  auto draw_signal_bars = [&](ImVec2 base, ImU32 base_col, float alpha_mult) {
    uint8_t br_r = static_cast<uint8_t>((base_col >> 0) & 0xFF);
    uint8_t br_g = static_cast<uint8_t>((base_col >> 8) & 0xFF);
    uint8_t br_b = static_cast<uint8_t>((base_col >> 16) & 0xFF);

    for (int b = 0; b < 4; ++b) {
      float bh = max_bar_h * (0.35f + b * 0.22f);
      ImVec2 b0 = {base.x + b * (bar_w + bar_gap), base.y - bh};
      ImVec2 b1 = {b0.x + bar_w, base.y};
      bool bar_on = (rssi_pct >= (b + 1) * 23);
      uint8_t base_a = bar_on ? 235 : 125;
      ImU32 col = IM_COL32(br_r, br_g, br_b, static_cast<uint8_t>(base_a * alpha_mult));

      // Hard black key outline
      dl->AddRectFilled({b0.x - 1.0f, b0.y - 1.0f}, {b1.x + 2.0f, b1.y + 1.0f}, key);
      dl->AddRectFilled(b0, b1, col);
    }
  };

  ImU32 sig_col;
  if (!stats.video_seen) {
    sig_col = IM_COL32(255, 90, 90, 245); // Red (No Video)
  } else if (rssi_pct > 60) {
    sig_col = IM_COL32(140, 255, 140, 235); // Green (Good)
  } else if (rssi_pct > 30) {
    sig_col = IM_COL32(255, 235, 110, 235); // Yellow (Fair)
  } else {
    sig_col = IM_COL32(255, 90, 90, 235); // Red (Weak)
  }

  // Soft horizontal luma dispersion (blur)
  draw_signal_bars({bar_base.x - 1.5f, bar_base.y}, sig_col, 0.40f * icon_flicker);
  draw_signal_bars({bar_base.x + 2.5f, bar_base.y}, sig_col, 0.40f * icon_flicker);

  // Main signal bars with analog luma flicker
  draw_signal_bars(bar_base, sig_col, icon_flicker);

  add_analog_osd_text(rssi_pos, sig_col, rssi_buf, 8);

  // The displayed clock only changes once per second; avoid localtime/strftime
  // on every render iteration (the main loop can run hundreds of times/sec).
  static std::time_t cached_clock_second = static_cast<std::time_t>(-1);
  static char clock_buf[64]{};
  const std::time_t raw_time = std::time(nullptr);
  if (raw_time != cached_clock_second) {
    cached_clock_second = raw_time;
    std::tm tm_info{};
    if (localtime_r(&raw_time, &tm_info)) {
      std::strftime(clock_buf, sizeof(clock_buf), "%Y-%m-%d %H:%M:%S", &tm_info);
    } else {
      clock_buf[0] = '\0';
    }
  }
  if (clock_buf[0] != '\0') {
    add_analog_osd_text({vmin.x + margin,
                         vmax.y - margin - (font_size * 2.0f + 3.0f)},
                        dim, clock_buf, 7);
  }

  const char* status = stats.video_seen ? "VIDEO" : "NO SIGNAL";
  add_analog_osd_text({vmin.x + margin,
                       vmax.y - margin - font_size},
                      stats.video_seen ? osd : warn, status, 5);

  std::snprintf(buf, sizeof(buf), "F%u", stats.frames);
  ts = text_size(buf);
  add_analog_osd_text({vmax.x - margin - ts.x,
                       vmax.y - margin - font_size},
                      dim, buf, 6);

  // Recording indicator (red blinking REC dot)
  if (is_recording) {
    const ImU32 rec_col = IM_COL32(220, 30, 30, 235);
    // Blink the dot at ~2 Hz via discrete stepping
    uint32_t blink_step = static_cast<uint32_t>(time * 2.0f);
    if (blink_step % 2 == 0) {
      const float dot_r = font_size * 0.25f;
      ImVec2 dot_pos = {vmin.x + (w * 0.5f) - font_size * 1.5f, vmax.y - margin - font_size + font_size * 0.5f};
      dl->AddCircleFilled(dot_pos, dot_r + 1.0f, key, 16);
      dl->AddCircleFilled(dot_pos, dot_r, rec_col, 16);
      add_analog_osd_text({dot_pos.x + dot_r + 4.0f, vmax.y - margin - font_size}, rec_col, "REC", 9);
    }
  }

  // Minimal center reticle. The old large neon crosshair/corner brackets made
  // the result resemble a HUD. Two short keyed ticks are more FPV-like.
  const ImVec2 c{snap(vmin.x + w * 0.5f), snap(vmin.y + h * 0.5f)};
  const float gap = 4.0f;
  const float arm = 10.0f;
  dl->AddLine({c.x - arm, c.y}, {c.x - gap, c.y}, key, 3.0f);
  dl->AddLine({c.x + gap, c.y}, {c.x + arm, c.y}, key, 3.0f);
  dl->AddLine({c.x, c.y - arm}, {c.x, c.y - gap}, key, 3.0f);
  dl->AddLine({c.x, c.y + gap}, {c.x, c.y + arm}, key, 3.0f);
  dl->AddLine({c.x - arm, c.y}, {c.x - gap, c.y}, dim, 1.0f);
  dl->AddLine({c.x + gap, c.y}, {c.x + arm, c.y}, dim, 1.0f);
  dl->AddLine({c.x, c.y - arm}, {c.x, c.y - gap}, dim, 1.0f);
  dl->AddLine({c.x, c.y + gap}, {c.x, c.y + arm}, dim, 1.0f);
}

// ── Virtual Gamepad Visualisation ────────────────────────────────────

struct PadVis {
  float lx{0.5f}, ly{0.5f}, rx{0.5f}, ry{0.5f};
  bool a{}, b{}, x{}, y{}, lb{}, rb{}, back{}, start{};
};

void DrawVirtualGamepad(ImDrawList* dl, ImVec2 p0, ImVec2 p1, const PadVis& s) {
  const float outer_w = p1.x - p0.x;
  const float outer_h = p1.y - p0.y;
  if (outer_w < 80.0f || outer_h < 80.0f) return;

  const ImU32 panel = IM_COL32(18, 21, 26, 255);
  const ImU32 rim   = IM_COL32(76, 84, 96, 255);
  const ImU32 grid  = IM_COL32(70, 78, 90, 90);
  const ImU32 dot   = IM_COL32(0, 220, 135, 255);
  const ImU32 off   = IM_COL32(53, 59, 68, 255);
  const ImU32 txt   = IM_COL32(185, 193, 205, 255);

  // Layout: Top 55% for big joysticks, bottom 45% for boxed buttons
  const float stick_h = outer_h * 0.55f;
  
  // Joysticks (Outside the box)
  const float sr = std::min(outer_w * 0.24f, stick_h * 0.45f);
  const float knob = sr * 0.28f;

  auto draw_stick = [&](ImVec2 c, float sx, float sy) {
    dl->AddCircleFilled(c, sr, IM_COL32(24, 28, 34, 255), 32);
    dl->AddCircle(c, sr, rim, 32, 1.5f);
    dl->AddLine({c.x - sr * 0.8f, c.y}, {c.x + sr * 0.8f, c.y}, grid, 1.0f);
    dl->AddLine({c.x, c.y - sr * 0.8f}, {c.x, c.y + sr * 0.8f}, grid, 1.0f);
    ImVec2 d{c.x + (sx - 0.5f) * 2.0f * sr * 0.72f,
             c.y + (sy - 0.5f) * 2.0f * sr * 0.72f};
    dl->AddCircleFilled(d, knob, dot, 20);
    dl->AddCircle(d, knob, IM_COL32(210, 255, 235, 180), 20, 1.0f);
  };

  const ImVec2 left_stick{p0.x + outer_w * 0.25f, p0.y + stick_h * 0.50f};
  const ImVec2 right_stick{p0.x + outer_w * 0.75f, p0.y + stick_h * 0.50f};
  draw_stick(left_stick, s.lx, s.ly);
  draw_stick(right_stick, s.rx, s.ry);

  // Bottom box for all other buttons (Inside the box)
  const ImVec2 b0{p0.x + 2.0f, p0.y + stick_h + 2.0f};
  const ImVec2 b1{p1.x - 2.0f, p1.y - 2.0f};
  dl->AddRectFilled(b0, b1, panel, 8.0f);
  dl->AddRect(b0, b1, rim, 8.0f, 0, 1.0f);

  const ImVec2 c0{b0.x + 6.0f, b0.y + 6.0f};
  const ImVec2 c1{b1.x - 6.0f, b1.y - 6.0f};
  const float bw = c1.x - c0.x;
  const float bh = c1.y - c0.y;

  // Center points for sections - proportional and balanced for small & large windows
  const ImVec2 dpad_c{c0.x + bw * 0.17f, c0.y + bh * 0.50f};
  const ImVec2 face_c{c0.x + bw * 0.83f, c0.y + bh * 0.50f};

  // D-pad (left) - Scaled up to match face buttons cluster size
  const float ds = std::min(bw * 0.060f, bh * 0.17f);
  dl->AddRectFilled({dpad_c.x - ds * 0.45f, dpad_c.y - ds * 1.35f},
                    {dpad_c.x + ds * 0.45f, dpad_c.y + ds * 1.35f}, off, 3.0f);
  dl->AddRectFilled({dpad_c.x - ds * 1.35f, dpad_c.y - ds * 0.45f},
                    {dpad_c.x + ds * 1.35f, dpad_c.y + ds * 0.45f}, off, 3.0f);
  dl->AddRect({dpad_c.x - ds * 0.45f, dpad_c.y - ds * 1.35f},
              {dpad_c.x + ds * 0.45f, dpad_c.y + ds * 1.35f}, rim, 3.0f);
  dl->AddRect({dpad_c.x - ds * 1.35f, dpad_c.y - ds * 0.45f},
              {dpad_c.x + ds * 1.35f, dpad_c.y + ds * 0.45f}, rim, 3.0f);

  // Face buttons (right) with text
  const float br = std::min(bw * 0.035f, bh * 0.10f);
  const float bs = br * 2.0f;
  auto face = [&](ImVec2 pos, const char* label, bool on, ImU32 on_col) {
    dl->AddCircleFilled(pos, br, on ? on_col : off, 20);
    dl->AddCircle(pos, br, rim, 20, 1.0f);
    const auto ts = ImGui::CalcTextSize(label);
    dl->AddText({pos.x - ts.x * 0.5f, pos.y - ts.y * 0.5f}, txt, label);
  };
  face({face_c.x, face_c.y - bs}, "Y", s.y, IM_COL32(220, 180, 30, 255));
  face({face_c.x, face_c.y + bs}, "A", s.a, IM_COL32(50, 200, 70, 255));
  face({face_c.x - bs, face_c.y}, "X", s.x, IM_COL32(50, 120, 220, 255));
  face({face_c.x + bs, face_c.y}, "B", s.b, IM_COL32(220, 50, 50, 255));

  // Middle buttons (LB, RB, BACK, START) with text - scalable width
  const float pill_w = std::min(bw * 0.14f, bh * 0.45f);
  const float pill_h = std::min(bw * 0.07f, bh * 0.22f);
  auto pill = [&](ImVec2 center, const char* label, bool on) {
    ImVec2 a{center.x - pill_w * 0.5f, center.y - pill_h * 0.5f};
    ImVec2 b{center.x + pill_w * 0.5f, center.y + pill_h * 0.5f};
    dl->AddRectFilled(a, b, on ? dot : off, pill_h * 0.5f);
    dl->AddRect(a, b, rim, pill_h * 0.5f, 0, 1.0f);
    const auto ts = ImGui::CalcTextSize(label);
    dl->AddText({center.x - ts.x * 0.5f, center.y - ts.y * 0.5f}, txt, label);
  };
  
  // Top row: LB, RB
  pill({c0.x + bw * 0.42f, c0.y + bh * 0.30f}, "LB", s.lb);
  pill({c0.x + bw * 0.58f, c0.y + bh * 0.30f}, "RB", s.rb);
  // Bottom row: BACK, START
  pill({c0.x + bw * 0.42f, c0.y + bh * 0.70f}, "BACK", s.back);
  pill({c0.x + bw * 0.58f, c0.y + bh * 0.70f}, "START", s.start);
}

}  // namespace

int main() {
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) != 0) {
    std::fprintf(stderr, "SDL init failed: %s\n", SDL_GetError());
    return 1;
  }
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_Window* window = SDL_CreateWindow(
      "TDG Direct WIFI_8K", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
      1400, 860, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
  if (!window) {
    std::fprintf(stderr, "SDL window creation failed: %s\n", SDL_GetError());
    SDL_Quit();
    return 1;
  }
  SDL_GLContext gl = SDL_GL_CreateContext(window);
  if (!gl) {
    std::fprintf(stderr, "OpenGL context creation failed: %s\n", SDL_GetError());
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  // A recorder subprocess exiting must not terminate the flight UI with SIGPIPE.
  std::signal(SIGPIPE, SIG_IGN);

  // VSync adds up to one display interval of control/video latency. Run the
  // presentation path unsynchronised and yield briefly at the end of each loop.
  SDL_GL_SetSwapInterval(0);
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();

  // Keep the normal UI font untouched, but build a dedicated small,
  // pixel-snapped font for the analog OSD. Dear ImGui's built-in ProggyClean
  // face works well here and avoids shipping another font asset.
  ImGuiIO& io = ImGui::GetIO();
  io.FontGlobalScale = 1.0f;
  io.Fonts->AddFontDefault();
  ImFontConfig osd_font_cfg{};
  osd_font_cfg.SizePixels = 24.0f;
  osd_font_cfg.OversampleH = 1;
  osd_font_cfg.OversampleV = 1;
  osd_font_cfg.PixelSnapH = true;
  ImFont* osd_font = io.Fonts->AddFontDefault(&osd_font_cfg);

  ImGui_ImplSDL2_InitForOpenGL(window, gl);
  ImGui_ImplOpenGL3_Init("#version 130");

  tdg::direct::DroneSession drone;
  SDL_GameController* gamepad = nullptr;

  // Control state
  std::uint8_t roll = 128, pitch = 128, throttle = 0, yaw = 128;
  constexpr std::uint8_t kMin = 0, kMax = 255;
  bool use_gamepad = true;
  std::uint8_t pulse_flags = 0;
  auto pulse_until = std::chrono::steady_clock::time_point{};
  auto last_control = std::chrono::steady_clock::now() - std::chrono::milliseconds(40);
  std::uint64_t last_control_state = std::numeric_limits<std::uint64_t>::max();

  // Mode indicators (local tracking only; FC toggles internally)
  bool headless_active = false;
  bool motors_armed = false;
  bool safety_lock = true;

  // Video
  GLuint texture = create_texture();
  GLuint scanline_texture = create_scanline_texture();
  int texture_width = 0, texture_height = 0;
  tdg::direct::RgbFrame upload_frame;

  // Camera & Display FPS tracking (1.0 second time window)
  auto fps_window_start = std::chrono::steady_clock::now();
  auto last_frame_arrival = std::chrono::steady_clock::now();
  std::uint64_t window_start_decoded_frames = 0;
  std::uint32_t window_display_frames = 0;
  float camera_fps = 0.0f;
  float display_fps = 0.0f;

  bool running = true;
  int view_mode = 0; // 0 = Normal, 1 = Compact UI (Bigger Video), 2 = Full Video
  bool enable_analog_filter = true;
  bool enable_video_enhance = true;
  bool enable_denoise = true;
  float contrast_boost = 1.07f;
  float saturation_boost = 1.06f;
  float osd_text_scale = 1.0f;
  drone.set_video_enhancement(enable_video_enhance, enable_denoise,
                              contrast_boost, saturation_boost);

  // Recording state. Framebuffer readback remains on the GL thread, but all
  // potentially blocking pipe I/O is handled by a latest-only writer thread.
  bool is_recording = false;
  AsyncRecorder recorder;
  std::string rec_filename;
  std::vector<std::uint8_t> rec_pixels;
  ImVec2 rec_region_min{0, 0}, rec_region_max{0, 0};
  int rec_w = 0, rec_h = 0;
  auto rec_start_time = std::chrono::steady_clock::now();
  auto last_rec_frame_time = std::chrono::steady_clock::now();

  auto capture_size = [&]() -> std::pair<int, int> {
    if (rec_region_max.x <= rec_region_min.x || rec_region_max.y <= rec_region_min.y)
      return {0, 0};
    int fb_w = 0, fb_h = 0;
    SDL_GL_GetDrawableSize(window, &fb_w, &fb_h);
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    if (fb_w <= 0 || fb_h <= 0 || display.x <= 0.0f || display.y <= 0.0f) return {0, 0};
    const float sx = static_cast<float>(fb_w) / display.x;
    const float sy = static_cast<float>(fb_h) / display.y;
    const int width = static_cast<int>((rec_region_max.x - rec_region_min.x) * sx) & ~1;
    const int height = static_cast<int>((rec_region_max.y - rec_region_min.y) * sy) & ~1;
    return {width, height};
  };

  auto start_recording = [&](int cap_w, int cap_h) {
    if (is_recording || cap_w < 4 || cap_h < 4) return;
    const std::time_t t = std::time(nullptr);
    std::tm tm_info{};
    if (!localtime_r(&t, &tm_info)) return;

    char fname[128];
    if (std::strftime(fname, sizeof(fname), "REC_%Y%m%d_%H%M%S.mp4", &tm_info) == 0) return;
    rec_filename = fname;
    rec_w = cap_w & ~1;
    rec_h = cap_h & ~1;
    rec_pixels.assign(static_cast<std::size_t>(rec_w) * rec_h * 3u, 0);

    if (recorder.start(rec_w, rec_h, rec_filename)) {
      is_recording = true;
      rec_start_time = std::chrono::steady_clock::now();
      last_rec_frame_time = rec_start_time;
    } else {
      rec_pixels.clear();
      rec_w = rec_h = 0;
    }
  };

  auto stop_recording = [&]() {
    if (!is_recording && rec_w == 0 && rec_h == 0) return;
    is_recording = false;
    recorder.stop();
    rec_pixels.clear();
    rec_w = rec_h = 0;
  };

  auto toggle_recording = [&]() {
    if (is_recording) {
      stop_recording();
      return;
    }
    const auto [cw, ch] = capture_size();
    start_recording(cw, ch);
  };

  while (running) {
    auto frame_start = std::chrono::steady_clock::now();
    // ── SDL events ───────────────────────────────────────────────────
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT) running = false;
      if (event.type == SDL_KEYDOWN) {
        if (event.key.keysym.sym == SDLK_v || event.key.keysym.sym == SDLK_f) {
          view_mode = (view_mode + 1) % 3;
        }
        if (event.key.keysym.sym == SDLK_r) {
          toggle_recording();
        }
      }
      if (event.type == SDL_CONTROLLERDEVICEADDED && !gamepad)
        gamepad = SDL_GameControllerOpen(event.cdevice.which);
      if (event.type == SDL_CONTROLLERDEVICEREMOVED && gamepad &&
          SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(gamepad)) ==
              event.cdevice.which) {
        SDL_GameControllerClose(gamepad);
        gamepad = nullptr;
      }
      if (event.type == SDL_CONTROLLERBUTTONDOWN && use_gamepad) {
        auto btn = event.cbutton.button;
        if (btn == SDL_CONTROLLER_BUTTON_DPAD_UP ||
            btn == SDL_CONTROLLER_BUTTON_DPAD_LEFT ||
            btn == SDL_CONTROLLER_BUTTON_DPAD_RIGHT) {
          AddLog("[Gamepad] DPAD UP/L/R -> Action: Cycle view mode");
          view_mode = (view_mode + 1) % 3;
        }
        if (btn == SDL_CONTROLLER_BUTTON_DPAD_DOWN) {
          AddLog("[Gamepad] DPAD DOWN -> Action: Toggle recording");
          toggle_recording();
        }
        if (btn == SDL_CONTROLLER_BUTTON_A) { 
          safety_lock = !safety_lock; 
          AddLog("[Gamepad] A Button -> Action: Safety Lock is %s", safety_lock ? "LOCKED" : "OFF");
          if (gamepad) {
            if (safety_lock) {
              // Heavy, distinct thud for LOCKED
              SDL_GameControllerRumble(gamepad, 0xFFFF, 0xFFFF, 300);
            } else {
              // Quick high-frequency buzz for UNLOCKED
              SDL_GameControllerRumble(gamepad, 0x0000, 0xC000, 150);
            }
          }
        }
        
        if (!safety_lock) {
          if (btn == SDL_CONTROLLER_BUTTON_B) { pulse_flags |= 0x02; AddLog("[Gamepad] B Button -> Action: Auto Land"); }
          if (btn == SDL_CONTROLLER_BUTTON_X) { pulse_flags |= 0x04; AddLog("[Gamepad] X Button -> Action: Emergency Stop"); }
          if (btn == SDL_CONTROLLER_BUTTON_Y) { pulse_flags |= 0x08; AddLog("[Gamepad] Y Button -> Action: 360 Flip"); }
          if (btn == SDL_CONTROLLER_BUTTON_LEFTSHOULDER)  { pulse_flags |= 0x20; motors_armed = false; AddLog("[Gamepad] LB Bumper -> Action: Lock Motors"); }
          if (btn == SDL_CONTROLLER_BUTTON_RIGHTSHOULDER) { pulse_flags |= 0x40; motors_armed = true;  AddLog("[Gamepad] RB Bumper -> Action: Unlock Motors"); }
          if (btn == SDL_CONTROLLER_BUTTON_BACK)  { pulse_flags |= 0x80; AddLog("[Gamepad] BACK -> Action: Calibrate Gyro"); }
          if (btn == SDL_CONTROLLER_BUTTON_START) { pulse_flags |= 0x10; headless_active = !headless_active; AddLog("[Gamepad] START -> Action: Headless %s", headless_active ? "ON" : "OFF"); }
          if (btn == SDL_CONTROLLER_BUTTON_GUIDE) { AddLog("[Gamepad] HOME -> No action mapped"); }
          if (btn != SDL_CONTROLLER_BUTTON_A) {
            pulse_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(180);
          }
        } else {
          if (btn != SDL_CONTROLLER_BUTTON_A && 
              btn != SDL_CONTROLLER_BUTTON_DPAD_UP && 
              btn != SDL_CONTROLLER_BUTTON_DPAD_DOWN && 
              btn != SDL_CONTROLLER_BUTTON_DPAD_LEFT && 
              btn != SDL_CONTROLLER_BUTTON_DPAD_RIGHT) {
            AddLog("[Gamepad] Ignored button press -> Safety lock is ON");
          }
        }
      }
    }

    const auto now = std::chrono::steady_clock::now();

    // ── Safety Lock Heartbeat ─────────────────────────────────────────
    static auto next_lock_pulse = now + std::chrono::milliseconds(3000);
    if (use_gamepad && gamepad && safety_lock) {
      if (now >= next_lock_pulse) {
        // Very brief, low-intensity pulse every 3 seconds
        // Doesn't wear out motors or drain battery
        SDL_GameControllerRumble(gamepad, 0x0000, 0x2000, 50);
        next_lock_pulse = now + std::chrono::milliseconds(3000);
      }
    } else {
      // Keep pushing the timer forward when not locked
      next_lock_pulse = now + std::chrono::milliseconds(3000);
    }

    // ── Input polling ────────────────────────────────────────────────
    if (use_gamepad && gamepad && !safety_lock) {
      yaw = axis_to_wire(
          SDL_GameControllerGetAxis(gamepad, SDL_CONTROLLER_AXIS_LEFTX));
      throttle = throttle_to_wire(
          SDL_GameControllerGetAxis(gamepad, SDL_CONTROLLER_AXIS_LEFTY), true);
      roll = axis_to_wire(
          SDL_GameControllerGetAxis(gamepad, SDL_CONTROLLER_AXIS_RIGHTX));
      pitch = axis_to_wire(
          SDL_GameControllerGetAxis(gamepad, SDL_CONTROLLER_AXIS_RIGHTY), true);
    } else {
      yaw = 128; throttle = 0; roll = 128; pitch = 128;
    }

    std::uint8_t active_flags = 0;
    if (now < pulse_until) {
      active_flags = pulse_flags;
    } else {
      pulse_flags = 0;
    }

    const std::uint8_t wire_flags = safety_lock ? 0 : active_flags;
    const std::uint64_t control_state =
        pack_control_state(roll, pitch, throttle, yaw, wire_flags);

    const bool control_changed = control_state != last_control_state;
    const bool change_due = control_changed && now - last_control >= std::chrono::milliseconds(5);
    const bool heartbeat_due = now - last_control >= std::chrono::milliseconds(40);
    if (change_due || heartbeat_due) {
      drone.send_controls(roll, pitch, throttle, yaw, wire_flags);
      last_control = now;
      last_control_state = control_state;
    }

    // ── Capture newest video frame + update preallocated GL texture ───
    // Decode + optional enhancement already happened on the worker thread; this
    // path is intentionally limited to one texture upload and a few counters.
    const bool got_new_frame = drone.take_latest(upload_frame);
    if (got_new_frame) {
      glBindTexture(GL_TEXTURE_2D, texture);
      if (upload_frame.width != texture_width || upload_frame.height != texture_height) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, upload_frame.width, upload_frame.height, 0,
                     GL_RGB, GL_UNSIGNED_BYTE, upload_frame.pixels.data());
        texture_width = upload_frame.width;
        texture_height = upload_frame.height;
      } else {
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, upload_frame.width, upload_frame.height,
                        GL_RGB, GL_UNSIGNED_BYTE, upload_frame.pixels.data());
      }
      window_display_frames++;
      last_frame_arrival = now;
    }

    const auto stats = drone.stats();

    // ── 1.0 Second Time-Window FPS Calculation ─────────────────────────
    const float window_elapsed = std::chrono::duration<float>(now - fps_window_start).count();
    const float time_since_last_frame = std::chrono::duration<float>(now - last_frame_arrival).count();

    if (window_elapsed >= 1.0f) {
      const std::uint64_t decoded_delta = (stats.frames >= window_start_decoded_frames) ?
                                          (stats.frames - window_start_decoded_frames) : 0;
      camera_fps = static_cast<float>(decoded_delta) / window_elapsed;
      display_fps = static_cast<float>(window_display_frames) / window_elapsed;

      window_start_decoded_frames = stats.frames;
      window_display_frames = 0;
      fps_window_start = now;
    }

    // Timeout: If frames stop arriving for >= 1.0s or video lost, set FPS to 0
    if (!stats.video_seen || time_since_last_frame >= 1.0f) {
      camera_fps = 0.0f;
      display_fps = 0.0f;
    }

    // ── ImGui frame ──────────────────────────────────────────────────
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos({0, 0}, ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiCond_Always);
    ImGui::Begin("##main", nullptr,
        ImGuiWindowFlags_NoResize  | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove);

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    
    // View mode layout height calculation
    float video_h = avail.y * 0.60f;
    if (view_mode == 0) {
      const float desired_video_h = avail.y * 0.60f;
      const float bottom_reserve = std::min(300.0f, avail.y * 0.46f);
      video_h = std::max(180.0f, std::min(desired_video_h, avail.y - bottom_reserve));
    } else if (view_mode == 1) {
      video_h = avail.y * 0.82f;
    } else {
      video_h = avail.y;
    }

    // ── Video panel (top, full width) ────────────────────────────────
    ImGui::BeginChild("video", {avail.x, video_h}, true,
                      ImGuiWindowFlags_NoScrollbar);
    {
      if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        view_mode = (view_mode + 1) % 3;
      }

      ImVec2 content_min = ImGui::GetCursorScreenPos();
      ImVec2 content_sz  = ImGui::GetContentRegionAvail();
      ImVec2 osd_min = content_min;
      ImVec2 osd_max = {content_min.x + content_sz.x, content_min.y + content_sz.y};

      if (texture_width > 0) {
        bool rotated = texture_height > texture_width;
        int logical_w = rotated ? texture_height : texture_width;
        int logical_h = rotated ? texture_width : texture_height;

        const float scale = std::min(content_sz.x / logical_w,
                                     content_sz.y / logical_h);
        const ImVec2 img_sz(logical_w * scale, logical_h * scale);
        ImVec2 offset{std::max(0.0f, (content_sz.x - img_sz.x) * 0.5f),
                      std::max(0.0f, (content_sz.y - img_sz.y) * 0.5f)};
        ImVec2 cur = ImGui::GetCursorPos();
        ImGui::SetCursorPos({cur.x + offset.x, cur.y + offset.y});
        
        ImVec2 p0 = ImGui::GetCursorScreenPos();
        ImVec2 p1 = {p0.x + img_sz.x, p0.y};
        ImVec2 p2 = {p0.x + img_sz.x, p0.y + img_sz.y};
        ImVec2 p3 = {p0.x, p0.y + img_sz.y};

        DrawVideoImage(ImGui::GetWindowDrawList(), texture, p0, p1, p2, p3,
                       rotated, enable_analog_filter);
        ImGui::Dummy(img_sz);
        
        osd_min = p0;
        osd_max = p2;
      } else {
        ImGui::SetCursorPos({avail.x * 0.18f, video_h * 0.42f});
        ImGui::TextUnformatted(
            "Connect this laptop to the FC WIFI_8K network, "
            "then power the drone on.");
      }
      
      // Render the OSD into the same analog presentation layer as the video
      int disp_w = texture_width > 0 ? (texture_height > texture_width ? texture_height : texture_width) : 0;
      int disp_h = texture_height > 0 ? (texture_height > texture_width ? texture_width : texture_height) : 0;
      DrawOSD(ImGui::GetWindowDrawList(), osd_font, osd_min, osd_max,
              camera_fps, stats, disp_w, disp_h, view_mode, osd_text_scale, is_recording);
      DrawAnalogFilter(ImGui::GetWindowDrawList(), scanline_texture, osd_min, osd_max, enable_analog_filter);

      // Track the video panel region for recording capture
      rec_region_min = osd_min;
      rec_region_max = osd_max;
    }
    ImGui::EndChild();

    if (view_mode != 2) {
      // ── Bottom panel: 3 columns ──────────────────────────────────────
      const float bot_h = ImGui::GetContentRegionAvail().y;
      const float sp = ImGui::GetStyle().ItemSpacing.x;
      const float col1_w = (avail.x - sp * 2) * 0.32f;
      const float col2_w = (avail.x - sp * 2) * 0.42f;

    // ── Column 1: Virtual Gamepad ────────────────────────────────────
    ImGui::BeginChild("gamepad_viz", {col1_w, bot_h}, true,
                      ImGuiWindowFlags_NoScrollbar);
    {
      PadVis pv;
      pv.lx = yaw / 255.0f;
      pv.ly = 1.0f - throttle / 255.0f;
      pv.rx = roll / 255.0f;
      pv.ry = 1.0f - pitch / 255.0f;
      if (gamepad) {
        pv.a = SDL_GameControllerGetButton(gamepad, SDL_CONTROLLER_BUTTON_A);
        if (!safety_lock) {
          pv.b     = SDL_GameControllerGetButton(gamepad, SDL_CONTROLLER_BUTTON_B);
          pv.x     = SDL_GameControllerGetButton(gamepad, SDL_CONTROLLER_BUTTON_X);
          pv.y     = SDL_GameControllerGetButton(gamepad, SDL_CONTROLLER_BUTTON_Y);
          pv.lb    = SDL_GameControllerGetButton(gamepad, SDL_CONTROLLER_BUTTON_LEFTSHOULDER);
          pv.rb    = SDL_GameControllerGetButton(gamepad, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);
          pv.back  = SDL_GameControllerGetButton(gamepad, SDL_CONTROLLER_BUTTON_BACK);
          pv.start = SDL_GameControllerGetButton(gamepad, SDL_CONTROLLER_BUTTON_START);
        }
      }
      ImVec2 gp_min = ImGui::GetCursorScreenPos();
      ImVec2 gp_sz  = ImGui::GetContentRegionAvail();
      DrawVirtualGamepad(ImGui::GetWindowDrawList(),
                         gp_min, {gp_min.x + gp_sz.x, gp_min.y + gp_sz.y}, pv);
      ImGui::Dummy(gp_sz);
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ── Column 2: Flight Dashboard ───────────────────────────────────
    ImGui::BeginChild("dashboard", {col2_w, bot_h}, true);
    {
      // --- VIDEO & RECORDING ---
      ImGui::TextColored({0.4f, 0.8f, 0.4f, 1.0f}, "VIDEO & OSD");
      ImGui::Separator();

      bool processing_changed = ImGui::Checkbox("Image Enhance", &enable_video_enhance);
      if (enable_video_enhance) {
        ImGui::SameLine();
        processing_changed |= ImGui::Checkbox("Denoise", &enable_denoise);
      }
      if (processing_changed) {
        drone.set_video_enhancement(enable_video_enhance, enable_denoise,
                                    contrast_boost, saturation_boost);
      }
      ImGui::SameLine();
      ImGui::Checkbox("Analog Filter", &enable_analog_filter);

      ImGui::AlignTextToFramePadding();
      ImGui::Text("OSD Size:");
      ImGui::SameLine();
      if (ImGui::Button(" - ") && osd_text_scale > 0.5f) { osd_text_scale -= 0.1f; }
      ImGui::SameLine();
      ImGui::Text("%.0f%%", osd_text_scale * 100.0f);
      ImGui::SameLine();
      if (ImGui::Button(" + ") && osd_text_scale < 2.5f) { osd_text_scale += 0.1f; }
      ImGui::SameLine();
      ImGui::Dummy({10.0f, 0.0f});
      ImGui::SameLine();

      if (!is_recording) {
        if (ImGui::Button("  Record  ")) toggle_recording();
      } else {
        auto elapsed = std::chrono::steady_clock::now() - rec_start_time;
        int secs = static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count());
        char rec_label[64];
        std::snprintf(rec_label, sizeof(rec_label), "  Stop [%02d:%02d]  ", secs / 60, secs % 60);
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.1f, 0.1f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9f, 0.15f, 0.15f, 1.0f));
        if (ImGui::Button(rec_label)) toggle_recording();
        ImGui::PopStyleColor(2);
        ImGui::SameLine();
        ImGui::TextColored({0.9f, 0.3f, 0.3f, 1.0f}, "%s", rec_filename.c_str());
      }
      ImGui::Spacing();
      ImGui::Spacing();

      // --- INPUT & STATUS ---
      ImGui::TextColored({0.4f, 0.8f, 0.4f, 1.0f}, "INPUT & SYSTEM STATUS");
      ImGui::Separator();

      ImGui::Checkbox("Use gamepad", &use_gamepad);
      ImGui::SameLine();
      if (ImGui::Button("View Keybinds")) ImGui::OpenPopup("Gamepad Legend");
      ImGui::SameLine();
      ImGui::TextDisabled("| %s", gamepad ? SDL_GameControllerName(gamepad) : "Not connected");

      if (ImGui::BeginPopupModal("Gamepad Legend", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextColored({0.4f, 0.8f, 0.4f, 1.0f}, "Joysticks (Mode 2)");
        ImGui::BulletText("Left Stick (Up): Throttle (0%% at center, 100%% at top)");
        ImGui::BulletText("Left Stick (L/R): Yaw");
        ImGui::BulletText("Right Stick (U/D): Pitch");
        ImGui::BulletText("Right Stick (L/R): Roll");
        ImGui::Spacing();
        ImGui::TextColored({0.4f, 0.8f, 0.4f, 1.0f}, "Action Buttons");
        ImGui::BulletText("A Button: Safety Lock (Toggle)");
        ImGui::BulletText("B Button: Auto Land");
        ImGui::BulletText("X Button: Emergency Stop");
        ImGui::BulletText("Y Button: 360 Flip");
        ImGui::Spacing();
        ImGui::TextColored({0.4f, 0.8f, 0.4f, 1.0f}, "Bumpers & System");
        ImGui::BulletText("LB Bumper: Lock Motors (Disarm)");
        ImGui::BulletText("RB Bumper: Unlock Motors (Arm)");
        ImGui::BulletText("Back Button: Calibrate Gyro");
        ImGui::BulletText("Start Button: Toggle Headless Mode");
        ImGui::Spacing();
        ImGui::Separator();
        if (ImGui::Button("Close", ImVec2(-1, 0))) { ImGui::CloseCurrentPopup(); }
        ImGui::EndPopup();
      }

      ImGui::Text("Drone State: ");
      ImGui::SameLine();
      if (safety_lock) {
        ImGui::TextColored({0.95f, 0.20f, 0.20f, 1.0f}, "[SAFETY LOCKED]");
      } else {
        ImGui::TextColored({0.20f, 0.85f, 0.20f, 1.0f}, "[SAFETY OFF]");
      }
      if (headless_active) {
        ImGui::SameLine();
        ImGui::TextColored({0.0f, 0.85f, 0.85f, 1.0f}, "[HEADLESS]");
      }
      if (motors_armed) {
        ImGui::SameLine();
        ImGui::TextColored({0.95f, 0.35f, 0.10f, 1.0f}, "[ARMED]");
      }
      ImGui::Spacing();
      ImGui::Spacing();

      // --- FLIGHT CONTROLS ---
      ImGui::TextColored({0.4f, 0.8f, 0.4f, 1.0f}, "FLIGHT COMMANDS");
      ImGui::Separator();

      ImGui::BeginDisabled(safety_lock);

      const float sp = ImGui::GetStyle().ItemSpacing.x;
      const float btn_w = ImGui::GetContentRegionAvail().x;
      if (ColoredButton("LAND", {btn_w, 28}, {0.20f, 0.45f, 0.88f, 1.0f})) {
        pulse_flags |= 0x02;
        pulse_until = now + std::chrono::milliseconds(180);
      }
      ImGui::Spacing();
      if (ColoredButton("!! EMERGENCY STOP !!", {btn_w, 36}, {0.88f, 0.12f, 0.12f, 1.0f})) {
        pulse_flags |= 0x04;
        pulse_until = now + std::chrono::milliseconds(180);
      }
      ImGui::Separator();

      const float half_w = (btn_w - sp) * 0.5f;
      if (ColoredButton("360 FLIP", {half_w, 24}, {0.85f, 0.55f, 0.10f, 1.0f})) {
        pulse_flags |= 0x08;
        pulse_until = now + std::chrono::milliseconds(180);
      }
      ImGui::SameLine();
      {
        ImVec4 hl_col = headless_active ? ImVec4(0.10f, 0.72f, 0.75f, 1.0f) : ImVec4(0.35f, 0.38f, 0.42f, 1.0f);
        if (ColoredButton(headless_active ? "HEADLESS [ON]" : "HEADLESS [OFF]", {half_w, 24}, hl_col)) {
          pulse_flags |= 0x10;
          headless_active = !headless_active;
          pulse_until = now + std::chrono::milliseconds(180);
        }
      }

      if (ColoredButton("LOCK MOTORS", {half_w, 24}, {0.55f, 0.28f, 0.78f, 1.0f})) {
        pulse_flags |= 0x20;
        motors_armed = false;
        pulse_until = now + std::chrono::milliseconds(180);
      }
      ImGui::SameLine();
      if (ColoredButton("UNLOCK MOTORS", {half_w, 24}, {0.78f, 0.50f, 0.15f, 1.0f})) {
        pulse_flags |= 0x40;
        motors_armed = true;
        pulse_until = now + std::chrono::milliseconds(180);
      }

      if (ColoredButton("CALIBRATE GYRO", {half_w, 24}, {0.80f, 0.78f, 0.15f, 1.0f})) {
        pulse_flags |= 0x80;
        pulse_until = now + std::chrono::milliseconds(180);
      }
      ImGui::SameLine();
      if (ImGui::Button("Rotate Image", {half_w, 24})) {
        drone.command(tdg::direct::CameraCommand::Rotate);
      }

      ImGui::EndDisabled();
      ImGui::Separator();
      ImGui::TextColored({1.0f, 0.70f, 0.20f, 1.0f}, "! Xbox: Left stick springs to center.");
      ImGui::TextWrapped("Throttle uses left stick Y-axis (Mode 2).");
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ── Column 3: Telemetry ──────────────────────────────────────────
    ImGui::BeginChild("telemetry", {0, bot_h}, true);
    {
      ImGui::TextColored({0.4f, 0.8f, 0.4f, 1.0f}, "TRANSMITTER");
      ImGui::Separator();
      ImGui::SliderScalar("Roll",     ImGuiDataType_U8, &roll,     &kMin, &kMax);
      ImGui::SliderScalar("Pitch",    ImGuiDataType_U8, &pitch,    &kMin, &kMax);
      ImGui::SliderScalar("Throttle", ImGuiDataType_U8, &throttle, &kMin, &kMax);
      ImGui::SliderScalar("Yaw",      ImGuiDataType_U8, &yaw,      &kMin, &kMax);
      ImGui::Spacing();
      ImGui::Spacing();
      
      ImGui::TextColored({0.4f, 0.8f, 0.4f, 1.0f}, "DIAGNOSTICS");
      ImGui::Separator();
      ImGui::Text("Camera FPS:   %.1f", camera_fps);
      ImGui::Text("Display FPS:  %.1f", display_fps);
      ImGui::Text("Render FPS:   %.1f", ImGui::GetIO().Framerate);
      ImGui::TextWrapped("Control: <=200 Hz on change, 25 Hz keepalive");
      ImGui::Text("Frames:       %u", stats.frames);
      ImGui::Text("UDP pkts:     %u", stats.datagrams);
      ImGui::Text("Discards:     %u", stats.discarded);
      ImGui::Text("Decode err:   %u", stats.decode_errors);
      ImGui::Text("JPEG dropped: %u", stats.dropped_jpegs);
      if (is_recording || recorder.dropped() != 0) {
        ImGui::Text("REC dropped:  %u", recorder.dropped());
      }
      ImGui::Text("Video starts: %u", stats.video_restarts);
      ImGui::Spacing();
      ImGui::Spacing();
      
      ImGui::TextColored({0.4f, 0.8f, 0.4f, 1.0f}, "ACTION LOG");
      ImGui::Separator();
      for (const auto& log : ui_logs) {
        ImGui::TextWrapped("%s", log.c_str());
      }
    }
    ImGui::EndChild();
    } // end if (view_mode != 2)

    ImGui::End();

    // ── GL render ────────────────────────────────────────────────────
    ImGui::Render();
    int w = 0, h = 0;
    SDL_GL_GetDrawableSize(window, &w, &h);
    glViewport(0, 0, w, h);
    glClearColor(0.05f, 0.06f, 0.08f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // ── Recording: capture without blocking on FFmpeg pipe I/O ───────
    // Capture before SwapWindow so we read the frame just rendered. RGB pack
    // alignment must be 1 or widths whose row size is not divisible by 4 can
    // be padded past the destination buffer.
    const auto now_time = std::chrono::steady_clock::now();
    if (is_recording && !recorder.healthy()) stop_recording();
    if (is_recording && rec_w > 0 && rec_h > 0 &&
        now_time - last_rec_frame_time >= std::chrono::milliseconds(33)) {
      last_rec_frame_time = now_time;
      const ImVec2 display = ImGui::GetIO().DisplaySize;
      if (display.x > 0.0f && display.y > 0.0f) {
        const float sx = static_cast<float>(w) / display.x;
        const float sy = static_cast<float>(h) / display.y;
        const int rx = static_cast<int>(std::lround(rec_region_min.x * sx));
        const int bottom = static_cast<int>(std::lround(rec_region_max.y * sy));
        const int ry = h - bottom;
        if (rx >= 0 && ry >= 0 && rx + rec_w <= w && ry + rec_h <= h &&
            rec_pixels.size() == static_cast<std::size_t>(rec_w) * rec_h * 3u) {
          glPixelStorei(GL_PACK_ALIGNMENT, 1);
          glReadPixels(rx, ry, rec_w, rec_h, GL_RGB, GL_UNSIGNED_BYTE, rec_pixels.data());
          (void)recorder.submit(rec_pixels);  // latest recording frame wins if encoder is busy
        }
      }
    }

    SDL_GL_SwapWindow(window);

    // Keep the UI/input loop near 500 Hz, but yield aggressively so decode and
    // recording workers are not starved by a long busy-spin. The last fraction
    // of a millisecond uses scheduler yields rather than an additional 1 ms sleep.
    constexpr auto kLoopPeriod = std::chrono::microseconds(2000);
    for (;;) {
      const auto elapsed = std::chrono::steady_clock::now() - frame_start;
      if (elapsed >= kLoopPeriod) break;
      const auto remaining = kLoopPeriod - elapsed;
      if (remaining > std::chrono::microseconds(1200))
        SDL_Delay(1);
      else
        std::this_thread::yield();
    }
  }

  // Stop recording on exit
  stop_recording();

  // ── Cleanup ────────────────────────────────────────────────────────
  if (gamepad) SDL_GameControllerClose(gamepad);
  glDeleteTextures(1, &scanline_texture);
  glDeleteTextures(1, &texture);
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(gl);
  SDL_DestroyWindow(window);
  SDL_Quit();
}
