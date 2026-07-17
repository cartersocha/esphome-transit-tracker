#include "transit_tracker.h"
#include "string_utils.h"

#include <cstdlib>
#include <strings.h>
#include <map>
#include <algorithm>

#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "mbedtls/sha256.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/components/json/json_util.h"
#include "esphome/components/network/util.h"

namespace esphome {
namespace transit_tracker {

static const char *const TAG = "transit_tracker.component";

static constexpr int CONNECT_FAILURE_ERROR_THRESHOLD = 3;
static constexpr int CONNECT_FAILURE_REBOOT_THRESHOLD = 15;
static constexpr unsigned long HEARTBEAT_TIMEOUT_MS = 60000;
static constexpr int STALE_TRIP_SECONDS = 60;

static std::string compute_device_id() {
  uint8_t mac[6];
  esphome::get_mac_address_raw(mac);
  uint8_t hash[32];
  mbedtls_sha256(mac, sizeof(mac), hash, 0);
  char hex[65];
  for (int i = 0; i < 32; i++) {
    snprintf(hex + i * 2, 3, "%02x", hash[i]);
  }
  return std::string(hex);
}

static bool parse_hex_color(const std::string &str, uint32_t &out) {
  if (str.empty()) {
    return false;
  }
  char *end = nullptr;
  unsigned long value = std::strtoul(str.c_str(), &end, 16);
  if (end == str.c_str() || *end != '\0') {
    return false;
  }
  out = static_cast<uint32_t>(value);
  return true;
}

void TransitTracker::setup() {
  this->ws_client_.set_on_message([this](const std::string &payload) {
    this->handle_message_(payload);
  });

  this->ws_client_.set_on_connected([this]() {
    // defer the actual subscribe send and status update to loop()
    this->last_heartbeat_ = millis();
    this->has_ever_connected_ = true;
    this->consecutive_disconnects_ = 0;
    this->pending_subscribe_ = true;
  });

  this->ws_client_.set_on_disconnected([this]() {
    this->on_disconnect_();
  });

  {
    std::string user_agent;
    std::string headers;
    bool has_device_id = false;
    for (const auto &h : this->extra_headers_) {
      if (strcasecmp(h.first.c_str(), "user-agent") == 0) {
        user_agent = h.second;
      } else {
        if (strcasecmp(h.first.c_str(), "x-device-id") == 0) {
          has_device_id = true;
        }
        headers += h.first + ": " + h.second + "\r\n";
      }
    }

    if (!has_device_id) {
      auto device_id = compute_device_id();
      if (!device_id.empty()) {
        headers += "X-Device-Id: " + device_id + "\r\n";
      }
    }

    if (user_agent.empty()) {
#ifdef ESPHOME_PROJECT_NAME
      user_agent = ESPHOME_PROJECT_NAME "/" ESPHOME_PROJECT_VERSION
                   " ESPHome/" ESPHOME_VERSION " (" ESPHOME_VARIANT ") esp-idf/" IDF_VER;
#else
      user_agent = "ESPHome/" ESPHOME_VERSION " (" ESPHOME_VARIANT ") esp-idf/" IDF_VER;
#endif
    }

    this->ws_client_.set_user_agent(user_agent);
    this->ws_client_.set_headers(headers);
  }

  if (this->base_url_.empty()) {
    ESP_LOGW(TAG, "No base URL set; websocket will not start");
  } else {
    this->ws_client_.set_uri(this->base_url_);
    this->ws_client_.start();
  }

  this->set_interval("check_stale_trips", 10000, [this]() {
    if (!this->ws_client_.is_connected()) {
      return;
    }

    auto now = this->rtc_->now();
    if (!now.is_valid()) {
      return;
    }

    bool has_stale_trips = false;
    {
      std::lock_guard<std::mutex> lock(this->schedule_state_.mutex);
      for (const auto &trip : this->schedule_state_.trips) {
        if (now.timestamp - trip.departure_time > STALE_TRIP_SECONDS) {
          has_stale_trips = true;
          break;
        }
      }
    }

    if (has_stale_trips) {
      ESP_LOGW(TAG, "Stale trips detected (rtc=%d, last_heartbeat=%lu, uptime=%lu)",
               now.timestamp, this->last_heartbeat_.load(), millis());
    }
  });
}

void TransitTracker::loop() {
  if (this->pending_subscribe_.exchange(false)) {
    this->status_clear_error();
    this->send_subscribe_();
  }

  unsigned long heartbeat = this->last_heartbeat_.load();
  if (heartbeat != 0 && millis() - heartbeat > HEARTBEAT_TIMEOUT_MS) {
    ESP_LOGW(TAG, "No heartbeat for %lu ms (last_heartbeat=%lu, uptime=%lu)",
             millis() - heartbeat, heartbeat, millis());
    this->last_heartbeat_ = 0;
  }
}

void TransitTracker::dump_config() {
  ESP_LOGCONFIG(TAG, "Transit Tracker:");
  ESP_LOGCONFIG(TAG, "  Base URL: %s", this->base_url_.c_str());
  ESP_LOGCONFIG(TAG, "  Schedule: %s", this->schedule_string_.c_str());
  ESP_LOGCONFIG(TAG, "  Limit: %d", this->limit_);
  ESP_LOGCONFIG(TAG, "  List mode: %s", this->list_mode_.c_str());
  ESP_LOGCONFIG(TAG, "  Display departure times: %s", this->display_departure_times_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  Scroll Headsigns: %s", this->scroll_headsigns_ ? "true" : "false");
}

void TransitTracker::reconnect(const char *reason) {
  if (this->fully_closed_) {
    return;
  }

  ESP_LOGI(TAG, "Reconnecting websocket (reason: %s)", reason);
  this->last_heartbeat_ = 0;
  this->ws_client_.stop();

  if (this->base_url_.empty()) {
    ESP_LOGW(TAG, "Base URL is not set - cannot reconnect");
  } else {
    this->ws_client_.set_uri(this->base_url_);
    this->ws_client_.start();
  }
}

void TransitTracker::close(bool fully) {
  if (fully) {
    this->fully_closed_ = true;
  }
  this->ws_client_.stop();
}

void TransitTracker::on_shutdown() {
  this->cancel_interval("check_stale_trips");
  this->close(true);
}

void TransitTracker::on_disconnect_() {
  if (this->fully_closed_) {
    return;
  }

  int attempts = ++this->consecutive_disconnects_;
  ESP_LOGW(TAG, "Websocket disconnected (consecutive=%d, network_connected=%s, free_heap=%u)",
           attempts, esphome::network::is_connected() ? "yes" : "no",
           static_cast<unsigned>(esp_get_free_heap_size()));

  if (attempts >= CONNECT_FAILURE_ERROR_THRESHOLD) {
    this->status_set_error(LOG_STR("Failed to connect to WebSocket server"));
  }

  if (attempts >= CONNECT_FAILURE_REBOOT_THRESHOLD) {
    ESP_LOGE(TAG, "Could not connect to WebSocket server within %d attempts; rebooting to recover",
             CONNECT_FAILURE_REBOOT_THRESHOLD);
    App.reboot();
  }
}

void TransitTracker::send_subscribe_() {
  auto message = json::build_json([this](JsonObject root) {
    root["event"] = "schedule:subscribe";

    auto data = root["data"].to<JsonObject>();
    if (!this->feed_code_.empty()) {
      data["feedCode"] = this->feed_code_;
    }
    data["routeStopPairs"] = this->schedule_string_;
    data["limit"] = this->limit_;
    data["sortByDeparture"] = this->display_departure_times_;
    data["listMode"] = this->list_mode_;
  });

  ESP_LOGD(TAG, "Subscribing (%u bytes)", static_cast<unsigned>(message.size()));
  ESP_LOGV(TAG, "Subscribe payload: %s", message.c_str());
  if (!this->ws_client_.send_text(message)) {
    ESP_LOGW(TAG, "Subscribe send failed");
  }
}

void TransitTracker::handle_message_(const std::string &payload) {
  ESP_LOGV(TAG, "Received message (%u bytes): %s", static_cast<unsigned>(payload.size()), payload.c_str());

  bool valid = json::parse_json(payload, [this, &payload](JsonObject root) -> bool {
    auto event = root["event"].as<std::string>();

    if (event == "heartbeat") {
      ESP_LOGD(TAG, "Received heartbeat");
      this->last_heartbeat_ = millis();
      return true;
    }

    if (event != "schedule") {
      ESP_LOGW(TAG, "Ignoring unknown event '%s' (%u bytes)", event.c_str(),
               static_cast<unsigned>(payload.size()));
      return true;
    }

    ESP_LOGD(TAG, "Received schedule update (%u bytes)", static_cast<unsigned>(payload.size()));

    std::vector<Trip> new_trips;
    auto data = root["data"].as<JsonObject>();
    auto trip_array = data["trips"].as<JsonArray>();
    new_trips.reserve(trip_array.size());

    for (auto trip : trip_array) {
      std::string headsign = trip["headsign"].as<std::string>();
      for (const auto &abbr : this->abbreviations_) {
        size_t pos = headsign.find(abbr.first);
        if (pos != std::string::npos) {
          ESP_LOGV(TAG, "Applying abbreviation '%s' -> '%s'", abbr.first.c_str(), abbr.second.c_str());
          headsign.replace(pos, abbr.first.length(), abbr.second);
        }
      }

      auto route_id = trip["routeId"].as<std::string>();
      auto route_style = this->route_styles_.find(route_id);

      Color route_color = this->default_route_color_;
      std::string route_name = trip["routeName"].as<std::string>();

      if (route_style != this->route_styles_.end()) {
        route_color = route_style->second.color;
        route_name = route_style->second.name;
      } else if (!trip["routeColor"].isNull()) {
        auto color_str = trip["routeColor"].as<std::string>();
        uint32_t parsed_color;
        if (parse_hex_color(color_str, parsed_color)) {
          route_color = Color(parsed_color);
        } else if (!color_str.empty()) {
          ESP_LOGW(TAG, "Ignoring invalid routeColor '%s' for route %s",
                   color_str.c_str(), route_id.c_str());
        }
      }

      // stopId may be absent on older server versions; default to empty
      std::string stop_id = !trip["stopId"].isNull() ? trip["stopId"].as<std::string>() : "";

      new_trips.push_back({
        .route_id = route_id,
        .stop_id = stop_id,
        .route_name = route_name,
        .route_color = route_color,
        .headsign = headsign,
        .arrival_time = trip["arrivalTime"].as<time_t>(),
        .departure_time = trip["departureTime"].as<time_t>(),
        .is_realtime = trip["isRealtime"].as<bool>(),
      });
    }

    {
      std::lock_guard<std::mutex> lock(this->schedule_state_.mutex);
      this->schedule_state_.trips = std::move(new_trips);
      this->update_display_rows_();
    }

    return true;
  });

  if (!valid) {
    ESP_LOGW(TAG, "Failed to parse message (%u bytes); preview: %.120s",
             static_cast<unsigned>(payload.size()), payload.c_str());
    this->status_set_error(LOG_STR("Failed to parse schedule data"));
  }
}

void TransitTracker::set_abbreviations_from_text(const std::string &text) {
  this->abbreviations_.clear();
  for (const auto &line : split(text, '\n')) {
    auto parts = split(line, ';');

    if (parts.size() == 1) {
      // If only one part is provided, treat it as a removal (replace with empty string)
      this->add_abbreviation(parts[0], "");
      continue;
    }

    if (parts.size() != 2) {
      ESP_LOGW(TAG, "Invalid abbreviation line: %s", line.c_str());
      continue;
    }

    this->add_abbreviation(parts[0], parts[1]);
  }
}

void TransitTracker::set_route_styles_from_text(const std::string &text) {
  this->route_styles_.clear();
  for (const auto &line : split(text, '\n')) {
    auto parts = split(line, ';');
    if (parts.size() != 3) {
      ESP_LOGW(TAG, "Invalid route style line: %s", line.c_str());
      continue;
    }
    uint32_t color;
    if (!parse_hex_color(parts[2], color)) {
      ESP_LOGW(TAG, "Invalid route style color '%s' in line: %s", parts[2].c_str(), line.c_str());
      continue;
    }
    this->add_route_style(parts[0], parts[1], Color(color));
  }
}

void TransitTracker::draw_text_centered_(const char *text, Color color) {
  int display_center_x = this->display_->get_width() / 2;
  int display_center_y = this->display_->get_height() / 2;
  this->display_->print(display_center_x, display_center_y, this->font_, color, display::TextAlign::CENTER, text);
}

void TransitTracker::set_realtime_color(const Color &color) {
  this->realtime_color_ = color;
  this->realtime_color_dark_ = Color(
    (color.r * 0.5),
    (color.g * 0.5),
    (color.b * 0.5)
  );
}

const uint8_t realtime_icon[6][6] = {
  {0, 0, 0, 3, 3, 3},
  {0, 0, 3, 0, 0, 0},
  {0, 3, 0, 0, 2, 2},
  {3, 0, 0, 2, 0, 0},
  {3, 0, 2, 0, 0, 1},
  {3, 0, 2, 0, 1, 1}
};

void HOT TransitTracker::draw_realtime_icon_(int x, int y, int frame) {
  auto is_segment_lit = [frame](uint8_t segment) {
    switch (segment) {
      case 1: return frame >= 1 && frame <= 3;
      case 2: return frame >= 2 && frame <= 4;
      case 3: return frame >= 3 && frame <= 5;
      default: return false;
    }
  };

  const Color lit_color = this->realtime_color_;
  const Color unlit_color = this->realtime_color_dark_;

  for (uint8_t i = 0; i < 6; ++i) {
    for (uint8_t j = 0; j < 6; ++j) {
      uint8_t segment_number = realtime_icon[i][j];
      if (segment_number == 0) {
        continue;
      }

      Color icon_color = is_segment_lit(segment_number) ? lit_color : unlit_color;
      this->display_->draw_pixel_at(x + j, y + i, icon_color);
    }
  }
}

void HOT TransitTracker::draw_schedule() {
  if (this->display_ == nullptr) {
    ESP_LOGW(TAG, "No display attached, cannot draw schedule");
    return;
  }

  if (!esphome::network::is_connected()) {
    this->draw_text_centered_("Waiting for network", Color(0x252627));
    return;
  }

  if (!this->rtc_->now().is_valid()) {
    this->draw_text_centered_("Waiting for time sync", Color(0x252627));
    return;
  }

  if (this->base_url_.empty()) {
    this->draw_text_centered_("No base URL set", Color(0x252627));
    return;
  }

  if (this->status_has_error()) {
    this->draw_text_centered_("Error loading schedule", Color(0xFE4C5C));
    return;
  }

  if (!this->has_ever_connected_.load()) {
    this->draw_text_centered_("Loading...", Color(0x252627));
    return;
  }

  this->schedule_state_.mutex.lock();

  if (this->display_rows_.empty()) {
    bool no_trips = this->schedule_state_.trips.empty();
    this->schedule_state_.mutex.unlock();

    if (no_trips) {
      auto message = "No upcoming arrivals";
      if (this->display_departure_times_) {
        message = "No upcoming departures";
      }
      this->draw_text_centered_(message, Color(0x252627));
    }
    return;
  }

  int nominal_font_height = this->font_->get_ascender() + this->font_->get_descender();
  unsigned long uptime = millis();
  uint rtc_now = this->rtc_->now().timestamp;

  // Calculate realtime icon frame once per loop to save CPU
  const int num_frames = 6;
  const int idle_frame_duration = 3000;
  const int anim_frame_duration = 200;
  const int cycle_duration = idle_frame_duration + (num_frames - 1) * anim_frame_duration;
  unsigned long cycle_time = uptime % cycle_duration;
  int icon_frame = 0;
  if (cycle_time >= idle_frame_duration) {
    icon_frame = 1 + (cycle_time - idle_frame_duration) / anim_frame_duration;
  }

  int _;
  int total_times_width = 0;

  int num_total_rows = this->display_rows_.size();
  int num_pages = (num_total_rows + items_per_page - 1) / items_per_page;
  if (num_pages < 1) num_pages = 1;

  if (!this->double_time_) {
    this->page_index_ = 0;
  }

  int start_idx = (this->page_index_ % num_pages) * items_per_page;
  int end_idx = std::min(start_idx + items_per_page, num_total_rows);

  // Widest route badge across the visible rows -> every headsign starts at a fixed column
  auto get_max_route_width = [&](int start, int end) -> int {
    int max_w = 0;
    for (int i = start; i < end; i++) {
      int w, a, b;
      this->font_->measure(this->display_rows_[i].primary_trip->route_name.c_str(), &w, &a, &a, &b);
      if (w > max_w) max_w = w;
    }
    return max_w;
  };

  // Per-time-column max text width + whether any trip in the column is realtime,
  // so the times line up in fixed columns and the realtime icon gets reserved space.
  struct ColumnFormat {
      int max_text_width = 0;
      bool has_realtime = false;
  };

  auto get_column_formats = [&](int start, int end) -> std::vector<ColumnFormat> {
    int times_to_draw = this->double_time_ ? 2 : 1;
    std::vector<ColumnFormat> formats(times_to_draw);

    for (int i = start; i < end; i++) {
      const auto &row = this->display_rows_[i];
      for (int j = 0; j < times_to_draw; j++) {
        if (j < (int) row.trips.size()) {
          const Trip* t = row.trips[j];
          std::string ts = this->localization_.fmt_duration_from_now(
            this->display_departure_times_ ? t->departure_time : t->arrival_time,
            rtc_now
          );
          int w, a, b;
          this->font_->measure(ts.c_str(), &w, &a, &a, &b);

          if (w > formats[j].max_text_width) {
              formats[j].max_text_width = w;
          }
          if (t->is_realtime) {
              formats[j].has_realtime = true;
          }
        }
      }
    }
    return formats;
  };

  // Helper lambda to calculate scroll duration for a range of rows
  auto calc_scroll_duration = [&](int start, int end) -> int {
    if (!this->scroll_headsigns_) return 0;

    int max_route_w = get_max_route_width(start, end);
    auto col_formats = get_column_formats(start, end);

    int total_times_w = 0;
    for (const auto& fmt : col_formats) {
        if (fmt.max_text_width > 0) {
            total_times_w += fmt.max_text_width + (fmt.has_realtime ? 8 : 0) + 2;
        }
    }

    int largest_headsign_overflow = 0;

    for (int i = start; i < end; i++) {
        const auto &row = this->display_rows_[i];
        int headsign_w, a, b;
        this->font_->measure(row.primary_trip->headsign.c_str(), &headsign_w, &a, &a, &b);

        int headsign_clipping_end = this->display_->get_width() - total_times_w;
        int headsign_clipping_start = max_route_w + 3;
        int headsign_max_width = headsign_clipping_end - headsign_clipping_start;

        if (headsign_max_width <= 0) continue;

        int headsign_overflow = headsign_w - headsign_max_width;

        static constexpr int min_scroll_threshold = 3;
        if (headsign_overflow >= min_scroll_threshold) {
            largest_headsign_overflow = std::max(largest_headsign_overflow, headsign_overflow);
        }
    }

    if (largest_headsign_overflow >= 3) {
        return idle_time_left + idle_time_right + 2 * (largest_headsign_overflow * 1000 / scroll_speed);
    }
    return 0;
  };

  int scroll_cycle_duration = calc_scroll_duration(start_idx, end_idx);
  int page_dwell = std::max(5000, scroll_cycle_duration);

  if (this->double_time_ && uptime - this->last_page_change_ > (unsigned long)page_dwell) {
      this->page_index_ = (this->page_index_ + 1) % num_pages;
      this->last_page_change_ = uptime;
      this->scroll_cycle_start_ = uptime;

      start_idx = (this->page_index_ % num_pages) * items_per_page;
      end_idx = std::min(start_idx + items_per_page, num_total_rows);
      scroll_cycle_duration = calc_scroll_duration(start_idx, end_idx);
  }

  int effective_scroll_duration = scroll_cycle_duration;

  int current_max_route_width = get_max_route_width(start_idx, end_idx);
  auto col_formats = get_column_formats(start_idx, end_idx);

  total_times_width = 0;
  for (const auto& fmt : col_formats) {
      if (fmt.max_text_width > 0) {
          total_times_width += fmt.max_text_width + (fmt.has_realtime ? 8 : 0) + 2;
      }
  }

  // Calculate vertical centering
  int num_rows_on_page = end_idx - start_idx;
  int max_trips_height = num_rows_on_page * nominal_font_height - this->font_->get_descender();
  int y_offset = (this->display_->get_height() - max_trips_height) / 2;
  if (y_offset < 0) y_offset = 0;

  for (int idx = start_idx; idx < end_idx; idx++) {
    const auto &row = this->display_rows_[idx];
    // Draw route name
    this->display_->print(0, y_offset, this->font_, row.primary_trip->route_color, display::TextAlign::TOP_LEFT, row.primary_trip->route_name.c_str());

    // Draw times from right to left in fixed-width columns so they line up
    int time_x = this->display_->get_width() + 1;
    int times_to_draw = this->double_time_ ? 2 : 1;

    for (int i = times_to_draw - 1; i >= 0; i--) {
        const auto& fmt = col_formats[i];
        if (fmt.max_text_width == 0) continue;

        int col_width = fmt.max_text_width + (fmt.has_realtime ? 8 : 0);

        if (i < (int) row.trips.size()) {
            const Trip* t = row.trips[i];
            std::string time_str = this->localization_.fmt_duration_from_now(
              this->display_departure_times_ ? t->departure_time : t->arrival_time,
              rtc_now
            );
            Color color = t->is_realtime ? this->realtime_color_ : Color(0xa7a7a7);

            // Left-align text within the column for a consistent gap from the icon
            int text_x = time_x - fmt.max_text_width;

            this->display_->print(text_x, y_offset, this->font_, color, display::TextAlign::TOP_LEFT, time_str.c_str());

            if (t->is_realtime) {
               int icon_x = text_x - 8;
               int icon_y = y_offset + nominal_font_height - 11;
               this->draw_realtime_icon_(icon_x, icon_y, icon_frame);
            }
        }

        time_x -= (col_width + 2);
    }

    // Calculate headsign clipping area - must match scroll calculation exactly
    int headsign_clipping_start = current_max_route_width + 3;
    // Use the max times width to define the consistent right edge for headsigns
    int headsign_clipping_end = this->display_->get_width() - total_times_width;
    int headsign_max_width = headsign_clipping_end - headsign_clipping_start;

    int headsign_actual_width;
    this->font_->measure(row.primary_trip->headsign.c_str(), &headsign_actual_width, &_, &_, &_);

    int headsign_overflow = headsign_actual_width - headsign_max_width;

    // Apply minimum threshold to prevent flickering
    static constexpr int min_scroll_threshold = 3;
    if (headsign_overflow > 0 && headsign_overflow < min_scroll_threshold) {
      headsign_overflow = 0;
    }

    // Calculate scroll offset
    int scroll_offset = 0;
    if (headsign_overflow > 0 && effective_scroll_duration > 0) {
      int scroll_time = headsign_overflow * 1000 / scroll_speed;
      int scroll_cycle_time = (uptime - this->scroll_cycle_start_) % effective_scroll_duration;

      if (scroll_cycle_time < idle_time_left) {
        // Idle left
      } else if (scroll_cycle_time < idle_time_left + scroll_time) {
        int time_since_scroll_start = scroll_cycle_time - idle_time_left;
        scroll_offset = time_since_scroll_start * scroll_speed / 1000;
      } else if (scroll_cycle_time < idle_time_left + scroll_time + idle_time_right) {
        scroll_offset = headsign_overflow;
      } else if (scroll_cycle_time < idle_time_left + 2 * scroll_time + idle_time_right) {
        int time_since_scroll_start = scroll_cycle_time - (idle_time_left + scroll_time + idle_time_right);
        scroll_offset = headsign_overflow - (time_since_scroll_start * scroll_speed / 1000);
      }
    }

    // Draw headsign with clipping
    if (headsign_clipping_end > headsign_clipping_start) {
      this->display_->start_clipping(headsign_clipping_start, y_offset - 2, headsign_clipping_end, y_offset + nominal_font_height + 2);
      this->display_->print(headsign_clipping_start - scroll_offset, y_offset, this->font_, row.primary_trip->headsign.c_str());
      this->display_->end_clipping();
    }

    y_offset += nominal_font_height;
  }

  this->schedule_state_.mutex.unlock();
}

void TransitTracker::update_display_rows_() {
  this->display_rows_.clear();
  size_t max_trips = this->double_time_ ? 2 : 1;

  if (this->double_time_) {
    // Group by route_id + stop_id using std::map (alphabetically sorted by route_id)
    std::map<std::pair<std::string, std::string>, std::vector<const Trip*>> grouped_trips;
    for (const Trip &trip : this->schedule_state_.trips) {
      auto key = std::make_pair(trip.route_id, trip.stop_id);
      if (grouped_trips[key].size() < max_trips) {
        grouped_trips[key].push_back(&trip);
      }
    }

    int rows_processed = 0;
    for (const auto &group : grouped_trips) {
      if (rows_processed >= this->limit_) break;
      const auto &trips = group.second;
      if (trips.empty()) continue;

      DisplayRow row;
      row.primary_trip = trips[0];
      row.trips = trips;

      this->display_rows_.push_back(row);
      rows_processed++;
    }
  } else {
    // Group by route_id + stop_id but preserve server order (time sorted)
    std::map<std::pair<std::string, std::string>, size_t> group_indices;

    for (const Trip &trip : this->schedule_state_.trips) {
      auto key = std::make_pair(trip.route_id, trip.stop_id);

      auto it = group_indices.find(key);
      if (it != group_indices.end()) {
        size_t index = it->second;
        if (this->display_rows_[index].trips.size() < max_trips) {
          this->display_rows_[index].trips.push_back(&trip);
        }
      } else {
        if (this->display_rows_.size() >= (size_t) this->limit_) {
          continue;
        }

        DisplayRow row;
        row.primary_trip = &trip;
        row.trips.push_back(&trip);

        this->display_rows_.push_back(row);
        group_indices[key] = this->display_rows_.size() - 1;
      }
    }
  }
}

}  // namespace transit_tracker
}  // namespace esphome
