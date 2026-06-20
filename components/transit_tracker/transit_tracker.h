#pragma once

#include <atomic>
#include <map>
#include <utility>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/display/display.h"
#include "esphome/components/font/font.h"
#include "esphome/components/time/real_time_clock.h"

#include "schedule_state.h"
#include "localization.h"
#include "websocket_client.h"

namespace esphome {
namespace transit_tracker {

struct RouteStyle {
  std::string name;
  Color color;
};

class TransitTracker : public Component {
  public:
    void setup() override;
    void loop() override;
    void dump_config() override;
    void on_shutdown() override;

    float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

    bool is_connected() const { return this->ws_client_.is_connected(); }
    void reconnect(const char *reason);
    void close(bool fully = false);

    void draw_schedule();

    Localization* get_localization() { return &this->localization_; }

    void set_display(display::Display *display) { display_ = display; }
    void set_font(font::Font *font) { font_ = font; }
    void set_rtc(time::RealTimeClock *rtc) { rtc_ = rtc; }

    void set_base_url(const std::string &base_url) { base_url_ = base_url; }
    void set_feed_code(const std::string &feed_code) { feed_code_ = feed_code; }
    void set_display_departure_times(bool display_departure_times) { display_departure_times_ = display_departure_times; }
    void set_schedule_string(const std::string &schedule_string) { schedule_string_ = schedule_string; }
    void set_list_mode(const std::string &list_mode) { list_mode_ = list_mode; }
    void set_limit(int limit) { limit_ = limit; }
    void set_scroll_headsigns(bool scroll_headsigns) { scroll_headsigns_ = scroll_headsigns; }
    void set_double_time(bool double_time) { double_time_ = double_time; }

    void set_unit_display(UnitDisplay unit_display) { this->localization_.set_unit_display(unit_display); }
    void add_abbreviation(const std::string &from, const std::string &to) { abbreviations_[from] = to; }
    void add_header(const std::string &name, const std::string &value) { extra_headers_.emplace_back(name, value); }
    void set_default_route_color(const Color &color) { default_route_color_ = color; }
    void add_route_style(const std::string &route_id, const std::string &name, const Color &color) { route_styles_[route_id] = RouteStyle{name, color}; }

    void set_abbreviations_from_text(const std::string &text);
    void set_route_styles_from_text(const std::string &text);

    void set_realtime_color(const Color &color);

    struct DisplayRow {
      const Trip* primary_trip;
      std::vector<const Trip*> trips;
    };

  protected:
    static constexpr int scroll_speed = 10; // pixels/second
    static constexpr int idle_time_left = 8000;
    static constexpr int idle_time_right = 2000;

    void draw_text_centered_(const char *text, Color color);
    void draw_realtime_icon_(int x, int y, int frame);

    void update_display_rows_();
    std::vector<DisplayRow> display_rows_;

    Localization localization_{};
    ScheduleState schedule_state_;

    display::Display *display_;
    font::Font *font_;
    time::RealTimeClock *rtc_;

    WebSocketClient ws_client_;

    void handle_message_(const std::string &payload);
    void send_subscribe_();
    void on_disconnect_();

    std::atomic<int> consecutive_disconnects_{0};
    std::atomic<unsigned long> last_heartbeat_{0};
    std::atomic<bool> has_ever_connected_{false};
    std::atomic<bool> pending_subscribe_{false};
    std::atomic<bool> fully_closed_{false};

    std::string base_url_;
    std::string feed_code_;
    std::string schedule_string_;
    std::string list_mode_;
    bool display_departure_times_ = true;
    int limit_;

    std::vector<std::pair<std::string, std::string>> extra_headers_;
    std::map<std::string, std::string> abbreviations_;
    Color default_route_color_ = Color(0x028e51);
    std::map<std::string, RouteStyle> route_styles_;
    bool scroll_headsigns_ = false;
    bool double_time_ = false;

    // Cached scroll state to prevent mid-cycle jumps
    unsigned long scroll_cycle_start_ = 0;

    // Pagination
    int page_index_ = 0;
    unsigned long last_page_change_ = 0;
    static constexpr int items_per_page = 3;

    Color realtime_color_ = Color(0x20FF00);
    Color realtime_color_dark_ = Color(0x00A700);
};


}  // namespace transit_tracker
}  // namespace esphome
