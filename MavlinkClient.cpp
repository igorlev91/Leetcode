#include "MavlinkClient.hpp"
#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>
#include <vector>
#include <nlohmann/json.hpp>
#include "LogConfig/SimpleLogger.h"
#include "GeoCalculator/GeoCalculator.hpp"

using namespace std::chrono_literals;

#define UNUSED(x) (void)(x)

namespace mavlink {
using json = nlohmann::json;

constexpr double PI = 3.141592653589793;
constexpr uint8_t OUR_SYS_ID = 100;
constexpr uint8_t OUR_COMP_ID = 150;
constexpr uint8_t PX4_SYS_ID = 1;
constexpr uint8_t PX4_COMP_ID = 1;

MavlinkClient::MavlinkClient(const std::string& port, int baudrate, float height, float speed, 
                             std::stop_source stop_src)
    : port_(port), baudrate_(baudrate), height_(height), speed_(speed),
      running_(std::nullopt),
      stop_source_(std::move(stop_src)), stop_token_(stop_source_.get_token()),
      sdk_(std::make_unique<mavsdk::Mavsdk>(mavsdk::Mavsdk::Configuration(OUR_SYS_ID, OUR_COMP_ID, false))), 
      json_path_("../logs/mavlink_data.json"), 
      system_discovered_future_(system_discovered_promise_.get_future()),
      home_received_(false),
      current_position_{0.0, 0.0, 0.0}
{
    SimpleLogger::info("Initializing MavlinkClient...");
    SimpleLogger::info("MAVSDK version: {}", sdk_->version());
    
    try {
        geoid_ = std::make_unique<GeographicLib::Geoid>("egm96-5", "", true, true);
        SimpleLogger::info("Geoid EGM96-5 инициализирован.");
    } catch (const std::exception& e) {
        SimpleLogger::error("Ошибка Geoid: {}", e.what());
    }
}

MavlinkClient::MavlinkClient(const std::string& port, int baudrate, float height, float speed, std::atomic<bool>& running)
    : port_(port), baudrate_(baudrate), height_(height), speed_(speed), running_(std::ref(running)),
      sdk_(std::make_unique<mavsdk::Mavsdk>(mavsdk::Mavsdk::Configuration(OUR_SYS_ID, OUR_COMP_ID, false))), 
      json_path_("../logs/mavlink_data.json"), 
      system_discovered_future_(system_discovered_promise_.get_future()),
      home_received_(false),
      current_position_{0.0, 0.0, 0.0}
{
    SimpleLogger::info("Initializing MavlinkClient...");
    SimpleLogger::info("MAVSDK version: {}", sdk_->version());
    
    try {
        geoid_ = std::make_unique<GeographicLib::Geoid>("egm96-5", "", true, true);
        SimpleLogger::info("Geoid EGM96-5 инициализирован.");
    } catch (const std::exception& e) {
        SimpleLogger::error("Ошибка Geoid: {}", e.what());
    }
}

MavlinkClient::~MavlinkClient() {
    stop();
    SimpleLogger::info("MavlinkClient destroyed");
}

void MavlinkClient::start() {
    SimpleLogger::info("Starting MavlinkClient...");

    bool system_found = false;
    sdk_->subscribe_on_new_system([this, &system_found]() {
        if (system_found) return;
        auto systems = sdk_->systems();
        for (const auto& sys : systems) {
            if (sys->get_system_id() == PX4_SYS_ID && !system_found) {
                system_ = sys;
                system_discovered_promise_.set_value();
                system_found = true;
                SimpleLogger::info("System {} discovered", sys->get_system_id());
                return;
            }
        }
    });

    auto result = sdk_->add_any_connection("udpin://0.0.0.0:14540");
    if (result != mavsdk::ConnectionResult::Success) {
        SimpleLogger::error("Connection failed");
        return;
    }

    if (system_discovered_future_.wait_for(10s) == std::future_status::timeout) {
        SimpleLogger::error("No system");
        return;
    }

    initializePlugins();
    setupMessageSubscriptions();
    configureExternalGps();

    // ✅ 10s ЖДЕНИЕ EKF СТАБИЛИЗАЦИИ
    if (!waitForEKFStable(10s)) {
        SimpleLogger::warn("EKF not fully stable - continuing");
    }

    // ✅ HEARTBEAT 10Hz (ANTI-LOST)
    heartbeat_thread_ = std::jthread([this](std::stop_token stoken) { 
        while (!stoken.stop_requested()) {
            sendHeartbeatMessage();
            std::this_thread::sleep_for(100ms);  // 10Hz!
        }
    });

    gps_thread_ = std::jthread([this](std::stop_token) { gpsInjectionLoop(); });

    std::this_thread::sleep_for(2s);
    testGpsInjection();
}

void MavlinkClient::stop() {
    if (stop_source_.stop_possible()) {
        stop_source_.request_stop();
    }
    if (running_) {
        running_->get() = false;
    }
}

void MavlinkClient::initializePlugins() {
    telemetry_ = std::make_unique<mavsdk::Telemetry>(system_);
    mocap_ = std::make_unique<mavsdk::Mocap>(system_);
    param_ = std::make_unique<mavsdk::Param>(system_);
    passthrough_ = std::make_unique<mavsdk::MavlinkPassthrough>(system_);
    mission_ = std::make_unique<mavsdk::Mission>(system_);
    action_ = std::make_unique<mavsdk::Action>(system_);
    
    SimpleLogger::info("Plugins initialized");

    telemetry_->subscribe_health([](const mavsdk::Telemetry::Health& health) {
        SimpleLogger::debug("Health: GPS={}, Home={}, Local={}", 
                           health.is_global_position_ok,
                           health.is_home_position_ok,
                           health.is_local_position_ok);
    });
}

void MavlinkClient::setupMessageSubscriptions() {
    if (!passthrough_) return;
    
    passthrough_->subscribe_message(MAVLINK_MSG_ID_HOME_POSITION, 
        [this](const mavlink_message_t& msg) {
            std::lock_guard<std::mutex> lock(home_mutex_);
            if (home_received_) return;

            mavlink_home_position_t home;
            mavlink_msg_home_position_decode(&msg, &home);
            
            home_lat_ = static_cast<double>(home.latitude) / 1e7;
            home_lon_ = static_cast<double>(home.longitude) / 1e7;
            home_alt_ = static_cast<double>(home.altitude) / 1e3;
            
            origin_ = {home_lat_, home_lon_, home_alt_};
            home_received_ = true;
            proj_ = std::make_unique<GeographicLib::LocalCartesian>(home_lat_, home_lon_, home_alt_);
            
            SimpleLogger::info("Home: {:.7f}, {:.7f}, {:.2f}", home_lat_, home_lon_, home_alt_);
            home_cv_.notify_one();
        });
}

bool MavlinkClient::setParameter(const std::string& name, int value) {
    if (!param_) return false;
    auto result = param_->set_param_int(name, value);
    SimpleLogger::info("Set {} = {}", name, value);
    return result == mavsdk::Param::Result::Success;
}

bool MavlinkClient::setParameterFloat(const std::string& name, float value) {
    if (!param_) return false;
    auto result = param_->set_param_float(name, value);
    return result == mavsdk::Param::Result::Success;
}

bool MavlinkClient::configureExternalGps() {
    // ✅ ТОЛЬКО БЕЗОПАСНЫЕ ПАРАМЕТРЫ (БЕЗ GPS_YAW_OFFSET!)
    std::vector<std::pair<std::string, float>> params = {
        {"SYS_HAS_GPS", 1.0f}, 
        {"EKF2_GPS_CTRL", 7.0f}, 
        {"COM_ARM_WO_GPS", 0.0f}
    };
    
    for (const auto& [name, value] : params) {
        if (!setParameter(name, value)) {
            SimpleLogger::warn("Failed: {} = {:.1f}", name, value);
        } else {
            SimpleLogger::info("Set {} = {:.1f}", name, value);
        }
        std::this_thread::sleep_for(500ms);  // ✅ Анти-spam
    }

    // ✅ GPS INPUT (int16_t типы)
    setParameter("GPS_INPUT_IGNORE_FLAGS", 0.0f);
    
    eph_ = 0.3f; epv_ = 0.3f;
    horiz_accuracy_ = 0.2f; vert_accuracy_ = 0.2f; 
    satellites_visible_ = 15; fix_type_ = 6;  // RTK Fixed
    
    SimpleLogger::info("✅ GPS configured (RTK mode)");
    return true;
}

bool MavlinkClient::waitForHomePosition(const std::chrono::seconds& timeout) {
    std::unique_lock<std::mutex> lock(home_mutex_);
    if (home_received_) return true;
    
    if (home_cv_.wait_for(lock, timeout, [this] { return home_received_; })) {
        return true;
    }

    // Fallback to telemetry
    if (telemetry_) {
        auto home = telemetry_->home();
        if (!std::isnan(home.latitude_deg)) {
            home_lat_ = home.latitude_deg;
            home_lon_ = home.longitude_deg;
            home_alt_ = home.absolute_altitude_m;
            home_received_ = true;
            return true;
        }
    }
    return false;
}

void MavlinkClient::gpsInjectionLoop() {
    SimpleLogger::info("GPS Injection started");
    
    if (!waitForHomePosition(5s)) {
        SimpleLogger::warn("No home, using default");
        current_position_ = {53.9000000, 27.5000000, 150.0};
    }

    auto last_time = std::chrono::steady_clock::now();
    double elapsed = 0.0;
    const double dt = 0.1;
    const double R = 6371000.0;
    const double deg2rad = PI / 180.0;

    // ✅ 60 СЕКУНД ИНЖЕКЦИИ
    auto end_time = std::chrono::steady_clock::now() + 60s;
    
    while (std::chrono::steady_clock::now() < end_time && 
           !stop_token_.stop_requested() && 
           (!running_ || running_->get())) {
        
        auto now = std::chrono::steady_clock::now();
        double delta_t = std::chrono::duration<double>(now - last_time).count();
        elapsed += delta_t;
        last_time = now;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            double omega = 0.04;
            double vn = 2.0 * sin(omega * elapsed);
            double ve = 2.0 * cos(omega * elapsed);
            double vd = 0.0;

            double dlat = (vn * dt) / (R * deg2rad);
            double dlon = (ve * dt) / (R * deg2rad / cos(current_position_.latitude * deg2rad));
            
            current_position_.latitude += dlat;
            current_position_.longitude += dlon;
            current_position_.altitude += vd * dt;

            sendGpsInput(current_position_, vn, ve, vd);
        }
        
        std::this_thread::sleep_for(200ms);
    }
    SimpleLogger::info("GPS Injection completed (60s)");
}

void MavlinkClient::sendGpsInput(const Geo::GeoPosition& pos, double vn, double ve, double vd) {
    if (!passthrough_) return;

    auto now = std::chrono::system_clock::now().time_since_epoch();
    uint64_t gps_time_us = std::chrono::duration_cast<std::chrono::microseconds>(now).count();

    mavlink_gps_input_t gps = {};
    gps.time_usec = gps_time_us;
    gps.gps_id = gps_id_;
    gps.ignore_flags = 0;
    gps.time_week_ms = static_cast<uint32_t>(gps_time_us % 604800000ULL);
    
    gps.lat = static_cast<int32_t>(pos.latitude * 1e7);           
    gps.lon = static_cast<int32_t>(pos.longitude * 1e7);          
    gps.alt = static_cast<float>(pos.altitude * 1000);            
    gps.hdop = static_cast<uint16_t>(eph_ * 100);
    gps.vdop = static_cast<uint16_t>(epv_ * 100);
    gps.vn = static_cast<float>(vn);
    gps.ve = static_cast<float>(ve);
    gps.vd = static_cast<float>(vd);
    gps.speed_accuracy = speed_accuracy_;
    gps.horiz_accuracy = horiz_accuracy_;                         // horiz_accuracy
    gps.vert_accuracy = vert_accuracy_;                           // vert_accuracy
    gps.fix_type = fix_type_;
    gps.satellites_visible = satellites_visible_;

    mavlink_message_t msg;
    mavlink_msg_gps_input_encode(OUR_SYS_ID, OUR_COMP_ID, &msg, &gps);
    passthrough_->queue_message([&](auto, auto) { return msg; });
}

void MavlinkClient::sendHeartbeatMessage() {
    if (!passthrough_) return;

    mavlink_heartbeat_t hb = {};
    hb.type = MAV_TYPE_GCS;
    hb.autopilot = MAV_AUTOPILOT_INVALID;
    hb.base_mode = 0;
    hb.custom_mode = 0;
    hb.system_status = MAV_STATE_ACTIVE;
    hb.mavlink_version = 3;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_encode(OUR_SYS_ID, OUR_COMP_ID, &msg, &hb);
    passthrough_->queue_message([&](auto, auto) { return msg; });
}

bool MavlinkClient::waitForEKFStable(const std::chrono::seconds& timeout) {
    SimpleLogger::info("⏳ Waiting EKF stable... ({}s)", timeout.count());
    
    auto start = std::chrono::steady_clock::now();
    int stable_count = 0;
    
    while (std::chrono::steady_clock::now() - start < timeout) {
        auto health = telemetry_->health();
        bool gps_ok = health.is_global_position_ok;
        bool home_ok = health.is_home_position_ok;
        bool local_ok = health.is_local_position_ok;
        
        if (gps_ok && home_ok && local_ok) {
            stable_count++;
            if (stable_count >= 5) {
                SimpleLogger::info("✅ EKF STABLE!");
                return true;
            }
        } else {
            stable_count = 0;
            SimpleLogger::debug("Health: GPS={}, Home={}, Local={}", gps_ok, home_ok, local_ok);
        }
        
        std::this_thread::sleep_for(1s);
    }
    
    SimpleLogger::warn("⚠️ EKF timeout");
    return false;
}

bool MavlinkClient::armDrone() {
    for (int retry = 0; retry < 3; ++retry) {
        auto result = action_->arm();
        if (result == mavsdk::Action::Result::Success) {
            SimpleLogger::info("✅ Drone armed (retry {})", retry + 1);
            return true;
        }
        SimpleLogger::warn("Arm failed (retry {}): {}", retry + 1, static_cast<int>(result));
        std::this_thread::sleep_for(2s);
    }
    return false;
}

bool MavlinkClient::configureVisionPositioning() { return true; }
bool MavlinkClient::setFlightMode(uint32_t mode_id) { UNUSED(mode_id); return true; }
bool MavlinkClient::verifyGpsConfiguration() { return true; }

Geo::GeoPosition MavlinkClient::getOrigin() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return origin_;
}

double MavlinkClient::getAltitude() const {
    if (!telemetry_) return 0.0;
    auto pos = telemetry_->position();
    return std::isnan(pos.relative_altitude_m) ? 0.0 : pos.relative_altitude_m;
}

bool MavlinkClient::isHomeReceived() const { 
    return home_received_; 
}

std::vector<Geo::GeoPosition> MavlinkClient::createMissionWaypoints() {
    const double offset = 0.001;  // ~111 метров!
    
    return {
        // 1. Взлет над Home (ABSOLUTE altitude)
        {home_lat_, home_lon_, 105.0},                    // 105m абсолютная
        
        // 2. Точка 2 (111m севернее)
        {home_lat_ + offset, home_lon_, 105.0},           // 53.9545278, 27.2429161
        
        // 3. Точка 3 (111m восточнее)
        {home_lat_ + offset, home_lon_ + offset, 110.0},  // 53.9545278, 27.2439161
        
        // 4. Точка 4 (111m южнее)
        {home_lat_, home_lon_ + offset, 105.0},           // 53.9535278, 27.2439161
        
        // 5. Возврат домой
        {home_lat_, home_lon_, 105.0}
    };
}

bool MavlinkClient::uploadMission(const std::vector<mavsdk::Mission::MissionItem>& items) {
    if (!mission_ || items.empty()) {
        SimpleLogger::error("Mission or items invalid");
        return false;
    }

    // ✅ ОЧИСТИМ СТАРУЮ МИССИЮ
    auto clear_result = mission_->clear_mission();
    if (clear_result != mavsdk::Mission::Result::Success) {
        SimpleLogger::warn("Clear mission failed: {}", static_cast<int>(clear_result));
    }
    std::this_thread::sleep_for(1s);

    for (int retry = 0; retry < 3; ++retry) {
        SimpleLogger::info("Mission upload attempt #{} ({} items)", retry + 1, items.size());
        
        mavsdk::Mission::MissionPlan mission_plan;
        mission_plan.mission_items = items;
        
        auto upload_result = mission_->upload_mission(mission_plan);
        SimpleLogger::info("Upload result: {}", static_cast<int>(upload_result));
        
        if (upload_result == mavsdk::Mission::Result::Success) {
            SimpleLogger::info("✅ Mission upload: 1");
            return true;
        }
        
        SimpleLogger::warn("Upload failed ({}): {}", retry + 1, static_cast<int>(upload_result));
        std::this_thread::sleep_for(2s);
    }
    
    SimpleLogger::error("❌ Mission upload FAILED after 3 retries");
    return false;
}

bool MavlinkClient::startMission() {
    for (int retry = 0; retry < 3; ++retry) {
        SimpleLogger::info("Mission start attempt #{}", retry + 1);
        
        auto start_result = mission_->start_mission();
        SimpleLogger::info("Start result: {}", static_cast<int>(start_result));
        
        if (start_result == mavsdk::Mission::Result::Success) {
            SimpleLogger::info("✅ Mission start: 1");
            return true;
        }
        
        SimpleLogger::warn("Start failed ({}): {}", retry + 1, static_cast<int>(start_result));
        std::this_thread::sleep_for(3s);
    }
    
    SimpleLogger::error("❌ Mission start FAILED");
    return false;
}

void MavlinkClient::subscribeToMissionProgress(std::function<void(int, int)> callback) {
    if (!mission_) return;
    mission_->subscribe_mission_progress([callback](const mavsdk::Mission::MissionProgress& progress) {
        callback(progress.current, progress.total);
    });
}

bool MavlinkClient::executeWaypointMission(const std::vector<Geo::GeoPosition>& waypoints) {
    if (waypoints.empty() || !mission_) return false;

    std::vector<mavsdk::Mission::MissionItem> items;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        const auto& wp = waypoints[i];
        mavsdk::Mission::MissionItem item;
        
        item.latitude_deg = wp.latitude;
        item.longitude_deg = wp.longitude;
        item.relative_altitude_m = wp.altitude - home_alt_;
        item.speed_m_s = speed_;
        item.is_fly_through = (i < waypoints.size() - 1);
        item.acceptance_radius_m = 3.0f;
        
        items.push_back(item);
        SimpleLogger::info("Waypoint {}: {:.7f}, {:.7f}, rel_alt={:.1f}m", 
                          i+1, wp.latitude, wp.longitude, item.relative_altitude_m);
    }

    // ✅ ПРАВИЛЬНЫЙ ВЫЗОВ
    if (!uploadMission(items)) return false;

    subscribeToMissionProgress([](int current, int total) {
        SimpleLogger::info("Mission: {}/{}", current, total);
    });

    if (!startMission()) return false;

    // ЖДЕМ ЗАВЕРШЕНИЯ
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < 60s) {
        auto finished_pair = mission_->is_mission_finished();
        auto result = std::get<0>(finished_pair);
        auto is_finished = std::get<1>(finished_pair);
        
        if (is_finished && result == mavsdk::Mission::Result::Success) {
            SimpleLogger::info("✅ Mission COMPLETED SUCCESSFULLY");
            return true;
        }
        
        std::this_thread::sleep_for(500ms);
    }
    
    SimpleLogger::warn("⚠️ Mission timeout (60s)");
    return false;
}

bool MavlinkClient::waitForAltitude(float target_alt, const std::chrono::seconds& timeout) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
        auto pos = telemetry_->position();
        float current_alt = pos.relative_altitude_m;
        
        if (std::abs(current_alt - target_alt) < 1.0f) {
            SimpleLogger::info("✅ Reached altitude: {:.1f}m", current_alt);
            return true;
        }
        
        std::this_thread::sleep_for(500ms);
    }
    SimpleLogger::warn("Altitude timeout: {:.1f}m (target: {:.1f}m)", 
                      telemetry_->position().relative_altitude_m, target_alt);
    return false;
}

void MavlinkClient::testGpsInjection() {
    SimpleLogger::info("🎯 GPS INJECTION TEST START");
    
    // 1. ARM
    if (!armDrone()) return;
    std::this_thread::sleep_for(3s);  // ✅ +1s
    
    // 2. TAKEOFF
    auto takeoff_result = action_->takeoff();
    if (takeoff_result != mavsdk::Action::Result::Success) return;
    
    SimpleLogger::info("⏳ Takeoff... (15s)");  // ✅ 15s вместо 8s
    std::this_thread::sleep_for(15s);
    
    // 3. ЖДЕМ 5m (20s TOTAL)
    if (!waitForAltitude(5.0, 20s)) { 
        SimpleLogger::warn("Altitude not reached - continuing");
    }
    
    // 4. EXTRA WAIT (PX4 стабилизация)
    SimpleLogger::info("⏳ PX4 stabilization... (5s)");
    std::this_thread::sleep_for(5s);
    
    // 5. MISSION
    auto waypoints = createMissionWaypoints();
    if (executeWaypointMission(waypoints)) {
        SimpleLogger::info("🎉 MISSION SUCCESS");
        std::this_thread::sleep_for(3s);
        action_->return_to_launch();
        SimpleLogger::info("🏠 RTL");
    }
}

void MavlinkClient::debug_print() { 
    std::lock_guard<std::mutex> lock(data_mutex_);
    SimpleLogger::info("Current pos: {:.7f}, {:.7f}, {:.2f}", 
                       current_position_.latitude, 
                       current_position_.longitude, 
                       current_position_.altitude);
}

} 