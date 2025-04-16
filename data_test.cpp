#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavlink/common/mavlink.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip> 
#include <cmath>
#include <atomic>  // For thread-safe flag

using namespace mavsdk;

//varibles
double target_lat = -35.3629024;
double target_lon = 149.1652153;

double distance_between(double current_lat, double current_lon, double leader_lat, double leader_lon) {
    // Convert degrees to radians
    const double R = 6378137.0;  // Earth radius in meters
    double dlat = (current_lat - leader_lat) * M_PI / 180.0;
    double dlon = (current_lon - leader_lon) * M_PI / 180.0;

    // Haversine formula
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(leader_lat * M_PI / 180.0) * std::cos(current_lat * M_PI / 180.0) * 
               std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    // Distance in meters
    double distance = R * c;

    return distance;
}

bool takeoff(System& system, float takeoff_altitude_m) {
    Action action(system);
    Telemetry telemetry(system);

    // Set desired takeoff altitude
    auto altitude_result = action.set_takeoff_altitude(takeoff_altitude_m);
    if (altitude_result != Action::Result::Success) {
        std::cerr << "Failed to set takeoff altitude: " << altitude_result << std::endl;
        return false;
    }

    // Wait for system to be ready
    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for system to be ready..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Arm the drone
    auto arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << std::endl;
        return false;
    }

    std::cout << "Armed successfully." << std::endl;

    // Takeoff
    auto takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << std::endl;
        return false;
    }

    std::cout << "Takeoff initiated to " << takeoff_altitude_m << " meters..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));
    return true;
}


bool set_flight_mode(System& system, uint8_t base_mode, uint8_t custom_mode) {
    MavlinkPassthrough mavlink_passthrough(system);

    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(
        mavlink_passthrough.get_our_sysid(),
        mavlink_passthrough.get_our_compid(),
        &msg,
        system.get_system_id(),
        base_mode,
        custom_mode
    );

    return mavlink_passthrough.send_message(msg) == MavlinkPassthrough::Result::Success;
}


bool set_velocity(System& system, float vx, float vy, float vz, float yaw_rate_deg) {
    Offboard offboard(system);

    Offboard::VelocityBodyYawspeed velocity{};
    velocity.forward_m_s = vx;
    velocity.right_m_s = vy;
    velocity.down_m_s = vz;
    velocity.yawspeed_deg_s = yaw_rate_deg;

    // Start the offboard mode
    std::cout << "Sending initial setpoint..." << std::endl;
    offboard.set_velocity_body(velocity);
    auto offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << std::endl;
        return false;
    }

    std::cout << "Offboard started. Sending velocity..." << std::endl;
    offboard.set_velocity_body(velocity);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Stop offboard mode after command
    offboard.stop();

    return true;
}

int main() {

    Mavsdk::Configuration config{ComponentType::GroundStation};
    Mavsdk mavsdk(config);

    ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14550");

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << std::endl;
        return 1;
    }

    std::shared_ptr<System> system;
    while (true) {
        auto systems = mavsdk.systems();
        if (!systems.empty()) {
            system = systems.at(0);
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    Telemetry telemetry(*system);
    telemetry.subscribe_health_all_ok([](bool all_ok) {
        std::cout << "Health status: " << (all_ok ? "OK" : "NOT OK") << std::endl;
    });
    telemetry.subscribe_position([](Telemetry::Position position) {
        std::cout << std::fixed << std::setprecision(7);
        std::cout << "Latitude:  " << position.latitude_deg << std::endl;
        std::cout << "Longitude: " << position.longitude_deg << std::endl;
        std::cout << "Altitude:  " << position.relative_altitude_m << " m" << std::endl;
        std::cout << "----" << std::endl;
    });

    telemetry.subscribe_battery([](Telemetry::Battery battery) {
        std::cout << "Battery: " << battery.remaining_percent * 100.0f << "%" << std::endl;
    });

    telemetry.subscribe_flight_mode([](Telemetry::FlightMode flight_mode) {
        std::cout << "Flight mode: " << flight_mode << std::endl;
    });

    // Set mode to GUIDED
    set_flight_mode(*system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4);
    std::cout << "Flight mode set to GUIDED." << std::endl;
    // Call the takeoff function
    if (!takeoff(*system,10.0f)) {
        return 1;
    }
    std::cout << "Takeoff successful." << std::endl;

    std::atomic<bool> target_reached{false};

    telemetry.subscribe_position([&](Telemetry::Position position) {
        double current_lat = position.latitude_deg;
        double current_lon = position.longitude_deg;
        double dist = distance_between(current_lat, current_lon, target_lat, target_lon);
        std::cout << "Distance to target: " << dist << " meters." << std::endl;
        if (dist <= 2.0) {
            std::cout << "Target reached!" << std::endl;
            target_reached = true;
        }
    });
    
    while (!target_reached) {
        set_velocity(*system, 1.0f, 0.0f, 0.0f, 0.0f);  
        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Don't hammer the CPU
    }

    std::this_thread::sleep_for(std::chrono::seconds(10));
    return 0;
}

//g++ data_test.cpp -lmavsdk -lpthread -o data_test (compile command)