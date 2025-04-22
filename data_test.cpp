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
#include <vector>

using namespace mavsdk;

//varibles
double target_lat = -35.3629024;
double target_lon = 149.1652153;
double distance = 10.0; // Distance in meters
double angle = 90.0; // Angle in degrees
double heading = 0.0; // Heading in degrees
double follower_heading = 1.0; // Follower heading in degrees
int counter = 0;
float target_alt = 10.0; // Target altitude in meters

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

bool goto_waypoint(System& system, float target_lat, float target_lon, float target_alt)
{
    MavlinkPassthrough mavlink_passthrough{system};
    Telemetry telemetry{system};

    mavlink_message_t msg;
    mavlink_msg_set_position_target_global_int_pack(
        mavlink_passthrough.get_our_sysid(),
        mavlink_passthrough.get_our_compid(),
        &msg,
        static_cast<uint32_t>(0), // time_boot_ms (ignored)
        1, // target system
        1, // target component
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // use relative altitude
        0b0000111111111000, // type_mask: ignore everything except position
        static_cast<int32_t>(target_lat * 1e7), // latitude in int degrees
        static_cast<int32_t>(target_lon * 1e7), // longitude in int degrees
        target_alt, // relative altitude in meters
        0, 0, 0, // vx, vy, vz (ignored)
        0, 0, 0, // acceleration (ignored)
        0, 0     // yaw, yaw_rate (ignored)
    );

    auto result = mavlink_passthrough.send_message(msg);
    if (result != MavlinkPassthrough::Result::Success) {
        std::cerr << "Failed to send waypoint: " << static_cast<int>(result) << std::endl;
        return false;
    }
    
    std::cout << "Goto command sent to reach target waypoint." << std::endl;

    bool target_reached = false;

    while (!target_reached) {
        Telemetry::Position position = telemetry.position();

        double current_lat = position.latitude_deg;
        double current_lon = position.longitude_deg;

        double dist = distance_between(current_lat, current_lon, target_lat, target_lon);
        std::cout << "Distance to target: " << dist << " meters." << std::endl;

        if (dist <= 2.0) {
            std::cout << "Target reached!" << std::endl;
            target_reached = true;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return true;

}

std::pair<float, float> relative_pos(double lat, double lon, double distance, double heading, double follower_heading) {
    const double EARTH_RADIUS = 6378137.0; // Earth's radius in meters

    double heading_rad = heading * M_PI / 180.0;
    double new_heading_rad = heading_rad + (follower_heading * M_PI / 180.0);

    // Convert degrees to radians
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    // Calculate the difference in latitude and longitude
    double delta_lat = distance * std::cos(new_heading_rad) / EARTH_RADIUS;
    double delta_lon = distance * std::sin(new_heading_rad) / (EARTH_RADIUS * std::cos(lat_rad));

    double new_lat_rad = lat_rad + delta_lat;
    double new_lon_rad = lon_rad + delta_lon;

    // Convert back to degrees
    float new_lat = static_cast<float>(new_lat_rad * 180.0 / M_PI);
    float new_lon = static_cast<float>(new_lon_rad * 180.0 / M_PI);

    return std::make_pair(new_lat, new_lon);
}

std::vector<double> get_global_position(System& system) {
    Telemetry telemetry(system);
    Telemetry::Position position = telemetry.position();

    std::vector<double> global_position;
    global_position.push_back(position.latitude_deg);
    global_position.push_back(position.longitude_deg);
    global_position.push_back(position.relative_altitude_m);

    return global_position;
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

//main function

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

    // Telemetry telemetry(*system);
    // telemetry.subscribe_health_all_ok([](bool all_ok) {
    //     std::cout << "Health status: " << (all_ok ? "OK" : "NOT OK") << std::endl;
    // });

    while (true) {  

        std::this_thread::sleep_for(std::chrono::seconds(10));
        std::vector<double> position = get_global_position(*system);
        std::cout << "Current position: " << std::fixed << std::setprecision(7) 
                << "Latitude: " << position[0] << ", Longitude: " << position[1] 
                << ", Altitude: " << position[2] << " meters." << std::endl;

        // telemetry.subscribe_battery([](Telemetry::Battery battery) {
        //     std::cout << "Battery: " << static_cast<int32_t> (battery.remaining_percent) <<" %" << std::endl;
        // });

        // telemetry.subscribe_flight_mode([](Telemetry::FlightMode flight_mode) {
        //     std::cout << "Flight mode: " << flight_mode << std::endl;
        // });
    
        // Set mode to GUIDED
        // Call the takeoff function

        std::atomic<bool> target_reached{false};
        counter++;
        if (counter == 1) {
            // Set the flight mode to GUIDED
            set_flight_mode(*system, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4);
            std::cout << "Flight mode set to GUIDED." << std::endl;

            //arm and takeoff
            if (!takeoff(*system,10.0f)) {
                return 1;
            }
            std::cout << "Takeoff successful." << std::endl;

            //start mission
            goto_waypoint(*system, target_lat, target_lon, target_alt);

        }
    }
    
    // while (!target_reached) {
    //     set_velocity(*system, 1.0f, 0.0f, 0.0f, 0.0f);  
    //     std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Don't hammer the CPU
    // }

    return 0;
}

//g++ data_test.cpp -lmavsdk -lpthread -o data_test (compile command)