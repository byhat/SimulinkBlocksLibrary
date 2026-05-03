#include <iostream>
#include <thread>
#include <cstring>
#include <vector>
#include <functional>
#include <atomic>
#include <csignal>
#include <chrono>
#include <cmath>
#include <memory>
#include "AsyncExcelWriter.h"

#include "../../include/SimulinkBlocksLibrary.hpp"
#include "../../include/Flightgear/net_ctrls.hxx"
#include "../../include/Flightgear/net_fdm.hxx"

using namespace SimulinkBlock;

// Configuration parameters
constexpr double WAIT_TIME = 10.0;           // Seconds to wait for FlightGear world to load (increased for stability)
double TARGET_ALTITUDE = 200.0;             // Target altitude in meters (default, can be overridden by command-line)
constexpr double CONTROL_RATE = 30.0;        // Control loop frequency in Hz
constexpr double DT = 1.0 / CONTROL_RATE;    // Control loop period in seconds
constexpr double THROTTLE_RATE = 0.05;      // Increase throttle by 5% per second

// Takeoff parameters
constexpr double TAKEOFF_THROTTLE = 1.0;     // Full throttle for takeoff
constexpr double TAKEOFF_SPEED = 50.0;       // Takeoff speed in m/s
constexpr double CLIMB_PITCH = 15.0;         // Climb pitch angle in degrees

// Autopilot PID gains (tune these for your aircraft)
constexpr double ALTITUDE_KP = 0.5;
constexpr double ALTITUDE_KI = 0.01;
constexpr double ALTITUDE_KD = 0.2;

constexpr double PITCH_KP = 0.8;
constexpr double PITCH_KI = 0.0;
constexpr double PITCH_KD = 0.3;

constexpr double ROLL_KP = 1.0;  // Moderate roll control for zero roll
constexpr double ROLL_KI = 0.05;
constexpr double ROLL_KD = 0.1;

// Test state machine
enum class TestState {
    WAITING,        // Waiting for FlightGear to load
    ENGINE_START,   // Starting the engine
    TAKING_OFF,     // Taking off on autopilot
    CLIMBING,       // Climbing to target altitude
    TESTING,        // Executing test and recording data
    RESETTING,      // Resetting to starting position
    DONE            // All tests completed
};


std::atomic_bool shutdown_requested{false};
std::atomic_bool flightgear_connected{false};

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nReceived SIGINT (Ctrl+C). Shutting down gracefully...\n";
        shutdown_requested.store(true, std::memory_order_relaxed);
    }
}

// Value getter function type
using ValueGetter = std::function<double()>;

// Parameter structure for data logging
struct Parameter {
    std::string name;
    ValueGetter getter;
};

// Test configuration
struct TestConfig {
    std::string name;
    double duration;  // Test duration in seconds
    std::function<void(FGNetCtrls&, const FGNetFDM&, double)> control_function;
};

/**
 * @brief Reset UAV to starting position by sending saved FDM state to FlightGear
 * @param fdm_sender UDP sender for FDM data (port 5504)
 * @param saved_fdm The saved FDM state to restore
 */
void resetToSavedFDM(SendUdp<FGNetFDM>& fdm_sender, const FGNetFDM& saved_fdm) {
    std::cout << "Resetting to saved FDM state..." << std::endl;
    std::cout << "  Lat: " << L2B(saved_fdm.latitude) << ", Lon: " << L2B(saved_fdm.longitude) << std::endl;
    std::cout << "  Alt: " << L2B(saved_fdm.altitude) << " m" << std::endl;
    std::cout << "  Phi: " << L2B(saved_fdm.phi) << ", Theta: " << L2B(saved_fdm.theta) << ", Psi: " << L2B(saved_fdm.psi) << std::endl;
    
    // Send the saved FDM state to FlightGear on port 5504
    fdm_sender.send(saved_fdm);
    
    std::cout << "FDM reset command sent." << std::endl;
}

/**
 * @brief Normalize angle to [-π, π] range
 */
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}


/**
 * @brief Engine startup control function for Cessna 172
 */
void engineStartControl(FGNetCtrls& ctrls, const FGNetFDM& fdm, double time_in_state) {
    // Set up engine controls for startup
    for (int i = 0; i < 4; ++i) {
        ctrls.magnetos[i] = B2L(3);  // Magnetos ON (both)
        ctrls.master_bat[i] = B2L(1);  // Master battery ON
        ctrls.master_alt[i] = B2L(1);  // Master alternator ON
        ctrls.mixture[i] = B2L(1.0);  // Full rich mixture
        ctrls.fuel_pump_ok[i] = B2L(1);  // Fuel pump OK
        ctrls.fuel_pump_power[i] = B2L(1);  // Fuel pump power ON
        ctrls.engine_ok[i] = B2L(1);  // Engine OK
        ctrls.mag_left_ok[i] = B2L(1);  // Left magneto OK
        ctrls.mag_right_ok[i] = B2L(1);  // Right magneto OK
        ctrls.spark_plugs_ok[i] = B2L(1);  // Spark plugs OK
    }
    ctrls.num_engines = B2L(1);  // Single engine aircraft
    
    // Release brakes
    ctrls.brake_parking = B2L(0.0);
    ctrls.brake_left = B2L(0.0);
    ctrls.brake_right = B2L(0.0);
    
    // Engage starter for first 5 seconds
    if (time_in_state < 5.0) {
        for (int i = 0; i < 4; ++i) {
            ctrls.starter_power[i] = B2L(1);  // Starter ON
        }
        // Slight throttle during startup
        for (int i = 0; i < 4; ++i) {
            ctrls.throttle[i] = B2L(0.1);  // 10% throttle
        }
    } else {
        // Disengage starter after 5 seconds
        for (int i = 0; i < 4; ++i) {
            ctrls.starter_power[i] = B2L(0);  // Starter OFF
        }
        // Increase throttle to idle
        for (int i = 0; i < 4; ++i) {
            ctrls.throttle[i] = B2L(0.2);  // 20% throttle (idle)
        }
    }
    
    // Keep controls neutral
    ctrls.elevator = B2L(0.0);
    ctrls.aileron = B2L(0.0);
    ctrls.rudder = B2L(0.0);
    
    // Debug output
    static int debug_counter = 0;
    if (++debug_counter >= 30) {  // Every ~1 second at 30Hz
        debug_counter = 0;
        std::cout << "  Engine Start - Time: " << time_in_state << "s, RPM: " << L2B(fdm.rpm[0]) << std::endl;
    }
}

/**
 * @brief Simple takeoff control function with immediate max throttle and altitude autopilot
 */
void takeoffControl(FGNetCtrls& ctrls, const FGNetFDM& fdm, double time_in_state,
                    LateralControl<double>& lateral, LongitudalControl<double>& longitudal,
                    double& current_throttle) {
    // Start engine - set magnetos to ON (both)
    for (int i = 0; i < 4; ++i) {
        ctrls.magnetos[i] = B2L(3);  // Magnetos ON (both)
        ctrls.master_bat[i] = B2L(1);  // Master battery ON
        ctrls.master_alt[i] = B2L(1);  // Master alternator ON
        ctrls.starter_power[i] = B2L(1);  // Starter ON
        ctrls.engine_ok[i] = B2L(1);  // Engine OK
        ctrls.mag_left_ok[i] = B2L(1);  // Left magneto OK
        ctrls.mag_right_ok[i] = B2L(1);  // Right magneto OK
        ctrls.spark_plugs_ok[i] = B2L(1);  // Spark plugs OK
        ctrls.fuel_pump_ok[i] = B2L(1);  // Fuel pump OK
        ctrls.fuel_pump_power[i] = B2L(1);  // Fuel pump power ON
        ctrls.mixture[i] = B2L(1.0);  // Full rich mixture
    }
    
    // Set number of engines
    ctrls.num_engines = B2L(1);  // Single engine aircraft
    
    // Release parking brake
    ctrls.brake_parking = B2L(0.0);
    ctrls.brake_left = B2L(0.0);
    ctrls.brake_right = B2L(0.0);
    
    // Gradually increase throttle
    if (current_throttle < 1.0) {
        current_throttle += THROTTLE_RATE * DT;
        if (current_throttle > 1.0) current_throttle = 1.0;
    }
    for (int i = 0; i < 4; ++i) {
        ctrls.throttle[i] = B2L(current_throttle);
    }
    
    // Use lateral control for roll (keep wings level)
    lateral.step(0.0,  // Desired yaw = 0 (straight flight)
                 L2B(fdm.psi),
                 L2B(fdm.psidot),
                 normalizeAngle(L2B(fdm.phi)),
                 L2B(fdm.phidot),
                 DT);
    auto latOut = lateral.getOutput();
    ctrls.aileron = B2L(latOut.first);
    ctrls.rudder = B2L(latOut.second);
    
    // Use longitudinal control for pitch/altitude
    // Target: climb to TARGET_ALTITUDE with desired speed
    longitudal.step(TARGET_ALTITUDE,
                    60.0,  // Desired speed in m/s
                    L2B(fdm.altitude),
                    L2B(fdm.v_body_u),
                    L2B(fdm.theta),
                    L2B(fdm.thetadot),
                    DT);
    auto lonOut = longitudal.getOutput();
    ctrls.elevator = B2L(lonOut.first);
    // Throttle is already set above
    
    // Debug output every second
    static int debug_counter = 0;
    if (++debug_counter >= 30) {
        debug_counter = 0;
        std::cout << "  Alt: " << L2B(fdm.altitude) << " m, Speed: " << L2B(fdm.v_body_u) << " m/s" << std::endl;
        std::cout << "  Throttle: " << L2B(ctrls.throttle[0]) << ", Elevator: " << L2B(ctrls.elevator) << std::endl;
        std::cout << "  Engine RPM: " << L2B(fdm.rpm[0]) << std::endl;
        std::cout << "  Roll: " << L2B(fdm.phi) << " rad, Aileron: " << L2B(ctrls.aileron) << std::endl;
    }
}

/**
 * @brief Climb to target altitude with altitude autopilot
 */
void climbControl(FGNetCtrls& ctrls, const FGNetFDM& fdm,
                  LateralControl<double>& lateral, LongitudalControl<double>& longitudal) {
    double current_alt = L2B(fdm.altitude);
    
    // Maintain engine state
    for (int i = 0; i < 4; ++i) {
        ctrls.magnetos[i] = B2L(3);  // Magnetos ON
        ctrls.master_bat[i] = B2L(1);  // Master battery ON
        ctrls.master_alt[i] = B2L(1);  // Master alternator ON
        ctrls.mixture[i] = B2L(1.0);  // Full rich mixture
        
        // Reduce throttle as we approach target altitude
        // Use 80% throttle when far from target, reduce to 60% when close
        double alt_error = TARGET_ALTITUDE - current_alt;
        double throttle = 0.8;
        if (alt_error < 200.0) {
            // Within 200m of target, reduce throttle
            throttle = 0.6 + (alt_error / 200.0) * 0.2;
        }
        ctrls.throttle[i] = B2L(std::clamp(throttle, 0.0, 1.0));
    }
    ctrls.num_engines = B2L(1);
    
    // Use lateral control for roll (keep wings level)
    lateral.step(0.0,  // Desired yaw = 0
                 L2B(fdm.psi),
                 L2B(fdm.psidot),
                 normalizeAngle(L2B(fdm.phi)),
                 L2B(fdm.phidot),
                 DT);
    auto latOut = lateral.getOutput();
    ctrls.aileron = B2L(latOut.first);
    ctrls.rudder = B2L(latOut.second);
    
    // Use longitudinal control for pitch/altitude
    longitudal.step(TARGET_ALTITUDE,
                    60.0,  // Desired speed in m/s
                    L2B(fdm.altitude),
                    L2B(fdm.v_body_u),
                    L2B(fdm.theta),
                    L2B(fdm.thetadot),
                    DT);
    auto lonOut = longitudal.getOutput();
    ctrls.elevator = B2L(lonOut.first);
}

/**
 * @brief Level flight control for testing
 */
void levelFlightControl(FGNetCtrls& ctrls, const FGNetFDM& fdm,
                        LateralControl<double>& lateral, LongitudalControl<double>& longitudal) {
    // Maintain engine state
    for (int i = 0; i < 4; ++i) {
        ctrls.magnetos[i] = B2L(3);  // Magnetos ON
        ctrls.master_bat[i] = B2L(1);  // Master battery ON
        ctrls.master_alt[i] = B2L(1);  // Master alternator ON
        ctrls.mixture[i] = B2L(1.0);  // Full rich mixture
        ctrls.throttle[i] = B2L(0.7);  // Cruise throttle
    }
    ctrls.num_engines = B2L(1);
    
    // Use lateral control for roll (keep wings level)
    lateral.step(0.0,  // Desired yaw = 0
                 L2B(fdm.psi),
                 L2B(fdm.psidot),
                 normalizeAngle(L2B(fdm.phi)),
                 L2B(fdm.phidot),
                 DT);
    auto latOut = lateral.getOutput();
    ctrls.aileron = B2L(latOut.first);
    ctrls.rudder = B2L(latOut.second);
    
    // Use longitudinal control for pitch/altitude
    longitudal.step(TARGET_ALTITUDE,
                    60.0,  // Desired speed in m/s
                    L2B(fdm.altitude),
                    L2B(fdm.v_body_u),
                    L2B(fdm.theta),
                    L2B(fdm.thetadot),
                    DT);
    auto lonOut = longitudal.getOutput();
    ctrls.elevator = B2L(lonOut.first);
}

/**
 * @brief Apply lateral and longitudinal control to maintain stable flight
 */
void applyStableFlightControl(FGNetCtrls& ctrls, const FGNetFDM& fdm,
                               LateralControl<double>& lateral, LongitudalControl<double>& longitudal) {
    // Maintain engine state
    for (int i = 0; i < 4; ++i) {
        ctrls.magnetos[i] = B2L(3);  // Magnetos ON
        ctrls.master_bat[i] = B2L(1);  // Master battery ON
        ctrls.master_alt[i] = B2L(1);  // Master alternator ON
        ctrls.mixture[i] = B2L(1.0);  // Full rich mixture
    }
    ctrls.num_engines = B2L(1);
    
    // Use lateral control for roll (keep wings level)
    lateral.step(0.0,  // Desired yaw = 0
                 L2B(fdm.psi),
                 L2B(fdm.psidot),
                 normalizeAngle(L2B(fdm.phi)),
                 L2B(fdm.phidot),
                 DT);
    auto latOut = lateral.getOutput();
    ctrls.aileron = B2L(latOut.first);
    ctrls.rudder = B2L(latOut.second);
    
    // Use longitudinal control for pitch/altitude
    longitudal.step(TARGET_ALTITUDE,
                    60.0,  // Desired speed in m/s
                    L2B(fdm.altitude),
                    L2B(fdm.v_body_u),
                    L2B(fdm.theta),
                    L2B(fdm.thetadot),
                    DT);
    auto lonOut = longitudal.getOutput();
    ctrls.elevator = B2L(lonOut.first);
}

/**
 * @brief Execute a test with custom control function
 */
void executeTest(FGNetCtrls& ctrls, const FGNetFDM& fdm, double time_in_state,
                 const TestConfig& test_config, AsyncExcelWriter& writer,
                 const std::vector<Parameter>& parameters) {
    // Apply test-specific control
    test_config.control_function(ctrls, fdm, time_in_state);
    
    // Log data
    std::vector<double> row_data;
    row_data.reserve(parameters.size());
    for (const auto& param : parameters) {
        row_data.push_back(param.getter());
    }
    writer.addToQueue(std::move(row_data));
}

int main() {
    std::signal(SIGINT, signal_handler);

    // Hardcoded altitudes to test (10 altitudes from 200m to 2000m)
    std::vector<double> altitudes = {200, 400, 600, 800, 1000, 1200, 1400, 1600, 1800, 2000};
    
    std::cout << "Running tests at " << altitudes.size() << " altitudes:" << std::endl;
    for (double alt : altitudes) {
        std::cout << "  " << alt << " m" << std::endl;
    }

    // Initialize FlightGear receivers
    FlightGearReceiver<FGNetCtrls> ctrls_receiver(5501);
    FlightGearReceiver<FGNetFDM> fdm_receiver(5503);

    // Initialize FlightGear control sender
    SendUdp<FGNetCtrls> ctrl_sender("127.0.0.1", 5502);
    
    // Initialize FlightGear FDM sender for reset (port 5504)
    SendUdp<FGNetFDM> fdm_sender("127.0.0.1", 5504);

    FGNetCtrls ctrls;
    FGNetFDM fdm;
    std::memset(&ctrls, 0, sizeof(FGNetCtrls));
    std::memset(&fdm, 0, sizeof(FGNetFDM));

    // Initialize LateralControl (roll/yaw) - using proven coefficients from FlightGearExample
    LateralControl<double> lateral;
    lateral.setAileronControllCoeffs(-0.1, -1.0, -0.1, -0.5, -0.01);
    lateral.setRudderControllCoeffs(-0.1, -0.1);
    lateral.setRollSaturationLimits(-0.3, 0.3);
    lateral.setRudderSaturationLimits(-0.3, 0.3);
    lateral.setAileronsSaturationLimits(-1.0, 1.0);
    lateral.enableYawAngleControl(false);  // Disable yaw control for straight flight
    lateral.enableRollAngleControl(true);   // Enable roll control to keep wings level
    lateral.enableRudderControl(false);

    // Initialize LongitudalControl (pitch/altitude) - using proven coefficients from FlightGearExample
    LongitudalControl<double> longitudal;
    longitudal.setAltitudePidCoeffs(1.5, 0.5, 0.1);
    longitudal.setAngularVelocityPidCoeffs(-1.0, -0.01, -0.01);
    longitudal.setPitchAnglePidCoeffs(1.0, 0.0, 0.0);
    longitudal.setVelocityPidCoeffs(1.0, 0.01, 0);
    longitudal.setSaturationLimits(-0.3, 0.3);
    longitudal.enableAltitudeControl(true);
    longitudal.enableAngularVelocityControl(true);
    longitudal.enablePitchAngleControl(true);

    // Throttle state for gradual increase
    double current_throttle = 0.0;

    // Define data logging parameters
    std::vector<Parameter> parameters = {
        {"time", [&]() { return 0.0; }},  // Will be set in loop
        {"altitude", [&]() { return L2B(fdm.altitude); }},
        {"latitude", [&]() { return L2B(fdm.latitude); }},
        {"longitude", [&]() { return L2B(fdm.longitude); }},
        {"phi", [&]() { return L2B(fdm.phi); }},
        {"theta", [&]() { return L2B(fdm.theta); }},
        {"psi", [&]() { return L2B(fdm.psi); }},
        {"phidot", [&]() { return L2B(fdm.phidot); }},
        {"thetadot", [&]() { return L2B(fdm.thetadot); }},
        {"psidot", [&]() { return L2B(fdm.psidot); }},
        {"v_body_u", [&]() { return L2B(fdm.v_body_u); }},
        {"v_body_v", [&]() { return L2B(fdm.v_body_v); }},
        {"v_body_w", [&]() { return L2B(fdm.v_body_w); }},
        {"vcas", [&]() { return L2B(fdm.vcas); }},
        {"A_X_pilot", [&]() { return L2B(fdm.A_X_pilot); }},
        {"A_Y_pilot", [&]() { return L2B(fdm.A_Y_pilot); }},
        {"A_Z_pilot", [&]() { return L2B(fdm.A_Z_pilot); }},
        {"alpha", [&]() { return L2B(fdm.alpha); }},
        {"beta", [&]() { return L2B(fdm.beta); }},
        {"elevator", [&]() { return L2B(ctrls.elevator); }},
        {"throttle_0", [&]() { return L2B(ctrls.throttle[0]); }},
        {"aileron", [&]() { return L2B(ctrls.aileron); }},
        {"rudder", [&]() { return L2B(ctrls.rudder); }},
        {"flaps", [&]() { return L2B(ctrls.flaps); }},
        {"state", [&]() { return 0; }}  // Will be set in loop
    };

    // Create headers
    std::vector<std::string> headers;
    headers.reserve(parameters.size());
    for (const auto& p : parameters) {
        headers.push_back(p.name);
    }

    // Define test configurations - 11 flight tests for neural network training data
    // Note: Tests capture lateral and longitudal controllers by reference
    auto& lateral_ref = lateral;
    auto& longitudal_ref = longitudal;
    
    // Cruise Flight Tests (3 tests)
    std::vector<TestConfig> tests = {
        // Test 1: Разгон в крейсерском полёте (Acceleration in cruise flight)
        // Increase throttle to max, record response
        {
            "Разгон в крейсерском полёте (Acceleration in cruise flight)",
            30.0,  // 30 seconds
            [&](FGNetCtrls& c, const FGNetFDM& f, double t) {
                // Apply stable flight control
                applyStableFlightControl(c, f, lateral_ref, longitudal_ref);
                
                // Acceleration: Increase throttle to max after 5 seconds
                double throttle = (t < 5.0) ? 0.7 : 1.0;
                for (int i = 0; i < 4; ++i) {
                    c.throttle[i] = B2L(throttle);
                }
            }
        },
        
        // Test 2: Выбег в крейсерском полёте (Deceleration in cruise flight)
        // Decrease throttle to idle, record response
        {
            "Выбег в крейсерском полёте (Deceleration in cruise flight)",
            30.0,  // 30 seconds
            [&](FGNetCtrls& c, const FGNetFDM& f, double t) {
                // Apply stable flight control
                applyStableFlightControl(c, f, lateral_ref, longitudal_ref);
                
                // Deceleration: Decrease throttle to idle after 5 seconds
                double throttle = (t < 5.0) ? 0.7 : 0.0;
                for (int i = 0; i < 4; ++i) {
                    c.throttle[i] = B2L(throttle);
                }
            }
        },
        
        // Test 3: Снижение на малом газу (Descent at low throttle)
        // Low throttle descent, record response
        {
            "Снижение на малом газу (Descent at low throttle)",
            30.0,  // 30 seconds
            [&](FGNetCtrls& c, const FGNetFDM& f, double t) {
                // Apply stable flight control
                applyStableFlightControl(c, f, lateral_ref, longitudal_ref);
                
                // Low throttle for descent
                for (int i = 0; i < 4; ++i) {
                    c.throttle[i] = B2L(0.2);  // 20% throttle
                }
            }
        },
        
        // Control Surface Tests (6 tests)
        
        // Test 4: Управление по тангажу (Pitch control)
        // Elevator step inputs
        {
            "Управление по тангажу (Pitch control)",
            25.0,  // 25 seconds
            [&](FGNetCtrls& c, const FGNetFDM& f, double t) {
                // Apply stable flight control
                applyStableFlightControl(c, f, lateral_ref, longitudal_ref);
                
                // Maintain cruise throttle
                for (int i = 0; i < 4; ++i) {
                    c.throttle[i] = B2L(0.7);
                }
                
                // Elevator step inputs: positive (push down), then negative (pull up)
                if (t > 5.0 && t < 10.0) {
                    c.elevator = B2L(0.15);  // Push down - nose down (reduced from 0.4)
                } else if (t >= 10.0 && t < 15.0) {
                    c.elevator = B2L(-0.15);  // Pull up - nose up (reduced from -0.4)
                }
            }
        },
        
        // Test 5: Управление по крену (Roll control)
        // Aileron step inputs
        {
            "Управление по крену (Roll control)",
            25.0,  // 25 seconds
            [&](FGNetCtrls& c, const FGNetFDM& f, double t) {
                // Apply stable flight control
                applyStableFlightControl(c, f, lateral_ref, longitudal_ref);
                
                // Maintain cruise throttle
                for (int i = 0; i < 4; ++i) {
                    c.throttle[i] = B2L(0.7);
                }
                
                // Aileron step inputs: roll right, then roll left
                if (t > 5.0 && t < 10.0) {
                    c.aileron = B2L(0.2);  // Roll right (reduced from 0.5)
                } else if (t >= 10.0 && t < 15.0) {
                    c.aileron = B2L(-0.2);  // Roll left (reduced from -0.5)
                }
            }
        },
        
        // Test 6: Управление по рысканию (Yaw control)
        // Rudder step inputs
        {
            "Управление по рысканию (Yaw control)",
            25.0,  // 25 seconds
            [&](FGNetCtrls& c, const FGNetFDM& f, double t) {
                // Apply stable flight control
                applyStableFlightControl(c, f, lateral_ref, longitudal_ref);
                
                // Maintain cruise throttle
                for (int i = 0; i < 4; ++i) {
                    c.throttle[i] = B2L(0.7);
                }
                
                // Rudder step inputs: yaw right, then yaw left
                if (t > 5.0 && t < 10.0) {
                    c.rudder = B2L(0.2);  // Yaw right (reduced from 0.5)
                } else if (t >= 10.0 && t < 15.0) {
                    c.rudder = B2L(-0.2);  // Yaw left (reduced from -0.5)
                }
            }
        },
        
        // Test 7: Малые отклонения органов управления — Тангаж (Small control deviations - Pitch)
        // Small elevator inputs
        {
            "Малые отклонения органов управления — Тангаж (Small control deviations - Pitch)",
            25.0,  // 25 seconds
            [&](FGNetCtrls& c, const FGNetFDM& f, double t) {
                // Apply stable flight control
                applyStableFlightControl(c, f, lateral_ref, longitudal_ref);
                
                // Maintain cruise throttle
                for (int i = 0; i < 4; ++i) {
                    c.throttle[i] = B2L(0.7);
                }
                
                // Small elevator inputs: small positive, then small negative
                if (t > 5.0 && t < 10.0) {
                    c.elevator = B2L(0.15);  // Small push down
                } else if (t >= 10.0 && t < 15.0) {
                    c.elevator = B2L(-0.15);  // Small pull up
                }
            }
        },
        
        // Test 8: Малые отклонения органов управления — Крен (Small control deviations - Roll)
        // Small aileron inputs
        {
            "Малые отклонения органов управления — Крен (Small control deviations - Roll)",
            25.0,  // 25 seconds
            [&](FGNetCtrls& c, const FGNetFDM& f, double t) {
                // Apply stable flight control
                applyStableFlightControl(c, f, lateral_ref, longitudal_ref);
                
                // Maintain cruise throttle
                for (int i = 0; i < 4; ++i) {
                    c.throttle[i] = B2L(0.7);
                }
                
                // Small aileron inputs: small right, then small left
                if (t > 5.0 && t < 10.0) {
                    c.aileron = B2L(0.2);  // Small roll right
                } else if (t >= 10.0 && t < 15.0) {
                    c.aileron = B2L(-0.2);  // Small roll left
                }
            }
        },
        
        // Test 9: Малые отклонения органов управления — Рыскание (Small control deviations - Yaw)
        // Small rudder inputs
        {
            "Малые отклонения органов управления — Рыскание (Small control deviations - Yaw)",
            25.0,  // 25 seconds
            [&](FGNetCtrls& c, const FGNetFDM& f, double t) {
                // Apply stable flight control
                applyStableFlightControl(c, f, lateral_ref, longitudal_ref);
                
                // Maintain cruise throttle
                for (int i = 0; i < 4; ++i) {
                    c.throttle[i] = B2L(0.7);
                }
                
                // Small rudder inputs: small right, then small left
                if (t > 5.0 && t < 10.0) {
                    c.rudder = B2L(0.2);  // Small yaw right
                } else if (t >= 10.0 && t < 15.0) {
                    c.rudder = B2L(-0.2);  // Small yaw left
                }
            }
        },
        
        // Longitudinal Motion Tests (2 tests)
        
        // Test 10: Балансировка в продольном канале (Longitudinal channel trimming)
        // Trim elevator for level flight
        {
            "Балансировка в продольном канале (Longitudinal channel trimming)",
            30.0,  // 30 seconds
            [&](FGNetCtrls& c, const FGNetFDM& f, double t) {
                // Apply stable flight control
                applyStableFlightControl(c, f, lateral_ref, longitudal_ref);
                
                // Maintain cruise throttle
                for (int i = 0; i < 4; ++i) {
                    c.throttle[i] = B2L(0.7);
                }
            }
        },
        
        // Test 11: Динамика короткопериодического движения (Short-period dynamics)
        // Pitch doublet or impulse
        {
            "Динамика короткопериодического движения (Short-period dynamics)",
            25.0,  // 25 seconds
            [&](FGNetCtrls& c, const FGNetFDM& f, double t) {
                // Apply stable flight control using proven autopilot
                applyStableFlightControl(c, f, lateral_ref, longitudal_ref);
                
                // Pitch doublet: quick up-down impulse to excite short-period mode
                if (t > 5.0 && t < 6.0) {
                    c.elevator = B2L(-0.2);  // Quick pull up (reduced from -0.5)
                } else if (t >= 6.0 && t < 7.0) {
                    c.elevator = B2L(0.2);  // Quick push down (reduced from 0.5)
                }
                // Return to normal control after doublet
            }
        }
    };

    // State machine variables
    TestState state = TestState::WAITING;
    double time_in_state = 0.0;
    double total_time = 0.0;
    int current_test = 0;
    FGNetFDM saved_fdm{};  // Saved FDM state for reset
    bool saved_fdm_initialized = false;
    bool first_test_at_altitude = true;  // Track if this is the first test at current altitude

    std::cout << "=== FlightGear Tests for Neural Network Training Data ===" << std::endl;
    std::cout << "Waiting " << WAIT_TIME << " seconds for FlightGear world to load..." << std::endl;

    try {
        // Create Excel writer for each test
        std::unique_ptr<AsyncExcelWriter> writer;
        
        // Function to create a new writer for a test
        auto createWriter = [&](int test_index, double altitude) {
            std::string filename = "test_" + std::to_string(test_index + 1) +
                                  "_" + tests[test_index].name +
                                  "_alt" + std::to_string(static_cast<int>(altitude)) + "m.xlsx";
            writer = std::make_unique<AsyncExcelWriter>(filename, headers);
            std::cout << "Logging to: " << filename << std::endl;
        };
        
        // Function to wait for writer to finish and prepare for next test
        auto finishCurrentTest = [&]() {
            std::cout << "Test " << (current_test + 1) << " completed." << std::endl;
            std::cout << "Waiting for async writer to finish writing data..." << std::endl;
            
            // Wait for the writer to finish all queued writes
            // The destructor will flush remaining data, but we need to give it time
            std::this_thread::sleep_for(std::chrono::seconds(25));
            std::cout << "Excel file should now be complete." << std::endl;
            
            // Move to next test
            current_test++;
            
            if (current_test >= static_cast<int>(tests.size())) {
                std::cout << "All tests at current altitude completed!" << std::endl;
                state = TestState::DONE;
            } else {
                // For subsequent tests at the same altitude, reset to saved FDM state
                // but with altitude set to 90% of target altitude
                first_test_at_altitude = false;
                std::cout << "Starting next test: " << tests[current_test].name << std::endl;
                
                // Create new writer for next test
                writer.reset();  // This will safely destroy the old writer
                writer = nullptr;  // Ensure it's null so createWriter will create a new one
                
                // Reset to saved FDM state with modified altitude
                if (saved_fdm_initialized) {
                    FGNetFDM reset_fdm = saved_fdm;
                    // Set altitude to 90% of target altitude
                    reset_fdm.altitude = B2L(0.9 * TARGET_ALTITUDE);
                    resetToSavedFDM(fdm_sender, reset_fdm);
                } else {
                    std::cerr << "Warning: Saved FDM not initialized, skipping reset." << std::endl;
                }
                
                // Go to TESTING state
                state = TestState::TESTING;
                time_in_state = 0.0;
            }
        };
        
        // Wait for FlightGear connection with timeout
        {
            std::cout << "Waiting for FlightGear connection..." << std::endl;
            auto start_wait = std::chrono::steady_clock::now();
            bool connected = false;
            
            while (!shutdown_requested && !connected) {
                try {
                    // Try to get data - this will block until data is available
                    const auto& test_fdm = fdm_receiver.getOutput();
                    flightgear_connected.store(true, std::memory_order_release);
                    connected = true;
                    std::cout << "FlightGear connected successfully!" << std::endl;
                } catch (...) {
                    // If getOutput throws, connection not ready yet
                }
                
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - start_wait).count();
                
                if (elapsed > 30) {
                    std::cerr << "Warning: Could not connect to FlightGear after 30 seconds." << std::endl;
                    std::cerr << "Please ensure FlightGear is running with the correct UDP settings." << std::endl;
                    std::cerr << "Continuing anyway..." << std::endl;
                    break;
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        
        // Altitude loop - iterate through all altitudes
        for (size_t alt_idx = 0; alt_idx < altitudes.size(); ++alt_idx) {
            TARGET_ALTITUDE = altitudes[alt_idx];
            
            std::cout << "\n========================================" << std::endl;
            std::cout << "Starting tests at altitude: " << TARGET_ALTITUDE << " m" << std::endl;
            std::cout << "Altitude " << (alt_idx + 1) << " of " << altitudes.size() << std::endl;
            std::cout << "========================================\n" << std::endl;
            
            // Reset test index for this altitude
            current_test = 0;
            first_test_at_altitude = true;  // This is the first test at this altitude
            
            // If this is not the first altitude, aircraft is already in the air
            // Skip WAITING, ENGINE_START, TAKING_OFF states and go directly to CLIMBING/TESTING
            if (alt_idx > 0) {
                state = TestState::CLIMBING;
                std::cout << "Aircraft already in air. Climbing to new target altitude..." << std::endl;
            } else {
                state = TestState::WAITING;
                std::cout << "Starting from ground..." << std::endl;
            }
            time_in_state = 0.0;
            total_time = 0.0;
            
            // Main control loop for current altitude
            while (!shutdown_requested && state != TestState::DONE) {
            // Get latest data from FlightGear
            try {
                // Copy controls from FlightGear to preserve engine state and other settings
                // Then we'll modify the control surfaces as needed
                std::memcpy(&ctrls, &ctrls_receiver.getOutput(), sizeof(FGNetCtrls));
                std::memcpy(&fdm, &fdm_receiver.getOutput(), sizeof(FGNetFDM));
                
                // Ensure the version is set correctly for FlightGear to accept controls
                ctrls.version = B2L(27);  // FG_NET_CTRLS_VERSION
                
                // Debug: Print altitude every second
                static int debug_counter = 0;
                debug_counter++;
                if (debug_counter >= 30) {  // Every 1 second (30 * DT)
                    double alt = L2B(fdm.altitude);
                    double speed = L2B(fdm.v_body_u);
                    std::cout << "DEBUG: Altitude=" << alt << " m, Speed=" << speed << " m/s" << std::endl;
                    debug_counter = 0;
                }
            } catch (const std::exception& e) {
                std::cerr << "Error receiving data from FlightGear: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            // Update time
            time_in_state += DT;
            total_time += DT;

            // Update time parameter for logging
            parameters[0].getter = [&]() { return total_time; };

            // State machine
            switch (state) {
                case TestState::WAITING:
                    parameters[22].getter = [&]() { return 0.0; };  // WAITING = 0
                    if (time_in_state >= WAIT_TIME) {
                        std::cout << "World loaded. Starting engine..." << std::endl;
                        state = TestState::ENGINE_START;
                        time_in_state = 0.0;
                    }
                    break;

                case TestState::ENGINE_START: {
                    parameters[22].getter = [&]() { return 1.0; };  // ENGINE_START = 1
                    engineStartControl(ctrls, fdm, time_in_state);
                    ctrl_sender.send(ctrls);

                    // Check if engine is running (RPM > 500)
                    if (L2B(fdm.rpm[0]) > 500.0 && time_in_state > 5.0) {
                        std::cout << "Engine started! RPM: " << L2B(fdm.rpm[0]) << std::endl;
                        std::cout << "Starting takeoff..." << std::endl;
                        std::cout << "Releasing all brakes..." << std::endl;
                        // Explicitly release all brakes before transitioning to TAKING_OFF
                        ctrls.brake_parking = 0.0;
                        ctrls.brake_left = 0.0;
                        ctrls.brake_right = 0.0;
                        ctrl_sender.send(ctrls);
                        state = TestState::TAKING_OFF;
                        time_in_state = 0.0;
                    } else if (time_in_state > 15.0) {
                        // Timeout - engine didn't start, proceed anyway
                        std::cout << "Engine start timeout. Proceeding with takeoff..." << std::endl;
                        std::cout << "Releasing all brakes..." << std::endl;
                        // Explicitly release all brakes before transitioning to TAKING_OFF
                        ctrls.brake_parking = 0.0;
                        ctrls.brake_left = 0.0;
                        ctrls.brake_right = 0.0;
                        ctrl_sender.send(ctrls);
                        state = TestState::TAKING_OFF;
                        time_in_state = 0.0;
                    }
                    break;
                }

                case TestState::TAKING_OFF: {
                    parameters[22].getter = [&]() { return 2.0; };  // TAKING_OFF = 2
                    takeoffControl(ctrls, fdm, time_in_state, lateral, longitudal, current_throttle);
                    ctrl_sender.send(ctrls);

                    double current_alt = L2B(fdm.altitude);
                    if (current_alt > 50.0 && time_in_state > 10.0) {
                        std::cout << "Airborne. Climbing to " << TARGET_ALTITUDE << " m..." << std::endl;
                        state = TestState::CLIMBING;
                        time_in_state = 0.0;
                    }
                    break;
                }

                case TestState::CLIMBING: {
                    parameters[22].getter = [&]() { return 3.0; };  // CLIMBING = 3
                    climbControl(ctrls, fdm, lateral, longitudal);
                    ctrl_sender.send(ctrls);

                    double current_alt = L2B(fdm.altitude);
                    double alt_error = std::abs(current_alt - TARGET_ALTITUDE);
                    
                    // Debug output every 1 second
                    if (static_cast<int>(time_in_state) % 1 == 0 && time_in_state > 0) {
                        std::cout << "  Alt: " << current_alt << " m, Error: " << alt_error
                                  << " m, Time in state: " << time_in_state << " s" << std::endl;
                    }
                    
                    // Transition to TESTING when reaching 90% of target altitude
                    // This ensures we record data from just before reaching the target
                    if (current_alt >= 0.9 * TARGET_ALTITUDE && time_in_state > 3.0) {
                        std::cout << "Reached 90% of target altitude (" << current_alt << " m). Starting test "
                                  << (current_test + 1) << "/" << tests.size()
                                  << ": " << tests[current_test].name << std::endl;
                        
                        // Save FDM state for reset between tests at this altitude (only for first test)
                        if (first_test_at_altitude) {
                            std::cout << "Storing FDM state for reset between tests..." << std::endl;
                            std::memcpy(&saved_fdm, &fdm, sizeof(FGNetFDM));
                            saved_fdm_initialized = true;
                            std::cout << "FDM state stored:" << std::endl;
                            std::cout << "  Lat: " << L2B(saved_fdm.latitude) << ", Lon: " << L2B(saved_fdm.longitude) << std::endl;
                            std::cout << "  Alt: " << L2B(saved_fdm.altitude) << " m" << std::endl;
                            std::cout << "  Phi: " << L2B(saved_fdm.phi) << ", Theta: " << L2B(saved_fdm.theta) << ", Psi: " << L2B(saved_fdm.psi) << std::endl;
                            std::cout << "  Speed: " << L2B(fdm.v_body_u) << " m/s" << std::endl;
                        }
                        
                        state = TestState::TESTING;
                        time_in_state = 0.0;
                    }
                    break;
                }

                case TestState::TESTING: {
                    parameters[22].getter = [&]() { return 4.0; };  // TESTING = 4
                    
                    // Create writer for this test if not already created
                    if (!writer) {
                        createWriter(current_test, TARGET_ALTITUDE);
                    }
                    
                    // Check for crash - if altitude drops below 25m, end the program
                    double current_alt = L2B(fdm.altitude);
                    if (current_alt < 25.0) {
                        std::cerr << "CRASH DETECTED! Altitude dropped to " << current_alt << " m during test "
                                  << (current_test + 1) << ": " << tests[current_test].name << std::endl;
                        std::cerr << "Test failed. Ending program." << std::endl;
                        state = TestState::DONE;
                        break;
                    }
                    
                    executeTest(ctrls, fdm, time_in_state, tests[current_test],
                               *writer, parameters);
                    ctrl_sender.send(ctrls);

                    if (time_in_state >= tests[current_test].duration) {
                        finishCurrentTest();
                    }
                    break;
                }

                case TestState::RESETTING:
                    // This state is no longer used for resetting between tests at the same altitude
                    // It's only used when changing altitudes (handled in the altitude loop)
                    parameters[22].getter = [&]() { return 5.0; };  // RESETTING = 5
                    if (saved_fdm_initialized) {
                        resetToSavedFDM(fdm_sender, saved_fdm);
                    } else {
                        std::cerr << "Warning: Saved FDM not initialized, skipping reset." << std::endl;
                    }
                    
                    // Wait a bit for reset to take effect
                    if (time_in_state > 5.0) {
                        std::cout << "Reset complete. Starting next altitude..." << std::endl;
                        state = TestState::DONE;  // Will exit the altitude loop
                        time_in_state = 0.0;
                    }
                    break;

                case TestState::DONE:
                    parameters[22].getter = [&]() { return 6.0; };  // DONE = 6
                    break;
            }

            // Sleep for control rate
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(DT * 1000)));
            }
            
            // Wait for any remaining async writer operations to complete before moving to next altitude
            std::cout << "All tests at " << TARGET_ALTITUDE << " m completed. Waiting for final file operations..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(25));
            
            // If this is not the last altitude, reset aircraft for next altitude
            if (alt_idx < altitudes.size() - 1) {
                std::cout << "\nPreparing for next altitude..." << std::endl;
                if (saved_fdm_initialized) {
                    resetToSavedFDM(fdm_sender, saved_fdm);
                }
                std::this_thread::sleep_for(std::chrono::seconds(5));
                current_throttle = 0.0;
                writer.reset();
                writer = nullptr;
            }
        }

        std::cout << "Finalizing data files... Please wait." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Application exited cleanly." << std::endl;
    return 0;
}
