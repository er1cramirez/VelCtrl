#include "RobustVelCtrl.h"
#include <flair/core/Vector3Df.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

// Simple test routine
void test_controller(RobustVelCtrl& controller, const std::string& output_file) {
    std::ofstream outfile(output_file);
    
    if (!outfile.is_open()) {
        std::cerr << "Failed to open output file: " << output_file << std::endl;
        return;
    }
    
    // Write header
    outfile << "time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,u_x,u_y,u_z,udot_x,udot_y,udot_z,posd_x,posd_y,posd_z,veld_x,veld_y,veld_z\n";
    
    // Initial conditions
    Vector3Df position(1.0, 1.0, 0.0);
    Vector3Df velocity(0.0, 0.0, 0.0);
    Vector3Df acceleration(0.0, 0.0, 0.0);
    
    // Desired states
    Vector3Df position_d(0.0, 0.0, 0.0);
    Vector3Df velocity_d(0.0, 0.0, 0.0);
    Vector3Df acceleration_d(0.0, 0.0, 0.0);
    
    // Output states
    Vector3Df u, u_dot;
    
    // Simple integration loop for testing
    float dt = 0.01f;
    float sim_time = 5.0f;  // 5 seconds simulation
    int steps = static_cast<int>(sim_time / dt);
    
    // Reset controller
    controller.reset();
    
    for (int i = 0; i < steps; i++) {
        float t = i * dt;
        
        // Add some periodic disturbance at t = 2 seconds
        Vector3Df disturbance(0.0, 0.0, 0.0);
        if (t >= 2.0) {
            disturbance.x = 0.2 * std::sin(2.0 * M_PI * t);
        }
        
        // Calculate control
        controller.calculate_hlc(
            position, velocity, acceleration,
            position_d, velocity_d, acceleration_d,
            u, u_dot);
        
        // Log data
        outfile << t << ","
                << position.x << "," << position.y << "," << position.z << ","
                << velocity.x << "," << velocity.y << "," << velocity.z << ","
                << u.x << "," << u.y << "," << u.z << ","
                << u_dot.x << "," << u_dot.y << "," << u_dot.z << ","
                << position_d.x << "," << position_d.y << "," << position_d.z << ","
                << velocity_d.x << "," << velocity_d.y << "," << velocity_d.z << "\n";
        
        // Simple integration for position and velocity
        velocity = velocity + dt * (u + disturbance);
        position = position + dt * velocity;
        
        // Simple numerical differentiation for acceleration
        acceleration = (u + disturbance);
    }
    
    outfile.close();
    std::cout << "Test completed. Results saved to " << output_file << std::endl;
}

int main() {
    // Create controller instance
    RobustVelCtrl controller;
    
    // Test P controller
    std::cout << "Testing P controller..." << std::endl;
    controller.init(ControllerType::P_CONTROL);
    controller.setParam("kv", Vector3Df(3.0, 3.0, 3.0));
    controller.setParam("mg", 9.81f);
    test_controller(controller, "p_control_results.csv");
    
    // Test First-Order SMC
    std::cout << "Testing First-Order SMC..." << std::endl;
    controller.init(ControllerType::FIRST_ORDER_SMC);
    controller.setParam("lambda", Vector3Df(3.0, 3.0, 3.0));
    controller.setParam("K", Vector3Df(5.0, 5.0, 5.0));
    controller.setParam("phi", Vector3Df(0.1, 0.1, 0.1));
    test_controller(controller, "first_order_smc_results.csv");
    
    // Test Super-Twisting Algorithm
    std::cout << "Testing Super-Twisting Algorithm..." << std::endl;
    controller.init(ControllerType::SUPER_TWISTING);
    controller.setParam("lambda", Vector3Df(3.0, 3.0, 3.0));
    controller.setParam("alpha", Vector3Df(1.5, 1.5, 1.5));
    controller.setParam("beta", Vector3Df(1.1, 1.1, 1.1));
    test_controller(controller, "super_twisting_results.csv");
    
    // Test Continuous Nested SMC
    std::cout << "Testing Continuous Nested SMC..." << std::endl;
    controller.init(ControllerType::CONTINUOUS_NESTED);
    controller.setParam("k1", Vector3Df(6.0, 6.0, 6.0));
    controller.setParam("k2", Vector3Df(4.0, 4.0, 4.0));
    controller.setParam("k3", Vector3Df(4.0, 4.0, 4.0));
    test_controller(controller, "continuous_nested_results.csv");
    
    // Test Higher-Order STA
    std::cout << "Testing Higher-Order STA..." << std::endl;
    controller.init(ControllerType::HIGHER_ORDER_STA);
    controller.setParam("k1", Vector3Df(4.0, 4.0, 4.0));
    controller.setParam("k2", Vector3Df(1.0, 1.0, 1.0));
    controller.setParam("k3", Vector3Df(2.0, 2.0, 2.0));
    controller.setParam("k4", Vector3Df(2.0, 2.0, 2.0));
    test_controller(controller, "higher_order_sta_results.csv");
    
    // Test with estimator
    std::cout << "Testing Super-Twisting with Super-Twisting estimator..." << std::endl;
    controller.init(ControllerType::SUPER_TWISTING, EstimatorType::SUPER_TWISTING_DIFF);
    test_controller(controller, "super_twisting_with_estimator_results.csv");
    
    std::cout << "All tests completed." << std::endl;
    
    return 0;
}