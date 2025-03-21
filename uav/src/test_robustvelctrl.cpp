#include "RobustVelCtrl.h"
#include <Vector3Df.h>
#include <iostream>

using namespace flair::core;

int main() {
    // Create RobustVelCtrl instance
    RobustVelCtrl controller;
    
    // Initialize with Continuous Nested SMC
    if (!controller.init(ControllerType::CONTINUOUS_NESTED)) {
        std::cerr << "Failed to initialize controller" << std::endl;
        return 1;
    }
    
    std::cout << "Controller initialized successfully." << std::endl;
    
    // Set parameters
    controller.setParam("mg", 9.81f);
    controller.setParam("k1", Vector3Df(6.0f, 6.0f, 6.0f));
    controller.setParam("k2", Vector3Df(4.0f, 4.0f, 4.0f));
    controller.setParam("k3", Vector3Df(4.0f, 4.0f, 4.0f));
    
    // Test calculation
    Vector3Df pos(1.0f, 1.0f, 0.0f);
    Vector3Df vel(0.0f, 0.0f, 0.0f);
    Vector3Df acc(0.0f, 0.0f, 0.0f);
    
    Vector3Df pos_d(0.0f, 0.0f, 1.0f);
    Vector3Df vel_d(0.0f, 0.0f, 0.0f);
    Vector3Df acc_d(0.0f, 0.0f, 0.0f);
    
    Vector3Df u, u_dot;
    
    controller.calculate_hlc(pos, vel, acc, pos_d, vel_d, acc_d, u, u_dot);
    
    std::cout << "Control output: " << u.x << ", " << u.y << ", " << u.z << std::endl;
    
    return 0;
}