#pragma once
#include <vector>

namespace rl_deploy{
    enum class Mode { 
        Idle, 
        Standing, 
        Sitting,
        EmergencySitting,  // Slower sit for safety recovery
        Walking, 
        Pedipulation,
        Damping, 
        Killed
    };
    struct ButtonState {
        bool stand{false}, 
            sit{false},
            emergency_sit{false},
            start{false},
            pedipulate{false},
            f1{false},
            soft_abort{false}, 
            kill{false};
    };
    struct StatusFlags {
        bool good_stand{false}, 
            good_sit{false}, 
            is_walking{false};
    };

    // Toggle individual safety mechanisms on/off for testing
    struct SafetyConfig {
        bool enable_joint_limit_monitor{true}; // Emergency sit on joint limit violation
    };
} 
