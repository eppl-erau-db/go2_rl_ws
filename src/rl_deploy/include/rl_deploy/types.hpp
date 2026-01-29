#pragma once
#include <vector>

namespace rl_deploy{
    enum class Mode { 
        Idle, 
        Standing, 
        Sitting, 
        Walking, 
        Damping, 
        Killed
    };
    struct ButtonState {
        bool stand{false}, 
            sit{false}, 
            start{false},
            stop_walking{false}, 
            soft_abort{false}, 
            kill{false};
    };
    struct StatusFlags {
        bool good_stand{false}, 
            good_sit{false}, 
            is_walking{false};
    };
} 
