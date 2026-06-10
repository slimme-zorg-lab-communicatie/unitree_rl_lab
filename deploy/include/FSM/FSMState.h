#pragma once

#include <mutex>
#include <chrono>
#include <cstdint>
#include <string>
#include <map>

#include "Types.h"
#include "param.h"
#include "FSM/BaseState.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include "unitree_joystick_dsl.hpp"

struct VirtualJoystick
{
    uint32_t keys = 0;
    float lx = 0.0f;
    float ly = 0.0f;
    float rx = 0.0f;
    float ry = 0.0f;
};

class FSMState : public BaseState
{
public:
    FSMState(int state, std::string state_string) 
    : BaseState(state, state_string) 
    {
        spdlog::info("Initializing State_{} ...", state_string);

        auto transitions = param::config["FSM"][state_string]["transitions"];

        if(transitions)
        {
            auto transition_map = transitions.as<std::map<std::string, std::string>>();

            for(auto it = transition_map.begin(); it != transition_map.end(); ++it)
            {
                std::string target_fsm = it->first;
                if(!FSMStringMap.right.count(target_fsm))
                {
                    spdlog::warn("FSM State_'{}' not found in FSMStringMap!", target_fsm);
                    continue;
                }

                int fsm_id = FSMStringMap.right.at(target_fsm);

                std::string condition = it->second;
                unitree::common::dsl::Parser p(condition);
                auto ast = p.Parse();
                auto func = unitree::common::dsl::Compile(*ast);

                registered_checks.emplace_back(
                    std::make_pair(
                        [func, condition]()->bool
                        {
                            // 1. First try virtual joystick.
                            if (FSMState::virtual_joystick_is_fresh())
                            {
                                auto joy = FSMState::get_virtual_joystick();

                                if (FSMState::virtual_condition_match(condition, joy))
                                {
                                    return true;
                                }

                                // If virtual joystick is active, do not also let stale/physical
                                // joystick accidentally trigger transitions.
                                return false;
                            }

                            // 2. Fallback to original physical/MuJoCo joystick path.
                            return func(FSMState::lowstate->joystick);
                        },
                        fsm_id
                    )
                );
            }
        }

        // register for all states
        registered_checks.emplace_back(
            std::make_pair(
                []()->bool{ return lowstate->isTimeout(); },
                FSMStringMap.right.at("Passive")
            )
        );
    }

    void pre_run()
    {
        lowstate->update();
        if(keyboard) keyboard->update();
    }

    void post_run()
    {
        lowcmd->unlockAndPublish();
    }

    static std::unique_ptr<LowCmd_t> lowcmd;
    static std::shared_ptr<LowState_t> lowstate;
    static std::shared_ptr<Keyboard> keyboard;

    static std::mutex virtual_joystick_mutex;
    static VirtualJoystick virtual_joystick;
    static std::chrono::steady_clock::time_point virtual_joystick_stamp;
    static bool virtual_joystick_enabled;

    static void set_virtual_joystick(
        uint32_t keys,
        float lx,
        float ly,
        float rx,
        float ry
    );

    static VirtualJoystick get_virtual_joystick();
    static bool virtual_joystick_is_fresh();

    static bool virtual_condition_match(
        const std::string& condition,
        const VirtualJoystick& joy
    );
};