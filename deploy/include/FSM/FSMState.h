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
#include <unitree/dds_wrapper/common/unitree_joystick.hpp>

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
                        [func]()->bool
                        {
                            if (FSMState::virtual_joystick_is_fresh())
                            {
                                return func(FSMState::get_virtual_joystick());
                            }

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

    static inline std::mutex virtual_joystick_mutex;
    static inline unitree::common::UnitreeJoystick virtual_joystick;
    static inline std::chrono::steady_clock::time_point virtual_joystick_stamp =
        std::chrono::steady_clock::now() - std::chrono::seconds(10);
    static inline bool virtual_joystick_enabled = false;

    static void update_unitree_joystick_from_wireless(
        unitree::common::UnitreeJoystick& joy,
        uint32_t keys,
        float lx,
        float ly,
        float rx,
        float ry
    )
    {
        unitree::common::BtnUnion btn{};
        btn.value = static_cast<uint16_t>(keys & 0xFFFFu);

        joy.back(btn.components.Select);
        joy.start(btn.components.Start);
        joy.LB(btn.components.L1);
        joy.RB(btn.components.R1);
        joy.F1(btn.components.f1);
        joy.F2(btn.components.f2);
        joy.A(btn.components.A);
        joy.B(btn.components.B);
        joy.X(btn.components.X);
        joy.Y(btn.components.Y);
        joy.up(btn.components.up);
        joy.down(btn.components.down);
        joy.left(btn.components.left);
        joy.right(btn.components.right);
        joy.LT(btn.components.L2);
        joy.RT(btn.components.R2);
        joy.lx(lx);
        joy.ly(ly);
        joy.rx(rx);
        joy.ry(ry);
    }

    static void set_virtual_joystick(
        uint32_t keys,
        float lx,
        float ly,
        float rx,
        float ry
    )
    {
        std::lock_guard<std::mutex> lock(virtual_joystick_mutex);

        update_unitree_joystick_from_wireless(
            virtual_joystick,
            keys,
            lx,
            ly,
            rx,
            ry
        );

        virtual_joystick_stamp = std::chrono::steady_clock::now();
    }

    static unitree::common::UnitreeJoystick get_virtual_joystick()
    {
        std::lock_guard<std::mutex> lock(virtual_joystick_mutex);
        return virtual_joystick;
    }

    static bool virtual_joystick_is_fresh()
    {
        std::lock_guard<std::mutex> lock(virtual_joystick_mutex);

        const auto now = std::chrono::steady_clock::now();
        const double age = std::chrono::duration<double>(
            now - virtual_joystick_stamp
        ).count();

        return virtual_joystick_enabled && age < 0.3;
    }
};