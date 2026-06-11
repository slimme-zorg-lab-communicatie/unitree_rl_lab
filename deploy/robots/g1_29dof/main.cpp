#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "State_Mimic.h"

#include <mutex>
#include <chrono>
#include <memory>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
using WirelessController_t = unitree_go::msg::dds_::WirelessController_;

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = std::make_shared<Keyboard>();

static std::shared_ptr<unitree::robot::ChannelSubscriber<WirelessController_t>>
    virtual_joystick_sub = nullptr;

void virtual_joystick_callback(const void *message)
{
    const auto *msg = static_cast<const WirelessController_t *>(message);

    FSMState::set_virtual_joystick(
        msg->keys(),
        msg->lx(),
        msg->ly(),
        msg->rx(),
        msg->ry()
    );

    static int count = 0;
    if (++count % 50 == 0)
    {
        spdlog::info(
            "Virtual joystick: keys={} lx={:.2f} ly={:.2f} rx={:.2f} ry={:.2f}",
            msg->keys(),
            msg->lx(),
            msg->ly(),
            msg->rx(),
            msg->ry()
        );
    }
}

void start_virtual_joystick_subscriber()
{
    FSMState::virtual_joystick_enabled = true;

    virtual_joystick_sub =
        std::make_shared<unitree::robot::ChannelSubscriber<WirelessController_t>>(
            "rt/wireless_controller"
        );

    virtual_joystick_sub->InitChannel(virtual_joystick_callback, 10);

    spdlog::info("Listening for virtual joystick on rt/wireless_controller");
}

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::g1::subscription::LowCmd>();
    usleep(0.2 * 1e6);

    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        // exit(0);
    }

    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();

    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     G1-29dof Controller \n";

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(
        0,
        vm["network"].as<std::string>()
    );

    init_fsm_state();

    start_virtual_joystick_subscriber();

    FSMState::lowcmd->msg_.mode_machine() = 5; // 29dof

    if(!FSMState::lowcmd->check_mode_machine(FSMState::lowstate))
    {
        spdlog::critical("Unmatched robot type.");
        exit(-1);
    }
    
    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    std::cout << "FSM transitions use button combos from config.yaml.\n";
    std::cout << "Virtual joystick subscriber active on rt/wireless_controller.\n";

    while (true)
    {
        sleep(1);
    }
    
    return 0;
}