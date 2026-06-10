#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "State_Mimic.h"

#include <mutex>
#include <chrono>
#include <memory>
#include <algorithm>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>

using WirelessController_t = unitree_go::msg::dds_::WirelessController_;

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = std::make_shared<Keyboard>();

std::mutex FSMState::virtual_joystick_mutex;
VirtualJoystick FSMState::virtual_joystick;
std::chrono::steady_clock::time_point FSMState::virtual_joystick_stamp =
    std::chrono::steady_clock::now() - std::chrono::seconds(10);
bool FSMState::virtual_joystick_enabled = true;

static std::shared_ptr<unitree::robot::ChannelSubscriber<WirelessController_t>>
    virtual_joystick_sub = nullptr;

// Common Unitree WirelessController bit layout.
static constexpr uint32_t BTN_R1     = 1u << 0;
static constexpr uint32_t BTN_L1     = 1u << 1;
static constexpr uint32_t BTN_START  = 1u << 2;
static constexpr uint32_t BTN_SELECT = 1u << 3;
static constexpr uint32_t BTN_R2     = 1u << 4;
static constexpr uint32_t BTN_L2     = 1u << 5;
static constexpr uint32_t BTN_F1     = 1u << 6;
static constexpr uint32_t BTN_F2     = 1u << 7;
static constexpr uint32_t BTN_A      = 1u << 8;
static constexpr uint32_t BTN_B      = 1u << 9;
static constexpr uint32_t BTN_X      = 1u << 10;
static constexpr uint32_t BTN_Y      = 1u << 11;
static constexpr uint32_t BTN_UP     = 1u << 12;
static constexpr uint32_t BTN_RIGHT  = 1u << 13;
static constexpr uint32_t BTN_DOWN   = 1u << 14;
static constexpr uint32_t BTN_LEFT   = 1u << 15;

static bool has_buttons(uint32_t keys, uint32_t required)
{
    return (keys & required) == required;
}

static std::string normalize_condition(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
        [](unsigned char c){ return std::tolower(c); });

    s.erase(
        std::remove_if(
            s.begin(),
            s.end(),
            [](unsigned char c)
            {
                return std::isspace(c) || c == '[' || c == ']' || c == '(' || c == ')';
            }
        ),
        s.end()
    );

    return s;
}

void FSMState::set_virtual_joystick(
    uint32_t keys,
    float lx,
    float ly,
    float rx,
    float ry
)
{
    std::lock_guard<std::mutex> lock(virtual_joystick_mutex);

    virtual_joystick.keys = keys;
    virtual_joystick.lx = lx;
    virtual_joystick.ly = ly;
    virtual_joystick.rx = rx;
    virtual_joystick.ry = ry;

    virtual_joystick_stamp = std::chrono::steady_clock::now();
}

VirtualJoystick FSMState::get_virtual_joystick()
{
    std::lock_guard<std::mutex> lock(virtual_joystick_mutex);
    return virtual_joystick;
}

bool FSMState::virtual_joystick_is_fresh()
{
    std::lock_guard<std::mutex> lock(virtual_joystick_mutex);

    const auto now = std::chrono::steady_clock::now();
    const double age = std::chrono::duration<double>(
        now - virtual_joystick_stamp
    ).count();

    return virtual_joystick_enabled && age < 0.3;
}

bool FSMState::virtual_condition_match(
    const std::string& condition,
    const VirtualJoystick& joy
)
{
    spdlog::info("checking virtual condition='{}' keys={}", condition, joy.keys);
    const std::string c = normalize_condition(condition);

    // These cover the transitions printed in main.cpp:
    // [L2 + Up] -> FixStand
    // [R1 + X]  -> RL / Velocity control

    if (
        c.find("l2") != std::string::npos &&
        c.find("up") != std::string::npos
    )
    {
        return has_buttons(joy.keys, BTN_L2 | BTN_UP);
    }

    if (
        c.find("r1") != std::string::npos &&
        c.find("x") != std::string::npos
    )
    {
        return has_buttons(joy.keys, BTN_R1 | BTN_X);
    }

    // Optional: add more known combos here if your FSM YAML uses them.
    if (
        c.find("l2") != std::string::npos &&
        c.find("down") != std::string::npos
    )
    {
        return has_buttons(joy.keys, BTN_L2 | BTN_DOWN);
    }

    if (
        c.find("r1") != std::string::npos &&
        c.find("a") != std::string::npos
    )
    {
        return has_buttons(joy.keys, BTN_R1 | BTN_A);
    }

    return false;
}

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

    std::cout << "Press [L2 + Up] to enter FixStand mode.\n";
    std::cout << "And then press [R1 + X] to start controlling the robot.\n";
    std::cout << "Virtual joystick subscriber active on rt/wireless_controller.\n";

    while (true)
    {
        sleep(1);
    }
    
    return 0;
}