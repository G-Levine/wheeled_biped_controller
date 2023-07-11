// IsaacGymPolicy.hpp
#ifndef ISAACGYMPOLICY_HPP
#define ISAACGYMPOLICY_HPP

#include <torch/script.h>
#include <array>

constexpr int NUM_OBS = 21;
constexpr int NUM_ACTIONS = 4;

constexpr double MIN_LEG_POS = -0.35;
constexpr double MAX_LEG_POS = -0.15;
constexpr double MAX_WHEEL_VEL = 30.0;
constexpr std::array<double, 4> ACTION_MIN = {MIN_LEG_POS, -MAX_WHEEL_VEL, MIN_LEG_POS, -MAX_WHEEL_VEL};
constexpr std::array<double, 4> ACTION_MAX = {MAX_LEG_POS, MAX_WHEEL_VEL, MAX_LEG_POS, MAX_WHEEL_VEL};

constexpr std::array<double, 4> DEFAULT_DOF_POS = {-0.25, 0.0, -0.25, 0.0};
constexpr std::array<double, NUM_ACTIONS> DEFAULT_ACTION = {0.0, 0.0, 0.0, 0.0};

constexpr double COMM_SCALE_X = 2.0;
constexpr double COMM_SCALE_Y = 2.0;
constexpr double COMM_SCALE_YAW = 0.25;

constexpr double OBS_SCALE_LIN_VEL = 2.0;
constexpr double OBS_SCALE_ANG_VEL = 0.25;
constexpr double OBS_SCALE_DOF_POS = 1.0;
constexpr double OBS_SCALE_DOF_VEL = 0.05;

constexpr double ACTION_SCALE_POS = 0.05;
constexpr double ACTION_SCALE_VEL = 1.0;

constexpr std::array<double, 4> ACTION_SCALES = {ACTION_SCALE_POS, ACTION_SCALE_VEL, ACTION_SCALE_POS, ACTION_SCALE_VEL};


class IsaacGymPolicy
{
public:
    IsaacGymPolicy(const std::string &checkpoint_path);
    std::array<double, NUM_ACTIONS> step(const std::array<double, NUM_OBS>& obs_array);

private:
    torch::jit::script::Module policy;
    at::Tensor prev_actions;
};

#endif  // ISAACGYMPOLICY_HPP
