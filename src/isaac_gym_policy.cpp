#include "wheeled_biped_controller/isaac_gym_policy.hpp"
#include <iostream>
#include <memory>
#include <array>

IsaacGymPolicy::IsaacGymPolicy(const std::string &checkpoint_path) 
{
    try {
        std::cout << "Loading model\n";
        policy = torch::jit::load(checkpoint_path);
    }
    catch (const c10::Error& e) {
        std::cout << "Error loading the model\n";
    }
    
    policy.to(torch::kDouble);
    prev_actions = torch::zeros({NUM_ACTIONS}, torch::kDouble);

    obs_tensor = torch::zeros({NUM_OBS}, torch::kDouble);
    action_min_tensor = torch::from_blob(const_cast<double*>(ACTION_MIN.data()), {ACTION_MIN.size()}, torch::kDouble);
    action_max_tensor = torch::from_blob(const_cast<double*>(ACTION_MAX.data()), {ACTION_MAX.size()}, torch::kDouble);
    default_dof_pos_tensor = torch::from_blob(const_cast<double*>(DEFAULT_DOF_POS.data()), {DEFAULT_DOF_POS.size()}, torch::kDouble);
    action_scales_tensor = torch::from_blob(const_cast<double*>(ACTION_SCALES.data()), {ACTION_SCALES.size()}, torch::kDouble);
    
    processed_obs = torch::zeros_like(obs_tensor);
    actions = torch::zeros({NUM_ACTIONS}, torch::kDouble);
    actions_scaled = torch::zeros_like(actions);
}

std::array<double, NUM_ACTIONS> IsaacGymPolicy::step(const std::array<double, NUM_OBS>& obs_array)
{
    torch::NoGradGuard no_grad;

    obs_tensor.copy_(torch::from_blob(const_cast<double*>(obs_array.data()), {NUM_OBS}, torch::kDouble));

    // Create a copy of the input tensor for processing
    processed_obs.copy_(obs_tensor);
    processed_obs.squeeze_(0);

    // Apply transformations
    processed_obs.slice(0,0,3) *= OBS_SCALE_ANG_VEL;
    processed_obs[6] *= COMM_SCALE_X;
    processed_obs[7] *= COMM_SCALE_Y;
    processed_obs[8] *= COMM_SCALE_YAW;
    processed_obs.slice(0,9,13).sub_(default_dof_pos_tensor).mul_(OBS_SCALE_DOF_POS);
    processed_obs[10] = 0.0;
    processed_obs[12] = 0.0;
    processed_obs.slice(0,13,17).mul_(OBS_SCALE_DOF_VEL);
    processed_obs.slice(0,17,21) = prev_actions;

    // Run through model
    processed_obs.unsqueeze_(0);
    actions.copy_(at::squeeze(policy.forward({processed_obs}).toTensor()));

    // Post-processing
    actions_scaled.copy_(actions.mul(action_scales_tensor).add(default_dof_pos_tensor));
    actions_scaled.clamp_(action_min_tensor, action_max_tensor);

    prev_actions.copy_(actions.clamp((action_min_tensor - default_dof_pos_tensor) / action_scales_tensor,
                                (action_max_tensor - default_dof_pos_tensor) / action_scales_tensor));

    std::array<double, NUM_ACTIONS> actions_array;
    std::memcpy(actions_array.data(), actions_scaled.data_ptr<double>(), NUM_ACTIONS * sizeof(double));

    return actions_array;
}