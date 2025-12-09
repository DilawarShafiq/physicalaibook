# Reinforcement Learning for Manipulation

**Week 9: Learning Robotic Skills through RL**

## Overview

Reinforcement Learning (RL) enables robots to learn complex manipulation skills through trial and error. This chapter covers RL fundamentals, practical algorithms for robotics, and deployment strategies for learned policies.

## Learning Objectives

- Understand RL fundamentals for robotics
- Implement PPO and SAC algorithms
- Design effective reward functions
- Train manipulation policies in simulation
- Transfer learned policies to real robots
- Handle sparse rewards and exploration

## RL Fundamentals

### Markov Decision Process (MDP)

Robot manipulation as an MDP:
- **State (s)**: Robot joint positions, object poses, sensor readings
- **Action (a)**: Joint velocities, end-effector movements
- **Reward (r)**: Progress toward goal, penalties for failure
- **Transition**: Physics simulation or real-world dynamics

### Policy Learning

```python
# policy_network.py
import torch
import torch.nn as nn
import numpy as np

class ActorCriticPolicy(nn.Module):
    """Actor-Critic policy for robot manipulation"""

    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super().__init__()

        # Shared feature extractor
        self.features = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU()
        )

        # Actor head (policy)
        self.actor_mean = nn.Linear(hidden_dim, action_dim)
        self.actor_logstd = nn.Parameter(torch.zeros(action_dim))

        # Critic head (value function)
        self.critic = nn.Linear(hidden_dim, 1)

    def forward(self, state):
        """Forward pass for both actor and critic"""
        features = self.features(state)

        # Actor: Gaussian policy
        action_mean = self.actor_mean(features)
        action_std = torch.exp(self.actor_logstd)

        # Critic: State value
        value = self.critic(features)

        return action_mean, action_std, value

    def get_action(self, state, deterministic=False):
        """Sample action from policy"""
        action_mean, action_std, _ = self.forward(state)

        if deterministic:
            return action_mean
        else:
            dist = torch.distributions.Normal(action_mean, action_std)
            action = dist.sample()
            return action

    def evaluate_actions(self, states, actions):
        """Evaluate log probability and entropy of actions"""
        action_mean, action_std, values = self.forward(states)

        dist = torch.distributions.Normal(action_mean, action_std)
        log_probs = dist.log_prob(actions).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)

        return values, log_probs, entropy
```

## PPO: Proximal Policy Optimization

### PPO Algorithm Implementation

```python
# ppo_trainer.py
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np

class PPOTrainer:
    """PPO algorithm for robot manipulation"""

    def __init__(
        self,
        policy,
        lr=3e-4,
        gamma=0.99,
        gae_lambda=0.95,
        clip_param=0.2,
        value_coef=0.5,
        entropy_coef=0.01,
        max_grad_norm=0.5
    ):
        self.policy = policy
        self.optimizer = optim.Adam(policy.parameters(), lr=lr)

        # Hyperparameters
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip_param = clip_param
        self.value_coef = value_coef
        self.entropy_coef = entropy_coef
        self.max_grad_norm = max_grad_norm

    def compute_gae(self, rewards, values, dones):
        """Compute Generalized Advantage Estimation"""
        advantages = []
        gae = 0

        for t in reversed(range(len(rewards))):
            if t == len(rewards) - 1:
                next_value = 0
            else:
                next_value = values[t + 1]

            delta = rewards[t] + self.gamma * next_value * (1 - dones[t]) - values[t]
            gae = delta + self.gamma * self.gae_lambda * (1 - dones[t]) * gae
            advantages.insert(0, gae)

        advantages = torch.tensor(advantages, dtype=torch.float32)
        returns = advantages + torch.tensor(values, dtype=torch.float32)

        return advantages, returns

    def update(self, states, actions, old_log_probs, returns, advantages, num_epochs=10, batch_size=64):
        """PPO policy update"""

        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        for _ in range(num_epochs):
            # Mini-batch updates
            indices = np.arange(len(states))
            np.random.shuffle(indices)

            for start in range(0, len(states), batch_size):
                end = start + batch_size
                batch_indices = indices[start:end]

                batch_states = states[batch_indices]
                batch_actions = actions[batch_indices]
                batch_old_log_probs = old_log_probs[batch_indices]
                batch_returns = returns[batch_indices]
                batch_advantages = advantages[batch_indices]

                # Evaluate current policy
                values, log_probs, entropy = self.policy.evaluate_actions(
                    batch_states, batch_actions
                )

                # Policy loss (clipped surrogate objective)
                ratio = torch.exp(log_probs - batch_old_log_probs)
                surr1 = ratio * batch_advantages
                surr2 = torch.clamp(ratio, 1 - self.clip_param, 1 + self.clip_param) * batch_advantages
                policy_loss = -torch.min(surr1, surr2).mean()

                # Value loss
                value_loss = nn.MSELoss()(values.squeeze(), batch_returns)

                # Entropy bonus
                entropy_loss = -entropy.mean()

                # Total loss
                loss = policy_loss + self.value_coef * value_loss + self.entropy_coef * entropy_loss

                # Optimize
                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad_norm)
                self.optimizer.step()

        return {
            'policy_loss': policy_loss.item(),
            'value_loss': value_loss.item(),
            'entropy': -entropy_loss.item()
        }
```

### Training Loop with Isaac Gym

```python
# train_ppo.py
import torch
import numpy as np
from isaac_gym_env import ParallelReachEnv
from policy_network import ActorCriticPolicy
from ppo_trainer import PPOTrainer

def train_manipulation():
    """Train robot manipulation with PPO"""

    # Environment
    num_envs = 1024
    env = ParallelReachEnv(num_envs=num_envs)

    # Policy
    state_dim = 10  # Joint positions + goal position
    action_dim = 6  # Joint velocities
    policy = ActorCriticPolicy(state_dim, action_dim).cuda()

    # Trainer
    trainer = PPOTrainer(policy)

    # Training loop
    num_iterations = 1000
    horizon = 128  # Steps per iteration

    for iteration in range(num_iterations):
        states = []
        actions = []
        rewards = []
        dones = []
        values = []
        log_probs = []

        # Collect trajectories
        state = env.reset()
        for step in range(horizon):
            # Get action from policy
            with torch.no_grad():
                action = policy.get_action(state)
                _, _, value = policy(state)

            # Step environment
            next_state, reward, done, _ = env.step(action)

            # Store transition
            states.append(state)
            actions.append(action)
            rewards.append(reward)
            dones.append(done)
            values.append(value)

            state = next_state

        # Compute advantages
        states = torch.stack(states)
        actions = torch.stack(actions)
        values = torch.stack(values).squeeze()
        advantages, returns = trainer.compute_gae(rewards, values, dones)

        # Update policy
        metrics = trainer.update(states, actions, log_probs, returns, advantages)

        # Logging
        if iteration % 10 == 0:
            avg_reward = np.mean([r.mean().item() for r in rewards])
            print(f"Iteration {iteration}")
            print(f"  Avg Reward: {avg_reward:.2f}")
            print(f"  Policy Loss: {metrics['policy_loss']:.4f}")
            print(f"  Value Loss: {metrics['value_loss']:.4f}")

        # Save checkpoint
        if iteration % 100 == 0:
            torch.save(policy.state_dict(), f'policy_{iteration}.pth')

if __name__ == '__main__':
    train_manipulation()
```

## Reward Engineering

### Designing Effective Rewards

```python
# reward_functions.py
import torch

class ManipulationRewards:
    """Reward functions for manipulation tasks"""

    @staticmethod
    def reaching_reward(ee_pos, goal_pos, prev_dist=None):
        """Reward for reaching task"""
        # Distance to goal
        dist = torch.norm(ee_pos - goal_pos, dim=-1)

        # Reward shaping: progress reward
        if prev_dist is not None:
            progress = prev_dist - dist
            reward = progress * 10.0  # Scale progress
        else:
            reward = -dist

        # Bonus for reaching goal
        reached = (dist < 0.05).float()
        reward += reached * 100.0

        return reward, dist

    @staticmethod
    def grasping_reward(ee_pos, object_pos, gripper_force, is_grasped):
        """Reward for grasping task"""
        # Distance to object
        dist_to_object = torch.norm(ee_pos - object_pos, dim=-1)

        # Approaching reward
        approach_reward = -dist_to_object * 5.0

        # Grasping reward
        grasp_reward = is_grasped.float() * 50.0

        # Grip force penalty (avoid excessive force)
        force_penalty = -torch.clamp(gripper_force - 1.0, min=0) * 0.1

        total_reward = approach_reward + grasp_reward + force_penalty

        return total_reward

    @staticmethod
    def pick_and_place_reward(ee_pos, object_pos, goal_pos, is_grasped, object_at_goal):
        """Reward for pick and place task"""

        if not is_grasped:
            # Phase 1: Approach and grasp object
            dist_to_object = torch.norm(ee_pos - object_pos, dim=-1)
            reward = -dist_to_object * 10.0

            # Bonus for grasping
            if torch.all(dist_to_object < 0.05):
                reward += 100.0
        else:
            # Phase 2: Move to goal
            dist_to_goal = torch.norm(object_pos - goal_pos, dim=-1)
            reward = -dist_to_goal * 10.0

            # Bonus for placement
            if object_at_goal:
                reward += 200.0

        return reward

    @staticmethod
    def add_regularization(reward, actions, velocities):
        """Add regularization penalties"""
        # Action smoothness
        action_penalty = -0.01 * torch.sum(actions ** 2, dim=-1)

        # Velocity penalty (discourage jerky motions)
        velocity_penalty = -0.001 * torch.sum(velocities ** 2, dim=-1)

        return reward + action_penalty + velocity_penalty
```

## Sim-to-Real Transfer

### Domain Randomization for Transfer

```python
# domain_randomization.py
import torch
import numpy as np

class DomainRandomizer:
    """Apply domain randomization during training"""

    def __init__(self, env):
        self.env = env

    def randomize_physics(self):
        """Randomize physics parameters"""
        # Mass variation ±20%
        mass_mult = np.random.uniform(0.8, 1.2)

        # Friction variation
        friction = np.random.uniform(0.3, 1.2)

        # Damping variation
        damping = np.random.uniform(0.5, 2.0)

        self.env.set_physics_params(mass_mult, friction, damping)

    def randomize_observations(self, obs):
        """Add sensor noise to observations"""
        # Gaussian noise
        noise = torch.randn_like(obs) * 0.01

        # Random dropout (sensor failures)
        dropout_mask = (torch.rand_like(obs) > 0.05).float()

        return obs + noise * dropout_mask

    def randomize_dynamics(self):
        """Randomize actuator dynamics"""
        # Actuator delay
        delay = np.random.randint(0, 3)  # 0-2 timesteps

        # Actuator noise
        noise_std = np.random.uniform(0.01, 0.05)

        return delay, noise_std
```

## Lab 9.1: PPO for Reaching

### Objective
Train reaching policy with PPO.

### Requirements
1. Implement PPO algorithm
2. Design reaching reward function
3. Train in Isaac Gym with 1024 envs
4. Achieve >90% success rate
5. Visualize learned behavior

## Lab 9.2: Pick and Place

### Objective
Learn pick-and-place manipulation.

### Requirements
1. Multi-phase reward function
2. Train with domain randomization
3. Test on varied objects and poses
4. Measure success rate and execution time

## Lab 9.3: Sim-to-Real Transfer

### Objective
Deploy learned policy to real robot.

### Requirements
1. Train policy with aggressive randomization
2. Export policy for inference
3. Deploy to real robot arm
4. Measure performance gap
5. Document transfer challenges

## Summary

This chapter covered:
- **RL Fundamentals**: MDPs and policy learning
- **PPO Algorithm**: Stable on-policy learning
- **Reward Engineering**: Designing effective rewards
- **Domain Randomization**: Sim-to-real transfer
- **Parallel Training**: Scaling with Isaac Gym

RL enables robots to learn complex skills through experience!

---

**Next**: [End-to-End Perception-Action Pipelines →](./perception-action)