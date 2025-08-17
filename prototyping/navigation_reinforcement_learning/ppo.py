import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Normal
import numpy as np


class Actor(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Actor, self).__init__()
        self.fc1 = nn.Linear(state_dim, 2048)
        self.fc2 = nn.Linear(2048, 1024)
        self.fc3 = nn.Linear(1024, 512)
        self.fc4 = nn.Linear(512, 256)
        self.fc5 = nn.Linear(256, 64)
        self.fc6 = nn.Linear(64, action_dim)
        self.log_std = nn.Parameter(torch.zeros(1, action_dim))

    def forward(self, state):
        x = F.relu(self.fc1(state))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = F.relu(self.fc4(x))
        x = F.relu(self.fc5(x))
        action_mean = torch.tanh(self.fc6(x))
        action_std = torch.exp(self.log_std)
        return action_mean, action_std


class Critic(nn.Module):
    def __init__(self, state_dim):
        super(Critic, self).__init__()
        self.fc1 = nn.Linear(state_dim, 2048)
        self.fc2 = nn.Linear(2048, 1024)
        self.fc3 = nn.Linear(1024, 512)
        self.fc4 = nn.Linear(512, 256)
        self.fc5 = nn.Linear(256, 64)
        self.fc6 = nn.Linear(64, 1)

    def forward(self, state):
        x = F.relu(self.fc1(state))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = F.relu(self.fc4(x))
        x = F.relu(self.fc5(x))
        value = self.fc6(x)
        return value


class ReplayBuffer:
    def __init__(self):
        self.states = []
        self.actions = []
        self.rewards = []
        self.next_states = []
        self.dones = []
        self.log_probs = []  # store log probs of actions

    def store(self, state, action, reward, next_state, done, log_prob):
        self.states.append(state)
        self.actions.append(action)
        self.rewards.append(reward)
        self.next_states.append(next_state)
        self.dones.append(done)
        self.log_probs.append(log_prob)

    def get_batch(self):
        batch = {
            'states': torch.tensor(np.array(self.states), dtype=torch.float32),
            'actions': torch.tensor(np.array(self.actions), dtype=torch.float32),
            'rewards': torch.tensor(np.array(self.rewards), dtype=torch.float32),
            'next_states': torch.tensor(np.array(self.next_states), dtype=torch.float32),
            'dones': torch.tensor(np.array(self.dones), dtype=torch.float32),
            'log_probs': torch.tensor(np.array(self.log_probs), dtype=torch.float32),
        }
        self.clear()
        return batch

    def clear(self):
        self.states = []
        self.actions = []
        self.rewards = []
        self.next_states = []
        self.dones = []
        self.log_probs = []


class PPOAgent:
    def __init__(self, state_dim, action_dim, lr_actor=1e-4, lr_critic=1e-4, gamma=0.995,
                 epsilon_clip=0.2, K_epochs=10, entropy_coef=0.01, gae_lambda=0.98,
                 action_space_low=None, action_space_high=None):

        self.gamma = gamma
        self.epsilon_clip = epsilon_clip
        self.K_epochs = K_epochs
        self.entropy_coef = entropy_coef
        self.gae_lambda = gae_lambda

        self.actor = Actor(state_dim, action_dim)
        self.critic = Critic(state_dim)
        self.optimizer_actor = optim.Adam(self.actor.parameters(), lr=lr_actor)
        self.optimizer_critic = optim.Adam(
            self.critic.parameters(), lr=lr_critic)

        self.buffer = ReplayBuffer()

        # Set device to use gpu
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}")
        self.actor.to(self.device)
        self.critic.to(self.device)

        self.action_space_low = torch.tensor(
            action_space_low, dtype=torch.float32, device=self.device)
        self.action_space_high = torch.tensor(
            action_space_high, dtype=torch.float32, device=self.device)

    def select_action(self, observation):

        processed_state = observation

        state_tensor = torch.tensor(
            processed_state, dtype=torch.float32).unsqueeze(0)
        state_tensor = state_tensor.to(self.device)

        # Forward Pass
        with torch.no_grad():
            action_mean, action_std = self.actor(state_tensor)

        # Create normal distribution and sample from it
        dist = Normal(action_mean, action_std)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(axis=-1)

        clipped_action = torch.clamp(
            action, self.action_space_low, self.action_space_high)

        return clipped_action.squeeze(0).cpu().numpy(), log_prob.cpu().item()

    def compute_advantages_and_returns(self, rewards, values, next_values, dones):
        # Squeeze out the batch dimension
        values = values.flatten()
        next_values = next_values.flatten()

        # TD Errors
        deltas = rewards + self.gamma * next_values * (1 - dones) - values

        advantages = torch.zeros_like(rewards)
        returns = torch.zeros_like(values)

        last_advantage = 0
        last_return = 0

        for t in reversed(range(len(rewards))):
            if dones[t]:
                last_advantage = 0
                last_return = 0

            advantages[t] = deltas[t] + self.gamma * \
                self.gae_lambda * last_advantage
            last_advantage = advantages[t]

            returns[t] = rewards[t] + self.gamma * last_return
            last_return = returns[t]

        # Normalize advantages
        std_adv = advantages.std()
        if std_adv > 1e-6:
            advantages = (advantages - advantages.mean()) / std_adv
        else:
            advantages = advantages - advantages.mean()

        return advantages.unsqueeze(1), returns.unsqueeze(1)

    def update(self):
        # Get batch from replay buffer and clear buffer
        batch = self.buffer.get_batch()

        # Organize data
        states = batch['states']
        actions = batch['actions']
        rewards = batch['rewards']
        next_states = batch['next_states']
        dones = batch['dones']
        old_log_probs = batch['log_probs']

        # Move tensors to GPU
        states = states.to(self.device)
        actions = actions.to(self.device)
        rewards = rewards.to(self.device)
        next_states = next_states.to(self.device)
        dones = dones.to(self.device)
        old_log_probs = old_log_probs.to(self.device)

        # Compute current values with critic
        with torch.no_grad():
            values = self.critic(states).squeeze(1)
            next_values = self.critic(next_states).squeeze(1)

        # Ensure these are one dimensional
        rewards = rewards.flatten()
        dones = dones.flatten()

        # Calculate advantages and returns
        advantages, returns = self.compute_advantages_and_returns(
            rewards, values, next_values, dones)

        # Reshape log probabilities for element wise operations
        old_log_probs = old_log_probs.unsqueeze(1)

        for _ in range(self.K_epochs):
            # Evaluate current policy on collected states
            action_mean, action_std = self.actor(states)
            dist = Normal(action_mean, action_std)
            new_log_probs = dist.log_prob(actions).sum(axis=-1).unsqueeze(1)

            # Calculate ration of probability of action under new policy vs old policy
            ratio = torch.exp(new_log_probs - old_log_probs)

            # Surrogate Objective Function (Loss Function)
            surrogate1 = ratio * advantages
            surrogate2 = torch.clamp(
                ratio, 1 - self.epsilon_clip, 1 + self.epsilon_clip) * advantages

            # Actor
            # Maximize the min of the two surrogates
            actor_loss = -torch.min(surrogate1, surrogate2).mean()
            # Encourage exploration with entropy bonus
            entropy = dist.entropy().mean()
            actor_loss = actor_loss - self.entropy_coef * entropy

            # Critic
            current_values = self.critic(states)
            critic_loss = F.mse_loss(current_values, returns)

            # Update Actor
            self.optimizer_actor.zero_grad()
            actor_loss.backward()
            nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=0.5)
            self.optimizer_actor.step()

            # Update Critic
            self.optimizer_critic.zero_grad()
            critic_loss.backward()
            nn.utils.clip_grad_norm_(self.critic.parameters(), max_norm=0.5)
            self.optimizer_critic.step()

            # print(f"Actor Loss: {actor_loss.item():.4f}, Critic Loss: {critic_loss.item():.4f}, Entropy: {entropy.item():.4f}")
