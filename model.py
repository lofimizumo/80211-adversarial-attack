import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
from torch.distributions import Categorical

class PolicyNetwork(nn.Module):
    def __init__(self, state_size, action_size):
        super(PolicyNetwork, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(state_size, 16),
            nn.ReLU(),
            nn.Linear(64,64),
            nn.ReLU(),
            nn.Linear(64,32),
            nn.ReLU(),
            nn.Linear(32, action_size)
        )

    def forward(self, x):
        x = self.fc(x)
        return torch.softmax(x, dim=-1)

class ReinforceAgent:
    def __init__(self, state_size, action_size, lr=1e-4):
        self.policy = PolicyNetwork(state_size, action_size)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=lr)
        self.saved_log_probs = []
        self.rewards = []
        self.eps = 1e-8
        self.train_mode = True
        nn.init.xavier_uniform_(self.policy.fc[0].weight)
        nn.init.xavier_uniform_(self.policy.fc[2].weight)
        nn.init.xavier_uniform_(self.policy.fc[4].weight)
        nn.init.xavier_uniform_(self.policy.fc[6].weight)
    
    def eval(self):
        self.train_mode = False
        self.policy.eval()
    
    def train(self):
        self.train_mode = True
        self.policy.train()
    
    def save_model(self, model_name):
        torch.save(self.policy.state_dict(), model_name)
    
    def load_model(self, model_name):
        self.policy.load_state_dict(torch.load(model_name))

    def choose_action(self, state, max_mcs = None, epsilon=0.1):
        if not torch.is_tensor(state):
            state = torch.tensor(state, dtype=torch.float32).reshape(1,-1)
        probs = self.policy(state)
        
        if self.train_mode:
            if np.random.rand() < epsilon:
                action = torch.tensor([np.random.choice(len(probs[0]))])
                m = Categorical(probs)
                self.saved_log_probs.append(m.log_prob(action))
            elif max_mcs:
                action = torch.tensor(max_mcs)
                m = Categorical(probs)
                self.saved_log_probs.append(m.log_prob(action))
            else:
                m = Categorical(probs)
                action = m.sample()
                self.saved_log_probs.append(m.log_prob(action))
        else:
            action = torch.argmax(probs)
            
        return action.item()

    def update(self, gamma=0.99):
        R = 0
        policy_loss = []
        returns = []
        for r in self.rewards[::-1]:
            R = r + gamma * R
            returns.insert(0, R)
        returns = torch.tensor(returns)
        if returns.std() > 0:
            returns = (returns - returns.mean()) / (returns.std() + self.eps)
        for log_prob, R in zip(self.saved_log_probs, returns):
            policy_loss.append(-log_prob * R)
        self.optimizer.zero_grad()
        policy_loss = torch.cat(policy_loss).sum()
        policy_loss.backward()
        print(f'loss: {policy_loss.item()}')
        self.optimizer.step()

        del self.rewards[:]
        del self.saved_log_probs[:]
