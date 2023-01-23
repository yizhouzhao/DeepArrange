from collections import OrderedDict

import numpy as np
import torch
import torch.optim as optim
from torch import nn as nn

from learning.utils import soft_update_from_to

class DDPGTrainer():
    """
    Deep Deterministic Policy Gradient
    """
    def __init__(
            self,
            qf,
            target_qf,
            policy,
            target_policy,

            discount=0.99,
            reward_scale=1.0,

            policy_learning_rate=1e-4,
            qf_learning_rate=1e-3,
            qf_weight_decay=0,
            target_hard_update_period=1000,
            tau=0.05,
            use_soft_update=True,
            qf_criterion=None,
            optimizer_class=optim.Adam,

            min_q_value=-np.inf,
            max_q_value=np.inf,

            device = torch.device("cuda"),
    ):
        super().__init__()
        self.name = "ddpg"
        self.device = device
        if qf_criterion is None:
            qf_criterion = nn.MSELoss()

        self.qf = qf.to(self.device)
        self.target_qf = target_qf.to(self.device)
        self.policy = policy.to(self.device)
        self.target_policy = target_policy.to(self.device)

        self.discount = discount
        self.reward_scale = reward_scale

        self.policy_learning_rate = policy_learning_rate
        self.qf_learning_rate = qf_learning_rate
        self.qf_weight_decay = qf_weight_decay
        self.target_hard_update_period = target_hard_update_period
        self.tau = tau
        self.use_soft_update = use_soft_update
        self.qf_criterion = qf_criterion
        self.min_q_value = min_q_value
        self.max_q_value = max_q_value

        self.qf_optimizer = optimizer_class(
            self.qf.parameters(),
            lr=self.qf_learning_rate,
        )
        self.policy_optimizer = optimizer_class(
            self.policy.parameters(),
            lr=self.policy_learning_rate,
        )

        self.eval_statistics = OrderedDict()
        self._n_train_steps_total = 0
        self._need_to_update_eval_statistics = True

    def update(self, batch):
        rewards = batch['rewards'].to(self.device)
        terminals = batch['terminals'].to(self.device)
        obs = batch['observations'].to(self.device)
        actions = batch['actions'].to(self.device)
        next_obs = batch['next_observations'].to(self.device)

        obj_features = batch['object_features'].to(self.device)

        """
        Policy operations.
        """

        policy_actions = self.policy(obs, obj_features)
        q_output = self.qf(obs, obj_features, policy_actions)
        policy_loss = - q_output.mean()

        """
        Critic operations.
        """

        next_actions = self.target_policy(next_obs, obj_features)
        # speed up computation by not backpropping these gradients
        next_actions.detach()
        target_q_values = self.target_qf(
            next_obs,
            obj_features,
            next_actions,
        )
        q_target = rewards + (1. - terminals) * self.discount * target_q_values
        q_target = q_target.detach()
        q_target = torch.clamp(q_target, self.min_q_value, self.max_q_value)
        q_pred = self.qf(obs, obj_features, actions)
        # bellman_errors = (q_pred - q_target) ** 2
        raw_qf_loss = self.qf_criterion(q_pred, q_target)

        if self.qf_weight_decay > 0:
            reg_loss = self.qf_weight_decay * sum(
                torch.sum(param ** 2)
                for param in self.qf.regularizable_parameters()
            )
            qf_loss = raw_qf_loss + reg_loss
        else:
            qf_loss = raw_qf_loss

        """
        Update Networks
        """

        self.policy_optimizer.zero_grad()
        policy_loss.backward()
        self.policy_optimizer.step()

        self.qf_optimizer.zero_grad()
        qf_loss.backward()
        self.qf_optimizer.step()

        self._update_target_networks()

    def _update_target_networks(self):
        if self.use_soft_update:
            soft_update_from_to(self.policy, self.target_policy, self.tau)
            soft_update_from_to(self.qf, self.target_qf, self.tau)

