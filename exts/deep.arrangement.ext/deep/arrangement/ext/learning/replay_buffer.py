# replay buffer
import os
import random
import numpy as np
import torch

from collections import deque


from learning.config import ONE_SCENE_STEPS
from task.config import DATA_PATH, FEATURE_PATH

class ReplayBuffer():
    def __init__(self, max_size = 1e5) -> None:
        self.dataset = deque(maxlen = max_size)
    
    def add_sample(self, scene_feature, object_feature, next_scene_feature, action, reward):
        data_piece = (scene_feature, object_feature, next_scene_feature, action, reward)
        self.dataset.append(data_piece)

    def add_scene_sample(self, scene_dict, steps = ONE_SCENE_STEPS):
        task_type = scene_dict["base"]["task_choice"]
        side_choice = scene_dict["base"]["side_choice"]
        traj_id = scene_dict["base"]["id"]

        image_folder = os.path.join(DATA_PATH, task_type, side_choice, str(traj_id)) 
        for i in range(steps):
            scene_feature = torch.load(os.path.join(image_folder, str(i) + ".pt"))
            next_scene_feature = torch.load(os.path.join(image_folder, str(i + 1) + ".pt"))
            
            object_info = scene_dict["objects"][i]
            object_type = object_info["type"]
            obj_name = object_info["name"][:-4]
            object_feature = torch.load(os.path.join(FEATURE_PATH, object_type, obj_name + ".pt")).cpu().data

            action = object_info["action"]
            reward = object_info["reward"]["affordance"]

            # TODO: post process reward

            self.add_sample(scene_feature, object_feature, next_scene_feature, action, reward)

    def sample_batch(self, batch_size = 4):
        indexes = random.sample(range(len(self.dataset)), batch_size)
        s, so, a, r, s_next = [], [], [], [], []
        for i in indexes:
            scene_feature, object_feature, next_scene_feature, action, reward = self.dataset[i]
            s.append(scene_feature)
            so.append(object_feature)
            s_next.append(next_scene_feature)
            a.append(action)
            r.append(reward)

        s = torch.stack(s, dim = 0)
        so = torch.stack(so, dim = 0)
        s_next = torch.stack(s_next, dim = 0)

        a = torch.tensor(a).float()
        r = torch.tensor(r).float()
        t = torch.zeros(batch_size)

        return {
            "rewards": r,
            "terminals": t,
            "observations": s,
            "actions": a,
            "next_observations": s_next,
            "object_features": so,
        }



