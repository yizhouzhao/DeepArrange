# replay buffer
import os
from collections import deque
import torch

from learning.config import ONE_SCENE_STEPS
from task.config import DATA_PATH

class ReplayBuffer():
    def __init__(self, max_size = 1e5) -> None:
        self.dataset = deque(maxlen = max_size)
    
    def add_sample(self, scene_feature, object_feature, next_scene_feature, action, reward):
        data_piece = (scene_feature, object_feature, next_scene_feature, action, reward)
        self.dataset.append(data_piece)

    def add_scene_sample(self, scene_dict, steps = ONE_SCENE_STEPS):
        task_type = scene_dict["base"]["task_type"]
        side_choice = scene_dict["base"]["side_choice"]
        traj_id = scene_dict["base"]["traj_id"]

        image_folder = os.path.join(DATA_PATH, task_type, side_choice, str(traj_id)) 
        for i in range(steps):
            




