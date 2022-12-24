import random
import numpy as np
import os
import time
from PIL import Image

import torch
torch.__version__
device = torch.device("cuda")

import getpass
user = getpass.getuser()
print(user)

usd_path = f"omniverse://localhost/Users/{user}/uva_sac.usd"
from omni.isaac.kit import SimulationApp    
simulation_app = SimulationApp({"headless": True, "open_usd": usd_path,  "livesync_usd": usd_path}) 

from uva_env import UvaEnv
env = UvaEnv()

from task.utils import add_scene_default
add_scene_default()

print(list(env.stage.TraverseAll()))

env.clean()
env.world.step(render=True)

from task.config import DATA_PATH, FEATURE_PATH
task_type = "Table"
side_choice = "Border"
base_asset_id = 0
load_nucleus = True

from task.scene import ArrangeScene
scene = ArrangeScene(task_type, side_choice, base_asset_id = 0, traj_id = 0, load_nucleus = load_nucleus)
env.scene = scene

# add base
scene.add_base_asset()
env.world.step(render=True)

from uv.reward import Rewarder
rewarder = Rewarder(env.world)
env.rewarder = rewarder

from render.helper import RenderHelper
render = RenderHelper(task_type, side_choice)

render.add_task_cameras()
render.set_cameras()

# Learning
# feature extraction
from learning.utils import extract_image_clip_feature_and_save, obtain_action_from_trainer
from transformers import CLIPProcessor, CLIPModel

feature_model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32").to(device)
feature_processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")

# replay buffer
import json
from learning.replay_buffer import ReplayBuffer
from learning.config import *

buffer = ReplayBuffer(max_size=2000)

# trainer
from learning.network.sac import *

policy = Policy()

qf1 = QFunction()
qf2 = QFunction()
target_qf1 = QFunction()
target_qf2 = QFunction()

from learning.sac_trainer import SACTrainer

trainer = SACTrainer(policy, qf1, qf2, target_qf1, target_qf2, 
     use_automatic_entropy_tuning = False, 
     policy_lr=1e-3, 
     qf_lr=1e-3,
     target_update_period = 5)

# main

use_network = True
debug = True

total_traj = 0
total_step = 0

# traj config
for traj_id in range(200):
    total_traj += 1
    
    base_asset_id = 0
    env.scene.base_asset_id = base_asset_id
    env.scene.traj_id = traj_id
    image_folder = os.path.join(DATA_PATH, task_type, side_choice, str(traj_id))

    # base
    # scene.add_base_asset()
    env.world.step(render = True)

    # get images
    env.world.render()
    images = render.get_images()
    render.save_rgb(images[0]['rgb'], image_folder, "0")
    
    ## extract feature
    
#     extract_image_feature_and_save(images[0]['rgb'][:,:,:3], 
#         feature_extractor, os.path.join(image_folder, str(0) + ".pt"))
    extract_image_clip_feature_and_save(f"{image_folder}/{0}.png", feature_model, feature_processor, 
        f"{image_folder}/{0}.pt",)
    

    # trajectory
    for step in range(5):
        total_step += 1
        
        # sample an object
        env.add_scene_obj(mode = "random")
        
        # TODO: get action from sampling
        if not use_network or total_traj < 10 or np.random.rand() < 0.2:
            x, y = np.tanh(np.random.randn()), np.tanh(np.random.randn())
        else:
            image_feature_file = f"{image_folder}/{step}.pt"
            
            object_info = env.scene.objects[-1]
            object_type = object_info["type"]
            obj_name = object_info["name"][:-4]
            object_feature_file = os.path.join(FEATURE_PATH, object_type, obj_name + ".pt")
            x, y = obtain_action_from_trainer(image_feature_file, object_feature_file, trainer)
        
        # load the object into the scene
        env.put_last_object((x, y)) 
        env.world.step(render=True)
        
        # register the object to the world for physics update
        env.register_last_object()
        env.world.step(render=True)

        # get images
        env.world.render()
        images = render.get_images()
        render.save_rgb(images[0]['rgb'], image_folder, str(step + 1))

        ## calculate reward
        env.calculate_last_reward(simulation_step = 30)
        
        ## extract feature
#         extract_image_feature_and_save(images[0]['rgb'][:,:,:3], 
#             feature_extractor, os.path.join(image_folder, str(step + 1) + ".pt"))
        extract_image_clip_feature_and_save(f"{image_folder}/{step + 1}.png", feature_model, feature_processor, 
            f"{image_folder}/{step + 1}.pt",)
    

        ## reset
        env.world.reset()
        env.world.step(render=True)
        
        ## trainer nework
        if use_network and total_step % UPDATE_TRAINER_STEPS == 0 and total_traj > 5:
            batch = buffer.sample_batch(batch_size = BATCH_SIZE)
            trainer.update(batch)
            
            if debug:
                rewards = batch['rewards'].to(device)
                terminals = batch['terminals'].to(device)
                obs = batch['observations'].to(device)
                actions = batch['actions'].to(device)
                next_obs = batch['next_observations'].to(device)
                obj_features = batch['object_features'].to(device)
                
                dist = trainer.policy(obs, obj_features)
                pred = trainer.qf1(obs, obj_features, actions)
                print(f"debug {total_traj}/{total_step}", #"\n dist: ", dist.mean.flatten().tolist(), dist.stddev.flatten().tolist(),
                      "\n pred:", pred.flatten().tolist(),
                      "\n rewards: ", rewards.flatten().tolist())


    # Record
    record = env.scene.get_scene_data()
    env.scene.save_scene_data()
    # print("record: ", record)
    
    # Add record to buffer
    buffer.add_scene_sample(record)

    # Reset (env clean)
    env.clean(clean_all = False)
    env.step(render = True)