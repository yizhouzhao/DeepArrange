# utilities
import os
import numpy as np
import torch
from PIL import Image

def extract_image_feature_and_save(image_array, feature_extractor, save_path, device = torch.device("cuda")):
    """
    Extract image feature and save to path
    ::params:
        image: [H x W x 3]
        feature_extractor: a model
        save_path: save path 
    """
    x = torch.from_numpy(image_array).float().to(device)
    x = x.permute((2,0,1)).unsqueeze(0)
    x_feature = feature_extractor(x).squeeze()
    torch.save(x_feature.cpu().data, save_path)

def extract_image_clip_feature_and_save(image_file, model, processor, save_path, device = torch.device("cuda")):
    """
    Extract clip feature and save to path
    """
    image = Image.open(image_file).convert('RGB')
    inputs = processor(images=image, return_tensors="pt").to(device)    
    x_feature = model.get_image_features(**inputs).squeeze().cpu().data
    torch.save(x_feature.cpu().data, save_path)

def obtain_action_from_trainer(image_feature_file, object_feature_file, trainer, device = torch.device("cuda"), scaler = 1.0):
    """
    Obtain action from Trainer
    ::params:
        image_feature_file: image file
        object_feature_file: object feature file
        trainer: a trainer has policy
    """
    image_feature = torch.load(image_feature_file).unsqueeze(0).to(device)
    object_feature = torch.load(object_feature_file).unsqueeze(0).to(device)

    dist = trainer.policy(image_feature, object_feature)
    action = dist.sample(scaler = scaler)
    action = action.flatten().tolist()
    return action


def soft_update_from_to(source, target, tau):
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(
            target_param.data * (1.0 - tau) + param.data * tau
        )

    