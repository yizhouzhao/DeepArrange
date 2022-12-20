# utilities
import os
import numpy as np
import torch

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



def soft_update_from_to(source, target, tau):
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(
            target_param.data * (1.0 - tau) + param.data * tau
        )