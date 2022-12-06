import os
import torch
from torchvision.io import read_image
from PIL import image

from task.config import ASSET_PATH, FEATURE_PATH
from learning.network.resnet import ResNetFeatureExtractor

class ObjectFeatureParser():
    """
    Use resetnet to parser image features and save them into a folder
    """
    def __init__(self, object_type) -> None:

        # load asset
        self.object_type = object_type
        self.asset_path = ASSET_PATH # parsing local images only
        object_folder = [obj for obj in os.listdir(os.path.join(self.asset_path, "I", object_type))]
        self.object_list = list(filter(lambda x: x.endswith(".usd"), object_folder))
        self.image_folder = os.path.join(self.asset_path, "I", object_type, ".thumbs", "256x256")

        # load model
        self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        self.model = ResNetFeatureExtractor(output_layer='avgpool').to(self.device)

        # make saving folder
        self.save_path = os.path.join(FEATURE_PATH, object_type)
        if not os.path.exists(self.save_path):
            os.mkdir(self.save_path)

    def parse_one_feature(self, idx):
        """
        Extract feature for one object
        """
        obj = self.object_list[idx]
        obj_name = obj[:-4]
        print("Extracting feature for object: ", self.object_list[idx])
        image_file = os.path.join(self.image_folder, obj + ".png")

        x = read_image(image_file)
        x = x.to(self.device)
        x_feature = self.model(x).squeeze()
        torch.save(x_feature, os.path.join(self.save_path, obj_name + ".pt"))
