import torch
from torch import nn
from torchvision.models.inception import inception_v3

class InceptionScorer():
    def __init__(self, model = "inception_v3", resize = False) -> None:
        """
        Value: init inception model
        """
        # Set up dtype
        if torch.cuda.is_available():
            self.dtype = torch.cuda.FloatTensor
        else:
            print("WARNING: You don't have a CUDA device")
            self.dtype = torch.FloatTensor

        # load model
        self.model = inception_v3(pretrained=True, transform_input=False).type(self.dtype)
        self.model.eval()

        # up sampling for resize
        self.resize = resize
        if self.resize:
            self.up = nn.Upsample(size=(299, 299), mode='bilinear').type(self.dtype) 
    
    def get_inception_score(self, x):   
        """
        Get inception score for one image
        """
        if self.resize:
            x = self.up(x)
        x = self.model(x)
        x = torch.softmax(x, dim = -1).data.cpu().numpy()
        return x
