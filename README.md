# Fast command

cd Research/DeepArrange
cd exts/deep.arrangement.ext/deep/arrangement/ext

/home/yizhou/.local/share/isaac_sim-2022.1.1/jupyter_notebook.sh
~/.local/share/ov/pkg/isaac_sim-2022.1.1/python.sh 
~/.local/share/ov/pkg/isaac_sim-2022.1.1/jupyter_notebook.sh

## new version isaac-sim doesn't work
~/.local/share/ov/pkg/isaac_sim-2022.2.0/jupyter_notebook.sh


# Data Labeling question
1. How natural/real does the synthesized image look like?
(Great, good, normal, Bad, Poor)
1. How does the synthesized image look like a wall decoration (desk arangement, table arangement, bookshelf)?
(Great, good, normal, Bad, Poor)

Task:
Wall decoration:
1000 Trajectories x 3 objects(image .png) x 2 questions x 3 samples x $0.01 = 180 dollars



## windows
C:\Users\zhaoy\AppData\Local\ov\pkg\isaac_sim-2022.1.1\python.bat


## link folder
ln -s learning exts/deep.arrangement.ext/deep/arrangement/ext/learning

# Dependencies

python -m pip install torch

cd exts/deep.arrangement.ext/deep/arrangement/ext



# DeepArrange

1. Scene

水果、书桌、植物、床、书架、挂画

2. Assets

Fruits:
    餐桌
        [1] 圆桌 [2] 长桌 [3] 方桌
    水果
        [1] Apple [2] Avocado [3] Kiwi [4] Lime [5] Pomegrante [6] Orange


3. Evaluation metrics

Utility: Gravity/Perturbation/ Easy access
Value: Inception score/CLIP score/VQA score/User study


D:\DeepArrange\Asset


# isaac-sim path