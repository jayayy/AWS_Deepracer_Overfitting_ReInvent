<b>My journey for ATCi DeepRacer</b>
1. This repository is the guide to winning Deepracer championship by the logic of F1 race lines. 
2. We here find the most optimal racing line, the maximum speed on turns to make sure the car doesn't flow right out of it and hence, the most optimimum action space. As discrete is easier to train then the continous space, we will continue with discrete space. 
3. We then, train the model for a period of 5-6 hours on mid LR (0.0003) and MSE for quick learning and then for 3-4 hours, switch to low LR (0.00001), Hubert loss and increase batch size to (128) for convergence.
