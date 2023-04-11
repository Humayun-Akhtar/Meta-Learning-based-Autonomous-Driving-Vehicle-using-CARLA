# Meta-Learning-based-Autonomous-Driving-Vehicle-using-CARLA

## Abstract
Autonomous waypoint following is an established test case for autonomous driving scenarios, with implementations simple enough to be executed by standard proportional controllers, to higher order Deep Learning based controllers. In this project we aim to explore the design and implications of using a PID controller, a naive RL agent, and a Meta-Learning enabled RL Agent for a driving scenario with adverse conditions. The training and testing would be done in the CARLA simulation environment with variation in terrain and road characteristics.

## Simulation Environments
For all stages of training and testing, we aim to use the CARLA environment Dosovitskiy et al. [2017] with multiple scenarios. We aim to have 2 training scenarios; each with a varying level of terrain
and rain. Testing would be done on 4 other environments, with random changes to the location and levels for terrain and rain. All the models would be trained on the Jeep Wrangler model available on CARLA.

## References

Sébastien M. R. Arnold, Praateek Mahajan, Debajyoti Datta, Ian Bunner, and Konstantinos Saitas Zarkias. learn2learn: A library for meta-learning research. 8 2020.

Ignasi Clavera, Jonas Rothfuss, John Schulman, Yasuhiro Fujita, Tamim Asfour, and Pieter Abbeel. Model-based reinforcement learning via meta-policy optimization. 9 2018.

Alexey Dosovitskiy, German Ros, Felipe Codevilla, Antonio Lopez, and Vladlen Koltun. Carla: An open urban driving simulator. 11 2017.

Chelsea Finn, Pieter Abbeel, and Sergey Levine. Model-agnostic meta-learning for fast adaptation of deep networks. 3 2017.

Ashish Kumar, Zipeng Fu, Deepak Pathak, and Jitendra Malik. Rma: Rapid motor adaptation for legged robots. 7 2021. URL http://arxiv.org/abs/2107.04034.

Óscar Pérez-Gil, Rafael Barea, Elena López-Guillén, Luis M. Bergasa, Carlos Gómez-Huélamo,Rodrigo Gutiérrez, and Alejandro Díaz-Díaz. Deep reinforcement learning based control for autonomous vehicles in carla. Multimedia Tools and Applications, 81:3553–3576, 1 2022. ISSN 1380-7501. doi: 10.1007/s11042-021-11437-3. URL https://link.springer.com/10.1007/s11042-021-11437-3.
