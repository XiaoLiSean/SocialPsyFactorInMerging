# SocialPsyFactorInMerging

This repository documents the (partial) codebase for developments in the following papers. The development is on MATLAB. Simulation videos are available on the webpage [https://xiaolisean.github.io/publication/2023-10-31-TCST2024](https://xiaolisean.github.io/publication/2023-10-31-TCST2024).
 
Please cite the following Conference and Journal papers:
```
@inproceedings{li2024interaction,
  title={Interaction-aware decision-making for autonomous vehicles in forced merging scenario leveraging social psychology factors},
  author={Li, Xiao and Liu, Kaiwen and Tseng, H Eric and Girard, Anouck and Kolmanovsky, Ilya},
  booktitle={2024 American Control Conference (ACC)},
  pages={285--291},
  year={2024},
  organization={IEEE}
}
```

```
@article{li2024decision,
  title={Decision-Making for Autonomous Vehicles With Interaction-Aware Behavioral Prediction and Social-Attention Neural Network},
  author={Li, Xiao and Liu, Kaiwen and Tseng, H Eric and Girard, Anouck and Kolmanovsky, Ilya},
  journal={IEEE Transactions on Control Systems Technology},
  year={2024},
  publisher={IEEE}
}
```

## Test Forced Merging Algorithm in Simulated Traffic
1. run `DecisionMakingSimulation.m` to simulate customized highway traffic and forced merging scenarios. The highway configurations can be changed using file `./decisionMaking/highwayParams.m`
   
## Test Forced Merging Algorithm in Real World Dataset
1. please download the HighD dataset [https://levelxdata.com/highd-dataset/](https://levelxdata.com/highd-dataset/) and extract it to the folder `3rdPartyHighDRepo` in a subfolder named `data`
2. run `HighDSimulation.m` to visualize the HighD traffic
3. run `DecisionMakingHighD.m` to simulate customized highway traffic and forced merging scenarios. 
