# Optimized_Adaptive_MPC

This is the implementation of the work published in the following paper "Optimized adaptive MPC for lateral control of autonomous vehicles".
The paper is freely accessible at this link: https://hal.science/hal-03485108v1/preview/ICCMA2021_YK_VP_NA_VV_DI_Optimized_adaptive_MPC_for_lateral_control_of.pdf 

## Steps to run the code:

This implementation requires MATLAB 2018b or a more recent version.
-  # DMPC: 
   ### This part is an implementation of discrete MPC with Laguere function for linear systems (vehicle lateral control in this case)
     1. Run the script 'Launch_MPC.m'.
     2. The script loads trajectory data (Double lane change maneuver or other trajectories).
     3. The script loads the parameters for the vehicle model and MPC controller (Params.mat).
     4. Simulation starts with the chosen parameters. You can try different trajectories 
      
        and change the MPC parameters to compare its performance.
     5. Note that it is important to adjust MPC constraints accordingly, or it will diverge.


 -  # PSO_MPC: 
    ### This code optimizes the MPC parameters (Np, Nc, Q, R, etc.) with an enhanced PSO algorithm through iterative simulations

  1. Run the script 'PSO_MPC.m'.
  2. The script sets up the optimization problem through the script "mpc_param.m", 
    
     and executes the optimization through the script "PSO_test.m".
  3. The code simulates the vehicle model in "MPC.slx" to optimize the controller parameters by minimizing 
    
     the cost function defined in "mpc_param" (please check the paper for more details)
  4. At the end of the optimization, the data of optimal parameters will be saved 
    
     in an Excel file (check "mpc_param" to know which file) 

### You might want to check a closely related implementation in this repository (https://github.com/yassinekebbati/NN_MPC-vs-ANFIS_MPC)

### If you find this work useful or use it in your work, please cite the main paper:

Kebbati, Y., Puig, V., Ait-Oufroukh, N., Vigneron, V., & Ichalal, D. (2021, November). Optimized adaptive MPC for lateral control of autonomous vehicles. In 2021 9th International Conference on Control, Mechatronics and Automation (ICCMA) (pp. 95-103). IEEE.

@inproceedings{kebbati2021optimized,
  title={Optimized adaptive MPC for lateral control of autonomous vehicles},
  author={Kebbati, Yassine and Puig, Vicen{\c{c}} and Ait-Oufroukh, Naima and Vigneron, Vincent and Ichalal, Dalil},
  booktitle={2021 9th International Conference on Control, Mechatronics and Automation (ICCMA)},
  pages={95--103},
  year={2021},
  organization={IEEE}
}

