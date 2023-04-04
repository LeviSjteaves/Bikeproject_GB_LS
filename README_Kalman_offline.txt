MATLAB startup file = Kalman_offline.m 

line 17: Sim_time =... % 20sec for Yixiao's data

line 22: select =... % select which input measurements you want to use. 
0 is default for the measurement data provided by Yixiao. 
1 is for the recorded measurements from a simulation (which follows an 8/infinity trajectory shape)

line 46: v=... % The velocity which is used for the kalman estimation. 
Yixiao's data has no constant velocity (but reaches a velocity of about 2)
The simulation data is recoreded with a constant velocity of 3

line 167: Q and R matrix % Now an identity matrix

SIMULINK file = Kalman_offline_sim.slx

A scope is places in the simulink. The desired data can be connected from the muxes which are placed in the simulink as well.
The order of states/measurements is indicated below each mux. 
For comparing the estimation of Yixiao with the estimation of Gaizka&Levi, one can use the upper two muxes.