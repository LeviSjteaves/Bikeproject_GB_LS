%% clear the possible remnant on previous running

set(0,'defaulttextinterpreter','none');
dbclear all;
clear;
close all;
clc;

%% Simulation Settings and Bike Parameters
% General Parameters

    % Gravitational Acceleration
        g = -9.81;
    % Name of the model
        model = 'Main_v2';
    % Simulation time
        sim_time = 100;
    % Sampling Time
        Ts = 0.01; 

% Open the Simulink Model
    open([model '.slx']);
% Choose the solver
    set_param(model,'AlgebraicLoopSolver','TrustRegion');
% Choose The Bike - Options: 'red' or 'black'
    bike = 'red';
% Load the parameters of the specified bicycle
    bike_params = LoadBikeParameters(bike); 

% Initial states

% Initial Roll
        initial_state.roll = deg2rad(0);
        initial_state.roll_rate = deg2rad(0);
% Initial Steering
        initial_state.steering = deg2rad(0);
% Initial Pose (X,Y,\theta)
        initial_state.x = 0;
        initial_state.y = 0;
        initial_state.heading = deg2rad(0);
        initial_pose = [initial_state.x; initial_state.y; initial_state.heading];

%% Unpacked bike_params
h = bike_params.h_mod;
lr = bike_params.lr_mod;
lf = bike_params.lf_mod; 
lambda = bike_params.lambda_mod;
c = bike_params.c_mod;
m = bike_params.m_mod;
h_imu = bike_params.IMU_height_mod;


%% Load measurements


measurements = load();

%% Kalman Filter

% A matrix (linear bicycle model with constant velocity)
% Est_States := [X Y psi phi phi_dot delta v]
A = [0 0 0 0 0 0 1;
     0 0 v 0 0 v*(lr/(lf+lr))*sin(lambda) 0;
     0 0 0 0 0 (v/(lr+lf))*sin(lambda) 0;
     0 0 0 0 1 0 0;
     0 0 0 (g/h) 0 ((v^2*h-lr*g*c)/(h^2*(lr+lf)))*sin(lambda) 0;
     0 0 0 0 0 0 0;
     0 0 0 0 0 0 0];

% B matrix (linear bicycle model with constant velocity)
B = [0 0 0 0 ((lr*v)/(h*(lr+lf)))*sin(lambda) 1 0]';

% Including GPS
C1 = [1 0 0 0 0 0 0;
     0 1 0 0 0 0 0;
     0 0 0 g+((-h_imu*g)/h) 0 (-h_imu*(h*v^2-(g*lr*c)))*sin(lambda)/((lr+lf)*h^2) + (v^2)*sin(lambda)/(lr+lf) 0;
     0 0 0 0 1 0 0;
     0 0 0 0 0 (v)*sin(lambda)/(lr+lf) 0;
     0 0 0 0 0 1 0;
     0 0 0 0 0 0 1];

D1 = [0 0 (-h_imu*lr*v)*sin(lambda)/((lr+lf)*h) 0 0 0 0]';

% Excluding GPS
C2 = [g+((-h_imu*g)/h) 0 (-h_imu*(h*v^2-(g*lr*c)))*sin(lambda)/((lr+lf)*h^2) + (v^2)*sin(lambda)/(lr+lf) 0;
      0 1 0 0;
      0 0 (v)*sin(lambda)/(lr+lf) 0;
      0 0 1 0;
      0 0 0 1];

D2 = [(-h_imu*lr*v)*sin(lambda)/((lr+lf)*h) 0 0 0 0]';

% Discretize the model
A_d = (eye(size(A))+Ts*A);
B_d = Ts*B;

% Q and R matrix
Q = eye(7);
Q2 = eye(4);
R1 = eye(7);
R2 = eye(5);

% Compute Kalman Gain
    % including GPS
    [P1,Kalman_gain1,eig] = idare(A_d',C1',Q,R1,[],[]);
    eig1 = abs(eig);
    Kalman_gain1 = Kalman_gain1';
    Ts_GPS = 0.1; % sampling rate of the GPS
    counter = (Ts_GPS / Ts) - 1 ; % Upper limit of the counter for activating the flag

    % excluding GPS
    A_d2 = A_d;
    A_d2(1:3,:) = [];
    A_d2(:,1:3) = [];
    [P2,Kalman_gain2,eig] = idare(A_d2',C2',Q2,R2,[],[]);
    eig2 = abs(eig);
    Kalman_gain2 = Kalman_gain2';


% Polish the kalman gain (values <10-5 are set to zero)
for i = 1:size(Kalman_gain1,1)
    for j = 1:size(Kalman_gain1,2)
        if Kalman_gain1(i,j) < 10^-5
            Kalman_gain1(i,j) = 0;
        end
    end
end 
for i = 1:size(Kalman_gain2,1)
    for j = 1:size(Kalman_gain2,2)
        if Kalman_gain2(i,j) < 10^-5
            Kalman_gain2(i,j) = 0;
        end
    end
end

 %% Start the Simulation
if Run_tests == 0 || Run_tests == 2

tic
try Results = sim(model);
    catch error_details %note: the model has runned for one time here
end
toc

% Simulation Messages and Warnings
if Results.stop.Data(end) == 1
    disp('Message: End of the trajectory has been reached');
end

%% Ploting
%name of the plot
Tnumber = 'No test case: General simulation run';
        Plot_bikesimulation_results(Tnumber, Ts, test_curve, Results, bike_params);

%% Space to define the specific DEBUG plots :)
figure()
subplot(2,2,1);
plot(Results.steerrate_bikemodel.Time(:,1),Results.steerrate_bikemodel.Data(:,1))
hold on
plot(Results.steerrate_estimator.Time(:,1),Results.steerrate_estimator.Data(:,1))
title('steerrate deltadot')
legend('true bikemodel','estimator')

subplot(2,2,2);
plot(Results.states_bikemodel.Time(:,1),Results.states_bikemodel.Data(:,1))
hold on
plot(Results.states_estimator.Time(:,1),Results.states_estimator.Data(:,4))
title('Roll phi')
legend('true bikemodel','estimator')
subplot(2,2,3);
plot(Results.states_bikemodel.Time(:,1),Results.states_bikemodel.Data(:,2))
hold on
plot(Results.states_estimator.Time(:,1),Results.states_estimator.Data(:,6))
title('Steering angle delta')
legend('true bikemodel','estimator')
subplot(2,2,4);
plot(Results.states_bikemodel.Time(:,1),Results.states_bikemodel.Data(:,3))
hold on
plot(Results.states_estimator.Time(:,1),Results.states_estimator.Data(:,5))
title('Roll rate phidot')
legend('true bikemodel','estimator')
end
