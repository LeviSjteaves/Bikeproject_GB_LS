%% clear the possible remnant on previous running

set(0,'defaulttextinterpreter','none');
dbclear all;
clear;
close all;
clc;

%% Simulation Settings and Bike Parameters
% General Parameters

    % Gravitational Acceleration
        g = 9.81;
    % Name of the model
        model = 'Kalman_offline_sim';
    % Sampling Time
        Ts = 0.027; 
    % Open the Simulink Model
        open([model '.slx']);
    % Choose the solver
        set_param(model,'AlgebraicLoopSolver','TrustRegion');

    
%%%%%%%%%%%%%GUI%%%%%%%%%%%%%%%%%
% % reduce/increase simulation time for desired timescale on x-axis of the plots
%     sim_time = 50;
% 0 for Yixiao's measurement data, 1 for measurement data which is recorded during a simulation (infinity shape)
% 2 for measurement data 'data' from labview
    select = 2;
% Take into account a valid speed. 
% Yixiao's measurements: 0-2 m/s.
% Simulation measurements: 3 m/s
    v=2; 
% set the initial global coordinate system for gps coordinates
    gps_delay = 5;
% Choose The Bike - Options: 'red' or 'black' 
% Yixiao uses black 
% Simulation uses red
    bike = 'red';
% Load the parameters of the specified bicycle
    bike_params = LoadBikeParameters(bike); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Unpacked bike_params

h = bike_params.h_mod;
lr = bike_params.lr_mod;
lf = bike_params.lf_mod; 
lambda = bike_params.lambda_mod;
c = bike_params.c_mod;
m = bike_params.m_mod;
h_imu = bike_params.IMU_height_mod;
gpsflag = [0 0];

% Correction IMU
IMU_x = bike_params.IMU_x_mod;
IMU_roll = bike_params.IMU_roll_mod;
IMU_pitch = bike_params.IMU_pitch_mod;            
IMU_yaw = bike_params.IMU_yaw_mod;

% Convert orientation offsets to radians
roll = IMU_roll * pi / 180;
pitch = IMU_pitch * pi / 180;
yaw = IMU_yaw * pi / 180;

% Calculate transformation matrix
Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];   % Rotation matrix for roll
Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)]; % Rotation matrix for pitch
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];        % Rotation matrix for yaw

T = Rz*Ry*Rx; 


%% Load measurements
% BikeData_Yixiao
csv_name = 'BikeData_20230222-205544.csv';   data_range = [7, 34.1902]; %  tstvagain v24 3 seg step0
CSV_With_EXCEPTION = 1;

if CSV_With_EXCEPTION == 1
    data_Yi = readtable(csv_name,'headerlines',1); 
    data1_len = size(data_Yi,1);
    opts = detectImportOptions(csv_name);
    opts.DataLines = [4 data1_len]; % remove the last row, as it would be the termination flag
    data_Yi = readtable(csv_name,opts);
    fid = fopen(csv_name);
    csv_Data = textscan(fid, '%[^\n]', data1_len+2, 'HeaderLines', 0);
    csv_Data = csv_Data{1};
    ExitFlag = split(csv_Data{data1_len+1}, ',');
    ExitData = split(csv_Data{data1_len+2}, ',');
     
else
    data_Yi = readtable(csv_name,'headerlines',1);
end

% BikeData from simulation
data_sim = readtable('bikedata_simulink_simulation.csv');
data_sim_states = readtable('bikedata_simulation_bikestates.csv');

% BikeData from labview
data_lab = readtable('data.csv');

%% Load trajectory

Table_traj = readtable('trajectorymat.csv');
Table_traj(1,:) = [];

%% Select right measurements and input AND initialize
% Converting GPS data to X and Y position
% Setting X and Y to zero at first long/lat datapoint
if select == 0
% Yixiao data
longitude0 = deg2rad(data_Yi.longitude(2));
latitude0 = deg2rad(data_Yi.latitude(2));
Earth_rad = 6371000.0;

X = Earth_rad * (deg2rad(data_Yi.longitude) - longitude0) * cos(latitude0);
Y = Earth_rad * (deg2rad(data_Yi.latitude) - latitude0);

ay = data_Yi.ay;
omega_x = data_Yi.RollRate;
omega_z = data_Yi.gy;
delta_enc = data_Yi.SteeringAngle - data_Yi.steering_offset;
v_enc = data_Yi.FilteredVelocity;

% Prepare measurement data for the offline kalman
measurementsGPS = [data_Yi.GPS_timestamp X Y];
measurementsGPS(1,:) = [];
measurements = [data_Yi.Time ay omega_x omega_z delta_enc v_enc];
measurements(1,:) = [];
steer_rate = [data_Yi.Time data_Yi.SteeringAngle];
steer_rate(1,:) = [];

% Initial Roll
        initial_state.roll = deg2rad(0);
        initial_state.roll_rate = deg2rad(0);
% Initial Steering
        initial_state.steering = deg2rad(0);
% Initial Pose (X,Y,theta)
        initial_state.x = Table_traj.Var1(1);
        initial_state.y = Table_traj.Var2(1);
        initial_state.heading = Table_traj.Var3(1);
        initial_pose = [initial_state.x; initial_state.y; initial_state.heading];
        initial_state_estimate = initial_state;
end 

if select == 1
% Simulink simulation data    
measurementsGPS = [data_sim.Time data_sim.GPS_X data_sim.GPS_Y];
measurementsGPS(1,:) = [];
measurements = [data_sim.Time data_sim.a_y data_sim.w_x data_sim.w_z data_sim.Delta_enc data_sim.Velocity];
measurements(1,:) = [];
steer_rate = [data_sim.Time data_sim.Steerrate];
steer_rate(1,:) = [];

% Initial Roll
        initial_state.roll = deg2rad(0);
        initial_state.roll_rate = deg2rad(0);
% Initial Steering
        initial_state.steering = deg2rad(0);
% Initial Pose (X,Y,theta)
        initial_state.x = data_sim_states.X(1);
        initial_state.y = data_sim_states.Y(1);
        initial_state.heading = deg2rad(data_sim_states.Psi(1));
        initial_pose = [initial_state.x; initial_state.y; initial_state.heading];
        initial_state_estimate = initial_state;

end

if select == 2

% [data_lab_,data_lab_idx,data_lab_ic1] = unique(data_lab.StateEstimateX_m_,'stable');
% data_lab = data_lab(data_lab_idx,:);

reset_traj = find(data_lab.ResetTraj==1,1,'last');
data_lab(1:reset_traj,:) = [];

% Labview data
gps_init = find(data_lab.flag>gps_delay, 1 );
longitude0 = deg2rad(0);
latitude0 = deg2rad(0);
Earth_rad = 6371000.0;

X = Earth_rad * (data_lab.LongGPS_deg_ - longitude0) * cos(latitude0);
Y = Earth_rad * (data_lab.LatGPS_deg_ - latitude0);

data_lab.Time = (data_lab.Time_ms_-data_lab.Time_ms_(1))*0.001;

ay = -data_lab.AccelerometerY_rad_s_2_;
omega_x = data_lab.GyroscopeX_rad_s_;
omega_z = data_lab.GyroscopeZ_rad_s_;
delta_enc = data_lab.SteeringAngleEncoder_rad_;
v_enc = data_lab.SpeedVESC_rad_s_*bike_params.r_wheel;

% Prepare measurement data for the offline kalman
measurementsGPS = [data_lab.Time X Y];
measurementsGPS(1:gps_init,:) = [];
measurements = [data_lab.Time ay omega_x omega_z delta_enc v_enc];
measurements(1,:) = [];
steer_rate = [data_lab.Time data_lab.SteerrateInput_rad_s_];
steer_rate(1,:) = [];
gpsflag = [data_lab.Time data_lab.flag];

Table_traj.Var1 = Table_traj.Var1+X(1);
Table_traj.Var2 = Table_traj.Var2+Y(1);

% Initial Roll
        initial_state.roll = deg2rad(0);
        initial_state.roll_rate = deg2rad(0);
% Initial Steering
        initial_state.steering = deg2rad(0);
% Initial Pose (X,Y,theta)
        initial_state.x = Table_traj.Var1(1);
        initial_state.y = Table_traj.Var2(1);
        initial_state.heading = Table_traj.Var3(1);
        initial_pose = [initial_state.x; initial_state.y; initial_state.heading];
        initial_state_estimate = initial_state;

sample_diff = diff(measurements(:,1));

end


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
B = [0 0 0 0 ((lr*v)/(h*(lr+lf))) 1 0]';

% Including GPS
C1 = [1 0 0 0 0 0 0;
     0 1 0 0 0 0 0;
     0 0 0 -g+((-h_imu*g)/h) 0 (-h_imu*(h*v^2-(g*lr*c)))*sin(lambda)/((lr+lf)*h^2) + (v^2)*sin(lambda)/(lr+lf) 0;
     0 0 0 0 1 0 0;
     0 0 0 0 0 (v)*sin(lambda)/(lr+lf) 0;
     0 0 0 0 0 1 0;
     0 0 0 0 0 0 1];

D1 = [0 0 (-h_imu*lr*v)/((lr+lf)*h) 0 0 0 0]';

% Excluding GPS
C2 = [(-g+((-h_imu*g)/h)) 0 (-h_imu*(h*v^2-(g*lr*c)))*sin(lambda)/((lr+lf)*h^2)+(v^2)*sin(lambda)/(lr+lf) 0;
      0 1 0 0;
      0 0 (v)*sin(lambda)/(lr+lf) 0;
      0 0 1 0;
      0 0 0 1];

D2 = [(-h_imu*lr*v)/((lr+lf)*h) 0 0 0 0]';

% Discretize the model
A_d = (eye(size(A))+Ts*A);
B_d = Ts*B;

% Q and R matrix
Q = eye(7);
R = eye(7);

% Compute Kalman Gain
    % including GPS
    [P1,Kalman_gain1,eig] = idare(A_d',C1',Q,R,[],[]);
    eig1 = abs(eig);
    Kalman_gain1 = Kalman_gain1';
    Ts_GPS = 0.1; % sampling rate of the GPS
    counter = (Ts_GPS / Ts) - 1 ; % Upper limit of the counter for activating the flag

% Polish the kalman gain (values <10-5 are set to zero)
for i = 1:size(Kalman_gain1,1)
    for j = 1:size(Kalman_gain1,2)
        if abs(Kalman_gain1(i,j)) < 10^-5
            Kalman_gain1(i,j) = 0;
        end
    end
end 

% Kalman_gain excluding GPS
Kalman_gain2 = Kalman_gain1(4:7,3:7);

 %% Start the Simulation

sim_time = data_lab.Time(end);
tic
try Results = sim(model);
    catch error_details %note: the model has runned for one time here
end
toc

%% Comparing different Q R

% 
%             Q =[1 0 0 0 0 0 0;
%               0 1 0 0 0 0 0;
%               0 0 1 0 0 0 0;
%               0 0 0 5e-5 0 0 0;
%               0 0 0 0 1e-3 0 0;
%               0 0 0 0 0 10 0;
%               0 0 0 0 0 0 0.01];
%             R =[1 0 0 0 0 0 0;
%               0 1 0 0 0 0 0;
%               0 0 2.073153 0 0 0 0;
%               0 0 0 1.66e-6 0 0 0;
%               0 0 0 0 0.0167315 0 0;
%               0 0 0 0 0 7.117e-5 0;
%               0 0 0 0 0 0 32.499];

% Q = eye(7);
% R = eye(7);

% Q and R matrix
% Parameters of Q
Q_GPS = 0.1;
Q_Psi = 0.1;
Q_roll = 1e-9;
Q_rollrate = 5;
Q_delta = 10;
Q_v = 0.5;
Qscale = 1;
Q =Qscale* [Q_GPS 0 0 0 0 0 0;
              0 Q_GPS 0 0 0 0 0;
              0 0 Q_Psi 0 0 0 0;
              0 0 0 Q_roll 0 0 0;
              0 0 0 0 Q_rollrate 0 0;
              0 0 0 0 0 Q_delta 0;
              0 0 0 0 0 0 Q_v];

% Parameters of R
R_GPS = 1.567682871320335;
R_ay = 0.256431376435930;
R_wx = 3.941639024088922e-12;
R_wz = 0.023363599865703;
R_delta = 4.175280633723090e-05;
R_v = 0.1;
Rscale = 1;
R =Rscale* [R_GPS 0 0 0 0 0 0;
              0 R_GPS 0 0 0 0 0;
              0 0 R_ay 0 0 0 0;
              0 0 0 R_wx 0 0 0;
              0 0 0 0 R_wz 0 0;
              0 0 0 0 0 R_delta 0;
              0 0 0 0 0 0 R_v];


% Compute Kalman Gain
    % including GPS
    [P1,Kalman_gain1,eig_K] = idare(A_d',C1',Q,R,[],[]);
    eig1_K = abs(eig_K);
    Kalman_gain1 = Kalman_gain1';
    Ts_GPS = 0.1; % sampling rate of the GPS
    counter = (Ts_GPS / Ts) - 1 ; % Upper limit of the counter for activating the flag 
%      Kalman_gain1(:,4) = 0;
%      Kalman_gain1(:,3) = 0;
%      Kalman_gain1(:,5) = 0;
%      Kalman_gain1(:,6) = 0;

% Polish the kalman gain (values <10-5 are set to zero)
for i = 1:size(Kalman_gain1,1)
    for j = 1:size(Kalman_gain1,2)
        if abs(Kalman_gain1(i,j)) < 10^-5
            Kalman_gain1(i,j) = 0;
        end
    end
end 

% Kalman_gain excluding GPS
Kalman_gain2 = Kalman_gain1(4:7,3:7);

%save matrices
matrixmat = [A_d; B_d'; C1; D1';Kalman_gain1];

filename_matrix = 'matrixmat.csv'; % Specify the filename
csvwrite(filename_matrix, matrixmat); % Write the matrix to the CSV file

tic
try Results2 = sim(model);
    catch error_details %note: the model has runned for one time here
end
toc

%% Plot results
close all

%GPS correction Yixiao
alp = deg2rad(0);
GPSXY = [data_Yi.x_estimated data_Yi.y_estimated];
RotGPS = [cos(alp) sin(alp); -sin(alp) cos(alp)];
GPS_estimated_cor = GPSXY*RotGPS;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Yixiao's measurements
if select == 0
fig = figure();
plot(Results.sim_Kalman.Data(:,1), Results.sim_Kalman.Data(:,2))
hold on
plot(GPS_estimated_cor(:,1),GPS_estimated_cor(:,2))
plot(measurementsGPS(:,2),measurementsGPS(:,3))
xlabel('X position (m)')
ylabel('Y position (m)')
grid on
legend('offline Kalman estimation', 'Yixiao estimation','GPS measurements')
title('Comparison with Yixiao measurement data')

fig = figure();
subplot(421)
plot(Results.sim_Kalman.Time,Results.sim_Kalman.Data(:,1))
hold on
plot(Results2.sim_Kalman.Time,Results2.sim_Kalman.Data(:,1))
plot(data_Yi.Time, GPS_estimated_cor(:,1))
plot(measurementsGPS(:,1),measurementsGPS(:,2))
xlabel('Time (s)')
ylabel('X position (m)')
grid on
legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation','GPS measurements')
title('Comparison with Yixiao measurement data')

subplot(423)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,2))
hold on
plot(Results2.sim_Kalman.Time, Results2.sim_Kalman.Data(:,2))
plot(data_Yi.Time, GPS_estimated_cor(:,2))
plot(measurementsGPS(:,1),measurementsGPS(:,3))
xlabel('Time (s)')
ylabel('Y position (m)')
grid on
legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation','GPS measurements')

subplot(425)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,3))
hold on
plot(Results2.sim_Kalman.Time, Results2.sim_Kalman.Data(:,3))
plot(data_Yi.Time, data_Yi.yaw_estimated+alp)
xlabel('Time (s)')
ylabel('heading (rad)')
grid on
legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation')

subplot(422)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,4))
hold on
plot(Results2.sim_Kalman.Time, Results2.sim_Kalman.Data(:,4))
plot(data_Yi.Time,data_Yi.Roll)
plot(Results2.integration_rollrate.Time, Results2.integration_rollrate.Data)
xlabel('Time (s)')
ylabel('Roll (rad)')
grid on
legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation','Integration Rollrate')
title('Comparison with Yixiao measurement data')

subplot(424)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,5))
hold on
plot(Results2.sim_Kalman.Time, Results2.sim_Kalman.Data(:,5))
plot(data_Yi.Time, data_Yi.RollRate)
xlabel('Time (s)')
ylabel('Roll Rate (rad/s)')
grid on
legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation')

subplot(426)
plot(Results.sim_Kalman.Time,Results.sim_Kalman.Data(:,6))
hold on
plot(Results2.sim_Kalman.Time,Results2.sim_Kalman.Data(:,6))
plot(data_Yi.Time,data_Yi.SteeringAngle - data_Yi.steering_offset)
xlabel('Time (s)')
ylabel('Steering Angle (rad)')
grid on
legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation')

% subplot(427)
% plot(Results.y_hat.Time,Results.y_hat.Data)
% hold on
% plot(Results2.integration_steerrate)
% xlabel('Time (s)')
% ylabel('velocity (m/s)')
% grid on
% legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation')

subplot(428)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,7))
hold on
plot(Results2.sim_Kalman.Time, Results2.sim_Kalman.Data(:,7))
plot(data_Yi.Time, data_Yi.v_estimated)
xlabel('Time (s)')
ylabel('velocity (m/s)')
grid on
legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation')

% figure
% subplot(221)
% plot(Results.y_hat.Time,Results.y_hat.Data(:,1))
% hold on
% plot(measurements(:,1),measurements(:,2))
% xlabel('Time (s)')
% ylabel(' (m/s^2)')
% title('a_y')
% grid on
% legend('prediction','meas')
% subplot(222)
% plot(Results.y_hat.Time,Results.y_hat.Data(:,2))
% hold on
% plot(measurements(:,1),measurements(:,3))
% xlabel('Time (s)')
% ylabel(' (rad/s)')
% title('w_x')
% grid on
% legend('prediction','meas')
% subplot(223)
% plot(Results.y_hat.Time,Results.y_hat.Data(:,3))
% hold on
% plot(measurements(:,1),measurements(:,4))
% xlabel('Time (s)')
% ylabel(' (rad/s)')
% title('w_z')
% grid on
% legend('prediction','meas')
% subplot(224)
% plot(Results.y_hat.Time,Results.y_hat.Data(:,4))
% hold on
% plot(measurements(:,1),measurements(:,5))
% xlabel('Time (s)')
% ylabel(' (rad)')
% title('delta_enc')
% grid on
% legend('prediction','meas')

% figure
% plot(Results.sim_Kalman.Time,Results.sim_Kalman.Data(:,6))
% hold on
% plot(Results2.sim_Kalman.Time,Results2.sim_Kalman.Data(:,6))
% plot(data1.Time,data1.SteeringAngle - data1.steering_offset)
% plot(data1.Time,data1.SteeringAngle)
% plot(data1.Time,steer_rate_calc)
% xlabel('Time (s)')
% ylabel('Steering Angle (rad)')
% grid on
% legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation','Measured steering angle','Diff steerangle')
end

if select == 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%simulink simulation data
fig = figure()
plot(Results2.sim_Kalman.Data(:,1), Results2.sim_Kalman.Data(:,2))
hold on
plot(data_sim_states.X,data_sim_states.Y)
plot(data_sim.GPS_X,data_sim.GPS_Y)
xlabel('X position (m)')
ylabel('Y position (m)')
grid on
legend('offline Kalman estimation', 'True state','GPS measurements')
title('Comparison with simulation measurement data')

fig = figure()
subplot(421)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,1))
hold on
plot(data_sim_states.Time, data_sim_states.X)
plot(data_sim.Time,data_sim.GPS_X)
xlabel('Time (s)')
ylabel('X position (m)')
grid on
legend('offline Kalman estimation', 'True state','GPS measurements')
title('Comparison with simulation measurement data')

subplot(423)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,2))
hold on
plot(data_sim_states.Time, data_sim_states.Y)
plot(data_sim.Time,data_sim.GPS_Y)
xlabel('Time (s)')
ylabel('Y position (m)')
grid on
legend('offline Kalman estimation', 'True state','GPS measurements')

subplot(425)
plot(Results.sim_Kalman.Time, rad2deg(Results.sim_Kalman.Data(:,3)))
hold on
plot(data_sim_states.Time, rad2deg(data_sim_states.Psi))
xlabel('Time (s)')
ylabel('heading (deg)')
grid on
legend('offline Kalman estimation', 'True state')

subplot(422)
plot(Results.sim_Kalman.Time, rad2deg(Results.sim_Kalman.Data(:,4)))
hold on
plot(data_sim_states.Time, rad2deg(data_sim_states.Roll))
xlabel('Time (s)')
ylabel('Roll (deg)')
grid on
legend('offline Kalman estimation', 'True state')
title('Comparison with simulation measurement data')

subplot(424)
plot(Results.sim_Kalman.Time, rad2deg(Results.sim_Kalman.Data(:,5)))
hold on
plot(data_sim_states.Time, rad2deg(data_sim_states.Rollrate))
xlabel('Time (s)')
ylabel('Roll Rate (deg/s)')
grid on
legend('offline Kalman estimation', 'True state')

subplot(426)
plot(Results.sim_Kalman.Time, rad2deg(Results.sim_Kalman.Data(:,6)))
hold on
plot(data_sim_states.Time, rad2deg(data_sim_states.Delta))
xlabel('Time (s)')
ylabel('Steering Angle (deg)')
grid on
legend('offline Kalman estimation', 'True state')

subplot(428)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,7))
hold on
plot(data_sim_states.Time, data_sim_states.Velocity)
plot(data_sim.Time, data_sim.Velocity)
xlabel('Time (s)')
ylabel('Velocity (m/s)')
grid on
legend('offline Kalman estimation', 'True state', 'Measured')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%labview data
if select == 2
fig = figure();
% plot(Results2.sim_Kalman.Data(:,1)-measurementsGPS(1,2), Results2.sim_Kalman.Data(:,2)-measurementsGPS(1,3))
% hold on
% plot(data_lab.StateEstimateX_m_ -measurementsGPS(1,2),data_lab.StateEstimateY_m_-measurementsGPS(1,3))
plot(measurementsGPS(:,2),measurementsGPS(:,3))
hold on
plot(Table_traj.Var1(:),Table_traj.Var2(:))
xlabel('X position (m)')
ylabel('Y position (m)')
axis equal
grid on
legend('offline Kalman estimation', 'online estimation', 'GPS measurements','trajectory')
title('Comparison with Kalman estimator on bike')

figure()
scatter(1:901,sample_diff)


fig = figure();
subplot(421)
plot(Results2.sim_Kalman.Time,Results2.sim_Kalman.Data(:,1)-measurementsGPS(1,2))
hold on
plot(data_lab.Time, data_lab.StateEstimateX_m_-measurementsGPS(1,2))
plot(measurementsGPS(:,1),measurementsGPS(:,2)-measurementsGPS(1,2))
xlabel('Time (s)')
ylabel('X position (m)')
grid on
legend('offline Kalman estimation Tuned R', 'Online estimation','GPS measurements')
title('Comparison with Kalman estimator on bike')

subplot(423)
plot(Results2.sim_Kalman.Time, Results2.sim_Kalman.Data(:,2)-measurementsGPS(1,3))
hold on
plot(data_lab.Time, data_lab.StateEstimateY_m_-measurementsGPS(1,3))
plot(measurementsGPS(:,1),measurementsGPS(:,3)-measurementsGPS(1,3))
xlabel('Time (s)')
ylabel('Y position (m)')
grid on
legend('offline Kalman estimation Tuned R', 'Online estimation','GPS measurements')

subplot(425)
plot(Results2.sim_Kalman.Time, rad2deg(Results2.sim_Kalman.Data(:,3)))
hold on
plot(data_lab.Time, rad2deg(wrapToPi(data_lab.StateEstimatePsi_rad_)))
xlabel('Time (s)')
ylabel('heading (deg)')
grid on
legend('offline Kalman estimation Tuned R', 'Online estimation')

subplot(422)
plot(Results2.sim_Kalman.Time, rad2deg(Results2.sim_Kalman.Data(:,4)))
hold on
plot(data_lab.Time,rad2deg(data_lab.StateEstimateRoll_rad_))
plot(Results2.integration_rollrate.Time, rad2deg(Results2.integration_rollrate.Data))
% plot(data_lab.Time,rad2deg(data_lab.Rollref))
xlabel('Time (s)')
ylabel('Roll (deg)')
ylim([-1 50])
grid on
legend('offline Kalman estimation Tuned R', 'Online estimation','Integration Rollrate','rollref')
title('Comparison with Kalman estimator on bike')

subplot(424)
plot(Results2.sim_Kalman.Time, rad2deg(Results2.sim_Kalman.Data(:,5)))
hold on
plot(data_lab.Time, rad2deg(data_lab.StateEstimateRollrate_rad_s_))
plot(data_lab.Time, rad2deg(data_lab.GyroscopeX_rad_s_))
xlabel('Time (s)')
ylabel('Roll Rate (deg/s)')
grid on
legend('offline Kalman estimation Tuned R', 'Online estimation', 'Measurement')

subplot(426)
plot(Results2.sim_Kalman.Time,rad2deg(Results2.sim_Kalman.Data(:,6)))
hold on
plot(data_lab.Time,rad2deg(data_lab.StateEstimateDelta_rad_))
plot(data_lab.Time,rad2deg(data_lab.SteeringAngleEncoder_rad_))
xlabel('Time (s)')
ylabel('Steering Angle (deg)')
grid on
legend('offline Kalman estimation Tuned R', 'Online estimation', 'Measurement')

subplot(428)
plot(Results2.sim_Kalman.Time, Results2.sim_Kalman.Data(:,7))
hold on
plot(data_lab.Time, data_lab.StateEstimateVelocity_m_s_)
plot(data_lab.Time, v_enc)
xlabel('Time (s)')
ylabel('velocity (m/s)')
grid on
legend('offline Kalman estimation Tuned R', 'Online estimation','Measurement')

figure
subplot(221)
plot(Results2.y_hat.Time,Results2.y_hat.Data(:,1))
hold on
plot(measurements(:,1),measurements(:,2))
xlabel('Time (s)')
ylabel(' (m/s^2)')
title('a_y')
grid on
legend('prediction','meas')

subplot(222)
plot(Results2.y_hat.Time,rad2deg(Results2.y_hat.Data(:,2)))
hold on
plot(measurements(:,1),rad2deg(measurements(:,3)))
xlabel('Time (s)')
ylabel(' (deg/s)')
title('w_x')
grid on
legend('prediction','meas')

subplot(223)
plot(Results2.y_hat.Time,rad2deg(Results2.y_hat.Data(:,3)))
hold on
plot(measurements(:,1),rad2deg(measurements(:,4)))
xlabel('Time (s)')
ylabel(' (deg/s)')
title('w_z')
grid on
legend('prediction','meas')

subplot(224)
plot(Results2.y_hat.Time,Results2.y_hat.Data(:,4))
hold on
plot(measurements(:,1),measurements(:,6))
xlabel('Time (s)')
ylabel(' (m/s)')
title('velocity')
grid on
legend('prediction','meas')
end

figure()
subplot(221)
plot(data_lab.Time,rad2deg(data_lab.Rollref))
hold on
plot(data_lab.Time,rad2deg(data_lab.DpsirefContribution)+rad2deg(data_lab.HeadingContribution)+rad2deg(data_lab.LateralContribution))
grid on
xlabel('Time (s)')
ylabel('Angle(deg)')
title('roll vs delta references')
legend('Rollreference','deltaref')


subplot(222)
plot(data_lab.Time,rad2deg(data_lab.LateralContribution))
hold on
plot(data_lab.Time,rad2deg(data_lab.HeadingContribution))
plot(data_lab.Time,rad2deg(data_lab.DpsirefContribution))
xlabel('Time (s)')
ylabel('Angle(deg)')
title('delta contributions')
grid on
legend('lateral','heading','dpsi')

subplot(223)
plot(data_lab.Time,data_lab.Error1)
xlabel('Time (s)')
ylabel('error1 (m)')
grid on
title('Lateral error')

subplot(224)
plot(data_lab.Time,rad2deg(data_lab.Error2))
xlabel('Time (s)')
ylabel('error2 [Deg]')
grid on
title('Heading error')

figure()
subplot(211)
plot(data_lab.Time,rad2deg(data_lab.SteerrateInput_rad_s_))
hold on
plot(data_lab.Time,rad2deg(data_lab.SteeringAngleEncoder_rad_))
xlabel('Time (s)')
ylabel('angle [Deg]')
grid on
title('Steerrate from balancing controller')

subplot(212) 
plot(data_lab.Time,rad2deg(data_lab.Input))
title('Input balancing controller (generated rollref)')
xlabel('Time (s)')
ylabel('angle [Deg]')

%% Utility Functions

function Parameters = LoadBikeParameters(bike)

    if strcmp(bike,'red')
        % Red bike
 % real parameters and positions on bike
        Parameters.inertia_front = 0.245;  %[kg.m^2] inertia of the front wheel
        Parameters.r_wheel = 0.311;        % radius of the wheel
        Parameters.h = 0.2085 + Parameters.r_wheel;   % height of center of mass [m]
        Parameters.lr = 0.4964;             % distance from rear wheel to frame's center of mass [m]
        Parameters.lf = 1.095-0.4964;       % distance from front wheel to frame's center of mass [m]
        Parameters.c = 0.06;               % length between front wheel contact point and the extention of the fork axis [m]
        Parameters.m = 45;                 % Bike mass [kg]
        Parameters.lambda = deg2rad(70);   % angle of the fork axis [deg]
        Parameters.IMU_height = 0.615;      % IMU height [m]
        Parameters.IMU_x = 0.0;           % x Position of the IMU measured from rear wheel (parallel to bike) [m]
        Parameters.IMU_roll = 0;           % Orientation offset in roll (degrees)
        Parameters.IMU_pitch = 0;            % Orientation offset in pitch (degrees)
        Parameters.IMU_yaw = 0;             % Orientation offset in yaw (degrees)
        Parameters.Xgps = 0.0;             % Actual GPS position offset X (measured from middlepoint position parralel to bike heading) [m]
        Parameters.Ygps = 0.0;             % Actual GPS position offset Y (measured from middlepoint position perpendicular to bike heading) [m]
        Parameters.Hgps = 0.0;             % GPS position height (measured from the groud to the GPS)   [m]
        
        % Parameters in the model
        Parameters.inertia_front_mod = 0.245;  %[kg.m^2] inertia of the front wheel
        Parameters.r_wheel_mod = 0.311;        % radius of the wheel
        Parameters.h_mod = 0.2085 + Parameters.r_wheel_mod;   % height of center of mass [m]
        Parameters.lr_mod = 0.4964;             % distance from rear wheel to frame's center of mass [m]
        Parameters.lf_mod = 1.095-0.4964;       % distance from front wheel to frame's center of mass [m]
        Parameters.c_mod = 0.06;                % length between front wheel contact point and the extention of the fork axis [m]
        Parameters.m_mod = 45;                  % Bike mas [kg]
        Parameters.lambda_mod = deg2rad(70);    % angle of the fork axis [deg]
        Parameters.IMU_height_mod = 0.615;      % IMU height [m]
        Parameters.IMU_x_mod = 0.0;           % x Position of the IMU measured from rear wheel (parallel to bike) [m]
        Parameters.IMU_roll_mod = 0;           % Orientation offset in roll (degrees)
        Parameters.IMU_pitch_mod = 0;            % Orientation offset in pitch (degrees)
        Parameters.IMU_yaw_mod = 0;             % Orientation offset in yaw (degrees)
        Parameters.Xgps_mod = 0.0;              % GPS position X accoarding to the model [m] 
        Parameters.Ygps_mod = 0.0;              % GPS position Y accoarding to the model [m]
        Parameters.Hgps_mod = 0.0;              % GPS position height accoarding to the model   [m]
        

        %
        Parameters.uneven_mass = false;    % true = use uneven mass distribution in bike model ; 0 = do not use

    elseif strcmp(bike,'black')
        % Black bike

        % Parameters on bike (actual measured)
        Parameters.inertia_front = 0.245;  %[kg.m^2] inertia of the front wheel
        Parameters.h = 0.534 ;             % height of center of mass [m]
        Parameters.lr = 0.4964;             % distance from rear wheel to frame's center of mass [m]
        Parameters.lf = 1.095-0.4964;       % distance from front wheel to frame's center of mass [m]
        Parameters.c = 0.06;               % length between front wheel contact point and the extention of the fork axis [m]
        Parameters.m = 31.3;               % Bike mass [kg]
        Parameters.lambda = deg2rad(66);   % angle of the fork axis [deg]  !!! TO BE MEASURED
        Parameters.IMU_height = 0.215;     % IMU height [m]
        Parameters.IMU_x = 0.0;           % x Position of the IMU measured from rear wheel (parallel to bike) [m]
        Parameters.IMU_roll = 0;           % Orientation offset in roll (degrees)
        Parameters.IMU_pitch = 0;            % Orientation offset in pitch (degrees)
        Parameters.IMU_yaw = 0;             % Orientation offset in yaw (degrees)
        Parameters.Xgps = 0.0;             % Actual GPS position offset X (measured from middlepoint position parralel to bike heading) [m]
        Parameters.Ygps = 0.0;             % Actual GPS position offset Y (measured from middlepoint position perpendicular to bike heading) [m]
        Parameters.Hgps = 0.0;             % GPS position height (measured from the groud to the GPS)   [m]
        
        % Parameters in model 
        Parameters.inertia_front_mod = 0.245;  %[kg.m^2] inertia of the front wheel
        Parameters.h_mod = 0.534 ;             % height of center of mass [m]
        Parameters.lr_mod = 0.4964;             % distance from rear wheel to frame's center of mass [m]
        Parameters.lf_mod = 1.095-0.4964;       % distance from front wheel to frame's center of mass [m]
        Parameters.c_mod = 0.06;               % length between front wheel contact point and the extention of the fork axis [m]
        Parameters.m_mod = 31.3;               % Bike mass [kg]
        Parameters.lambda_mod = deg2rad(66);   % angle of the fork axis [deg]  !!! TO BE MEASURED
        Parameters.IMU_height_mod = 0.215;     % IMU height [m]
        Parameters.IMU_x_mod = 0.0;            % x Position of the IMU measured from rear wheel (parallel to bike) [m]
        Parameters.IMU_roll_mod = 0;           % Orientation offset in roll (degrees)
        Parameters.IMU_pitch_mod = 0;            % Orientation offset in pitch (degrees)
        Parameters.IMU_yaw_mod = 0;             % Orientation offset in yaw (degrees)
        Parameters.Xgps_mod = 0.0;             % Actual GPS position offset X (measured from middlepoint position parralel to bike heading) [m]
        Parameters.Ygps_mod = 0.0;             % Actual GPS position offset Y (measured from middlepoint position perpendicular to bike heading) [m]
        Parameters.Hgps_mod = 0.0;             % GPS position height (measured from the groud to the GPS)   [m]
       
        %
        Parameters.uneven_mass = false;    % true = use uneven mass distribution in bike model ; 0 = do not use
         
    end
end
