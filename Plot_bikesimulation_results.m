function [] = Plot_bikesimulation_results(Tnumber, Results, bike_params)

% Trajectory
figure('Name',Tnumber);
subplot(3,2,[1,3,5]);
hold on;
plot3(Results.refX.Data,Results.refY.Data,1:length(Results.refX.Data));
plot3(Results.trueX.Data(:,1),Results.trueY.Data(:,1),Results.trueX.Time(:,1));
plot3(Results.estimatedX.Data(:,1),Results.estimatedY.Data(:,1),Results.estimatedY.Time(:,1));
view(0,90)
legend('Ref','True', 'Estimated');
xlabel('X-dir [m]');
ylabel('Y-dir [m]');
axis equal
grid on;
title('Trajectory');

subplot(3,2,2)
plot(Results.error2.Time,Results.error2.Data) 
xlabel('Time [s]')
ylabel('Distance [m]')
grid on
title('Lateral error')

subplot(3,2,4)
plot(Results.error1.Time,Results.error1.Data)
xlabel('Time {s]')
ylabel('Angle [rad]')
grid on
title('Heading error')

subplot(3,2,6)
hold on
plot(Results.ids.Time,Results.ids.Data)
plot(Results.closest_point.Time,Results.closest_point.Data)
legend( 'Selected point index','New closest detected')
xlabel('Iteration [-]')
ylabel('Index [-]')
grid on
title('Closest point index selection')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name',Tnumber);

% X, Y, Psi
subplot(3,2,1)
hold on;
plot(Results.refX.Time(:,1),Results.refX.Data(:,1));
plot(Results.trueX.Time(:,1),Results.trueX.Data(:,1));
plot(Results.estimatedX.Time(:,1),Results.estimatedX.Data(:,1));
legend('Ref X','True X','Estimated X');
xlabel('Time [t]');
ylabel('Position X [m]');
% ylim([-50 50])
% xlim([-100 100])
grid on;
title('X-coordinate');

subplot(3,2,3);
hold on;
plot(Results.refY.Time(:,1),Results.refY.Data(:,1));                
plot(Results.trueY.Time(:,1),Results.trueY.Data(:,1));              
plot(Results.estimatedY.Time(:,1),Results.estimatedY.Data(:,1));    
legend('Ref Y','True Y','Estimated Y');
xlabel('Time [t]');
ylabel('Position Y [m]');
% ylim([-50 50])
% xlim([-100 100])
grid on;
title('Y-coordinate');

subplot(3,2,5)
hold on;
plot(Results.refPsi.Time(:,1),Results.refPsi.Data(:,1));
plot(Results.truePsi.Time(:,1),Results.truePsi.Data(:,1));
plot(Results.estimatedPsi.Time(:,1),Results.estimatedPsi.Data(:,1));
legend('Ref psi','True psi','Estimated psi');
xlabel('Time [t]');
ylabel('Angle [rad]');
% ylim([-50 50])
% xlim([-100 100])
grid on;
title('Heading');

% Roll and Roll_rate
subplot(3,2,2)
hold on;
plot(Results.refRoll.Time(:,1),Results.refRoll.Data(:,1));
plot(Results.trueRoll.Time(:,1),Results.trueRoll.Data(:,1));
plot(Results.estimatedRoll.Time(:,1),Results.estimatedRoll.Data(:,1));
legend('Roll ref','True Roll','Estimated Roll');
xlabel('Time [t]');
ylabel('Angle [rad]');
ylim([-3 3])
% xlim([-100 100])
grid on;
title('Roll');

subplot(3,2,4)
hold on
plot(Results.trueRoll_rate.Time(:,1),Results.trueRoll_rate.Data(:,1));
plot(Results.estimatedRoll_rate.Time(:,1),Results.estimatedRoll_rate.Data(:,1));
legend('True Roll rate','Estimated Roll rate');
xlabel('Time [t]');
ylabel('Angle rate [rad/s]');
ylim([-3 3])
% xlim([0 0])
grid on;
title('Rollrate');

% Steer angle and rate
subplot(3,2,6)
hold on;
plot(Results.refSteer_angle.Time(:,1),Results.refSteer_angle.Data(:,1));
plot(Results.trueSteer_angle.Time(:,1),Results.trueSteer_angle.Data(:,1)*sin(bike_params.lambda));
plot(Results.estimatedSteer_angle.Time(:,1),Results.estimatedSteer_angle.Data(:,1));
xlabel('Time [t]')
ylabel('Angle [rad]')
yyaxis right
plot(Results.steer_rate.Time(:,1),Results.steer_rate.Data(:,1))
ylabel('Rate [rad/s]')
legend('Ref Steer angle','True Steer angle e','Estimated Steer angle e', 'Steer rate')
% ylim([-3 3])
% xlim([0 0])
grid on
title('Steer angle')



end