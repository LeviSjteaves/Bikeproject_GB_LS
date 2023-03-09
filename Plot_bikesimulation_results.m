function [] = Plot_bikesimulation_results(Tnumber, test_curve,Results, bike_params, traj_plot, Ts)

% Trajectory
figure('Name',Tnumber);
subplot(3,2,[1,3,5]);
hold on;
plot3(test_curve(:,1),test_curve(:,2),1:length(test_curve(:,1)));
plot3(Results.trueX.Data(:,1),Results.trueY.Data(:,1),Results.trueX.Time(:,1));
plot3(Results.estimatedX.Data(:,1),Results.estimatedY.Data(:,1),Results.estimatedY.Time(:,1));
view(0,90)
legend('Ref','True', 'Estimated');
xlabel('X-dir [m]');
ylabel('Y-dir [m]');
% ylim([traj_plot.ymin traj_plot.ymax])
% xlim([traj_plot.xmin traj_plot.xmax])
axis equal
grid on;
title('Trajectory');


subplot(3,2,2)
plot(Results.error1.Time,Results.error1.Data)
xlabel('Time [s]')
ylabel('Distance [m]')
title('Lateral error')

subplot(3,2,4)
plot(Results.error2.Time,rad2deg(Results.error2.Data))
xlabel('Time [s]')
ylabel('Angle [deg]')
grid on
title('Heading error')

subplot(3,2,6)
hold on
plot(Results.ids.Time,Results.ids.Data)
plot(Results.closest_point.Time,Results.closest_point.Data)
plot(Results.closestpoint_heading.Time,Results.closestpoint_heading.Data)
legend( 'Selected point index','New closest detected')
xlabel('Time [s]')
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
plot(Results.refPsi.Time(:,1),rad2deg(Results.refPsi.Data(:,1)));
plot(Results.truePsi.Time(:,1),rad2deg(Results.truePsi.Data(:,1)));
plot(Results.estimatedPsi.Time(:,1),rad2deg(Results.estimatedPsi.Data(:,1)));
legend('Ref psi','True psi','Estimated psi');
xlabel('Time [t]');
ylabel('Angle [Deg]');
% ylim([-50 50])
% xlim([-100 100])
grid on;
title('Heading');

% Roll and Roll_rate
subplot(3,2,2)
hold on;
plot(Results.refRoll.Time(:,1),rad2deg(Results.refRoll.Data(:,1)));
plot(Results.trueRoll.Time(:,1),rad2deg(Results.trueRoll.Data(:,1)));
plot(Results.estimatedRoll.Time(:,1),rad2deg(Results.estimatedRoll.Data(:,1)));
legend('Roll ref','True Roll','Estimated Roll');
xlabel('Time [t]');
ylabel('Angle [Deg]');
% ylim([-3 3])
% xlim([-100 100])
grid on;
title('Roll');

subplot(3,2,4)
hold on
plot(Results.trueRoll_rate.Time(:,1),rad2deg(Results.trueRoll_rate.Data(:,1)));
plot(Results.estimatedRoll_rate.Time(:,1),rad2deg(Results.estimatedRoll_rate.Data(:,1)));
legend('True Roll rate','Estimated Roll rate');
xlabel('Time [t]');
ylabel('Angle rate [Deg/s]');
%ylim([-3 3])
% xlim([0 0])
grid on;
title('Rollrate');

% Steer angle and rate
subplot(3,2,6)
hold on;
plot(Results.refSteer_angle.Time(:,1),rad2deg(Results.refSteer_angle.Data(:,1)));
plot(Results.trueSteer_angle.Time(:,1),rad2deg(Results.trueSteer_angle.Data(:,1)*sin(bike_params.lambda)));
plot(Results.estimatedSteer_angle.Time(:,1),rad2deg(Results.estimatedSteer_angle.Data(:,1)));
xlabel('Time [t]')
ylabel('Angle [Deg]')
yyaxis right
plot(Results.steer_rate.Time(:,1),rad2deg(Results.steer_rate.Data(:,1)))
ylabel('Rate [Deg/s]')
legend('Ref Steer angle','True Steer angle e','Estimated Steer angle e', 'Steer rate')
% ylim([-3 3])
% xlim([0 0])
grid on
title('Steer angle')

figure()
y = fft(Results.steer_rate.Data(:,1));
fs = 1/Ts;
f = (0:length(y)-1)*fs/length(y);
plot(f,abs(y))
xlabel('Frequency (Hz)')
ylabel('Magnitude')
title('Magnitude Steerrate')


end