function [initialization_new] = referenceTest(traj,Th,Ts,initialization)
    % Check if the trajectory is valid
    [n,m] = size(traj);

    % Minimun number of reference points
    if n<(Th/Ts)
        disp('Warning: Not enough reference points');
        % Correction needed!!!?
    end
    
    % Initialization errors
    initialization_new = initialization;
    if traj(1,1) ~= initialization(1) || traj(1,2) ~= initialization(2) || traj(1,3) ~= initialization(3)
        disp('Warning: initialization is not correct');
        initialization_new(1) = traj(1,1);
        initialization_new(2) = traj(1,2);
        initialization_new(3) = traj(1,3);
    end
        
    % Density of datapoints
    Min_density_distance = 10;
    index = 0;
    for i = 1:n-1
        dist = sqrt((traj(i,1)-traj(i+1,1))^2+(traj(i,2)-traj(i+1,2))^2);
        if dist > Min_density_distance
            index = [index i];
            % Do you want to correct!!
        end
    end    
    if length(index) ~= 1
        disp('Warning: Some ref points are more than 10m apart from each other, index: ')
        disp(index(2:end))
    end
  
    % Sharp turn
    index = 0;
    for i = 1:n-1
       if abs(traj(i,3)-traj(i+1,3))> pi/3 % We can change it
           index = [index i];
           % issue: what happens if we record the trajectory with GPS and
           % it is quite noisy
       end
    end
    if length(index) ~= 1
        disp('Warning: Too sharp turn during the trajectory, index: ')
        disp(index(2:end))
    end
end
