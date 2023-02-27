function [initialization_new] = referenceTest_v3(traj,initialization)
    % Check if the trajectory is valid
    [n] = size(traj);

    % Minimun number of reference points
    if 1<n-(n/10)
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
    for i = 1:n-1
        dist = sqrt((traj(i,1)-traj(i+1,1))^2+(traj(i,2)-traj(i+1,2))^2);
        if dist > Min_density_distance
            disp('Warning: Too sparse reference')
            % Do you want to correct!!
        end
    end    
  
%     % Sharp turn
%     for i = 1:n-1
%        if abs(traj(i,3)-traj(i+1,3))> pi/3 % We can change it
%            disp('Warning: Too sharp turn during the trajectory')
%            % issue: what happens if we record the trajectory with GPS and
%            % it is quite noisy
%        end
%     end

end
