function [q, t_max] = LSPBTrajectory(si, v0, sf, vf, ti, tf, timestep)
    n_joints = length(si);
    t_max = tf; % Start with the given final time
    
    % Find the maximum time required for all joints
    for i = 1:n_joints
        [~, t_i] = LSPBTrajectory_SingleJoint(si(i), v0(i), sf(i), vf(i), ti, tf, timestep);
        if t_i > t_max
            t_max = t_i;
        end
    end
    
    % Generate synchronized trajectories for all joints
    q = zeros(t_max/timestep + 1, n_joints);
    for i = 1:n_joints
        q(:, i) = LSPBTrajectory_SingleJoint(si(i), v0(i), sf(i), vf(i), ti, t_max, timestep);
    end
end

function [q, t_max] = LSPBTrajectory_SingleJoint(si, v0, sf, vf, ti, tf, timestep)
    % Calculate the blend time (tb) as 20% of the total time (tf)
    tb = 0.2 * tf;
    
    % Calculate the constant velocity (v) during the linear segment
    v = (sf - si) / (tf - tb);
    
    % Calculate the acceleration (alpha) during the parabolic segments
    alpha = v / tb;
    
    % Generate the time vector from initial time (ti) to final time (tf) with the given timestep
    time = (ti:timestep:tf)';
    
    % Initialize the position vector (q) with zeros, the same length as the time vector
    q = zeros(length(time), 1);
    
    % Loop through each time step to calculate the corresponding position
    for i = 1:length(time)
        t = time(i); % Current time step
        
        % If within the initial parabolic segment
        if ((ti <= t) && (t <= ti + tb))
            q(i) = si + alpha / 2 * t^2;
        
        % If within the linear segment
        elseif ((ti + tb <= t) && (t <= ti + tf - tb))
            q(i) = (si + sf - v * tf) / 2 + v * t;
        
        % If within the final parabolic segment
        elseif ((ti + tf - tb <= t) && (t <= ti + tf))
            q(i) = sf - alpha / 2 * tf^2 - alpha / 2 * t^2 + alpha * (tf * t);
        
        % If time is outside the specified interval (should not happen)
        else
            q(i) = 0;
        end
    end
    
    % Return the maximum time (tf) as t_max
    t_max = tf;
end
