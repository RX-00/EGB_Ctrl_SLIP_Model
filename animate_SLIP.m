function [] = animate_SLIP(q, s, t)
%ANIMATE_SLIP animate the SLIP model
%   Take in the state vector and forces to model the SLIP model

    figure(100);                % figure number
    cla                         % clear axes
    xlim([-10.5, 10.5]);        % x-axis limit shown in animation
    ylim([-0.3, 2]);            % y-axis limit shown in animation
    grid on;                    % turn on the grid in the animation
    axis equal;                 % axes are equal to create a square
    title('SLIP Animation');
    xlabel('distance');
    ylabel('height');
    
    % used to calculate the leg length
    x = q(1, 1);                         % initial x body position
    y = q(1, 3);                         % initial y body position
    xtd = q(1, 5);                       % initial x touchdown of the foot
    ytd = s.gnd_height;                  % initial y touchdown of the foot
    d = sqrt((x - xtd)^2 + (y - ytd)^2); % initial length of the spring leg
    
    % Visual patches
    ground_patch = patch([-100, -100, 100, 100], [s.gnd_height, -10, -10, s.gnd_height], [0.5, 0.5, 0.5]); % ground patch
    
    body_patch = patch(q(1, 1) + 0.1 * sin(0: 0.1: 2 * pi), q(1, 3) + 0.1 * cos(0: 0.1: 2 * pi), [70, 216, 226]./255); % body patch of the mass of the SLIP model
    
    startTheta = -(q(1,7) - pi / 2); % initial theta
    if (q(1, 2) > 0) % detects which way the SLIP model is traveling (left or right) -> (negative x-dir or positive x-dir)
        startTheta = -startTheta; % reverse the touchdown angle if going positively
    end
    
    % draw the leg patch for the spring leg
    leg_patch = patch(q(1, 1) + [0.01,0.01,-0.01,-0.01] * cos(startTheta) + s.d0 * [0,1,1,0] * sin(startTheta),...);
         q(1, 3) + [0.01,0.01,-0.01,-0.01] * sin(startTheta) + s.d0 * [0,-1,-1,0] * cos(startTheta), 'k');

    drawnow;
    
    % Loop through the data and update the graphics
    for i = 1:(length(t) - 1) % minus one in order to not animate the last part of the data that changes from stance (fallen) to flight (even though irl that is impossible)
        
        % update the current position of the body patch
        body_patch.Vertices = [q(i, 1) + 0.1 * sin(0: 0.1: 2 * pi); q(i, 3) + 0.1 * cos(0: 0.1: 2 * pi)]';
        
        % used to calculate the leg length
        x = q(i, 1);                         % current x body position
        y = q(i, 3);                         % current y body position
        xtd = q(i, 5);                       % current x touchdown of the foot
        ytd = s.gnd_height;                  % current y touchdown of the foot
        d0 = s.d0;                           % current leg length (just did this since I was too lazy to type s. multiple times for some reason)
        d = sqrt((x - xtd)^2 + (y - ytd)^2); % current length of the spring leg
         
        
        % NOTE: This algorithm was originally for pitch angle of the leg
        % from the body and so in order to use it with your touchdown angle
        % (right side angle of leg touching ground) you need to add
        % pi / 2
        % NOTE: This line uses the pitch angle from if the leg was straight
        % up and down to where the leg actually is
        %leg_patch.Vertices = [q(i, 1) + [0.01,0.01,-0.01,-0.01] * cos(inputTheta) + d * [0,1,1,0] * sin(inputTheta);...);
                       %q(i, 3) + [0.01,0.01,-0.01,-0.01] * sin(inputTheta) + d * [0,-1,-1,0] * cos(inputTheta)]';

                       
        if(q(i, 6) == 1) % animation for leg in stance phase
            inputTheta = asin((xtd - x) / d);
            leg_patch.Vertices = [q(i, 1) + [0.01,0.01,-0.01,-0.01] * cos(inputTheta) + d * [0,1,1,0] * sin(inputTheta);...);
                       q(i, 3) + [0.01,0.01,-0.01,-0.01] * sin(inputTheta) + d * [0,-1,-1,0] * cos(inputTheta)]';
        else % animation for the leg in flight phase

            inputTheta = (q(i, 7) - pi / 2);           
            
            
            if (q(i, 2) > 0)
                inputTheta = -inputTheta;
            end
            
            leg_patch.Vertices = [q(i, 1) + [0.01,0.01,-0.01,-0.01] * cos(inputTheta) + d0 * [0,1,1,0] * sin(inputTheta);...);
                       q(i, 3) + [0.01,0.01,-0.01,-0.01] * sin(inputTheta) + d0 * [0,-1,-1,0] * cos(inputTheta)]';
        end                   
                   
  
        ylim([-0.5, 3]);
        
        % Increment the screen by 0.5 m increments
       % xlim([-1.5, 1.5] + round(q(i, 1) * 2) / 2);
       
        % other way of scrolling
        axis([2*round(0.5*q(i,1))-2,2*round(0.5*q(i,1))+2,-0.5,2])
        drawnow;
        %pause(0.1);
    end
end

