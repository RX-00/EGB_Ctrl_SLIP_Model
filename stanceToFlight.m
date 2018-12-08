function [valFs, isterminal, direction] = stanceToFlight(t, q, s)
% STANCETOFLIGHT Detect when stance changes to flight
%   If the spring force is zero then you know you have left the ground,
%   thus transitioning into the flight phase

    d = sqrt((q(1) - q(5))^2 + (q(3) - s.gnd_height)^2);     % calculate the current length of the spring leg
    valFs = [s.k * (s.d0 - d), q(3) - s.gnd_height];         % if the spring force is 0 then you've left the ground, or body height is 0
    isterminal = [1, 1];                      % stops the integration
    
    direction = [-1, ];                       % The zero can be approached from negtive direction, or either way
end