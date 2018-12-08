function [position, isterminal, direction] = flightToStance(t, q, s)
% FLIGHTTOSTANCE Detect when flight changes to stance
%   If the y coordinate is equal to the y touchdown (td) value then you
%   know you have hit the ground. given the touchdown angle
    
    ytd = s.d0 * sin(q(7)) + s.gnd_height;            % calculate the touchdown height of the SLIP model
    
    % Check for multiple triggers
    position = [q(3) - ytd, q(3) - s.gnd_height];     % detect current height == touchdown height, q(3) == 0, when these equal zero -> stop integration
    isterminal = [1, 1];               % stop the integration
    
    direction = [-1, ];                % The zero can be approached from negtive direction, or either way
end