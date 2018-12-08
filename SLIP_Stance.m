function dy = SLIP_Stance(t, q, s)
% SLIP_Stance attempt at a passive SLIP model by Roy X.
% Find (x, y) of the point mass!
% This is the STANCE phase DYNAMICS
% s stands for the struct of all the parameters
%
% q = [ x, x dot, y, y dot, ft pos, phase, touchdown theta]
% NOTE: changed the y state vector name to q just for clarity
% Thus, in order to define x and y you must do
% x = y(:,1), y = y(:,3)

% These are the equations of motion for the dynamics of stance phase
    
    x = q(1);           % x body position of the point mass of the SLIP
    y = q(3);           % y body position of the point mass of the SLIP
    
    xtd = q(5);         % Touchdown position of the foot along x-axis
    ytd = s.gnd_height; % Touchdown position of the foot along y-axis
  
    % Functions that describe the motion of the SLIP Model's COM
    d = sqrt((x - xtd)^2 + (y - ytd)^2); % Calculate spring leg length
    assert(d ~= 0, 'COM distance from zero must not be zero')
    sinT = (y - ytd)/ d;                 % Calculate sin(theta) value based on lengths, I believe this was a left over from not originally including theta in the state vector, although I am not 100% sure the theta in the state vector is updated enough to use it instead of this, I do not think so currently as it is only changed by the controller or natural dynamics of the system, so theta is not a sweeping record of the angle of the leg, only the angle of attack, aka touchdown angle
    cosT = (x - xtd)/ d;                 % Calculate cos(theta) value based on lengths
    Fs = s.k * (s.d0 - d);               % Force of the spring
    Fy = Fs * sinT;                      % y-component of the force of the spring
    Fx = Fs * cosT;                      % x-component of the force of the spring
    Fyt = Fy - s.m * s.g;                % Total net y-dir Force on the SLIP model
        
    dy(1, 1) = q(2);               % x dot
    dy(2, 1) = (Fx / s.m);         % x double dot
    dy(3, 1) = q(4);               % y dot
    dy(4, 1) = (Fyt / s.m);        % y double dot
    dy(5, 1) = 0;                  % foot position upon touchdown
    dy(6, 1) = 0;                  % what phase you're in, but you don't want to set the value here or else it will be part of the integration
    dy(7, 1) = 0;                  % touchdown theta
end