function [xf, theta] = raibertPController(q, s, t)
%RAIBERTCONTROLLER Simple Raibert proportional controller for SLIP monoped
%   This controller is called at lift off (entering flight phase)
%   in order to adjust the leg touchdown angle to maintain
%   the desired forward velocity

    % This is the feedback gain for ft displacement from neutral point
    % measured in m/(m/s), which is just s
    
    % NOTE: for tuning the gain there's not a general "rule" for
    % approaching a number, try random numbers and then tune from the best
    % there
    k = -0.2;          % tuned gain
    xf = 0;            % forward displacement of foot with respect to COM (x of the body of the SLIP model)
    theta = 0;         % calculated theta from the controller
    
    if q(end, 6) == 0 % If the model is in flight phase
        
        theta = pi / 2 + k * (q(end, 2) - s.d_fwrd_vel); % Caculate the new theta to maintain/approach the target forward velocity
        
        xf = cos(theta) / s.d0;                          % Calculate the new forward displacement of the foot (NOTE: THIS IS UNUSED)
        
    end
    
    % If the model is in stance, the controller does not need to do
    % anything (for this SLIP model at least)    
end