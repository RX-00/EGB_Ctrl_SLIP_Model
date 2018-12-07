function [xf, theta] = raibertController(q, s, t)
%RAIBERTCONTROLLER Simple Raibert controller for SLIP monoped
%   This controller is called at lift off (entering flight phase)
%   in order to adjust the leg touchdown angle to maintain
%   the desired forward velocity

    % This is the feedback gain for ft displacement from neutral point
    % measured in m/(m/s), which is just s
    k = .05;
    fwrd_vel = q(2);
    xf = q(end, 5);
    theta = s.theta;
    
    if q(end, 6) == 0 % If the model is in flight phase
        
        %-------------------------------------------------------
        % NOTE: This was an attempt to implement the controller
        % directly, as seen from the equations. Didn't work out
        % too well. Look at the P controller version instead.
        %-------------------------------------------------------
        
        % Find the foot placement
        heightOfMatrix = size(q);
        stance_amt = heightOfMatrix(1) - 1; % How much is actually in stance
        Ts = t(stance_amt) - t(1); % Duration of stance phase
        CG_print = fwrd_vel * Ts; % length of CG print
        xf0 = CG_print / 2; % Neutral point of the CG print
        xfdelta = k * (fwrd_vel - s.d_fwrd_vel); % displacement of ft from neutral point
        
        xf = xf0 + xfdelta; % where to place the foot
        
        % Find the touchdown angle
        thetaLegBody = asin(xf / s.d0);
        theta = thetaLegBody + pi / 2;
    
    end
    
    % If the model is in stance, the controller does not need to do
    % anything (for this SLIP model at least)    
end

