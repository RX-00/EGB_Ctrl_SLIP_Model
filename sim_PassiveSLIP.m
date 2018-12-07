% sim_PassiveSLIP attempt at a passive SLIP model by Roy X.
% Find (x, y) of the point mass!
% 
% q = [ x, x dot, y, y dot, xtd, phase flag, theta]
%
% theta is defined as the angle formed from the right side of the touchdown
% to the leg itself, this is the touchdown angle in this simulator
%
% phase flag is (0 for flight, 1 for stance)

%% Establish System Parameters

clear; close all; clc

% input struct for all the chosen variables and parameters for the physics
% equations
input.theta = 90;                     % touchdown angle in degrees
input.theta = input.theta * pi / 180; % Convert degrees to radians
assert(input.theta < pi, 'ERROR: Touchdown theta must not be greater than pi')
input.d0 = .8;                        % leg length (m)
input.k = 4500;                       % spring stiffness constant (N/m)
input.m = 20;                         % mass of the SLIP model (kg)
input.g = 9.81;                       % gravity constant (m/s/s)
input.d_fwrd_vel = 0.9;               % target forward velocity (m/s)
input.currentApexY = 0;               % current apex height y_i (m)
input.prevApexY = 0;                  % previous apex height y_i-1 (m)
input.gnd_height = 0;                 % height of the ground at a specific point along the x-axis

% Starting conditions of the state vector x, fwrd vel, y, upwrd vel,
% foot position upon touchdown, and what phase you're in (0 for flight, 1
% for stance)
q0 = [0; 0.1; 1.3; 0; 0; 0; input.theta];

%-------------------------------------------------------------------- 
% TODO: Implement the controller for Spring-Legged Locomotion on
% Uneven Ground: A Control Approach to Keep the Running Speed
% Constant
%--------------------------------------------------------------------

refine = 4;         % determines how refined the calculations are for ODE45

% Flight event for phase transition from flight to stance
flightEvent = @(t, q) flightToStance(t, q, input); % Identify end of flight (body height, y, is 0 either by being zero or y - d0*sin(theta) is 0, aka when the SLIP hits the ground at an angle -> y = d0*sin(theta))
% Stance event for phase transition from stance to flight
stanceEvent = @(t, q) stanceToFlight(t, q, input); % Identify end of stance (Force of the spring is 0, aka k * (d0 - d) == 0, or y, pos of the body, is 0 (which I don't think needs to be specified here, since y == 0 is supposed to just indicate the SLIP model as fallen))

% due to the plot here ode plots everytime by itself, helps see what's
% going on inbetween
optionsFlight = odeset('Events', flightEvent, 'OutputFcn', @odeplot, 'OutputSel', 1, ...
    'Refine', refine); % End of flight trigger, is passed to ODE for flight
optionsStance = odeset('Events', stanceEvent, 'OutputFcn', @odeplot, 'OutputSel', 1, ...
    'Refine', refine); % End of stance trigger, is passed to ODE for stance

% time stuff
tspan = [0 50];        % How long in seconds the simulation will run for
tStep = 0.009;         % How big of a time step the simulation moves through when solving
tstart = tspan(1);     % Start of the simulation time
tend = tspan(end);     % End of the simulation time
twhile = tstart;       % global solution time

tout = [];             % time throughout the simulation
qout = [];             % state vectors throughout the simulation
KEout = [];            % kinetic energy throughout the simulation
PEout = [];            % potential energy throughout the simulation

teout = [];            % time when events occur throughout the simulation
qeout = [];            % state when events occur throughout the simulation
ieout = [];            % phase which event trigger the switch throughout the simulation

% Flight function dynamics
flightDyn = @(t, q) SLIP_Flight(t, q, input); % Dynamics of SLIP in flight phase

% Stance function dynamics
stanceDyn = @(t, q) SLIP_Stance(t, q, input); % Dynamics of SLIP in stance phase

% Old tests of stance and flight isolated
%[t, q] = ode45(stanceDyn, tspan, q0);
%[t, q] = ode45(flightDyn, tspan, q0);

bounce_num = 0;        % How many times the SLIP model has bounced on the ground

%% ODE Loop to solve the system
while isempty(tout) || tout(end) < tend - tStep
    if q0(6) == 0 % The model is in flight phase until transition to stance phase is triggered       

        optionsFlight = odeset('Events', flightEvent, 'OutputFcn', @odeplot, 'OutputSel', 1, ...
    'Refine', refine);
        [t, q, te, qe, ie] = ode45(flightDyn, [tstart(end):tStep:tend], q0, optionsFlight);
        
        tstart = t;                     % Set the time of the simulation to the current time
        q(end, 5) = q(end,1) - input.d0 * cos(pi - q(end, 7)); % Calculate foot touchdown pos (xtd)
        q(end, 6) = 1;                  % Set the phase flag to stance
        q0 = q(end,:);                  % Update the current state vector
        bounce_num = bounce_num + 1;
        
        % Calculate apex height in flight & time fall here?
        for i = 1:(length(q))
            if(q(3) > input.currentApexY)
                input.prevApexY = input.currentApexY;
                input.currentApexY = q(3);
                fprintf('Current apex y_i: %f, Previous apex y_i-1: %f\n',...
                    input.currentApexY, input.prevApexY);
            end
        end
        

        % Accumulate output
        nt = length(t);
        tout = [tout; t(2:nt)];
        qout = [qout; q(2:nt,:)];
        teout = [teout; te];
        qeout = [qeout; te];
        ieout = [ieout; te];
        
        % Check if everything is alright, i.e. not y < 0
        if q(end, 3) <= 0
            % Terminate the program for the SLIP model has fallen
            fprintf('SLIP Model has fallen (y < 0) at t = %f \n', tout(end))
            break;
        end
        
    else % The model is in stance phase until transition to flight phase is triggered
        
        optionsStance = odeset('Events', stanceEvent, 'OutputFcn', @odeplot, 'OutputSel', 1, ...
    'Refine', refine);
        [t, q, te, qe, ie] = ode45(stanceDyn, [tstart(end):tStep:tend], q0, optionsStance);
        
        tstart = t;                      % Set the time of the simulation to the current time
        q(end, 6) = 0;                   % Set the phase flag to flight
        q0 = q(end,:);                   % Update the current state vector
        
        % RAIBERT P CONTROLLER
        [xf, theta] = raibertPController(q, input, t);
        q0(7) = theta;
        
        
        % Accumulate output
        nt = length(t);
        tout = [tout; t(2:nt)];
        qout = [qout; q(2:nt,:)];
        teout = [teout; te];
        qeout = [qeout; te];
        ieout = [ieout; te];
        
        % Check if everything is alright, i.e. not y < 0
        if q(end, 3) <= 0
            % Terminate the program for the SLIP model has fallen
            fprintf('SLIP Model has fallen (y < 0) at t = %f \n', tout(end))
            break;
        end
    end
    
end


plot(qout(:,1), qout(:,3));
fprintf('Bounced %d times \n', bounce_num)

xlabel('distance');
ylabel('height');
title('SLIP Model COM Trajectory');
hold off


% Calculate Energy in the system
for i = 1:(length(tout)) % the last value is when the SLIP has fallen
    if(qout(i, 6) == 1) % energy in stance
        d = sqrt((qout(i, 1) - qout(i, 5)).^2 + (qout(i, 3).^2));
        KEout = [KEout; 1 / 2 * input.m * ((qout(i, 2)).^2 + (qout(i, 4)).^2)];
        PEout = [PEout; input.m * input.g * qout(i, 3) + 1/2 * input.k * (input.d0 - d).^2];
    else % energy in flight
        KEout = [KEout; 1 / 2 * input.m * ((qout(i, 2)).^2 + (qout(i, 4)).^2)];
        PEout = [PEout; input.m * input.g * qout(i, 3)];
    end
end


% PLOT ALL OF YOUR DATA
figure(50);
plot(tout, KEout);
hold on;
plot(tout, PEout);
hold on;
plot(tout, PEout + KEout);
for i = 1:(length(teout))
    line([teout(i) teout(i)], [0 300]);
end
legend('Kinetic Energy','Potential Energy','Total Energy','trigger event time');
hold on;

animate_SLIP(qout, input, tout);