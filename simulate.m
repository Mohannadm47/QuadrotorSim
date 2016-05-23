%% Quadrotor.

clear all;
quadrotor = QuadRotor;

% Simulation times, in seconds.
start_time = 0;
end_time = 10;
dt = quadrotor.dt;
times = start_time:dt:end_time;

% Number of points in the simulation.
N = numel(times);

% Initial simulation state.
x = quadrotor.x;

% Simulate some disturbance in the angular velocity.
% The magnitude of the deviation is in radians / second.
%deviation = 100;
%thetadot = deg2rad(2 * deviation * rand(3, 1) - deviation);
index = 1;
r = 7;

% Step through the simulation, updating the state.
for t = times
	
	error = r - quadrotor.x(3,1);
	
	if(error < 0)
		error = 0;
	end
	
	'help1'
	all_x (:,index) = quadrotor.x(1:3);
	'help2'
	
	quadrotor.u = ones(4,1)*error*15;
	
	quadrotor.x = quadrotor.step;

% 	omega = thetadot2omega(thetadot, theta);
%   Compute linear and angular accelerations.
% 	a = acceleration(i, theta, xdot, m, g, k, kd);
% 	omegadot = angular_acceleration(i, omega, I, L, b, k);
% 	
% 	omega = omega + dt * omegadot;
% 	thetadot = omega2thetadot(omega, theta);
% 	theta = theta + dt * thetadot;
% 	xdot = xdot + dt * a;
% 	x = x + dt * xdot;
	index = index + 1;
end

plot(all_x(3,:));