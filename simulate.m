%% Quadrotor.

clear all;
quadrotor = QuadRotor;
quadrotor.x(7:9) = [0.0; 0; 0.14];

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
r = [0; 0; 35; 0; 0; 0];

k = quadrotor.k;
b = quadrotor.b;
L = quadrotor.L;
T = [zeros(2,4); k*ones(1,4); L*k, 0, -L*k, 0; 0, L*k, 0, -k*L; b, -b, b, -b];

prev_error = zeros(6,1);

% Step through the simulation, updating the state.
for t = times(1)
	
	fprintf('###########\n');
	fprintf('Iteration %d\n', t/dt);
fprintf('###########\n');
	
	
	error(1:3, :) = r(1:3) - quadrotor.x(1:3);
	fprintf('Rotation Error:\n');
	error(4:6, :) = R_err(rotation(quadrotor.x(7:9)'), rotation(r(4:6)'))
	angles = quadrotor.x(7:9)
	%error = r - quadrotor.x(3,1);
	
	R = [rotation(quadrotor.x(7:9)'), zeros(3); zeros(3), rotation(quadrotor.x(7:9)')];

	pinv(T)*R'*error*10
	quadrotor.u = pinv(T)*R'*error*10;
	
	all_x (:,index) = quadrotor.x;
	
	%quadrotor.u = ones(4,1)*error*15;
	
	quadrotor.x = quadrotor.step;
	prev_error = error;

	index = index + 1;
end

plot(all_x(3,:));
all_x(:, end)