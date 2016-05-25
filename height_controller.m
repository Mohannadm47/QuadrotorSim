%% Quadrotor.

clear all;
quadrotor = QuadRotor;
quadrotor.x(1:3) = [0.0; 0.0; 10.0];
quadrotor.x(7:9) = [0.2; 0;0];

% Simulation times, in seconds.
start_time = 0;
end_time = 10;
dt = quadrotor.dt;
times = start_time:dt:end_time;

% Number of points in the simulation.
N = numel(times);

% Initial simulation state.
x = quadrotor.x;

% Generate a trajectory.
r_i = [quadrotor.x(1:3); quadrotor.x(7:9)];
r_f = [0.0; 0.0; 15.0; 0.0; 0.0; 0.0];
v_i = [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
v_f = [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];

% Pole placement
A = [zeros(6), eye(6); zeros(6), zeros(6)];
B = [zeros(6,6); eye(6,6)];
lambda = [-2, -4, -5, -7, -10, -15, -20, -25, -27, -30, -35, -40];
K = place(A, B, lambda);
Kp = K(:,1:6);
Kd = K(:,7:12);

[p, v, a] = poly3(r_i', r_f', v_i', v_f', 0:dt:end_time);

i = 1;

m = quadrotor.m;
k = quadrotor.k;
b = quadrotor.b;
L = quadrotor.L;
I = quadrotor.I;
T = [k*ones(1,4); L*k, 0, -L*k, 0; 0, L*k, 0, -k*L; b, -b, b, -b];

prev_error = zeros(6,1);
error = zeros(6,1);
error_dot = zeros(6,1);

M = [m*eye(3), zeros(3); zeros(3), I];
G = [0;0;m*quadrotor.g;0;0;0];
% G = [0;0;0;0;0;0];

error_mag = zeros(size(times));

for i = 1:length(times)
	
	current_p = quadrotor.x(1:3)
	current_v = quadrotor.x(4:6);
	current_theta = quadrotor.x(7:9)
	current_thetadot = quadrotor.x(10:12);
	current_omega = thetadot2omega(current_thetadot, current_theta);
	S = [0;0;0; cross(current_omega, I*current_omega)];
	
	
	error(1:3, :) = p(i,1:3)' - current_p;
	error_dot(1:3, :) = v(i,1:3)' - current_v;
	error(4:6, :) = R_err(rotation(current_theta'), rotation(p(i,4:6)'));
	error_dot(4:6, :) = v(i,4:6)' - current_thetadot;
	
	error_mag(i) = error'*error;
	
	R = (rotation(quadrotor.x(7:9)));

	F = M*(a(i,:)' + Kp*error + Kd*error_dot) + S + G
	
  reshape(quadrotor.x,3,4)
	F(1:3) = inv(R)*F(1:3);
	
	u = inv(T)*F(3:6);
	u(u<0) = 0.0;
	
	quadrotor.u = u;
	
	all_x (:,i) = quadrotor.x;

	
	 if mod(i,10) == 0
        cla
        line = eye(3)*rotation(quadrotor.x(7:9));
        for j = 1:3
            plot3([0,line(1,j)] + quadrotor.x(1),[0,line(2,j)] + quadrotor.x(2),[0,line(3,j)] + quadrotor.x(3)); hold on;
				end
        axis([-10 + quadrotor.x(1),10 + quadrotor.x(1),-10 + quadrotor.x(2),10 + quadrotor.x(2),0,20])
        grid on
        pause(0.01);
	 end
		
	quadrotor.x = quadrotor.step;
	prev_error = error;

end

final_state = reshape(all_x(:, end),3,4)

figure(2);
plot(all_x(3,:));
figure(3);
plot(error_mag);
