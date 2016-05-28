%% Quadrotor.

clear all;
close all;
format short;



quadrotor = QuadRotor;
m = quadrotor.m;
k = quadrotor.k;
b = quadrotor.b;
L = quadrotor.L;
I = quadrotor.I;
T = [k*ones(1,4); L*k, 0, -L*k, 0; 0, L*k, 0, -k*L; b, -b, b, -b];
M = [m*eye(3), zeros(3); zeros(3), I];
g = quadrotor.g;
G = [0;0;g;0;0;0];


quadrotor.x(1:3) = [0.0; 0.0; 10.0];
quadrotor.x(4:6) = [0; 0; 0];
quadrotor.x(7:9) = [0.0; 0; 0];
quadrotor.x(10:12) = [pi/4; 0.0; 0.0];

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
r_f = [10.0; 10.0; 15.0; 0;0;0];
v_i = [0.0; 0.0; 0.0; 0.0; 0.0; 0];
v_f = [0.0; 0.0; 0.0; 0.0; 0.0; 0];

Kp = 100;
Kd = 10;

Kp2 = 5;
Kd2 = 2;

% Kp2 = 0.0;
% Kd2 = 0.0;

[p, v, a] = poly3(r_i', r_f', v_i', v_f', 0:dt:end_time);

r = 10;
tx = pi/8;
t = times;

p(:,1:3) = bsxfun(@plus,[cos(t*tx)*r;sin(t*tx)*r;0.1*t], quadrotor.x(1:3) - [0;0;r])' ;
v(:,1:3) = [-tx*sin(t*tx)*r;tx*cos(t*tx)*r;0.1*ones(size(t))]';
a(:,1:3) = [-tx^2*cos(t*tx)*r;-tx^2*sin(t*tx)*r;zeros(size(t))]';

% p(:,1:3) = bsxfun(@plus,[0.1*t;cos(t*tx)*r;sin(t*tx)*r], quadrotor.x(1:3) - [0;r;0])' ;
% v(:,1:3) = [0.1*ones(size(t));-tx*sin(t*tx)*r;tx*cos(t*tx)*r]';
% a(:,1:3) = [zeros(size(t));-tx^2*cos(t*tx)*r;-tx^2*sin(t*tx)*r]';

v(:,6) = ones(size(v(:,6)))*tx*-1;
p(:,6) = t*tx*-1;
quadrotor.x(1:3) = p(1,1:3);
quadrotor.x(4:6) = v(1,1:3);

F = M(1:3,1:3)*(G(1:3) + a(1,1:3)') + quadrotor.x(4:6)*quadrotor.kd;
r_rotation = [-1*F(2),F(1),p(1,6)]./[F(3),F(3),1];


% quadrotor.x(7:9) = p(1,4:6);
quadrotor.x(7:9) = r_rotation + [pi/12, pi/12, 0];
% quadrotor.x(7:9) = [pi/4, pi/4, 0];
quadrotor.x(10:12) = v(1,4:6);



prev_error = zeros(6,1);
error = zeros(6,1);
error_dot = zeros(6,1);
F = zeros(6,1);

% p_error = zeros(6,1);



error_mag = zeros(size(times));

figure(1)
for i = 1:length(times)
	
    tic;
	current_p = quadrotor.x(1:3);
	current_v = quadrotor.x(4:6);
	current_theta = quadrotor.x(7:9);
	current_thetadot = quadrotor.x(10:12);
	current_omega = thetadot2omega(current_thetadot, current_theta);
	S = [0;0;0; cross(current_omega, I*current_omega)];
	
	R = (rotation(quadrotor.x(7:9)));
    
	error(1:3) = (p(i,1:3)' - current_p);
	error_dot(1:3) = (v(i,1:3)' - current_v);
    F = zeros(6,1);
    
    F(1:3) = M(1:3,1:3)*([error(1)*Kp2;error(2)*Kp2;error(3)*Kp] + [error_dot(1)*Kd2;error_dot(2)*Kd2;error_dot(3)*Kd] + G(1:3) + a(i,1:3)') + quadrotor.x(4:6)*quadrotor.kd;
    F(3) = F(3)/(cos(current_theta(1))*cos(current_theta(2)));
    
%     F(3) = M(3,3)*(error(3)*Kp + error_dot(3)*Kd + g + a(i,3) + quadrotor.x(6)*quadrotor.kd);
%     F(1:3) = F(1:3)'*R;
%     r_rotation = [-1*(error(2)*Kp2 + error_dot(2)*Kd2 + a(i,2) + quadrotor.x(5)*quadrotor.kd), error(1)*Kp2 + error_dot(1)*Kd2 + a(i,1) + quadrotor.x(4)*quadrotor.kd,p(i,6)]./[F(3)/m,F(3)/m,1];
    r_rotation = [-F(2),F(1),p(i,6)]./[F(3),F(3),1];
%     r_rotation = [-asin(F(2)/F(3)),asin(F(1)/F(3)),p(i,6)];
    

    error(4:6) = R_err(rotation(current_theta'), rotation(r_rotation));
% 	error_dot(4:6) = v(i,4:6)' - current_thetadot;
%     error_dot(4:6) = (error(4:6)-prev_error(4:6))/dt - current_thetadot;
    error_dot(4:6) = [(error(4:5)-prev_error(4:5))/dt;v(i,6)] - current_thetadot;
%     error_dot(4:6) =  - current_thetadot;
    
%     [error_dot(4:6),current_thetadot]
    
% 	p_error(4:6) = error(4:6);
% 	error_mag(i) = error'*error;
    error_mag(i) = error(4:6)'*error(4:6);
%     error_mag(i) = error(1:2)'*error(1:2);
%     error_mag(i) = error(1);
%     error_mag(i) = error(3);
%     error'
%     current_theta

    F(4:6) = M(4:6,4:6)*(error(4:6)*Kp + error_dot(4:6)*Kd) + S(4:6); %a(i,4:6)'

	
%     reshape(quadrotor.x,3,4)
    
	u = inv(T)*F(3:6)
% 	u(u<0) = 0.0;
	
	quadrotor.u = u;
	
	all_x (:,i) = quadrotor.x;

	
	 if mod(i,100) == 0
        cla
        line = eye(3)*rotation(quadrotor.x(7:9));
        for j = 1:3
            plot3([0,line(1,j)] + quadrotor.x(1),[0,line(2,j)] + quadrotor.x(2),[0,line(3,j)] + quadrotor.x(3)); hold on;
        end
        axis([-10 + quadrotor.x(1),10 + quadrotor.x(1),-10 + quadrotor.x(2),10 + quadrotor.x(2),0,20])
        grid on
        
        t2=toc;
        pause(dt - t2);
     end
     
		
	quadrotor.x = quadrotor.step;
	prev_error = error;

end

final_state = reshape(all_x(:, end),3,4)

% figure(2);
% % plot(all_x(3,:));
figure(3);
plot(times,error_mag);