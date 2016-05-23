classdef QuadRotor
	
	properties (Access = protected)
		trans_
		ax_
	end
	
	properties  (Access = public)
		x  % state [x; dx; phi; dphi]
		dt % sampling time.
		L % Length
		b % 
		k % const relating sq of motor's angular velocity to thrust from a motor.
		I % intertia
		m % mass
		g % gravity
		kd % Aerodynamic damping.
		u % control
		
	end
	
	methods (Access = protected)
		function dx=dynamics(quadrotor,X)
			
			x = X(1:3);
			xdot = X(4:6);
			theta = X(7:9);
			thetadot = X(10:12);
			
			omega = thetadot2omega(thetadot, theta);
			
			% Compute linear and angular accelerations.
			a = acceleration(quadrotor.u, theta, xdot, quadrotor.m, quadrotor.g, quadrotor.k, quadrotor.kd);
			omegadot = angular_acceleration(quadrotor.u, omega, quadrotor.I, quadrotor.L, quadrotor.b, quadrotor.k);
			
			omega = omega + quadrotor.dt * omegadot;
			thetadot = omega2thetadot(omega, theta);
			theta = theta + quadrotor.dt * thetadot;
			xdot = xdot + quadrotor.dt * a;
			x = x + quadrotor.dt * xdot;
			
			dx = [x;xdot;theta;thetadot];
			
		end
		
		segway = init(segway);
	end
	
	methods
		function quadrotor = QuadRotor()
			quadrotor = quadrotor.init;
		end
		x_new=step(quadrotor);
	end
	
end
