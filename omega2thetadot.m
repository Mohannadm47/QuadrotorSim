function theta_dot = omega2thetadot (omega, angles)
psi = angles(3);
theta = angles(2);
phi = angles(1);
theta_dot = [1, sin(phi)*sin(theta)/cos(theta),  cos(phi)*sin(theta)/cos(theta); 
	           0, cos(theta),                     -sin(phi);
	           0, sin(phi)/cos(theta),             cos(phi)/cos(theta)] * omega;
end