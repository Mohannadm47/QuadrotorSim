function theta_dot = omega2thetadot (omega, angles)
psi = angles(1);
theta = angles(2);

theta_dot = [1, sin(psi)*sin(theta)/cos(theta),  cos(psi)*sin(theta)/cos(theta); 
	           0, cos(theta),                     -sin(psi);
	           0, sin(psi)/cos(theta),             cos(psi)/cos(theta)] * omega;
         
end