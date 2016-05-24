function omega = thetadot2omega(theta_dot, angles)
psi = angles(1);
theta = angles(2);

omega = inv([1, sin(psi)*sin(theta)/cos(theta),  cos(psi)*sin(theta)/cos(theta); 
	       0, cos(theta),                     -sin(psi);
	       0, sin(psi)/cos(theta),             cos(psi)/cos(theta)]) * theta_dot;
end