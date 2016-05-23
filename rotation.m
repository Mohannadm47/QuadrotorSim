function R = rotation(angles)
c_phi = cos(angles(1));
c_theta = cos(angles(2));
c_psi = cos(angles(3));

s_phi = sin(angles(1));
s_theta = sin(angles(2));
s_psi = sin(angles(3));

R = [c_phi*c_psi - c_theta*s_phi*s_psi, -c_psi*s_phi - c_phi*c_theta*s_psi, s_theta*s_psi;...
	c_theta*c_psi*s_phi + c_phi*s_psi, c_phi*c_theta*c_psi - s_phi*s_psi, -c_psi*s_theta; ...
	s_phi*s_theta, c_phi*s_theta, c_theta];
end