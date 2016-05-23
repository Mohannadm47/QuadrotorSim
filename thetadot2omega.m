function omega = thetadot2omega(theta_dot, angles)
omega = [1, 0, -sin(angles(2)); 0, cos(angles(1)), cos(angles(2))*sin(angles(1)); 0, -sin(angles(1)), cos(angles(1))*cos(angles(2))] * theta_dot;
end