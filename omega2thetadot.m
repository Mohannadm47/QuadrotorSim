function theta_dot = omega2thetadot (omega, angles)
theta_dot = [1, 0, -sin(angles(2)); 0, cos(angles(1)), cos(angles(2))*sin(angles(1)); 0, -sin(angles(1)), cos(angles(1))*cos(angles(2))]' * omega;
end