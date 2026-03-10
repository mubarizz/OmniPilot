% Define motor speeds as a vector (column vector)
omega = [5000; 5000; 5000; 5000]; 

% Calculate all forces at once
f = k_thrust * omega.^2; 

% Calculate total thrust
F_total = sum(f);
%Gravity force on the drone 
F_grativy = mass*gravity;

% X - configuration 
l_eff = arm_length / 2^0.5;

roll_torque = l_eff * (f(1) - f(2) - f(3) + f(4));
pitch_torque = l_eff * (f(1) + f(2) - f(3) - f(4));
% Based on our motor numbering (1 & 3 are CW, 2 & 4 are CCW)
yaw_torque = k_drag * (omega(1)^2 - omega(2)^2 + omega(3)^2 - omega(4)^2);

%Angular accelerations

% Roll acceleration (rad/s^2)
alpha_roll  = roll_torque / Ixx; 

% Pitch acceleration (rad/s^2)
alpha_pitch = pitch_torque / Iyy;

% Yaw acceleration (rad/s^2)
alpha_yaw   = yaw_torque / Izz;
