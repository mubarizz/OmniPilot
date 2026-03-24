% Defining drone's properties 
%Physical Constants for the Crazyflie 2.1 Digital Twin

p.mass = 0.6; % mass of the drone [kg]
p.arm_length = 0.25; % distance from center to arm [m]
p.gravity = 9.81;

%Aerodynamics coefficients

p.k_thrust = 3.16e-10; %Thrust coefficient [N/RPM^2]
p.k_drag = 7.94e-12; %Drag coefficient [Nm/RPM^2]
p.C_drag = 0.01; % Air resistance coefficient
p.D_pqr = 0.001;
p.u_hover = p.mass * p.gravity;

%Moments of Inertia
p.Ixx = 0.005;       % Roll inertia
p.Iyy = 0.005;       % Pitch inertia
p.Izz = 0.009;       % Yaw inertia

% Inertia Matrix (Inertia Tensor)
I = diag([p.Ixx, p.Iyy, p.Izz]);

%Defining initial values
x = 0; y = 0; z = 0; u = 0; v = 0; w = 0; phi = 0; theta = 0; psi = 0; p_rate = 0; q_rate = 0; r_rate = 0;

% State vector
X = [x; y; z; u; v; w; phi; theta; psi; p_rate; q_rate; r_rate];

% PID Gains (Will be tuned after tests)
p.Kp_z = 10.0; % Proportional
p.Ki_z = 2; % Integral
p.Kd_z = 5; % Derivative


p.Kp_phi = 3.5; % Proportional
p.Ki_phi = 0.01; % Integral
p.Kd_phi = 4.5; % Derivative

p.Kp_th = 3.5; % Proportional
p.Ki_th = 0.01; % Integral
p.Kd_th = 4.5; % Derivative

p.Kp_ps = 2.5; % Proportional
p.Ki_ps = 0.05; % Integral
p.Kd_ps = 0.1; % Derivative




hudpr_gains = dsp.UDPReceiver('LocalIPPort', 5007, 'MessageDataType', 'single');