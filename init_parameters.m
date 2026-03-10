% Defining drone's properties 
%Physical Constants for the Crazyflie 2.1 Digital Twin

mass = 0.029; % mass of the drone [kg]
arm_length = 0.046; % distance from center to arm [m]
gravity = 9.81;

%Aerodynamics coefficients

k_thrust = 3.16e-10; %Thrust coefficient [N/RPM^2]
k_drag = 7.94e-12; %Drag coefficient [Nm/RPM^2]

%Moments of Inertia
Ixx = 1.657e-5;       % Roll inertia
Iyy = 1.665e-5;       % Pitch inertia
Izz = 2.926e-5;       % Yaw inertia

% Inertia Matrix (Inertia Tensor)
I = diag([Ixx, Iyy, Izz]);