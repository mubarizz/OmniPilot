clear; clc; close all;

% 1. Load Crazyflie 2.1 Physics
init_parameters; 

% 2. Simulation Setup
dt = 0.01;                  % 100Hz Control Loop
time = 0 : dt : 10;         % 10 Seconds
z_target = 2.0;             % Target: 2 Meters

% 3. Initialize State Vector (12 states)
% [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
X = zeros(12,1); 
X(3) = 0;                   % Start at ground level
X(8) = deg2rad(10);         % Start with a 10-degree Pitch (theta) for realism

% 4. Initialize PID Memory
pid_mem.integral = 0;     % Ensure these names match your pid_control.m
pid_mem.prev_error = 0;

% 5. Data Logging (Pre-allocation)
z_history = zeros(length(time), 1);
theta_history = zeros(length(time), 1); % Let's track pitch too!

%%%%% PID Simulation Loop %%%%%
for i = 1:length(time)
    % 1. SENSE: Get current altitude 
    z_current = X(3); 
    
    % 2. THINK: Run the PID controller
    % This returns the REQUIRED motor speed (omega) to reach target
    [omega_val, pid_mem] = pid_control(z_target, z_current, dt, pid_mem, p);
    
    % 3. ACT: Apply RPM to all 4 motors
    % (In pure Z-control, all motors spin at the same speed)
    omega = [omega_val; omega_val; omega_val; omega_val];
    
    % 4. PHYSICS: Calculate the derivatives using your drone_dynamics.m
    X_dot = drone_dynamics(X, omega, p);
    
    % 5. UPDATE: Euler Integration (Move forward in time)
    X = X + X_dot * dt;
    
    % --- Ground Collision Logic ---
    if X(3) < 0
        X(3) = 0;  % Drone cannot go below the floor
        X(6) = 0;  % Vertical velocity becomes 0 on impact
    end
    
    % 6. LOG: Save values for plotting
    z_history(i) = X(3);
end

%%%%% Visualization %%%%%
figure('Name', 'Crazyflie 2.1 Flight Data');

% Plot Altitude
plot(time, z_history, 'b', 'LineWidth', 2);
hold on;
yline(z_target, 'r--', 'Setpoint', 'LabelHorizontalAlignment','left');
grid on;
ylabel('Altitude (m)');
title('Altitude PID Control (Z-Axis)');

