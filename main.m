clear; clc; close all;

% 1. Load Crazyflie 2.1 Physics
init_parameters; 

% 2. Simulation Setup
dt = 0.01;                  % 100Hz Control Loop
time = 0 : dt : 15;         % 10 Seconds
target = [1.5;deg2rad(7);deg2rad(5);deg2rad(10)];
u = udpport("byte", "LocalHost", "127.0.0.1");

% 3. Initialize State Vector (12 states)
% [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
X = zeros(12,1); 
X(7) = deg2rad(0);  % Start with a 10-degree Pitch (theta) for realism
X(8) = deg2rad(0);

% 4. Initialize PID Memory
pid_mem.integral_z = 0;  pid_mem.prev_error_z = 0;
pid_mem.integral_phi = 0; pid_mem.prev_error_phi = 0;
pid_mem.integral_th = 0; pid_mem.prev_error_th = 0;
pid_mem.integral_ps = 0; pid_mem.prev_error_ps = 0;

% --- PRE-ALLOCATION (Before the loop) ---
z_history     = zeros(length(time), 1);
phi_history   = zeros(length(time), 1);
theta_history = zeros(length(time), 1);
psi_history   = zeros(length(time), 1);

%%%%% PID Simulation Loop %%%%%
for i = 1:length(time)
    % 1. SENSE: Get current altitude 
    current = [X(3); X(7); X(8); X(9)]; 
    
    % 2. THINK: Run the PID controller
    % This returns the REQUIRED motor speed (omega) to reach target
    [omega, pid_mem] = pid_control(target, current, dt, pid_mem, p);
    
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
    x_history(i) = X(1);
    y_history(i) = X(2);
    z_history(i) = X(3);
    phi_history(i)   = rad2deg(X(7));
    theta_history(i) = rad2deg(X(8));
    psi_history(i)   = rad2deg(X(9));
    % 7. UDP sending
    data = single([X(1),X(2),X(3),X(7),X(8),X(9)]);
    write(u, typecast(data,'uint8'), "127.0.0.1", 5005)
end

%%%%% Visualization %%%%%
figure('Name', 'Crazyflie 2.1 Full PID Performance');

% --- Altitude Plot ---
subplot(2,2,1);
plot(time, z_history, 'b', 'LineWidth', 2);
hold on; yline(target(1), 'r--'); grid on;
ylabel('Altitude (m)'); title('Z-Axis');

% --- Roll Plot ---
subplot(2,2,2);
plot(time, phi_history, 'm', 'LineWidth', 1.5);
hold on; yline(target(2), 'r--'); grid on;
ylabel('Roll (deg)'); title('Phi (\phi)');

% --- Pitch Plot ---
subplot(2,2,3);
plot(time, theta_history, 'g', 'LineWidth', 1.5);
hold on; yline(target(3), 'r--'); grid on;
ylabel('Pitch (deg)'); title('Theta (\theta)');

% --- Yaw Plot ---
subplot(2,2,4);
plot(time, psi_history, 'c', 'LineWidth', 1.5);
hold on; yline(target(4), 'r--'); grid on;
ylabel('Yaw (deg)'); title('Psi (\psi)');

figure;
subplot(1,2,1);
plot(time, x_history, 'r', 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('X (m)'); title('X Position');

subplot(1,2,2);
plot(time, y_history, 'b', 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('Y (m)'); title('Y Position');

% Top-down 2D path
figure;
plot(x_history, y_history, 'w', 'LineWidth', 2); grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('XY Trajectory'); axis equal;
