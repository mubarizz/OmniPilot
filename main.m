clear; clc; close all;
clear u u_in hudpr_gains;
clear functions;
clearvars;
delete(findall(0, 'Type', 'udpport'));
% 1. Load Crazyflie 2.1 Physics
init_parameters;

% 2. Simulation Setup
dt = 0.01;                  % 100Hz Control Loop
time = 0 : dt : 60;         % 60 Seconds

% Inner loop target: [z; phi; theta; psi]
% phi and theta will be computed by outer PID
% z and psi are set directly
target = [1.5; deg2rad(0); deg2rad(0); deg2rad(10)];

% Outer loop position targets (x, y, z, psi)
pos_target = [0.0; 0.0; 1.5; deg2rad(10)];

% UDP sockets
u     = udpport("byte", "LocalHost", "127.0.0.1");
u_in  = udpport("byte", "LocalHost", "127.0.0.1", "LocalPort", 5006);

% 3. Initialize State Vector (12 states)
% [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
X = zeros(12,1);
X(7) = deg2rad(0);
X(8) = deg2rad(0);

% 4. Initialize PID Memory — inner loop
pid_mem.integral_z   = 0;  pid_mem.prev_error_z   = 0;
pid_mem.integral_phi = 0;  pid_mem.prev_error_phi = 0;
pid_mem.integral_th  = 0;  pid_mem.prev_error_th  = 0;
pid_mem.integral_ps  = 0;  pid_mem.prev_error_ps  = 0;

% 5. Initialize Outer PID Memory — x and y position
outer_mem.integral_x  = 0;  outer_mem.prev_error_x  = 0;
outer_mem.integral_y  = 0;  outer_mem.prev_error_y  = 0;

% Outer PID gains — tune these for x/y position control
% --- Adjusted for Zero Overshoot and Stability ---
Kp_x = 0.15;   Ki_x = 0;  Kd_x = 0.25;  
Kp_y = 0.15;   Ki_y = 0;  Kd_y = 0.25;  

% Restrict tilt to prevent the drone from building too much speed
max_angle = deg2rad(18);

% --- PRE-ALLOCATION ---
x_history     = zeros(length(time), 1);
y_history     = zeros(length(time), 1);
z_history     = zeros(length(time), 1);
phi_history   = zeros(length(time), 1);
theta_history = zeros(length(time), 1);
psi_history   = zeros(length(time), 1);

%%%%% PID Simulation Loop %%%%%
for i = 1:length(time)

    % 0. CHECK: Read new position target from Python if available
    available = u_in.NumBytesAvailable;
    if available >= 16
        if available > 16
            read(u_in, available - 16, 'uint8');  % flush old packets
        end
        raw = read(u_in, 16, 'uint8');
        new_pos = double(typecast(uint8(raw), 'single'));
        % new_pos = [x_target, y_target, z_target, psi_target]
        pos_target(1) = new_pos(1);   % x
        pos_target(2) = new_pos(2);   % y
        pos_target(3) = new_pos(3);   % z
        pos_target(4) = new_pos(4);   % psi (already in radians from Python)
    end

    % 1. OUTER PID: x position → theta (pitch) setpoint
    error_x = pos_target(1) - X(1);
    outer_mem.integral_x = outer_mem.integral_x + error_x * dt;
    outer_mem.integral_x = max(min(outer_mem.integral_x, 0.5), -0.5);  % anti-windup
    deriv_x = (error_x - outer_mem.prev_error_x) / dt;
    theta_sp = Kp_x * error_x + Ki_x * outer_mem.integral_x + Kd_x * deriv_x;
    theta_sp = max(min(theta_sp, max_angle), -max_angle);  % clamp
    outer_mem.prev_error_x = error_x;

    % 2. OUTER PID: y position → phi (roll) setpoint
    error_y = pos_target(2) - X(2);
    outer_mem.integral_y = outer_mem.integral_y + error_y * dt;
    outer_mem.integral_y = max(min(outer_mem.integral_y, 0.5), -0.5);  % anti-windup
    deriv_y = (error_y - outer_mem.prev_error_y) / dt;
    phi_sp = -(Kp_y * error_y + Ki_y * outer_mem.integral_y + Kd_y * deriv_y);
    phi_sp = max(min(phi_sp, max_angle), -max_angle);  % clamp
    outer_mem.prev_error_y = error_y;

    % 3. BUILD inner loop target from outer PID outputs
    target(1) = pos_target(3);  % z setpoint (direct)
    target(2) = phi_sp;         % phi setpoint (from outer y PID)
    target(3) = theta_sp;       % theta setpoint (from outer x PID)
    target(4) = pos_target(4);  % psi setpoint (direct)

    % 4. SENSE: Get current state for inner loop
    current = [X(3); X(7); X(8); X(9)];

    % 5. THINK: Run inner PID controller (unchanged)
    [omega, pid_mem] = pid_control(target, current, dt, pid_mem, p);

    % 6. PHYSICS
    X_dot = drone_dynamics(X, omega, p);

    % 7. UPDATE: Euler Integration
    X = X + X_dot * dt;

    % Ground Collision Logic
    if X(3) < 0
        X(3) = 0;
        X(6) = 0;
    end

    % 8. LOG
    x_history(i)     = X(1);
    y_history(i)      = X(2);
    z_history(i)      = X(3);
    phi_history(i)    = rad2deg(X(7));
    theta_history(i)  = rad2deg(X(8));
    psi_history(i)    = rad2deg(X(9));

    % 9. UDP send state to Python
    data = single([X(1), X(2), X(3), X(7), X(8), X(9)]);
    write(u, typecast(data, 'uint8'), "127.0.0.1", 5005);

    pause(0.01);
end

clear u u_in

%%%%% Visualization %%%%%
figure('Name', 'Crazyflie 2.1 Full PID Performance');

subplot(2,2,1);
plot(time, z_history, 'b', 'LineWidth', 2);
hold on; yline(pos_target(3), 'r--'); grid on;
ylabel('Altitude (m)'); title('Z-Axis');

subplot(2,2,2);
plot(time, phi_history, 'm', 'LineWidth', 1.5);
hold on; grid on;
ylabel('Roll (deg)'); title('Phi (\phi)');

subplot(2,2,3);
plot(time, theta_history, 'g', 'LineWidth', 1.5);
hold on; grid on;
ylabel('Pitch (deg)'); title('Theta (\theta)');

subplot(2,2,4);
plot(time, psi_history, 'c', 'LineWidth', 1.5);
hold on; grid on;
ylabel('Yaw (deg)'); title('Psi (\psi)');

figure;
subplot(1,2,1);
plot(time, x_history, 'r', 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('X (m)'); title('X Position');

subplot(1,2,2);
plot(time, y_history, 'b', 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('Y (m)'); title('Y Position');

figure;
plot(x_history, y_history, 'w', 'LineWidth', 2); grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('XY Trajectory'); axis equal;
