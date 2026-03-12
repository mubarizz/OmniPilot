clear; clc; close all;
init_parameters;
% Initial State (X, Y, Z, U, V, W, Phi, Theta, Psi, P, Q, R)
% In main.m, change your initial X:
X_initial = zeros(12,1); 
X_initial(8) = deg2rad(10); % Start with a 10-degree Pitch (theta)

tspan = [0, 5];
omega_hover = [16000; 16000; 16000; 16000];


%I will try to get multiple omegas from a list
t_list = [0,2,3,4,5];
speed_list = [[15000,15000,15000,15000],
              [17000,17000,17000,17000],
              [15500,15500,15500,15500],
              [18000,14000,14000,18000],
              [18000,14000,14000,18000]];


% Simplified call: only t, X, and omega
[t, states] = ode45(@(t, X) drone_dynamics(t, X, t_list, speed_list,p), tspan, X_initial);

% Plot Altitude
z_height = states(:,3);
z_height(z_height < 0) = 0; %Force height to be 0 if it goes negative
states(:,3) = z_height;
plot(t, z_height);
ylabel('Altitude (m)'); xlabel('Time (s)');
title('Flight Test'); grid on;

figure(3);
plot3(states(:,1), states(:,2), states(:,3), 'b', 'LineWidth', 2);
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Flight Path');
axis equal; 