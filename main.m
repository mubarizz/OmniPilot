clear; clc; close all;

% Initial State (X, Y, Z, U, V, W, Phi, Theta, Psi, P, Q, R)
X_initial = zeros(12,1); 

tspan = [0, 5];
omega_hover = [18000; 18000; 18000; 18000];

% Simplified call: only t, X, and omega
[t, states] = ode45(@(t, X) drone_dynamics(t, X, omega_hover), tspan, X_initial);

% Plot Altitude
plot(t, states(:,3));
ylabel('Altitude (m)'); xlabel('Time (s)');
title('Flight Test'); grid on;