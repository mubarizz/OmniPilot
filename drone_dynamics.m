function dX = drone_dynamics(t, X, omega)
    % --- CONSTANTS (Hardcoded inside to prevent indexing errors) ---
    m = 0.029;
    g = 9.81;
    L = 0.046;
    k_t = 3.16e-10;
    k_d = 7.94e-12;
    Ixx = 1.657e-5; Iyy = 1.665e-5; Izz = 2.926e-5;

    % --- UNPACK STATES ---
    phi = X(7); theta = X(8); psi = X(9);
    
    % --- FORCES & TORQUES ---
    f = k_t * omega.^2; 
    F_total = sum(f);
    l_eff = L / sqrt(2);
    
    tau_phi   = l_eff * (f(1) - f(2) - f(3) + f(4));
    tau_theta = l_eff * (f(1) + f(2) - f(3) - f(4));
    tau_psi   = k_d * (omega(1)^2 - omega(2)^2 + omega(3)^2 - omega(4)^2);

    % --- ROTATION & ACCELERATION ---
    s_phi = sin(phi); c_phi = cos(phi);
    s_the = sin(theta); c_the = cos(theta);
    s_psi = sin(psi); c_psi = cos(psi);

    R = [c_the*c_psi, s_phi*s_the*c_psi - c_phi*s_psi, c_phi*s_the*c_psi + s_phi*s_psi;
         c_the*s_psi, s_phi*s_the*s_psi + c_phi*c_psi, c_phi*s_the*s_psi - s_phi*c_psi;
         -s_the,      s_phi*c_the,                    c_phi*c_the];

    accel = (R * [0; 0; F_total]) / m - [0; 0; g];

    % --- DERIVATIVES (dX) ---
    dX = zeros(12,1);
    dX(1:3) = X(4:6);          % Velocity
    dX(4:6) = accel;           % Acceleration
    dX(7:9) = X(10:12);        % Angular Velocity
    dX(10)  = tau_phi / Ixx;
    dX(11)  = tau_theta / Iyy;
    dX(12)  = tau_psi / Izz;
end