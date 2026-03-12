function dX = drone_dynamics(t, X, t_list,speed_list, p)
    omega = interp1(t_list, speed_list,t)';
    % --- UNPACK STATES ---
    phi = X(7); theta = X(8); psi = X(9);
    
    % --- FORCES & TORQUES ---
    f = p.k_thrust * omega.^2; 
    F_total = sum(f);
    l_eff = p.arm_length / sqrt(2);
    
    tau_phi   = l_eff * (f(1) - f(2) - f(3) + f(4));
    tau_theta = l_eff * (f(1) + f(2) - f(3) - f(4));
    tau_psi   = p.k_drag * (omega(1)^2 - omega(2)^2 + omega(3)^2 - omega(4)^2);

    F_drag = -p.C_drag*X(4:6);

    % --- ROTATION & ACCELERATION ---
    s_phi = sin(phi); c_phi = cos(phi);
    s_the = sin(theta); c_the = cos(theta);
    s_psi = sin(psi); c_psi = cos(psi);

    R = [c_the*c_psi, s_phi*s_the*c_psi - c_phi*s_psi, c_phi*s_the*c_psi + s_phi*s_psi;
         c_the*s_psi, s_phi*s_the*s_psi + c_phi*c_psi, c_phi*s_the*s_psi - s_phi*c_psi;
         -s_the,      s_phi*c_the,                    c_phi*c_the];

    accel = (R * [0; 0; F_total]) / p.mass - [0; 0; p.gravity] + (F_drag/p.mass);

    % --- DERIVATIVES (dX) ---
    dX = zeros(12,1);
    dX(1:3) = X(4:6);          % Velocity
    dX(4:6) = accel;           % Acceleration
    dX(7:9) = X(10:12);        % Angular Velocity
    dX(10)  = (tau_phi - p.D_pqr*X(10)) / p.Ixx;
    dX(11)  = (tau_theta- p.D_pqr*X(11)) / p.Iyy;
    dX(12)  = (tau_psi- p.D_pqr*X(12)) / p.Izz;
end