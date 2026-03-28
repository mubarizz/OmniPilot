function [omega, memory] = pid_control(target, current, dt, memory, p)
    % Persistent variables to store current gains and memory
    persistent Kp_z Ki_z Kd_z Kp_phi Ki_phi Kd_phi Kp_th Ki_th Kd_th Kp_ps Ki_ps Kd_ps

    % --- INITIALIZATION & GAIN SYNC ---
    % We update these every call or just once to ensure they match the 'p' struct
    Kp_z = p.Kp_z; Ki_z = p.Ki_z; Kd_z = p.Kd_z;
    Kp_phi = p.Kp_phi; Ki_phi = p.Ki_phi; Kd_phi = p.Kd_phi;
    Kp_th = p.Kp_th; Ki_th = p.Ki_th; Kd_th = p.Kd_th;
    Kp_ps = p.Kp_ps; Ki_ps = p.Ki_ps; Kd_ps = p.Kd_ps;

    % --- PID CALCULATIONS ---
    
    % 1. Altitude (Z)
    err_z = target(1) - current(1);
    memory.integral_z = memory.integral_z + (err_z * dt);
    % Anti-windup: limit integral to prevent massive overshoot
    memory.integral_z = max(min(memory.integral_z, 2), -2); 
    
    der_z = (err_z - memory.prev_error_z) / dt;
    u_z = (err_z * Kp_z) + (memory.integral_z * Ki_z) + (der_z * Kd_z) + p.u_hover;
    memory.prev_error_z = err_z;

    % 2. Roll (Phi)
    err_phi = target(2) - current(2);
    memory.integral_phi = memory.integral_phi + (err_phi * dt);
    der_phi = (err_phi - memory.prev_error_phi) / dt;
    u_phi = (err_phi * Kp_phi) + (memory.integral_phi * Ki_phi) + (der_phi * Kd_phi);
    memory.prev_error_phi = err_phi;

    % 3. Pitch (Theta)
    err_th = target(3) - current(3);
    memory.integral_th = memory.integral_th + (err_th * dt);
    der_th = (err_th - memory.prev_error_th) / dt;
    u_th = (err_th * Kp_th) + (memory.integral_th * Ki_th) + (der_th * Kd_th);
    memory.prev_error_th = err_th;

    % 4. Yaw (Psi)
    err_ps = target(4) - current(4);
    memory.integral_ps = memory.integral_ps + (err_ps * dt);
    der_ps = (err_ps - memory.prev_error_ps) / dt;
    u_ps = (err_ps * Kp_ps) + (memory.integral_ps * Ki_ps) + (der_ps * Kd_ps);
    memory.prev_error_ps = err_ps;

    % --- MOTOR MIXING (X-Configuration) ---
    F_total = max(u_z, 0);
    L = p.arm_length / sqrt(2);
    k_t = p.k_thrust;
    k_d = p.k_drag;

    % Motor force distribution
    f1 = (F_total/4) - (u_phi/(4*L)) + (u_th/(4*L)) + (u_ps/(4*(k_d/k_t)));
    f2 = (F_total/4) + (u_phi/(4*L)) + (u_th/(4*L)) - (u_ps/(4*(k_d/k_t)));
    f3 = (F_total/4) + (u_phi/(4*L)) - (u_th/(4*L)) + (u_ps/(4*(k_d/k_t)));
    f4 = (F_total/4) - (u_phi/(4*L)) - (u_th/(4*L)) - (u_ps/(4*(k_d/k_t)));

    % Apply physical limits and convert to Motor Speeds (Rad/S)
    f_motor = max([f1; f2; f3; f4], 0); 
    omega = sqrt(f_motor / k_t);
    
    % Physical RPM limit for the 29g quadcopter motors
    omega = min(omega, 18000); 
end