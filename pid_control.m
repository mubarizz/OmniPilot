function [omega, memory] = pid_control(target, current, dt, memory, p)
    % Persistent variables survive between function calls
    persistent Kp_z Ki_z Kd_z Kp_phi Ki_phi Kd_phi Kp_th Ki_th Kd_th Kp_ps Ki_ps Kd_ps
    persistent hudpr_gains

    % --- INITIALIZATION (Runs only once at the start) ---
    if isempty(Kp_z)
        % Load Baseline Predictions for a 0.6kg Drone
        Kp_z = p.Kp_z; Ki_z = p.Ki_z; Kd_z = p.Kd_z;
        Kp_phi = p.Kp_phi; Ki_phi = p.Ki_phi; Kd_phi = p.Kd_phi;
        Kp_th = p.Kp_th; Ki_th = p.Ki_th; Kd_th = p.Kd_th;
        Kp_ps = p.Kp_ps; Ki_ps = p.Ki_ps; Kd_ps = p.Kd_ps;
        
        % Safely open the UDP port for Sliders
        try
            % Clear any ghost objects on this port first
            old_port = findall(0, 'Type', 'udpport', 'LocalPort', 5007);
            if ~isempty(old_port), delete(old_port); end
            
            hudpr_gains = udpport("byte", "LocalHost", "127.0.0.1", "LocalPort", 5007);
            fprintf('PID UDP Listener started on Port 5007\n');
        catch
            fprintf('Warning: Could not bind Port 5007. Sliders may be inactive.\n');
        end
    end
    
    % --- LIVE SLIDER UPDATES ---
    if ~isempty(hudpr_gains) && hudpr_gains.NumBytesAvailable >= 20
        % 1. Read the entire buffer to clear any lag
        raw_bytes = read(hudpr_gains, hudpr_gains.NumBytesAvailable, "uint8");
        all_floats = typecast(raw_bytes, 'single');
        
        % We need at least 4 values after the header to have a full packet
        found_sync = false;
        for k = (length(all_floats)-4) : -1 : 1
            if all_floats(k) == 999.0
                id    = all_floats(k+1);
                new_p = all_floats(k+2);
                new_i = all_floats(k+3);
                new_d = all_floats(k+4);
                found_sync = true;
                break; 
            end
        end

        % 3. Apply the gains if sync was successful
        if found_sync && any(id == [3, 4, 5, 6])
            switch id
                case 3, Kp_z = new_p; Ki_z = new_i; Kd_z = new_d;
                case 4, Kp_phi = new_p; Ki_phi = new_i; Kd_phi = new_d;
                case 5, Kp_th = new_p; Ki_th = new_i; Kd_th = new_d;
                case 6, Kp_ps = new_p; Ki_ps = new_i; Kd_ps = new_d;
            end
            fprintf('LATEST GAINS SYNCED: Axis %.0f | P:%.2f I:%.2f D:%.2f\n', id, new_p, new_i, new_d);
        end
    end
    % --- PID CALCULATIONS ---
    
    % 1. Altitude (Z) - Includes Feed-Forward Hover Thrust
    err_z = target(1) - current(1);
    memory.integral_z = memory.integral_z + (err_z * dt);
    % Anti-windup: limit integral to 20% of total thrust
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

    % Forces required per motor
    f1 = (F_total/4) - (u_phi/(4*L)) + (u_th/(4*L)) + (u_ps/(4*(k_d/k_t)));
    f2 = (F_total/4) + (u_phi/(4*L)) + (u_th/(4*L)) - (u_ps/(4*(k_d/k_t)));
    f3 = (F_total/4) + (u_phi/(4*L)) - (u_th/(4*L)) + (u_ps/(4*(k_d/k_t)));
    f4 = (F_total/4) - (u_phi/(4*L)) - (u_th/(4*L)) - (u_ps/(4*(k_d/k_t)));

    % Conversion to Motor Speeds (RPM/RadS)
    f_motor = max([f1; f2; f3; f4], 0); 
    omega = sqrt(f_motor / k_t);
    
    % Crazyflie 2.1 physical limit (~22,000 RPM converted to match your constants)
    omega = min(omega, 19200); 
end