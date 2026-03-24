













function [omega,memory] =  pid_control(target, current, dt, memory,p)

    % Altitude (Z) PID    

    error_z = target(1) - current(1); %error calculation
    

    Pz_out = error_z * p.Kp_z; %Proportional term

    memory.integral_z = memory.integral_z + (error_z * dt);
    memory.integral_z = max(min(memory.integral_z, 0.5/p.Ki_z), -0.5/p.Ki_z); % anti-windup

    Iz_out = memory.integral_z * p.Ki_z; %Integral term

    derivative_z = (error_z - memory.prev_error_z) / dt;
    Dz_out = derivative_z * p.Kd_z; %Derivative term

    u_z = Pz_out + Iz_out + Dz_out + p.u_hover;
    memory.prev_error_z = error_z;


    %Roll (phi)
    error_phi = target(2) - current(2); %error calculation
    Pphi_out = error_phi * p.Kp_phi; %Proportional term

    memory.integral_phi = memory.integral_phi + (error_phi * dt);
    Iphi_out = memory.integral_phi * p.Ki_phi; %Integral term

    derivative_phi = (error_phi - memory.prev_error_phi) / dt;
    Dphi_out = derivative_phi * p.Kd_phi; %Derivative term

    u_phi = Pphi_out + Iphi_out + Dphi_out;
    memory.prev_error_phi = error_phi;


    % Pitch (Theta)
    error_th = target(3) - current(3); %error calculation
    Pth_out = error_th * p.Kp_th; %Proportional term

    memory.integral_th = memory.integral_th + (error_th * dt);
    Ith_out = memory.integral_th * p.Ki_th; %Integral term

    derivative_th = (error_th - memory.prev_error_th) / dt;
    Dth_out = derivative_th * p.Kd_th; %Derivative term

    u_th = Pth_out + Ith_out + Dth_out;
    memory.prev_error_th = error_th;

    % Yaw (psi)
    error_ps = target(4) - current(4); %error calculation
    Pps_out = error_ps * p.Kp_ps; %Proportional term

    memory.integral_ps = memory.integral_ps + (error_ps * dt);
    Ips_out = memory.integral_ps * p.Ki_ps; %Integral term

    derivative_ps = (error_ps - memory.prev_error_ps) / dt;
    Dps_out = derivative_ps * p.Kd_ps; %Derivative term

    u_ps = Pps_out + Ips_out + Dps_out;
    memory.prev_error_ps = error_ps;
    
    % X-Configuration 
    omega = zeros(4,1);
    RPM_base = sqrt(max(u_z/4, 0) / p.k_thrust);
    omega(1) = RPM_base - u_phi + u_th + u_ps; %Front-left
    omega(2) = RPM_base + u_phi + u_th - u_ps; %Front-right
    omega(3) = RPM_base + u_phi - u_th + u_ps; % Rear-Right (Slightly different signs)
    omega(4) = RPM_base - u_phi - u_th - u_ps; % Rear-Left

    omega = max(min(omega, 1800), 0);
end

    
