function [throttle,memory] =  pid_control(z_target, z_current, dt, memory,p)
    error = z_target - z_current; %error calculation
    P_out = error * p.Kp_z; %Proportional term

    memory.integral = memory.integral + (error * dt);
    I_out = memory.integral * p.Ki_z; %Integral term

    derivative = (error - memory.prev_error) / dt;
    D_out = derivative * p.Kd_z; %Derivative term

    %Feed forward 
    u_hover = (p.mass * p.gravity);

    total_force = u_hover + P_out + I_out + D_out;

    %Since force = kt*omega^2

    throttle = sqrt(max(0,total_force/(4*p.k_thrust)));

    memory.prev_error = error;

end

    
