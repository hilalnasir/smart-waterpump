
% smart_pump_sim.m
% Simulates PID control of a water tank with inflow pump

% Parameters
dt = 0.1;
T = 100;
time = 0:dt:T;
tank_area = 1.0;
max_flow = 1.0;
desired_level = 5.0;

% PID gains
Kp = 2.0;
Ki = 0.5;
Kd = 1.0;

% Initialize
level = 0;
integral = 0;
prev_error = 0;
levels = zeros(size(time));
errors = zeros(size(time));
flows = zeros(size(time));

for i = 1:length(time)
    error = desired_level - level;
    integral = integral + error * dt;
    derivative = (error - prev_error) / dt;
    control_signal = Kp*error + Ki*integral + Kd*derivative;
    flow_in = max(0, min(max_flow, control_signal));
    
    outflow = 0.3 * sqrt(max(level, 0));  % avoid negative sqrt
    d_level = (flow_in - outflow) / tank_area;
    level = level + d_level * dt;
    level = max(0, level);

    % Store
    levels(i) = level;
    errors(i) = error;
    flows(i) = flow_in;
    prev_error = error;
end

% Plot
figure;
plot(time, levels, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Water Level (m)');
title('PID-Controlled Water Tank Level');
grid on;

% Export data
data = table(time', levels', errors', flows', ...
    'VariableNames', {'Time_s', 'Level_m', 'Error_m', 'Inflow_m3ps'});
writetable(data, 'sim_output.csv');
