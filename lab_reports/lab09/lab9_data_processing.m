close all
clear

% experimental data
filename = 'data.xlsx';
[~, sheets] = xlsfinfo(filename);
figure;
set(gcf, 'Position', [50, 0, 700, 500]);
hold on;

if 0
    for i = 2:length(sheets)
        exp_data = readtable(filename, 'Sheet', sheets{i});
        t = exp_data{:, 1};
        rpm = exp_data{:, 2};    
        plot(t, rpm, 'DisplayName', sheets{i});
    end
    title('Motor Response to Step Input at Different Duty Cycles');
else
    % simulink vs exp
    pwm100 = readtable(filename, 'Sheet', 'pwm100');
    exp_t = pwm100{:, 1};
    exp_rpm = pwm100{:, 2};
    load('simout.mat')
    sim_t = data.time;
    sim_rpm = data.data;
    sim_t = sim_t(sim_t <= 2.56);
    sim_rpm = sim_rpm(sim_t <= 2.56);
    plot(exp_t, exp_rpm, 'DisplayName', 'experiment')
    plot(sim_t, sim_rpm, 'DisplayName', 'simulation', 'LineWidth', 1.5)
    title('Experimental vs Simulated Response for 100% Duty Cycle');
end

xlabel('Time (s)');
ylabel('Speed (RPM)');
legend('Location', 'southeast');
grid on;
hold off;