function path = lka_add_ego_sensors(sys, name, pos)
% LKA_ADD_EGO_SENSORS Adds the wheel-speed + gyro sensor-model subsystem to a model.
%
% Syntax:
%   path = lka_add_ego_sensors(sys, name, pos)
%
% Inputs Speed [m/s], YawRate [rad/s] (sample time LKA.Ts) -> outputs SpeedMeas, YawRateMeas
% through lka_ego_sensor_model with seeded Random Number blocks (LKA.EgoSen.Seed).
    path = [sys '/' name];
    add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', pos);
    delete_line(path, 'In1/1', 'Out1/1');
    delete_block([path '/In1']);
    delete_block([path '/Out1']);
    add_block('simulink/Sources/In1', [path '/Speed'], 'Port', '1', 'Position', [20 40 50 54]);
    add_block('simulink/Sources/In1', [path '/YawRate'], 'Port', '2', 'Position', [20 110 50 124]);
    add_block('simulink/Sources/Random Number', [path '/Speed Noise'], 'Mean', '0', 'Variance', '1', ...
        'Seed', 'LKA.EgoSen.Seed(1)', 'SampleTime', 'LKA.Ts', 'Position', [20 180 60 210]);
    add_block('simulink/Sources/Random Number', [path '/Gyro Noise'], 'Mean', '0', 'Variance', '1', ...
        'Seed', 'LKA.EgoSen.Seed(2)', 'SampleTime', 'LKA.Ts', 'Position', [20 240 60 270]);
    add_block('simulink/Signal Routing/Mux', [path '/Noise'], 'Inputs', '2', 'Position', [110 180 115 270]);
    mf = [path '/Sensor Model'];
    add_block('simulink/User-Defined Functions/MATLAB Function', mf, 'Position', [200 40 360 270]);
    ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', mf);
    ch.Script = strjoin({
        'function [vMeas, rMeas] = fcn(v, r, n, LKA)'
        '% Wheel-speed sensor and gyro with scale error, bias and noise (lka_ego_sensor_model.m)'
        '[vMeas, rMeas] = lka_ego_sensor_model(v, r, n, LKA.EgoSen);'}, newline);
    d = ch.find('-isa', 'Stateflow.Data', 'Name', 'LKA');
    d.Scope = 'Parameter';
    d.Tunable = false;
    add_block('simulink/Sinks/Out1', [path '/SpeedMeas'], 'Port', '1', 'Position', [440 60 470 74]);
    add_block('simulink/Sinks/Out1', [path '/YawRateMeas'], 'Port', '2', 'Position', [440 200 470 214]);
    add_line(path, 'Speed/1', 'Sensor Model/1');
    add_line(path, 'YawRate/1', 'Sensor Model/2');
    add_line(path, 'Speed Noise/1', 'Noise/1');
    add_line(path, 'Gyro Noise/1', 'Noise/2');
    add_line(path, 'Noise/1', 'Sensor Model/3');
    add_line(path, 'Sensor Model/1', 'SpeedMeas/1');
    add_line(path, 'Sensor Model/2', 'YawRateMeas/1');
end
