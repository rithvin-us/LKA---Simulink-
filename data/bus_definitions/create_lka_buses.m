% ==============================================================================
% create_lka_buses.m
% Generates Simulink Bus objects for modular model referencing and MAAB compliance.
% ==============================================================================

function create_lka_buses()

    % 1. Lane Detections Bus (Perception to Controller)
    clear elems;
    elems(1) = Simulink.BusElement;
    elems(1).Name = 'lateralOffset'; % [m]
    elems(1).DataType = 'double';
    
    elems(2) = Simulink.BusElement;
    elems(2).Name = 'headingError';  % [rad]
    elems(2).DataType = 'double';
    
    elems(3) = Simulink.BusElement;
    elems(3).Name = 'roadCurvature'; % [1/m]
    elems(3).DataType = 'double';
    
    elems(4) = Simulink.BusElement;
    elems(4).Name = 'confidence';    % [0 to 1]
    elems(4).DataType = 'double';
    
    LaneDetectionsBus = Simulink.Bus;
    LaneDetectionsBus.Elements = elems;
    LaneDetectionsBus.Description = 'Bus containing lane geometry detections from camera sensor';
    assignin('base', 'LaneDetectionsBus', LaneDetectionsBus);

    % 2. Vehicle State Bus (Vehicle Dynamics to Controller)
    clear elems;
    elems(1) = Simulink.BusElement;
    elems(1).Name = 'Vx';            % Longitudinal speed [m/s]
    elems(1).DataType = 'double';
    
    elems(2) = Simulink.BusElement;
    elems(2).Name = 'Vy';            % Lateral speed [m/s]
    elems(2).DataType = 'double';
    
    elems(3) = Simulink.BusElement;
    elems(3).Name = 'yawRate';       % Yaw rate r [rad/s]
    elems(3).DataType = 'double';
    
    elems(4) = Simulink.BusElement;
    elems(4).Name = 'yawAngle';      % Global yaw orientation psi [rad]
    elems(4).DataType = 'double';
    
    elems(5) = Simulink.BusElement;
    elems(5).Name = 'X';             % Global X coordinate [m]
    elems(5).DataType = 'double';
    
    elems(6) = Simulink.BusElement;
    elems(6).Name = 'Y';             % Global Y coordinate [m]
    elems(6).DataType = 'double';
    
    VehicleStateBus = Simulink.Bus;
    VehicleStateBus.Elements = elems;
    VehicleStateBus.Description = 'Bus containing 2-DOF / 3-DOF vehicle state feedback';
    assignin('base', 'VehicleStateBus', VehicleStateBus);

    % 3. Steering Command Bus (Controller to Actuator/Driver Overlay)
    clear elems;
    elems(1) = Simulink.BusElement;
    elems(1).Name = 'steerAngleCmd'; % Commanded front wheel angle [rad]
    elems(1).DataType = 'double';
    
    elems(2) = Simulink.BusElement;
    elems(2).Name = 'steerTorqueCmd';% Commanded assist torque [N*m]
    elems(2).DataType = 'double';
    
    elems(3) = Simulink.BusElement;
    elems(3).Name = 'isLkaActive';   % Active/Standby flag
    elems(3).DataType = 'boolean';
    
    SteeringCmdBus = Simulink.Bus;
    SteeringCmdBus.Elements = elems;
    SteeringCmdBus.Description = 'Bus delivering LKA steering demands to steering plant';
    assignin('base', 'SteeringCmdBus', SteeringCmdBus);

end
