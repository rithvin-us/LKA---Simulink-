% ==============================================================================
% create_lka_buses.m
% Generates Simulink Bus objects for modular model referencing and MAAB compliance.
%
% LKA system buses (ISO 8855 vehicle frame: x forward, y left, rear-axle origin):
%   LaneBoundaryBus - one detected lane boundary as a clothoid
%                     y(x) = LateralOffset + tan(HeadingAngle)*x + Curvature*x^2/2
%                            + CurvatureDerivative*x^3/6
%                     (same fields as Automated Driving Toolbox lane boundaries)
%   LaneSensorBus   - lane detector output: Left + Right boundary
%   LaneEstimateBus - tracked lane centre (Kalman filter output)
%   LkaInfoBus      - LKA status, intermediate signals and turning radius
% Legacy buses (script baseline): LaneDetectionsBus, VehicleStateBus, SteeringCmdBus.
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

    % 4. Lane Boundary Bus (one boundary, clothoid model)
    LaneBoundaryBus = makeBus('One lane boundary as a clothoid in the vehicle frame', { ...
        'LateralOffset',       'double', 'm',      'Boundary y at x = 0 (+ left)'
        'HeadingAngle',        'double', 'rad',    'Boundary heading relative to vehicle x axis (+ CCW)'
        'Curvature',           'double', '1/m',    'Boundary curvature (+ turning left)'
        'CurvatureDerivative', 'double', '1/m^2',  'Curvature rate along the boundary'
        'Strength',            'double', '',       'Detection strength 0..1 (0 = not detected)'
        'ViewRange',           'double', 'm',      'Look-ahead distance up to which the boundary is detected'});
    assignin('base', 'LaneBoundaryBus', LaneBoundaryBus);

    % 5. Lane Sensor Bus (detector output)
    elems = Simulink.BusElement.empty;
    elems(1) = Simulink.BusElement; elems(1).Name = 'Left';  elems(1).DataType = 'Bus: LaneBoundaryBus';
    elems(2) = Simulink.BusElement; elems(2).Name = 'Right'; elems(2).DataType = 'Bus: LaneBoundaryBus';
    elems(3) = Simulink.BusElement; elems(3).Name = 'FrameValid'; elems(3).DataType = 'boolean';
    LaneSensorBus = Simulink.Bus;
    LaneSensorBus.Elements = elems;
    LaneSensorBus.Description = 'Lane detector output: left and right ego-lane boundaries';
    assignin('base', 'LaneSensorBus', LaneSensorBus);

    % 6. Lane Estimate Bus (tracked lane centre)
    LaneEstimateBus = makeBus('Tracked ego-lane centre at the rear axle', { ...
        'CenterOffset',        'double', 'm',      'Lane-centre y at x = 0 (+ centre is left of vehicle)'
        'HeadingAngle',        'double', 'rad',    'Lane-centre heading relative to vehicle (+ CCW)'
        'Curvature',           'double', '1/m',    'Lane-centre curvature (+ left)'
        'CurvatureDerivative', 'double', '1/m^2',  'Lane-centre curvature rate'
        'LaneWidth',           'double', 'm',      'Estimated lane width'
        'Quality',             'double', '',       'Estimate quality 0..1'
        'CoastTime',           'double', 's',      'Time since last accepted measurement'
        'NumBoundaries',       'double', '',       'Boundaries used this frame (0, 1, 2)'
        'MeasValid',           'boolean', '',      'A measurement was fused this frame'});
    assignin('base', 'LaneEstimateBus', LaneEstimateBus);

    % 7. LKA Info Bus (status + intermediate signals)
    LkaInfoBus = makeBus('LKA status, intermediate control signals and turning radius', { ...
        'State',          'uint8',   '',       '0 OFF, 1 STANDBY, 2 ACTIVE, 3 DEGRADED, 4 TAKEOVER'
        'Active',         'boolean', '',       'LKA has steering authority'
        'Authority',      'double',  '',       'Authority ramp 0..1'
        'ControllerSelect','double', '',       '1 pure pursuit, 2 Stanley 2025, 3 MPC'
        'CurvatureReq',   'double',  '1/m',    'Curvature requested by the selected controller'
        'CurvaturePP',    'double',  '1/m',    'Pure pursuit curvature (incl. integral)'
        'CurvatureStanley','double', '1/m',    'Stanley 2025 curvature'
        'CurvatureMPC',   'double',  '1/m',    'MPC curvature'
        'CurvatureFF',    'double',  '1/m',    'Road curvature feed-forward (lane estimate)'
        'CurvatureCmd',   'double',  '1/m',    'Curvature after a_y / jerk limiter'
        'CurvatureLimit', 'double',  '1/m',    'Active curvature magnitude limit'
        'Lookahead',      'double',  'm',      'Pure pursuit look-ahead distance'
        'TargetOffset',   'double',  'm',      'Lane-centre offset at the look-ahead point'
        'SteerReq',       'double',  'rad',    'Road-wheel angle before actuator manager'
        'SteerCmd',       'double',  'rad',    'Road-wheel angle command (output)'
        'CurvatureOut',   'double',  '1/m',    'Path curvature implied by SteerCmd'
        'TurnRadius',     'double',  'm',      'Turning radius implied by SteerCmd (+ left, clipped)'
        'LatAccelCmd',    'double',  'm/s^2',  'v^2 * CurvatureOut'
        'Saturated',      'boolean', '',       'Steering at angle limit (anti-windup)'
        'LdwLeft',        'boolean', '',       'Lane departure warning, left'
        'LdwRight',       'boolean', '',       'Lane departure warning, right'
        'DlcLeft',        'double',  'm',      'Distance to left line crossing (wheel edge)'
        'DlcRight',       'double',  'm',      'Distance to right line crossing (wheel edge)'
        'TlcLeft',        'double',  's',      'Time to left line crossing'
        'TlcRight',       'double',  's',      'Time to right line crossing'});
    assignin('base', 'LkaInfoBus', LkaInfoBus);

end

function bus = makeBus(description, spec)
    elems = Simulink.BusElement.empty;
    for k = 1:size(spec, 1)
        e = Simulink.BusElement;
        e.Name = spec{k, 1};
        e.DataType = spec{k, 2};
        e.Unit = spec{k, 3};
        e.Description = spec{k, 4};
        elems(k) = e; %#ok<AGROW>
    end
    bus = Simulink.Bus;
    bus.Elements = elems;
    bus.Description = description;
end
