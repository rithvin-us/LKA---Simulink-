% ==============================================================================
% vehicle_params.m
% Physical parameters for a standard SAE / aBAJA All-Terrain Vehicle (ATV)
%
% References:
%   1. Rajamani, R. (2012). Vehicle Dynamics and Control. Springer.
%   2. Gillespie, T. D. (1992). Fundamentals of Vehicle Dynamics. SAE.
%
% Cross-check: the 2025 IPG CarMaker Baja (262067_VelocitaRacing.car) measured
% wheelbase 1.341 m, track 1.31 m, ~347 kg. Update here if the 2026 car differs.
% ==============================================================================

vehicle = struct();

% --- Inertial and Mass Properties ---
vehicle.mass = 240.0;           % Total vehicle mass including driver [kg]
vehicle.Iz   = 180.0;           % Yaw moment of inertia around z-axis [kg*m^2]

% --- Geometry Dimensions ---
vehicle.wheelbase = 1.45;       % Wheelbase L = a + b [m]
vehicle.trackWidth = 1.30;      % Track width [m]
vehicle.a = 0.75;               % Distance from C.G. to front axle [m]
vehicle.b = vehicle.wheelbase - vehicle.a; % Distance from C.G. to rear axle [m]
vehicle.cgHeight = 0.40;        % Height of center of gravity [m]
vehicle.tireWidth = 0.20;       % Tire section width (23x7-10 / 22x8-10 class) [m]
vehicle.wheelRadius = 0.29;     % Loaded wheel radius [m]

% --- Tire Cornering Properties (Linear Tire Model / Pacejka approximation) ---
% Front and rear total axle cornering stiffness [N/rad]
vehicle.Cf = 22000.0;           % Front axle cornering stiffness [N/rad]
vehicle.Cr = 26000.0;           % Rear axle cornering stiffness [N/rad]
vehicle.mu = 0.8;               % Tire-road friction coefficient (dry asphalt) [-]

% --- Steering Actuation Limits ---
vehicle.maxSteerAngle = deg2rad(32.0); % Maximum road wheel steer angle [rad] (Asterix hardware: 32 deg)
vehicle.maxSteerRate  = deg2rad(60.0); % Maximum steering angular velocity [rad/s]
vehicle.steerLagTau   = 0.10;          % Steering actuator first-order lag [s]

% --- Operating Conditions ---
vehicle.nominalSpeed = 8.0;     % Nominal vehicle longitudinal speed Vx [m/s] (~28.8 km/h)

% Assign to base workspace
assignin('base', 'vehicle', vehicle);
