function kpis = evaluate_kpis(t, lateralError, latAccel, steerAngle, controllerParams)
% EVALUATE_KPIS Computes performance metrics for LKA according to ISO 11270 guidelines.
%
% References:
%   ISO 11270:2014 - LKAS Performance requirements & limits.

    kpis = struct();

    % Compute metrics
    kpis.maxLatError  = max(abs(lateralError)); % Maximum absolute lateral deviation [m]
    kpis.rmsLatError  = sqrt(mean(lateralError.^2)); % RMS lateral deviation [m]
    kpis.maxLatAccel  = max(abs(latAccel)); % Peak lateral acceleration [m/s^2]
    kpis.maxSteerDeg  = max(abs(rad2deg(steerAngle))); % Peak steer angle [deg]

    % ISO 11270 Pass/Fail Criteria
    accelLimit = controllerParams.maxLateralAccel; % 3.0 m/s^2
    laneDepartureLimit = 0.35; % m

    kpis.passAccelLimit = kpis.maxLatAccel <= accelLimit;
    kpis.passLaneLimit  = kpis.maxLatError <= laneDepartureLimit;
    kpis.overallPass    = kpis.passAccelLimit && kpis.passLaneLimit;

    % Print Summary Report
    fprintf('\n----------------- LKA KPI SUMMARY (ISO 11270) -----------------\n');
    fprintf('Max Lateral Cross-Track Error: %6.3f m  (Threshold: <= %.2f m) [%s]\n', ...
        kpis.maxLatError, laneDepartureLimit, passFailStr(kpis.passLaneLimit));
    fprintf('RMS Lateral Error:             %6.3f m\n', kpis.rmsLatError);
    fprintf('Peak Lateral Acceleration:     %6.3f m/s^2 (Threshold: <= %.2f m/s^2) [%s]\n', ...
        kpis.maxLatAccel, accelLimit, passFailStr(kpis.passAccelLimit));
    fprintf('Peak Steering Angle Command:   %6.2f deg\n', kpis.maxSteerDeg);
    fprintf('Overall ISO 11270 Compliance:  [%s]\n', passFailStr(kpis.overallPass));
    fprintf('----------------------------------------------------------------\n\n');

end

function str = passFailStr(isPassing)
    if isPassing
        str = 'PASS';
    else
        str = 'FAIL';
    end
end
