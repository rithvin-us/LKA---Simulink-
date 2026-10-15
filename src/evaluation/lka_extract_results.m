function res = lka_extract_results(out, sc, LKA, vehicle)
% LKA_EXTRACT_RESULTS Flattens the logged signals of one LKA_MIL_TestBench run.
%
% Syntax:
%   res = lka_extract_results(out, sc, LKA, vehicle)
%
% Inputs:
%   out     - Simulink.SimulationOutput of LKA_MIL_TestBench
%   sc      - scenario element from generate_lka_scenarios
%   LKA     - calibration used for the run
%   vehicle - plant parameters used for the run
%
% Output:
%   res - struct with plain column vectors grouped by rate:
%         res.gt   ground truth and plant signals (10 ms)
%         res.ctl  controller signals (LKA.Ts)

    ls = out.logsout;
    g = @(n) ls.getElement(n).Values;

    names = {'Pure Pursuit', 'Stanley 2025', 'MPC'};
    res = struct();
    res.Scenario = sc.Name;
    res.Description = sc.Description;
    res.Road = sc.Road;
    res.ControllerSelect = LKA.ControllerSelect;
    res.ControllerName = names{LKA.ControllerSelect};
    res.LKA = LKA;
    res.vehicle = vehicle;

    % ---------------- ground truth / plant (10 ms metrics rate)
    lat = g('gt_lat_offset');
    t = lat.Time;
    gt.t = t;
    gt.latOffset   = col(lat);
    gt.headingErr  = col(g('gt_heading_err'));
    gt.margins     = rows(g('gt_wheel_margins'), 4);
    gt.inLane      = logical(col(g('gt_in_lane')));
    gt.station     = col(g('gt_station'));
    gt.roadCurv    = col(g('gt_road_curvature'));
    pose = rows(g('cg_pose'), 3);
    tp = g('cg_pose').Time;
    gt.x   = interp1(tp, pose(:,1), t);
    gt.y   = interp1(tp, pose(:,2), t);
    gt.psi = interp1(tp, pose(:,3), t);
    gt.speed     = resampleTo(g('speed'), t);
    gt.steer     = resampleTo(g('steer_actual'), t);
    gt.yawRate   = resampleTo(g('yaw_rate'), t);
    gt.latAccel  = resampleTo(g('lat_accel'), t);
    res.gt = gt;

    % ---------------- controller (LKA.Ts)
    info = g('lka_info');
    tc = info.SteerCmd.Time;
    ctl.t = tc;
    f = fieldnames(info);
    for k = 1:numel(f)
        ctl.(f{k}) = double(col(info.(f{k})));
    end
    est = g('lane_estimate');
    fe = fieldnames(est);
    for k = 1:numel(fe)
        ctl.(['Est' fe{k}]) = double(resampleTo(est.(fe{k}), tc));
    end
    sen = g('lane_sensor');
    ctl.SenLeftOffset    = resampleTo(sen.Left.LateralOffset, tc);
    ctl.SenRightOffset   = resampleTo(sen.Right.LateralOffset, tc);
    ctl.SenLeftStrength  = resampleTo(sen.Left.Strength, tc);
    ctl.SenRightStrength = resampleTo(sen.Right.Strength, tc);
    drop = rows(g('dropout'), 2);
    td = g('dropout').Time;
    ctl.ForcedDropLeft  = interp1(td, drop(:,1), tc, 'previous', 'extrap');
    ctl.ForcedDropRight = interp1(td, drop(:,2), tc, 'previous', 'extrap');
    % Ground truth at the controller rate (rear-axle lane-centre offset, for estimator error)
    yc = interp1(t, -gt.latOffset, tc, 'linear', 'extrap');
    psiC = interp1(t, -gt.headingErr, tc, 'linear', 'extrap');
    ctl.TrueCenterOffsetRear = yc - psiC * vehicle.b;
    ctl.TrueHeading = psiC;
    ctl.TrueCurvature = interp1(t, gt.roadCurv, tc, 'linear', 'extrap');
    ctl.Station = interp1(t, gt.station, tc, 'linear', 'extrap');
    ctl.Speed = interp1(t, gt.speed, tc, 'linear', 'extrap');
    res.ctl = ctl;
end

function v = col(ts)
    v = squeeze(ts.Data);
    v = v(:);
end

function M = rows(ts, n)
    D = ts.Data;
    if ndims(D) == 3
        M = reshape(permute(D, [3 2 1]), [], n);
    elseif size(D, 2) == n
        M = D;
    else
        M = D.';
    end
end

function v = resampleTo(ts, t)
    v = interp1(ts.Time, double(squeeze(ts.Data)), t, 'linear', 'extrap');
    v = v(:);
end
