function kpi = lka_compute_kpis(res)
% LKA_COMPUTE_KPIS Lane-keeping key performance indicators of one run.
%
% Rulebook (aBAJA 2027 Rev00, K.2): any tyre touching the line = outside the lane;
% all four tyres out at any instant, a manual take-over or > 33 km/h = DQ; the
% score grows with the share of time spent inside the lane. The exact score
% formula did not survive PDF text extraction, so InLaneRatio = T_in / T_total
% is reported as the score proxy.
%
% Comfort / regulation: peak lateral acceleration and the 0.5 s moving-average
% lateral jerk (UNECE R79 ACSF B1: 3 m/s^2, 5 m/s^3).

    gt = res.gt; ctl = res.ctl;
    dt = median(diff(gt.t));
    moving = gt.speed > 0.5;
    if ~any(moving), moving = true(size(gt.t)); end
    tMove = gt.t(moving);

    kpi = struct();
    kpi.Scenario = res.Scenario;
    kpi.Controller = res.ControllerName;

    % -------- lane keeping (ground truth, CG)
    lat = gt.latOffset(moving);
    kpi.MaxLatOffset_m = max(abs(lat));
    kpi.RmsLatOffset_m = sqrt(mean(lat.^2));
    kpi.P95LatOffset_m = prctile(abs(lat), 95);
    kpi.MaxHeadingErr_deg = rad2deg(max(abs(gt.headingErr(moving))));
    kpi.MinWheelMargin_m = min(gt.margins(moving, :), [], 'all');

    kpi.TotalTime_s = numel(tMove) * dt;
    kpi.TimeInLane_s = sum(gt.inLane(moving)) * dt;
    kpi.InLaneRatio = kpi.TimeInLane_s / kpi.TotalTime_s;
    kpi.AnyWheelOut_s = sum(~gt.inLane(moving)) * dt;
    kpi.AllWheelsOut = any(all(gt.margins(moving, :) < 0, 2));

    % -------- speed
    kpi.MaxSpeed_kmh = 3.6 * max(gt.speed);
    kpi.SpeedDQ = kpi.MaxSpeed_kmh > 33;

    % -------- steering / comfort
    kpi.MaxSteer_deg = rad2deg(max(abs(gt.steer)));
    kpi.MaxSteerRate_degps = rad2deg(max(abs(gradient(gt.steer, dt))));
    kpi.MaxLatAccel_mps2 = max(abs(gt.latAccel(moving)));
    jerk = movmean(gradient(gt.latAccel, dt), max(1, round(0.5 / dt)));
    kpi.MaxLatJerk05_mps3 = max(abs(jerk(moving)));

    % -------- turning radius
    activeCtl = ctl.Active > 0.5 & ctl.Speed > 1;
    R = abs(ctl.TurnRadius(activeCtl));
    kpi.MinTurnRadius_m = min([R; inf]);
    kpi.MinRoadRadius_m = 1 / max(abs(gt.roadCurv(moving)));

    % -------- supervision and perception
    movingCtl = ctl.Speed > 0.5;
    kpi.Availability = mean(ismember(ctl.State(movingCtl), [2 3]));
    kpi.TakeOverRequests = sum(diff([0; ctl.State == 4]) == 1);
    ldw = ctl.LdwLeft > 0.5 | ctl.LdwRight > 0.5;
    kpi.LdwEvents = sum(diff([0; ldw]) == 1);
    estErr = ctl.EstCenterOffset - ctl.TrueCenterOffsetRear;
    valid = movingCtl & ctl.EstQuality > 0;
    kpi.EstOffsetRmsErr_m = sqrt(mean(estErr(valid).^2));

    % -------- verdict
    kpi.RulebookPass = kpi.AnyWheelOut_s == 0 && ~kpi.AllWheelsOut && ~kpi.SpeedDQ ...
        && kpi.TakeOverRequests == 0;
    kpi.ComfortPass = kpi.MaxLatAccel_mps2 <= 3.0 && kpi.MaxLatJerk05_mps3 <= 5.0;
end
