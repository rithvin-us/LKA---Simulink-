function files = plot_lka_results(res, kpi, outDir)
% PLOT_LKA_RESULTS Result figures of one LKA MIL run.
%
% Syntax:
%   files = plot_lka_results(res, kpi, outDir)
%
% Figure 1 "Lane keeping overview"
%   track plan view, vehicle tyre envelope inside the lane, true vs estimated
%   lateral offset, tyre-to-line margin, heading error, path curvature
%   (road / feed-forward / command / output), turning radius vs road radius,
%   steering command vs actual, speed and LKA state.
% Figure 2 "Perception and supervision"
%   raw detector boundaries and strengths, lane-centre estimate vs truth,
%   curvature estimate, tracker quality and coast time, supervisor state and
%   lane departure warnings, distance / time to line crossing.
%
% Curved sections of the road are shaded in every time plot.
% PNG files are written to outDir (if given) and their paths returned.

    if nargin < 3, outDir = ''; end
    files = {};
    tag = sprintf('%s - %s', strrep(res.Scenario, '_', ' '), res.ControllerName);
    gt = res.gt; c = res.ctl; road = res.Road; veh = res.vehicle;
    edge = 0.5 * veh.trackWidth + 0.5 * veh.tireWidth;
    limit = 0.5 * median(road.width) - 0.5 * road.lineWidth - edge;   % max |offset| with all tyres in

    col = lines(5);
    cRoad = [0.35 0.35 0.35]; cTrue = col(1,:); cEst = col(2,:); cCmd = col(5,:); cLim = [0.8 0.1 0.1];

    % ============================================================== Figure 1
    f1 = figure('Name', ['LKA overview: ' tag], 'Color', 'w', 'Position', [40 40 1500 1000]);
    lightTheme(f1);
    tl = tiledlayout(f1, 5, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, sprintf('LKA closed loop - %s', tag), 'FontWeight', 'bold');
    subtitle(tl, sprintf(['max |lat| %.3f m | RMS %.3f m | min tyre-to-line %.3f m | in lane %.1f %% | ' ...
        'max a_y %.2f m/s^2 | max jerk(0.5 s) %.2f m/s^3 | min R %.0f m (road %.0f m)'], ...
        kpi.MaxLatOffset_m, kpi.RmsLatOffset_m, kpi.MinWheelMargin_m, 100 * kpi.InLaneRatio, ...
        kpi.MaxLatAccel_mps2, kpi.MaxLatJerk05_mps3, kpi.MinTurnRadius_m, kpi.MinRoadRadius_m));

    % --- plan view
    ax = nexttile(tl, [1 2]); hold(ax, 'on'); grid(ax, 'on');
    plot(ax, road.xL, road.yL, '-', 'Color', cRoad, 'LineWidth', 1.2);
    plot(ax, road.xR, road.yR, '-', 'Color', cRoad, 'LineWidth', 1.2);
    plot(ax, road.x, road.y, '--', 'Color', [0.6 0.6 0.6]);
    xr = gt.x - veh.b * cos(gt.psi); yr = gt.y - veh.b * sin(gt.psi);
    plot(ax, xr, yr, '-', 'Color', cTrue, 'LineWidth', 1.6);
    sEnd = max(gt.station);
    keep = road.s <= sEnd + 20;
    xlim(ax, [min(road.x(keep)) max(road.x(keep))]);
    xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]');
    title(ax, 'Plan view (axes not to scale): lane lines, lane centre, rear-axle path');
    legend(ax, {'left line', 'right line', 'lane centre', 'vehicle path'}, 'Location', 'best');

    % --- tyre envelope inside the lane
    ax = nexttile(tl, [1 2]); hold(ax, 'on'); grid(ax, 'on');
    shadeCurves(ax, gt.t, gt.roadCurv);
    halfLane = 0.5 * median(road.width) - 0.5 * road.lineWidth;
    yl = [-halfLane - 0.3, halfLane + 0.3];
    fill(ax, [gt.t; flipud(gt.t)], [gt.latOffset - edge; flipud(gt.latOffset + edge)], cTrue, ...
        'FaceAlpha', 0.25, 'EdgeColor', 'none');
    plot(ax, gt.t, gt.latOffset, '-', 'Color', cTrue, 'LineWidth', 1.2);
    yline(ax, halfLane, '-', 'Color', cRoad, 'LineWidth', 2);
    yline(ax, -halfLane, '-', 'Color', cRoad, 'LineWidth', 2);
    yline(ax, 0, ':', 'Color', [0.5 0.5 0.5]);
    ylim(ax, yl); xlabel(ax, 't [s]'); ylabel(ax, 'lateral position [m]');
    title(ax, 'Vehicle tyre envelope (shaded) between the inner edges of the lane lines (+ left)');

    % --- lateral offset
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    shadeCurves(ax, gt.t, gt.roadCurv);
    plot(ax, gt.t, gt.latOffset, 'Color', cTrue, 'LineWidth', 1.2);
    plot(ax, c.t, -c.EstCenterOffset, 'Color', cEst);
    yline(ax, [-limit limit], '--', 'Color', cLim);
    xlabel(ax, 't [s]'); ylabel(ax, 'offset [m]');
    title(ax, 'Lateral offset from lane centre (+ left)');
    legend(ax, {'true (CG)', 'estimated (rear axle)', 'tyre on line'}, 'Location', 'best');

    % --- margins
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    shadeCurves(ax, gt.t, gt.roadCurv);
    plot(ax, gt.t, gt.margins, 'LineWidth', 1);
    yline(ax, 0, '-', 'Color', cLim, 'LineWidth', 1.5);
    xlabel(ax, 't [s]'); ylabel(ax, 'margin [m]');
    title(ax, 'Tyre outer edge to line inner edge (< 0 = outside lane, rulebook K.2.4)');
    legend(ax, {'front left', 'front right', 'rear left', 'rear right'}, 'Location', 'best', 'NumColumns', 2);

    % --- curvature
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    shadeCurves(ax, gt.t, gt.roadCurv);
    plot(ax, gt.t, gt.roadCurv, 'Color', cRoad, 'LineWidth', 1.5);
    plot(ax, c.t, c.CurvatureFF, 'Color', cEst);
    plot(ax, c.t, c.CurvatureCmd, 'Color', cCmd, 'LineWidth', 1.1);
    plot(ax, c.t, c.CurvatureOut, '--', 'Color', cTrue);
    xlabel(ax, 't [s]'); ylabel(ax, '\kappa [1/m]');
    title(ax, 'Path curvature (+ left)');
    legend(ax, {'road (true)', 'feed-forward (estimate)', 'command (limited)', 'from steering cmd'}, 'Location', 'best');

    % --- turning radius
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    shadeCurves(ax, gt.t, gt.roadCurv);
    Rroad = 1 ./ max(abs(gt.roadCurv), 1 / res.LKA.Out.RadiusMax);
    plot(ax, gt.t, Rroad, 'Color', cRoad, 'LineWidth', 1.5);
    Rl = abs(c.TurnRadius); Rl(c.TurnRadius < 0) = NaN;
    Rr = abs(c.TurnRadius); Rr(c.TurnRadius >= 0) = NaN;
    plot(ax, c.t, Rl, '.', 'Color', col(4,:), 'MarkerSize', 5);
    plot(ax, c.t, Rr, '.', 'Color', col(3,:), 'MarkerSize', 5);
    set(ax, 'YScale', 'log'); ylim(ax, [5 res.LKA.Out.RadiusMax * 1.2]);
    xlabel(ax, 't [s]'); ylabel(ax, '|R| [m]');
    title(ax, 'LKA turning radius output vs road radius (1000 m = straight)');
    legend(ax, {'road radius', 'LKA radius, turning left', 'LKA radius, turning right'}, 'Location', 'best');

    % --- steering
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    shadeCurves(ax, gt.t, gt.roadCurv);
    plot(ax, c.t, rad2deg(c.SteerCmd), 'Color', cCmd, 'LineWidth', 1.1);
    plot(ax, gt.t, rad2deg(gt.steer), '--', 'Color', cTrue);
    xlabel(ax, 't [s]'); ylabel(ax, '\delta [deg]');
    title(ax, sprintf('Road-wheel angle (+ left), limit \\pm%.0f deg', rad2deg(res.LKA.Veh.SteerMax)));
    legend(ax, {'LKA command', 'actuator output'}, 'Location', 'best');

    % --- speed + state
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    yyaxis(ax, 'left');
    plot(ax, gt.t, 3.6 * gt.speed, 'LineWidth', 1.2);
    yline(ax, 30, ':'); yline(ax, 33, '--', 'Color', cLim);
    ylabel(ax, 'speed [km/h]'); ylim(ax, [0 40]);
    yyaxis(ax, 'right');
    stairs(ax, c.t, c.State, 'LineWidth', 1.2);
    ylim(ax, [-0.5 4.5]); yticks(ax, 0:4);
    yticklabels(ax, {'OFF', 'STANDBY', 'ACTIVE', 'DEGRADED', 'TAKEOVER'});
    xlabel(ax, 't [s]');
    title(ax, 'Speed (33 km/h = DQ) and LKA state');

    finishShading(f1);
    files{end+1} = save(f1, outDir, sprintf('%s_%s_overview.png', res.Scenario, safe(res.ControllerName)));

    % ============================================================== Figure 2
    f2 = figure('Name', ['LKA perception: ' tag], 'Color', 'w', 'Position', [60 60 1500 900]);
    lightTheme(f2);
    tl = tiledlayout(f2, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, sprintf('Lane perception and supervision - %s', tag), 'FontWeight', 'bold');
    subtitle(tl, sprintf('estimator RMS offset error %.3f m | availability %.1f %% | take-over requests %d | LDW events %d', ...
        kpi.EstOffsetRmsErr_m, 100 * kpi.Availability, kpi.TakeOverRequests, kpi.LdwEvents));

    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    yyaxis(ax, 'left');
    plot(ax, c.t, c.SenLeftOffset, '.', 'MarkerSize', 4);
    plot(ax, c.t, c.SenRightOffset, '.', 'MarkerSize', 4);
    ylabel(ax, 'boundary offset [m]');
    yyaxis(ax, 'right');
    stairs(ax, c.t, c.SenLeftStrength, '-'); stairs(ax, c.t, c.SenRightStrength, '--');
    ylim(ax, [-0.05 1.05]); ylabel(ax, 'strength');
    xlabel(ax, 't [s]'); title(ax, 'Detector output: left / right boundary offset and strength (0 = missed)');

    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    shadeCurves(ax, gt.t, gt.roadCurv);
    plot(ax, c.t, c.TrueCenterOffsetRear, 'Color', cTrue, 'LineWidth', 1.2);
    plot(ax, c.t, c.EstCenterOffset, 'Color', cEst);
    plot(ax, c.t, c.EstCenterOffset - c.TrueCenterOffsetRear, 'Color', [0.5 0.5 0.5]);
    xlabel(ax, 't [s]'); ylabel(ax, '[m]');
    title(ax, 'Lane-centre offset at rear axle: Kalman estimate vs truth');
    legend(ax, {'true', 'estimate', 'error'}, 'Location', 'best');

    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    plot(ax, c.t, c.TrueCurvature, 'Color', cTrue, 'LineWidth', 1.2);
    plot(ax, c.t, c.EstCurvature, 'Color', cEst);
    xlabel(ax, 't [s]'); ylabel(ax, '\kappa [1/m]');
    title(ax, 'Lane curvature: estimate vs truth');
    legend(ax, {'true', 'estimate'}, 'Location', 'best');

    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    yyaxis(ax, 'left'); plot(ax, c.t, c.EstQuality, 'LineWidth', 1.2); ylim(ax, [0 1.05]); ylabel(ax, 'quality');
    yyaxis(ax, 'right'); plot(ax, c.t, c.EstCoastTime, 'LineWidth', 1.0); ylabel(ax, 'coast time [s]');
    xlabel(ax, 't [s]'); title(ax, 'Tracker quality and time without measurement');

    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    stairs(ax, c.t, c.State, 'LineWidth', 1.4);
    stairs(ax, c.t, 0.9 * c.LdwLeft - 0.05, '--'); stairs(ax, c.t, 0.8 * c.LdwRight - 0.1, ':');
    stairs(ax, c.t, c.ForcedDropLeft * 0.7 + 4.6, '-'); stairs(ax, c.t, c.ForcedDropRight * 0.7 + 4.6, '-.');
    ylim(ax, [-0.3 5.5]); yticks(ax, 0:4);
    yticklabels(ax, {'OFF', 'STANDBY', 'ACTIVE', 'DEGRADED', 'TAKEOVER'});
    xlabel(ax, 't [s]'); title(ax, 'Supervisor state, LDW flags, forced detector outages (top)');
    legend(ax, {'state', 'LDW left', 'LDW right', 'outage left', 'outage right'}, 'Location', 'best');

    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    yyaxis(ax, 'left');
    plot(ax, c.t, c.DlcLeft); plot(ax, c.t, c.DlcRight, '--');
    yline(ax, res.LKA.Ldw.DlcWarn, ':'); ylabel(ax, 'DLC [m]');
    yyaxis(ax, 'right');
    plot(ax, c.t, min(c.TlcLeft, c.TlcRight)); ylim(ax, [0 10]); ylabel(ax, 'min TLC [s]');
    xlabel(ax, 't [s]'); title(ax, 'Distance and time to line crossing (tyre edge, estimated lane)');

    finishShading(f2);
    files{end+1} = save(f2, outDir, sprintf('%s_%s_perception.png', res.Scenario, safe(res.ControllerName)));
end

function shadeCurves(ax, t, k)
    % Remember the curved sections; drawn by finishShading once the data is plotted
    ax.UserData = struct('t', t, 'k', k);
end

function finishShading(fig)
    % Shade curved road sections behind the data, within the final y-limits
    axs = findall(fig, 'Type', 'axes');
    for a = axs'
        if ~isstruct(a.UserData) || ~isfield(a.UserData, 'k'), continue; end
        t = a.UserData.t; k = a.UserData.k;
        yl = ylim(a);
        on = abs(k) > 1e-4;
        d = diff([0; on(:); 0]);
        st = find(d == 1); en = find(d == -1) - 1;
        hold(a, 'on');
        for i = 1:numel(st)
            p = patch(a, t([st(i) en(i) en(i) st(i)]), yl([1 1 2 2]), [0.90 0.92 0.98], ...
                'EdgeColor', 'none', 'HandleVisibility', 'off');
            uistack(p, 'bottom');
        end
        ylim(a, yl);
    end
end

function lightTheme(fig)
    try
        theme(fig, 'light');
    catch
    end
end

function f = save(fig, outDir, name)
    f = '';
    if ~isempty(outDir)
        if ~isfolder(outDir), mkdir(outDir); end
        f = fullfile(outDir, name);
        exportgraphics(fig, f, 'Resolution', 130);
    end
end

function s = safe(s)
    s = regexprep(s, '[^A-Za-z0-9]+', '');
end
