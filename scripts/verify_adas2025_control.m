function T = verify_adas2025_control(options)
% VERIFY_ADAS2025_CONTROL Closed-loop check of the 2025 ADAS_2026 steering chain.
%
% Syntax:
%   T = verify_adas2025_control()
%   T = verify_adas2025_control(Plot=false, MeasX=4)
%
% Runs the replica lka_adas2025_steer on the official LKA track (S1_Nominal, 0 -> 30 km/h)
% with the Baja single-track plant and the MIL steering actuator (rate limit, saturation,
% first-order lag). The lane measurement is ideal and metric (true lane geometry at MeasX
% ahead of the rear axle, the image bottom row of the 2025 pipeline), so every difference
% between the variants comes from the control chain:
%   as_coded         heading sign bug + 1 s mean + 3.0 m lane width + 5 m/s fallback speed
%   as_coded_no_mean as above without the 1 s mean
%   sign_fixed_mean  heading sign fixed, 1 s mean kept
%   fixed            sign fixed, per-frame command, 3.5 m width, measured speed
% Output: KPI table (also results/adas2025/adas2025_replica_kpis.csv and a figure).

    arguments
        options.Plot (1,1) logical = true
        options.OutDir char = ''
        options.MeasX (1,1) double = 4.0
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    if isempty(options.OutDir)
        options.OutDir = fullfile(root, 'results', 'adas2025');
    end
    run(fullfile(root, 'config', 'startup.m'));
    vehicle = evalin('base', 'vehicle');
    sc = generate_lka_scenarios(vehicle);
    sc = sc(strcmp({sc.Name}, 'S1_Nominal'));

    variants = {
        'as_coded',         'As coded: heading sign bug, 1 s mean, 3.0 m lane, 5 m/s', ...
            struct('HeadingBug', true,  'MeanWindow', 1.0, 'WidthScale', 3.0/3.5, 'AssumedSpeed', 5.0)
        'as_coded_no_mean', 'As coded without the 1 s mean', ...
            struct('HeadingBug', true,  'MeanWindow', 0,   'WidthScale', 3.0/3.5, 'AssumedSpeed', 5.0)
        'sign_fixed_mean',  'Heading sign fixed, 1 s mean kept', ...
            struct('HeadingBug', false, 'MeanWindow', 1.0, 'WidthScale', 3.0/3.5, 'AssumedSpeed', 5.0)
        'fixed',            'Sign fixed, per-frame command, 3.5 m lane, measured speed', ...
            struct('HeadingBug', false, 'MeanWindow', 0,   'WidthScale', 1.0,     'AssumedSpeed', NaN)};

    rows = cell(size(variants, 1), 1);
    traces = cell(size(rows));
    for k = 1:size(variants, 1)
        [P, st] = lka_adas2025_params();
        f = fieldnames(variants{k, 3});
        for j = 1:numel(f)
            P.(f{j}) = variants{k, 3}.(f{j});
        end
        [kpi, traces{k}] = simulateVariant(sc, vehicle, P, st, options.MeasX);
        rows{k} = catStruct(struct('Variant', string(variants{k, 1}), 'Description', string(variants{k, 2})), kpi);
    end
    T = struct2table(vertcat(rows{:}));
    disp(T(:, {'Variant', 'MaxLatOffset_m', 'InLaneRatio', 'FirstLineTouch_s', 'AllWheelsOut', 'MaxSteer_deg'}));
    if options.Plot
        if ~isfolder(options.OutDir), mkdir(options.OutDir); end
        writetable(T, fullfile(options.OutDir, 'adas2025_replica_kpis.csv'));
        plotVariants(traces, variants, vehicle, sc, options.OutDir);
    end
end

function [kpi, tr] = simulateVariant(sc, vehicle, P, st, measX)
    dtSim = 0.005;
    Ts = 0.05;
    nSub = round(Ts / dtSim);
    r = sc.Road;
    x = sc.P.X0(:);
    N = round(sc.P.StopTime / dtSim);
    idx = 0; idxCg = 0; t = 0;
    cmd = 0; cmdRl = 0; delta = 0;
    tr = struct('t', nan(N, 1), 'lat', nan(N, 1), 'inLane', false(N, 1), 'steer', nan(N, 1));
    lineTouch = NaN;
    allOut = false;
    for i = 1:N
        v = max(min(sc.P.V0 + sc.P.Accel * t, sc.P.VSet), 0.1);
        if mod(i - 1, nSub) == 0
            [~, ~, rear] = lka_single_track_dynamics(x, delta, v, vehicle);
            [cp, curv, w, ~, idx] = lka_road_closest_point(rear(1), rear(2), idx, ...
                r.s, r.x, r.y, r.psi, r.kappa, r.dkappa, r.width);
            [~, ~, c] = lka_lane_geometry(rear, cp, curv(1), curv(2), w);
            yc = c(1) + tan(c(2)) * measX + 0.5 * c(3) * measX^2;
            psiC = c(2) + c(3) * measX;
            [cmd, st] = lka_adas2025_steer(yc, psiC, v, x(5), st, P, Ts);
        end
        % MIL steering actuator: rate limit -> saturation -> first-order lag
        step = vehicle.maxSteerRate * dtSim;
        cmdRl = cmdRl + min(max(cmd - cmdRl, -step), step);
        cmdSat = min(max(cmdRl, -vehicle.maxSteerAngle), vehicle.maxSteerAngle);
        delta = delta + (cmdSat - delta) / vehicle.steerLagTau * dtSim;
        dx = lka_single_track_dynamics(x, delta, v, vehicle);
        x = x + dx * dtSim;
        t = t + dtSim;
        [cpg, curvg, wg, ~, idxCg] = lka_road_closest_point(x(1), x(2), idxCg, ...
            r.s, r.x, r.y, r.psi, r.kappa, r.dkappa, r.width);
        [lat, ~, margins, inLane] = lka_ground_truth_metrics(x(1:3)', cpg, curvg, wg, r.lineWidth, vehicle);
        tr.t(i) = t; tr.lat(i) = lat; tr.inLane(i) = inLane; tr.steer(i) = delta;
        if ~inLane && isnan(lineTouch)
            lineTouch = t;
        end
        allOut = allOut || all(lka_wheels_out(margins, vehicle.trackWidth));
        if abs(lat) > 5
            break                                   % left the road: the run is over
        end
    end
    keep = ~isnan(tr.t);
    tr = structfun(@(f) f(keep), tr, 'UniformOutput', false);
    moving = tr.t >= 0.5;
    kpi.MaxLatOffset_m = max(abs(tr.lat(moving)));
    kpi.InLaneRatio = mean(tr.inLane(moving));
    kpi.FirstLineTouch_s = lineTouch;
    kpi.AllWheelsOut = allOut;
    kpi.MaxSteer_deg = rad2deg(max(abs(tr.steer)));
end

function s = catStruct(a, b)
    s = a;
    f = fieldnames(b);
    for k = 1:numel(f)
        s.(f{k}) = b.(f{k});
    end
end

function plotVariants(traces, variants, vehicle, sc, outDir)
    f = figure('Color', 'w', 'Position', [100 100 1100 520]);
    try
        theme(f, 'light');
    catch
    end
    ax = axes(f); hold(ax, 'on'); grid(ax, 'on');
    limit = 0.5 * (sc.Road.width(1) - vehicle.trackWidth - vehicle.tireWidth - sc.Road.lineWidth);
    for k = 1:numel(traces)
        plot(ax, traces{k}.t, traces{k}.lat, 'LineWidth', 1.4, 'DisplayName', variants{k, 2});
    end
    yline(ax, [limit -limit], 'k--', 'HandleVisibility', 'off');
    ylim(ax, [-3 3]);
    xlabel(ax, 'time [s]'); ylabel(ax, 'CG lateral offset [m] (+ left)');
    title(ax, '2025 ADAS\_2026 steering chain on the official LKA track (replica, ideal measurement)');
    legend(ax, 'Location', 'southoutside');
    exportgraphics(f, fullfile(outDir, 'adas2025_replica_lateral_offset.png'), 'Resolution', 150);
    close(f);
end
