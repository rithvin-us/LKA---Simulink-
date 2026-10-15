function file = plot_lka_comparison(runs, outDir)
% PLOT_LKA_COMPARISON Compares controllers on one scenario.
%
% Syntax:
%   file = plot_lka_comparison(runs, outDir)
%
% runs - struct array with fields res and kpi (same scenario, different controllers)
%
% Panels: true lateral offset, minimum tyre-to-line margin, commanded curvature
% vs road, road-wheel angle, and a KPI bar chart (max / RMS offset, max lateral
% jerk, share of time in lane).

    if nargin < 2, outDir = ''; end
    sc = runs(1).res.Scenario;
    f = figure('Name', ['LKA controller comparison: ' sc], 'Color', 'w', 'Position', [80 80 1500 900]);
    try, theme(f, 'light'); catch, end
    tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, sprintf('Controller comparison - %s', strrep(sc, '_', ' ')), 'FontWeight', 'bold');
    subtitle(tl, runs(1).res.Description);
    names = arrayfun(@(r) r.res.ControllerName, runs, 'UniformOutput', false);
    col = lines(numel(runs));

    ax = nexttile(tl, [1 2]); hold(ax, 'on'); grid(ax, 'on');
    for k = 1:numel(runs)
        plot(ax, runs(k).res.gt.t, runs(k).res.gt.latOffset, 'Color', col(k,:), 'LineWidth', 1.2);
    end
    veh = runs(1).res.vehicle; road = runs(1).res.Road;
    lim = 0.5 * median(road.width) - 0.5 * road.lineWidth - 0.5 * veh.trackWidth - 0.5 * veh.tireWidth;
    yline(ax, [-lim lim], '--', 'Color', [0.8 0.1 0.1]);
    xlabel(ax, 't [s]'); ylabel(ax, 'lateral offset [m]');
    title(ax, 'True lateral offset from lane centre (+ left); dashed = a tyre touches the line');
    legend(ax, [names, {'tyre on line'}], 'Location', 'best');

    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    for k = 1:numel(runs)
        plot(ax, runs(k).res.gt.t, min(runs(k).res.gt.margins, [], 2), 'Color', col(k,:));
    end
    yline(ax, 0, 'Color', [0.8 0.1 0.1], 'LineWidth', 1.5);
    xlabel(ax, 't [s]'); ylabel(ax, '[m]'); title(ax, 'Smallest tyre-to-line margin');

    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    plot(ax, runs(1).res.gt.t, runs(1).res.gt.roadCurv, 'Color', [0.3 0.3 0.3], 'LineWidth', 1.5);
    for k = 1:numel(runs)
        plot(ax, runs(k).res.ctl.t, runs(k).res.ctl.CurvatureOut, 'Color', col(k,:));
    end
    xlabel(ax, 't [s]'); ylabel(ax, '\kappa [1/m]');
    title(ax, 'Commanded path curvature (= 1 / turning radius) vs road');
    legend(ax, [{'road'}, names], 'Location', 'best');

    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    for k = 1:numel(runs)
        plot(ax, runs(k).res.gt.t, rad2deg(runs(k).res.gt.steer), 'Color', col(k,:));
    end
    xlabel(ax, 't [s]'); ylabel(ax, '\delta [deg]'); title(ax, 'Road-wheel angle (actuator output)');

    ax = nexttile(tl);
    K = arrayfun(@(r) r.kpi, runs);
    data = [[K.MaxLatOffset_m]; [K.RmsLatOffset_m] * 5; [K.MaxLatJerk05_mps3] / 10; [K.InLaneRatio]]';
    b = bar(ax, data);
    set(ax, 'XTickLabel', names); grid(ax, 'on');
    legend(ax, {'max |offset| [m]', 'RMS offset x5 [m]', 'max jerk(0.5 s) / 10 [m/s^3]', 'time in lane [-]'}, ...
        'Location', 'bestoutside');
    title(ax, 'Key performance indicators');
    for i = 1:numel(b)
        xtips = b(i).XEndPoints; ytips = b(i).YEndPoints;
        text(ax, xtips, ytips, compose('%.2f', b(i).YData), 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', 'FontSize', 7);
    end

    file = '';
    if ~isempty(outDir)
        if ~isfolder(outDir), mkdir(outDir); end
        file = fullfile(outDir, sprintf('%s_comparison.png', sc));
        exportgraphics(f, file, 'Resolution', 130);
    end
end
