function url = start_lka_dashboard(port)
% START_LKA_DASHBOARD Starts the local LKA monitoring dashboard and opens it in the browser.
%
% Syntax:
%   start_lka_dashboard            % http://127.0.0.1:8050
%   url = start_lka_dashboard(8060)
%
% Runs tools/lka_dashboard.py (Python standard library only) in its own process, so
% MATLAB stays free for run_lka_on_video / run_lka_vision, which publish their live
% telemetry with lka_monitor_publish. The page shows the annotated camera frame, the
% LKA signals, the time per pipeline stage, the GPU (nvidia-smi) and the finished
% results (test_res/summary.csv, results/vision/lka_vision_kpis.csv). The server only
% listens on 127.0.0.1. Stop it by closing its console window.
    if nargin < 1
        port = 8050;
    end
    url = sprintf('http://127.0.0.1:%d', port);
    root = fileparts(fileparts(mfilename('fullpath')));
    script = fullfile(root, 'tools', 'lka_dashboard.py');
    if ~isRunning(url)
        py = 'python';
        try
            pe = pyenv;
            if strlength(pe.Executable) > 0
                py = char(pe.Executable);
            end
        catch
        end
        if ispc
            cmd = sprintf('start "LKA dashboard" /min "%s" "%s" --port %d', py, script, port);
        else
            cmd = sprintf('"%s" "%s" --port %d > /dev/null 2>&1 &', py, script, port);
        end
        status = system(cmd);
        if status ~= 0
            error('lka:dashboard:start', 'Could not start %s with %s.', script, py);
        end
        t0 = tic;
        while ~isRunning(url) && toc(t0) < 10
            pause(0.3);
        end
        if ~isRunning(url)
            error('lka:dashboard:start', 'Dashboard did not answer on %s.', url);
        end
    end
    fprintf('[LKA dashboard] %s\n', url);
    web(url, '-browser');
end

function ok = isRunning(url)
    try
        webread([url '/api/live'], weboptions('Timeout', 1));
        ok = true;
    catch
        ok = false;
    end
end
