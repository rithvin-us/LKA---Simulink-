function lka_monitor_publish(action, data, frame)
% LKA_MONITOR_PUBLISH Live telemetry for the LKA dashboard (tools/lka_dashboard.py).
%
% Syntax:
%   lka_monitor_publish('start', info)          new run: info.Name, Kind ('video' | 'sim'),
%                                               Detector, Gpu, OutDir, Frames, Duration, Phase
%   lka_monitor_publish('phase', text)          e.g. 'writing annotated video' (clears the series)
%   lka_monitor_publish('sample', s)            one frame: struct of scalars (Time, Steer, ...)
%   lka_monitor_publish('sample', s, frame)     ... with the annotated camera frame (uint8, or a
%                                               function handle that makes it, only called when
%                                               the frame is actually written)
%   lka_monitor_publish('done', summary)
%
% Writes results/monitor/live.json (run, phase, latest sample, the last 1200 samples of
% every field) and results/monitor/live.jpg. Files are written to a temporary name and
% renamed, so the dashboard never reads half a file. JSON at most every 0.2 s, the frame
% at most every 0.4 s (start / phase / done are always written), so publishing costs the
% LKA loop well under 1 ms per frame on average. Errors are swallowed: monitoring must
% never stop a run.
    persistent run series latest lastJson lastFrame phase
    try
        dirOut = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'results', 'monitor');
        if ~isfolder(dirOut)
            mkdir(dirOut);
        end
        now_ = posixtime(datetime('now'));
        force = false;
        switch lower(action)
            case 'start'
                run = data;
                run.Started = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
                series = struct();
                latest = struct();
                phase = 'running';
                if isfield(data, 'Phase'), phase = data.Phase; end
                lastJson = 0; lastFrame = 0;
                force = true;
            case 'phase'
                phase = data;
                series = struct();
                force = true;
            case 'sample'
                if isempty(run)
                    return
                end
                latest = data;
                f = fieldnames(data);
                for i = 1:numel(f)
                    v = double(data.(f{i}));
                    if ~isscalar(v), continue; end
                    if ~isfield(series, f{i}), series.(f{i}) = zeros(1, 0); end
                    series.(f{i})(end + 1) = v;
                    if numel(series.(f{i})) > 1200
                        series.(f{i}) = series.(f{i})(end - 1199:end);
                    end
                end
                if nargin >= 3 && now_ - lastFrame >= 0.4
                    if isa(frame, 'function_handle')
                        frame = frame();
                    end
                    writeAtomic(fullfile(dirOut, 'live.jpg'), @(p) imwrite(frame, p, 'jpg', 'Quality', 80));
                    lastFrame = now_;
                end
            case 'done'
                phase = 'done';
                run.Summary = data;
                run.Finished = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
                force = true;
            otherwise
                error('lka:monitor:action', 'Unknown action "%s".', action);
        end
        if force || now_ - lastJson >= 0.2
            s = struct('Run', run, 'Phase', phase, 'Latest', latest, 'Series', series, 'Updated', now_);
            txt = jsonencode(s);
            writeAtomic(fullfile(dirOut, 'live.json'), @(p) writeText(p, txt));
            lastJson = now_;
        end
    catch
        % monitoring is best effort
    end
end

function writeAtomic(file, writer)
    [d, n, e] = fileparts(file);
    tmp = fullfile(d, [n '.tmp' e]);
    writer(tmp);
    movefile(tmp, file, 'f');
end

function writeText(file, txt)
    fid = fopen(file, 'w', 'n', 'UTF-8');
    fwrite(fid, txt, 'char');
    fclose(fid);
end
