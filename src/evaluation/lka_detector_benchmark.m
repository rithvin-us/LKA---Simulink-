function T = lka_detector_benchmark(datasetDir, detectorNames, V)
% LKA_DETECTOR_BENCHMARK Scores lane detectors on a recorded dataset.
%
% Syntax:
%   T = lka_detector_benchmark(datasetDir, detectorNames, V)       V = LKA.Vision
%
% Dataset format: LkaFrameLogger / make_synthetic_vision_dataset (meta.mat with
% LaneLabelId and ground-truth boundaries at the rear axle). Each frame is scored on
% its own (fresh detector state) so the numbers describe the detector, not the tracker:
%   Precision, Recall, F1, IoU, PixelAccuracy (lka_seg_metrics, 3 px tolerance; NaN when
%                           the labels contain no lane-marking class)
%   DetRateLeft/Right       share of frames with that boundary found
%   MaeY5/10/20_m, P95Y10_m lateral error of the fitted boundary at x = 5/10/20 m
%   MaeHeading_deg          boundary heading error at the rear axle
%   MsMean, MsP95           detector time per frame (first frame excluded)
% On the official track the ground truth is the OpenDRIVE lane border, while the paint
% sits slightly off it (edge line ~0.13 m outside, inner stripe of the double centre
% line ~0.15 m inside): the boundary errors carry that offset for every detector.

    S = load(fullfile(datasetDir, 'meta.mat'));
    meta = S.meta;
    cam = lka_camera_model(V.Camera, V.Bev);
    xEval = [5 10 20];
    n = numel(meta.files);
    rows = cell(numel(detectorNames), 1);
    for d = 1:numel(detectorNames)
        Vd = V;
        Vd.Detector = detectorNames{d};
        det = lka_detector_create(Vd);
        camD = cam;
        camD.XFar = min(cam.XFar, det.MaxRange);      % same trusted range as in the closed loop
        seg = zeros(n, 5);
        err = nan(n, 2, numel(xEval) + 1);
        found = false(n, 2);
        ms = zeros(n, 1);
        for i = 1:n
            img = imread(fullfile(datasetDir, [meta.files{i} '.png']));
            gtMask = imread(fullfile(datasetDir, [meta.files{i} '_label.png'])) == meta.LaneLabelId;
            t0 = tic;
            prob = det.Detect(img);
            ms(i) = 1000 * toc(t0);
            m = lka_seg_metrics(max(prob, [], 3), gtMask, V.Search.ProbThreshold, 3);
            seg(i, :) = [m.Precision m.Recall m.F1 m.IoU m.PixelAccuracy];
            sensor = lka_vision_lane_step(lka_bev_warp(cam, prob), camD, Vd, lka_vision_state_init(Vd), V.Ts);
            gts = {meta.gtLeft(i, :), meta.gtRight(i, :)};
            sides = {'Left', 'Right'};
            for k = 1:2
                b = sensor.(sides{k});
                found(i, k) = b.Strength > 0;
                if found(i, k)
                    g = gts{k};
                    yDet = b.LateralOffset + tan(b.HeadingAngle) * xEval + b.Curvature / 2 * xEval.^2 + b.CurvatureDerivative / 6 * xEval.^3;
                    yGt = g(1) + tan(g(2)) * xEval + g(3) / 2 * xEval.^2 + g(4) / 6 * xEval.^3;
                    err(i, k, 1:numel(xEval)) = abs(yDet - yGt);
                    err(i, k, end) = abs(rad2deg(b.HeadingAngle - g(2)));
                end
            end
        end
        e = reshape(err(:, :, 1:numel(xEval)), [], numel(xEval));
        h = reshape(err(:, :, end), [], 1);
        timed = ms(min(2, n):end);
        r.Detector = string(detectorNames{d});
        r.Frames = n;
        r.Precision = mean(seg(:, 1), 'omitnan');
        r.Recall = mean(seg(:, 2), 'omitnan');
        r.F1 = mean(seg(:, 3), 'omitnan');
        r.IoU = mean(seg(:, 4), 'omitnan');
        r.PixelAccuracy = mean(seg(:, 5), 'omitnan');
        r.DetRateLeft = mean(found(:, 1));
        r.DetRateRight = mean(found(:, 2));
        r.MaeY5_m = mean(e(:, 1), 'omitnan');
        r.MaeY10_m = mean(e(:, 2), 'omitnan');
        r.MaeY20_m = mean(e(:, 3), 'omitnan');
        r.P95Y10_m = prctile(e(:, 2), 95);
        r.MaeHeading_deg = mean(h, 'omitnan');
        r.MsMean = mean(timed);
        r.MsP95 = prctile(timed, 95);
        rows{d} = r;
    end
    T = struct2table([rows{:}]);
end
