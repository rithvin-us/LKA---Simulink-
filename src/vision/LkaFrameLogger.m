classdef LkaFrameLogger < matlab.System
% LKAFRAMELOGGER Records camera frames, semantic labels and ground-truth boundaries.
%
% Inputs: RGB image, semantic label image, rear-axle pose [x y psi], ground-truth left and
% right boundary [offset; heading; curvature; dkappa] (lka_lane_geometry). Every
% Vision.Bench.Decimation-th frame is written to Vision.Bench.DatasetDir (benchmark
% format, see lka_detector_benchmark); an empty DatasetDir disables the logger.
% Development / validation only (rule I.2.3): the logger feeds nothing back.

    properties (Nontunable)
        Vision = struct()     % LKA.Vision (Bench.DatasetDir, Bench.Decimation, Bench.LaneLabelId)
        Ts = 0.05             % frame period [s]
    end

    properties (Access = private)
        Folder = ''
        K = 0
        N = 0
        Meta
    end

    methods
        function obj = LkaFrameLogger(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.K = 0;
            obj.N = 0;
            obj.Folder = obj.Vision.Bench.DatasetDir;
            obj.Meta = struct('t', zeros(0, 1), 'pose', zeros(0, 3), 'gtLeft', zeros(0, 4), ...
                'gtRight', zeros(0, 4), 'files', {cell(0, 1)});
            if ~isempty(obj.Folder) && ~isfolder(obj.Folder)
                mkdir(obj.Folder);
            end
        end

        function stepImpl(obj, img, label, pose, gtLeft, gtRight)
            obj.K = obj.K + 1;
            if isempty(obj.Folder) || mod(obj.K - 1, obj.Vision.Bench.Decimation) ~= 0
                return
            end
            obj.N = obj.N + 1;
            name = sprintf('frame_%05d', obj.N);
            imwrite(img, fullfile(obj.Folder, [name '.png']));
            imwrite(uint8(label), fullfile(obj.Folder, [name '_label.png']));
            obj.Meta.t(end + 1, 1) = (obj.K - 1) * obj.Ts;
            obj.Meta.pose(end + 1, :) = pose(:)';
            obj.Meta.gtLeft(end + 1, :) = gtLeft(:)';
            obj.Meta.gtRight(end + 1, :) = gtRight(:)';
            obj.Meta.files{end + 1, 1} = name;
        end

        function releaseImpl(obj)
            if isempty(obj.Folder) || obj.N == 0
                return
            end
            meta = obj.Meta;
            meta.Ts = obj.Ts * obj.Vision.Bench.Decimation;
            meta.LaneLabelId = obj.Vision.Bench.LaneLabelId;
            save(fullfile(obj.Folder, 'meta.mat'), 'meta');
        end

        function n = getNumInputsImpl(~)
            n = 5;
        end

        function n = getNumOutputsImpl(~)
            n = 0;
        end
    end

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl
            simMode = 'Interpreted execution';
        end

        function flag = showSimulateUsingImpl
            flag = false;
        end
    end
end
