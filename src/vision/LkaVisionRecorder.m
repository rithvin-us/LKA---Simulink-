classdef LkaVisionRecorder < matlab.System
% LKAVISIONRECORDER Writes the annotated LKA camera video (lka_annotate_frame).
%
% Inputs: camera image, bird's-eye preview, LaneSensorBus, steering command [deg, + left],
% LKA state (LkaInfoBus.State). The video path is FileName, or Vision.Bench.VideoFile when
% FileName is empty (MATLAB System blocks take char properties literally, so Simulink
% models leave FileName empty and set LKA.Vision.Bench.VideoFile). No path: disabled.

    properties (Nontunable)
        Vision = struct()     % LKA.Vision
        Ts = 0.05             % frame period [s]
        FileName = ''         % .mp4 path (MATLAB use); empty = Vision.Bench.VideoFile
    end

    properties (Access = private)
        Writer
        Cam
    end

    methods
        function obj = LkaVisionRecorder(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.Writer = [];
            file = obj.FileName;
            if isempty(file) && isfield(obj.Vision, 'Bench')
                file = obj.Vision.Bench.VideoFile;
            end
            if isempty(file)
                return
            end
            folder = fileparts(file);
            if ~isempty(folder) && ~isfolder(folder)
                mkdir(folder);
            end
            obj.Cam = lka_camera_model(obj.Vision.Camera, obj.Vision.Bev);
            obj.Writer = VideoWriter(file, 'MPEG-4');
            obj.Writer.FrameRate = round(1 / obj.Ts);
            open(obj.Writer);
        end

        function stepImpl(obj, img, preview, sensor, steerDeg, state)
            if isempty(obj.Writer)
                return
            end
            hud = struct('SteerDeg', double(steerDeg), 'State', double(state));
            writeVideo(obj.Writer, lka_annotate_frame(img, preview, sensor, hud, obj.Cam));
        end

        function releaseImpl(obj)
            if ~isempty(obj.Writer)
                close(obj.Writer);
                obj.Writer = [];
            end
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
