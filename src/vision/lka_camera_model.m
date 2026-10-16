function cam = lka_camera_model(C, B)
% LKA_CAMERA_MODEL monoCamera and metric bird's-eye view of the lane camera.
%
% Syntax:
%   cam = lka_camera_model(C, B)        C = LKA.Vision.Camera, B = LKA.Vision.Bev
%
% Vehicle frame of the monoCamera = LKA frame: ISO 8855, origin on the ground below
% the rear-axle centre (SensorLocation = [C.X C.Y]). Positive pitch looks down.
%
% Outputs (struct):
%   Sensor  monoCamera
%   Bev     birdsEyeView over [XNear XFar -HalfWidth HalfWidth]
%   RowX    x [m] of every bird's-eye row (column vector, row 1 = far end)
%   ColY    y [m] of every bird's-eye column (row vector, column 1 = left edge)
%   XNear   first bird's-eye x [m], just beyond the closest ground seen by the bottom row
%   XFar    last bird's-eye x [m]
%   Params  C

    intr = cameraIntrinsics(C.FocalLength, C.PrincipalPoint, C.ImageSize);
    cam.Sensor = monoCamera(intr, C.Height, 'Pitch', C.PitchDeg, 'SensorLocation', [C.X C.Y]);

    below = deg2rad(C.PitchDeg) + atan((C.ImageSize(1) - C.PrincipalPoint(2)) / C.FocalLength(2));
    cam.XNear = C.X + C.Height / tan(below) + B.NearMargin;
    cam.XFar  = C.X + B.Range;
    nRows = round((cam.XFar - cam.XNear) / B.Resolution);
    nCols = round(2 * B.HalfWidth / B.Resolution);
    cam.Bev = birdsEyeView(cam.Sensor, [cam.XNear, cam.XFar, -B.HalfWidth, B.HalfWidth], [nRows nCols]);

    xy = imageToVehicle(cam.Bev, [ones(nRows, 1), (1:nRows)']);
    cam.RowX = xy(:, 1);
    xy = imageToVehicle(cam.Bev, [(1:nCols)', ones(nCols, 1)]);
    cam.ColY = xy(:, 2)';
    cam.Params = C;
end
