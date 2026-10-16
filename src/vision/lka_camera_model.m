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
%   Sampler bilinear source pixels of every bird's-eye pixel (lka_bev_warp: same result as
%           transformImage(Bev, I), but batched and on the GPU)
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
    cam.Sampler = bevSampler(cam.Sensor, cam.RowX, cam.ColY, C.ImageSize);
    cam.Params = C;
end

function S = bevSampler(sensor, rowX, colY, imageSize)
% For every bird's-eye pixel the four neighbouring camera pixels and their bilinear
% weights. Like transformImage (imwarp, linear, fill 0): a point outside the pixel
% centres 1..W, 1..H gets weight 0.
    [Y, X] = meshgrid(colY, rowX);
    uv = vehicleToImage(sensor, [X(:) Y(:)]);
    h = imageSize(1); w = imageSize(2);
    u = uv(:, 1); v = uv(:, 2);
    in = u >= 1 & u <= w & v >= 1 & v <= h;
    u(~in) = 1; v(~in) = 1;
    u0 = min(floor(u), w - 1); v0 = min(floor(v), h - 1);   % u = W uses pixels W-1, W
    fu = u - u0; fv = v - v0;
    uu = [u0, u0 + 1, u0, u0 + 1];
    vv = [v0, v0, v0 + 1, v0 + 1];
    wt = [(1 - fu) .* (1 - fv), fu .* (1 - fv), (1 - fu) .* fv, fu .* fv];
    wt(~in, :) = 0;
    S.Index = uint32(vv + (uu - 1) * h);
    S.Weight = single(wt);
    S.Size = [numel(rowX) numel(colY)];
    S.ImageSize = [h w];
    S.Key = sprintf('%.17g_', sum(double(S.Index(:, 1))), sum(S.Weight(:, 2)), h, w, S.Size);
end
