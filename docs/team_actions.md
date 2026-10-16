# What the team still has to do (LKA, status 2026-10-16)

The software side is complete for the virtual round: controller, simulation benches, camera
lane detection (team CNN on the GPU), track-video tool and live dashboard are in `main` and
tested. The items below need **people, measurements, data or installs** that cannot be done
from the code. They are sorted by priority. Each item says why it matters, how to do it, and
what to hand back.

---

## P1 — before the virtual round

### 1. Measure the camera that films the track videos

**Why.** A single camera cannot tell a narrow lane seen from low down from a wide lane seen
from high up. The video tool therefore assumes a 1.2 m lens height and a 70° field of view.
Every distance, speed and radius from the college videos is only as good as these two numbers
(today about ±20 %).

**How.**
1. Write down the phone model and the video mode used (resolution, frame rate, which lens: main
   1x, not ultra-wide).
2. Field of view: look up the main-camera spec (for example "26 mm equivalent"), or better,
   calibrate the camera:
   - print a checkerboard (MATLAB: `open checkerboardPattern.pdf`), tape it to a flat board;
   - film or photograph it from 15-20 angles in the same video mode;
   - MATLAB app **Camera Calibrator**, export `cameraParams`, save it as
     `data/camera/phone_cameraParams.mat`.
3. Mount: with the phone in its holder on the vehicle, measure with a tape:
   - lens height above the road [m];
   - distance of the lens ahead of the rear axle [m];
   - sideways offset from the vehicle centre line [m] (+ left).
4. Mount the phone rigidly (no hand-held clips), in landscape, level.

**Give back.** A short text file `vids/camera.txt` with the numbers above (and the `.mat` if
calibrated). Then the videos are re-run with
`run_lka_on_test_videos(HFovDeg=<deg>, CameraHeight=<m>)`.

### 2. Measure the college track lanes

**Why.** The controller and the lane search check that the two detected lines are a
plausible lane-width apart. The official track has 3.5 m lanes; the college lane measured from
the videos is about 1.9-2.1 m, but that number depends on item 1.

**How.** At 5 places (two straights, the start of a bend, the narrowest point of the snake
section, one wide point), measure with a tape:
- the distance between the **inner edges** of the two painted lines [m];
- the width of one painted line [m].
Take one photo of each place with the tape visible.

**Give back.** The numbers in `vids/track.txt`. Use them as `LaneWidth=<m>` in the video tool.
For a live test on that track, set `lkaLaneWidth = <m>;` before `run('config/startup.m')`.

### 3. Record a speed reference together with a video

**Why.** Speed and yaw rate from the videos come from the road texture (visual odometry): about
±20 % on 832x464 WhatsApp clips. One run with a real speed log calibrates the method, and the
tool can then use the log directly.

**How.**
1. Install a GPS logger app that exports CSV with time and speed (1 Hz or faster), or log wheel
   speed from the vehicle if available. The phone gyro (yaw rate) is a bonus.
2. Start the video and the logger, then clap once in front of the camera while pressing a mark
   in the logger (sync point).
3. Drive one lap at 10-30 km/h, staying in the lane centre.
4. Convert to CSV columns `time_s, speed_mps[, yaw_rate_radps]`, with time 0 = the clap.

**Give back.** `vids/<clip>.mp4` and `vids/<clip>_speed.csv`. Run:
`run_lka_on_video('vids/<clip>.mp4', Camera='auto', SpeedLog='vids/<clip>_speed.csv')`.

### 4. Record better videos (and do not send them through WhatsApp)

**Why.** WhatsApp shrinks the video to 832x464 and compresses it heavily; far lines and the
road texture smear, which costs detection range and odometry accuracy.

**How.**
- 1920x1080 (or 1280x720) at 30 fps, main lens, phone mounted as in item 1.
- Start every clip with the vehicle standing still for 2 s on a straight, in the lane centre
  (the camera pitch is calibrated from the first second).
- Share the original files (Google Drive / USB), not through a messenger.
- Record day and dusk, and the snake section slowly.

**Give back.** Files in `vids/`; then `run_lka_on_test_videos()` writes `test_res/`.

### 5. Get the organisers' official track video, if they publish one

**Why.** It is the best preview of the competition camera view.

**How.** Put the file in `vids/` and run `run_lka_on_test_videos()`. For the competition
camera (not a phone), give its field of view and mount (item 1) if known.

### 6. Confirm the Baja vehicle parameters

**Why.** The controller converts the commanded curvature into a road-wheel angle with the
wheelbase, and the test bench simulates the vehicle with mass, CG and tyre data. Our file says
wheelbase 1.45 m and 240 kg; the 2025 CarMaker Baja had 1.341 m and about 347 kg.

**How.** Measure or take from the CAD / build sheet:
- wheelbase, front and rear track width;
- mass with driver, CG distance from the front axle;
- maximum road-wheel steering angle left / right;
- steering speed of the actuator (deg/s) and its delay;
- tyre size and type.

**Give back.** The values, or edit `data/parameters/vehicle_params.m` directly. Then run
`run_lka_mil()` and `run_lka_vision(Detectors={'cnn'})` and compare with
`docs/lka_controller.md` §6 and §8.

### 7. Watch the result videos and report wrong frames

**Why.** Automatic numbers (lines found 90 %) do not tell whether the found line is the right
one. A person catches wrong lines in seconds.

**How.** `start_lka_dashboard()` → Results → play each video, or open
`test_res/<clip>/lka_video_annotated.mp4`. The green area must be the lane the vehicle is
driving in, green / blue lines on the painted lines, magenta = where the LKA would steer.

**Give back.** A list `clip, time [s], what is wrong` (for example "12.23.28, 18.0 s, left line
on the kerb"). These frames are also the first ones to label in item 8.

---

## P2 — improves the team CNN on the college track

### 8. Fine-tune the team CNN (UFLDv2) on college-track frames

**Why.** The CNN was trained on TuSimple highway images. On the college snake section
(clip 12.23.28) it takes road edges and kerbs for lane lines (both lines found in only 60 % of
the frames). MATLAB runs the model exactly like Colab (checked to 0.06 px), so the only way to
make it see this track is more training data. Code-side tricks were tested and did not help.

**How.**
1. Extract about 300-500 frames from the college videos (every 0.5 s; include the frames from
   item 7), resize to 1280x720.
2. Label every lane line as a polyline (CVAT or labelme), ordered left to right: left-left,
   ego-left, ego-right, right-right (the same 4 slots the model outputs).
3. Convert the labels to the TuSimple JSON format (`h_samples` rows 160...710 every 10 px).
4. In the team Colab, continue training from `model_best.pth` with a small learning rate
   (about 1/10 of the original) for a few epochs. Mix in TuSimple images so it does not forget
   highways. Hold back 20 % of the college frames for validation.
5. Export: `python tools/export_ufld_onnx.py <new.pth> team_cnn.onnx`.
6. Put the new `team_cnn.onnx` on the team Drive and in `models/perception/` (the file is too
   big for GitHub). Delete `models/perception/team_cnn_dlnetwork.mat` so MATLAB re-imports it.
7. Check: `runtests('tests/unit_tests/test_ufld_reference_parity.m')`,
   `run_lka_on_test_videos()`, `run_lka_vision(Detectors={'cnn'})` (S1 must stay ≤ 0.05 m).

**Give back.** The new `.pth` and `.onnx` (Drive), the validation accuracy from Colab, and the
before / after `test_res/summary.csv`.

---

## P3 — tools and installs (decision by the team lead / lab admin)

### 9. RoadRunner R2026a (competition co-simulation, plan phase 5)

**Why.** The final simulation runs in RoadRunner with the competition skeleton
(`aBaja2026.slx`). The LKA is wired in (`integrate_lka_into_skeleton`) but has never run against
RoadRunner because it is not installed.

**How.** MathWorks account → Downloads → R2026a → install **RoadRunner** and **RoadRunner
Scenario** (licence from the aBaja student offer or campus). In MATLAB: `rrApp = roadrunnerSetup;`
once. Then follow `docs/lka_controller.md` §5. The first run must check the sign of the start
pose and yaw (`LKA.Cosim.YawOffsetDeg`).

### 10. CUDA Toolkit + cuDNN + TensorRT (optional, faster CNN)

**Why.** The CNN now takes about 32 ms per frame on the RTX 4050 through MATLAB. GPU Coder with
TensorRT would bring it to a few milliseconds and is the same path as on the Jetson.

**How.** Install the CUDA Toolkit, cuDNN and TensorRT versions listed for MATLAB R2026a GPU Coder,
set `CUDA_PATH`, `NVIDIA_CUDNN`, `NVIDIA_TENSORRT`, then check with `coder.checkGpuInstall`.

### 11. NVIDIA power setting (optional, one minute)

NVIDIA Control Panel → Manage 3D settings → Program settings → MATLAB → Power management mode:
**Prefer maximum performance**. The laptop GPU then stops dropping its clock between frames.

### 12. Every team member's PC

- MATLAB R2026a with the toolboxes listed in `README.md`, plus the add-on **Deep Learning
  Toolbox Converter for ONNX Model Format**.
- `git pull` the `main` branch; copy `models/perception/team_cnn.onnx` from the team Drive.
- Do not save the Simulink models in an older MATLAB release.

---

## P4 — after the virtual round (Jetson / real vehicle)

### 13. Fix the 2025 Python stack before it drives the real vehicle

`docs/adas2026_audit.md` lists the findings with evidence. Order:
1. **C1 + C2** steering and yaw-damping signs (the 2025 law steers the wrong way);
2. **C3-C5** firmware: strict serial parser, steering watchdog, brake latch; bench test wheels-up;
3. **H1** calibrate the ZED and use a metric bird's-eye view (port of `lka_camera_model.m`);
4. **H2, H3, M5** lane width 3.5 m, per-frame command (no 1 s mean), measured speed;
5. **H4, H6, H7** gyro sign, NaN guards, YOLOP fed RGB;
6. the remaining M and L items.

### 14. Bench and track tests on the real vehicle

Wheels-up test of steering direction and limits first, then low speed on a straight, then the
track at increasing speed, with the LKA able to be switched off instantly by the driver.

---

Questions about any item: the technical details are in `docs/lka_controller.md`, the plain
introduction in `docs/LKA_Team_Guide.html`.
