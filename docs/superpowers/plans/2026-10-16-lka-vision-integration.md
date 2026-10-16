# LKA Vision Integration (ADAS_2026 → MATLAB) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the ground-truth lane input of the aBaja 2026 LKA with a camera lane detector built from the verified, corrected parts of the 2025 ADAS_2026 Python stack. The detector is switchable between YOLOP and the team CNN (UFLDv2, ResNet-18, TuSimple), uses a calibrated metric bird's-eye view, and is validated camera-in-the-loop in Unreal on the official LKA track. It produces annotated videos, a detector benchmark and an audit of the 2025 code.

**Architecture:** Camera image → detector (YOLOP / team CNN through one ONNX contract, or a classic reference detector) → lane-line probability map → calibrated `monoCamera`/`birdsEyeView` metric bird's-eye → sliding-window search plus robust polynomial fit per boundary → the **existing** `LaneSensorBus` → the **existing** `LKA_Controller` (Kalman lane tracker, supervisor, pure pursuit / Stanley / MPC). The 2025 Python code stays read-only. Its good ideas (continuity search, gates, quality tiers) are ported fixed; its bugs are documented and reproduced in a MATLAB replica.

**Tech Stack:** MATLAB/Simulink R2026a, Automated Driving Toolbox (monoCamera, birdsEyeView, Simulation 3D blocks, Unreal `AutoVrtlEnv.exe`), Deep Learning Toolbox + "Deep Learning Toolbox Converter for ONNX Model Format" add-on, Parallel Computing Toolbox (RTX 4050 6 GB GPU), Computer Vision / Image Processing Toolboxes, MATLAB Unit Test. Python 3.14 + onnxruntime (CPU) only for the ONNX parity check and the Python sign check.

## Global Constraints

- Platform: Windows, MATLAB R2026a, simulation on RTX 4050 (6 GB) or better. No Jetson constraints in this plan (the Jetson comes after the virtual round).
- Frame convention everywhere: ISO 8855 vehicle frame, x forward, y left, yaw CCW positive, origin = rear-axle centre on the ground; positive curvature / steering / turning radius = left.
- `LaneBoundaryBus` model: `y(x) = LateralOffset + tan(HeadingAngle)*x + Curvature*x^2/2 + CurvatureDerivative*x^3/6`, boundary = painted-line centre.
- Rulebook aBAJA 2027 I.2.3: "ground truth sensors, road state sensors, or any direct environment state access" are prohibited as inputs to the final algorithm. They are allowed for development, validation and visualisation only. Submission configuration = `LKA_LANE_SOURCE = 1` (camera).
- Rulebook K.2.4/K.2.5: any tyre touching the line = outside the lane; DQ if > 33 km/h or all 4 wheels out; score `S = T_inside_lane / T_total_time^2`.
- `ADAS_2026/` is read-only (separate repo RatheeswarS/ADAS_2026) and git-ignored; only the 20 MB lane ONNX model is copied into this repo.
- Detector selection: `LKA.Vision.Detector ∈ {'yolop', 'cnn', 'classic'}`. YOLOP and the team CNN load from ONNX files described by sidecar JSONs. The sidecar `type` picks the adapter: `seg` (pixel mask, YOLOP) or `ufld_v2` (row/column anchors).
- Team CNN = **UFLDv2** (cfzd/Ultra-Fast-Lane-Detection-v2): ResNet-18, TuSimple config, input 800×320 (resize to 400×800, keep the bottom 320 rows), 56 row anchors (rows 160–710 of 720), 41 column anchors, 4 lanes, 100 grid cells. Its decoded lane points are rasterised into the same probability-map contract as YOLOP.
- The UFLDv2 ONNX file is larger than GitHub's 100 MB limit, so it is git-ignored and shared through Drive (`models/perception/README.md`).
- Every new MATLAB file starts with a header comment in the style of the existing `src/` files (purpose, syntax, inputs/outputs, conventions).
- Work on branch `feat/lka-vision`. Never push. Every commit message ends with:
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  ```
- Run MATLAB tests with `runtests('<file>')` (MCP tool `run_matlab_test_file`) from the repo root `E:\Abaja Labs\2026\LKA - Matlab`.

## Techniques used (what and why)

| Technique | Where | Why |
|---|---|---|
| Semantic lane-line segmentation (YOLOP, BDD100K weights) | Task 7 | Reuses the 2025 model. It already exports to ONNX (opset 12, 36 ms/frame on CPU). |
| One probability-map contract + JSON sidecar per model | Tasks 1, 7 | YOLOP (mask) and UFLDv2 (anchors) feed the same bird's-eye / fit / tracker chain, so the comparison is fair and switching is one parameter. |
| UFLDv2 anchor decoding (`pred2coords` logic) + rasterisation | Task 7 | The team CNN outputs per-anchor lane positions, not a mask. Decoded points become thin probability polylines. |
| Calibrated inverse perspective mapping (`monoCamera`, `birdsEyeView`) | Task 3 | The 2025 hand-tuned trapezoid gave non-metric heading (×0.24), a fake heading from offset (0.5 m → 4°) and no curvature. In simulation the camera intrinsics and mount are known exactly. |
| Sliding-window search with a continuity prior and a distance gate | Task 5 | Fixes the 2025 blob-pick bugs (no distance gate, re-acquisition lock). Robust to dashed lines and curves. |
| Robust (Huber IRLS) ridge-regularised polynomial fit in metres | Task 5 | Gives a clothoid boundary for the existing `LaneSensorBus`, with outlier rejection the 2025 fit lacked. |
| Existing Kalman lane tracker, supervisor, PP / Stanley / MPC | unchanged | Already verified in MIL (Phases 1–4). Coasting, one-line fallback and take-over requests stay there, not in the detector. |
| Sensor models (wheel speed, gyro) | Task 9 | Rule I.2.3: controller inputs must behave like real sensors. |
| Camera-in-the-loop SIL in Unreal with the official track USD | Tasks 10–11 | Tests real rendered images before RoadRunner is installed. Ground truth comes from the OpenDRIVE road. |
| Tolerant F1 / IoU, lateral error at 5/10/20 m, latency | Task 12 | Pixel accuracy is meaningless for thin lines: an all-background mask scores about 96%. |
| MATLAB replica of the 2025 control chain | Task 2 | Proves (or disproves) the audit findings closed-loop, not only by reading code. |

**Claude skills during execution:** `subagent-driven-development` (or `executing-plans`) for the task loop, `test-driven-development` inside every task, `verification-before-completion` before ticking a task, `requesting-code-review` after Tasks 6, 8, 11 and 13.

## Uncertainty register

| # | Uncertainty | Resolved in | How / fallback |
|---|---|---|---|
| U1 | Does YOLOP (real dashcam training) detect lines in Unreal renders? | Task 11, 12 | Benchmark F1 and lateral error. Fallback: classic detector, or fine-tune YOLOP on sim frames (separate plan). |
| U2 | How good is the team CNN (UFLDv2) on our roads? Its "96%" is the TuSimple point accuracy of the best epoch (0.960, F1 0.964 in the Colab log), a legitimate metric. But the best epoch was chosen on the TuSimple test split (optimistic), and TuSimple is US-highway daytime only. | Task 12 | Same-frame benchmark against YOLOP and classic on the official track; lateral error in metres is the deciding number. |
| U14 | UFLDv2 ONNX export (notebook cell 27 never ran: `TEST_MODEL` is still the `<run_folder>` placeholder); output names and order of `deploy/pt2onnx.py` | Task 7 | The team exports in Colab (steps in Task 7). The adapter identifies the 4 outputs by shape, not by name. Decoding is unit-tested with synthetic outputs before the model arrives. |
| U3 | Does MATLAB import the YOLOP ONNX graph correctly (Slice, Resize, Clip ops)? | Task 7 | Parity test against onnxruntime (max abs diff < 1e-3). Fallback: onnxruntime through the Python interface. |
| U4 | Skeleton camera extrinsics ("Hood center" of the Muscle car) | Task 11, Phase 5 | Measured with the camera transform outport; finalised in the RoadRunner co-sim. |
| U5 | Track (OpenDRIVE) → Simulation 3D world mapping | Task 11 | Prior from the aBaja2026 InitFcn (`X = x, Y = -y, yaw = -psi`); `calibrate_vision_bench` tries all 4 candidate maps. |
| U6 | Does Unreal load `LKA.usd` without RoadRunner installed? | Task 10 | Smoke run. Fallback: built-in "Curved road" scene with semantic-label ground truth. |
| U7 | Semantic label id of lane markings | Task 11 | Auto-detected by `calibrate_vision_bench`. |
| U8 | Simulation 3D vehicle origin height / hood occlusion of the camera | Task 11 | Implied camera height from the measured lane width; bottom-row label check; move the bench camera forward if the hood is visible. |
| U9 | GPU memory (Unreal + YOLOP on 6 GB) | Task 11 | `gpuDevice` free memory logged. Fallback `LKA.Vision.UseGpu = false`. |
| U10 | Sign of the skeleton `CurrYawRate` | Task 13, Phase 5 | `LKA.Cosim.YawRateSign = -1`, checked against the logged pose-derived yaw rate. |
| U11 | YOLOP licence text | Task 1 | MIT licence of hustvl/YOLOP, copied verbatim into `models/perception/YOLOP_LICENSE.txt`. |
| U12 | Simulation 3D library block parameter names | Task 10 Step 1 | Printed from the mask; the build script uses the printed names. |
| U13 | Baja geometry (wheelbase 1.45 m in `vehicle_params.m` vs 1.341 m in the 2025 CarMaker file) | open (user) | Not blocking; the vision chain is geometry-agnostic. |

---

### Task 1: Repo hygiene, perception assets, detector contract

**Files:**
- Modify: `.gitignore` (append)
- Modify: `config/startup.m:13-30` (paths)
- Create: `models/perception/yolop_ll_384x640.onnx` (copy of `ADAS_2026/YOLOP/weights/yolop_ll_384x640.onnx`)
- Create: `models/perception/yolop.json`, `models/perception/cnn.json`, `models/perception/README.md`, `models/perception/YOLOP_LICENSE.txt`
- Create: `data/scenes/LKA/` (copy of `D:\abaja_rr\baja\Exports\*`)
- Test: `tests/unit_tests/test_vision_assets.m`

**Interfaces:**
- Produces two sidecar schemas, both with `type, name, onnx, input_size[H W], channel_order 'RGB'|'BGR', scale, mean[3], std[3]`:
  - `type = 'seg'` adds `resize 'letterbox'|'stretch', pad_value, output 'softmax'|'sigmoid'|'prob', lane_channel`. Read by `lka_detector_onnx_seg` (Task 7).
  - `type = 'ufld_v2'` adds `crop_ratio, num_grid_row, num_cls_row, num_grid_col, num_cls_col, num_lanes, row_lanes, col_lanes, row_anchor_px[2], anchor_image_height, local_width, line_width_px`. Read by `lka_detector_ufld` (Task 7).
- Also produces the scene file `data/scenes/LKA/LKA.usd` (Task 10).

- [ ] **Step 1: Create the branch**

```bash
git checkout -b feat/lka-vision
```

- [ ] **Step 2: Write the failing test** `tests/unit_tests/test_vision_assets.m`

```matlab
% ==============================================================================
% test_vision_assets.m
% Perception assets: detector sidecars, YOLOP ONNX model, LKA track scene, git hygiene.
% ==============================================================================

function tests = test_vision_assets
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.TestData.root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(testCase.TestData.root, 'config', 'startup.m'));
end

function testDetectorSidecarsParse(testCase)
    d = fullfile(testCase.TestData.root, 'models', 'perception');
    common = {'type', 'name', 'onnx', 'input_size', 'channel_order', 'scale', 'mean', 'std'};
    perType.seg = {'resize', 'pad_value', 'output', 'lane_channel'};
    perType.ufld_v2 = {'crop_ratio', 'num_grid_row', 'num_cls_row', 'num_grid_col', 'num_cls_col', ...
        'num_lanes', 'row_lanes', 'col_lanes', 'row_anchor_px', 'anchor_image_height', 'local_width', 'line_width_px'};
    for name = ["yolop", "cnn"]
        cfg = jsondecode(fileread(fullfile(d, name + ".json")));
        testCase.verifyTrue(all(isfield(cfg, common)), name);
        testCase.verifyTrue(isfield(perType, cfg.type), name);
        testCase.verifyTrue(all(isfield(cfg, perType.(cfg.type))), name);
        testCase.verifyNumElements(cfg.input_size, 2, name);
        testCase.verifyNumElements(cfg.mean, 3, name);
        testCase.verifyNumElements(cfg.std, 3, name);
    end
    yolop = jsondecode(fileread(fullfile(d, 'yolop.json')));
    testCase.verifyEqual(yolop.type, 'seg');
    cnn = jsondecode(fileread(fullfile(d, 'cnn.json')));
    testCase.verifyEqual(cnn.type, 'ufld_v2');
end

function testYolopOnnxPresent(testCase)
    f = fullfile(testCase.TestData.root, 'models', 'perception', 'yolop_ll_384x640.onnx');
    testCase.verifyTrue(isfile(f));
    info = dir(f);
    testCase.verifyGreaterThan(info.bytes, 20e6);
    testCase.verifyLessThan(info.bytes, 50e6);          % below GitHub's 50 MB warning
end

function testLkaSceneExported(testCase)
    d = fullfile(testCase.TestData.root, 'data', 'scenes', 'LKA');
    testCase.verifyTrue(isfile(fullfile(d, 'LKA.usd')));
    testCase.verifyTrue(isfile(fullfile(d, 'Roads.mesh.usd')));
end

function testAdas2026IsIgnored(testCase)
    [st, out] = system(sprintf('git -C "%s" check-ignore ADAS_2026/README.md', testCase.TestData.root));
    testCase.verifyEqual(st, 0, out);
end
```

- [ ] **Step 3: Run it, expect FAIL**

Run: `runtests('tests/unit_tests/test_vision_assets.m')`
Expected: FAIL. The sidecar files are not found, the ONNX file is missing, and `check-ignore` returns 1.

- [ ] **Step 4: Append to `.gitignore`**

```gitignore

# 2025 Python ADAS stack: separate repo (RatheeswarS/ADAS_2026), read-only reference
ADAS_2026/
# Team CNN (UFLDv2) ONNX is > 100 MB: shared through Drive, see models/perception/README.md
models/perception/team_cnn.onnx
# Imported ONNX networks (regenerated from the .onnx by lka_import_onnx_detector)
models/perception/*_dlnetwork.mat
models/perception/+lka_onnx_*/
# Camera datasets, videos, ONNX parity fixtures
data/vision_datasets/
results/**/*.mp4
tests/fixtures/*.bin
```

- [ ] **Step 5: Copy the assets** (PowerShell)

```powershell
New-Item -ItemType Directory -Force "models\perception", "data\scenes\LKA" | Out-Null
Copy-Item "ADAS_2026\YOLOP\weights\yolop_ll_384x640.onnx" "models\perception\"
Copy-Item "D:\abaja_rr\baja\Exports\*" "data\scenes\LKA\"
```

- [ ] **Step 6: Create `models/perception/yolop.json`**

```json
{
  "type": "seg",
  "name": "YOLOP lane-line head (hustvl/YOLOP End-to-end.pth, lane-only export from ADAS_2026)",
  "onnx": "yolop_ll_384x640.onnx",
  "input_size": [384, 640],
  "channel_order": "BGR",
  "scale": 255,
  "mean": [0.485, 0.456, 0.406],
  "std": [0.229, 0.224, 0.225],
  "resize": "letterbox",
  "pad_value": 114,
  "output": "softmax",
  "lane_channel": 2
}
```

(The ADAS_2026 runtime feeds BGR frames, normalises with the ImageNet statistics in that channel order and applies a softmax over the two sigmoid outputs: `model_runtime.py:29-31, 106, 120`. The letterbox maps 1280×720 to 640×360 plus 12 px of padding top and bottom: `augmentations.py`.)

- [ ] **Step 7: Create `models/perception/cnn.json`** (team UFLDv2, values from `configs/tusimple_res18.py` as printed in the Colab training log)

```json
{
  "type": "ufld_v2",
  "name": "Team CNN: UFLDv2 ResNet-18, TuSimple (Colab UFLDv2_Lane_Detection_Colab.ipynb, model_best.pth)",
  "onnx": "team_cnn.onnx",
  "input_size": [320, 800],
  "crop_ratio": 0.8,
  "channel_order": "RGB",
  "scale": 255,
  "mean": [0.485, 0.456, 0.406],
  "std": [0.229, 0.224, 0.225],
  "num_grid_row": 100,
  "num_cls_row": 56,
  "num_grid_col": 100,
  "num_cls_col": 41,
  "num_lanes": 4,
  "row_lanes": [2, 3],
  "col_lanes": [1, 4],
  "row_anchor_px": [160, 710],
  "anchor_image_height": 720,
  "local_width": 1,
  "line_width_px": 5
}
```

(UFLDv2 TuSimple conventions: `row_anchor = linspace(160, 710, 56)/720`, `col_anchor = linspace(0, 1, 41)`. Lanes 2 and 3 (1-based; the ego lines) are decoded from the row anchors, lanes 1 and 4 (the adjacent lines) from the column anchors, as in the repository's `pred2coords`. Preprocessing: RGB, resize to `(320/0.8) x 800 = 400 x 800`, keep the bottom 320 rows, ImageNet normalisation.)

- [ ] **Step 8: Create `models/perception/README.md`**

````markdown
# Lane detection models

Every lane detector produces the same thing: **camera image in → per-pixel lane-line
probability map out** (`det.Detect(rgb)`). The rest of the chain (bird's-eye view, search,
fit, tracker) is shared. A neural model is an ONNX file plus a JSON sidecar next to it; the
sidecar `type` selects the adapter. Select the detector with `LKA.Vision.Detector`.

| Detector | Sidecar | Adapter | Model |
|---|---|---|---|
| `yolop` | `yolop.json` (`seg`) | `lka_detector_onnx_seg.m` | `yolop_ll_384x640.onnx`: YOLOP lane-line head, exported from ADAS_2026 (`export_yolop_onnx.py`) |
| `cnn` | `cnn.json` (`ufld_v2`) | `lka_detector_ufld.m` | `team_cnn.onnx`: team UFLDv2 ResNet-18 (TuSimple), **not in git** (> 100 MB), copy from the team Drive |
| `classic` | none | `lka_detector_classic.m` | white/yellow local-contrast reference detector |

## Exporting the team UFLDv2 model (Colab, `UFLDv2_Lane_Detection_Colab.ipynb`)

1. Run cells 1–4 (GPU, Drive, clone, `my_interp` build).
2. Find the checkpoint: `!find /content/drive/MyDrive/lane_logs/tusimple_res18 -name model_best.pth`.
3. Set `TEST_MODEL` in cell 25 to that exact path (it is still the `<run_folder>` placeholder) and run
   cell 25 once. Record the printed TuSimple test accuracy / F1 here.
4. Run cell 27: `!python deploy/pt2onnx.py --config_path configs/tusimple_res18.py --model_path {TEST_MODEL}`.
5. Check the export:
   ```python
   import onnx, glob, os
   f = sorted(glob.glob(os.path.dirname(TEST_MODEL) + '/*.onnx') + glob.glob('*.onnx'))[-1]
   m = onnx.load(f); print(f)
   print([(o.name, [d.dim_value for d in o.type.tensor_type.shape.dim]) for o in m.graph.output])
   ```
   Expected: 4 outputs with shapes `[1,100,56,4]` (loc_row), `[1,100,41,4]` (loc_col),
   `[1,2,56,4]` (exist_row), `[1,2,41,4]` (exist_col), any order. Input `[1,3,320,800]`.
6. Put the `.onnx` on the team Drive, then copy it to `models/perception/team_cnn.onnx` on the
   simulation PC.

## Contract for other models

- `type: "seg"`: ONNX input `1 x 3 x H x W` float32; output `1 x C x H x W` (same H, W).
  `C = 1` gives a lane logit (`"output": "sigmoid"`) or a probability (`"output": "prob"`).
  `C ≥ 2` gives class scores with the lane class at `lane_channel` (1-based, `"output": "softmax"`).
  The preprocessing is described by `channel_order`, `scale`, `mean`/`std`, and `resize`
  (`letterbox` with `pad_value`, or `stretch`).
- `type: "ufld_v2"`: UFLDv2 row/column-anchor heads as above. A different dataset config
  (CULane, CurveLanes) only changes the sidecar numbers.
- A model with any other output needs its own adapter returning the same probability map.

## Team CNN notebook review (2026-10-16)

- Training: UFLDv2 ResNet-18, TuSimple train split (3 626 images), 100 epochs, SGD lr 0.05.
  Evaluated every epoch on the TuSimple test split (2 782 images, 348 batches).
- The "96 %" is the TuSimple **point accuracy** (0.9596–0.9600, F1 0.962–0.964 in the last
  epochs). It is a standard lane metric, not pixel accuracy, so it is meaningful.
- `model_best.pth` is the best epoch **on the test split**, so the number is slightly optimistic
  (no separate validation split).
- Domain: TuSimple is US highways in daylight (1280×720, same as the simulator camera). The
  official LKA track and the Unreal renders are different; the benchmark measures that gap.
- The checkpoint file (771 MB) is much larger than the network (ResNet-18 UFLDv2); it probably
  also stores optimizer state. Only the ONNX export is needed for simulation.

## How a model is judged

`scripts/run_detector_benchmark.m` scores every detector on the same frames:
tolerant precision / recall / F1 (3 px), IoU, **pixel accuracy (reported only to show its
background bias: an empty mask already scores about 96%)**, detection rate per side, lateral
error of the fitted boundary at 5 / 10 / 20 m, heading error, ms per frame, and closed-loop
lane-keeping KPIs.

## Provenance and licence

YOLOP: https://github.com/hustvl/YOLOP (MIT licence, see `YOLOP_LICENSE.txt`), weights
`End-to-end.pth` trained on BDD100K (BDD100K licence: research use).
````

- [ ] **Step 9: Create `models/perception/YOLOP_LICENSE.txt`**

Fetch the licence with WebFetch from `https://raw.githubusercontent.com/hustvl/YOLOP/main/LICENSE` (read-only fetch, no download) and paste the exact text into the file. If the repository licence is not MIT, stop and tell the user before continuing.

- [ ] **Step 10: Add the new source folders to `config/startup.m`**

In the `pathsToAdd` cell array, after `fullfile(projectRoot, 'src', 'evaluation');`, add:

```matlab
    fullfile(projectRoot, 'src', 'vision');
    fullfile(projectRoot, 'src', 'sensors');
    fullfile(projectRoot, 'src', 'legacy');
    fullfile(projectRoot, 'models', 'perception');
    fullfile(projectRoot, 'tests', 'helpers');
```

Then create the empty folders: `New-Item -ItemType Directory -Force src\vision, src\sensors, src\legacy, tests\helpers, tools\adas2026`.

- [ ] **Step 11: Run the test, expect PASS**

Run: `runtests('tests/unit_tests/test_vision_assets.m')`
Expected: 4 passed.

- [ ] **Step 12: Commit**

```bash
git add .gitignore config/startup.m models/perception data/scenes tests/unit_tests/test_vision_assets.m
git commit -m "chore(vision): add YOLOP lane model, detector contract and LKA track scene

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 2: ADAS_2026 audit document and closed-loop replica of the 2025 control chain

**Files:**
- Create: `docs/adas2026_audit.md`
- Create: `tools/adas2026/verify_signs.py`
- Create: `src/legacy/lka_adas2025_params.m`, `src/legacy/lka_adas2025_steer.m`
- Create: `scripts/verify_adas2025_control.m`
- Test: `tests/unit_tests/test_adas2025_replica.m`

**Interfaces:**
- Consumes: `lka_single_track_dynamics(x, delta, vx, vehicle)`, `lka_road_closest_point(...)`, `lka_lane_geometry(...)`, `lka_ground_truth_metrics(...)`, `generate_lka_scenarios(vehicle)` (all existing).
- Produces: `[P, state] = lka_adas2025_params()`; `[deltaLeft, state] = lka_adas2025_steer(yc, psiC, v, yawRate, state, P, dt)` (rad, + left); `T = verify_adas2025_control(Plot=, OutDir=, MeasX=)` returns a table with columns `Variant, Description, MaxLatOffset_m, InLaneRatio, FirstLineTouch_s, AllWheelsOut, MaxSteer_deg`.

- [ ] **Step 1: Write the failing test** `tests/unit_tests/test_adas2025_replica.m`

```matlab
% ==============================================================================
% test_adas2025_replica.m
% Replica of the ADAS_2026 (2025) Python steering chain: sign of each term and
% closed-loop effect of the heading-sign bug (docs/adas2026_audit.md, C1).
% Frame: ISO 8855, + = left. psiC > 0: lane heads to the left of the vehicle.
% ==============================================================================

function tests = test_adas2025_replica
    tests = functiontests(localfunctions);
end

function setupOnce(~)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
end

function testHeadingTermSignAsCodedIsWrong(testCase)
    [P, st] = lka_adas2025_params();
    P.MeanWindow = 0; P.SlewRateDps = inf;
    P.HeadingBug = true;
    dAs = lka_adas2025_steer(0, deg2rad(5), 8, 0, st, P, 0.05);
    P.HeadingBug = false;
    dFix = lka_adas2025_steer(0, deg2rad(5), 8, 0, st, P, 0.05);
    testCase.verifyLessThan(dAs, 0);        % steers right although the lane turns left
    testCase.verifyGreaterThan(dFix, 0);
end

function testCrossTrackTermSign(testCase)
    % lane centre 0.5 m to the left -> both variants steer left
    [P, st] = lka_adas2025_params();
    P.MeanWindow = 0; P.SlewRateDps = inf;
    for bug = [true false]
        P.HeadingBug = bug;
        testCase.verifyGreaterThan(lka_adas2025_steer(0.5, 0, 8, 0, st, P, 0.05), 0);
    end
end

function testOneSecondMeanDelaysCommand(testCase)
    [P, st] = lka_adas2025_params();
    P.HeadingBug = false; P.MeanWindow = 1.0; P.SlewRateDps = inf;
    out = zeros(1, 25);
    for k = 1:25
        [out(k), st] = lka_adas2025_steer(0, deg2rad(4), 8, 0, st, P, 0.05);
    end
    testCase.verifyEqual(out(1:19), zeros(1, 19));           % held at the last window mean (0)
    testCase.verifyEqual(out(20), deg2rad(4), 'AbsTol', 1e-9); % first window closes after 1.0 s
end

function testClosedLoopHeadingBugIsUnstable(testCase)
    T = verify_adas2025_control(Plot=false);
    asCoded = T(T.Variant == "as_coded", :);
    fixed = T(T.Variant == "fixed", :);
    % If 'fixed' is not 100 % in lane, record it in docs/adas2026_audit.md (the corrected
    % 2025 law is then not competitive either). Do not relax this assertion silently.
    testCase.verifyEqual(fixed.InLaneRatio, 1, 'AbsTol', 1e-9);
    testCase.verifyGreaterThan(asCoded.MaxLatOffset_m, 2 * fixed.MaxLatOffset_m);
end
```

- [ ] **Step 2: Run it, expect FAIL**

Run: `runtests('tests/unit_tests/test_adas2025_replica.m')`
Expected: FAIL with "Undefined function 'lka_adas2025_params'".

- [ ] **Step 3: Create `src/legacy/lka_adas2025_params.m`**

```matlab
function [P, state] = lka_adas2025_params()
% LKA_ADAS2025_PARAMS Calibration of the 2025 ADAS_2026 Python steering chain (replica).
%
% Syntax:
%   [P, state] = lka_adas2025_params()
%
% Values copied from ADAS_2026 (read-only reference, see docs/adas2026_audit.md):
%   steering_v2.py:52-60        K_GAIN, K_SOFT, K_YAW_DAMP, MAX_STEERING_DEG, MAX_STEER_RATE_DPS
%   arduino_steering_link.py:199 1 s mean output window
%   lane_detection_node_birdeye.py:108  lane_width_m = 3.0 (track is 3.5 m) -> WidthScale
%   ego_state_estimator.py:29   default_speed_mps = 5.0 (no speed sensor) -> AssumedSpeed
%   arduinocode.ino:160-161     firmware full lock 24 deg

    P.K             = 0.8;          % K_GAIN
    P.KSoft         = 1.0;          % K_SOFT [m/s]
    P.KYawDamp      = 0.15;         % K_YAW_DAMP [deg per deg/s]
    P.MaxSteerDeg   = 30.0;         % MAX_STEERING_DEG
    P.SlewRateDps   = 60.0;         % MAX_STEER_RATE_DPS
    P.FirmwareMaxDeg = 24.0;        % firmware recalcCPD /24
    P.MeanWindow    = 1.0;          % SecondMeanSteering output_period_s (0 = per-frame)
    P.HeadingBug    = true;         % true = sign as coded (steering_v2.py:81)
    P.UseImu        = false;        % NullImuSource: no yaw damping on the 2025 car
    P.WidthScale    = 3.0 / 3.5;    % CTE scaling from the assumed 3.0 m lane
    P.AssumedSpeed  = 5.0;          % speed fed to Stanley [m/s]; NaN = use measured
    P.HeadingScale  = 1.0;          % measured heading / true heading (1 = metric IPM)

    state = struct('Last', NaN, 'Sum', 0, 'N', 0, 'Elapsed', 0, 'Out', 0);
end
```

- [ ] **Step 4: Create `src/legacy/lka_adas2025_steer.m`**

```matlab
function [deltaLeft, state] = lka_adas2025_steer(yc, psiC, v, yawRate, state, P, dt)
% LKA_ADAS2025_STEER Replica of the ADAS_2026 Python steering chain in the LKA frame.
%
% Syntax:
%   [deltaLeft, state] = lka_adas2025_steer(yc, psiC, v, yawRate, state, P, dt)
%
% Chain replicated (ADAS_2026, read-only): steering_v2.stanley_steering_deg ->
% SlewRateLimiter -> SecondMeanSteering (1 s mean) -> firmware 24 deg lock.
%
% Their conventions: cte > 0 = lane centre RIGHT of the vehicle; heading_deg has the
% SAME sign as the lane heading psiC of this repo (lane turning left -> positive,
% see docs/adas2026_audit.md C1); steering + = RIGHT. As coded:
%   steer_right = heading + atan(K*cte/(Ksoft + v))  [deg]
% The heading term must oppose the error, so the corrected law is
%   steer_right = -heading + atan(K*cte/(Ksoft + v)).
%
% Inputs:
%   yc      - lane-centre offset at the measurement point (+ left) [m]
%   psiC    - lane heading relative to the vehicle (+ CCW) [rad]
%   v       - measured speed [m/s] (replaced by P.AssumedSpeed unless NaN)
%   yawRate - gyro yaw rate (+ CCW) [rad/s], used only with P.UseImu
%   state   - from lka_adas2025_params
%   P       - from lka_adas2025_params
%   dt      - call period [s]
% Outputs:
%   deltaLeft - road-wheel angle command (+ left) [rad]

    cte = -yc * P.WidthScale;
    headingDeg = P.HeadingScale * rad2deg(psiC);
    vEff = v;
    if ~isnan(P.AssumedSpeed)
        vEff = P.AssumedSpeed;
    end
    vEff = max(vEff, 0);
    if P.HeadingBug
        steerRight = headingDeg + rad2deg(atan(P.K * cte / (P.KSoft + vEff)));
    else
        steerRight = -headingDeg + rad2deg(atan(P.K * cte / (P.KSoft + vEff)));
    end
    if P.UseImu
        rDps = rad2deg(yawRate);                         % gyro, + left
        if P.HeadingBug
            steerRight = steerRight - P.KYawDamp * rDps; % as coded: steering_v2.py:93
        else
            steerRight = steerRight + P.KYawDamp * rDps; % damping opposes the rotation
        end
    end
    steerRight = min(max(steerRight, -P.MaxSteerDeg), P.MaxSteerDeg);

    % SlewRateLimiter (first call passes through)
    if isnan(state.Last)
        state.Last = steerRight;
    else
        step = P.SlewRateDps * dt;
        state.Last = state.Last + min(max(steerRight - state.Last, -step), step);
    end

    % SecondMeanSteering: emit the mean of each window, hold it in between
    if P.MeanWindow > 0
        state.Sum = state.Sum + state.Last;
        state.N = state.N + 1;
        state.Elapsed = state.Elapsed + dt;
        if state.Elapsed >= P.MeanWindow - 1e-9
            state.Out = state.Sum / state.N;
            state.Sum = 0; state.N = 0; state.Elapsed = 0;
        end
        out = state.Out;
    else
        out = state.Last;
    end
    out = min(max(out, -P.FirmwareMaxDeg), P.FirmwareMaxDeg);
    deltaLeft = -deg2rad(out);
end
```

- [ ] **Step 5: Create `scripts/verify_adas2025_control.m`**

```matlab
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
        rows{k} = [struct('Variant', string(variants{k, 1}), 'Description', string(variants{k, 2})), kpi];
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
        allOut = allOut || all(margins < 0);
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

function plotVariants(traces, variants, vehicle, sc, outDir)
    f = figure('Color', 'w', 'Position', [100 100 1100 520]);
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
```

- [ ] **Step 6: Run the test, expect PASS**

Run: `runtests('tests/unit_tests/test_adas2025_replica.m')`
Expected: 4 passed. Also run `T = verify_adas2025_control()` and keep the printed table and `results/adas2025/adas2025_replica_lateral_offset.png` for the audit document.

- [ ] **Step 7: Create `tools/adas2026/verify_signs.py`** (Python evidence for audit finding C1/H1; runs the team's own functions)

```python
"""Synthetic sign/scale check of the ADAS_2026 bird's-eye pipeline (audit C1, H1).

Renders a ground-truth lane-line probability map through a pinhole camera with a
known pose relative to the lane, runs the team's own IPM + extraction + fit + the
CTE/heading math of lane_detection_node_birdeye.py:484-515, and prints it next to
the truth. The camera (1.2 m high, 10 deg pitch, 1280x720, f = 1109 px) is NOT the
2025 ZED mount: the sign results hold for any camera, the scale numbers do not.

Usage:  python tools/adas2026/verify_signs.py "<path to ADAS_2026/src>"
"""
import sys

import numpy as np

sys.path.insert(0, sys.argv[1])
from perception.lane_core.ipm import build_ipm_transform, warp_to_birdeye  # noqa: E402
from perception.lane_core.roi_mask import apply_roi_mask  # noqa: E402
from perception.lane_core.bird_eye_extraction import extract_birdeye_boundaries  # noqa: E402
from perception.lane_core.curves import fit_lane_curve  # noqa: E402
from perception.lane_core.steering_v2 import stanley_steering_deg  # noqa: E402

W, H, F, CX, CY = 1280, 720, 1109.0, 640.0, 360.0
CAM_H, PITCH = 1.2, np.radians(10.0)
LANE_W, LINE_W = 3.5, 0.15


def render(yv, psi, kappa):
    """yv: vehicle lateral position in the lane (+ left); psi: vehicle yaw rel. lane (+ CCW)."""
    u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    xc, yc = (u - CX) / F, (v - CY) / F
    o = np.array([np.cos(PITCH), 0, -np.sin(PITCH)])
    dn = np.array([-np.sin(PITCH), 0, -np.cos(PITCH)])
    rt = np.array([0, -1.0, 0])
    d = o[None, None, :] + yc[..., None] * dn + xc[..., None] * rt
    ok = d[..., 2] < -1e-6
    t = np.where(ok, CAM_H / np.maximum(-d[..., 2], 1e-9), 0)
    gx, gy = t * d[..., 0], t * d[..., 1]
    ok &= gx < 60
    s = gx * np.cos(psi) - gy * np.sin(psi)
    lat = yv + gx * np.sin(psi) + gy * np.cos(psi) - 0.5 * kappa * s ** 2
    on = ok & ((np.abs(lat - LANE_W / 2) < LINE_W / 2) | (np.abs(lat + LANE_W / 2) < LINE_W / 2))
    prob = np.where(on, 0.95, 0.02).astype(np.float32)
    frame = np.full((H, W, 3), 90, np.uint8)
    frame[on] = 235
    return prob, frame


def pipeline(prob, frame):
    prob = apply_roi_mask(prob)
    wf, wp = warp_to_birdeye(frame, prob, build_ipm_transform(W, H))
    wm = (wp > 0.5).astype(np.uint8)
    wh, ww = wm.shape[:2]
    (ly, lx, lc), (ry, rx, rc) = extract_birdeye_boundaries(wm, wp, wf)
    left = fit_lane_curve(ly, lx, lc) if len(ly) else None
    right = fit_lane_curve(ry, rx, rc) if len(ry) else None
    if left is None or right is None:
        return None
    eyl = float(np.clip(wh - 1, left.y_min, left.y_max))
    eyr = float(np.clip(wh - 1, right.y_min, right.y_max))
    width_px = right.x_at(eyr) - left.x_at(eyl)
    centre_x = (left.x_at(eyl) + right.x_at(eyr)) / 2.0
    cte = (centre_x - ww / 2.0) * LANE_W / width_px
    heading = float(np.degrees(np.arctan((left.slope_at(eyl) + right.slope_at(eyr)) / 2.0)))
    return cte, heading


CASES = [
    ("centred, straight", 0.0, 0.0, 0.0, "none"),
    ("car 0.5 m LEFT of centre", 0.5, 0.0, 0.0, "RIGHT(+)"),
    ("car 0.5 m RIGHT of centre", -0.5, 0.0, 0.0, "LEFT(-)"),
    ("car yawed 3 deg LEFT (CCW)", 0.0, np.radians(3), 0.0, "RIGHT(+)"),
    ("car yawed 3 deg RIGHT (CW)", 0.0, np.radians(-3), 0.0, "LEFT(-)"),
    ("centred, left curve R127.5", 0.0, 0.0, 1 / 127.5, "LEFT(-)"),
    ("centred, right curve R127.5", 0.0, 0.0, -1 / 127.5, "RIGHT(+)"),
]
print(f"{'case':30s} {'cte_m':>7s} {'head_deg':>9s} {'stanley_deg':>11s}  correct steer")
for name, yv, psi, k, need in CASES:
    r = pipeline(*render(yv, psi, k))
    if r is None:
        print(f"{name:30s} lanes not found")
        continue
    cte, head = r
    print(f"{name:30s} {cte:7.3f} {head:9.2f} {stanley_steering_deg(cte, head, 8.33):11.2f}  {need}")
```

Run once: `python tools/adas2026/verify_signs.py "E:/Abaja Labs/2026/LKA - Matlab/ADAS_2026/src"`.
Expected (recorded 2026-10-16): yawed 3° left gives heading −0.72° and a command of −0.77° (wrong direction); 0.5 m left gives cte 0.117 m and heading 3.97° (offset leaking into heading); R127.5 curves give heading ≈ 0.

- [ ] **Step 8: Write `docs/adas2026_audit.md`**

Fill the two `<from Step 6>` cells of the replica table with the numbers printed by `verify_adas2025_control()`.

````markdown
# ADAS_2026 (2025 lane-keeping stack): audit

**Scope.** `ADAS_2026/` (RatheeswarS/ADAS_2026, 3 commits, last 2026-10-16): about 6 k lines of team Python
(`src/`, runners, bench tools), vendored YOLOP, `arduinocode.ino`. It was reviewed on 2026-10-16 by three
independent code reviews (perception, control/safety, runners/actuator/firmware), a synthetic-camera
test that runs the team's own functions (`tools/adas2026/verify_signs.py`) and a closed-loop MATLAB
replica of the control chain (`scripts/verify_adas2025_control.m`). The folders
`Team-Asterix---Software-Submission/` and `Team-Asterix-AEB/` are empty.

**Pipeline as built.** ZED left eye → YOLOP lane-line segmentation (BDD100K weights, TensorRT/ONNX)
→ fixed hand-tuned bird's-eye trapezoid → blob selection + quadratic fit in pixels → CTE/heading
→ 4-state Kalman `[cte, cte_rate, heading, heading_rate]` → Stanley (K 0.8) → slew 60°/s → 1 s mean
→ Arduino `STA<deg>` (CONFIG mode).

## Findings

Severity: **C** critical (wrong steering or unsafe hardware behaviour), **H** high, **M** medium, **L** low/docs.

| ID | Where | Finding | Evidence | Fix (Jetson phase) |
|---|---|---|---|---|
| C1 | `steering_v2.py:81` | Heading term has the wrong sign: a car pointing left of the lane is steered further left (unstable heading loop). | Synthetic test: car yawed 3° left gives a −0.77° (right-positive) command, i.e. a left turn. Replica: see table below. | `np.radians(-heading_deg)` |
| C2 | `steering_v2.py:93` | Yaw-damping sign is flipped as well (latent: only with an IMU). Must be fixed together with C1. | Derivation in review; `lka_adas2025_steer` with `UseImu`. | `+ k_yaw_damp * yaw_rate_dps` |
| C3 | `sda_serial_backend.py:106`, `run_video_to_arduino.py:97`, `run_video_annotated*.py --port`, `bench_steering_to_arduino.py:70` | The `SDA <v>` protocol is not implemented in `arduinocode.ino`; the line falls into the `S<kmh>` branch (`:889-895`) and starts the throttle PID. | Firmware parser. | Retire SDA; parse `S<digits>` strictly and NAK the rest. |
| C4 | `arduinocode.ino:687-696` | No steering watchdog in CONFIG mode: on link loss the wheels hold the last angle. | Code path `if (sysState != STATE_RUN) return;` | Ramp the steering target to centre on timeout in every state. |
| C5 | `arduinocode.ino:692` | `setBrakePercent(100)` is re-issued every loop while tripped and resets the actuator stall timer, so `ACT_MOVE_TIMEOUT_MS` never fires (motor at full PWM). | Code path `:355-356, :406`. | Latch on the transition only. |
| H1 | `ipm.py` | Uncalibrated bird's-eye: heading is not metric, lateral offset leaks into heading, curvature is invisible at the read row; on the simulator camera (pitch 0, horizon at row 360) the trapezoid top (row 302) lies above the horizon. | Synthetic test: heading ×0.24, 0.5 m offset → 3.97°, R127.5 → 0.02°. | Calibrate the ZED (intrinsics + mount) and use a metric IPM (as done in `lka_camera_model.m`). |
| H2 | `lane_detection_node_birdeye.py:108` (+ runners) | `lane_width_m = 3.0`; the track lanes are 3.5 m, so CTE is 14 % low and the jump gate is off. | Code. | `--lane-width 3.5` |
| H3 | `arduino_steering_link.py:220, :224` | The 1 s mean adds about 1 s of delay (≈ 8 m at 30 km/h); the lane-loss ramp is a 20° step per window, not 20°/s. | Replica table. | Per-frame command after C1; ramp with `rate * dt`. |
| H4 | `kalman_tracker.py:114`, `safety_monitor.py:72` | IMU sign errors (latent): the dropout prediction turns the wrong way; the yaw plausibility check trips for any steer above about 1.2°. | Review derivation. | Use one gyro sign convention (+ left) everywhere. |
| H5 | `bench_steering_to_arduino.py:72-76` | `STR/STL` are relative in the firmware (`:864-873`); the bench tool integrates commands towards the stop. | Firmware parser. | Send absolute `STA<deg>`. |
| H6 | `conditioner.py:54`, `safety_monitor.py:90`, `steering_v2.py:94` | NaN is never rejected and reaches the Kalman filter and the Arduino. | Review. | `math.isfinite` guards. |
| M1 | `lane_detection_node_birdeye.py:496` | A reconstructed (synthetic) side is committed as the continuity reference, so the real line is never re-acquired while the other side persists. | Review. | Commit direct fits only (done in `lka_vision_lane_step.m`). |
| M2 | `lane_detection_node_birdeye.py:472` | The per-side 1.8 s coast is unreachable (reconstruction runs first). | Review. | Coast before reconstructing, or update the docs. |
| M3 | `bird_eye_extraction.py:103` | The continuity search has no distance gate (last tier accepts any blob, even on the other side). | Review. | Windowed search (done in `lka_search_boundary.m`). |
| M4 | `kalman_tracker.py:54, :58` | Stale is a frame count (15) while the hold is time-based; `MAX_PREDICT_DT` silently truncates long gaps. | Review. | Time-based stale. |
| M5 | `lane_detection_node_birdeye.py:405`, `ego_state_estimator.py:54` | Jump gate and Stanley use the 5 m/s fallback speed, not the measured speed. | Review. | Use `$TEL` speed; default 8.33 m/s. |
| M6 | `lane_detection_node_birdeye.py:486` | Pair width and centre are evaluated at different rows when one fit ends higher. | Review. | One shared row. |
| M7 | `lane_detection_node_birdeye.py:450` | The width calibrator also learns from latched shoulders (both-historied pairs). | Review. | Gate `observe` with `_pair_width_ok`. |
| M8 | `conditioner.py:62` | Fixed 3 m/s CTE-rate limit rejects genuine corrections above ~21° heading at 30 km/h. | Review. | Scale with speed. |
| M9 | firmware `:160-161` vs `steering_v2.py:59` | ±24° firmware lock vs ±30° Python clamp. | Code. | Measure the road-wheel lock and use one value. |
| M10 | `steering_v2.py:108` | Slew dt uncapped: after a stall the limiter allows a large jump. | Review. | Cap dt at 0.1 s. |
| M11 | `run_video_annotated - v2.py:416, :906` | v2 drops the TensorRT default; stereo recovery overwrites the tracked state after the Kalman filter. | Review. | Restore `default_weights()`; feed recovery into the tracker. |
| L1 | `README.md`, `BRIDGE.md`, `BENCH.md`, `LKA_README.md` | Three different wire protocols are documented (SDA / STR / STA); only STA matches the firmware. `reference_generator.py:27` documents the CTE sign backwards. | Docs vs code. | One protocol section. |
| L2 | `tests/test_lane_hold_gate.py` | Imports torch through the node module; covers 3 helpers, not the coast/decay it claims. | Review. | Lazy import, add coast tests. |

### Closed-loop replica (official track, Baja plant, ideal metric measurement 4 m ahead)

| Variant | Max lateral offset | In lane |
|---|---|---|
| as coded (C1 + 1 s mean + 3.0 m + 5 m/s) | <from Step 6> | <from Step 6> |
| fixed (sign, per-frame, 3.5 m, measured speed) | <from Step 6> | <from Step 6> |

Figure: `results/adas2025/adas2025_replica_lateral_offset.png`.

## What was kept and where it lives now

| ADAS_2026 idea | MATLAB implementation |
|---|---|
| YOLOP lane head, letterbox, ImageNet normalisation on BGR | `lka_detector_onnx_seg.m` + `models/perception/yolop.json` |
| Bird's-eye warp | `lka_camera_model.m` (calibrated `monoCamera` / `birdsEyeView`, metric) |
| Continuity-aware selection, lateral jump gate | warm prior window in `lka_search_boundary.m` |
| Quality tiers, partial fits | boundary `Strength` from coverage and probability (`lka_fit_boundary.m`) |
| Width consistency gate, width calibrator | metric lane-width estimate and pair check in `lka_vision_lane_step.m` |
| Missing-side reconstruction, hold/coast, confidence decay | existing `lka_estimate_lane_center.m`, Kalman coast, supervisor DEGRADED/TAKEOVER |
| Kalman `[cte, rate, heading, rate]` | existing `lka_lane_tracker_step.m` (vehicle-motion model) |
| Stanley K 0.8 chain | `lka_adas2025_steer.m` (replica, verification only) |
| Annotated HUD video | `LkaVisionRecorder.m` |

## Jetson-phase fix queue (in this order)

1. C1 + C2 (one commit), bench-check with `heading=+5, cte=0 → negative` and `cte=+0.5, heading=0 → positive`.
2. C3–C5 firmware (strict parser, steering watchdog, brake latch), then a wheels-up bench test.
3. H1: calibrate the ZED, metric IPM (port `lka_camera_model.m` logic).
4. H2, H3, M5: lane width 3.5, per-frame command, measured speed.
5. H4, H6: gyro sign, NaN guards.
6. M1–M4, M6–M11, L1–L2.
````

- [ ] **Step 9: Commit**

```bash
git add docs/adas2026_audit.md tools/adas2026 src/legacy scripts/verify_adas2025_control.m tests/unit_tests/test_adas2025_replica.m results/adas2025
git commit -m "docs(adas2026): audit of the 2025 stack and closed-loop replica of its steering chain

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 3: Vision parameters and calibrated camera model

**Files:**
- Create: `src/vision/lka_vision_params.m`, `src/vision/lka_camera_model.m`
- Modify: `src/utilities/lka_build_params.m` (append before the final `end`)
- Test: `tests/unit_tests/test_vision_camera.m`

**Interfaces:**
- Produces: `V = lka_vision_params(profile)` (struct `LKA.Vision`, fields below); `cam = lka_camera_model(C, B)` with fields `Sensor` (monoCamera, vehicle origin = rear axle), `Bev` (birdsEyeView), `RowX` (n×1 m, row 1 = far), `ColY` (1×m m, column 1 = left), `XNear`, `XFar`, `Params` (= C).

- [ ] **Step 1: Write the failing test** `tests/unit_tests/test_vision_camera.m`

```matlab
% ==============================================================================
% test_vision_camera.m
% Camera model of the lane detector: pixel <-> vehicle frame (ISO 8855, rear axle).
% ==============================================================================

function tests = test_vision_camera
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    testCase.TestData.V = LKA.Vision;
end

function testPitchZeroRowToDistance(testCase)
    C = struct('FocalLength', [1109 1109], 'PrincipalPoint', [640 360], 'ImageSize', [720 1280], ...
        'X', 2.6, 'Y', 0, 'Height', 1.1, 'PitchDeg', 0);
    cam = lka_camera_model(C, testCase.TestData.V.Bev);
    xy = imageToVehicle(cam.Sensor, [640 560]);
    testCase.verifyEqual(xy(1), 2.6 + 1.1 * 1109 / 200, 'AbsTol', 1e-6);
    testCase.verifyEqual(xy(2), 0, 'AbsTol', 1e-9);
end

function testImageLeftIsVehicleLeft(testCase)
    cam = lka_camera_model(testCase.TestData.V.Camera, testCase.TestData.V.Bev);
    xy = imageToVehicle(cam.Sensor, [400 600]);
    testCase.verifyGreaterThan(xy(2), 0);
end

function testPitchDownMovesHorizonUp(testCase)
    cam = lka_camera_model(testCase.TestData.V.Camera, testCase.TestData.V.Bev);   % pitch 8 deg
    xy = imageToVehicle(cam.Sensor, [640 210]);    % 0.3 deg below the horizon (row 204.1)
    testCase.verifyGreaterThan(xy(1), 100);
end

function testBevAxes(testCase)
    cam = lka_camera_model(testCase.TestData.V.Camera, testCase.TestData.V.Bev);
    testCase.verifyLessThan(cam.RowX(2) - cam.RowX(1), 0);       % row 1 is the far end
    testCase.verifyLessThan(cam.ColY(2) - cam.ColY(1), 0);       % column 1 is the left edge
    testCase.verifyEqual(cam.RowX(end), cam.XNear, 'AbsTol', 0.06);
    testCase.verifyEqual(cam.ColY(1), testCase.TestData.V.Bev.HalfWidth, 'AbsTol', 0.06);
end

function testNearLimitIsInsideImage(testCase)
    cam = lka_camera_model(testCase.TestData.V.Camera, testCase.TestData.V.Bev);
    uv = vehicleToImage(cam.Sensor, [cam.XNear 0]);
    testCase.verifyLessThanOrEqual(uv(2), 720);
end

function testParamsInLkaStruct(testCase)
    V = testCase.TestData.V;
    testCase.verifyTrue(isfolder(V.DetectorDir));
    testCase.verifyTrue(isfile(V.Bench.ScenePath));
    testCase.verifyEqual(V.Ts, 0.05);
    testCase.verifyTrue(ismember(V.Detector, {'yolop', 'cnn', 'classic'}));
end
```

- [ ] **Step 2: Run it, expect FAIL**

Run: `runtests('tests/unit_tests/test_vision_camera.m')`
Expected: FAIL in setupOnce, "Unrecognized field name 'Vision'".

- [ ] **Step 3: Create `src/vision/lka_vision_params.m`**

```matlab
function V = lka_vision_params(profile)
% LKA_VISION_PARAMS Calibration of the camera lane detector (LKA.Vision).
%
% Syntax:
%   V = lka_vision_params(profile)        profile 'baja' | 'skeleton'
%
% Camera: Simulation 3D Camera of aBaja2026.slx (1280x720, f = 1109 px, no distortion).
% Extrinsics in the LKA vehicle frame (ISO 8855, origin = rear-axle centre on the ground):
%   'baja'     ZED-like mount used by the synthetic tests and LKA_Vision_TestBench.
%   'skeleton' "Hood center" mount of the Muscle car in aBaja2026.slx; initial estimate,
%              replaced by the calibrate_vision_bench measurement (Task 11) and the
%              RoadRunner co-simulation check (Phase 5).
% Pipeline: detector -> probability map -> metric bird's-eye (birdsEyeView) ->
% sliding-window search (Search) -> robust fit (Fit) -> frame-to-frame logic (Track).

    arguments
        profile (1,:) char {mustBeMember(profile, {'baja', 'skeleton'})}
    end

    % ---------------------------------------------------------- camera
    V.Camera.FocalLength    = [1109 1109];   % [px]
    V.Camera.PrincipalPoint = [640 360];     % [px]
    V.Camera.ImageSize      = [720 1280];    % [rows cols]
    switch profile
        case 'baja'
            V.Camera.X        = 1.60;        % ahead of the rear axle [m]
            V.Camera.Y        = 0;
            V.Camera.Height   = 1.20;        % [m]
            V.Camera.PitchDeg = 8;           % nose down [deg]
        case 'skeleton'
            V.Camera.X        = 2.60;
            V.Camera.Y        = 0;
            V.Camera.Height   = 1.10;
            V.Camera.PitchDeg = 0;
    end

    % ---------------------------------------------------------- bird's-eye view
    V.Bev.Resolution = 0.05;     % [m/px]
    V.Bev.Range      = 30;       % look-ahead from the camera [m]
    V.Bev.HalfWidth  = 6;        % lateral half-width [m]
    V.Bev.NearMargin = 0.5;      % start this far beyond the first visible ground point [m]

    % ---------------------------------------------------------- detector
    V.Detector    = 'yolop';     % 'yolop' | 'cnn' | 'classic'
    V.DetectorDir = '';          % set by lka_build_params (models/perception)
    V.UseGpu      = true;

    % ---------------------------------------------------------- sliding-window search
    V.Search.ProbThreshold = 0.5;
    V.Search.SliceLength   = 1.0;    % [m]
    V.Search.ColdHalfWidth = 0.9;    % window half-width around the nominal lane [m]
    V.Search.WarmHalfWidth = 0.45;   % around the previous boundary (lateral jump gate) [m]
    V.Search.MinSliceMass  = 4;      % summed probability needed in a slice window [px]
    V.Search.MaxGap        = 8;      % stop after this length without a hit [m]
    V.Search.MinCenterDist = 0.4;    % cold search keeps this far from the vehicle axis [m]
    V.Search.LineHalfWidth = 0.25;   % pixels this close to the slice peak feed the fit [m]

    % ---------------------------------------------------------- robust fit
    V.Fit.MinPoints        = 60;
    V.Fit.MinSpan          = 4;               % [m]
    V.Fit.QuadSpan         = 8;               % quadratic above this span [m]
    V.Fit.CubicSpan        = 15;              % cubic above this span [m]
    V.Fit.Ridge            = [0 0 2 20];      % penalty on the scaled [c0 c1 c2 c3]
    V.Fit.HuberDelta       = 0.08;            % [m]
    V.Fit.Iterations       = 3;
    V.Fit.FullStrengthSpan = 12;              % span that gives full strength [m]
    V.Fit.ProbFull         = 0.7;             % mean probability that gives full strength

    % ---------------------------------------------------------- frame-to-frame logic
    V.Track.HoldTime     = 1.0;               % previous direct fit is a prior this long [s]
    V.Track.WidthInit    = 3.5;               % [m]
    V.Track.WidthTau     = 2.0;               % [s]
    V.Track.WidthTol     = 0.6;               % pair width vs estimate [m]
    V.Track.WidthMin     = 2.5;               % [m]
    V.Track.WidthMax     = 5.0;               % [m]
    V.Track.HeadingMax   = deg2rad(35);       % [rad]
    V.Track.CurvatureMax = 0.1;               % [1/m]

    % ---------------------------------------------------------- outputs
    V.PreviewSize  = [135 60];                % bird's-eye preview for the recorder [px]
    V.LatencySteps = 1;                       % modelled processing delay [frames]

    % ---------------------------------------------------------- Unreal test bench
    V.Bench.ScenePath    = '';                % set by lka_build_params (data/scenes/LKA/LKA.usd)
    V.Bench.FrameMap     = 3;                 % aBaja2026 InitFcn: X = x, Y = -y, yaw = -psi
    V.Bench.MountZOffset = 0;                 % camera height correction [m] (calibrate_vision_bench)
    V.Bench.PitchSign    = 1;                 % sign of the Simulation 3D Camera pitch mount
    V.Bench.LaneLabelId  = 0;                 % semantic id of lane markings (calibrate_vision_bench)
    V.Bench.VideoFile    = '';
    V.Bench.DatasetDir   = '';
    V.Bench.Decimation   = 5;                 % dataset: keep every Nth frame
end
```

- [ ] **Step 4: Create `src/vision/lka_camera_model.m`**

```matlab
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
```

- [ ] **Step 5: Wire into `src/utilities/lka_build_params.m`**

Insert before the final `end` of the function (after the `LKA.Cosim = Cosim;` line):

```matlab

    % ---------------------------------------- Camera lane detector (lka_vision_params.m)
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    LKA.Vision = lka_vision_params(profile);
    LKA.Vision.DetectorDir = fullfile(root, 'models', 'perception');
    LKA.Vision.Bench.ScenePath = fullfile(root, 'data', 'scenes', 'LKA', 'LKA.usd');
    LKA.Vision.Ts = LKA.Ts;
```

- [ ] **Step 6: Run the test, expect PASS**

Run: `runtests('tests/unit_tests/test_vision_camera.m')`
Expected: 6 passed. Then confirm nothing regressed: `runtests('tests/unit_tests/test_lka_core.m')`, expected 19 passed.

- [ ] **Step 7: Commit**

```bash
git add src/vision src/utilities/lka_build_params.m tests/unit_tests/test_vision_camera.m
git commit -m "feat(vision): calibrated camera model and metric bird's-eye view

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 4: Synthetic lane-scene renderer (test fixture)

**Files:**
- Create: `tests/helpers/lka_default_scene.m`, `tests/helpers/lka_render_lane_scene.m`
- Test: `tests/unit_tests/test_vision_renderer.m`

**Interfaces:**
- Consumes: `lka_camera_model` (Task 3).
- Produces: `scene = lka_default_scene()` (fields `Offset` m + left, `Yaw` rad + CCW, `Curvature` 1/m, `Width` 3.5, `LineWidth` 0.125, `Dashed` [l r] logical, `DashLen` 3, `GapLen` 6, `Missing` [l r] logical, `Distractor` NaN or lane-frame lateral position m); `[img, prob, gt] = lka_render_lane_scene(cam, scene)` → `img` uint8 H×W×3, `prob` single H×W (0.95 on lines, 0.02 elsewhere), `gt.Left`/`gt.Right` with `LateralOffset, HeadingAngle, Curvature, CurvatureDerivative` (vehicle frame, rear axle).

- [ ] **Step 1: Write the failing test** `tests/unit_tests/test_vision_renderer.m`

```matlab
% ==============================================================================
% test_vision_renderer.m
% The synthetic renderer used by the vision tests draws lines where the ground
% truth says, with the LKA sign convention.
% ==============================================================================

function tests = test_vision_renderer
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    testCase.TestData.cam = lka_camera_model(LKA.Vision.Camera, LKA.Vision.Bev);
end

function testLinePixelsWhereExpected(testCase)
    cam = testCase.TestData.cam;
    [img, prob] = lka_render_lane_scene(cam, lka_default_scene());
    uv = round(vehicleToImage(cam.Sensor, [10 1.75; 10 -1.75; 10 0]));
    testCase.verifyEqual(prob(uv(1, 2), uv(1, 1)), single(0.95));
    testCase.verifyEqual(prob(uv(2, 2), uv(2, 1)), single(0.95));
    testCase.verifyEqual(prob(uv(3, 2), uv(3, 1)), single(0.02));
    testCase.verifyEqual(size(img), [720 1280 3]);
end

function testGroundTruthOffset(testCase)
    s = lka_default_scene(); s.Offset = 0.5;          % vehicle 0.5 m left of the centre
    [~, ~, gt] = lka_render_lane_scene(testCase.TestData.cam, s);
    testCase.verifyEqual(gt.Left.LateralOffset, 1.25, 'AbsTol', 1e-3);
    testCase.verifyEqual(gt.Right.LateralOffset, -2.25, 'AbsTol', 1e-3);
end

function testGroundTruthYaw(testCase)
    s = lka_default_scene(); s.Yaw = deg2rad(3);      % vehicle yawed left -> lane rotates right
    [~, ~, gt] = lka_render_lane_scene(testCase.TestData.cam, s);
    testCase.verifyEqual(gt.Left.HeadingAngle, -deg2rad(3), 'AbsTol', deg2rad(0.01));
end

function testGroundTruthCurvature(testCase)
    s = lka_default_scene(); s.Curvature = 1 / 127.5;
    [~, ~, gt] = lka_render_lane_scene(testCase.TestData.cam, s);
    testCase.verifyEqual(gt.Left.Curvature, 1 / 127.5, 'AbsTol', 1e-4);
    testCase.verifyEqual(gt.Right.Curvature, 1 / 127.5, 'AbsTol', 1e-4);
end

function testMissingLineNotDrawn(testCase)
    cam = testCase.TestData.cam;
    s = lka_default_scene(); s.Missing = [false true];
    [~, prob] = lka_render_lane_scene(cam, s);
    uv = round(vehicleToImage(cam.Sensor, [10 -1.75]));
    testCase.verifyEqual(prob(uv(2), uv(1)), single(0.02));
end
```

- [ ] **Step 2: Run it, expect FAIL**

Run: `runtests('tests/unit_tests/test_vision_renderer.m')`
Expected: FAIL with "Undefined function 'lka_render_lane_scene'".

- [ ] **Step 3: Create `tests/helpers/lka_default_scene.m`**

```matlab
function scene = lka_default_scene()
% LKA_DEFAULT_SCENE Straight 3.5 m lane, vehicle centred and aligned (renderer input).
    scene = struct('Offset', 0, 'Yaw', 0, 'Curvature', 0, 'Width', 3.5, 'LineWidth', 0.125, ...
        'Dashed', [false false], 'DashLen', 3, 'GapLen', 6, 'Missing', [false false], ...
        'Distractor', NaN);
end
```

- [ ] **Step 4: Create `tests/helpers/lka_render_lane_scene.m`**

```matlab
function [img, prob, gt] = lka_render_lane_scene(cam, scene)
% LKA_RENDER_LANE_SCENE Synthetic camera image of a lane with known geometry.
%
% Syntax:
%   [img, prob, gt] = lka_render_lane_scene(cam, scene)
%
% Every ground pixel of the camera (lka_camera_model) is projected to the road and
% tested against the lane lines. Lane frame: s along the lane, l to the left; centre
% line l = Curvature*s^2/2, lines at +-Width/2. The vehicle rear axle sits at
% (s, l) = (0, Offset) with heading Yaw (+ CCW) relative to the lane.
%
% Outputs:
%   img   uint8 RGB, asphalt 70, paint 235
%   prob  single lane-line probability (0.95 paint, 0.02 elsewhere)
%   gt    .Left / .Right ground-truth boundaries in the vehicle frame (rear axle):
%         LateralOffset, HeadingAngle, Curvature, CurvatureDerivative

    C = cam.Params;
    H = C.ImageSize(1); W = C.ImageSize(2);
    [u, v] = meshgrid(1:W, 1:H);
    horizon = C.PrincipalPoint(2) - C.FocalLength(2) * tan(deg2rad(C.PitchDeg));
    ground = v(:) > horizon + 2;
    xy = nan(numel(u), 2);
    xy(ground, :) = imageToVehicle(cam.Sensor, [u(ground) v(ground)]);
    x = xy(:, 1); y = xy(:, 2);
    ok = ground & x > C.X & x < C.X + 80;

    c = cos(scene.Yaw); s = sin(scene.Yaw);
    sLane = x .* c - y .* s;
    rel = scene.Offset + x .* s + y .* c - 0.5 * scene.Curvature * sLane.^2;
    hw = scene.LineWidth / 2;
    onL = abs(rel - scene.Width / 2) < hw;
    onR = abs(rel + scene.Width / 2) < hw;
    dashOn = mod(sLane, scene.DashLen + scene.GapLen) < scene.DashLen;
    if scene.Dashed(1), onL = onL & dashOn; end
    if scene.Dashed(2), onR = onR & dashOn; end
    if scene.Missing(1), onL(:) = false; end
    if scene.Missing(2), onR(:) = false; end
    on = ok & (onL | onR);
    if ~isnan(scene.Distractor)
        on = on | (ok & abs(rel - scene.Distractor) < hw);
    end
    on = reshape(on, H, W);

    gray = uint8(70 * ones(H, W));
    gray(on) = 235;
    img = repmat(gray, 1, 1, 3);
    prob = single(0.02 * ones(H, W));
    prob(on) = 0.95;

    gt.Left = boundary(+1);
    gt.Right = boundary(-1);

    function b = boundary(side)
        sv = linspace(-5, 60, 400)';
        lv = side * scene.Width / 2 + 0.5 * scene.Curvature * sv.^2 - scene.Offset;
        xb = sv * c + lv * s;
        yb = -sv * s + lv * c;
        keep = xb >= 0 & xb <= 40;
        p = polyfit(xb(keep), yb(keep), 3);
        b = struct('LateralOffset', p(4), 'HeadingAngle', atan(p(3)), ...
            'Curvature', 2 * p(2), 'CurvatureDerivative', 6 * p(1));
    end
end
```

- [ ] **Step 5: Run the test, expect PASS**

Run: `runtests('tests/unit_tests/test_vision_renderer.m')`
Expected: 5 passed.

- [ ] **Step 6: Commit**

```bash
git add tests/helpers tests/unit_tests/test_vision_renderer.m
git commit -m "test(vision): synthetic lane-scene renderer with ground truth

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 5: Sliding-window boundary search and robust fit

**Files:**
- Create: `src/vision/lka_search_boundary.m`, `src/vision/lka_fit_boundary.m`
- Test: `tests/unit_tests/test_vision_lanefit.m`

**Interfaces:**
- Consumes: `cam` (Task 3); renderer (Task 4) in tests.
- Produces: `[pts, nHit] = lka_search_boundary(bevProb, cam, prior, S)` with `prior.Fun` (@(x) y), `prior.HalfWidth` (m), `prior.Side` (+1 left / −1 right), `prior.Cold` (logical); `pts` = n×3 double `[x y w]`. And `[b, ok] = lka_fit_boundary(pts, F)` where `b` has exactly the `LaneBoundaryBus` fields in bus order: `LateralOffset, HeadingAngle, Curvature, CurvatureDerivative, Strength, ViewRange`.

- [ ] **Step 1: Write the failing test** `tests/unit_tests/test_vision_lanefit.m`

```matlab
% ==============================================================================
% test_vision_lanefit.m
% Sliding-window boundary search and robust polynomial fit in the metric BEV.
% ==============================================================================

function tests = test_vision_lanefit
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    testCase.TestData.V = LKA.Vision;
    testCase.TestData.cam = lka_camera_model(LKA.Vision.Camera, LKA.Vision.Bev);
end

% ------------------------------------------------------------------ fit
function testFitRecoversPolynomial(testCase)
    x = linspace(5, 30, 400)';
    y = 1.6 + 0.02 * x + 0.003 * x.^2;
    [b, ok] = lka_fit_boundary([x y 0.9 * ones(size(x))], testCase.TestData.V.Fit);
    testCase.verifyTrue(ok);
    testCase.verifyEqual(b.LateralOffset, 1.6, 'AbsTol', 1e-3);
    testCase.verifyEqual(b.HeadingAngle, atan(0.02), 'AbsTol', 1e-4);
    testCase.verifyEqual(b.Curvature, 0.006, 'AbsTol', 1e-4);
    testCase.verifyEqual(fieldnames(b)', {'LateralOffset', 'HeadingAngle', 'Curvature', ...
        'CurvatureDerivative', 'Strength', 'ViewRange'});
end

function testFitRejectsOutliers(testCase)
    rng(3);
    x = linspace(5, 30, 400)';
    y = 1.75 + 0 * x;
    bad = rand(size(x)) < 0.1;
    y(bad) = y(bad) + 1.0;
    b = lka_fit_boundary([x y 0.9 * ones(size(x))], testCase.TestData.V.Fit);
    testCase.verifyEqual(b.LateralOffset, 1.75, 'AbsTol', 0.02);
end

function testFitNeedsEnoughPoints(testCase)
    x = linspace(5, 30, 20)';
    [b, ok] = lka_fit_boundary([x x * 0 + 1.75 x * 0 + 0.9], testCase.TestData.V.Fit);
    testCase.verifyFalse(ok);
    testCase.verifyEqual(b.Strength, 0);
end

function testShortSpanIsLinearAndWeak(testCase)
    x = linspace(6, 11, 200)';
    y = 1.75 + 0.01 * x.^2;
    [b, ok] = lka_fit_boundary([x y 0.9 * ones(size(x))], testCase.TestData.V.Fit);
    testCase.verifyTrue(ok);
    testCase.verifyEqual(b.Curvature, 0);
    testCase.verifyLessThan(b.Strength, 0.5);
end

% ------------------------------------------------------------------ search
function testColdSearchFindsBothLines(testCase)
    [bev, cam, S] = bevOf(testCase, lka_default_scene());
    L = lka_search_boundary(bev, cam, coldPrior(+1), S);
    R = lka_search_boundary(bev, cam, coldPrior(-1), S);
    testCase.verifyEqual(mean(L(:, 2)), 1.75, 'AbsTol', 0.03);
    testCase.verifyEqual(mean(R(:, 2)), -1.75, 'AbsTol', 0.03);
end

function testDashedLineStillFound(testCase)
    s = lka_default_scene(); s.Dashed = [true false];
    [bev, cam, S] = bevOf(testCase, s);
    [L, nHit] = lka_search_boundary(bev, cam, coldPrior(+1), S);
    testCase.verifyGreaterThanOrEqual(nHit, 4);
    testCase.verifyEqual(mean(L(:, 2)), 1.75, 'AbsTol', 0.04);
end

function testDistractorInsideWindowIgnored(testCase)
    s = lka_default_scene(); s.Distractor = 1.1;       % extra line 0.65 m inside the left line
    [bev, cam, S] = bevOf(testCase, s);
    L = lka_search_boundary(bev, cam, coldPrior(+1), S);
    testCase.verifyEqual(mean(L(:, 2)), 1.75, 'AbsTol', 0.05);
end

function testCurveFollowed(testCase)
    s = lka_default_scene(); s.Curvature = 1 / 50;
    [bev, cam, S] = bevOf(testCase, s);
    [L, nHit] = lka_search_boundary(bev, cam, coldPrior(+1), S);
    testCase.verifyGreaterThanOrEqual(nHit, 10);
    farX = max(L(:, 1));
    testCase.verifyGreaterThan(farX, 15);
end

% ------------------------------------------------------------------ helpers
function [bev, cam, S] = bevOf(testCase, scene)
    cam = testCase.TestData.cam;
    [~, prob] = lka_render_lane_scene(cam, scene);
    bev = double(transformImage(cam.Bev, prob));
    S = testCase.TestData.V.Search;
end

function p = coldPrior(side)
    p = struct('Fun', @(x) side * 1.75 + 0 * x, 'HalfWidth', 0.9, 'Side', side, 'Cold', true);
end
```

- [ ] **Step 2: Run it, expect FAIL**

Run: `runtests('tests/unit_tests/test_vision_lanefit.m')`
Expected: FAIL with "Undefined function 'lka_fit_boundary'".

- [ ] **Step 3: Create `src/vision/lka_fit_boundary.m`**

```matlab
function [b, ok] = lka_fit_boundary(pts, F)
% LKA_FIT_BOUNDARY Robust weighted polynomial y(x) of one lane boundary.
%
% Syntax:
%   [b, ok] = lka_fit_boundary(pts, F)        F = LKA.Vision.Fit
%
% pts = [x y w]: support pixels in the vehicle frame [m] with detector probability w.
% Degree from the longitudinal span (linear < QuadSpan <= quadratic < CubicSpan <= cubic),
% ridge penalty on the higher coefficients (scaled abscissa x/10) against wild
% extrapolation, Huber IRLS (HuberDelta) against outliers.
% Output b uses the LaneBoundaryBus fields and order:
%   y(x) = LateralOffset + tan(HeadingAngle)*x + Curvature*x^2/2 + CurvatureDerivative*x^3/6
% Strength = min(1, span/FullStrengthSpan) * min(1, mean(w)/ProbFull); 0 when ok is false.

    b = struct('LateralOffset', 0, 'HeadingAngle', 0, 'Curvature', 0, ...
        'CurvatureDerivative', 0, 'Strength', 0, 'ViewRange', 0);
    ok = false;
    if size(pts, 1) < F.MinPoints
        return
    end
    x = pts(:, 1); y = pts(:, 2); w0 = pts(:, 3);
    span = max(x) - min(x);
    if span < F.MinSpan
        return
    end
    if span >= F.CubicSpan
        deg = 3;
    elseif span >= F.QuadSpan
        deg = 2;
    else
        deg = 1;
    end

    A = (x / 10) .^ (0:deg);
    R = diag(F.Ridge(1:deg + 1));
    w = w0;
    for it = 1:F.Iterations
        Aw = A .* w;
        cs = (A' * Aw + R) \ (Aw' * y);
        r = y - A * cs;
        w = w0 .* min(1, F.HuberDelta ./ max(abs(r), eps));
    end
    c = [cs; zeros(3 - deg, 1)] ./ (10 .^ (0:3))';

    b.LateralOffset = c(1);
    b.HeadingAngle = atan(c(2));
    b.Curvature = 2 * c(3);
    b.CurvatureDerivative = 6 * c(4);
    b.Strength = min(1, span / F.FullStrengthSpan) * min(1, mean(w0) / F.ProbFull);
    b.ViewRange = max(x);
    ok = true;
end
```

- [ ] **Step 4: Create `src/vision/lka_search_boundary.m`**

```matlab
function [pts, nHit] = lka_search_boundary(bevProb, cam, prior, S)
% LKA_SEARCH_BOUNDARY Sliding-window search of one lane boundary in the metric BEV.
%
% Syntax:
%   [pts, nHit] = lka_search_boundary(bevProb, cam, prior, S)      S = LKA.Vision.Search
%
% The BEV is cut into SliceLength slices from cam.XNear to cam.XFar. In each slice a
% window of +-prior.HalfWidth around the expected boundary position is searched; the
% column profile above ProbThreshold is split into runs and the run closest to the
% window centre is taken (a second line or a road arrow inside the window is ignored).
% The expected position comes from the prior for the first hit, then from a straight
% line through the last (up to 6) slice peaks. A cold prior (no previous fit) also keeps
% the window on its own side of the vehicle (|y| > MinCenterDist).
%
% Inputs:
%   bevProb - BEV probability image (double, rows = cam.RowX, columns = cam.ColY)
%   prior   - .Fun @(x) expected y [m], .HalfWidth [m], .Side +1/-1, .Cold logical
% Outputs:
%   pts     - [x y w] support pixels [m, m, probability]
%   nHit    - number of slices with a detection

    rowX = cam.RowX;
    colY = cam.ColY;
    edges = cam.XNear:S.SliceLength:cam.XFar;
    pts = zeros(0, 3);
    nHit = 0;
    px = zeros(1, 0); py = zeros(1, 0);    % slice peaks found so far
    gap = 0;
    for k = 1:numel(edges) - 1
        rows = rowX >= edges(k) & rowX < edges(k + 1);
        if ~any(rows)
            continue
        end
        xMid = 0.5 * (edges(k) + edges(k + 1));
        yc = expected(xMid);
        cols = colY >= yc - prior.HalfWidth & colY <= yc + prior.HalfWidth;
        if prior.Cold
            cols = cols & prior.Side * colY > S.MinCenterDist;
        end
        cIdx = find(cols);
        block = bevProb(rows, cols);
        block(block < S.ProbThreshold) = 0;
        prof = sum(block, 1);
        runs = bwlabel(prof > 0);
        best = 0; bestDist = inf; yPeak = NaN;
        for j = 1:max(runs)
            m = runs == j;
            mass = sum(prof(m));
            if mass < S.MinSliceMass
                continue
            end
            yRun = sum(prof(m) .* colY(cIdx(m))) / mass;
            if abs(yRun - yc) < bestDist
                best = j; bestDist = abs(yRun - yc); yPeak = yRun;
            end
        end
        if best == 0
            gap = gap + S.SliceLength;
            if gap > S.MaxGap && nHit > 0
                break
            end
            continue
        end
        gap = 0;
        nHit = nHit + 1;
        px(end + 1) = xMid; py(end + 1) = yPeak; %#ok<AGROW>
        [r, cc] = find(block);
        wv = block(block > 0);
        rIdx = find(rows);
        xs = reshape(rowX(rIdx(r)), [], 1);
        ys = reshape(colY(cIdx(cc)), [], 1);
        near = abs(ys - yPeak) <= S.LineHalfWidth;
        pts = [pts; xs(near), ys(near), wv(near)]; %#ok<AGROW>
    end

    function y = expected(xq)
        if numel(px) >= 3
            n = min(numel(px), 6);
            p = polyfit(px(end - n + 1:end), py(end - n + 1:end), 1);
            y = polyval(p, xq);
        elseif ~isempty(px)
            y = prior.Fun(xq) + (py(end) - prior.Fun(px(end)));
        else
            y = prior.Fun(xq);
        end
    end
end
```

- [ ] **Step 5: Run the test, expect PASS**

Run: `runtests('tests/unit_tests/test_vision_lanefit.m')`
Expected: 8 passed.

- [ ] **Step 6: Commit**

```bash
git add src/vision/lka_search_boundary.m src/vision/lka_fit_boundary.m tests/unit_tests/test_vision_lanefit.m
git commit -m "feat(vision): sliding-window boundary search with distance gate and robust fit

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 6: Per-frame vision step (continuity, pair check, width, hold)

**Files:**
- Create: `src/vision/lka_vision_state_init.m`, `src/vision/lka_vision_lane_step.m`
- Test: `tests/unit_tests/test_vision_lane_step.m`

**Interfaces:**
- Consumes: `lka_search_boundary`, `lka_fit_boundary` (Task 5), `lka_camera_model` (Task 3).
- Produces: `state = lka_vision_state_init(V)` (`Width`, `Left`/`Right` = `{Valid, Age, Coeffs[1x4]}`); `[sensor, state, info] = lka_vision_lane_step(bevProb, cam, V, state, dt)`. `sensor` is a `LaneSensorBus` struct (`Left`, `Right`, `FrameValid` logical); `info` = 1×5 `[nPtsL nPtsR foundL foundR width]`.

- [ ] **Step 1: Write the failing test** `tests/unit_tests/test_vision_lane_step.m`

```matlab
% ==============================================================================
% test_vision_lane_step.m
% Camera frame (probability map) -> LaneSensorBus: accuracy, signs, robustness.
% Tolerances: offset 5 cm, heading 0.4 deg, curvature 1.5e-3 1/m (baja camera).
% ==============================================================================

function tests = test_vision_lane_step
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
    evalin('base', 'create_lka_buses;');
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    testCase.TestData.V = LKA.Vision;
    testCase.TestData.cam = lka_camera_model(LKA.Vision.Camera, LKA.Vision.Bev);
end

function testStraightCentred(testCase), checkScene(testCase, lka_default_scene()); end

function testOffsetLeft(testCase)
    s = lka_default_scene(); s.Offset = 0.5;
    sensor = checkScene(testCase, s);
    testCase.verifyLessThan(sensor.Left.LateralOffset, 1.75);     % boundaries move right
    testCase.verifyLessThan(sensor.Right.LateralOffset, -1.75);
end

function testOffsetRight(testCase), s = lka_default_scene(); s.Offset = -0.5; checkScene(testCase, s); end
function testYawLeft(testCase),     s = lka_default_scene(); s.Yaw = deg2rad(3);  checkScene(testCase, s); end
function testYawRight(testCase),    s = lka_default_scene(); s.Yaw = deg2rad(-3); checkScene(testCase, s); end
function testCurveLeft(testCase),   s = lka_default_scene(); s.Curvature = 1/127.5;  checkScene(testCase, s); end
function testCurveRight(testCase),  s = lka_default_scene(); s.Curvature = -1/127.5; checkScene(testCase, s); end
function testTightCurve(testCase),  s = lka_default_scene(); s.Curvature = 1/50; checkScene(testCase, s); end
function testDashedLeft(testCase),  s = lka_default_scene(); s.Dashed = [true false]; checkScene(testCase, s); end

function testDistractorIgnored(testCase)
    s = lka_default_scene(); s.Distractor = 1.1;
    checkScene(testCase, s);
end

function testMissingRight(testCase)
    s = lka_default_scene(); s.Missing = [false true];
    [sensor, gt] = runFrame(testCase, s);
    testCase.verifyEqual(sensor.Right.Strength, 0);
    testCase.verifyTrue(sensor.FrameValid);
    testCase.verifyEqual(sensor.Left.LateralOffset, gt.Left.LateralOffset, 'AbsTol', 0.05);
end

function testBothMissing(testCase)
    s = lka_default_scene(); s.Missing = [true true];
    sensor = runFrame(testCase, s);
    testCase.verifyFalse(sensor.FrameValid);
end

function testSkeletonCameraPitchZero(testCase)
    C = testCase.TestData.V.Camera;
    C.X = 2.6; C.Height = 1.1; C.PitchDeg = 0;
    cam = lka_camera_model(C, testCase.TestData.V.Bev);
    s = lka_default_scene(); s.Curvature = 1/127.5;
    [~, prob, gt] = lka_render_lane_scene(cam, s);
    sensor = lka_vision_lane_step(double(transformImage(cam.Bev, prob)), cam, testCase.TestData.V, ...
        lka_vision_state_init(testCase.TestData.V), 0.05);
    testCase.verifyEqual(sensor.Left.LateralOffset, gt.Left.LateralOffset, 'AbsTol', 0.08);
    testCase.verifyEqual(sensor.Left.Curvature, gt.Left.Curvature, 'AbsTol', 2.5e-3);
end

function testSequenceTracksOffsetRamp(testCase)
    V = testCase.TestData.V; cam = testCase.TestData.cam;
    state = lka_vision_state_init(V);
    for off = linspace(0, 0.4, 9)
        s = lka_default_scene(); s.Offset = off;
        [~, prob, gt] = lka_render_lane_scene(cam, s);
        [sensor, state] = lka_vision_lane_step(double(transformImage(cam.Bev, prob)), cam, V, state, 0.05);
        testCase.verifyEqual(sensor.Left.LateralOffset, gt.Left.LateralOffset, 'AbsTol', 0.05);
        testCase.verifyEqual(sensor.Right.LateralOffset, gt.Right.LateralOffset, 'AbsTol', 0.05);
    end
    testCase.verifyEqual(state.Width, 3.5, 'AbsTol', 0.05);
end

function testHoldExpires(testCase)
    V = testCase.TestData.V; cam = testCase.TestData.cam;
    [~, prob] = lka_render_lane_scene(cam, lka_default_scene());
    [~, state] = lka_vision_lane_step(double(transformImage(cam.Bev, prob)), cam, V, lka_vision_state_init(V), 0.05);
    empty = zeros(numel(cam.RowX), numel(cam.ColY));
    for k = 1:21
        [~, state] = lka_vision_lane_step(empty, cam, V, state, 0.05);
    end
    testCase.verifyFalse(state.Left.Valid);
    testCase.verifyFalse(state.Right.Valid);
end

function testBusFieldOrder(testCase)
    sensor = runFrame(testCase, lka_default_scene());
    bus = evalin('base', 'LaneBoundaryBus');
    testCase.verifyEqual(fieldnames(sensor.Left)', {bus.Elements.Name});
    sbus = evalin('base', 'LaneSensorBus');
    testCase.verifyEqual(fieldnames(sensor)', {sbus.Elements.Name});
    testCase.verifyClass(sensor.FrameValid, 'logical');
    testCase.verifyClass(sensor.Left.LateralOffset, 'double');
end

% ------------------------------------------------------------------ helpers
function [sensor, gt] = runFrame(testCase, scene)
    V = testCase.TestData.V; cam = testCase.TestData.cam;
    [~, prob, gt] = lka_render_lane_scene(cam, scene);
    sensor = lka_vision_lane_step(double(transformImage(cam.Bev, prob)), cam, V, lka_vision_state_init(V), 0.05);
end

function sensor = checkScene(testCase, scene)
    [sensor, gt] = runFrame(testCase, scene);
    testCase.verifyTrue(sensor.FrameValid);
    for side = ["Left", "Right"]
        b = sensor.(side); g = gt.(side);
        testCase.verifyGreaterThan(b.Strength, 0.3, side);
        testCase.verifyEqual(b.LateralOffset, g.LateralOffset, 'AbsTol', 0.05, side);
        testCase.verifyEqual(b.HeadingAngle, g.HeadingAngle, 'AbsTol', deg2rad(0.4), side);
        testCase.verifyEqual(b.Curvature, g.Curvature, 'AbsTol', 1.5e-3, side);
    end
end
```

- [ ] **Step 2: Run it, expect FAIL**

Run: `runtests('tests/unit_tests/test_vision_lane_step.m')`
Expected: FAIL with "Undefined function 'lka_vision_lane_step'".

- [ ] **Step 3: Create `src/vision/lka_vision_state_init.m`**

```matlab
function state = lka_vision_state_init(V)
% LKA_VISION_STATE_INIT Frame-to-frame state of the camera lane detector.
%   Width   lane-width estimate [m]
%   Left / Right: Valid (a direct fit is available as prior), Age [s] since that fit,
%                 Coeffs [c0 c1 c2 c3] of y(x) = c0 + c1 x + c2 x^2 + c3 x^3
    side = struct('Valid', false, 'Age', inf, 'Coeffs', zeros(1, 4));
    state = struct('Width', V.Track.WidthInit, 'Left', side, 'Right', side);
end
```

- [ ] **Step 4: Create `src/vision/lka_vision_lane_step.m`**

```matlab
function [sensor, state, info] = lka_vision_lane_step(bevProb, cam, V, state, dt)
% LKA_VISION_LANE_STEP One camera frame: bird's-eye probability map -> LaneSensorBus.
%
% Syntax:
%   [sensor, state, info] = lka_vision_lane_step(bevProb, cam, V, state, dt)
%
% Per side: sliding-window search around the previous DIRECT fit (warm, +-WarmHalfWidth,
% at most Track.HoldTime old) or around the nominal lane +-width/2 (cold), robust fit,
% plausibility (heading, curvature, side of the vehicle). With both sides found, the
% pair width at the first BEV row must match the width estimate (the weaker side is
% dropped otherwise) and updates it. Only direct fits become the next prior, which
% fixes ADAS_2026 finding M1 (a reconstructed side could lock out the real line).
% One-line operation, coasting and take-over stay in LKA_Controller (lane tracker,
% supervisor), so a missing side is reported with Strength 0.
%
% Inputs:
%   bevProb - transformImage(cam.Bev, probabilityMap)
%   cam     - lka_camera_model
%   V       - LKA.Vision
%   state   - lka_vision_state_init / previous call
%   dt      - frame period [s]
% Outputs:
%   sensor  - LaneSensorBus struct (Left, Right, FrameValid)
%   info    - [nPtsLeft nPtsRight foundLeft foundRight widthEstimate]

    bevProb = double(bevProb);
    names = {'Left', 'Right'};
    sides = [1, -1];
    W = state.Width;
    found = false(1, 2);
    bnd = cell(1, 2);
    nPts = zeros(1, 2);
    for k = 1:2
        prev = state.(names{k});
        prior.Side = sides(k);
        if prev.Valid && prev.Age <= V.Track.HoldTime
            cf = prev.Coeffs;
            prior.Fun = @(x) cf(1) + cf(2) * x + cf(3) * x.^2 + cf(4) * x.^3;
            prior.HalfWidth = V.Search.WarmHalfWidth;
            prior.Cold = false;
        else
            prior.Fun = @(x) sides(k) * W / 2 + 0 * x;
            prior.HalfWidth = V.Search.ColdHalfWidth;
            prior.Cold = true;
        end
        pts = lka_search_boundary(bevProb, cam, prior, V.Search);
        nPts(k) = size(pts, 1);
        [b, ok] = lka_fit_boundary(pts, V.Fit);
        found(k) = ok && abs(b.HeadingAngle) <= V.Track.HeadingMax ...
            && abs(b.Curvature) <= V.Track.CurvatureMax && sides(k) * b.LateralOffset > 0;
        bnd{k} = b;
    end

    if all(found)
        wPair = yAt(bnd{1}, cam.XNear) - yAt(bnd{2}, cam.XNear);
        if wPair < V.Track.WidthMin || wPair > V.Track.WidthMax || abs(wPair - W) > V.Track.WidthTol
            [~, weak] = min([bnd{1}.Strength, bnd{2}.Strength]);
            found(weak) = false;
        else
            state.Width = W + min(1, dt / V.Track.WidthTau) * (wPair - W);
        end
    end

    empty = struct('LateralOffset', 0, 'HeadingAngle', 0, 'Curvature', 0, ...
        'CurvatureDerivative', 0, 'Strength', 0, 'ViewRange', 0);
    sensor = struct('Left', empty, 'Right', empty, 'FrameValid', any(found));
    for k = 1:2
        if found(k)
            b = bnd{k};
            sensor.(names{k}) = b;
            state.(names{k}).Valid = true;
            state.(names{k}).Age = 0;
            state.(names{k}).Coeffs = [b.LateralOffset, tan(b.HeadingAngle), b.Curvature / 2, b.CurvatureDerivative / 6];
        else
            state.(names{k}).Age = state.(names{k}).Age + dt;
            if state.(names{k}).Age > V.Track.HoldTime
                state.(names{k}).Valid = false;
            end
        end
    end
    info = [nPts, double(found), state.Width];
end

function y = yAt(b, x)
    y = b.LateralOffset + tan(b.HeadingAngle) * x + b.Curvature / 2 * x.^2 + b.CurvatureDerivative / 6 * x.^3;
end
```

- [ ] **Step 5: Run the test, expect PASS**

Run: `runtests('tests/unit_tests/test_vision_lane_step.m')`
Expected: 16 passed. If a curvature case misses its tolerance, check the fit degree first (span vs `CubicSpan`). Do not widen the tolerance without recording the measured error in the commit message.

- [ ] **Step 6: Commit**

```bash
git add src/vision/lka_vision_state_init.m src/vision/lka_vision_lane_step.m tests/unit_tests/test_vision_lane_step.m
git commit -m "feat(vision): per-frame lane detection step producing LaneSensorBus

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

- [ ] **Step 7: Code review checkpoint** (`requesting-code-review` on Tasks 3–6).

---

### Task 7: Switchable detectors (YOLOP / team CNN via ONNX, classic reference)

**User steps first:**
1. In MATLAB: Home → Add-Ons → search **"Deep Learning Toolbox Converter for ONNX Model Format"** → Install (MathWorks login). Confirm with `exist('importNetworkFromONNX','file')` = 2.
2. Team: export UFLDv2 to ONNX in Colab (steps in `models/perception/README.md`, "Exporting the team UFLDv2 model") and copy it to `models/perception/team_cnn.onnx`.

The classic detector, the UFLDv2 decoding tests and every non-ONNX test work without either step; the ONNX tests skip themselves until the files and add-on exist.

**Files:**
- Create: `src/vision/lka_letterbox.m`, `src/vision/lka_onnx_output_hwc.m`, `src/vision/lka_import_onnx_detector.m`, `src/vision/lka_detector_onnx_seg.m`, `src/vision/lka_detector_classic.m`, `src/vision/lka_detector_create.m`
- Create (UFLDv2): `src/vision/lka_ufld_outputs.m`, `src/vision/lka_ufld_decode.m`, `src/vision/lka_rasterize_lanes.m`, `src/vision/lka_detector_ufld.m`
- Create: `tools/onnx_reference.py`
- Test: `tests/unit_tests/test_vision_detectors.m`

**Interfaces:**
- Consumes: sidecars (Task 1), `LKA.Vision` (Task 3).
- Produces:
  - `det = lka_detector_create(V)` with `det.Name` (char) and `det.Detect` (@(rgb uint8 H×W×3) → single H×W probability in [0, 1]). It dispatches on the sidecar `type`: `seg` → `lka_detector_onnx_seg`, `ufld_v2` → `lka_detector_ufld`.
  - `[out, pad, scale] = lka_letterbox(img, inputSize, padValue)` with `pad = [top left nh nw]`.
  - `Y = lka_onnx_output_hwc(Y, inputSize)`; `net = lka_import_onnx_detector(onnxFile)`.
  - `o = lka_ufld_outputs(outs, cfg)` → `o.LocRow [Gr×Cr×L]`, `o.LocCol [Gc×Cc×L]`, `o.ExistRow [2×Cr×L]`, `o.ExistCol [2×Cc×L]`.
  - `lanes = lka_ufld_decode(o, cfg, imgSize)` → cell of `[u v p k]` (pixel u, v; existence probability p; anchor index k).
  - `prob = lka_rasterize_lanes(lanes, imgSize, widthPx)`.
  - `tools/onnx_reference.py <model> <in.bin> <out_prefix>` writes `<out_prefix>_<k>.bin` for every output and `<out_prefix>.json` with the shapes.

- [ ] **Step 1: Write the failing test** `tests/unit_tests/test_vision_detectors.m`

```matlab
% ==============================================================================
% test_vision_detectors.m
% Lane detectors: letterbox, classic detector, factory errors, YOLOP ONNX parity.
% ==============================================================================

function tests = test_vision_detectors
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.TestData.root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(testCase.TestData.root, 'config', 'startup.m'));
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    testCase.TestData.V = LKA.Vision;
    testCase.TestData.cam = lka_camera_model(LKA.Vision.Camera, LKA.Vision.Bev);
end

function testLetterbox720pTo384x640(testCase)
    img = uint8(randi(255, 720, 1280, 3));
    [out, pad, scale] = lka_letterbox(img, [384 640], 114);
    testCase.verifySize(out, [384 640 3]);
    testCase.verifyEqual(pad, [12 0 360 640]);
    testCase.verifyEqual(scale, 0.5);
    testCase.verifyTrue(all(out(1:12, :, :) == 114, 'all'));
    testCase.verifyTrue(all(out(373:384, :, :) == 114, 'all'));
end

function testClassicDetectorOnSyntheticFrame(testCase)
    V = testCase.TestData.V; V.Detector = 'classic'; cam = testCase.TestData.cam;
    det = lka_detector_create(V);
    [img, ~, gt] = lka_render_lane_scene(cam, lka_default_scene());
    prob = det.Detect(img);
    uv = round(vehicleToImage(cam.Sensor, [10 1.75; 10 0]));
    testCase.verifyGreaterThan(prob(uv(1, 2), uv(1, 1)), 0.8);
    testCase.verifyLessThan(prob(uv(2, 2), uv(2, 1)), 0.1);
    sensor = lka_vision_lane_step(double(transformImage(cam.Bev, prob)), cam, V, lka_vision_state_init(V), 0.05);
    testCase.verifyEqual(sensor.Left.LateralOffset, gt.Left.LateralOffset, 'AbsTol', 0.08);
    testCase.verifyEqual(sensor.Right.LateralOffset, gt.Right.LateralOffset, 'AbsTol', 0.08);
end

function testUnknownDetectorErrors(testCase)
    V = testCase.TestData.V; V.Detector = 'magic';
    testCase.verifyError(@() lka_detector_create(V), 'lka:vision:unknownDetector');
end

function testMissingCnnModelErrors(testCase)
    V = testCase.TestData.V; V.Detector = 'cnn';
    testCase.assumeFalse(isfile(fullfile(V.DetectorDir, 'team_cnn.onnx')), 'team CNN already delivered');
    testCase.verifyError(@() lka_detector_create(V), 'lka:vision:modelMissing');
end

function testOutputReorder(testCase)
    Y = rand(1, 2, 384, 640);
    Z = lka_onnx_output_hwc(Y, [384 640]);
    testCase.verifySize(Z, [384 640 2]);
    testCase.verifyEqual(Z(5, 7, 2), Y(1, 2, 5, 7));
end

function testYolopImportMatchesOnnxRuntime(testCase)
    testCase.assumeEqual(exist('importNetworkFromONNX', 'file'), 2, 'ONNX importer add-on not installed');
    onnx = fullfile(testCase.TestData.root, 'models', 'perception', 'yolop_ll_384x640.onnx');
    rng(7);
    x = single(0.5 * randn(384, 640, 3));
    prefix = onnxReference(testCase, onnx, x);
    yRef = lka_onnx_output_hwc(readOnnxOutput(prefix, 1), [384 640]);
    net = lka_import_onnx_detector(onnx);
    y = lka_onnx_output_hwc(extractdata(predict(net, dlarray(x, 'SSCB'))), [384 640]);
    testCase.verifyLessThan(max(abs(y(:) - yRef(:))), 1e-3);
end

% ------------------------------------------------------------------ UFLDv2 (team CNN)
function testUfldDecodeSynthetic(testCase)
    cfg = jsondecode(fileread(fullfile(testCase.TestData.V.DetectorDir, 'cnn.json')));
    o = emptyUfld(cfg);
    o.ExistRow(:, :, 2) = repmat([0; 1], 1, cfg.num_cls_row);   % lane 2 exists at every row anchor
    o.LocRow(41, :, 2) = 10;                                    % arg-max cell 41 (0-based 40)
    lanes = lka_ufld_decode(o, cfg, [720 1280]);
    testCase.verifyNumElements(lanes, 1);
    P = lanes{1};
    testCase.verifySize(P, [cfg.num_cls_row 4]);
    e = exp([0 10 0] - 10);
    uExp = (sum(e .* [39 40 41]) / sum(e) + 0.5) / (cfg.num_grid_row - 1) * 1280 + 0.5;
    testCase.verifyEqual(P(:, 1), repmat(uExp, cfg.num_cls_row, 1), 'AbsTol', 1e-9);
    testCase.verifyEqual(P(1, 2), 160.5, 'AbsTol', 1e-9);
    testCase.verifyEqual(P(end, 2), 710.5, 'AbsTol', 1e-9);
    prob = lka_rasterize_lanes(lanes, [720 1280], cfg.line_width_px);
    testCase.verifyGreaterThan(prob(400, round(uExp)), 0.5);
    testCase.verifyEqual(prob(400, 100), single(0));
end

function testUfldShortLaneRejected(testCase)
    cfg = jsondecode(fileread(fullfile(testCase.TestData.V.DetectorDir, 'cnn.json')));
    o = emptyUfld(cfg);
    o.ExistRow(:, 1:20, 2) = repmat([0; 1], 1, 20);             % only 20 of 56 anchors (< half)
    testCase.verifyEmpty(lka_ufld_decode(o, cfg, [720 1280]));
end

function testUfldOutputsIdentifiedByShape(testCase)
    cfg = jsondecode(fileread(fullfile(testCase.TestData.V.DetectorDir, 'cnn.json')));
    outs = {rand(2, 41, 4, 'single'), rand(56, 4, 100, 'single'), ...     % mixed order and layouts
            rand(1, 100, 41, 4, 'single'), rand(1, 2, 56, 4, 'single')};
    o = lka_ufld_outputs(outs, cfg);
    testCase.verifySize(o.LocRow, [100 56 4]);
    testCase.verifySize(o.LocCol, [100 41 4]);
    testCase.verifySize(o.ExistRow, [2 56 4]);
    testCase.verifySize(o.ExistCol, [2 41 4]);
    testCase.verifyEqual(o.LocRow(7, 3, 2), outs{2}(3, 2, 7));
end

function testUfldImportMatchesOnnxRuntime(testCase)
    testCase.assumeEqual(exist('importNetworkFromONNX', 'file'), 2, 'ONNX importer add-on not installed');
    d = testCase.TestData.V.DetectorDir;
    onnx = fullfile(d, 'team_cnn.onnx');
    testCase.assumeTrue(isfile(onnx), 'team UFLDv2 ONNX not delivered yet');
    cfg = jsondecode(fileread(fullfile(d, 'cnn.json')));
    rng(8);
    x = single(0.5 * randn(320, 800, 3));
    prefix = onnxReference(testCase, onnx, x);
    refs = cell(1, 4);
    for k = 1:4
        refs{k} = readOnnxOutput(prefix, k);
    end
    oRef = lka_ufld_outputs(refs, cfg);
    net = lka_import_onnx_detector(onnx);
    outs = cell(1, numel(net.OutputNames));
    [outs{:}] = predict(net, dlarray(x, 'SSCB'));
    o = lka_ufld_outputs(cellfun(@extractdata, outs, 'UniformOutput', false), cfg);
    for f = ["LocRow", "LocCol", "ExistRow", "ExistCol"]
        testCase.verifyLessThan(max(abs(o.(f)(:) - oRef.(f)(:))), 1e-3, f);
    end
end

function testUfldDetectorRuns(testCase)
    testCase.assumeEqual(exist('importNetworkFromONNX', 'file'), 2, 'ONNX importer add-on not installed');
    V = testCase.TestData.V; V.Detector = 'cnn';
    testCase.assumeTrue(isfile(fullfile(V.DetectorDir, 'team_cnn.onnx')), 'team UFLDv2 ONNX not delivered yet');
    det = lka_detector_create(V);
    img = lka_render_lane_scene(testCase.TestData.cam, lka_default_scene());
    det.Detect(img);
    t0 = tic; prob = det.Detect(img); ms = 1000 * toc(t0);
    testCase.verifySize(prob, [720 1280]);
    fprintf('[vision] UFLDv2 %.1f ms/frame (GPU %d)\n', ms, V.UseGpu && canUseGPU());
end

% ------------------------------------------------------------------ helpers
function o = emptyUfld(cfg)
    L = cfg.num_lanes;
    o.LocRow = zeros(cfg.num_grid_row, cfg.num_cls_row, L);
    o.LocCol = zeros(cfg.num_grid_col, cfg.num_cls_col, L);
    o.ExistRow = repmat([1; 0], 1, cfg.num_cls_row, L);         % channel 1 wins: not present
    o.ExistCol = repmat([1; 0], 1, cfg.num_cls_col, L);
end

function prefix = onnxReference(testCase, onnx, x)
    root = testCase.TestData.root;
    inBin = [tempname '.bin'];
    prefix = tempname;
    fid = fopen(inBin, 'w'); fwrite(fid, permute(x, [2 1 3]), 'single'); fclose(fid);   % NCHW, C order
    [st, msg] = system(sprintf('python "%s" "%s" "%s" "%s"', ...
        fullfile(root, 'tools', 'onnx_reference.py'), onnx, inBin, prefix));
    testCase.assumeEqual(st, 0, ['onnxruntime reference unavailable: ' msg]);
end

function Y = readOnnxOutput(prefix, k)
    shapes = jsondecode(fileread([prefix '.json']));
    if iscell(shapes), s = shapes{k}(:)'; else, s = shapes(k, :); end
    fid = fopen(sprintf('%s_%d.bin', prefix, k), 'r');
    y = fread(fid, inf, 'single=>single');
    fclose(fid);
    Y = permute(reshape(y, fliplr(s)), numel(s):-1:1);     % C order -> ONNX dimension order
end

function testYolopDetectorRuns(testCase)
    testCase.assumeEqual(exist('importNetworkFromONNX', 'file'), 2, 'ONNX importer add-on not installed');
    V = testCase.TestData.V; V.Detector = 'yolop';
    det = lka_detector_create(V);
    img = lka_render_lane_scene(testCase.TestData.cam, lka_default_scene());
    det.Detect(img);                                   % warm-up (import cache, GPU)
    t0 = tic; prob = det.Detect(img); ms = 1000 * toc(t0);
    testCase.verifySize(prob, [720 1280]);
    testCase.verifyGreaterThanOrEqual(min(prob(:)), 0);
    testCase.verifyLessThanOrEqual(max(prob(:)), 1);
    fprintf('[vision] YOLOP %.1f ms/frame (GPU %d)\n', ms, V.UseGpu && canUseGPU());
end
```

- [ ] **Step 2: Run it, expect FAIL**

Run: `runtests('tests/unit_tests/test_vision_detectors.m')`
Expected: FAIL with "Undefined function 'lka_letterbox'" (the ONNX tests are skipped by their assumption if the add-on is missing).

- [ ] **Step 3: Create `src/vision/lka_letterbox.m`**

```matlab
function [out, pad, scale] = lka_letterbox(img, inputSize, padValue)
% LKA_LETTERBOX Aspect-preserving resize into inputSize with centred constant padding.
%
% Syntax:
%   [out, pad, scale] = lka_letterbox(img, inputSize, padValue)
%
% Same geometry as YOLOP letterbox_for_img(new_shape=640, auto=True) for 1280x720:
% scale 0.5 -> 640x360, 12 rows of padding top and bottom -> 384x640. Box filter
% resize (cv2.INTER_AREA equivalent for integer down-scaling).
% Output pad = [top left nh nw]: the image occupies out(top+1:top+nh, left+1:left+nw, :).

    [h, w, ~] = size(img);
    scale = min(inputSize(1) / h, inputSize(2) / w);
    nh = round(h * scale);
    nw = round(w * scale);
    rs = imresize(img, [nh nw], 'box');
    top = floor((inputSize(1) - nh) / 2);
    left = floor((inputSize(2) - nw) / 2);
    out = repmat(cast(padValue, 'like', img), [inputSize, size(img, 3)]);
    out(top + 1:top + nh, left + 1:left + nw, :) = rs;
    pad = [top, left, nh, nw];
end
```

- [ ] **Step 4: Create `src/vision/lka_onnx_output_hwc.m`**

```matlab
function Y = lka_onnx_output_hwc(Y, inputSize)
% LKA_ONNX_OUTPUT_HWC Reorders a segmentation network output to H x W x C.
%
% Imported ONNX networks may return SSCB, BCSS or unformatted layouts; the H and W
% dimensions are found by size (non-square inputs only).
    Y = squeeze(Y);
    sz = size(Y);
    hDim = find(sz == inputSize(1), 1);
    wDim = find(sz == inputSize(2), 1);
    if isempty(hDim) || isempty(wDim) || hDim == wDim
        error('lka:vision:outputShape', 'Network output %s does not match input %dx%d.', ...
            mat2str(sz), inputSize(1), inputSize(2));
    end
    if numel(sz) == 2
        Y = permute(Y, [hDim wDim]);
    else
        Y = permute(Y, [hDim wDim setdiff(1:3, [hDim wDim])]);
    end
end
```

- [ ] **Step 5: Create `src/vision/lka_import_onnx_detector.m`**

```matlab
function net = lka_import_onnx_detector(onnxFile)
% LKA_IMPORT_ONNX_DETECTOR Imports an ONNX segmentation network once and caches it.
%
% The dlnetwork is cached next to the model as <name>_dlnetwork.mat (git-ignored),
% custom layers generated by the importer go to the namespace +lka_onnx_<name>
% in the same folder (models/perception is on the path).
    [d, n] = fileparts(onnxFile);
    cacheFile = fullfile(d, [n '_dlnetwork.mat']);
    if isfile(cacheFile)
        a = dir(cacheFile); b = dir(onnxFile);
        if a.datenum >= b.datenum
            s = load(cacheFile, 'net');
            net = s.net;
            return
        end
    end
    if exist('importNetworkFromONNX', 'file') ~= 2
        error('lka:vision:onnxImporterMissing', ['Install the add-on "Deep Learning Toolbox ' ...
            'Converter for ONNX Model Format" to run ONNX lane detectors.']);
    end
    old = cd(d);
    restore = onCleanup(@() cd(old));
    net = importNetworkFromONNX(onnxFile, 'InputDataFormats', 'BCSS', ...
        'NamespaceName', matlab.lang.makeValidName(['lka_onnx_' n]));
    if ~net.Initialized
        net = initialize(net);
    end
    save(cacheFile, 'net');
end
```

- [ ] **Step 6: Create `src/vision/lka_detector_onnx_seg.m`**

```matlab
function det = lka_detector_onnx_seg(sidecarFile, useGpu)
% LKA_DETECTOR_ONNX_SEG Lane detector from an ONNX segmentation model + JSON sidecar.
%
% Syntax:
%   det = lka_detector_onnx_seg(sidecarFile, useGpu)
%   prob = det.Detect(rgb)        rgb uint8 H x W x 3 -> single H x W lane probability
%
% Contract and sidecar fields: models/perception/README.md. YOLOP and the team CNN
% both run through this function.

    cfg = jsondecode(fileread(sidecarFile));
    onnxFile = fullfile(fileparts(sidecarFile), cfg.onnx);
    if ~isfile(onnxFile)
        error('lka:vision:modelMissing', ['Lane model "%s" not found. Export it to ONNX as ' ...
            'described in models/perception/README.md.'], onnxFile);
    end
    net = lka_import_onnx_detector(onnxFile);
    inSize = cfg.input_size(:)';
    mean3 = reshape(single(cfg.mean), 1, 1, []);
    std3 = reshape(single(cfg.std), 1, 1, []);
    useGpu = useGpu && canUseGPU();
    det.Name = cfg.name;
    det.Detect = @detect;

    function prob = detect(rgb)
        im = rgb;
        if strcmpi(cfg.channel_order, 'BGR')
            im = im(:, :, [3 2 1]);
        end
        [h, w, ~] = size(im);
        if strcmpi(cfg.resize, 'letterbox')
            [x, pad] = lka_letterbox(im, inSize, cfg.pad_value);
        else
            x = imresize(im, inSize, 'bilinear');
            pad = [0 0 inSize];
        end
        x = (single(x) / single(cfg.scale) - mean3) ./ std3;
        X = dlarray(x, 'SSCB');
        if useGpu
            X = gpuArray(X);
        end
        Y = lka_onnx_output_hwc(gather(extractdata(predict(net, X))), inSize);
        switch lower(cfg.output)
            case 'softmax'
                e = exp(Y - max(Y, [], 3));
                p = e(:, :, cfg.lane_channel) ./ sum(e, 3);
            case 'sigmoid'
                p = 1 ./ (1 + exp(-Y(:, :, cfg.lane_channel)));
            otherwise
                p = Y(:, :, cfg.lane_channel);
        end
        p = p(pad(1) + 1:pad(1) + pad(3), pad(2) + 1:pad(2) + pad(4));
        prob = single(min(max(imresize(p, [h w], 'bilinear'), 0), 1));
    end
end
```

- [ ] **Step 7: Create `src/vision/lka_detector_classic.m`**

```matlab
function det = lka_detector_classic(~)
% LKA_DETECTOR_CLASSIC Reference lane detector: bright, thin, white/yellow paint.
%
% p = logistic((V - boxmean41(V) - 0.12) / 0.03) on pixels that are white (S < 0.25)
% or yellow (hue 0.10-0.19, S > 0.35); V, S, hue from rgb2hsv. No learning, used as
% the fallback while the ONNX add-on or the team CNN are unavailable and as the
% benchmark baseline.
    det.Name = 'classic (white/yellow local contrast)';
    det.Detect = @detect;

    function prob = detect(rgb)
        hsv = rgb2hsv(rgb);
        val = hsv(:, :, 3);
        sat = hsv(:, :, 2);
        hue = hsv(:, :, 1);
        contrast = val - imboxfilt(val, 41);
        paint = sat < 0.25 | (hue > 0.10 & hue < 0.19 & sat > 0.35);
        prob = single(1 ./ (1 + exp(-(contrast .* paint - 0.12) / 0.03)));
    end
end
```

- [ ] **Step 8: Create `src/vision/lka_detector_create.m`**

```matlab
function det = lka_detector_create(V)
% LKA_DETECTOR_CREATE Lane detector selected by V.Detector ('yolop' | 'cnn' | 'classic').
%
% Syntax:
%   det = lka_detector_create(V)          V = LKA.Vision
%   prob = det.Detect(rgb)
%
% 'yolop' and 'cnn' read models/perception/<name>.json; its "type" picks the adapter:
% 'seg' (pixel mask, lka_detector_onnx_seg) or 'ufld_v2' (anchors, lka_detector_ufld).
    switch lower(V.Detector)
        case {'yolop', 'cnn'}
            sidecar = fullfile(V.DetectorDir, [lower(V.Detector) '.json']);
            cfg = jsondecode(fileread(sidecar));
            switch cfg.type
                case 'seg'
                    det = lka_detector_onnx_seg(sidecar, V.UseGpu);
                case 'ufld_v2'
                    det = lka_detector_ufld(sidecar, V.UseGpu);
                otherwise
                    error('lka:vision:unknownModelType', 'Unknown model type "%s" in %s.', cfg.type, sidecar);
            end
        case 'classic'
            det = lka_detector_classic(V);
        otherwise
            error('lka:vision:unknownDetector', 'Unknown lane detector "%s" (yolop | cnn | classic).', V.Detector);
    end
end
```

- [ ] **Step 8b: Create `src/vision/lka_ufld_outputs.m`**

```matlab
function o = lka_ufld_outputs(outs, cfg)
% LKA_UFLD_OUTPUTS Identifies the four UFLDv2 heads by size and orders them [grid x anchor x lane].
%
% Syntax:
%   o = lka_ufld_outputs(outs, cfg)       outs = cell of network outputs (any order / layout)
%
% Output fields: LocRow [num_grid_row x num_cls_row x num_lanes], LocCol [num_grid_col x
% num_cls_col x num_lanes], ExistRow [2 x num_cls_row x num_lanes], ExistCol [2 x num_cls_col
% x num_lanes]. The ONNX output names of deploy/pt2onnx.py are not relied on.
    want = struct('LocRow', [cfg.num_grid_row cfg.num_cls_row], 'LocCol', [cfg.num_grid_col cfg.num_cls_col], ...
        'ExistRow', [2 cfg.num_cls_row], 'ExistCol', [2 cfg.num_cls_col]);
    names = fieldnames(want);
    o = struct();
    for k = 1:numel(outs)
        y = squeeze(outs{k});
        for j = 1:numel(names)
            if isfield(o, names{j})
                continue
            end
            p = matchPermutation(size(y), [want.(names{j}) cfg.num_lanes]);
            if ~isempty(p)
                o.(names{j}) = permute(y, p);
                break
            end
        end
    end
    if numel(fieldnames(o)) ~= 4
        error('lka:vision:outputShape', 'UFLDv2 outputs not recognised (sizes %s).', ...
            strjoin(cellfun(@(y) mat2str(size(y)), outs, 'UniformOutput', false), ', '));
    end
end

function p = matchPermutation(sz, target)
    p = [];
    if numel(sz) ~= 3
        return
    end
    P = perms(1:3);
    for i = 1:size(P, 1)
        if isequal(sz(P(i, :)), target)
            p = P(i, :);
            return
        end
    end
end
```

- [ ] **Step 8c: Create `src/vision/lka_ufld_decode.m`**

```matlab
function lanes = lka_ufld_decode(o, cfg, imgSize)
% LKA_UFLD_DECODE UFLDv2 anchor outputs -> lane points in image pixels.
%
% Syntax:
%   lanes = lka_ufld_decode(o, cfg, imgSize)          o from lka_ufld_outputs
%
% Port of pred2coords (cfzd/Ultra-Fast-Lane-Detection-v2). An anchor exists when
% exist channel 2 beats channel 1. A row lane (cfg.row_lanes) is used when more than half
% of its row anchors exist, a column lane (cfg.col_lanes) when more than a quarter of its
% column anchors exist. Position = expectation of the softmax over +-local_width cells
% around the arg-max cell, + 0.5, scaled by (num_grid - 1) to the image size.
% Output: cell array, one [u v p k] matrix per lane: MATLAB pixel coordinates (u, v),
% existence probability p, anchor index k.
    H = imgSize(1);
    W = imgSize(2);
    rowAnchor = linspace(cfg.row_anchor_px(1), cfg.row_anchor_px(2), cfg.num_cls_row) / cfg.anchor_image_height;
    colAnchor = linspace(0, 1, cfg.num_cls_col);
    lanes = {};
    for lane = reshape(cfg.row_lanes, 1, [])
        q = decodeLane(o.LocRow(:, :, lane), o.ExistRow(:, :, lane), cfg.num_cls_row / 2, W, cfg.local_width);
        if ~isempty(q)
            lanes{end + 1} = [q(:, 1) + 0.5, reshape(rowAnchor(q(:, 3)), [], 1) * H + 0.5, q(:, 2:3)]; %#ok<AGROW>
        end
    end
    for lane = reshape(cfg.col_lanes, 1, [])
        q = decodeLane(o.LocCol(:, :, lane), o.ExistCol(:, :, lane), cfg.num_cls_col / 4, H, cfg.local_width);
        if ~isempty(q)
            lanes{end + 1} = [reshape(colAnchor(q(:, 3)), [], 1) * W + 0.5, q(:, 1) + 0.5, q(:, 2:3)]; %#ok<AGROW>
        end
    end
end

function q = decodeLane(loc, ex, minCount, extent, localWidth)
% q = [position (0-based pixels), existence probability, anchor index]
    G = size(loc, 1);
    valid = ex(2, :) > ex(1, :);
    q = zeros(0, 3);
    if nnz(valid) <= minCount
        return
    end
    pEx = 1 ./ (1 + exp(ex(1, :) - ex(2, :)));
    for k = find(valid)
        [~, m] = max(loc(:, k));
        idx = (max(1, m - localWidth):min(G, m + localWidth))';
        e = exp(loc(idx, k) - max(loc(idx, k)));
        cell0 = sum(e .* (idx - 1)) / sum(e) + 0.5;
        q(end + 1, :) = [cell0 / (G - 1) * extent, pEx(k), k]; %#ok<AGROW>
    end
end
```

- [ ] **Step 8d: Create `src/vision/lka_rasterize_lanes.m`**

```matlab
function prob = lka_rasterize_lanes(lanes, imgSize, widthPx)
% LKA_RASTERIZE_LANES Lane point lists -> lane-line probability map.
%
% Syntax:
%   prob = lka_rasterize_lanes(lanes, imgSize, widthPx)     lanes from lka_ufld_decode
%
% Consecutive points of a lane (anchor gap <= 3) are joined by a widthPx line whose value
% is the smaller existence probability of its two ends. One insertShape call per frame.
    segs = zeros(0, 4);
    vals = zeros(0, 1);
    for k = 1:numel(lanes)
        P = lanes{k};
        if size(P, 1) < 2
            continue
        end
        join = diff(P(:, 4)) <= 3;
        a = P(1:end - 1, :);
        b = P(2:end, :);
        segs = [segs; a(join, 1:2), b(join, 1:2)]; %#ok<AGROW>
        vals = [vals; min(a(join, 3), b(join, 3))]; %#ok<AGROW>
    end
    canvas = zeros([imgSize 3], 'uint8');
    if ~isempty(segs)
        c = repmat(round(255 * vals), 1, 3);
        canvas = insertShape(canvas, 'Line', segs, 'Color', c, 'LineWidth', widthPx, 'SmoothEdges', false);
    end
    prob = single(canvas(:, :, 1)) / 255;
end
```

- [ ] **Step 8e: Create `src/vision/lka_detector_ufld.m`**

```matlab
function det = lka_detector_ufld(sidecarFile, useGpu)
% LKA_DETECTOR_UFLD Team CNN (UFLDv2) lane detector -> lane-line probability map.
%
% Syntax:
%   det = lka_detector_ufld(sidecarFile, useGpu)
%   prob = det.Detect(rgb)        rgb uint8 H x W x 3 -> single H x W
%
% Preprocessing as the UFLDv2 test transform: resize to (input_size(1)/crop_ratio) x
% input_size(2), keep the bottom input_size(1) rows, scale and ImageNet-normalise
% (RGB). The four heads are decoded (lka_ufld_decode) and rasterised
% (lka_rasterize_lanes), so the team CNN uses the same bird's-eye / fit / tracker chain as
% YOLOP.
    cfg = jsondecode(fileread(sidecarFile));
    onnxFile = fullfile(fileparts(sidecarFile), cfg.onnx);
    if ~isfile(onnxFile)
        error('lka:vision:modelMissing', ['Lane model "%s" not found. Export it to ONNX as ' ...
            'described in models/perception/README.md.'], onnxFile);
    end
    net = lka_import_onnx_detector(onnxFile);
    inSize = cfg.input_size(:)';
    fullH = floor(inSize(1) / cfg.crop_ratio + 1e-9);
    mean3 = reshape(single(cfg.mean), 1, 1, []);
    std3 = reshape(single(cfg.std), 1, 1, []);
    useGpu = useGpu && canUseGPU();
    nOut = numel(net.OutputNames);
    det.Name = cfg.name;
    det.Detect = @detect;

    function prob = detect(rgb)
        im = rgb;
        if strcmpi(cfg.channel_order, 'BGR')
            im = im(:, :, [3 2 1]);
        end
        [h, w, ~] = size(im);
        x = imresize(im, [fullH inSize(2)], 'bilinear');
        x = x(end - inSize(1) + 1:end, :, :);
        x = (single(x) / single(cfg.scale) - mean3) ./ std3;
        X = dlarray(x, 'SSCB');
        if useGpu
            X = gpuArray(X);
        end
        outs = cell(1, nOut);
        [outs{:}] = predict(net, X);
        o = lka_ufld_outputs(cellfun(@(y) gather(extractdata(y)), outs, 'UniformOutput', false), cfg);
        prob = lka_rasterize_lanes(lka_ufld_decode(o, cfg, [h w]), [h w], cfg.line_width_px);
    end
end
```

- [ ] **Step 9: Create `tools/onnx_reference.py`**

```python
"""Reference outputs of an ONNX model with onnxruntime (CPU) for the MATLAB parity tests.

Usage: python tools/onnx_reference.py <model.onnx> <input.bin> <out_prefix>
Input: raw little-endian float32 in C (NCHW) order, shape = model input.
Output: <out_prefix>_<k>.bin (k = 1..number of outputs, same encoding) and
<out_prefix>.json with the list of output shapes.
"""
import json
import sys

import numpy as np
import onnxruntime as ort


def main(onnx_path, in_bin, out_prefix):
    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0]
    shape = [d if isinstance(d, int) else 1 for d in inp.shape]
    x = np.fromfile(in_bin, dtype="<f4").reshape(shape)
    shapes = []
    for k, y in enumerate(sess.run(None, {inp.name: x}), start=1):
        y.astype("<f4").tofile(f"{out_prefix}_{k}.bin")
        shapes.append(list(y.shape))
    with open(out_prefix + ".json", "w") as f:
        json.dump(shapes, f)
    print("ok", shapes)


if __name__ == "__main__":
    main(*sys.argv[1:4])
```

- [ ] **Step 10: Run the test, expect PASS**

Run: `runtests('tests/unit_tests/test_vision_detectors.m')`
Expected: 12 passed. With the add-on missing: 8 passed + 4 filtered. With the add-on but no `team_cnn.onnx` yet: 10 passed + 2 filtered.

If `importNetworkFromONNX` fails or a parity check exceeds 1e-3, stop and report the importer message to the user. Fallback (U3): a detector that calls onnxruntime through `py.onnxruntime`; it needs `pyenv` with Python ≤ 3.12, which is a decision for the user.

Once `team_cnn.onnx` exists, also look at one decoded frame: `imshow(labeloverlay(img, det.Detect(img) > 0.5))`. The four UFLDv2 lanes must lie on painted lines, not on the image edges. If they are mirrored or shifted, the row/column-lane assignment (`row_lanes`, `col_lanes`) or the crop is wrong; fix the sidecar, not the code.

- [ ] **Step 11: Commit**

```bash
git add src/vision/lka_letterbox.m src/vision/lka_onnx_output_hwc.m src/vision/lka_import_onnx_detector.m src/vision/lka_detector_onnx_seg.m src/vision/lka_detector_classic.m src/vision/lka_detector_create.m src/vision/lka_ufld_outputs.m src/vision/lka_ufld_decode.m src/vision/lka_rasterize_lanes.m src/vision/lka_detector_ufld.m tools/onnx_reference.py tests/unit_tests/test_vision_detectors.m
git commit -m "feat(vision): switchable YOLOP / team UFLDv2 / classic lane detectors

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 8: System objects (Simulink detector block, annotated video)

**Files:**
- Create: `src/vision/LkaVisionLaneDetector.m`, `src/vision/LkaVisionRecorder.m`, `src/vision/lka_annotate_frame.m`
- Test: `tests/unit_tests/test_vision_system_objects.m`

**Interfaces:**
- Consumes: Tasks 3–7.
- Produces: System object `LkaVisionLaneDetector` (properties `Vision`, `Ts`; input image uint8 720×1280×3; outputs 1 `LaneSensorBus`, 2 preview single `V.PreviewSize`, 3 info 1×7 `[nPtsL nPtsR foundL foundR width msDetector msTotal]`). System object `LkaVisionRecorder` (properties `Vision`, `Ts`, `FileName`; inputs image, preview, LaneSensorBus, steer deg, LKA state; no outputs). `frame = lka_annotate_frame(img, preview, sensor, hud, cam)` with `hud.SteerDeg`, `hud.State`.

- [ ] **Step 1: Write the failing test** `tests/unit_tests/test_vision_system_objects.m`

```matlab
% ==============================================================================
% test_vision_system_objects.m
% LkaVisionLaneDetector / LkaVisionRecorder / lka_annotate_frame outside Simulink.
% ==============================================================================

function tests = test_vision_system_objects
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    V = LKA.Vision; V.Detector = 'classic'; V.UseGpu = false;
    testCase.TestData.V = V;
    testCase.TestData.cam = lka_camera_model(V.Camera, V.Bev);
end

function testDetectorObjectStep(testCase)
    V = testCase.TestData.V;
    det = LkaVisionLaneDetector('Vision', V, 'Ts', 0.05);
    [img, ~, gt] = lka_render_lane_scene(testCase.TestData.cam, lka_default_scene());
    [sensor, preview, info] = det(img);
    testCase.verifyEqual(sensor.Left.LateralOffset, gt.Left.LateralOffset, 'AbsTol', 0.08);
    testCase.verifySize(preview, V.PreviewSize);
    testCase.verifyClass(preview, 'single');
    testCase.verifySize(info, [1 7]);
    testCase.verifyGreaterThan(info(7), 0);
    release(det);
end

function testDetectorResetClearsState(testCase)
    det = LkaVisionLaneDetector('Vision', testCase.TestData.V, 'Ts', 0.05);
    img = lka_render_lane_scene(testCase.TestData.cam, lka_default_scene());
    det(img);
    reset(det);
    [~, ~, info] = det(uint8(70 * ones(720, 1280, 3)));
    testCase.verifyEqual(info(3:4), [0 0]);
    release(det);
end

function testAnnotateDrawsLeftBoundaryGreen(testCase)
    cam = testCase.TestData.cam;
    [img, prob, gt] = lka_render_lane_scene(cam, lka_default_scene());
    b = gt.Left; b.Strength = 1; b.ViewRange = 30;
    r = gt.Right; r.Strength = 1; r.ViewRange = 30;
    sensor = struct('Left', b, 'Right', r, 'FrameValid', true);
    preview = imresize(transformImage(cam.Bev, prob), testCase.TestData.V.PreviewSize);
    frame = lka_annotate_frame(img, preview, sensor, struct('SteerDeg', 1.5, 'State', 2), cam);
    testCase.verifySize(frame, size(img));
    uv = round(vehicleToImage(cam.Sensor, [12 gt.Left.LateralOffset + tan(gt.Left.HeadingAngle) * 12]));
    px = squeeze(frame(uv(2), uv(1), :));
    testCase.verifyGreaterThan(double(px(2)), 200);
    testCase.verifyLessThan(double(px(1)), 80);
end

function testRecorderWritesVideo(testCase)
    V = testCase.TestData.V; cam = testCase.TestData.cam;
    f = [tempname '.mp4'];
    rec = LkaVisionRecorder('Vision', V, 'Ts', 0.05, 'FileName', f);
    det = LkaVisionLaneDetector('Vision', V, 'Ts', 0.05);
    img = lka_render_lane_scene(cam, lka_default_scene());
    for k = 1:3
        [sensor, preview] = det(img);
        rec(img, preview, sensor, 0.5, uint8(2));
    end
    release(rec); release(det);
    testCase.verifyTrue(isfile(f));
    vr = VideoReader(f);
    testCase.verifyEqual(vr.NumFrames, 3);
end
```

- [ ] **Step 2: Run it, expect FAIL**

Run: `runtests('tests/unit_tests/test_vision_system_objects.m')`
Expected: FAIL with "Unrecognized function or variable 'LkaVisionLaneDetector'".

- [ ] **Step 3: Create `src/vision/LkaVisionLaneDetector.m`**

```matlab
classdef LkaVisionLaneDetector < matlab.System
% LKAVISIONLANEDETECTOR Camera image -> LaneSensorBus (MATLAB System block, interpreted).
%
% Detector (LKA.Vision.Detector: YOLOP | team CNN | classic) -> lane probability map ->
% metric bird's-eye view (lka_camera_model) -> lka_vision_lane_step.
% Outputs: 1 LaneSensorBus, 2 bird's-eye probability preview (single, PreviewSize),
%          3 [nPtsL nPtsR foundL foundR width msDetector msTotal].

    properties (Nontunable)
        Vision = struct()     % LKA.Vision
        Ts = 0.05             % frame period [s]
    end

    properties (Access = private)
        Cam
        Det
        State
    end

    methods
        function obj = LkaVisionLaneDetector(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.Cam = lka_camera_model(obj.Vision.Camera, obj.Vision.Bev);
            obj.Det = lka_detector_create(obj.Vision);
            obj.State = lka_vision_state_init(obj.Vision);
        end

        function resetImpl(obj)
            obj.State = lka_vision_state_init(obj.Vision);
        end

        function [sensor, preview, info] = stepImpl(obj, img)
            t0 = tic;
            prob = obj.Det.Detect(img);
            msDet = 1000 * toc(t0);
            bev = transformImage(obj.Cam.Bev, prob);
            [sensor, obj.State, info0] = lka_vision_lane_step(bev, obj.Cam, obj.Vision, obj.State, obj.Ts);
            preview = single(imresize(bev, obj.Vision.PreviewSize, 'bilinear'));
            info = [info0, msDet, 1000 * toc(t0)];
        end

        function n = getNumInputsImpl(~)
            n = 1;
        end

        function n = getNumOutputsImpl(~)
            n = 3;
        end

        function [s1, s2, s3] = getOutputSizeImpl(obj)
            s1 = [1 1];
            s2 = obj.Vision.PreviewSize;
            s3 = [1 7];
        end

        function [d1, d2, d3] = getOutputDataTypeImpl(~)
            d1 = 'LaneSensorBus';
            d2 = 'single';
            d3 = 'double';
        end

        function [c1, c2, c3] = isOutputComplexImpl(~)
            c1 = false; c2 = false; c3 = false;
        end

        function [f1, f2, f3] = isOutputFixedSizeImpl(~)
            f1 = true; f2 = true; f3 = true;
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts);
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
```

- [ ] **Step 4: Create `src/vision/lka_annotate_frame.m`**

```matlab
function frame = lka_annotate_frame(img, preview, sensor, hud, cam)
% LKA_ANNOTATE_FRAME Camera frame with detected boundaries, bird's-eye inset and HUD.
%
% Syntax:
%   frame = lka_annotate_frame(img, preview, sensor, hud, cam)
%
% Left boundary green, right boundary blue (only with Strength > 0), bird's-eye
% probability preview (parula, x3) top-right, one HUD line: boundary offsets at the rear
% axle, steering command [deg, + left], LKA state (0 OFF ... 4 TAKEOVER).

    frame = img;
    xs = linspace(cam.XNear, cam.XFar, 40)';
    sides = {'Left', 'Right'};
    colors = {[0 255 0], [0 160 255]};
    for k = 1:2
        b = sensor.(sides{k});
        if b.Strength <= 0
            continue
        end
        y = b.LateralOffset + tan(b.HeadingAngle) * xs + b.Curvature / 2 * xs.^2 + b.CurvatureDerivative / 6 * xs.^3;
        uv = vehicleToImage(cam.Sensor, [xs y]);
        in = all(isfinite(uv), 2) & uv(:, 1) >= 1 & uv(:, 1) <= size(img, 2) & uv(:, 2) >= 1 & uv(:, 2) <= size(img, 1);
        if nnz(in) >= 2
            frame = insertShape(frame, 'Line', reshape(uv(in, :)', 1, []), 'Color', colors{k}, 'LineWidth', 5);
        end
    end

    pv = im2uint8(ind2rgb(im2uint8(mat2gray(preview, [0 1])), parula(256)));
    pv = imresize(pv, 3, 'nearest');
    [ph, pw, ~] = size(pv);
    frame(11:10 + ph, end - pw - 9:end - 10, :) = pv;

    txt = sprintf('L %+.2f m  R %+.2f m  |  steer %+.1f deg  |  LKA state %d', ...
        sensor.Left.LateralOffset * (sensor.Left.Strength > 0), ...
        sensor.Right.LateralOffset * (sensor.Right.Strength > 0), hud.SteerDeg, round(hud.State));
    frame = insertText(frame, [10 10], txt, 'FontSize', 22, 'BoxColor', 'black', ...
        'BoxOpacity', 0.6, 'TextColor', 'white');
end
```

- [ ] **Step 5: Create `src/vision/LkaVisionRecorder.m`**

```matlab
classdef LkaVisionRecorder < matlab.System
% LKAVISIONRECORDER Writes the annotated LKA camera video (lka_annotate_frame).
%
% Inputs: camera image, bird's-eye preview, LaneSensorBus, steering command [deg, + left],
% LKA state (LkaInfoBus.State). Empty FileName disables the recorder.

    properties (Nontunable)
        Vision = struct()     % LKA.Vision
        Ts = 0.05             % frame period [s]
        FileName = ''         % .mp4 path
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
            if isempty(obj.FileName)
                return
            end
            folder = fileparts(obj.FileName);
            if ~isempty(folder) && ~isfolder(folder)
                mkdir(folder);
            end
            obj.Cam = lka_camera_model(obj.Vision.Camera, obj.Vision.Bev);
            obj.Writer = VideoWriter(obj.FileName, 'MPEG-4');
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
```

- [ ] **Step 6: Run the test, expect PASS**

Run: `runtests('tests/unit_tests/test_vision_system_objects.m')`
Expected: 4 passed.

- [ ] **Step 7: Commit**

```bash
git add src/vision/LkaVisionLaneDetector.m src/vision/LkaVisionRecorder.m src/vision/lka_annotate_frame.m tests/unit_tests/test_vision_system_objects.m
git commit -m "feat(vision): Simulink lane-detector and annotated-video System objects

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

- [ ] **Step 8: Code review checkpoint** (`requesting-code-review` on Tasks 7–8).

---

### Task 9: Rule-compliant ego-motion sensor model

**Files:**
- Create: `src/sensors/lka_ego_sensor_model.m`, `scripts/lka_add_ego_sensors.m`
- Modify: `src/utilities/lka_build_params.m` (append next to the Vision block)
- Test: `tests/unit_tests/test_ego_sensors.m`

**Interfaces:**
- Produces: `[vMeas, rMeas] = lka_ego_sensor_model(v, r, n, P)` (`n` = `[nSpeed nGyro]` standard normal, `P = LKA.EgoSen`); `path = lka_add_ego_sensors(sys, name, pos)` builds a subsystem with inputs `Speed`, `YawRate` and outputs `SpeedMeas`, `YawRateMeas` (used by Tasks 10 and 13).

- [ ] **Step 1: Write the failing test** `tests/unit_tests/test_ego_sensors.m`

```matlab
% ==============================================================================
% test_ego_sensors.m
% Wheel-speed and gyro models that feed LKA_Controller (rule I.2.3).
% ==============================================================================

function tests = test_ego_sensors
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    testCase.TestData.P = LKA.EgoSen;
end

function testNoiseFreeOutputs(testCase)
    P = testCase.TestData.P;
    [v, r] = lka_ego_sensor_model(8.0, 0.1, [0 0], P);
    testCase.verifyEqual(v, round(8.0 * (1 + P.SpeedScaleErr) / P.SpeedQuant) * P.SpeedQuant, 'AbsTol', 1e-12);
    testCase.verifyEqual(r, 0.1 + P.YawRateBias, 'AbsTol', 1e-12);
end

function testSpeedNeverNegative(testCase)
    v = lka_ego_sensor_model(0.0, 0, [-5 0], testCase.TestData.P);
    testCase.verifyEqual(v, 0);
end

function testNoiseScales(testCase)
    P = testCase.TestData.P;
    [~, r1] = lka_ego_sensor_model(5, 0, [0 1], P);
    testCase.verifyEqual(r1 - P.YawRateBias, P.YawRateNoiseStd, 'AbsTol', 1e-12);
end
```

- [ ] **Step 2: Run it, expect FAIL**

Run: `runtests('tests/unit_tests/test_ego_sensors.m')`
Expected: FAIL in setupOnce, "Unrecognized field name 'EgoSen'".

- [ ] **Step 3: Append to `src/utilities/lka_build_params.m`** (after the Vision block of Task 3)

```matlab

    % ---------------------------------------- Ego-motion sensors (rule I.2.3: realistic sensors)
    % Assumed automotive-grade values: wheel-speed scale error from tyre radius, MEMS gyro
    % residual bias after start-up calibration and white noise per 20 Hz sample.
    EgoSen.SpeedScaleErr   = 0.01;           % [-]
    EgoSen.SpeedNoiseStd   = 0.05;           % [m/s]
    EgoSen.SpeedQuant      = 0.01;           % [m/s]
    EgoSen.YawRateBias     = deg2rad(0.2);   % [rad/s]
    EgoSen.YawRateNoiseStd = deg2rad(0.3);   % [rad/s]
    EgoSen.Seed            = [2101 2102];
    LKA.EgoSen = EgoSen;
```

- [ ] **Step 4: Create `src/sensors/lka_ego_sensor_model.m`**

```matlab
function [vMeas, rMeas] = lka_ego_sensor_model(v, r, n, P)
% LKA_EGO_SENSOR_MODEL Wheel-speed sensor and gyro yaw rate as seen by the LKA.
%
% Syntax:
%   [vMeas, rMeas] = lka_ego_sensor_model(v, r, n, P)       P = LKA.EgoSen
%
% vMeas = quantised((1 + SpeedScaleErr)*v + SpeedNoiseStd*n(1)), >= 0      [m/s]
% rMeas = r + YawRateBias + YawRateNoiseStd*n(2)                          [rad/s, + CCW]
% n = [nSpeed nGyro] standard normal samples (Random Number blocks, seeded).
    vMeas = (1 + P.SpeedScaleErr) * v + P.SpeedNoiseStd * n(1);
    vMeas = max(0, round(vMeas / P.SpeedQuant) * P.SpeedQuant);
    rMeas = r + P.YawRateBias + P.YawRateNoiseStd * n(2);
end
```

- [ ] **Step 5: Create `scripts/lka_add_ego_sensors.m`**

```matlab
function path = lka_add_ego_sensors(sys, name, pos)
% LKA_ADD_EGO_SENSORS Adds the wheel-speed + gyro sensor-model subsystem to a model.
%
% Syntax:
%   path = lka_add_ego_sensors(sys, name, pos)
%
% Inputs Speed [m/s], YawRate [rad/s] (sample time LKA.Ts) -> outputs SpeedMeas, YawRateMeas
% through lka_ego_sensor_model with seeded Random Number blocks (LKA.EgoSen.Seed).
    path = [sys '/' name];
    add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', pos);
    delete_line(path, 'In1/1', 'Out1/1');
    delete_block([path '/In1']);
    delete_block([path '/Out1']);
    add_block('simulink/Sources/In1', [path '/Speed'], 'Port', '1', 'Position', [20 40 50 54]);
    add_block('simulink/Sources/In1', [path '/YawRate'], 'Port', '2', 'Position', [20 110 50 124]);
    add_block('simulink/Sources/Random Number', [path '/Speed Noise'], 'Mean', '0', 'Variance', '1', ...
        'Seed', 'LKA.EgoSen.Seed(1)', 'SampleTime', 'LKA.Ts', 'Position', [20 180 60 210]);
    add_block('simulink/Sources/Random Number', [path '/Gyro Noise'], 'Mean', '0', 'Variance', '1', ...
        'Seed', 'LKA.EgoSen.Seed(2)', 'SampleTime', 'LKA.Ts', 'Position', [20 240 60 270]);
    add_block('simulink/Signal Routing/Mux', [path '/Noise'], 'Inputs', '2', 'Position', [110 180 115 270]);
    mf = [path '/Sensor Model'];
    add_block('simulink/User-Defined Functions/MATLAB Function', mf, 'Position', [200 40 360 270]);
    ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', mf);
    ch.Script = strjoin({
        'function [vMeas, rMeas] = fcn(v, r, n, LKA)'
        '% Wheel-speed sensor and gyro with scale error, bias and noise (lka_ego_sensor_model.m)'
        '[vMeas, rMeas] = lka_ego_sensor_model(v, r, n, LKA.EgoSen);'}, newline);
    d = ch.find('-isa', 'Stateflow.Data', 'Name', 'LKA');
    d.Scope = 'Parameter';
    d.Tunable = false;
    add_block('simulink/Sinks/Out1', [path '/SpeedMeas'], 'Port', '1', 'Position', [440 60 470 74]);
    add_block('simulink/Sinks/Out1', [path '/YawRateMeas'], 'Port', '2', 'Position', [440 200 470 214]);
    add_line(path, 'Speed/1', 'Sensor Model/1');
    add_line(path, 'YawRate/1', 'Sensor Model/2');
    add_line(path, 'Speed Noise/1', 'Noise/1');
    add_line(path, 'Gyro Noise/1', 'Noise/2');
    add_line(path, 'Noise/1', 'Sensor Model/3');
    add_line(path, 'Sensor Model/1', 'SpeedMeas/1');
    add_line(path, 'Sensor Model/2', 'YawRateMeas/1');
end
```

- [ ] **Step 6: Run the tests, expect PASS**

Run: `runtests('tests/unit_tests/test_ego_sensors.m')` (expected: 3 passed), then `runtests('tests/unit_tests/test_lka_core.m')` (expected: 19 passed).

- [ ] **Step 7: Commit**

```bash
git add src/sensors scripts/lka_add_ego_sensors.m src/utilities/lka_build_params.m tests/unit_tests/test_ego_sensors.m
git commit -m "feat(sensors): wheel-speed and gyro models for rule-compliant controller inputs

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 10: Camera-in-the-loop test bench (Unreal, official LKA track)

**Files:**
- Create: `src/vision/lka_pose_to_sim3d.m`, `scripts/lka_add_vision_detector.m`, `scripts/build_lka_vision_bench.m`
- Create (generated): `models/LKA_Vision_TestBench.slx`
- Test: `tests/unit_tests/test_vision_bench_build.m`

**Interfaces:**
- Consumes: `models/LKA_MIL_TestBench.slx` (block names from `build_lka_models.m:748-926`), Tasks 3–9.
- Produces: `[X, Y, yawDeg] = lka_pose_to_sim3d(pose, frameMap)`; `path = lka_add_vision_detector(sys, name, pos)` (subsystem: in `Image`; out 1 `LaneSensor` delayed by `LKA.Vision.LatencySteps`, out 2 `Preview`, out 3 `VisionInfo`); model `LKA_Vision_TestBench` with blocks `LKA Camera` (out1 RGB, out2 semantic labels), `Vision Lane Detector`, `Ego Vehicle (Unreal)`, `Pose To Unreal`, `Semantic Sink`, `Vision Recorder`, `Ego Sensors (wheel speed + IMU)`; logged signals `lane_sensor`, `vision_info`, `speed_meas`, `yaw_rate_meas` plus all MIL bench logs.

- [ ] **Step 1: Print the Simulation 3D library mask parameter names** (U12)

Run in MATLAB:

```matlab
load_system('drivingsim3d');
for b = {'Simulation 3D Scene Configuration', 'Simulation 3D Camera', 'Simulation 3D Vehicle with Ground Following'}
    fprintf('== %s\n', b{1}); disp(get_param(['drivingsim3d/' b{1}], 'MaskNames')');
end
close_system('drivingsim3d', 0);
```

Expected to include: Scene: `ProjectFormat`, `RoadRunnerProjectPath`, `SampleTime`; Camera: `vehTag`, `mountLoc`, `offsetFlag`, `tmountOffset`, `rmountOffset`, `FocalLength`, `OpticalCenter`, `ImageSize`, `SemanticOutportEnabled`, `SampleTime`; Vehicle: `PassVehMesh`, `ActorName`, `SampleTime`. If a name differs, use the printed name in Step 6 and note the difference in the commit message.

- [ ] **Step 2: Write the failing test** `tests/unit_tests/test_vision_bench_build.m`

```matlab
% ==============================================================================
% test_vision_bench_build.m
% Pose mapping to Unreal and structure/compilation of LKA_Vision_TestBench.
% ==============================================================================

function tests = test_vision_bench_build
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.TestData.root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(testCase.TestData.root, 'config', 'startup.m'));
end

function testPoseMapSkeletonConvention(testCase)
    [X, Y, yaw] = lka_pose_to_sim3d([10 5 deg2rad(30)], 3);
    testCase.verifyEqual([X Y yaw], [10 -5 -30], 'AbsTol', 1e-12);
end

function testPoseMapAllCandidatesDiffer(testCase)
    p = [10 5 deg2rad(30)];
    out = zeros(4, 3);
    for k = 1:4
        [out(k, 1), out(k, 2), out(k, 3)] = lka_pose_to_sim3d(p, k);
    end
    testCase.verifyEqual(size(unique(out, 'rows'), 1), 4);
end

function testBenchBuildsAndCompiles(testCase)
    build_lka_vision_bench();
    mdl = 'LKA_Vision_TestBench';
    testCase.addTeardown(@() close_system(mdl, 0));
    for b = {'LKA Camera', 'Vision Lane Detector', 'Ego Vehicle (Unreal)', 'Pose To Unreal', ...
             'Simulation 3D Scene Configuration', 'Vision Recorder', 'Ego Sensors (wheel speed + IMU)'}
        testCase.verifyGreaterThan(getSimulinkBlockHandle([mdl '/' b{1}]), 0, b{1});
    end
    testCase.verifyEqual(getSimulinkBlockHandle([mdl '/Virtual Lane Sensor']), -1);
    LKA = evalin('base', 'LKA'); LKA.Vision.Detector = 'classic'; assignin('base', 'LKA', LKA);
    vehicle = evalin('base', 'vehicle');
    sc = generate_lka_scenarios(vehicle); sc = sc(1); r = sc.Road;
    assignin('base', 'SCN', sc.P);
    assignin('base', 'ROAD', struct('s', r.s, 'x', r.x, 'y', r.y, 'psi', r.psi, 'kappa', r.kappa, ...
        'dkappa', r.dkappa, 'width', r.width, 'lineWidth', r.lineWidth));
    set_param(mdl, 'SimulationCommand', 'update');          % throws on any error
end
```

- [ ] **Step 3: Run it, expect FAIL**

Run: `runtests('tests/unit_tests/test_vision_bench_build.m')`
Expected: FAIL with "Undefined function 'lka_pose_to_sim3d'".

- [ ] **Step 4: Create `src/vision/lka_pose_to_sim3d.m`**

```matlab
function [X, Y, yawDeg] = lka_pose_to_sim3d(pose, frameMap)
% LKA_POSE_TO_SIM3D Track pose (OpenDRIVE / RoadRunner frame) -> Simulation 3D inputs.
%
% Syntax:
%   [X, Y, yawDeg] = lka_pose_to_sim3d(pose, frameMap)     pose = [x y psi] (m, m, rad)
%
% frameMap 3 is the convention of the aBaja2026.slx InitFcn (RoadRunner -> Unreal:
% initialPosition = pos.*[1 -1 1], yaw negated); the other maps are the candidates
% tested by calibrate_vision_bench.
    x = pose(1); y = pose(2); psiDeg = rad2deg(pose(3));
    switch frameMap
        case 1
            X = -y; Y = -x; yawDeg = -90 - psiDeg;
        case 2
            X = x;  Y = y;  yawDeg = psiDeg;
        case 3
            X = x;  Y = -y; yawDeg = -psiDeg;
        otherwise
            X = y;  Y = x;  yawDeg = 90 - psiDeg;
    end
end
```

- [ ] **Step 5: Create `scripts/lka_add_vision_detector.m`**

```matlab
function path = lka_add_vision_detector(sys, name, pos)
% LKA_ADD_VISION_DETECTOR Adds the camera lane-detector subsystem (Image -> LaneSensorBus).
%
% Syntax:
%   path = lka_add_vision_detector(sys, name, pos)
%
% Contents: MATLAB System LkaVisionLaneDetector (Vision = LKA.Vision, Ts = LKA.Ts) and a
% Delay of LKA.Vision.LatencySteps frames (processing latency, compensated in the lane
% tracker by LKA.KF.LatencyComp).
% Outputs: 1 LaneSensor (delayed), 2 Preview (bird's-eye probability), 3 VisionInfo.
    path = [sys '/' name];
    add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', pos);
    delete_line(path, 'In1/1', 'Out1/1');
    delete_block([path '/In1']);
    delete_block([path '/Out1']);
    add_block('simulink/Sources/In1', [path '/Image'], 'Port', '1', 'Position', [20 100 50 114]);
    det = [path '/Lane Detector'];
    add_block('simulink/User-Defined Functions/MATLAB System', det, 'System', 'LkaVisionLaneDetector', ...
        'Position', [120 70 320 190]);
    set_param(det, 'Vision', 'LKA.Vision', 'Ts', 'LKA.Ts');
    add_block('simulink/Discrete/Delay', [path '/Processing Latency'], 'DelayLength', 'LKA.Vision.LatencySteps', ...
        'InitialCondition', '0', 'SampleTime', 'LKA.Ts', 'Position', [400 80 440 110]);
    outs = {'LaneSensor', 'Preview', 'VisionInfo'};
    for k = 1:3
        add_block('simulink/Sinks/Out1', [path '/' outs{k}], 'Port', num2str(k), ...
            'Position', [520 60 + 50 * k 550 74 + 50 * k]);
    end
    set_param([path '/LaneSensor'], 'OutDataTypeStr', 'Bus: LaneSensorBus');
    add_line(path, 'Image/1', 'Lane Detector/1');
    add_line(path, 'Lane Detector/1', 'Processing Latency/1');
    add_line(path, 'Processing Latency/1', 'LaneSensor/1');
    add_line(path, 'Lane Detector/2', 'Preview/1');
    add_line(path, 'Lane Detector/3', 'VisionInfo/1');
end
```

- [ ] **Step 6: Create `scripts/build_lka_vision_bench.m`**

```matlab
function build_lka_vision_bench()
% BUILD_LKA_VISION_BENCH Camera-in-the-loop LKA test bench on the official LKA track.
%
% Syntax:
%   build_lka_vision_bench()
%
% Starts from LKA_MIL_TestBench (same scenarios, Baja plant, ground-truth metrics) and
% replaces the emulated lane sensor with the camera chain:
%   rear-axle pose -> Simulation 3D Vehicle with Ground Following (Unreal, official LKA
%   track exported from RoadRunner as USD: data/scenes/LKA/LKA.usd)
%   -> Simulation 3D Camera (RGB + semantic labels) -> Vision Lane Detector
%   (LKA.Vision.Detector: YOLOP | team CNN | classic) -> processing latency -> LKA_Controller.
% Speed and yaw rate reach the controller through the wheel-speed / gyro sensor model.
% Road geometry and semantic labels are used only for metrics and datasets (rule I.2.3).
% Output: models/LKA_Vision_TestBench.slx

    root = fileparts(fileparts(mfilename('fullpath')));
    run(fullfile(root, 'config', 'startup.m'));
    evalin('base', 'lkaProfile = ''baja''; lka_controller_params; create_lka_buses;');
    modelDir = fullfile(root, 'models');
    if ~isfile(fullfile(modelDir, 'LKA_MIL_TestBench.slx'))
        build_lka_models();
    end
    mdl = 'LKA_Vision_TestBench';
    for m = {mdl, 'LKA_MIL_TestBench'}
        if bdIsLoaded(m{1}), close_system(m{1}, 0); end
    end
    load_system(fullfile(modelDir, 'LKA_MIL_TestBench.slx'));
    save_system('LKA_MIL_TestBench', fullfile(modelDir, [mdl '.slx']));   % save as
    set_param(mdl, 'Description', ['Camera-in-the-loop LKA bench: Baja single-track plant, Unreal ' ...
        'official LKA track, Simulation 3D Camera, switchable lane detector (YOLOP / team CNN / ' ...
        'classic), wheel-speed and gyro models, LKA_Controller, ground-truth metrics.']);

    % ---------------- remove the emulated lane sensor
    removeBlock([mdl '/Virtual Lane Sensor']);
    terminate(mdl, 'Lane Geometry (ground truth)', 1:3);
    terminate(mdl, 'Scenario', 2:3);

    % ---------------- Unreal scene, ego actor, camera
    scene = [mdl '/Simulation 3D Scene Configuration'];
    add_block('drivingsim3d/Simulation 3D Scene Configuration', scene, 'Position', [40 760 260 840]);
    set_param(scene, 'ProjectFormat', 'RoadRunner', ...
        'RoadRunnerProjectPath', evalin('base', 'LKA.Vision.Bench.ScenePath'), 'SampleTime', 'LKA.Ts');

    add_block('simulink/Discrete/Zero-Order Hold', [mdl '/Unreal Rate'], 'SampleTime', 'LKA.Ts', ...
        'Position', [1250 700 1290 730]);
    addMlfb(mdl, 'Pose To Unreal', strjoin({
        'function [X, Y, yawDeg] = fcn(rearPose, LKA)'
        '% Track pose (OpenDRIVE / RoadRunner frame) -> Simulation 3D world inputs'
        '[X, Y, yawDeg] = lka_pose_to_sim3d(rearPose, LKA.Vision.Bench.FrameMap);'}, newline), ...
        [1330 690 1480 770]);
    ego = [mdl '/Ego Vehicle (Unreal)'];
    add_block('drivingsim3d/Simulation 3D Vehicle with Ground Following', ego, 'Position', [1540 690 1700 790]);
    set_param(ego, 'PassVehMesh', 'Muscle car', 'ActorName', 'LkaEgo', 'SampleTime', 'LKA.Ts');

    cam = [mdl '/LKA Camera'];
    add_block('drivingsim3d/Simulation 3D Camera', cam, 'Position', [1540 840 1700 920]);
    set_param(cam, 'mountLoc', 'Origin', 'offsetFlag', 'on', ...
        'tmountOffset', '[LKA.Vision.Camera.X, LKA.Vision.Camera.Y, LKA.Vision.Camera.Height + LKA.Vision.Bench.MountZOffset]', ...
        'rmountOffset', '[0, LKA.Vision.Bench.PitchSign * LKA.Vision.Camera.PitchDeg, 0]', ...
        'FocalLength', 'LKA.Vision.Camera.FocalLength', 'OpticalCenter', 'LKA.Vision.Camera.PrincipalPoint', ...
        'ImageSize', 'LKA.Vision.Camera.ImageSize', 'SemanticOutportEnabled', 'on', 'SampleTime', 'LKA.Ts');
    try
        set_param(cam, 'vehTag', 'LkaEgo');
    catch ME
        warning('[LKA vision] Select parent "LkaEgo" in the LKA Camera dialog and save (%s).', ME.message);
    end

    lka_add_vision_detector(mdl, 'Vision Lane Detector', [1780 840 1960 960]);
    add_block('simulink/Sinks/Terminator', [mdl '/Semantic Sink'], 'Position', [1780 990 1800 1010]);

    ln(mdl, 'Vehicle Dynamics (Baja single-track)/2', 'Unreal Rate/1');
    ln(mdl, 'Unreal Rate/1', 'Pose To Unreal/1');
    for k = 1:3
        ln(mdl, sprintf('Pose To Unreal/%d', k), sprintf('Ego Vehicle (Unreal)/%d', k));
    end
    ln(mdl, 'LKA Camera/1', 'Vision Lane Detector/1');
    ln(mdl, 'LKA Camera/2', 'Semantic Sink/1');
    ln(mdl, 'Vision Lane Detector/1', 'LKA Controller/1');

    % ---------------- wheel-speed and gyro models in front of the controller
    delete_line(mdl, 'Speed To Controller Rate/1', 'LKA Controller/2');
    delete_line(mdl, 'Yaw Rate To Controller Rate/1', 'LKA Controller/3');
    es = 'Ego Sensors (wheel speed + IMU)';
    lka_add_ego_sensors(mdl, es, [1640 340 1740 440]);
    ln(mdl, 'Speed To Controller Rate/1', [es '/1']);
    ln(mdl, 'Yaw Rate To Controller Rate/1', [es '/2']);
    ln(mdl, [es '/1'], 'LKA Controller/2');
    ln(mdl, [es '/2'], 'LKA Controller/3');

    % ---------------- annotated video
    rec = [mdl '/Vision Recorder'];
    add_block('simulink/User-Defined Functions/MATLAB System', rec, 'System', 'LkaVisionRecorder', ...
        'Position', [2060 840 2220 960]);
    set_param(rec, 'Vision', 'LKA.Vision', 'Ts', 'LKA.Ts', 'FileName', 'LKA.Vision.Bench.VideoFile');
    add_block('simulink/Math Operations/Gain', [mdl '/Steer Deg'], 'Gain', '180/pi', 'Position', [1990 1000 2030 1030]);
    add_block('simulink/Signal Routing/Bus Selector', [mdl '/Select State'], 'OutputSignals', 'State', ...
        'Position', [1990 1060 2000 1090]);
    ln(mdl, 'LKA Camera/1', 'Vision Recorder/1');
    ln(mdl, 'Vision Lane Detector/2', 'Vision Recorder/2');
    ln(mdl, 'Vision Lane Detector/1', 'Vision Recorder/3');
    ln(mdl, 'LKA Controller/1', 'Steer Deg/1');
    ln(mdl, 'Steer Deg/1', 'Vision Recorder/4');
    ln(mdl, 'LKA Controller/3', 'Select State/1');
    ln(mdl, 'Select State/1', 'Vision Recorder/5');
    terminate(mdl, 'Vision Lane Detector', 3);

    % ---------------- logging
    logOut(mdl, 'Vision Lane Detector', 1, 'lane_sensor');
    logOut(mdl, 'Vision Lane Detector', 3, 'vision_info');
    logOut(mdl, es, 1, 'speed_meas');
    logOut(mdl, es, 2, 'yaw_rate_meas');

    save_system(mdl);
    fprintf('[LKA vision] Built %s\n', fullfile(modelDir, [mdl '.slx']));
end

function removeBlock(blk)
    lh = get_param(blk, 'LineHandles');
    h = [lh.Inport, lh.Outport];
    for k = find(h > 0)
        delete_line(h(k));
    end
    delete_block(blk);
end

function terminate(mdl, blk, ports)
    for p = ports
        t = add_block('simulink/Sinks/Terminator', sprintf('%s/%s Term %d', mdl, blk, p), 'MakeNameUnique', 'on');
        add_line(mdl, sprintf('%s/%d', blk, p), [get_param(t, 'Name') '/1'], 'autorouting', 'smart');
    end
end

function addMlfb(sys, name, code, pos)
    path = [sys '/' name];
    add_block('simulink/User-Defined Functions/MATLAB Function', path, 'Position', pos);
    ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', path);
    ch.Script = code;
    d = ch.find('-isa', 'Stateflow.Data', 'Name', 'LKA');
    d.Scope = 'Parameter';
    d.Tunable = false;
end

function ln(sys, src, dst)
    add_line(sys, src, dst, 'autorouting', 'smart');
end

function logOut(mdl, blk, port, name)
    ph = get_param([mdl '/' blk], 'PortHandles');
    lh = get_param(ph.Outport(port), 'Line');
    if lh > 0
        set_param(lh, 'Name', name);
    end
    set_param(ph.Outport(port), 'DataLogging', 'on', 'DataLoggingNameMode', 'SignalName');
end
```

- [ ] **Step 7: Run the test, expect PASS**

Run: `runtests('tests/unit_tests/test_vision_bench_build.m')`
Expected: 3 passed. A compile error that names a mask parameter → recheck Step 1. A bus error on `Vision Lane Detector/1` → check that `getOutputDataTypeImpl` returns `'LaneSensorBus'` and that `create_lka_buses` ran.

- [ ] **Step 8: Commit**

```bash
git add src/vision/lka_pose_to_sim3d.m scripts/lka_add_vision_detector.m scripts/build_lka_vision_bench.m models/LKA_Vision_TestBench.slx tests/unit_tests/test_vision_bench_build.m
git commit -m "feat(vision): camera-in-the-loop LKA test bench on the official track (Unreal)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 11: Bench calibration, first closed-loop camera runs, video

**Files:**
- Create: `scripts/calibrate_vision_bench.m`, `scripts/run_lka_vision.m`
- Modify: `src/vision/lka_vision_params.m` (bench values found by the calibration)
- Test: `tests/scenario_tests/test_lka_vision_bench.m`

**Interfaces:**
- Consumes: Task 10 model, Tasks 3–9.
- Produces: `report = calibrate_vision_bench(FrameMaps=)` (struct `FrameMap, LaneLabelId, OffsetError_m, Width_m, ImpliedHeight_m, BottomLabels`, plus `results/vision/calibration.png`); `results = run_lka_vision(Scenarios=, Detectors=, Controller=, Video=, Dataset=, OutDir=, Rebuild=)` (struct array `res`, `kpi`; CSV `results/vision/lka_vision_kpis.csv`; videos `results/vision/<scenario>_<detector>.mp4`).

- [ ] **Step 1: Create `scripts/calibrate_vision_bench.m`**

```matlab
function report = calibrate_vision_bench(options)
% CALIBRATE_VISION_BENCH Verifies the camera-bench assumptions on the official LKA track.
%
% Syntax:
%   report = calibrate_vision_bench()
%   report = calibrate_vision_bench(FrameMaps=[3 1 2 4])
%
% The ego is parked at the S1_Nominal start (centred, aligned). For each candidate
% track -> Unreal frame map one short simulation grabs the camera image and the semantic
% labels. Every label id is tried as a lane-line mask through lka_vision_lane_step; the
% (frame map, label id) whose boundaries best match the ground truth wins. Reported:
%   FrameMap, LaneLabelId    -> LKA.Vision.Bench.FrameMap / .LaneLabelId     (U5, U7)
%   ImpliedHeight_m           camera height implied by the 3.5 m lane         (U8)
%   BottomLabels              label ids in the bottom-centre image rows: anything other
%                             than road / lane marking there means the ego mesh is visible
%                             -> increase LKA.Vision.Camera.X (bench only)    (U8)
% Saves results/vision/calibration.png. Does not edit any parameter file.

    arguments
        options.FrameMaps double = [3 1 2 4]
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    run(fullfile(root, 'config', 'startup.m'));
    evalin('base', 'lkaProfile = ''baja''; lka_controller_params; create_lka_buses;');
    vehicle = evalin('base', 'vehicle');
    LKA = evalin('base', 'LKA');
    LKA.Vision.Detector = 'classic';
    sc = generate_lka_scenarios(vehicle);
    sc = sc(strcmp({sc.Name}, 'S1_Nominal'));
    P = sc.P; P.VSet = 0; P.V0 = 0; P.StopTime = 0.5;
    r = sc.Road;
    ROAD = struct('s', r.s, 'x', r.x, 'y', r.y, 'psi', r.psi, 'kappa', r.kappa, ...
        'dkappa', r.dkappa, 'width', r.width, 'lineWidth', r.lineWidth);
    assignin('base', 'SCN', P);
    assignin('base', 'ROAD', ROAD);

    % ground truth at the parked pose (rear axle)
    psi0 = P.X0(3);
    rear = [P.X0(1) - vehicle.b * cos(psi0), P.X0(2) - vehicle.b * sin(psi0), psi0];
    [cp, curv, w] = lka_road_closest_point(rear(1), rear(2), 0, r.s, r.x, r.y, r.psi, r.kappa, r.dkappa, r.width);
    [gtL, gtR] = lka_lane_geometry(rear, cp, curv(1), curv(2), w);

    mdl = 'LKA_Vision_TestBench';
    load_system(mdl);
    restore = onCleanup(@() close_system(mdl, 0));
    ph = get_param([mdl '/LKA Camera'], 'PortHandles');
    l2 = get_param(ph.Outport(2), 'Line');
    if l2 > 0, delete_line(l2); end
    add_block('simulink/Sinks/To Workspace', [mdl '/Cal Image'], 'VariableName', 'calImg', ...
        'MaxDataPoints', '1', 'SaveFormat', 'Array');
    add_block('simulink/Sinks/To Workspace', [mdl '/Cal Label'], 'VariableName', 'calLabel', ...
        'MaxDataPoints', '1', 'SaveFormat', 'Array');
    add_line(mdl, 'LKA Camera/1', 'Cal Image/1');
    add_line(mdl, 'LKA Camera/2', 'Cal Label/1');

    V = LKA.Vision;
    cam = lka_camera_model(V.Camera, V.Bev);
    best = struct('FrameMap', NaN, 'LaneLabelId', NaN, 'OffsetError_m', inf);
    for fm = options.FrameMaps
        LKA.Vision.Bench.FrameMap = fm;
        assignin('base', 'LKA', LKA);
        out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
        img = out.calImg(:, :, :, end);
        lab = out.calLabel(:, :, end);
        for id = reshape(unique(lab), 1, [])
            mask = single(lab == id);
            s = lka_vision_lane_step(transformImage(cam.Bev, mask), cam, V, lka_vision_state_init(V), LKA.Ts);
            if s.Left.Strength > 0 && s.Right.Strength > 0
                err = abs(s.Left.LateralOffset - gtL(1)) + abs(s.Right.LateralOffset - gtR(1));
                if err < best.OffsetError_m
                    best = struct('FrameMap', fm, 'LaneLabelId', double(id), 'OffsetError_m', err, ...
                        'Width_m', s.Left.LateralOffset - s.Right.LateralOffset, 'Sensor', s, ...
                        'Image', img, 'Label', lab);
                end
            end
        end
        fprintf('[calibrate] frame map %d: best offset error so far %.3f m\n', fm, best.OffsetError_m);
        if best.OffsetError_m < 0.15
            break
        end
    end
    if isnan(best.FrameMap)
        error('lka:vision:calibrationFailed', ['No frame map shows both lane lines at the S1 start. ' ...
            'Check that Unreal loads %s (U6) and inspect out.calImg.'], V.Bench.ScenePath);
    end

    report.FrameMap = best.FrameMap;
    report.LaneLabelId = best.LaneLabelId;
    report.OffsetError_m = best.OffsetError_m;
    report.Width_m = best.Width_m;
    report.ImpliedHeight_m = V.Camera.Height * (gtL(1) - gtR(1)) / best.Width_m;
    bottom = best.Label(end - 30:end, 560:720);
    report.BottomLabels = unique(bottom(:))';
    disp(report);
    fprintf(['Set in lka_vision_params.m (Bench): FrameMap = %d; LaneLabelId = %d; ' ...
        'MountZOffset = %.3f\n'], report.FrameMap, report.LaneLabelId, ...
        V.Camera.Height - report.ImpliedHeight_m);

    outDir = fullfile(root, 'results', 'vision');
    if ~isfolder(outDir), mkdir(outDir); end
    frame = lka_annotate_frame(best.Image, imresize(transformImage(cam.Bev, single(best.Label == best.LaneLabelId)), ...
        V.PreviewSize), best.Sensor, struct('SteerDeg', 0, 'State', 0), cam);
    imwrite(frame, fullfile(outDir, 'calibration.png'));
end
```

- [ ] **Step 2: Run the calibration** (Unreal starts, about 20 s per frame map)

Run: `report = calibrate_vision_bench()`
Expected: the `FrameMap 3` run already gives `OffsetError_m < 0.15`, `Width_m ≈ 3.5`, `ImpliedHeight_m ≈ 1.20`. Look at `results/vision/calibration.png`: green and blue lines must sit on the painted lines.

Decision points:
- `calibrationFailed` → the USD scene did not load (U6). Fallback: set the Scene Configuration to the built-in "Curved road" scene and use semantic-label ground truth only. Report this to the user before continuing.
- `ImpliedHeight_m` off by more than 3 % → set `MountZOffset` to the printed value.
- `BottomLabels` contains ids other than road and lane marking → increase `LKA.Vision.Camera.X` (baja profile) in 0.5 m steps, rebuild the bench, re-run.

- [ ] **Step 3: Write the calibrated values into `src/vision/lka_vision_params.m`**

Replace the `V.Bench.FrameMap`, `V.Bench.MountZOffset` and `V.Bench.LaneLabelId` lines with the printed values, each followed by the comment `% calibrate_vision_bench 2026-10-xx`. Then run `build_lka_vision_bench()` again.

- [ ] **Step 4: Write the failing scenario test** `tests/scenario_tests/test_lka_vision_bench.m`

```matlab
% ==============================================================================
% test_lka_vision_bench.m
% Closed-loop camera-in-the-loop run on the official LKA track (Unreal).
% Opt-in: set environment variable LKA_RUN_UNREAL_TESTS=1 (opens the Unreal window).
% ==============================================================================

function tests = test_lka_vision_bench
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.assumeEqual(getenv('LKA_RUN_UNREAL_TESTS'), '1', 'Unreal tests are opt-in');
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
end

function testClassicDetectorKeepsLane(testCase)
    r = run_lka_vision(Scenarios={'S1_Nominal'}, Detectors={'classic'}, Video=false);
    testCase.verifyEqual(r.kpi.InLaneRatio, 1, 'AbsTol', 1e-9);
    testCase.verifyLessThan(r.kpi.MaxLatOffset_m, 0.4);
    testCase.verifyFalse(r.kpi.SpeedDQ);
end

function testYolopDetectorKeepsLane(testCase)
    testCase.assumeEqual(exist('importNetworkFromONNX', 'file'), 2, 'ONNX importer add-on not installed');
    r = run_lka_vision(Scenarios={'S1_Nominal'}, Detectors={'yolop'}, Video=true);
    testCase.verifyEqual(r.kpi.InLaneRatio, 1, 'AbsTol', 1e-9);
    testCase.verifyLessThan(r.kpi.MaxLatOffset_m, 0.4);
end
```

- [ ] **Step 5: Run it, expect FAIL**

Run: `setenv('LKA_RUN_UNREAL_TESTS','1'); runtests('tests/scenario_tests/test_lka_vision_bench.m')`
Expected: FAIL with "Undefined function 'run_lka_vision'".

- [ ] **Step 6: Create `scripts/run_lka_vision.m`**

```matlab
function results = run_lka_vision(options)
% RUN_LKA_VISION Runs the camera-in-the-loop LKA bench (Unreal) for detectors and scenarios.
%
% Syntax:
%   results = run_lka_vision()
%   results = run_lka_vision(Scenarios={'S1_Nominal','S2_SpeedLimit'}, Detectors={'yolop','classic'})
%
% Name-value options:
%   Scenarios  - scenario names from generate_lka_scenarios (default {'S1_Nominal'})
%   Detectors  - {'yolop' | 'cnn' | 'classic'} (default {'yolop'})
%   Controller - 1 pure pursuit, 2 Stanley 2025, 3 MPC (default 1)
%   Video      - write results/vision/<scenario>_<detector>.mp4 (default true)
%   Dataset    - record frames for the detector benchmark (default false)
%   OutDir     - default results/vision
%   Rebuild    - rebuild LKA_Vision_TestBench first (default false)
% Output: struct array (res, kpi); CSV lka_vision_kpis.csv and figures in OutDir.

    arguments
        options.Scenarios cell = {'S1_Nominal'}
        options.Detectors cell = {'yolop'}
        options.Controller (1,1) double = 1
        options.Video (1,1) logical = true
        options.Dataset (1,1) logical = false
        options.OutDir char = ''
        options.Rebuild (1,1) logical = false
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    if isempty(options.OutDir)
        options.OutDir = fullfile(root, 'results', 'vision');
    end
    run(fullfile(root, 'config', 'startup.m'));
    evalin('base', 'lkaProfile = ''baja''; lka_controller_params; create_lka_buses;');
    vehicle = evalin('base', 'vehicle');
    LKA0 = evalin('base', 'LKA');
    mdl = 'LKA_Vision_TestBench';
    if options.Rebuild || ~isfile(fullfile(root, 'models', [mdl '.slx']))
        build_lka_vision_bench();
    end
    load_system(mdl);
    scenarios = generate_lka_scenarios(vehicle);
    scenarios = scenarios(ismember({scenarios.Name}, options.Scenarios));
    results = struct('res', {}, 'kpi', {});
    for s = 1:numel(scenarios)
        sc = scenarios(s);
        r = sc.Road;
        assignin('base', 'SCN', sc.P);
        assignin('base', 'ROAD', struct('s', r.s, 'x', r.x, 'y', r.y, 'psi', r.psi, 'kappa', r.kappa, ...
            'dkappa', r.dkappa, 'width', r.width, 'lineWidth', r.lineWidth));
        for d = 1:numel(options.Detectors)
            LKA = LKA0;
            LKA.ControllerSelect = options.Controller;
            LKA.Vision.Detector = options.Detectors{d};
            tag = sprintf('%s_%s', sc.Name, LKA.Vision.Detector);
            LKA.Vision.Bench.VideoFile = '';
            if options.Video
                LKA.Vision.Bench.VideoFile = fullfile(options.OutDir, [tag '.mp4']);
            end
            LKA.Vision.Bench.DatasetDir = '';
            if options.Dataset
                LKA.Vision.Bench.DatasetDir = fullfile(root, 'data', 'vision_datasets', tag);
            end
            assignin('base', 'LKA', LKA);
            fprintf('[LKA vision] %-16s camera %-8s ... ', sc.Name, LKA.Vision.Detector);
            t0 = tic;
            simOut = sim(mdl);
            res = lka_extract_results(simOut, sc, LKA, vehicle);
            res.ControllerName = sprintf('%s, camera %s', res.ControllerName, LKA.Vision.Detector);
            kpi = lka_compute_kpis(res);
            vi = squeeze(simOut.logsout.getElement('vision_info').Values.Data);
            if size(vi, 1) == 7, vi = vi'; end
            kpi.Detector = string(LKA.Vision.Detector);
            kpi.VisionMsMean = mean(vi(2:end, 7));
            kpi.BothLinesRatio = mean(vi(:, 3) & vi(:, 4));
            fprintf('%5.0f s | max|lat| %.3f m | in lane %5.1f %% | %.1f ms/frame\n', toc(t0), ...
                kpi.MaxLatOffset_m, 100 * kpi.InLaneRatio, kpi.VisionMsMean);
            plot_lka_results(res, kpi, options.OutDir);
            close all
            results(end + 1) = struct('res', res, 'kpi', kpi); %#ok<AGROW>
        end
    end
    assignin('base', 'LKA', LKA0);
    if ~isfolder(options.OutDir), mkdir(options.OutDir); end
    writetable(struct2table(arrayfun(@(x) x.kpi, results)), fullfile(options.OutDir, 'lka_vision_kpis.csv'));
end
```

- [ ] **Step 7: Run the scenario test, expect PASS**

Run: `setenv('LKA_RUN_UNREAL_TESTS','1'); runtests('tests/scenario_tests/test_lka_vision_bench.m')`
Expected: 2 passed. Watch `results/vision/S1_Nominal_yolop.mp4`.

Decision points:
- YOLOP fails while classic passes (U1, sim domain gap): keep classic for the virtual-round demo, record the YOLOP numbers, and propose the separate "fine-tune YOLOP on sim frames" plan. Do not tune the controller to hide perception errors.
- Out-of-memory on the GPU (U9): set `LKA.Vision.UseGpu = false` in the run and report the ms/frame.
- Run several times slower than real time: acceptable (accuracy first); record the ms/frame.

- [ ] **Step 8: Commit**

```bash
git add scripts/calibrate_vision_bench.m scripts/run_lka_vision.m src/vision/lka_vision_params.m models/LKA_Vision_TestBench.slx tests/scenario_tests/test_lka_vision_bench.m results/vision/*.png results/vision/*.csv
git commit -m "feat(vision): bench calibration and closed-loop camera runs on the LKA track

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

- [ ] **Step 9: Code review checkpoint** (`requesting-code-review` on Tasks 9–11).

---

### Task 12: Detector benchmark (YOLOP vs team CNN vs classic)

**Files:**
- Create: `src/vision/LkaFrameLogger.m`, `src/evaluation/lka_seg_metrics.m`, `src/evaluation/lka_detector_benchmark.m`, `scripts/make_synthetic_vision_dataset.m`, `scripts/run_detector_benchmark.m`
- Modify: `scripts/build_lka_vision_bench.m` (ground-truth boundaries + frame logger replace `Semantic Sink`)
- Test: `tests/unit_tests/test_vision_benchmark.m`

**Interfaces:**
- Consumes: Tasks 3–11.
- Produces: dataset folder format `frame_00001.png`, `frame_00001_label.png` (uint8), `meta.mat` (`meta.t, meta.pose[n×3], meta.gtLeft[n×4], meta.gtRight[n×4], meta.files{n}, meta.Ts, meta.LaneLabelId`), with gt rows `[offset heading curvature dkappa]` at the rear axle. Also `m = lka_seg_metrics(prob, gtMask, thr, tolPx)` (`Precision, Recall, F1, IoU, PixelAccuracy`), `T = lka_detector_benchmark(datasetDir, detectorNames, V)` (table, one row per detector), `make_synthetic_vision_dataset(folder, n, V)` and `T = run_detector_benchmark(Dataset=, Detectors=, ClosedLoop=, OutDir=)`.

- [ ] **Step 1: Write the failing test** `tests/unit_tests/test_vision_benchmark.m`

```matlab
% ==============================================================================
% test_vision_benchmark.m
% Segmentation metrics (incl. the pixel-accuracy trap) and the detector benchmark
% on a synthetic dataset.
% ==============================================================================

function tests = test_vision_benchmark
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    V = LKA.Vision; V.UseGpu = false;
    testCase.TestData.V = V;
end

function testPixelAccuracyTrap(testCase)
    gt = false(720, 1280); gt(:, 300:330) = true; gt(:, 950:980) = true;    % ~5 % lane pixels
    m = lka_seg_metrics(zeros(720, 1280, 'single'), gt, 0.5, 3);
    testCase.verifyGreaterThan(m.PixelAccuracy, 0.94);     % "accurate" while seeing nothing
    testCase.verifyEqual(m.F1, 0);
    testCase.verifyEqual(m.Recall, 0);
end

function testPerfectPrediction(testCase)
    gt = false(100, 100); gt(:, 40:42) = true;
    m = lka_seg_metrics(single(gt), gt, 0.5, 3);
    testCase.verifyEqual([m.Precision m.Recall m.F1 m.IoU], [1 1 1 1]);
end

function testBenchmarkOnSyntheticDataset(testCase)
    folder = tempname;
    make_synthetic_vision_dataset(folder, 8, testCase.TestData.V);
    T = lka_detector_benchmark(folder, {'classic'}, testCase.TestData.V);
    testCase.verifyEqual(height(T), 1);
    testCase.verifyGreaterThan(T.F1, 0.8);
    testCase.verifyGreaterThan(T.DetRateLeft, 0.85);
    testCase.verifyLessThan(T.MaeY10_m, 0.1);
end
```

- [ ] **Step 2: Run it, expect FAIL**

Run: `runtests('tests/unit_tests/test_vision_benchmark.m')`
Expected: FAIL with "Undefined function 'lka_seg_metrics'".

- [ ] **Step 3: Create `src/evaluation/lka_seg_metrics.m`**

```matlab
function m = lka_seg_metrics(prob, gtMask, thr, tolPx)
% LKA_SEG_METRICS Lane-line segmentation quality of one frame.
%
% Syntax:
%   m = lka_seg_metrics(prob, gtMask, thr, tolPx)
%
% Tolerant precision / recall (a prediction within tolPx of a true lane pixel counts,
% and vice versa), F1, strict IoU, and pixel accuracy. Pixel accuracy is reported only
% to expose its bias: lane pixels are ~3-5 % of a frame, so an empty mask scores ~96 %.
    pred = prob > thr;
    gtMask = logical(gtMask);
    se = strel('disk', tolPx);
    m.Precision = nnz(pred & imdilate(gtMask, se)) / max(nnz(pred), 1);
    m.Recall = nnz(gtMask & imdilate(pred, se)) / max(nnz(gtMask), 1);
    m.F1 = 2 * m.Precision * m.Recall / max(m.Precision + m.Recall, eps);
    m.IoU = nnz(pred & gtMask) / max(nnz(pred | gtMask), 1);
    m.PixelAccuracy = mean(pred(:) == gtMask(:));
end
```

- [ ] **Step 4: Create `scripts/make_synthetic_vision_dataset.m`**

```matlab
function make_synthetic_vision_dataset(folder, n, V)
% MAKE_SYNTHETIC_VISION_DATASET Random synthetic lane frames in the benchmark format.
%
% Syntax:
%   make_synthetic_vision_dataset(folder, n, V)          V = LKA.Vision
%
% Offsets +-0.6 m, yaw +-4 deg, curvature +-1/50 1/m, random dashed sides, rendered with
% lka_render_lane_scene. Lane-marking label id 1. Seeded (rng 11) for repeatability.
    if ~isfolder(folder), mkdir(folder); end
    cam = lka_camera_model(V.Camera, V.Bev);
    rng(11);
    meta = struct('t', zeros(n, 1), 'pose', zeros(n, 3), 'gtLeft', zeros(n, 4), ...
        'gtRight', zeros(n, 4), 'files', {cell(n, 1)}, 'Ts', V.Ts, 'LaneLabelId', 1);
    for i = 1:n
        s = lka_default_scene();
        s.Offset = 1.2 * (rand - 0.5);
        s.Yaw = deg2rad(8 * (rand - 0.5));
        s.Curvature = (2 * rand - 1) / 50;
        s.Dashed = rand(1, 2) < 0.3;
        [img, prob, gt] = lka_render_lane_scene(cam, s);
        name = sprintf('frame_%05d', i);
        imwrite(img, fullfile(folder, [name '.png']));
        imwrite(uint8(prob > 0.5), fullfile(folder, [name '_label.png']));
        meta.t(i) = (i - 1) * V.Ts;
        meta.gtLeft(i, :) = [gt.Left.LateralOffset gt.Left.HeadingAngle gt.Left.Curvature gt.Left.CurvatureDerivative];
        meta.gtRight(i, :) = [gt.Right.LateralOffset gt.Right.HeadingAngle gt.Right.Curvature gt.Right.CurvatureDerivative];
        meta.files{i} = name;
    end
    save(fullfile(folder, 'meta.mat'), 'meta');
end
```

- [ ] **Step 5: Create `src/evaluation/lka_detector_benchmark.m`**

```matlab
function T = lka_detector_benchmark(datasetDir, detectorNames, V)
% LKA_DETECTOR_BENCHMARK Scores lane detectors on a recorded dataset.
%
% Syntax:
%   T = lka_detector_benchmark(datasetDir, detectorNames, V)       V = LKA.Vision
%
% Dataset format: LkaFrameLogger / make_synthetic_vision_dataset (meta.mat with
% LaneLabelId and ground-truth boundaries at the rear axle). Each frame is scored on
% its own (fresh detector state) so the numbers describe the detector, not the tracker:
%   Precision, Recall, F1, IoU, PixelAccuracy (lka_seg_metrics, 3 px tolerance)
%   DetRateLeft/Right       share of frames with that boundary found
%   MaeY5/10/20_m, P95Y10_m lateral error of the fitted boundary at x = 5/10/20 m
%   MaeHeading_deg          boundary heading error at the rear axle
%   MsMean, MsP95           detector time per frame (first frame excluded)

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
            m = lka_seg_metrics(prob, gtMask, V.Search.ProbThreshold, 3);
            seg(i, :) = [m.Precision m.Recall m.F1 m.IoU m.PixelAccuracy];
            sensor = lka_vision_lane_step(transformImage(cam.Bev, prob), cam, Vd, lka_vision_state_init(Vd), V.Ts);
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
        r.Precision = mean(seg(:, 1));
        r.Recall = mean(seg(:, 2));
        r.F1 = mean(seg(:, 3));
        r.IoU = mean(seg(:, 4));
        r.PixelAccuracy = mean(seg(:, 5));
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
```

- [ ] **Step 6: Create `src/vision/LkaFrameLogger.m`**

```matlab
classdef LkaFrameLogger < matlab.System
% LKAFRAMELOGGER Records camera frames, semantic labels and ground-truth boundaries.
%
% Inputs: RGB image, semantic label image, rear-axle pose [x y psi], ground-truth left and
% right boundary [offset; heading; curvature; dkappa] (lka_lane_geometry). Every
% Decimation-th frame is written to Folder (benchmark format, see lka_detector_benchmark).
% Development / validation only (rule I.2.3): the logger feeds nothing back.

    properties (Nontunable)
        Folder = ''           % empty = disabled
        Decimation = 5
        Ts = 0.05
        LaneLabelId = 0
    end

    properties (Access = private)
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
            obj.Meta = struct('t', zeros(0, 1), 'pose', zeros(0, 3), 'gtLeft', zeros(0, 4), ...
                'gtRight', zeros(0, 4), 'files', {cell(0, 1)});
            if ~isempty(obj.Folder) && ~isfolder(obj.Folder)
                mkdir(obj.Folder);
            end
        end

        function stepImpl(obj, img, label, pose, gtLeft, gtRight)
            obj.K = obj.K + 1;
            if isempty(obj.Folder) || mod(obj.K - 1, obj.Decimation) ~= 0
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
            meta.Ts = obj.Ts * obj.Decimation;
            meta.LaneLabelId = obj.LaneLabelId;
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
```

- [ ] **Step 7: Add ground truth + logger to `scripts/build_lka_vision_bench.m`**

Replace the line `add_block('simulink/Sinks/Terminator', [mdl '/Semantic Sink'], 'Position', [1780 990 1800 1010]);` and the line `ln(mdl, 'LKA Camera/2', 'Semantic Sink/1');` with:

```matlab
    % ---------------- dataset recording (development / validation only, rule I.2.3)
    gtb = [mdl '/Ground Truth Boundaries (dev only)'];
    add_block('simulink/User-Defined Functions/MATLAB Function', gtb, 'Position', [1540 1000 1700 1100]);
    ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', gtb);
    ch.Script = strjoin({
        'function [gtLeft, gtRight] = fcn(rearPose, centerPose, curv, width)'
        '% True ego-lane boundaries at the rear axle (benchmark ground truth)'
        '[gtLeft, gtRight] = lka_lane_geometry(rearPose, centerPose, curv(1), curv(2), width);'}, newline);
    logger = [mdl '/Frame Logger (dev only)'];
    add_block('simulink/User-Defined Functions/MATLAB System', logger, 'System', 'LkaFrameLogger', ...
        'Position', [1780 990 1960 1110]);
    set_param(logger, 'Folder', 'LKA.Vision.Bench.DatasetDir', 'Decimation', 'LKA.Vision.Bench.Decimation', ...
        'Ts', 'LKA.Ts', 'LaneLabelId', 'LKA.Vision.Bench.LaneLabelId');
    for p = 1:3
        delete_line(mdl, sprintf('Lane Geometry (ground truth)/%d', p), ...
            sprintf('Lane Geometry (ground truth) Term %d/1', p));
        delete_block(sprintf('%s/Lane Geometry (ground truth) Term %d', mdl, p));
        ln(mdl, sprintf('Lane Geometry (ground truth)/%d', p), sprintf('Ground Truth Boundaries (dev only)/%d', p + 1));
    end
    ln(mdl, 'Unreal Rate/1', 'Ground Truth Boundaries (dev only)/1');
    ln(mdl, 'LKA Camera/1', 'Frame Logger (dev only)/1');
    ln(mdl, 'LKA Camera/2', 'Frame Logger (dev only)/2');
    ln(mdl, 'Unreal Rate/1', 'Frame Logger (dev only)/3');
    ln(mdl, 'Ground Truth Boundaries (dev only)/1', 'Frame Logger (dev only)/4');
    ln(mdl, 'Ground Truth Boundaries (dev only)/2', 'Frame Logger (dev only)/5');
```

In `tests/unit_tests/test_vision_bench_build.m`, add `'Frame Logger (dev only)', 'Ground Truth Boundaries (dev only)'` to the block list of `testBenchBuildsAndCompiles`. In `scripts/calibrate_vision_bench.m` nothing changes: it deletes whatever line leaves `LKA Camera/2`.

- [ ] **Step 8: Create `scripts/run_detector_benchmark.m`**

```matlab
function T = run_detector_benchmark(options)
% RUN_DETECTOR_BENCHMARK YOLOP vs team CNN vs classic on the same frames.
%
% Syntax:
%   T = run_detector_benchmark()                                    % record + score + closed loop
%   T = run_detector_benchmark(Dataset='data/vision_datasets/S1_Nominal_classic', ClosedLoop=false)
%
% Name-value options:
%   Dataset    - recorded dataset folder; empty = record one with run_lka_vision (S1, classic)
%   Detectors  - default: classic, yolop (if the ONNX add-on exists), cnn (if team_cnn.onnx exists)
%   ClosedLoop - also run run_lka_vision for each detector and append the lane KPIs (default true)
%   OutDir     - default results/benchmark
% Writes detector_benchmark.csv, detector_benchmark.md and detector_benchmark.png to OutDir.

    arguments
        options.Dataset char = ''
        options.Detectors cell = {}
        options.ClosedLoop (1,1) logical = true
        options.OutDir char = ''
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    if isempty(options.OutDir)
        options.OutDir = fullfile(root, 'results', 'benchmark');
    end
    run(fullfile(root, 'config', 'startup.m'));
    evalin('base', 'lkaProfile = ''baja''; lka_controller_params;');
    V = evalin('base', 'LKA.Vision');
    if isempty(options.Detectors)
        options.Detectors = {'classic'};
        if exist('importNetworkFromONNX', 'file') == 2
            options.Detectors{end + 1} = 'yolop';
            if isfile(fullfile(V.DetectorDir, 'team_cnn.onnx'))
                options.Detectors{end + 1} = 'cnn';
            end
        end
    end
    if isempty(options.Dataset)
        run_lka_vision(Scenarios={'S1_Nominal'}, Detectors={'classic'}, Video=false, Dataset=true);
        options.Dataset = fullfile(root, 'data', 'vision_datasets', 'S1_Nominal_classic');
    end

    T = lka_detector_benchmark(options.Dataset, options.Detectors, V);
    if options.ClosedLoop
        r = run_lka_vision(Scenarios={'S1_Nominal'}, Detectors=options.Detectors, Video=true);
        k = arrayfun(@(x) x.kpi, r);
        T.InLaneRatio = [k.InLaneRatio]';
        T.MaxLatOffset_m = [k.MaxLatOffset_m]';
        T.MinWheelMargin_m = [k.MinWheelMargin_m]';
    end

    if ~isfolder(options.OutDir), mkdir(options.OutDir); end
    writetable(T, fullfile(options.OutDir, 'detector_benchmark.csv'));
    fid = fopen(fullfile(options.OutDir, 'detector_benchmark.md'), 'w');
    names = T.Properties.VariableNames;
    fprintf(fid, '| %s |\n|%s\n', strjoin(names, ' | '), repmat('---|', 1, numel(names)));
    for i = 1:height(T)
        vals = cellfun(@(c) fmt(T{i, c}), names, 'UniformOutput', false);
        fprintf(fid, '| %s |\n', strjoin(vals, ' | '));
    end
    fclose(fid);

    f = figure('Color', 'w', 'Position', [100 100 1100 420]);
    tiledlayout(f, 1, 3);
    nexttile; bar(categorical(T.Detector), [T.F1 T.PixelAccuracy]); legend('F1', 'pixel accuracy'); title('Segmentation');
    nexttile; bar(categorical(T.Detector), [T.MaeY5_m T.MaeY10_m T.MaeY20_m]); legend('5 m', '10 m', '20 m'); title('Boundary error [m]');
    nexttile; bar(categorical(T.Detector), T.MsMean); title('Detector time [ms/frame]');
    exportgraphics(f, fullfile(options.OutDir, 'detector_benchmark.png'), 'Resolution', 150);
    close(f);
    disp(T);
end

function s = fmt(v)
    if isstring(v) || ischar(v)
        s = char(v);
    else
        s = sprintf('%.3g', v);
    end
end
```

- [ ] **Step 9: Run the tests, expect PASS**

Run: `runtests('tests/unit_tests/test_vision_benchmark.m')` (expected: 3 passed), then `runtests('tests/unit_tests/test_vision_bench_build.m')` (expected: 3 passed).

- [ ] **Step 10: Run the real benchmark** (Unreal)

Run: `T = run_detector_benchmark()`
Expected: `results/benchmark/detector_benchmark.{csv,md,png}` exist. Classic and YOLOP rows contain F1, boundary errors, ms/frame and closed-loop KPIs. Report the table to the user; it is the answer to U1 and, once the CNN arrives, to U2.

- [ ] **Step 11: Commit**

```bash
git add src/vision/LkaFrameLogger.m src/evaluation/lka_seg_metrics.m src/evaluation/lka_detector_benchmark.m scripts/make_synthetic_vision_dataset.m scripts/run_detector_benchmark.m scripts/build_lka_vision_bench.m models/LKA_Vision_TestBench.slx tests/unit_tests/test_vision_benchmark.m tests/unit_tests/test_vision_bench_build.m results/benchmark
git commit -m "feat(vision): detector benchmark with dataset recording and tolerant metrics

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 13: Competition model: camera lane source (variant) and sensor models

**Files:**
- Modify: `scripts/integrate_lka_into_skeleton.m` (full replacement below)
- Modify: `src/utilities/lka_build_params.m:209-213` (Cosim block)
- Modify: `data/parameters/lka_controller_params.m` (after `LKA = lka_build_params(...)`)
- Create: `tests/helpers/lka_skeleton_compile.m`
- Test: `tests/scenario_tests/test_skeleton_integration.m`
- Regenerated: `SoftwareVirtualWorldSimulation-master/Models/aBaja2026.slx`

**Interfaces:**
- Consumes: `lka_add_vision_detector`, `lka_add_ego_sensors` (Tasks 9–10), `VirtualLaneSensor` (existing).
- Produces: base variable `LKA_LANE_SOURCE` (1 camera = submission, 2 emulated = development); `LKA.Cosim.YawRateSign`; `[ok, msg] = lka_skeleton_compile(laneSource)`; logged signals `lka_lane_sensor`, `lka_yaw_rate_meas`, `lka_yaw_rate_pose`.

- [ ] **Step 1: Write the failing test** `tests/scenario_tests/test_skeleton_integration.m` and the helper

`tests/helpers/lka_skeleton_compile.m`:

```matlab
function [ok, msg] = lka_skeleton_compile(laneSource)
% LKA_SKELETON_COMPILE Updates aBaja2026.slx without RoadRunner (InitFcn swapped).
%
% Syntax:
%   [ok, msg] = lka_skeleton_compile(laneSource)        1 camera, 2 emulated
%
% The skeleton InitFcn needs a live rrSim; it is replaced for the update by the
% supporting setup helper plus a dummy start pose and restored afterwards (model
% closed without saving). Camera detector = classic, so the ONNX add-on is not needed.
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    skel = fullfile(root, 'SoftwareVirtualWorldSimulation-master');
    run(fullfile(root, 'config', 'startup.m'));
    addpath(fullfile(root, 'models'), fullfile(skel, 'Models'), fullfile(skel, 'SupportingFunctions'));
    evalin('base', 'lkaProfile = ''skeleton''; lka_controller_params; create_lka_buses;');
    evalin('base', sprintf('LKA_LANE_SOURCE = %d; LKA.Vision.Detector = ''classic'';', laneSource));
    mdl = 'aBaja2026';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    load_system(fullfile(skel, 'Models', [mdl '.slx']));
    cleanup = onCleanup(@() close_system(mdl, 0));
    set_param(mdl, 'InitFcn', ['helperSLTrajectoryFollowerWithRRScenarioSetup; ' ...
        'initialPosition = [0 0 0]; initialRotation = [0 0 0];']);
    try
        set_param(mdl, 'SimulationCommand', 'update');
        ok = true; msg = '';
    catch ME
        ok = false; msg = regexprep(ME.message, '<[^>]*>', '');
    end
end
```

`tests/scenario_tests/test_skeleton_integration.m`:

```matlab
% ==============================================================================
% test_skeleton_integration.m
% aBaja2026.slx with the LKA: both lane-source variants compile; the camera variant
% feeds the controller no ground truth (rule I.2.3).
% ==============================================================================

function tests = test_skeleton_integration
    tests = functiontests(localfunctions);
end

function testCameraVariantCompiles(testCase)
    [ok, msg] = lka_skeleton_compile(1);
    testCase.verifyTrue(ok, msg);
end

function testEmulatedVariantCompiles(testCase)
    [ok, msg] = lka_skeleton_compile(2);
    testCase.verifyTrue(ok, msg);
end

function testStructure(testCase)
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    load_system(fullfile(root, 'SoftwareVirtualWorldSimulation-master', 'Models', 'aBaja2026.slx'));
    testCase.addTeardown(@() close_system('aBaja2026', 0));
    lka = 'aBaja2026/LKA (Lane Keeping Assist)';
    testCase.verifyEqual(get_param([lka '/Lane Source'], 'Variant'), 'on');
    testCase.verifyEqual(get_param([lka '/Lane Source/Camera Lane Detector'], 'VariantControl'), 'LKA_LANE_SOURCE == 1');
    testCase.verifyGreaterThan(getSimulinkBlockHandle([lka '/Ego Sensors (wheel speed + IMU)']), 0);
    % the camera image reaches the LKA subsystem
    ph = get_param(lka, 'PortHandles');
    src = get_param(get_param(ph.Inport(5), 'Line'), 'SrcBlockHandle');
    testCase.verifyEqual(getfullname(src), 'aBaja2026/Simulation 3D Camera');
end
```

- [ ] **Step 2: Run it, expect FAIL**

Run: `runtests('tests/scenario_tests/test_skeleton_integration.m')`
Expected: `testStructure` FAILS (no `Lane Source` block). The compile tests may pass on the old integration.

- [ ] **Step 3: Update `src/utilities/lka_build_params.m` Cosim block**

```matlab
    % ---------------------------------------- Co-simulation adapter (skeleton only)
    Cosim.PoseToRearAxle = -Veh.b;       % Longitudinal offset from skeleton CurrPose point to rear axle [m]
    Cosim.YawOffsetDeg   = 0;            % Added to skeleton CurrPose yaw before use [deg]
    Cosim.YawRateSign    = -1;           % CurrYawRate [deg/s] -> + CCW (CurrPose yaw = -90 - psi3D); checked in Phase 5
    Cosim.LkaModes       = [1 2];        % Skeleton Mode values in which the LKA steers (LKA, Endurance)
    LKA.Cosim = Cosim;
```

- [ ] **Step 4: Add the lane-source switch to `data/parameters/lka_controller_params.m`**

Directly after the line `LKA = lka_build_params(lkaProfile, vehicle, sensor);` add:

```matlab
% Lane source of the competition model (variant control of "Lane Source" in aBaja2026.slx):
%   1 = camera lane detector (submission, rule I.2.3)   2 = emulated sensor (development only)
if ~exist('LKA_LANE_SOURCE', 'var')
    LKA_LANE_SOURCE = 1;
end
```

- [ ] **Step 5: Replace `scripts/integrate_lka_into_skeleton.m` entirely**

```matlab
function integrate_lka_into_skeleton(options)
% INTEGRATE_LKA_INTO_SKELETON Wires the LKA into the competition model aBaja2026.slx.
%
% Syntax:
%   integrate_lka_into_skeleton()
%   integrate_lka_into_skeleton(Save=false)
%
% What it does (idempotent: an existing LKA integration is removed first):
%   1. Loads SoftwareVirtualWorldSimulation-master/Models/aBaja2026.slx with the
%      LKA calibration for the skeleton plant (lkaProfile = 'skeleton').
%   2. Inside Subsystem2 exposes three signals as new outports:
%        LKA_VehicleInfo  previous-step vehicle state bus (CurrPose, CurrVelocity, CurrYawRate)
%        LKA_RefPose      closest route point [x y yaw_deg]   (emulated sensor / metrics only)
%        LKA_RefCurvature route curvature [1/m]               (emulated sensor / metrics only)
%   3. Adds the root subsystem "LKA (Lane Keeping Assist)":
%        Lane Source (variant subsystem, control variable LKA_LANE_SOURCE):
%          1 Camera Lane Detector - Simulation 3D Camera image -> YOLOP / team CNN / classic
%          2 Emulated Lane Sensor - route ground truth, development only (rule I.2.3)
%        Ego Sensors (wheel speed + IMU) -> LKA_Controller -> rad to deg
%      and drives Subsystem2/Controller_SteerCmd (road-wheel angle, deg, + left).
%   4. Logs steer command, turning radius, LKA info, lane estimate, lane sensor, the
%      measured yaw rate and the pose-derived yaw rate (YawRateSign check, not an input).
%
% With LKA_LANE_SOURCE = 1 the controller receives only camera lane boundaries, the
% wheel-speed speed and the gyro yaw rate. Saving keeps the model in R2026a format.

    arguments
        options.Save (1,1) logical = true
    end

    root = fileparts(fileparts(mfilename('fullpath')));
    skel = fullfile(root, 'SoftwareVirtualWorldSimulation-master');
    run(fullfile(root, 'config', 'startup.m'));
    addpath(fullfile(root, 'models'), fullfile(skel, 'Models'), fullfile(skel, 'SupportingFunctions'));
    evalin('base', 'lkaProfile = ''skeleton''; lka_controller_params; create_lka_buses;');

    mdl = 'aBaja2026';
    load_system(fullfile(skel, 'Models', [mdl '.slx']));
    s2 = [mdl '/Subsystem2'];
    lkaSys = [mdl '/LKA (Lane Keeping Assist)'];
    removeExisting(mdl, s2, lkaSys);

    % ------------------------------------------------ expose signals from Subsystem2
    taps = {
        'LKA_VehicleInfo',  [s2 '/Delay1'], 1
        'LKA_RefPose',      [s2 '/Reference Pose on Path'], 1
        'LKA_RefCurvature', [s2 '/Reference Pose on Path'], 2};
    nOut = numel(find_system(s2, 'SearchDepth', 1, 'BlockType', 'Outport'));
    for k = 1:size(taps, 1)
        src = get_param(taps{k, 2}, 'PortHandles');
        pos = get_param(taps{k, 2}, 'Position');
        blk = add_block('simulink/Sinks/Out1', [s2 '/' taps{k, 1}], 'Port', num2str(nOut + k), ...
            'Position', [pos(3) + 120, pos(2) + 40 * k, pos(3) + 150, pos(2) + 40 * k + 14]);
        dst = get_param(blk, 'PortHandles');
        add_line(s2, src.Outport(taps{k, 3}), dst.Inport(1), 'autorouting', 'smart');
    end

    % ------------------------------------------------ LKA subsystem at root level
    p2 = get_param(s2, 'Position');
    add_block('simulink/Ports & Subsystems/Subsystem', lkaSys, ...
        'Position', [p2(1) - 420, p2(4) + 80, p2(1) - 180, p2(4) + 300]);
    delete_line(lkaSys, 'In1/1', 'Out1/1');
    delete_block([lkaSys '/In1']);
    delete_block([lkaSys '/Out1']);
    ins = {'VehicleInfo', 'RefPose', 'RefCurvature', 'Mode', 'CameraImage'};
    for k = 1:numel(ins)
        add_block('simulink/Sources/In1', [lkaSys '/' ins{k}], 'Port', num2str(k));
    end
    outs = {'SteerCmdDeg', 'TurnRadius', 'LkaInfo', 'LaneEstimate', 'LaneSensor'};
    for k = 1:numel(outs)
        add_block('simulink/Sinks/Out1', [lkaSys '/' outs{k}], 'Port', num2str(k));
    end

    adapter = [lkaSys '/Skeleton Pose Adapter'];
    add_block('simulink/User-Defined Functions/MATLAB Function', adapter);
    ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', adapter);
    ch.Script = strjoin({
        'function [egoPose, centerPose, curv, speed, yawRate, enable, yawRatePose] = fcn(veh, refPose, refCurv, mode, LKA)'
        '% Skeleton signals -> LKA conventions (SI units, radians, rear-axle reference).'
        '% speed / yawRate stand for the wheel-speed sensor and the gyro (noise added by the'
        '% Ego Sensors block). egoPose, centerPose, curv feed only the emulated sensor'
        '% (development variant). yawRatePose is differentiated from the ground-truth pose'
        '% and only logged to verify LKA.Cosim.YawRateSign (rule I.2.3: not a controller input).'
        'persistent psiPrev'
        'psi = deg2rad(veh.CurrPose(3) + LKA.Cosim.YawOffsetDeg);'
        'if isempty(psiPrev), psiPrev = psi; end'
        'yawRatePose = lka_wrap_angle(psi - psiPrev) / LKA.Ts;'
        'psiPrev = psi;'
        'yawRate = LKA.Cosim.YawRateSign * deg2rad(veh.CurrYawRate);'
        'egoPose = [veh.CurrPose(1) + LKA.Cosim.PoseToRearAxle*cos(psi), ...'
        '           veh.CurrPose(2) + LKA.Cosim.PoseToRearAxle*sin(psi), psi];'
        'centerPose = [refPose(1), refPose(2), deg2rad(refPose(3) + LKA.Cosim.YawOffsetDeg)];'
        'curv = [refCurv(1), 0];'
        'speed = abs(veh.CurrVelocity);'
        'enable = any(mode == LKA.Cosim.LkaModes);'}, newline);
    d = ch.find('-isa', 'Stateflow.Data', 'Name', 'LKA'); d.Scope = 'Parameter'; d.Tunable = false;
    d = ch.find('-isa', 'Stateflow.Data', 'Name', 'veh'); d.DataType = 'Bus: BusVehicleInfo';

    add_block('simulink/Sources/Constant', [lkaSys '/Lane Width'], 'Value', 'LKA.Lane.WidthNominal', 'SampleTime', 'LKA.Ts');
    add_block('simulink/Sources/Constant', [lkaSys '/No Forced Outage'], 'Value', '[0;0]', 'SampleTime', 'LKA.Ts');
    add_block('simulink/Sources/Constant', [lkaSys '/Noise Scale'], 'Value', '1', 'SampleTime', 'LKA.Ts');
    buildLaneSource(lkaSys);
    lka_add_ego_sensors(lkaSys, 'Ego Sensors (wheel speed + IMU)', [500 300 600 380]);
    add_block('simulink/Ports & Subsystems/Model', [lkaSys '/LKA Controller'], ...
        'ModelName', 'LKA_Controller', 'SimulationMode', 'Normal');
    add_block('simulink/Math Operations/Gain', [lkaSys '/Rad To Deg (+ left)'], 'Gain', '180/pi');
    add_block('simulink/Sinks/Terminator', [lkaSys '/Yaw Rate Check (dev log)']);

    a = 'Skeleton Pose Adapter';
    es = 'Ego Sensors (wheel speed + IMU)';
    ln(lkaSys, 'VehicleInfo/1', [a '/1']);
    ln(lkaSys, 'RefPose/1', [a '/2']);
    ln(lkaSys, 'RefCurvature/1', [a '/3']);
    ln(lkaSys, 'Mode/1', [a '/4']);
    ln(lkaSys, 'CameraImage/1', 'Lane Source/1');
    ln(lkaSys, [a '/1'], 'Lane Source/2');
    ln(lkaSys, [a '/2'], 'Lane Source/3');
    ln(lkaSys, [a '/3'], 'Lane Source/4');
    ln(lkaSys, 'Lane Width/1', 'Lane Source/5');
    ln(lkaSys, 'No Forced Outage/1', 'Lane Source/6');
    ln(lkaSys, 'Noise Scale/1', 'Lane Source/7');
    ln(lkaSys, 'Lane Source/1', 'LKA Controller/1');
    ln(lkaSys, 'Lane Source/1', 'LaneSensor/1');
    ln(lkaSys, [a '/4'], [es '/1']);
    ln(lkaSys, [a '/5'], [es '/2']);
    ln(lkaSys, [es '/1'], 'LKA Controller/2');
    ln(lkaSys, [es '/2'], 'LKA Controller/3');
    ln(lkaSys, [a '/6'], 'LKA Controller/4');
    ln(lkaSys, [a '/7'], 'Yaw Rate Check (dev log)/1');
    ln(lkaSys, 'LKA Controller/1', 'Rad To Deg (+ left)/1');
    ln(lkaSys, 'Rad To Deg (+ left)/1', 'SteerCmdDeg/1');
    ln(lkaSys, 'LKA Controller/2', 'TurnRadius/1');
    ln(lkaSys, 'LKA Controller/3', 'LkaInfo/1');
    ln(lkaSys, 'LKA Controller/4', 'LaneEstimate/1');
    logPort([lkaSys '/' es], 2, 'lka_yaw_rate_meas');
    logPort([lkaSys '/' a], 7, 'lka_yaw_rate_pose');
    try Simulink.BlockDiagram.arrangeSystem(lkaSys); catch, end

    % ------------------------------------------------ root wiring
    ln(mdl, sprintf('Subsystem2/%d', nOut + 1), 'LKA (Lane Keeping Assist)/1');
    ln(mdl, sprintf('Subsystem2/%d', nOut + 2), 'LKA (Lane Keeping Assist)/2');
    ln(mdl, sprintf('Subsystem2/%d', nOut + 3), 'LKA (Lane Keeping Assist)/3');
    ln(mdl, 'Mode/1', 'LKA (Lane Keeping Assist)/4');
    ln(mdl, 'Simulation 3D Camera/1', 'LKA (Lane Keeping Assist)/5');
    ln(mdl, 'LKA (Lane Keeping Assist)/1', 'Subsystem2/1');

    names = {'lka_steer_cmd_deg', 'lka_turn_radius', 'lka_info', 'lane_estimate', 'lka_lane_sensor'};
    ph = get_param(lkaSys, 'PortHandles');
    for k = 1:numel(names)
        if k > 1
            t = add_block('simulink/Sinks/Terminator', sprintf('%s/LKA Log Sink %d', mdl, k), ...
                'Position', get_param(lkaSys, 'Position') + [300 40*k 270 40*k-140]);
            tp = get_param(t, 'PortHandles');
            add_line(mdl, ph.Outport(k), tp.Inport(1), 'autorouting', 'smart');
        end
        set_param(get_param(ph.Outport(k), 'Line'), 'Name', names{k});
        set_param(ph.Outport(k), 'DataLogging', 'on', 'DataLoggingNameMode', 'SignalName');
    end
    set_param(mdl, 'SignalLogging', 'on', 'SignalLoggingName', 'logsout');
    set_param(lkaSys, 'Description', ['aBaja 2026 LKA: lane source (camera detector YOLOP / team CNN / ' ...
        'classic, or emulated for development), wheel-speed and gyro models, LKA_Controller (lane tracker, ' ...
        'supervisor, pure pursuit / Stanley 2025 / MPC, a_y-jerk limiter, actuator manager). Output: ' ...
        'road-wheel angle [deg, + left] to Subsystem2/Controller_SteerCmd.']);

    if options.Save
        save_system(mdl);
        fprintf('[LKA] Integrated and saved %s\n', get_param(mdl, 'FileName'));
    end
end

function buildLaneSource(lkaSys)
    vs = [lkaSys '/Lane Source'];
    add_block('built-in/SubSystem', vs, 'Variant', 'on', 'Position', [300 40 480 260]);
    ins = {'CameraImage', 'EgoPose', 'CenterPose', 'Curvature', 'LaneWidth', 'Outage', 'NoiseScale'};
    for k = 1:numel(ins)
        add_block('built-in/Inport', [vs '/' ins{k}], 'Port', num2str(k), 'Position', [20 30 + 40 * k 50 44 + 40 * k]);
    end
    add_block('built-in/Outport', [vs '/LaneSensor'], 'Port', '1', 'Position', [600 100 630 114]);

    c1 = [vs '/Camera Lane Detector'];
    add_block('built-in/SubSystem', c1, 'VariantControl', 'LKA_LANE_SOURCE == 1', 'Position', [200 40 380 140]);
    choicePorts(c1, ins);
    lka_add_vision_detector(c1, 'Vision Lane Detector', [200 40 380 160]);
    add_line(c1, 'CameraImage/1', 'Vision Lane Detector/1');
    add_line(c1, 'Vision Lane Detector/1', 'LaneSensor/1');
    for k = 2:3
        add_block('built-in/Terminator', sprintf('%s/Vision Out %d', c1, k));
        add_line(c1, sprintf('Vision Lane Detector/%d', k), sprintf('Vision Out %d/1', k));
    end
    terminateInputs(c1, ins(2:end));

    c2 = [vs '/Emulated Lane Sensor (development only)'];
    add_block('built-in/SubSystem', c2, 'VariantControl', 'LKA_LANE_SOURCE == 2', 'Position', [200 180 380 280]);
    choicePorts(c2, ins);
    add_block('simulink/Ports & Subsystems/Subsystem Reference', [c2 '/Virtual Lane Sensor'], ...
        'ReferencedSubsystem', 'VirtualLaneSensor');
    for k = 2:numel(ins)
        add_line(c2, [ins{k} '/1'], sprintf('Virtual Lane Sensor/%d', k - 1));
    end
    add_line(c2, 'Virtual Lane Sensor/1', 'LaneSensor/1');
    terminateInputs(c2, ins(1));
end

function choicePorts(c, ins)
    for k = 1:numel(ins)
        add_block('built-in/Inport', [c '/' ins{k}], 'Port', num2str(k), 'Position', [20 30 + 40 * k 50 44 + 40 * k]);
    end
    add_block('built-in/Outport', [c '/LaneSensor'], 'Port', '1', 'Position', [600 100 630 114]);
end

function terminateInputs(c, names)
    for k = 1:numel(names)
        add_block('built-in/Terminator', [c '/Unused ' names{k}]);
        add_line(c, [names{k} '/1'], ['Unused ' names{k} '/1']);
    end
end

function logPort(blk, port, name)
    ph = get_param(blk, 'PortHandles');
    lh = get_param(ph.Outport(port), 'Line');
    if lh > 0
        set_param(lh, 'Name', name);
    end
    set_param(ph.Outport(port), 'DataLogging', 'on', 'DataLoggingNameMode', 'SignalName');
end

function removeExisting(mdl, s2, lkaSys)
    if getSimulinkBlockHandle(lkaSys) > 0
        lh = get_param(lkaSys, 'LineHandles');
        delete_line([lh.Inport(lh.Inport > 0), lh.Outport(lh.Outport > 0)]);
        delete_block(lkaSys);
    end
    sinks = find_system(mdl, 'SearchDepth', 1, 'Regexp', 'on', 'Name', '^LKA Log Sink');
    for k = 1:numel(sinks)
        delete_block(sinks{k});
    end
    for nm = {'LKA_VehicleInfo', 'LKA_RefPose', 'LKA_RefCurvature'}
        b = [s2 '/' nm{1}];
        if getSimulinkBlockHandle(b) > 0
            lh = get_param(b, 'LineHandles');
            if lh.Inport > 0, delete_line(lh.Inport); end
            delete_block(b);
        end
    end
    % Dangling lines left at the root after removing ports
    delete_line(find_system(mdl, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line', 'Connected', 'off'));
end

function ln(sys, src, dst)
    add_line(sys, src, dst, 'autorouting', 'smart');
end
```

If `add_block('built-in/SubSystem', vs, 'Variant', 'on')` is rejected in R2026a, use `add_block('simulink/Ports & Subsystems/Variant Subsystem', vs)`. Then delete the template's default choice subsystems and ports (list them with `find_system(vs, 'SearchDepth', 1)`) before adding the ports and choices above, and note this in the commit.

- [ ] **Step 6: Re-integrate and run the test, expect PASS**

Run: `integrate_lka_into_skeleton(); runtests('tests/scenario_tests/test_skeleton_integration.m')`
Expected: 3 passed.

- [ ] **Step 7: Commit**

```bash
git add scripts/integrate_lka_into_skeleton.m src/utilities/lka_build_params.m data/parameters/lka_controller_params.m tests/helpers/lka_skeleton_compile.m tests/scenario_tests/test_skeleton_integration.m SoftwareVirtualWorldSimulation-master/Models/aBaja2026.slx
git commit -m "feat(lka): camera lane source and sensor models in the competition model

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 14: Rulebook score, documentation, full verification

**Files:**
- Modify: `src/evaluation/lka_compute_kpis.m:1-12` (header) and after the `InLaneRatio` line
- Modify: `tests/scenario_tests/test_lka_mil_scenarios.m` (one assertion)
- Modify: `docs/lka_controller.md` (new chapter "Camera lane detection"), `README.md` (quick start), `models/README.md` (new bench)

**Interfaces:**
- Produces: `kpi.RulebookScore = T_in / T_total^2` [1/s].

- [ ] **Step 1: Write the failing assertion**

In `tests/scenario_tests/test_lka_mil_scenarios.m`, inside the S1 test after the existing KPI checks, add:

```matlab
    testCase.verifyEqual(kpi.RulebookScore, kpi.TimeInLane_s / kpi.TotalTime_s^2, 'RelTol', 1e-12);
```

(Use the variable name the file already uses for the KPI struct.)

- [ ] **Step 2: Run it, expect FAIL**

Run: `runtests('tests/scenario_tests/test_lka_mil_scenarios.m')`
Expected: FAIL with "Unrecognized field name 'RulebookScore'".

- [ ] **Step 3: Update `src/evaluation/lka_compute_kpis.m`**

Replace the header lines 4–8 (from `% Rulebook (aBAJA 2027 Rev00, K.2)` to `% is reported as the score proxy.`) with:

```matlab
% Rulebook (aBAJA 2027 Rev00, K.2.4-K.2.5): any tyre touching the line = outside the lane;
% all four tyres out at any instant, a manual take-over or > 33 km/h = DQ. Score
% S = T_inside_lane / T_total_time^2 (scaled to 150 points by the organisers), so faster
% runs score higher as long as the car stays in the lane. TotalTime_s here is the time
% from moving off to the end of the run (the start/stop lines are not modelled).
```

After the line `kpi.InLaneRatio = kpi.TimeInLane_s / kpi.TotalTime_s;` add:

```matlab
    kpi.RulebookScore = kpi.TimeInLane_s / kpi.TotalTime_s^2;   % S = T_in / T_total^2 [1/s]
```

- [ ] **Step 4: Run it, expect PASS**

Run: `runtests('tests/scenario_tests/test_lka_mil_scenarios.m')`
Expected: 5 passed.

- [ ] **Step 5: Documentation**

Add to `docs/lka_controller.md` a chapter **"Camera lane detection (rule I.2.3)"** containing:
1. The block diagram camera → detector → BEV → search/fit → `LaneSensorBus` → `LKA_Controller`, plus the variant switch `LKA_LANE_SOURCE`.
2. The camera model equations: pixel → ground for a flat road. Ray direction `d = o + xn·r + yn·dn` with `o = (cosθ, 0, −sinθ)`, `dn = (−sinθ, 0, −cosθ)`, `r = (0, −1, 0)`; ground point `p = c + (h/−d_z)·d`.
3. The search and fit parameters table (`lka_vision_params.m`).
4. A detector switch how-to (`LKA.Vision.Detector`, CNN contract link `models/perception/README.md`).
5. Benchmark results table (copy `results/benchmark/detector_benchmark.md`).
6. Camera-in-the-loop KPIs (copy `results/vision/lka_vision_kpis.csv` columns Scenario, Detector, MaxLatOffset_m, InLaneRatio, MinWheelMargin_m, RulebookScore, VisionMsMean).
7. A link to `docs/adas2026_audit.md`.

Add to `README.md` quick start:

```matlab
run('config/startup.m');
run_lka_mil();                                          % controller MIL (emulated detector)
report = calibrate_vision_bench();                      % once: Unreal frame map, label id, camera height
run_lka_vision(Detectors={'yolop','classic'});          % camera-in-the-loop, official track, videos
run_detector_benchmark();                               % YOLOP vs team CNN vs classic
integrate_lka_into_skeleton();                          % competition model (LKA_LANE_SOURCE = 1 camera)
```

Add `LKA_Vision_TestBench.slx` to `models/README.md`.

- [ ] **Step 6: Full verification** (`verification-before-completion`)

Run, in order, and paste the summary lines into the final report:

```matlab
r1 = runtests('tests/unit_tests');                 % all unit tests
r2 = runtests('tests/scenario_tests');             % MIL + skeleton (Unreal tests skipped unless opted in)
setenv('LKA_RUN_UNREAL_TESTS', '1');
r3 = runtests('tests/scenario_tests/test_lka_vision_bench.m');
table([r1 r2 r3])
```

Expected: all pass, except the known pre-existing legacy failure `test_iso_11270_compliance` (old script-based Stanley, 1.51 m, documented before this plan). Report it again; do not hide it.

- [ ] **Step 7: Commit**

```bash
git add src/evaluation/lka_compute_kpis.m tests/scenario_tests/test_lka_mil_scenarios.m docs/lka_controller.md README.md models/README.md results
git commit -m "docs(lka): camera lane detection chapter, rulebook score, verification results

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

- [ ] **Step 8: Final code review** (`requesting-code-review` on the whole branch), then `finishing-a-development-branch`.

---

## After this plan

- **Phase 5 (RoadRunner co-simulation)**, once RoadRunner R2026a is installed: run `aBaja2026.slx` with `LKA_LANE_SOURCE = 1`. Then:
  - verify the skeleton camera extrinsics (U4);
  - check `LKA.Cosim.YawRateSign` against the logged `lka_yaw_rate_pose` (U10);
  - export the RoadRunner video.
- **YOLOP fine-tuning on sim frames** (only if Task 12 shows a domain gap): a separate plan. It would use the `LkaFrameLogger` datasets with semantic labels as training masks.
- **Team CNN (UFLDv2)**: retrain variants (res34, CULane, own track images) need only a new ONNX export and, if the dataset config changes, new sidecar numbers in `cnn.json`. Then run `run_detector_benchmark()`. No code changes.
- **Jetson phase**: work through the fix queue in `docs/adas2026_audit.md`.
