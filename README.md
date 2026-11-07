# Drone Landing Simulation

A personal, AI-assisted study for European Rover Challenge (ERC) preparation. I compared ways of weighting camera observations when a simulated quadrotor lands on an ArUco marker in blurred, dark or partially occluded images.

![Landing error and observation coverage on held-out seeds](results/calibrated_comparison.png)

## Method

MATLAB renders a downward-facing camera and computes Farnebäck optical flow from consecutive frames. The estimator combines timestamped, noisy inertial and height measurements with image-derived motion and ArUco positions. Ground truth is used to generate sensor data, fit the observation noise offline and score the results. The estimator and controller use the simulated sensor readings during flight.

The calibration uses 994 accepted marker observations from seeds 111, 222 and 333. I held out seeds 444 and 555 and compared the original fixed observation noise, a calibrated constant noise level and a calibrated model that adjusts noise using image features and estimator state.

## Results

Mean horizontal landing error in centimetres, over two flights per condition and method.

| Image condition | Original fixed | Calibrated constant | Calibrated by features |
| --- | ---: | ---: | ---: |
| Blur | 1.34 | 0.77 | 0.65 |
| Low light | 4.12 | 0.35 | 0.53 |
| Partial occlusion | 1.36 | 0.78 | 0.65 |
| **All conditions** | **2.27** | **0.64** | **0.61** |

The calibrated constant captured most of the improvement and had the lowest error in low light. The feature-based model averaged only 0.03 cm lower than the calibrated constant. All 18 runs used synthetic images and an ideal simulated ground plane. The evaluation used two held-out seeds at the same degradation settings as calibration. Hardware accuracy and performance in new conditions remain untested.

[Per-run results](results/comparison.csv) · Example frames [blur](results/blur_example.png), [low light](results/dark_example.png), [partial occlusion](results/occlusion_example.png)

## Reproduce the comparison

Run from the repository root with MATLAB R2026a, Computer Vision Toolbox, Image Processing Toolbox, and Python 3 with NumPy.

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install numpy
matlab -batch "run_calibrated_images"
python analyse.py
matlab -batch "plot_results"
```

The runner uses the included frozen calibration and writes the per-flight logs to `results/`. The analysis checks the model digest, training data digests and separation of calibration and evaluation seeds. The included [comparison CSV](results/comparison.csv) and figure are the recorded study results. To refit the model, run `python fit_calibration.py` in a separate copy before the simulation. The nine calibration CSVs contain synthetic observations.
