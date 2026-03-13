# EKF Plotting & Analysis Tools

## Available datasets

| Dataset | Path | Type |
|---------|------|------|
| Test 1 | `data_paper/test-1/` | Static — AUV stationary |
| Test 2 | `data_paper/test-2/` | Dynamic — AUV in motion |

## Workflow

### 1. Prepare data

Copy the desired dataset CSVs (`imu_*.csv`, `usbl_*.csv`, `depth_*.csv`) into `sensor_logs/`.
All scripts auto-detect the most recent CSV by prefix in that folder.

### 2. Run EKF fusion

```bash
python3 ekf_sensor_fusion.py
```

Reads sensor CSVs from `sensor_logs/`, runs the EKF, saves the estimated trajectory as `sensor_logs/ekf_<timestamp>.csv` and displays the result plot.

### 3. Plot results

```bash
python3 ekf_plot.py
python3 ekf_plot.py --gt sensor_logs/ground_truth_*.csv   # with reference trajectory overlay
```

Three-panel plot: 2D map with EKF trajectory, USBL range comparison, depth profile.
Automatically loads the latest `ekf_*.csv` from `sensor_logs/`.

### 4. Sensor diagnostics

```bash
python3 sensor_diagnostics.py
```

Analyses raw sensor data from the CSVs in `sensor_logs/`.

**Important:** this script does **not** use `T_START`/`T_END` from `ekf_config.py`. To restrict the analysis window, manually trim the CSV files by removing unwanted rows based on the `timestamp_rel` column before running the script.

### 5. Reference trajectory editor

```bash
python3 ground_truth_editor.py
python3 ground_truth_editor.py --ekf sensor_logs/ekf_*.csv   # overlay EKF trajectory
```

Interactive GUI to draw a reference trajectory on the pool map.

**How it works:**
- Click to place waypoints on the 2D map.
- The tool interpolates through the waypoints using a **cubic spline** to produce a smooth trajectory.
- Press `S` to toggle between spline and linear interpolation.
- Press `Enter` to save as `ground_truth_<timestamp>.csv`.

Controls: left click = add, right click = remove, `Z` = undo, `C` = clear, `Esc` = exit without saving.

## Configuration (`ekf_config.py`)

| Parameter | Effect |
|-----------|--------|
| `T_START` / `T_END` | Time window filter — restricts fusion and plotting to `[T_START, T_END]` seconds. Set `None` to disable. |

Visualization elements (pool boundary, transponder position, obstacle rectangle, reference points) are also configured in this file.

## File naming convention

All CSVs follow the pattern `<prefix>_<YYYYMMDD_HHMMSS>.csv`:
`imu_`, `usbl_`, `depth_`, `ekf_`, `ground_truth_`.
