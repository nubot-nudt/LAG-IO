# Data format

Input files contain consecutive native little-endian IEEE-754 `float64` records without headers, as used by the provided datasets. The supported deployment is a little-endian Linux host. All six streams must already be synchronized, sampled at the configured rate, and expressed in seconds on the same time base. Missing records, duplicated timestamps, non-finite values, and truncated input are errors.

## Directory and file names

Keep the following recording directories when packaging the download. The sequence manifests resolve paths relative to `config/`.

| Sequence | Recording directory | Body IMU file within `gins/` |
| --- | --- | --- |
| asphalt road | `data/20250211/` | `HL20250211192224_ICM_0_IMU_align.bin` |
| Parkinglots | `data/20250218-1/` | `HL20250218211606_ICM_0_IMU_align.bin` |
| Grass | `data/20250218-2/` | `HL20250218224424_ICM_0_IMU_align.bin` |
| Construction site / Offroad | `data/20250219/` | `HL20250219213430_ICM_0_IMU_align.bin` |

Within each recording, the four leg files are `FL/Mean_imu_synced_align.bin`, `FR/Mean_imu_synced_align.bin`, `RL/Mean_imu_synced_align.bin`, and `RR/Mean_imu_synced_align.bin`. The encoder file is `robot/go2_sensor_synced_align.bin`. Include the processed GT files listed below when available. Construction site and Offroad select different time ranges from the same recording.

## IMU input

Each row has seven values:

| Columns (zero-based) | Meaning | Unit |
| --- | --- | --- |
| 0 | Timestamp | s |
| 1–3 | Angular velocity in the sensor frame | rad/s |
| 4–6 | Specific force in the sensor frame | m/s² |

Use rates, not integrated angle or velocity increments. Sensor axes must already follow the dataset convention. The local navigation frame has gravity along positive z; a level stationary IMU measures approximately `[0, 0, -9.782940329221166]`. Initial roll and pitch and gyro bias are estimated from the configured stationary alignment interval; initial yaw is zero. Start with the robot stationary. Foot lever arms are vectors from each leg IMU origin to its contact point, expressed in that IMU's frame.

The centered KDE buffer has 11 samples. At 200 Hz, IMU input must cover at least 0.025 s beyond `endtime`. The robot record used for a keyframe is matched to the buffer-center timestamp.

## Robot input

Each row has 29 values:

| Columns (zero-based) | Meaning | Unit |
| --- | --- | --- |
| 0 | Timestamp | s |
| 1–12 | Joint angles, three per leg | rad |
| 13–24 | Joint angular rates, three per leg | rad/s |
| 25–28 | Foot forces | N |

Leg order is front left, front right, rear left, rear right. Joint order is hip abduction, hip flexion, knee. The kinematic model uses the Unitree Go2 geometry and the coordinate conversion in `rotmat`. Foot force is present in the record format but does not gate the current contact detector.

## Output

`<Body|FL|FR|RL|RR>_traj.txt` is whitespace-delimited:

```text
time x y z vx vy vz roll pitch yaw
```

Positions and velocities are in the local navigation frame, in metres and m/s. Euler angles are in degrees. `<label>_imuerror.txt` contains:

```text
time gyro_bias_x gyro_bias_y gyro_bias_z acc_bias_x acc_bias_y acc_bias_z
```

Gyro bias is in degrees/hour; accelerometer bias is in mGal. Bias coordinates are sensor coordinates. Output includes the initial keyframe, contact-triggered keyframes, and periodic inertial-only keyframes when contact is absent. A final keyframe is exported at the last processed sample. Inertial-only intervals remain uncorrected by contact measurements and may drift.


## Ground truth for plotting

The processed `gt_tum_*.txt` files contain local Cartesian position in columns 1–3 (zero-based), after the timestamp in column 0. These files follow the initial-frame conversion used in the original plotting workflow. Quaternion columns are ignored by the position plot. Use `--sync --origin --swap-xy` to compare common timestamps, start both curves at the origin, and reproduce the local-y/local-x planar display. There is no whole-trajectory alignment or scale fitting.

| Sequence | GT path relative to `data/` |
| --- | --- |
| `asphalt road` | `20250211/gins/gt_tum_20250211-2.txt` |
| `Parkinglots` | `20250218-1/gins/gt_tum_20250218-1.txt` |
| `Grass` | Processed GT unavailable in the local dataset |
| `Construction site` | `20250219/gins/gt_tum_20250219-1.txt` |
| `Offroad` | `20250219/gins/gt_tum_20250219-3.txt` |

Some GT exports have equal adjacent timestamps after decimal rounding. Their positions are averaged at that timestamp before interpolation; the plotting command reports the number merged. Original files remain unchanged. Raw `gins_pva.txt` contains geodetic latitude, longitude, and height and must not be passed as XYZ metres.
