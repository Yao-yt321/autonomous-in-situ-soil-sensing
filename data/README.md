# Processed field data

This directory contains processed field data associated with the two control methods released in this repository:

1. DCA-MPC for path tracking and stopping at target detection points.
2. Adaptive probe penetration using axial force feedback estimated from robotic-arm joint torques.

The data are provided as compact CSV files corresponding to quantitative results reported in the manuscript
“Autonomous in situ soil sensing with location and penetration depth control”.

## Data source and collection

Field trials were conducted in May 2026 at Chunhui Family Farm in Gong'an County, Jingzhou, Hubei Province, China.
The field was managed under a rice-rapeseed rotation system.

For navigation experiments, 50 target detection points and six continuous paths totaling 798.9 m were used.
PP, Stanley, MPC, and DCA-MPC were evaluated on the same reference paths. Each controller completed three
independent full-route trials, and robot pose was recorded at 10 Hz.

For penetration experiments, all trials used DCA-MPC for path tracking and stopping. Three probe-feed strategies
were evaluated: adaptive force-feedback control, constant 0.10 m/s, and constant 0.30 m/s. One complete 50-point
field trial was conducted for each penetration strategy.

## Files

### navigation/path_tracking_summary.csv
Processed navigation-performance summary corresponding to Table 3 of the manuscript.

Columns report:
- controller;
- mean maximum absolute lateral offset;
- lateral RMSE mean and SD;
- mean maximum absolute heading offset;
- heading RMSE mean and SD;
- mean maximum stopping offset.

Straight and curved path segments are reported separately. Curved segments include gentle and sharp curves.

### navigation/dca_mpc_stopping_offsets.csv
Pointwise mean stopping offsets for DCA-MPC at the 50 target detection points, averaged over three field trials.

These values give a range of 2.11–5.22 cm and an overall mean of 4.04 cm, as reported for Fig. 13.

### penetration/penetration_depths.csv
Terminal penetration depths at 50 target detection points for:
- adaptive force-feedback feed speed;
- constant feed speed of 0.10 m/s;
- constant feed speed of 0.30 m/s.

The target penetration depth was 10.0 cm and the maximum allowable penetration depth was 10.5 cm.
The data reproduce the terminal-depth results reported for Fig. 14:
- adaptive: MAE 0.207 cm, SD 0.124 cm;
- 0.10 m/s: MAE 0.332 cm, SD 0.439 cm;
- 0.30 m/s: MAE 0.537 cm, SD 0.663 cm.

### penetration/representative_force_responses.csv
Representative estimated axial-force responses corresponding to Fig. 15a.

Representative points:
- hard soil: detection point 35, penetration resistance 3.47 MPa;
- moderately compacted soil: detection point 10, penetration resistance 1.53 MPa;
- soft soil: detection point 23, penetration resistance 0.84 MPa.

Each point includes adaptive, 0.10 m/s, and 0.30 m/s strategies.
The touchdown threshold was 5.0 N and the axial-force safety threshold was 66.7 N.

### penetration/reported_peak_force_statistics.csv
Compact statistics for the peak-force distributions reported for Fig. 15b.

The file contains the reported median, interquartile range, and the number and percentage of target points
reaching or exceeding the 66.7 N load limit for each penetration strategy.

## Units

- lateral and stopping offsets: cm
- heading offsets: degrees
- penetration depth: cm
- time: s
- axial force: N
- penetration resistance: MPa

## Accessibility

All released datasets are UTF-8 CSV files and can be read directly in MATLAB, Python, R, and spreadsheet software.
