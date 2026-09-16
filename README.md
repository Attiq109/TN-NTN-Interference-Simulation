# TN-NTN Integration

This repository contains a MATLAB simulation for terrestrial and non-terrestrial network integration. It focuses on the baseline coexistence scenario with terrestrial base stations, terrestrial user equipment, and aerial user equipment.

## Contents

- `main_task_TN_NTN_Integration.m`: main simulation script.
- `gain_ant.m`, `gain_ant_NTN.m`: antenna gain helper functions.
- `gen_hex.m`: hexagonal sector/user placement helper.
- `PathLoss_Vec.m`, `ProbLoS_Vec.m`, `rain_attenuation.m`: channel, LoS, path-loss, and rain attenuation helpers.
- `Results/TN_NTN`: generated scenario layout, plots, summary CSV, and MATLAB result file.

## Scenario

The simulation models:

- One desired terrestrial cell with three sectors.
- Six neighboring terrestrial interfering cells.
- Aerial user equipment distributed over the non-terrestrial layer.
- Interference from terrestrial base stations and aerial transmitters toward terrestrial victim users.
- Interference, SINR, throughput, and CDF statistics over Monte Carlo realizations.

## How To Run

Open MATLAB in this folder and run:

```matlab
main_task_TN_NTN_Integration
```

The script saves the scenario layout, interference-distance plot, CDF plots, CSV summary, and MAT result file under:

```text
Results/TN_NTN
```

By default, the script uses 1000 Monte Carlo realizations. For a faster smoke test, set:

```matlab
setenv('TN_NTN_REALIZATIONS', '20')
main_task_TN_NTN_Integration
```

Clear the override or set a larger value for smoother publication-quality curves.

## Main Outputs

- `Scenario_Layout.png`: deployment layout.
- `Interference_vs_Distance.png`: interference power versus terrestrial user distance.
- `CDF_Interference.png`: interference CDF.
- `CDF_SINR.png`: SINR outage CDF.
- `CDF_Throughput.png`: throughput CDF.
- `TN_NTN_summary.csv`: distance-binned numerical summary.
- `TN_NTN_results.mat`: MATLAB data for further analysis.

## Notes

The distance-based SINR and throughput plots are not generated because those binned averages can fluctuate strongly with random user placement and fading. The CDF outputs provide a more stable view of the same performance quantities.
