# loosely
Loosely coupled gps-ins algorithm with misalignment self-calibration

## Usage

### .mat file with carsim

- import mat file
- set initial values in carsim_conv.m
- run carsim_conv.m
- plot with fmtplot.m

### .bag file with ros

- open rosins.m
- edit the bagfile name in first line
- run rosins.m
- if you want, manually plot data in 'sol' structure.
