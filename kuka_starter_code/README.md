# Simulate Optitrack Values in Redis

Python script to Simulate and Log Optitrack Values in Redis.

### Quickstart
0. Run `cmake .. && make` in `/build` 
1. Run `./bin/simviz`
2. Run `./bin/controller`
3. Run `python kuka_starter_code/simulate_optitrack.py`

### Preset Flight Mode
```
python simulate_optitrack.py -p circle -t 1.0
python simulate_optitrack.py -p figure_eight -t 0.5
python simulate_optitrack.py -p sideways -t 1.0
```

### Logger Mode
```
python simulate_optitrack.py -d
```

### Options
```
usage: simulate_optitrack.py [-h] [-p PRESET_FLIGHT] [-t TIME_MULTIPLIER] [-d]

optional arguments:
  -h, --help            show this help message and exit
  -p PRESET_FLIGHT, --preset-flight PRESET_FLIGHT
                        Choice of flight preset for the target (figure_eight,
                        circle, sideways, triangle_periodic).
                        Default=figure_eight
  -t TIME_MULTIPLIER, --time-multiplier TIME_MULTIPLIER
                        Multiplier applied to the time of the target speed.
                        Default=0.05
  -d, --disable-simulator
                        Disable the simulator values. Only read values from
                        redis.
```
