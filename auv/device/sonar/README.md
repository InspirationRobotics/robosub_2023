# Sonar

Sonar package aims to provide a simple way to use sonar sensors ping1D and ping360.

## Usage

### ping1D

```python
from sonar import Ping1D

ping = Ping1D("/dev/ttyUSB2", 115200)

# Get distance in meters
distance = ping.get_distance()
```

### ping360

```python
from sonar import Ping360

ping = Ping360("/dev/ttyUSB2", 115200)

# get a full scan
for timestamp, angle, data in ping:
    # do something here with the data
    print(angle, data)

# get a single step
ts, angle, points = ping.step_scan()
```

## Run tests

anywhere in the repo, run:

```bash
pytest .
```
or 
```bash
python -m pytest .
```
