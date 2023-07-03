# Sonar

Sonar package aims to provide a simple way to use sonar sensors ping1D and ping360.

## Installation

```bash
pip install -e .
```

## Usage

### ping1D

```python
from sonar import Ping1D

ping = Ping1D("/dev/ttyUSB0", 115200)

# Get distance in meters
distance = ping.get_distance()
```

### ping360

```python
from sonar import Ping360

ping = Ping360("/dev/ttyUSB0", 115200)

# get a full scan
scan = ping.read_full_scan()

# get a single step
points = ping.read()
```
