# C Odometry Library (Differential Drive)

A lightweight and robust **2D odometry library in C** for differential-drive robots.

This library estimates robot **pose (x, y, θ)** and **velocity** using encoder ticks, with support for:

* Encoder wraparound (overflow)
* Signed / unsigned encoder ranges
* Per-wheel direction inversion
* Clean reset and initialization

---


## Project Structure
```
.
├── include/
│   └── odometry.h
├── src/
│   └── odometry.c
├── examples/
│   └── basic_example.c
├── CMakeLists.txt
├── README.md
└── LICENSE
```
---



## Build

```bash
mkdir build
cd build
cmake ..
make
./basic_example
```

---



## API Overview

| Function            | Description            |
| ------------------- | ---------------------- |
| `odom_init`         | Initialize odometry    |
| `odom_update`       | Update pose from ticks |
| `odom_get_pose`     | Get (x, y, θ)          |
| `odom_get_velocity` | Get (v, ω)             |
| `odom_reset`        | Reset everything       |
| `odom_reset_pose`   | Reset pose only        |
| `odom_reset_ticks`  | Reset tick baseline    |

---


##  Usage

### 1. Initialize
```bash
odometry_t odom;

odom_init(&odom,
          0.05,      // wheel radius (meters)
          0.30,      // wheel base (meters)
          2048.0,    // ticks per revolution
          -32768,    // encoder min
          32767,     // encoder max
          false,     // left inverted
          false);    // right inverted
```

### 2. First Update
Initialize Encoder Baseline. The first update does NOT move the robot. It only stores the baseline encoder values.
```c
odom_update(&odom, 12000, -12000, 0.01);
```

### 3. Normal Update Loop
```c
odom_update(&odom, left_tick, right_tick, dt);
```

### 4. Get Pose
```c
double x, y, theta;
odom_get_pose(&odom, &x, &y, &theta);
```

### 5. Get Velocity
```c
double v, w;
odom_get_velocity(&odom, &v, &w);
```

### 6. Example
```c
#include <stdio.h>
#include "odometry.h"

int main(void)
{
    odometry_t odom;
    double x, y, theta;
    double v, w;

    odom_init(&odom, 0.05, 0.30, 2048.0, -32768, 32767, false, false);

    /* First update (baseline) */
    odom_update(&odom, 12000, -12000, 0.01);

    /* Rotate in place */
    odom_update(&odom, 12100, -12100, 0.01);

    odom_get_pose(&odom, &x, &y, &theta);
    odom_get_velocity(&odom, &v, &w);

    printf("pose: x=%.6f, y=%.6f, theta=%.6f\n", x, y, theta);
    printf("vel : v=%.6f, w=%.6f\n", v, w);

    return 0;
}
```

---

## License
MIT License