# ION GNSS+ 2025 Factor Graph Tutorial
<br>
<br>

This repository contains a set of factor graph examples implemented in python using the GTSAM library.

<br>

### What is Georgia Tech Smoothing and Mapping (GTSAM):
 
- GTSAM is a C++ library (with Python and MATLAB bindings) for factor graphs
- https://github.com/borglab/gtsam
- "BSD-licensed C++ library that implements sensor fusion for robotics and computer vision applications, including SLAM (Simultaneous Localization and Mapping), VO (Visual Odometry), and SFM (Structure from Motion)."
- Developers are actively maintaining and developing the code base.
- Frank Dellaert and Michael Kaess. ``Factor graphs for robot perception.'' Foundations and Trends® in Robotics 6.1-2 (2017): 1-139.

<br>

## Examples:
---

* simple_tutorial:
    - Example 1 (odom only):
        -  Show a simple multi-robot odom solution ( each independent )
    - Example 2 (odom + ranging):
        - Extension of Ex1 with ranging to tie multiple robots to common frame
* gnss_tutorial:
    - Example 1: 
        - Single epoch ( static GNSS localization with pseudorange data )
        - batch estimation
    - Example 2/3:
        - Incorporating dynamics ( multi-epoch GNSS localization with pseudorange data )
        - batch estimation
    - Example 4:
        - Incorporating robust estimation techniques
        - batch estimation
        - additional background on robust estimation in `robust_est_overview.pdf`
    - Example 5:
        - Moving from batch to incremental estimation ( using ISAM2 )

<br>

## Running the software:
---

<br>

```bash
$ docker build -t fg-tutorial:latest .
$ docker run --rm -it -v "$PWD":/app fg-tutorial:latest
$ mkdir plots 
$ python3 examples/uwb_tutorial/odom_only.py 
```

---

### Running inside Docker

Inside the container, either run as a module:
```bash
python -m examples.gnss_tutorial.example_1
```
or run the script path directly (PYTHONPATH is set in the image):
```bash
python examples/gnss_tutorial/example_1.py
```
