# Issues while deployment

## 1. Unable to find either executable 'empy' or Python module 'em'

### 1.1. problem description
When `conda activate gazebo_rl` then using `catkin build` to build the project, encountered an build error:
```bash
CMake Error at /opt/ros/noetic/share/catkin/cmake/empy.cmake:30 (message):
  Unable to find either executable 'empy' or Python module 'em'...  try
  installing the package 'python3-empy'
```
But when try:
```
sudo apt-get update
sudo apt-get install python3-empy
```
It shows that `python3-empy` has been installed.

### 1.2. analysis
`python3-empy` is install in the package of system's python(/usr/bin/python), not in `gazebo_rl`'s python. But `catkin build` will use `gazebo_rl`'s python interpreter to build the project which is not install `empy` package.

### 1.3. solution
After activate `gazebo_rl`, run following command:
```bash
pip3 install empy
```

## 2. No module named 'catkin_pkg'

## 2.1. problem description
When `catkin build`, encountered an build error:

```bash
ImportError: "from catkin_pkg.package import parse_package" failed: No module named 'catkin_pkg'
Make sure that you have installed "catkin_pkg", it is up to date and on the PYTHONPATH.
```

## 2.2. analysis & solution
> The reason is the same as above.

```bash
pip3 install catkin_pkg
```
