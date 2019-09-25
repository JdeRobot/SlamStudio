## Installation Instructions:-
### Prerequisites:-
#### 1. Follow [Qt5 installation](https://wiki.qt.io/Install_Qt_5_on_Ubuntu).

#### 2. Install OpenGL
```
sudo apt-get update
sudo apt-get install libglu1-mesa-dev freeglut3-dev mesa-common-dev
```

## Build:
```
git clone https://github.com/JdeRobot/slam-testbed.git
git checkout debugging
cd slam-testbed
mkdir build 
cd build 
cmake ..
make -j4
```

## Run:
```
./slam_testbed
```

## Testing: 

>> File > Open dataset A > select the "original_data.txt" file in data folder.

>> File > Open dataset B > select the "modified_data.txt" file in data folder.

>> Estimator > Estimate Sequence from A to B. 

