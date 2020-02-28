---
permalink: /installation/
title: "Installation"

toc: true
toc_label: "TOC installation"
toc_icon: "cog"
---

We have tested the library in Ubuntu 18.04, but it should be easy to compile in other platforms.
SLAM-Testbed works under Qt5 to create the user interface, so it's multiplatform. Below are the steps for installing the software.

## Ubuntu
### Dependencies

#### QT5
We use QT5 to create the user interface. Download and install instructions can be found at: [Qt5 installation](https://wiki.qt.io/Install_Qt_5_on_Ubuntu). Or 
```
sudo apt-get update
sudo apt-get install qt5-default
```

#### OpenGL
You need to ensure that you have alredy installed OpenGL. If not, you can install it following these instructions:
 
```
sudo apt-get install libxmu-dev libxi-dev cmake
sudo apt-get install libglu1-mesa-dev freeglut3-dev mesa-common-dev
```


### Installation

Follow the next steps:

1. Download the software.

    ```
    git clone https://github.com/JdeRobot/slam-testbed
    ```

2. Build and compile.
    Move to the directory where you have the clone repository and compile following these commands:

    ```
	  mkdir build
	  cd build
	  cmake ..
	  make -j
    ```

3. Run the application:
	Run the application from 'Build' directory:

	```
	./slam-Testbed
	```


