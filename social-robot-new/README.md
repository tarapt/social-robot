# social-robot
## Installation Instructions
1. Install Ubuntu 18.04.2. Use acpi = off, to prevent freezing.
2. Add proxy script to .bashrc.
3. Install Nvidia Driver 415.
4. Install Cuda 10.0 from runfile.
5. Install v4l-utils.
   -  sudo -E apt install v4l-utils
6. Install git, GitKraken.
7. Install Zed SDK(requires CUDA).
8. Setup python libraries and virtual environments.
   1. Install pip, virtualenv, virtualenvwrapper.
   2. Add ~/.local/bin folder to path, which contains user specific python libs.
   3. Create a Python 3.6 Virtual Environment called zed.
   4. Create a Python 2.7 Virtual Environment called arnl.
   5. Install flask, cursed, requests, Pillow, imutils, face_recognition from pip.
   6. Install pep8, autopep8.
9.  Setup VSCode.
    1.  Install extensions - Markdown All in One, Python, TODO Highlight, Todo+.
    2. Select Python Interpreter.
    3. Make sure the paths in the file .vscode/c_cpp_properties.json are correct.
    4. Set Http Proxy in Preferences: http: // 172.31.2.4: 8080
    5. Install pylint, it will auto suggest on opening a python file.
        - zed: / home/tara/.virtualenvs/zed/bin/python - m pip - -proxy http: // 172.31.2.4: 8080 install - U pylint
        - arnl: / home/tara/.virtualenvs/arnl/bin/python - m pip - -proxy http: // 172.31.2.4: 8080 install - U "pylint<2.0.0"
10. Install dlib without CUDA support.
    1. sudo - E apt-get update
    2. sudo - E apt-get install build-essential cmake
    3. sudo - E apt-get install libopenblas-dev liblapack-dev
    4. sudo - E apt-get install libx11-dev libgtk-3-dev
    5.  sudo - E apt-get install python python-dev python-pip
    6.  sudo - E apt-get install python3 python3-dev python3-pip
    7.  pip install numpy
    8.  pip install dlib
11. Install OpenCV 4.0 without CUDA support.
    - python - m pip install opencv-contrib-python
12. Install MobileRobots softwares.
    1.  Aria
        -  sudo dpkg - i libaria_2.9.1+ubuntu16_amd64.deb
        -  sudo dpkg - i libaria-python_2.9.1+ubuntu16_amd64.deb
    2.  BaseArnl
        -  sudo dpkg - i arnl-base_1.9.2+ubuntu16_amd64.deb 
        -  sudo dpkg - i arnl-base-python_1.9.2+ubuntu16_amd64.deb
    3.  Arnl
        -  sudo dpkg - i libarnl_1.9.2+ubuntu16_amd64.deb 
        -  sudo dpkg - i libarnl-python_1.9.2+ubuntu16_amd64.deb
    4.  Sonarnl
        -  sudo dpkg - i libsonarnl_1.9.2+ubuntu16_amd64.deb
        -  sudo dpkg - i libsonarnl-python_1.9.2+ubuntu16_amd64.deb 
    5.  MobileSim
        -  sudo dpkg - i mobilesim_0.9.8+ubuntu16_amd64.deb
    6.  MobileEyes
        -  sudo dpkg - i mobileeyes_2.2.4-1_i386.deb
        -  sudo - E apt --fix-broken install
    7.  Mapper3
        -  sudo dpkg - i mapper3_2.2.5-1_i386.deb
        -  sudo - E apt --fix-broken install
    8.  Add the python libs to PYTHONPATH.
        - export PYTHONPATH ="$PYTHONPATH:/usr/local/Arnl/python"
13. Install Matlab
    - Install the following packages: 
    - Install Matlab Engine for Python
14. Install OpenPose without CUDA support.
    -  Add the python libs to PYTHONPATH.
        - export PYTHONPATH ="$PYTHONPATH:/home/tara/Project/openpose/build/python"

## To use stereo cameras
Type v4l2-ctl --list-devices to see the indices of the cameras
    
    Example:
    $ v4l2-ctl --list-devices
        HD Webcam: HD Webcam (usb-0000:00:14.0-11):
            /dev/video0
            /dev/video1

        UVC Camera (046d:0825) (usb-0000:00:14.0-3):
            /dev/video4
            /dev/video5

        UVC Camera (046d:0825) (usb-0000:00:14.0-8):
            /dev/video2
            /dev/video3

    Use index 2 and 4 for the cameras.

## To get the IP address of the host computer
    $ hostname -I
    
## Install matlab engine for python in virtual environment
1. Find out, where you have matlab installed. This can be done by runnign command 'matlabroot' in Matlab. This location is later referenced as matlabroot. (such  as /usr/local/MATLAB/R2018b/)
2. Make sure you have your virtualenv activated.
3. With command-line interface run the following commands (Might need root priviledges), where matlabroot is changed to location of matlab intallation:
```
    $ cd "matlabroot\extern\engines\python"
    $ python setup.py install
```