# social-robot
TODO: Decide which package to be installed in which environment.
## Installation Instructions
1. Install Ubuntu 18.04.2. Use acpi = off, to prevent freezing.
2. Add proxy script to .bashrc.
3. Install Nvidia Driver 415.
4. Install Cuda 10.0 from runfile.
5. Install cuDNN, TensorRT from deb files.
6. Install tensorflow-gpu in virtual environment for faced package for face recognition.
   - pip install tensorflow-gpu
   - [Don't, accuracy is bad, go for https://medium.com/wassa/modern-face-detection-based-on-deep-learning-using-python-and-mxnet-5e6377f22674] Install faced
     - Don't do this if you want tensorflow-gpu: pip install git+https://github.com/iitzco/faced.git
     - git clone https://github.com/iitzco/faced.git
     - change requirements.txt to use tensorflow-gpu
     - python setup.py build
     - python setup.py install
7. https://medium.com/wassa/modern-face-detection-based-on-deep-learning-using-python-and-mxnet-5e6377f22674
   - Install mxnet from http://mxnet.incubator.apache.org/versions/master/install/index.html
   - pip install mxnet-cu92
     - Requires cuda-9.2.
   - Alternatively use MTCNN for Tensorflow, https://github.com/ipazc/mtcnn
8. Install v4l-utils.
   -  sudo -E apt install v4l-utils
9.  Install git, GitKraken.
10. Install Zed SDK(requires CUDA).
11. Setup python libraries and virtual environments.
   1. Install pip, virtualenv, virtualenvwrapper.
   2. Add ~/.local/bin folder to path, which contains user specific python libs.
   3. Create a Python 3.6 Virtual Environment called zed. 
      1. Install zed-python-api
         1. Install cython, numpy.
         2. Clone zed-python-api.
         3. python setup.py build
         4. python setup.py install
      2. pip install PyOpenGL opencv-python
   4. Create a Python 2.7 Virtual Environment called arnl.
      1. Install flask, cursed, requests, Pillow, imutils, face_recognition from pip.
   5. Install sklearn for face recognition.
12. Setup VSCode.
    1. Install extensions - Markdown All in One, Python, TODO Highlight, Todo+.
    2. Select Python Interpreter.
    3. Make sure the paths in the file .vscode/c_cpp_properties.json are correct.
    4. Set Http Proxy in Preferences: http: // 172.31.2.4: 8080
    5. Install pep8, autopep8(for auto formatting), rope(for refactoring)
    6. Install pylint, it will auto suggest on opening a python file.
        - zed: / home/tara/.virtualenvs/zed/bin/python - m pip - -proxy http: // 172.31.2.4: 8080 install - U pylint
        - arnl: / home/tara/.virtualenvs/arnl/bin/python - m pip - -proxy http: // 172.31.2.4: 8080 install - U "pylint<2.0.0"
13. Install dlib with CUDA support.
    -  See the install script.
14. Install OpenCV 4.0 without CUDA support.
    - python - m pip install opencv-contrib-python
15. Install MobileRobots softwares.
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
16. Install Matlab
    - Install the following packages: 
    - Install Matlab Engine for Python
17. Install Tkinter and a GUI builder
    - sudo -E apt-get install python3-tk
    - sudo -E apt-get install python-tk
    - pip install pygubu
    - pip3 install pygubu
18. Install OpenPose without CUDA support.
    - Add the python libs to PYTHONPATH.
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

## dpkg lock issues
```
E: Could not get lock /var/lib/dpkg/lock-frontend - open (11: Resource temporarily unavailable)
E: Unable to acquire the dpkg frontend lock (/var/lib/dpkg/lock-frontend), is another process using it?
```
Run the following commands to fix it:
```
sudo rm /var/lib/apt/lists/lock
sudo rm /var/cache/apt/archives/lock
sudo rm /var/lib/dpkg/lock*
sudo dpkg --configure -a
sudo -E apt update
```