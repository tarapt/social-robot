sudo -E apt-get install build-essential cmake pkg-config
sudo -E apt-get install libx11-dev libatlas-base-dev
sudo -E apt-get install libgtk-3-dev
sudo -E apt-get install libboost-all-dev

# Download dlib from github, unzip, change the version number below if required

cd dlib-19.17/
mkdir build
cd build
cmake ..
cmake --build . --config Release
sudo make install
sudo ldconfig
cd ..

pkg-config --libs --cflags dlib-1

# Create a virtual environment: mkvirtualenv py3_dlib -p python3
workon py3_dlib

cd dlib-19.17
python setup.py install
# clean up(this step is required if you want to build dlib for both Python2 and Python3)
rm -rf dist
rm -rf tools/python/build
rm python_examples/dlib.so