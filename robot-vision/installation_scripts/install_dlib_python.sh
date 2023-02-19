cd ~/Downloads/dlib-19.17
python setup.py install
# clean up(this step is required if you want to build dlib for both Python2 and Python3)
rm -rf dist
rm -rf tools/python/build
rm python_examples/dlib.so