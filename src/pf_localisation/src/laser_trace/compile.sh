#!/bin/bash

g++ laser_trace.cpp -fPIC -O3  -msse4 -shared -I/usr/include/python2.7 -lboost_python-py27 -lpython2.7 -o laser_trace.so

