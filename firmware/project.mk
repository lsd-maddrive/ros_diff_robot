# ROS library
ROSLIB = ./ros_lib
include $(ROSLIB)/ros.mk

PROJECT_CSRC 	= main.c motors.c encoders.c odometry.c
PROJECT_CPPSRC 	= $(ROSSRC) ros.cpp

PROJECT_INCDIR	= $(ROSINC)

PROJECT_LIBS	= -lm

