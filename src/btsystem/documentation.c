/*!

\mainpage Barrett Technology WAM control code library

\section qs Programming Quickstart

To learn to write software for the WAM, Barrett recommends the following course.

- Follow the quickstart sheet shipped with the WAM to get the WAM set up and running.
- Compile and run each of the example programs
- Read the code of the example programs
- Scan through the list of Barrett code library functions
- Read the code for btdiag 

 \section intro Introduction

 The Barrett Technology software library provides the following functionality:

-# Communication with the safety module and motor controllers on the CAN bus using Barrett’s proprietary communications protocol.
-# Automatic recognition puck+motor pairs (actuators) attached to the CAN bus. Simplified command and control of actuators.
-# Initialization and control of 4DOF and 7DOF WAM robots
-# Basic control routines and heuristics (decision making routines). 
 -# Control routines include a PID algorithm and Trapezoidal trajectory generator. 
 -# Heuristics  include a set of functions and state variables to enable, disable, and switch between control methods safely.
 -# Teach and play: recording and playback of trajectories
-# A real-time framework for simplifying the process of insuring that control happens consistently and at a high sample rate.

To best utilize the WAM system the programmer will want to develop some familiarity with all of the above functionality. 
Barrett Technology has provided access to our proprietary code to help the programmer learn about the WAM system and 
eliminate guesswork at what exactly is happening when you call one of our library functions. If you would like to use 
or modify any of the code in the library for yourself; please contact Barrett Technology to find out what licensing 
requirements (if any) we have.

The following is an overview of the library functionality and what code files provide the functionality. 

  \section fctly Barrett Module Functionality

Essential:
- btwam.c: This is the primary file used for controlling the WAM.

Useful funtionality:
- btmath.c: vector and matrix library
- btrobot.c: robot kinematics & dynamics
- btlogger.c: realtime data logging
- btpath.c: space curves for use as trajectories or haptic objects
- btcontrol.c: Control objects. PID, etc.
- btcontrol_virt.c: virtualize objects for btcontrol
- bthaptics.c: simple haptics library
- btjointcontrol.c: jointspace state controller

Mostly internal use:
- btcan.c: CAN bus communication code
- btos.c: OS abstractions for easier porting
- btparser.c: Config file parser
- btsystem.c: Bussed actuators communication & management code
- control_loop.c: realtime thread initialization
- playlist.c: point to point playlist funcionality for btjointcontrol
- serial.c: serial comm library


*/