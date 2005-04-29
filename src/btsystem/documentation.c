/*!

\mainpage Barrett Technology WAM control code library

 \section intro Introduction

NOTE: this document has not been updated since 2003
 
 The Barrett Technology software library provides the following functionality.

-# Communication with the safety module and motor controllers on the CAN bus using Barrett’s proprietary communications protocol.
-# Automatic recognition puck+motor pairs (actuators) attached to the CAN bus. Simplified command and control of actuators.
-# Initialization and control of 4DOF and 7DOF WAM robots
-# Basic control routines and heuristics (decision making routines). 
 -# Control routines include a PID algorithm and Trapezoidal trajectory generator. 
 -# Heuristics  include a set of functions and state variables to enable, disable, and switch between control methods safely.
 -# Teach and play: recording and playback of trajectories
-# A real-time framework for simplifying the process of insuring that control happens consistently and at a high sample rate.

To best utilize the WAM system the programmer will want to develop some familiarity with all of the above functionality. Barrett Technology has provided access to our proprietary code to help the programmer learn about the WAM system and eliminate guesswork at what exactly is happening when you call one of our library functions. If you would like to use or modify any of the code in the library for yourself; please contact Barrett Technology to find out what licensing requirements (if any) we have.

The following is an overview of the library functionality and what code files provide the functionality. 

The pucks in the WAM are controlled through a serial CAN bus. btcan.c provides a minimal set of functions for communicating with the WAM pucks using the PCI CAN communications card that shipped with your WAM. For most WAM programming, communications to the WAM pucks is handled by functions defined in btsystem.c and btwam.c but the programmer should still develop a basic knowledge of btcan.c.

 btsystem.c provides functions for keeping track of and working with a set of actuators. An actuator is the combination of a motor and its motor controller. The actuator data structures maintain information on calibration values for a specific motor/controller pair and handles conversion to and from engineering units. btsystem.c does not assume any kinematic structure or physical organization. btsystem.c maintains a database of actuators that is stored in configuration files. It simplifies communication with actuators to insulate the programmer from the btcan.c functions which are more complex.

 The pucks apply torques commanded by the PC and reads the angle of the motor. The  control loop of the WAM must be closed by the QNX computer attached to the WAM. This requires consistent and deterministic timing. QNX  is a real-time operating system well suited to this type of task. There are many approaches to closing the time-critical control  loop. Barrett Technology has provided one solution to help jump-start the programmer. These timing functions are contained in control_loop.c. Any program you write for the WAM must have a function to implement the control thread. Typically you will start by copying the function we provide and modifying it as necessary. The name of this function is passed to the start_control_threads() function which will take care of registering your function with the timing functions so that your control thread is called regularly.

 In addition to the timing functions provided by control_loop.c, Barrett Technology has also provided a series of functions for providing simple motion control. SimpleControl.c defines the SimpleCtl structure, has functions for PID regulators, Trapezoidal trajectory profiles, and thread-safe ways to switch control states on the robot. These functions are generic, one-dimensional and meant to be used not just in our control functions but whenever the mathematics of these algorithms are needed in your own code.

 playlist.c provides functionality for recording and playing back multiple trajectories for multiple SimpleCtl controllers. doublebuffer.c  provides some helper functions for highspeed data logging but will require some modification on the part of the programmer for your specific application.

 btwam.c brings together all of the above libraries into a set of high-level commands. btwam.c does not duplicate any functionality provided by the underlying layers. Instead it is meant to save time for simple projects. btwam.c provides functions for homing the robot and moving the joints to specific positions.  btwam.c also provides the wam_vector data structure and mathematical functions for converting positions and torques between motor space and joint space. See Appendix B in the WAM system manual for details on these calculations.

 The Barrett Software library maintains a set of configuration files. They must be located in the directory of the executable. Appendix A gives the details of each file format. They are as follows:
  -# buses.dat : Database of communications busses used.
  -# motors.dat : Database of motor serial numbers and calibration values
  -# pucks.dat : Database of puck serial numbers and calibration values
  -# actuators.dat : Specifies the number of actuators in your system and which motors are paired with which pucks and which bus that puck is on.
  -# wam.dat : Specifies wam specific information including home positions, PID gains, and which actuators are connected to which robot joint. 
  -# motor##.tr : One file for each motor where ## is replaced  by the motor serial number. These files contain the torque ripple cancellation data.

*/