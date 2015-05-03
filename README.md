Traffic Light Control System
==============================

INEL4206 - MICROPROCESSORS AND EMBEDDED SYSTEMS
-----------------------------------------------

### Description
Software that controls a corresponding circuit to emulate traffic lights at an intersection. Such traffic lights are represented by LEDs connected in a matrix-style circuit which permits the control of such LEDs by selecting which rows and columns are to be activated at a specific time. Each traffic light includes the three conventional lights (red, yellow, and green), as well as an arrow to turn left (blue LED).  The street running from north to south is considered the main street and the one running from west to east is considered secondary.

Using the internal timer of the MCU, this software is able to analize several factors every second with timed interruptions. These include the current state of the system, the values in the timer registers of the state, and the values stored from the input in the P2 ports, which simulate the presence of a car in that specific side.

### Traffic Lights Interface Diagram:
	    	| Primary Lights   | Secondary Lights |
	    	 _______   _______   _______   _______
	    	|Light 1| |Light 2| |Light 3| |Light 4|
    P1.4----| GREEN | | GREEN | | GREEN | | GREEN |
    P1.5----| YELLOW| | YELLOW| | YELLOW| | YELLOW|
    P1.6----|  RED  | |  RED  | |  RED  | |  RED  |
    P1.7----| BLUE  | | BLUE  | | BLUE  | | BLUE  |
		    |_______| |_______| |_______| |_______|
       ^         |         |         |         |
    [0 = ON]     |         |         |         |
	    	   P1.0      P1.1      P1.2      P1.3  <--[1 = ON]

Built with IAR Embedded Workbench Version: 6.30.1