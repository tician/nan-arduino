nan-arduino
=====

Some arduino stuff

Contents
-----
- arbotix_accessories
 + Motion
  * mtn2bcpose.py - python script to convert RoboPlus Motion files to header
    files compatible with the arbotix BioloidController library.
  * mtn2rbbbpose.py - slightly expanded version to allow a bit more information
    to be used by the motion engine (like hardcoding servo IDs for each sequence).
- sketchbook - what it says on the tin.
 + libraries - what it says on the tin.
  * Bioloid_uno - modified version of the arbotix ax12 and BioloidController
    libraries to mostly work on the Arduino Uno.
  * dxl - header files for use with dynamixel devices.
  * flyingnun - functions to use a WiiNunchuck with an Arduino (uses Wire).
 + nun\_ros - turns an Arduino Uno + WiiNunchuck into a sensor\_msgs::joy
   publisher matching the ps3 output for one joystick, R1/R2, and accel.
