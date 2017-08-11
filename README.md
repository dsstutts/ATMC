Arduino Temperature Measurment and Control (ATMC)

This project seeks to develop Arduino applications and libraries to enable mechanical engineering students at
Missouri University of Science and Technology to use Arduino hardware to perform common experimental data acquisition and control functions.   The Arduino application, ATMC, measures six temperatures, while controlling one, using the MAX31856 SPI thermocouple interface chip.

ATMC supports the following commands entered at the Arduino input command line:
  A -- Control, acquire and store data
  a -- Stop everything and save data; wait until restart.
  C# -- Run in open loop with dutycycle #.
  c -- Cancel open loop and turn off power.
  h -- List of supported commands
  L -- Log data; use 'a' to stop and save.
  o -- Turn power off
  ss -- Print acquisition rate
  s# -- Set acquisition rate
  s# -- Set data acquisition interval where # is an integer option denoting:
    0 --> .1 sec (not implemented)   
    1 --> .2 sec
    2 --> .3 sec
    3 --> .4 sec
    4 --> .5 sec
    5 --> .6 sec
    6 --> .7 sec
    7 --> .8 sec
    8 --> .9 sec
    9 --> 1.0 sec\
  T# -- Set the x = 0 boundary temperature to # degrees C (currently not implemented) 
  thms -- set time where h,m, and s are hours, minutes, seconds integers 
  ty## -- set year where ## are the last two digits 
  tr## -- set month where ## are the digits of the month 
  td## -- set day 
  tt -- Show the current time.
  gg -- Report current PID gains.
  gp#i#d# -- Set PID gains, where # denotes floating point numbers.
  Note that while you may set one or two at a time, you must enter the PID gains and times in order!
  W#### -- Simultaneously set duty cycle and log data, where #### represents the duty cycle from 0 to 1023.
  Z -- Not implemented.
