Arduino Temperature Measurment and Control (ATMC)

This project seeks to develop Arduino applications and libraries to enable mechanical engineering students at Missouri University of Science and Technology to use Arduino hardware to perform common experimental data acquisition and control functions.   The Arduino application, ATMC, and its variants, can measure up to 18 temperatures, while controlling up to three, using the MAX31856 SPI thermocouple interface chip for measurement, and PID-controlled pulse width modulation (PWM) for boundary temperature control.  ATMC and its variants can also control the boundary flux by fixing the 
PWM duty cycle.

The following is currently what is returned by entering h on the command line:
[We are in the process of cleaning up this code, so this will change!]
I support the following commands:\r\n \\
  A -- Control, acquire and store data\r\n \\
  a -- Stop everything and save data; wait until restart.\r\n \\
  h -- List of supported commands\r\n \\
  L -- Log data; use 'a' to stop and save.\r\n \\
  o -- Turn power off\r\n \\
  ss -- Print acquisition rate\r\n \\
  s# -- Set data acquisition interval where # is an integer option denoting:\r\n \\
    0 --> .1 sec (not implemented) \r\n \\
    1 --> .2 sec\r\n \\
    2 --> .3 sec\r\n \\
    3 --> .4 sec\r\n \\
    4 --> .5 sec\r\n \\
    5 --> .6 sec\r\n \\
    6 --> .7 sec\r\n \\
    7 --> .8 sec\r\n \\
    8 --> .9 sec\r\n \\
    9 --> 1.0 sec\r\n \\
    10 --> 4 sec\r\n \\
  T# -- Set the x = 0 boundary temperature to # degrees C, turn power on, and log data\r\n \\
  q# -- Set the boundary temperature to # degrees C\r\n \\
  th#m#s# -- set time where h,m, and s are hours, minutes, seconds and #, integers \r\n \\
  ty## -- set year where ## are the last two digits \r\n \\
  tr## -- set month where ## are the digits of the month \r\n \\
  td## -- set day \r\n \\
  tt -- Show the current time.\r\n\ \\
  gg -- Report current PID gains.\r\n \\
     gp#i#d# -- Set PID gains, where # denotes floating point numbers.\r\n \\
     Note that while you may set one or two at a time, you must enter the PID \
     gains and times in order!\r\n \\
z -- Monitor data by printing to the serial monitor -- toggle logic.\r\n\ \\
W#### -- Simultaneously set duty cycle and log data, where #### represents the \
duty cycle from 0 to 1023.\r\n