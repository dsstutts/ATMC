Arduino Temperature Measurment and Control (ATMC)

This project seeks to develop Arduino applications and libraries to enable mechanical engineering 
students at Missouri University of Science and Technology to use Arduino hardware to perform common 
experimental data acquisition and control functions.   The Arduino application, ATMC, and its variants, 
can measure up to 18 temperatures, while controlling up to three, using the MAX31856 SPI thermocouple 
interface chip for measurement, and PID-controlled pulse width modulation (PWM) for 
boundary temperature control.  ATMC and its variants can also control the boundary flux by fixing the 
PWM duty cycle.

I recently (10/27/2025) added a subdirectory MultiPoint_Harmonic_Method containing both the Aduino 
and Python analysis codes, along with an example data file.  
At present, the analysis code definitely works, but will require some setup on your 
computer to work for you without modification.  First of all, you'll need all of the 
required Python modules, and as presently configured, a working installation of LaTeX, 
because I use LaTeX to format symbols on the matplotlib-generated figures.  
This requirement can be removed, and you're welcome to fork a branch to do so!  

To implement the Transient Method described in Tomanek, Lauren B. and Stutts, Daniel S.
"Material thermal properties estimation via a one-dimensional transient convection model"
2021 Applied Thermal Engineering, Vol. 184 Elsevier
p. 116362 

and 

Tomanek, Lauren B. and Stutts, Daniel S.
"Data on the Validation to Determine the Material Thermal Properties Estimation Via a One-Dimensional 
Transient Convection Model," 2022, Data in Brief , Vol. 40, p. 107632,

Use ATMC_SixTCBoard_SimplifiedCode.ino found in SimplifiedCode/    

The following is currently what is returned by entering h on the command line:
  a -- Stop everything and save data; wait until restart.
  h -- List of supported commands
  L -- Log data
  ss -- Print acquisition rate
  s# -- Set data acquisition interval where # is an integer option denoting:
    0 --> 0.1 sec
    1 --> 0.2 sec
    2 --> 0.3 sec
    3 --> 0.4 sec
    4 --> 0.5 sec
    5 --> 0.6 sec
    6 --> 0.7 sec
    7 --> 0.8 sec
    8 --> 0.9 sec
    9 --> 1.0 sec
    10 --> 4.0 sec
  W#### -- Simultaneously set duty cycle and log data, where #### represents  
the duty cycle from 0 to 1023.

This code is about to be deprecated and replaced by a more platform independent version.
This version only works on the Arduino MEGA.
