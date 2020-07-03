# Arduino-Drone
Drone controlled by an Arduino nano using a PID controller.

## On Board Software
For the PID, I wrote an Arduino library using <a href="http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/">this page</a>, by Brett Beauregard as reference.
I also used <a href="https://github.com/jrowberg/i2cdevlib.git">theese libraries</a> by <a href="https://github.com/jrowberg">jrowberg</a> to dialog with the MPU6050, and <a href="https://github.com/aanon4/FlySkyIBus.git">this library</a> by <a href="https://github.com/aanon4">Tim Wilkinson</a> to read the I-BUS data from the RC receiver.
For controlling the brushless motors I wrote my own little library.

My libraries are available on <a href = "https://github.com/Miguel-San/Arduino-Libraries">my Arduino Libraries repo</a>.

## Electronics
There are more connections on the gerber files than on the schematic. This is for flexibility with the PCB as this is a prototype.

## Ground Control
The script it's on a very early stage of development and there are bugs everywhere. Be carefull if you use it.

## Workstation
To tune and test the PID I made a support for the drone using and old camera tripod. In the future I will try to make another Ball_and_Base part for more flexibility. For reference, the ball diameter in the ball joint is 37mm. 
