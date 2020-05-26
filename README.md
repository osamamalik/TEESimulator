# Trans-Esophageal Echocardiography Simulator

* Osama Malik
* EN 4080
* SU2019

## Background

Trans-esophageal echocardiography (TEE) is the standard of care for most cardiac surgeries and interventional cardiology procedures. It uses a thin and flexible ultrasound probe that is inserted down the esophagus in position beside the heart, and the echocardiographer adjusts the position and orientation of the probe from outside. For new clinicians, learning how to adjust the probe into the correct location for various views of the heart can be quite challenging. The primary goal of this project was to develop an open source, inexpensive simulator for training clinicians in the use of TEE ultrasound. A mouse sensor was connected to an Arduino microcontroller to track the motion of a simulated ultrasound transducer. The Arduino communicated the mouse motion to an external computer. The goal was to use these readings to generate real-time CT and ultrasound images.


## Physical Interface

The esophagus is simulated with a tube that is approximately 28 cm long and has a diameter of 7.5 cm. The TEE transducer is simulated with a mouse sensor attached to an Arduino Nano microcontroller. These parts are attached to a thin wooden rod to allow them to be inserted into the tube. The mouse sensor rests on a disc to help with stability.


## Simulation Software

The TEE simulation software consists of three main components. The first is the mouse and Arduino microcontroller subsystem. The mouse sensor is connected to the Arduino, which is plugged into an external computer with microUSB cable. A sketch is uploaded to the Arduino that allows the reading of the mouse sensor’s x and y positions, which are then written to a serial port. The second component of the software enables real-time communication between the Arduino microcontroller and the host computer. This is accomplished by a C++ program that continuously reads the data that was written to the serial port. The third component is the visualization subsystem which displays sample images of a single patient’s chest CT scans. This is accomplished through the use of the Visualization Toolkit library and has been set-up in Xcode.


## Visualization Subsystem

There are 30 CT scans that the program uses. Each scan is in a meta image file format and has a unique thickness in millimeters, which the program takes into account. Numerous calibration experiments revealed 1 mm to correspond to 6 pixels on the screen. As the simulated transducer is inserted into the tube, the mouse sensor’s y value is obtained and the CT image updates to show the next sequential scan. As the transducer is retracted, the scan cycles back to the previous image. Similarly, as the transducer is rotated around the tube, the mouse sensor’s x value is obtained and the CT image is rotated. Calibration revealed the tube’s circumference to be approximately 750 pixels, so 1 degree of rotation is about 2 pixels on the screen. The visualization of the CT scans is in real-time, changing along with the physical position of the simulated transducer.

## Future

Now that the 3D CT simulation of a human chest has been successfully visualized, the next step is to generate ultrasound scans. This will be accomplished by using an existing algorithm that simulates ultrasound images from CT scans.

Beyond this, a visualization of a 3D heart model showing the location of the ultrasound plane will be added to the simulator software.
