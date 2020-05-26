# Trans-Esophageal Echocardiography Simulator

* Osama Malik
* EN 4080
* SU2019
* Project Supervisor: Dr. Burton Ma

# Design Document

The software architecture the simulator software system consists of three interconnected subsystems: the Arduino microcontroller and mouse subsystem; the real-time communication subsystem between the Arduino microcontroller and the host computer; and the visualization subsystem.


## Arduino Microcontroller and Mouse Subsystem

This subsystem consists of an Avago ADNS-9800 High Performance LaserStream Gaming Sensor and an Arduino Nano microcontroller board. The mouse sensor has been modified to be 5V compatible by removing the traces between the three sets of exposed pads on the board’s 3.3V side and soldering bridges on the pads of the 5V side. 

Its pinout details are:
<ul>
  <li>MI = MISO</li>
  <li>MO = MOSI</li>
  <li>SS = Slave Select / Chip Select</li>
  <li>MOT = Motion (active low interrupt line)</li>
  <li>AG = Analog Ground (connect to common ground near power supply)</li>
  <li>DG = Digital Ground (connect to common ground near power supply)</li>
  <li>VI = Voltage in up to +6V</li>
</ul>

Its dimensions are:
<ul>
  <li>Diameter: 31.5mm</li>
  <li>Screw holes: 27mm between centers</li>
  <li>Screw hole size: 2.26mm (fits 2-56 screw)</li>
  <li>Height: 9.7mm</li>
</ul>

It is attached to the Arduino with the wire configuration provided from https://www.tindie.com/products/jkicklighter/adns-9800-laser-motion-sensor/

A microUSB cable is attached to the Arduino and connected to the host computer. Using the Arduino desktop client, a sketch is uploaded to the Arduino. The sketch has been obtained from the manufacturer of the modified mouse sensor, found at https://github.com/mrjohnk/ADNS-9800, in particular the ADNS9800testPolling.ino and ADNS9800_SROM_A4.ino files found in the Arduino Example Sketches folder, which obtain the x and y positions of the mouse sensor. The ADNS9800testPolling sketch has been modified so that the x- and y- positions are scaled and relative to the starting point. These values are printed to the Serial port in the following format:
	x y!
where x is the x position number, y is the y position number, and ! indicates the end of the coordinate. The TEE simulator program then reads the values.



## Real-Time Communication Subsystem Between Arduino and Host Computer

The program makes use of a serial class which reads in the data written to the Serial port by the Arduino. This data is then parsed and saved into x and y variables. In the program’s main method, a TimerCallbackFunction is set up using the vtkProgrammableFilter and vtkCallbackCommand classes. The callback function is repeatedly invoked and checks for the mouse sensor’s current position. This is done by reading the serial data mentioned previously. If invoked frequently enough, the effect of real-time communication between the Arduino and host program is achieved.


## Visualization Subsystem

The visualization subsystem utilizes the Visualization Toolkit (VTK) library. When the program is first run, an initial scan is displayed. The initial scan is the first CT scan of 30 (obtained from the EMPIRE10 grand challenge) that can be displayed using the vtkMetaImageReader class. An actor consisting of this image is created and added to the vtkRenderer. When the previously mentioned TimerCallbackFunction is invoked, the mouse sensor’s position is updated and new x and y values are obtained. The scan that is displayed is then changed depending on what the x and y readings of the mouse sensor are. The y value corresponds to CT scan’s z value or thickness. It determines how many pixels to display a particular scan for. The x value corresponds to the CT scan’s angle of rotation. The rotation is achieved by obtaining the center of each individual CT scan, translating the image so the center is at the origin (0,0,0), rotating the image by the calculated degrees, and then translating the image back. The image actor is then updated the displayed CT scan is rotated on the display.
