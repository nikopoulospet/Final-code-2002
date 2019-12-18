Worcester Polytechnic Institute
Robotics Engineering Program


Team 12: Final Report

Submitted By
Brian Katz
Peter Nikopolous
Ryan O’Brien
Sawyer Wofford






Date Submitted	: 12/13/19
Date Completed	: 12/13/19
Course Professors	: Professor Lewin
Lab Section	: Lab Section 1











Table of Contents

Table of Contents	2
Introduction and Overview	3
Solutions and Justifications	4
Figure 1: CAD Rendering of BaseBot	5
Figure(s) 2-6: CAD Renderings of BaseBot	7
Figure 7: Complementary Filter Code	8
Figure 8: HC-SR04 Ultrasonic Sensor	11
Figure 9: Active Bandpass Filter	12
Figure 10: Peak Detector	13
Figure 11: Schmitt Trigger	14
Figure 12: Building Side Encoding Values	16
Figure 13: Street Grid with Plot Encoding	17
Table 1: Beacon Position Encoding	18
System Integration	19
Figure 14: IR Detector Sensor Block Diagram	21
Performance	21
Appendix A: Sensors Data and Code	23
Figure 15: Maxbotics Ultrasonic Sensor Experimentation Results	23
Figure 16: Sharp IR Distance Sensor Accuracy and Trendline Equation Results	24
Appendix B: Filter Calculations	25
Appendix C: Schmitt Trigger Calculations	26
Appendix D: Morphological Chart	26
Appendix E: State Diagrams	29











Introduction and Overview

	For the RBE2002 B19 final project, we must autonomously navigate around a field arranged like a grid of streets (representing Gotham) and find the location of Robin within a building (represented as an IR beacon flashing at 1 kHz). We must then report Robin’s location, provide him with some mechanical means of escape and return back to our starting location. There are some constraints on the project which are that the entire challenge must be completed within 5 minutes, the robot must fit within a 35 cm cube at its starting position, and if we choose to remove our mechanical means of escape from our robot in the process of rescuing Robin, we can immediately navigate back to the start rather than waiting for a 30 second penalty. In addition, there are certain areas of the field that are restricted, such as plots where buildings could be located and the areas surrounding a roadblock that can only be located between two plots.
	While we were given a BaseBot at the beginning of the class to help us with the navigation of the field, we had to integrate much more into the robot to fulfill the requirements of the challenge. Our robot needed multiple sensors in order to sense buildings and Robin’s beacon, such as ultrasonic sensors and IR detectors. We also used many other components for integration, such as a servo move this IR detector around so we can detect if the beacon is in any position around us. In addition, to ensure that our robot navigated the course reliably, we used a combination of an IMU and motor encoders to drive our robot to precise positions while keeping the heading straight. 

	Due to the time limit on the challenge, we had to integrate a driving algorithm that was both precise and efficient. The size constraints limited us vertically so our escape mechanism needed to actuate in order to reach the windows. We mounted a ladder that laid flat on the robot to a servo and rotated it to hook onto the lower lip of the window. The robot could then drive away which would pull the ladder out of its housing and leave it hanging on the window. This allowed us to complete this part of the challenge easily while avoiding the time penalty for not having a detaching mechanism.
Solutions and Justifications

	One of the first things we did to plan out our goals for this project was to create a list of functional requirements and then integrate those into a morphological chart. As shown in Appendix D, our morphological chart outlined our functional requirements and then expanded upon those by listing possible solutions for each function as well as the reasons for and against each solution.

	The first function we thought of was for our robot to traverse the field. This can be done in several ways, whether it be through the provided BaseBot drivetrain, using an omnidrive, or purchasing and implementing a quadcopter. Immediately, we were able to rule out the quadcopter due to its increased complexity over the other two ideas. An omnidrive has many clear advantages, such as having the ability to travel in any cardinal direction as well as less error since it is much easier to travel in straight lines with an omnidrive. Our other option would then be the BaseBot, which was sufficient enough for the purposes of our project. The BaseBot features a two wheel drive system, with a caster in the back. 

Figure 1: CAD Rendering of BaseBot

	To decide between the two, we weighed the costs and benefits of these two systems, focusing on the time we would need to spend to develop them. Since this function was not one of our main priorities, we decided that we would take the option that takes less time and less effort and allocate more time to our other functions. In this case, the BaseBot was chosen due to us having the most experience with it. One of the main factors we considered in this decision was the tuning of PID coefficients, which took us a very long time to complete on our BaseBot. Since the omnidrive would require us to tune two additional motors, we immediately decided against this and decided on the BaseBot. Also, the turnaround time to design, develop, and implement a new base to our robot would surpass the time remaining in the term to complete the project. While we were sacrificing crucial functionality that could have made other problems not appear in our final implementation. Our final execution of the BaseBot included the original design as well as the addition of several sensors and mechanisms that were added on top of the robot with ease due to the Basebot’s many holes on the top and bottom plates. 






Figure(s) 2-6: CAD Renderings of BaseBot
	The next function of our robot is that it must be able to navigate only on the streets. To clarify, this means that our robot must be able to know its heading and position such that it can only turn in intervals of 90 degrees while staying within the bounds of the streets. To accomplish this feat, we debated between the options of integrating an IMU or using encoders. As we discovered in Lab 4, a combination of these two solutions was the optimal choice. 

This way, we could prioritize our IMU data while changing our heading and our encoder data for distances. Some of the early analysis for this combined system was done mainly in Lab 4. In this lab, we had to navigate a course with high precision by using the IMU and encoders. We discovered that the use of a complementary filter aided us significantly in our endeavors, and allowed us to complete the challenge reliably. We experimented heavily on this matter, picking certain percentages and settled on a 99% IMU to 1% encoder ratio for our heading error control and only used the encoders for distance control. Doing so required us to implement more P controllers, and this needed more tuning of coefficients to ensure our robot corrected itself to the direction we told it to go while understanding the distance it needed to travel. The final implementation of this system remained the same after Lab 4, and still uses the multiple P controllers to handle heading and distance while using a complementary filter to weigh IMU data over the encoder data.
Figure 7: Complementary Filter Code

These sensors also fulfill another function of estimating our position on the field. Since we receive heading data from the IMU and distance data from the encoders, we can understand where we are on the field as well as where we are heading. This allows us to create code that will follow each square on the field and then determine which coordinate we are travelling to using a combination of heading and distance data.

	Another function of our robot is avoiding collisions with stray objects. This implies the presence of a roadblock as well as avoiding crashing into any of the buildings or open plots. This is a function that is mostly sensor-based, thus our possible solutions were ultrasonic sensors, IR distance sensors, and limit switches. We quickly ruled out limit switches due to having the possibility of coming into contact with any of the buildings and thus focused on long-range sensing. In order to weight the ultrasonic versus the IR sensor, we used our previous experience with the two sensors as a gauge for which was better suited for our project. 
We experimented with these sensors during Lab 2, where we wrote code for them and examined their downsides and benefits. This function took a lot more debating because we struggled with getting consistent readings from the ultrasonic during lab and found the IR sensor to work incredibly well. The IR sensor was incredibly consistent with long distances, but not so great at managing short distances that were needed to scan the roadblock as we come down a street. In addition, it was much easier to interpret as it followed an almost linear trendline, as show in the Appendix below. However, we determined that there was a possibility of IR noise due to the flashing IR beacon and thus decided against this sensor. This still left us with the inconsistency of the ultrasonic sensor found in Lab 2. We decided that we could still use an ultrasonic sensor, but instead of using the Maxbotics sensor, we would use an HR SC04 Ultrasonic Sensor, which features two ultrasonic blasters and can use very similar code to our last sensor. To make this sensor function, we measured the pulse-width of the sensor, and used a conversion factor found in the datasheet in order to convert the width of the pulses into a distance in mm. After some initial testing with this sensor, we determined that it was much more consistent and did not include an incredibly large deadband, another problem we found with the Maxbotics sensor.

Our final implementation mechanically required some modeling and used this sensor mounted beneath our robot to find the location of any objects that are in our way. This sensor will be used when we travel down a street and determines if there is a roadblock in the way of our motion.

Another function with the exact same solutions is locating the presence of buildings. Since this function is incredibly similar to the last, we decided to implement another ultrasonic sensor to keep us from writing more code and allow us to debug easier in the future. To implement this into our system, we mounted the ultrasonic sensor above our top plate to pulse centered on the plane between the two windows of the buildings. This will be mounted on a servo (with the IR detector) and will pivot to allow sensing of buildings (and beacon scanning) on both sides of the robot as we travel along the field. 

In order for us to interpret the ultrasonic data, we had to represent each building location as a certain range of values that the ultrasonic can see. When we first began testing the ultrasonic, we relied on a single ping to determine where a building was and generated our ranges from here. However, this was incredibly inconsistent and thus we moved to taking an average over a small period of time. This poses a few problems, as the ultrasonic sensor becomes inconsistent when reading the back row, and thus we decided to only have dedicated ranges for the buildings in the first two rows. This means that the only building that we cannot scan is the building in the bottom right-hand corner of the field. If the robot scans a building in the first or second row of the field, it does not know if there is a building behind the one it is scanning. Temporarily, we store the locations of the buildings behind the one being scanned as confirmed buildings as well. Our ultrasonic happens to return an average of a building in the second row when there is no building. To avoid confusion and marking an incorrect building, we also keep track of a maximum ultrasonic value which can differentiate between a building in the second row and not having a building at all. Therefore, when we find a map of the buildings, there is a possibility that there may be 3, 4, or 5 theoretical buildings depending on the locations of the buildings. 

The road block also adds some uncertainty in our scanning, as our ultrasonic will see the tip of the roadblock while driving by. Therefore, we must be able to differentiate between a roadblock and a building. We created more ranges and then pass in the roadblock into the same map while using the same scanning algorithm mentioned previously.

Figure 8: HC-SR04 Ultrasonic Sensor
One of the most important functions of our robot is the ability to detect Robin’s IR beacon. This function was to be done using the provided IR detector-emitter pair to find the emitter when flashing at 1 kHz. Other emitters will be present on the field, one with DC and one at 5 kHz, so a bandpass filter is needed in order to differentiate the beacon. We learned in class that this required very specific circuitry in order to end up with a useful output that could be detected in code. We decided that it would be best to end with a Schmitt trigger so we could directly set an interrupt off the circuit when the beacon is detected. In order to do this, however, a peak detector is needed to hold the amplitude of the filter output. The three stage circuit, bandpass filter into peak detector into Schmitt trigger, was provided as the best solution and followed material covered in class. 

While the three components themselves were an easy decision, the design of each one was a more involved process. First the bandpass filter had to be designed. We initially attempted to make a narrow band filter following the example in Practical Electronics pg 673 only to find that the values chosen did not actually require a narrow band. Instead, highpass and lowpass filter were just cascaded with an op amp between to amplify. This was much simpler both to calculate and design. From the calculations shown in Appendix B we ended up with a designed center frequency of 954Hz and a bandwidth of 386Hz. Since it is known the only other signals will be at considerably different frequencies, having that wide of a band is perfectly fine. The final circuit is shown in Figure vji.

Figure 9: Active Bandpass Filter

	The second component, a peak detector, is much simpler and had fewer design considerations. A simple peak detector consists of only a diode and capacitor and will hold the highest value it receives for as long as the capacitor can hold charge. We needed one in order to measure the maximum value of the filter’s output. This was tested with a 1uF, 10uF, and 100uF capacitor and we found that the larger capacitors took too long to charge and could miss seeing the beacon. Adding a resistor in parallel allowed the capacitor to discharge over time and also required tuning. Smaller resistors reduced the voltage output but also increased the discharge time, which is approximately 5RC . It would take 5 seconds to fully discharge the capacitor but we never charged it fully. In testing, it took about 2 seconds for the peak detector to decay below the detection threshold. 

We tried to use the largest resistor possible to avoid diminishing the already small voltage range we were working with. The diode required about 500mV to forward bias which only further diminished the input voltage. We found that the slower discharge made it slightly easier to detect the beacon while moving quickly. If we only got a brief glimpse of the beacon, fail to pass directly by it and get another brief glimpse while driving away, each glimpse will cause the output to increase. Since the output won’t have decayed much from the first, the second may be enough to cross the Schmitt trigger threshold. This may be a very niche use of a peak detector, but given how limited our detecting range is it proved useful. 
	

Figure 10: Peak Detector

The third and final component is an inverting Schmitt trigger which was used to take the output from the peak detector and determine whether or not the IR detector is sensing the 1kHz beacon. An inverting Schmitt trigger was used because it output a high value while not detecting the beacon which made debugging easier. If the circuit was not functioning for any reason and the sensor output low then it would appear to detect the beacon constantly. As compared to the opposite, where the output would be low until the beacon was detected, electrical errors could be identified before we started driving the robot and it was clear that a perceived error was caused by the electronics or the code. 

The circuit itself went through several iterations before ending up as seen in Figure exl. The trigger values of the circuit had to be retuned several times throughout testing when the actual output values of each component varied slightly from calculations (as expected). Once we mounted the breadboards containing all the circuitry and powered it off the ESP, rather than lab equipment, the values changed once again requiring one final tweak. We were working with fairly small voltages so the difference between a peak of 250 and 200mV was enough to fall below the upper Schmitt trigger voltage. The final trigger voltages were V+ = 200mV and V- = 97mV. Detailed calculations can be found in Appendix C.


Figure 11: Schmitt Trigger

	After the initial design of each component, many changes were made in order to increase the overall sensing range of the circuit. The gain of the filter was increased as much as possible while keeping the Schmitt trigger threshold low in order to detect the beacon from farther away. However this had a limit since amplifying too much meant the 5kHz signal would also trigger detection. 

	We then had to have some mechanical means of escape for Robin. We considered many solutions, from using a simple servo arm to a four-bar linkage to place something down and allow Robin to escape. While we had more experience with the four-bar linkage from previous classes, using a servo arm was the optimal solution for this project. This is because a servo arm will keep our mechanism simple and robust while allowing us to focus more time into the programming aspects of our project. A four-bar mechanism has much more control, as we can design it to pass through multiple positions. However, this requires much more effort and time compared to the servo arm which can be programmed rapidly. To avoid the time penalty for not removing our mechanism, we decided to design a deployable mechanism. 
We settled on a ladder that would reliably hook on the bottom rim of the window. It was stored in a slot that would allow it to be deployed as the robot drives forward but would not fall out otherwise. The side edge of the window drags the ladder from the slot as the robot drives forward. This minimalist design was chosen to reduce electrical and programming workload and rely purely on natural forces and center of gravity to deploy the ladder (as long as the ladder is placed in the window opening). 

	One of the functions where we had the most creativity was indicating that we have found Robin’s beacon. Here, our options did not have many consequences, and thus we decided to have some fun with the reporting. We theorized that we could use Piezo buzzers, LED light shows, or simply printing to an LCD screen. To decide amongst the three, we thought out which method would provide the quickest communication to the entire team as soon as the beacon was discovered. For this reason we chose the Piezo speaker. Also later on in the project we realized we would have most likely been forced to choose a Piezo vs an LCd or an array of LEDs due to the limited amount of PIN’s on the ESP32 that were quickly filled up by all our other sensors.

	Another function of our robot is the reporting of Robin’s location. We decided that this can be done one of two ways, either through a print message on an LCD screen or through the RBE 2001/2002 Field Controller App. While we could have integrated the LCD screen with the previous function and eliminate the use of a different solution, we decided against the LCD due to the amount of pins needed to make it run. It would have also been difficult to utilize without blocking code in the libraries required to make it run. Therefore, we decided to use the Field Controller App. We were able to print out messages in the IMU data tab and manipulated the IMU X, Y, and Z position values to display our robot’s X position, Y position and the address of Robin as a coded number, respectively. 

	The coded number sent to the Z position value was a 3 digit number with a leading digit, a digit representing the grid location of the building, and a digit representing which side of the building the beacon is on. There are 9 possible building locations and buildings can have anywhere between 1 and 4 accessible sides. We numbered the sides of each building as shown in Figure 12 below. Each building plot itself were also numbered according to Figure 13. For example, Plot B is represented by a value of 1 while Plot E would be 4. A function was then written that took in the robot’s X and Y position from the IMU as well as the recorded X and Y coordinate in the grid for both the robot and the building with the beacon. A series of if statements then determined exactly which address the beacon is located at and returns a message constructed within the logic. Table 1 below shows all the encoded values and the addresses they correspond to. The 2nd and 3rd digit were initially set to 9 and no else statements were used so if anything not specifically handled in the logic is sent to the function a 9 will remain in the input, indicating that something went wrong. 


Figure 12: Building Side Encoding Values


Figure 13: Street Grid with Plot Encoding

Table 1: Beacon Position Encoding

Message
Address
100
200 Oak Street
101
500 2nd Street
102
100 Beech Street
103
600 1st Street
110
400 Oak Street
111
500 3rd Street
112
300 Beech Street
113
600 2nd Street
120
600 Oak Street
122
500 Beech Street
123
600 3rd Street
130
200 Beech Street
131
300 2nd Street
132
100 Maple Street
133
400 1st Street



Message
Address
140
400 Beech Street
141
300 3rd Street
142
300 Maple Street
143
400 2nd Street
150
600 Beech Street
152
500 Maple Street
153
400 3rd Street
160
200 Maple Street
161
100 2nd Street
163
200 1st Street
170
400 Maple Street
171
100 3rd Street
173
200 2nd Street
180
600 Maple Street
183
200 3rd Street











System Integration
	Our overall system was assembled on the provided Basebot with a few modifications and additional 3D printed parts. Generally, our strategy in order to get a sense of the field configuration is to first drive across 1st and Oak Street to scan for building locations. Then, we will systematically navigate from building to building to search each window for the IR beacon. Every step of this required the use of multiple sensors and mechanisms. 

	To make the scanning of all buildings on the field possible, we first drive down 1st Street and scan down each row of the field. This is done with the upper ultrasonic sensor that is centered vertically between the windows of a building. This method is done until we get to the intersection of 1st and Oak. We then switch our sensing algorithm while the robot travels down Oak Street. A problem comes up of overwriting previous building values, so we decided that we will only alter building positions in front of a scanned building. This creates an accurate depiction of the field but means that the map that we pass into the path-planning state machine may have more than 5 buildings. 

	In order to search the buildings identified during the scanning procedure, the robot starts from the corner of the field. (5,0) on the field coordinates. Then the robot first checks to see if there are any buildings in the nearest row, if there aren’t then it will drive to the next row and scan the next row. If there are buildings in the row, the robot will drive to the furthest building in the row and begin scanning the building. Once the building is scanned it will move to the next building in the row. To scan the building, the robot will point it turret to the window, scan for the beacon, and then if no beacon was found it will turn the corner and repeat. In the map each plot has information relating to how many times the robot must scan and repeat to fully scan the building. 

Once the beacon is found, or if there are no buildings left to scan then the robot will drive home. The robot will drive to the nearest row clear of building plots and drive back to the home coordinates. All the driving is done by using 2 functions, a driveTo function that drives X number of plots in the X direction and then drives a Y number of plots in the Y direction. The second function just drives up one plot in the robots current heading. To accomplish this, the robot constantly updated its coordinate position on the map as well as its heading. This was achieved by using the forwards kinematics loop created in lab 4, to constantly check the encoder data to see if the robot had driven 1 plot of distance and incrementing the proper X or Y coordinate. 

Detecting the IR beacon required more than just the sensor circuit. The emitter/detector pair had a rather short range, only reliably sensing within a few inches. This meant that we needed to ensure we could get the detector very close to the emitter as we searched buildings. We decided to mount the detector on a simple arm to keep it at the correct height and extends slightly past the base of the robot. As the robot drove around a building, roughly in the middle of the street, the detector was positioned about 3 inches of the window. The arm was mounted on a turret that also held the upper ultrasonic sensor. Our robot primarily sensed on its right side which faced the grid while searching for buildings and is always facing buildings as we drive around them counter clockwise. In order to maintain this across all states, the turret rotated 180° to flip the orientation of the ultrasonic and arm before it began scanning for the IR beacon. This way we could both search for buildings and scan for the beacon without changing our method of driving. A block diagram of the entire system is shown below in Figure 14. 


Figure 14: IR Detector Sensor Block Diagram

Once the beacon was detected it had to then be handled in code. Since the overall output of the sensor was from a Schmitt trigger we only had to read one analog pin to see if it was outputting high or low. While not detecting the beacon the circuit output high at about 1.5 volt, or an ADC value of about 1900. When a beacon is detected it then dropped to about 30mV. We wrote a function, scanBeacon, that simply read the ADC pin and checked if the value was less than 1500 and returned true or false. This function was then used in a handful of states, rather than being in a beacon scanning state itself. The robot could drive near the beacon at many different stages of navigation so this was necessary in order to detect it outside of while specifically sweeping around a building. 

In order to handle all the functionality described above the main students robot state machine was expanded and two other state machines were added to handle searching and scanning. The same functionality could have been achieved without adding the two nested state machines but they were added to keep the code neater. The main state machine just handled the functions like communicating and returning home, where a few states of the state machine were broken up further into sub state machines handling searching and scanning. All the state diagrams are found in Appendix E. 



Performance
	During the final trial and presentation our robot performed very well. With the random configuration that was given to us, our robot was able to navigate around the field and scan all the buildings successfully. When it began moving to search each building, we found that we had to perform a small number of micro-adjustments to the robot by hand to ensure that the beacon would remain in the IR detector’s range. One of our major problems that we knew coming into the final demonstration was the amount of time it took for us to locate the beacon. Since we were using a heuristic method to move our robot to each building, we ended up with many intricacies that caused our robot to perform strange motions such as turning 270 degrees rather than 90 degrees to correct IMU value buildup. Additionally, the configuration of the field for the final demonstration warranted spots with uncertainty in whether a building existed. This was due to our scanning procedure scanning from the two outermost roads inward, marking uncertainties for locations not visible to the ultrasonic. To avoid going over the time limit, we were allowed to move the beacon into a building earlier in our scanning procedure, even though we were confident the robot would finish the task successfully (without regard to time) if we had not moved it. When this occurred, we successfully detected the beacon and deployed our rescue mechanism. Then we were able to travel back to the starting position and end the trial. 

	However, we mentioned that our robot could not detect the roadblock, and were allowed another trial just to see if the robot would act on the roadblock at all. This second trial went much better in terms of navigation, requiring little extra adjustment to truly navigate the field on its own. When we navigated to the roadblock the robot just pushed it out of the way, losing some points. Even with this nuance, we were still able to detect Robin’s beacon, deploy the means of escape, and navigate home once again. 
	While navigating we had the robot sending its x and y grid coordinates, as well as the x and y coordinates of the building it was currently searching (if any) to the field controller. The function we wrote to encode the address based on those coordinates functioned logically but did not output the correct address while searching during our demo. We watched the robot location updating as it moved and saw that it attempted to send out the address of the beacon before the robots location was updated. This explains why the address was rarely correct in testing, although we were unable to find a way to fix this before the demo. It would have required us to delay the address publish which is possible but tricky to do while remaining non blocking. 

	For the future, we would improve our driving algorithm to stay within the bounds of the time constraint. We could have used something such as a breadth-first search algorithm to find a more efficient path, and thus shave off considerable time from our trial. We also did not have enough time to implement proper obstacle avoidance, and therefore this would be a big focus if we were to improve the robot further. 


Appendix A: Sensors Data and Code
Figure 15: Maxbotics Ultrasonic Sensor Experimentation Results


Figure 16: Sharp IR Distance Sensor Accuracy and Trendline Equation Results



Figure 17: 4 Pin Ultrasonic Sensor Pulse-width Calculations & Code


Appendix B: Filter Calculations

Cutoff frequency: fc=12RCCenter frequency.
Capacitors were selected in advance based on availability

Ideal: fc1=800 Hz  fc2=1200 Hz
C=1uFR1=189.9  R2=132.6 

Actual: fc1=780 Hz  fc2=1166 Hz
C=1uFR1=204  R2=136.5 

Center frequency: fc=f1f2
fc=954 Hz

Non-Inverting Op Amp Gain: A=1+R2R1
R1=510  R2=3.7 k A=8.26


Appendix C: Schmitt Trigger Calculations 

Trigger Thresholds: V+=VCCR2R2+R1|R3 V-=VCCR2|R3R1+R2|R3
Ideal Values: VCC=3.3 V V+=200 mVV-=100 mV
R1=250  R2=10  R3=259 

Actual Values: V+=199 mVV-=96.9 mV
R1=251  R2=8.23  R3=261 


Appendix D: Morphological Chart
Below is an image of our morph chart. A link has also been provided since the image may be hard to read. 

https://docs.google.com/spreadsheets/d/1oWlO_WBTX6ns2JIXz_RFbUg6DyYsoesO_QZXvTRu4wY/edit?usp=sharing




















Appendix E: State Diagrams
Below is the 3 State diagrams for the state machines used in the robot. The first is the original students robot state machine, the second is the scanning state machine and the third is the searching state machine. 





















