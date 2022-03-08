I also made it possible to be controlled using a smartphone via Bluetooth communication. I made a custom Android application through which we can control the Mecanum wheels robot to move in any direction. Also, using the slider in the app we can control the speed of movement.
The brain of this robot platform is an Arduino Mega board which controls each wheel individually. Each wheel is attached on a NEMA 17 stepper motor, and knowing the fact that stepper motors can be precisely controlled, I added one more cool feature in the app through which we can program the robot to move automatically. Using the Save button we can save each position or step and then the robot can automatically run and repeat these steps. With the same button we can pause the automatic operation as well as reset or delete all steps so we can store new ones.
Mecanum Wheels Robot 3D Model
To begin with, I designed this Mecanum Wheels robot using a 3D modeling software. The base platform of this robot is a simple box which I will make out of 8mm tick MDF boards
The four stepper motors are attached to this platform and the Mecanum wheels are attached to the motor’s shafts.

You can find and download this 3D model, as well as explore it in your browser on Thangs.
How Mecanum Wheels Work
A Mecanum wheel is a wheel with rollers attached to its circumference. These rollers are positioned diagonally or at 45-degree angle to the axis of rotation of the wheel. This makes the wheel exert force in diagonal direction when moving forward of backward
So, by rotating the wheels in certain pattern, we utilize these diagonal forces and thus the robot can move in any direction.

We should also note here that we need two types of Mecanum wheels, often referred to as, left-handed and right-handed Mecanum wheels. The difference between them is the orientation of the rollers and they must be installed in the robot in specific locations. The rotation axis of each wheel’s top roller should point to the center of the robot.
Here’s a quick demonstration of how to robot moves depending on the wheels rotation direction.

If all four wheels move forward, the resulting move of the robot will be forward, and vice versa if all wheels move backward the robot will move backward. For moving to the right, the right wheels need rotate inside the robot, while the left wheels need rotate outside the robot. The resulting force due to the diagonally positioned rollers will make the robot move to the right. The same but opposite happens when moving to the left. With these wheels we can also achieve movement in diagonal direction by rotating only two wheels.
Making the Mecanum Wheels Robot
Nevertheless, now let me show you how I built this robot platform. As I mentioned, for making the base of the platform I’m using 8mm tick MDF boards. Using a table saw, first I cut all of the pieces according to the 3D model dimensions.
Next, using a 3mm drill and a 25mm Forstner bit I made the openings on the side panels for attaching the stepper motors. Once I got the pieces ready, I continued with assembling them. I used a wood glue and some screws for securing them. The most important thing here is to have the openings for the motors precisely made so that all of the wheels have even contact with the surface later on.
Of course, you could also 3D print this base platform, instead of making it with MDF, so I will include a 3D file of it on the website article. Finally, I spray painted the base and it’s cover with white color
Next are the Mecanum wheels. As I said earlier, these wheels can be a bit expensive to buy, so that’s why I decided to design and 3D print my own ones. The wheels are made out of two parts, outer and inner side which are secured together with some M4 bolts and nuts. They have 10 rollers each, and a shaft coupler specifically designed to fit a NEMA 17 stepper motor.
PCB Design
Nevertheless, in order to keep the electronics components organized and get rid of the wiring mess, I designed a custom PCB using the EasyEDA free online circuit design software. This PCB will actually act as an Arduino MEGA shield because we will be able to directly connect it on top of the Arduino Mega board. I used both the top and the bottom layer for running the connections. For those Arduno pins which I didn’t use, I included pin header connections so that they are available in case we want to use them for something in future. I also included 12V, 5V and GND connection pins, as well as pins for selecting the stepping resolution of the drivers.
Assembling the PCB
Ok now we can move on and assemble the PCB. I started with soldering the smaller components first, the resistors and the capacitors. Then I inserted and soldered male pin headers to the PCB which will be used for connecting it to the Arduino board.
Next, I placed all female pin headers in place and soldered them as well. As for the stepper motors connections and pins for selecting the stepping resolution I used male pin headers. This way we can directly connect the motors to the PCB and use jumpers for selecting the stepping resolution. Then I soldered the terminal blocks, the trimmer and the voltage regulator.
And that’s it, the PCB is now ready and we can move on with inserting the drivers and connecting the motors to it. First, I placed the jumpers for selecting the stepping resolution. I selected 16th step resolution by connecting the MS3 pins of the drivers to 5V.
Then on top of them I placed the DRV8825 drivers, as well as, connected the NRF24L01 module and the HC-05 Bluetooth module. Now we can simple attach the PCB to the Arduno board.
Next, I connected the battery to the appropriate terminal block and placed them into the base platform.
Here I inserted the power switch in place and connected it to the other terminal block. Right above the power switch I also inserted the battery indicator LED.
What’s left now is to connect the motors to the PCB. We should note here that when connecting opposite motors, we should connect their connectors opposite as well.  This is needed later when programming the robot, so that, for example, the forward command, would move both motors in same direction, although they are actually flipped and one would make clockwise and the other anticlockwise rotation.
At the end I can simple insert the cover at the top, and so we are done with this Mecanum Wheels robot project.
Mecanum Wheels Robot Arduino Code
What’s left for this video is to take a look at the Arduino code. Actually, there are two separate Arduino codes. This one is for controlling the robot using the NRF24L01 modules and other is for controlling to robot using a smartphone.

Arduino code for controlling the robot using the NRF24L01 modules:
Descripton: So, here we are using the RF24 library for the radio communication and the AccelStepper library for controlling the stepper motors. First we need to define the pins to which all of them are connected, define some variables needed for the program below, and in the setup section set the steppers maximum speed and begin the radio communication.

In the loop section we start by reading the data coming from the RC transmitter. The RC transmitter code as well as more details how this communication works can be found on my particular tutorial for it.

So depending on the received data, for example, if the left Joystick is moved forward, its value will be greater than 160 and in such a case will call the moveForward() custom function. If we taka a look at this function we can see that all it does is it sets the speed of the motors to positive. For moving backward, the speed is set to negative. So for moving in all other directions we just have to set the rotations of the wheels appropriately as explained in the beginning.

For executing these commands, in the loop section we need to call the runSpeed() functions for all steppers. In the loop section we also read the analog input from the voltage divider coming from the battery, and according to this value we can know when the battery voltage will drop under 11V so we can turn on the indicating LED.

Arduino code for controlling to robot using a smartphone:
Arduino Code
What’s left is to take a look how the Arduino code and the Android application work. As the code is a bit longer, for better understanding, I will post the source code of the program in sections with description for each section. And at the end of this article I will post the complete source code
So first we need to define the 6 servo, the 4 stepper motors and the Bluetooth communication, as well as define some variables need for the program below. In the setup section we set the maximum speed of the steppers, define the pins to which the servos are connected, begin the Bluetooth communication and set the robot arm to initial position
#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include <Servo.h>

Servo servo01;
Servo servo02;
Servo servo03;
Servo servo04;
Servo servo05;
Servo servo06;

SoftwareSerial Bluetooth(A8, 38); // Arduino(RX, TX) - HC-05 Bluetooth (TX, RX)

// Define the stepper motors and the pins the will use
AccelStepper LeftBackWheel(1, 42, 43);   // (Type:driver, STEP, DIR) - Stepper1
AccelStepper LeftFrontWheel(1, 40, 41);  // Stepper2
AccelStepper RightBackWheel(1, 44, 45);  // Stepper3
AccelStepper RightFrontWheel(1, 46, 47); // Stepper4

#define led 14

int wheelSpeed = 1500;

int lbw[50], lfw[50], rbw[50], rfw[50]; // arrays for storing positions/steps

int servo1Pos, servo2Pos, servo3Pos, servo4Pos, servo5Pos, servo6Pos; // current position
int servo1PPos, servo2PPos, servo3PPos, servo4PPos, servo5PPos, servo6PPos; // previous position
int servo01SP[50], servo02SP[50], servo03SP[50], servo04SP[50], servo05SP[50], servo06SP[50]; // for storing positions/steps
int speedDelay = 20;
int index = 0;
int dataIn;
int m = 0;

void setup() {
  // Set initial seed values for the steppers
  LeftFrontWheel.setMaxSpeed(3000);
  LeftBackWheel.setMaxSpeed(3000);
  RightFrontWheel.setMaxSpeed(3000);
  RightBackWheel.setMaxSpeed(3000);
  pinMode(led, OUTPUT);
  servo01.attach(5);
  servo02.attach(6);
  servo03.attach(7);
  servo04.attach(8);
  servo05.attach(9);
  servo06.attach(10);
  Bluetooth.begin(38400); // Default baud rate of the Bluetooth module
  Bluetooth.setTimeout(5);
  delay(20);
  Serial.begin(38400);
  // Move robot arm to initial position
  servo1PPos = 90;
  servo01.write(servo1PPos);
  servo2PPos = 100;
  servo02.write(servo2PPos);
  servo3PPos = 120;
  servo03.write(servo3PPos);
  servo4PPos = 95;
  servo04.write(servo4PPos);
  servo5PPos = 60;
  servo05.write(servo5PPos);
  servo6PPos = 110;
  servo06.write(servo6PPos);
}
Code language: Arduino (arduino)
Then in the loop section we start by checking whether there is any incoming data.

// Check for incoming data
  if (Bluetooth.available() > 0) {
    dataIn = Bluetooth.read();  // Read the data
    This data comes from the smartphone or the Android app, so let’s take a look what kind of data it is actually sending. The Android app is made using the MIT App Inventor online application. It consists of simple buttons which have appropriate images as background.
    If we take a look at the blocks of the app, we can see that all it does is it sends one-byte numbers when the buttons are clicked.

So, depending on clicked button, we tell the Arduino what to do. For example, if we receive the number ‘2’ the mecanum wheels platform will move forward, using the moveForward custom function.

if (dataIn == 2) {
      m = 2;
    }
//
if (m == 2) {
      moveForward();
    }
Code language: Arduino (arduino)
This custom function sets all four stepper motors to rotate forward.

void moveForward() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(wheelSpeed);
}
Code language: Arduino (arduino)
For moving in any other direction, we just need rotate the wheels in the appropriate directions.

For controlling the robot arm, we use the same method. Again, we have buttons in the app and when holding the buttons, the robot arm joints move in the particular direction.


if (dataIn == 2) {
      m = 2;
    }
//
if (m == 2) {
      moveForward();
    }
Code language: Arduino (arduino)
This custom function sets all four stepper motors to rotate forward.

void moveForward() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(wheelSpeed);
}
Code language: Arduino (arduino)
For moving in any other direction, we just need rotate the wheels in the appropriate directions.

For controlling the robot arm, we use the same method. Again, we have buttons in the app and when holding the buttons, the robot arm joints move in the particular direction.


if (dataIn == 2) {
      m = 2;
    }
//
if (m == 2) {
      moveForward();
    }
Code language: Arduino (arduino)
This custom function sets all four stepper motors to rotate forward.

void moveForward() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(wheelSpeed);
}
Code language: Arduino (arduino)
For moving in any other direction, we just need rotate the wheels in the appropriate directions.

For controlling the robot arm, we use the same method. Again, we have buttons in the app and when holding the buttons, the robot arm joints move in the particular direction.

As I mentioned earlier, in the original Robot Arm control app we were using sliders for controlling the positions of the servos but that was causing some problems because in that way we had to send Text to the Arduino, instead of 1-byte number. The problem is the Arduino sometimes misses the Text coming from the App and makes error or the Robot arm shakes and behaves abnormal.

In this way we simple send a single 1-byte number when a particular button is touched down.

The Arduino code enters the while loop of that number, and stays there until we touch up the button, because at that moment we send the number 0 which means the robot should do nothing.

// Move servo 1 in positive direction
    while (m == 16) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo01.write(servo1PPos);
      servo1PPos++;
      delay(speedDelay);
    }
    // Move servo 1 in negative direction
    while (m == 17) {
      if (Bluetooth.available() > 0) {
        m = Bluetooth.read();
      }
      servo01.write(servo1PPos);
      servo1PPos--;
      delay(speedDelay);
    }
Code language: Arduino (arduino)
So, depending on the touched buttons the servos move either in positive or negative direction. The same working principle applies for all servo motors. For changing the speed of movement, we use the values coming from the slider which range from 100 to 250.

// If arm speed slider is changed
    if (dataIn > 101 & dataIn < 250) {
      speedDelay = dataIn / 10; // Change servo speed (delay time)
    }
Code language: Arduino (arduino)
By dividing them by 10 we get values from 10 to 25, which are used as delay in microseconds in the whiles loops for driving the servos.

For storing the robot movements, we simply save the current positions of the servos and the steppers into arrays, each time the Save button is clicked.

// If button "SAVE" is pressed
    if (m == 12) {
      //if it's initial save, set the steppers position to 0
      if (index == 0) {
        LeftBackWheel.setCurrentPosition(0);
        LeftFrontWheel.setCurrentPosition(0);
        RightBackWheel.setCurrentPosition(0);
        RightFrontWheel.setCurrentPosition(0);
      }
      lbw[index] = LeftBackWheel.currentPosition();  // save position into the array
      lfw[index] = LeftFrontWheel.currentPosition();
      rbw[index] = RightBackWheel.currentPosition();
      rfw[index] = RightFrontWheel.currentPosition();

      servo01SP[index] = servo1PPos;  // save position into the array
      servo02SP[index] = servo2PPos;
      servo03SP[index] = servo3PPos;
      servo04SP[index] = servo4PPos;
      servo05SP[index] = servo5PPos;
      servo06SP[index] = servo6PPos;
      index++;                        // Increase the array index
      m = 0;
    }
Code language: Arduino (arduino)
Then when we press the Run button we call the runSteps() custom function. This custom function runs through all stored steps using some for and while loops.


if (m == 14) {
      runSteps();

      // If button "RESET" is pressed
      if (dataIn != 14) {
        stopMoving();
        memset(lbw, 0, sizeof(lbw)); // Clear the array data to 0
        memset(lfw, 0, sizeof(lfw));
        memset(rbw, 0, sizeof(rbw));
        memset(rfw, 0, sizeof(rfw));
        memset(servo01SP, 0, sizeof(servo01SP)); // Clear the array data to 0
        memset(servo02SP, 0, sizeof(servo02SP));
        memset(servo03SP, 0, sizeof(servo03SP));
        memset(servo04SP, 0, sizeof(servo04SP));
        memset(servo05SP, 0, sizeof(servo05SP));
        memset(servo06SP, 0, sizeof(servo06SP));
        index = 0;  // Index to 0
      }
    }
Code language: Arduino (arduino)
We should note that it starts from the first position and goes the last position, and repeats that over and over again. Therefore, when saving the steps, we actually need to position the robot in a way that the first step has the same position as the last step. While running through the steps we can also change the speed of both the platform and the robot arm, as well as pause and reset all the steps.
So that’s pretty much everything for this tutorial. The project works well, but please note that it’s far from perfect. The automatic movements might not be that precise because of the slipping of the mecanum wheels as well as the poor performance of the servo motors. These cheap servo motors can also shake or jitter even when not moving just because don’t have enough strength to hold the weight of the 3D printed parts.

I hope you enjoyed this tutorial and learned something new. Feel free to ask any question in the comments section below and check my Arduino Projects Collection.


