#include <XBOXRECV.h> 
#include <math.h>
#include <Servo.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

USB Usb;
XBOXRECV Xbox(&Usb);

Servo sissorLift;

//define Pi
#define pi 3.14
#define RELAYUP 24
#define RELAYDOWN 22

//Variables used to run autonomysly
int currentMillis = 0; //place to store the current clock count
int previousMillis = 0; //place to store the previous clock count
int straightTime = 500; //the amount of time it will go straight in mS
float turnTime = 0; //the amount of time it will turn in ms, assigned later
int angle = 90; //Place to store the angle desiered for eah turn
int angletime = 0; //number of mS(calculated later) for a turn of "angle(degined above)
const float angleconstant = (2550 / 360); //time of a full rotation at half speed, converted to time per degree rotated
int numSides = 0; //sets the numbe of sides in the shape to finish the shape

//Motor Drive Pins, if you want to write to a pin, the same enable must be high
//One PWNM pin must always be 0, both cannot be high at the same time
//PWNM Pins take values of 0-255
//Left Motor 
int leftEnable1 = 16;
int leftEnable2 = 18;
int rightEnable1 = 15;
int rightEnable2 = 17;

int leftPWM1 = 5;
int leftPWM2 = 3;
int rightPWM1 = 6;
int rightPWM2 = 11;

int servoPin = 4;

bool secondController = false;

int liftMode;

//Place holder for the mode the robot is in
//starts at 0(tankDrive)
//Modes: 0(tankDrive), 1(arcadeDrive), 2(freakOut), 3(undefined)
int modeSelect = 0;

//stores the ratio of speeds for the robot 0-10
int speedScale = 0;

//Maximum speed the motor can handle = 255
//Divide this number by the 4, which is the maximum of speedScale variable
float scaledMotorSpeed = (255 / 4);

//Motor control values for tankDrive
//each motor has a software enable, direction and speed
//Left Motor Control Values
bool leftMotorGo = false; //software enable
bool leftMotorForward = true; //motor direction
float leftMotorSpeed = 0; //desired speed of motor
//Right Motor Control Values
bool rightMotorGo = false; //software enable
bool rightMotorForward = true; //motor direction
float rightMotorSpeed = 0; //desired speed of motor

//Values for both motors combined Arcade mode
float leftMotorPercent = 0; //scaled percent of power to the left motor
float rightMotorPercent = 0; //scaled percent of power to the right motor
bool motorGo = false; //software enable for motors
bool motorForward = true; //direction of the motors
float motorSpeed = 0; //speed of the motors
float desiredHeading = 0; //desiredHeadig obtained from the joystick

//Place to store the Analog hat values
float leftHatY; //vlaue from the left hat Y direction
float leftHatX; //vlaue from the left hat X direction
float rightHatY; //vlaue from the Right hat Y direction
float rightHatX; //vlaue from the Right hat X direction

void setup() {
  Serial.begin(115200);
  while (!Serial);
  if (Usb.Init() == -1) { //If the usb shiled isn't found then wait
    //Serial.print(F("\r\nOSC did not start"));
    while (Usb.Init() == -1); //halt
  }
  //Serial.print(F("\r\nXbox Wireless Receiver Library Started"));
  
  setPwmFrequency(leftPWM1, 8);  // change Timer2 divisor to 8 gives 3.9kHz PWM freq
  setPwmFrequency(leftPWM2, 8);  // change Timer2 divisor to 8 gives 3.9kHz PWM freq
  setPwmFrequency(rightPWM1, 8);  // change Timer2 divisor to 8 gives 3.9kHz PWM freq
  setPwmFrequency(rightPWM2, 8);  // change Timer2 divisor to 8 gives 3.9kHz PWM freq
  
  //Set all of the motor pins to be Outputs
  pinMode(leftPWM1, OUTPUT);
  pinMode(leftPWM2, OUTPUT);
  pinMode(rightPWM1, OUTPUT);
  pinMode(rightPWM2, OUTPUT);

  pinMode(leftEnable1, OUTPUT);
  pinMode(leftEnable2, OUTPUT);
  pinMode(rightEnable1, OUTPUT);
  pinMode(rightEnable2, OUTPUT);

  pinMode(RELAYUP, OUTPUT);
  pinMode(RELAYDOWN, OUTPUT);
 
  //Turn on each individual H-Bridge chips
  digitalWrite(leftEnable1, HIGH);
  digitalWrite(leftEnable2, HIGH);
  digitalWrite(rightEnable1, HIGH);
  digitalWrite(rightEnable2, HIGH);

  sissorLift.attach(servoPin);
  sissorLift.writeMicroseconds(1600);  
  
}

void loop() {
  Usb.Task();
  if (Xbox.XboxReceiverConnected) //If the revciever is connected to the arduino
  {
    if (Xbox.Xbox360Connected[0] || Xbox.Xbox360Connected[1]) //If the controller is connected, only connect one conroller
    {
      //If L1 is pressed decreese max speed till it hits 0
      if (Xbox.getButtonClick(L1, 0))
      {
        if(speedScale > 0)
        {
          speedScale--;
        }
      }
      //If R1 is pressed increase max speed till it hits 4
      if (Xbox.getButtonClick(R1, 0))
      {
        if(speedScale < 4)
        {
          speedScale++;
        }
      }
      
      if(Xbox.getButtonClick(UP, 0))
      {
        liftMode = 1;
        Serial.println(liftMode);
      }
      if(Xbox.getButtonClick(DOWN, 0))
      {
        liftMode = 2;
        Serial.println(liftMode);
      }
      if(Xbox.getButtonClick(LEFT, 0) || Xbox.getButtonClick(RIGHT, 0))
      {
        liftMode = 0;
        Serial.println(liftMode);
      }
      
      if(Xbox.getButtonClick(START, 0))
      {
        secondController = true;
      }
      if(Xbox.getButtonClick(BACK, 0))
      {
        secondController = false;
      }
      
      //set light based on value of speedScale
      switch(speedScale)
      {
        case 0:
          Xbox.setLedMode(ALTERNATING,0); //sets all lights to alternating
          Xbox.setLedMode(ALTERNATING,1);
          break;
        case 1:
          Xbox.setLedOn(LED1,0); //turns on LED1
          Xbox.setLedOn(LED1,1);
          break;
        case 2:
          Xbox.setLedOn(LED2,0); //turns on LED2
          Xbox.setLedOn(LED2,1);
          break;
        case 3:
          Xbox.setLedOn(LED3,0); //turns on LED 3
          Xbox.setLedOn(LED3,1);
          break;
        case 4:
          Xbox.setLedOn(LED4,0); //turns on LED4
          Xbox.setLedOn(LED4,1);
          break;
      }
      
      switch(liftMode)
      {
        case 0:
          digitalWrite(RELAYUP, LOW);
          digitalWrite(RELAYDOWN, LOW);
          break;
        case 1:
          digitalWrite(RELAYUP, HIGH);
          digitalWrite(RELAYDOWN, LOW);
          break;
        case 2:
          digitalWrite(RELAYUP, LOW);
          digitalWrite(RELAYDOWN, HIGH);
          break;
      }
      
      //Code to select the desiered mode
      if(Xbox.getButtonClick(X, 0))
      {
        modeSelect = 0; //Set to Tank Drive Mode 
      }
      if(Xbox.getButtonClick(A, 0))
      {
        modeSelect = 1; //set to Arcade Mode 
      }
      if(Xbox.getButtonClick(B, 0))
      {
        modeSelect = 2; //Sets to shape Drawing Mode   
      }
      //uncomment the following code to impliment a mode, 3 if button Y is pressed
      //if(Xbox.getButtonClick(Y, 0))
      //{
      //  modeSelect = 3; //Sets to nothing at the moment, lay off me, I'm busy
      //}

      switch(modeSelect)
      {
        case 0://TankDrive Mode
          //check to see if the joysticks are moved at all
          if (Xbox.getAnalogHat(LeftHatX, 0) > 7500 || 
              Xbox.getAnalogHat(LeftHatX, 0) < -7500 || 
              Xbox.getAnalogHat(LeftHatY, 0) > 7500 || 
              Xbox.getAnalogHat(LeftHatY, 0) < -7500 || 
              Xbox.getAnalogHat(RightHatX, 0) > 7500 || 
              Xbox.getAnalogHat(RightHatX, 0) < -7500 || 
              Xbox.getAnalogHat(RightHatY, 0) > 7500 || 
              Xbox.getAnalogHat(RightHatY, 0) < -7500 ||
              ( secondController &&
              Xbox.getAnalogHat(LeftHatX, 1) > 7500 || 
              Xbox.getAnalogHat(LeftHatX, 1) < -7500 || 
              Xbox.getAnalogHat(LeftHatY, 1) > 7500 || 
              Xbox.getAnalogHat(LeftHatY, 1) < -7500 || 
              Xbox.getAnalogHat(RightHatX, 1) > 7500 || 
              Xbox.getAnalogHat(RightHatX, 1) < -7500 || 
              Xbox.getAnalogHat(RightHatY, 1) > 7500 || 
              Xbox.getAnalogHat(RightHatY, 1) < -7500))
          {
            //LeftMotor Control by left analog hat Y value
            if (Xbox.getAnalogHat(LeftHatY, 0) > 7500 || 
                Xbox.getAnalogHat(LeftHatY, 0) < -7500) 
            { 
              //Set the leftHatY vlaue to that of the y axis
              leftHatY = Xbox.getAnalogHat(LeftHatY, 0);
           
              //scale between 0 and the scaled motor speed, and set direction
              if ( leftHatY > 0)
              {
                leftHatY = map(leftHatY, 7500, 32767, 0, scaledMotorSpeed);
                leftMotorForward = true;
              }
              else
              {
                leftHatY = map(leftHatY, -32767, -7500, scaledMotorSpeed, 0); 
                leftMotorForward = false;
              }
              
              //Set motor speed based on joystick value, and enable left motor
              leftMotorSpeed = leftHatY * speedScale;
              leftMotorGo = true;
            }
            else if (secondController &&
                (Xbox.getAnalogHat(LeftHatY, 1) > 7500 || 
                Xbox.getAnalogHat(LeftHatY, 1) < -7500)) 
            { 
              //Set the leftHatY vlaue to that of the y axis
              leftHatY = Xbox.getAnalogHat(LeftHatY, 1);
           
              //scale between 0 and the scaled motor speed, and set direction
              if ( leftHatY > 1)
              {
                leftHatY = map(leftHatY, 7500, 32767, 0, scaledMotorSpeed);
                leftMotorForward = true;
              }
              else
              {
                leftHatY = map(leftHatY, -32767, -7500, scaledMotorSpeed, 0); 
                leftMotorForward = false;
              }
              
              //Set motor speed based on joystick value, and enable left motor
              leftMotorSpeed = leftHatY * speedScale;
              leftMotorGo = true;
            }
            else
            {
              //if no left values, then dissable the left motor
              leftMotorGo = false;
            }
           
            //Right Motor control by the Right analog hat Y value
            if(Xbox.getAnalogHat(RightHatY, 0) > 7500 || 
               Xbox.getAnalogHat(RightHatY, 0) < -7500)
            {
               //Set the rightHatY vlaue to that of the y axis
               rightHatY = Xbox.getAnalogHat(RightHatY, 0);
                
               //scale between 0 and the scaled motor speed, and set direction 
               if ( rightHatY > 0)
               {
                 rightHatY = map(rightHatY, 7500, 32767, 0, scaledMotorSpeed);
                 rightMotorForward = true;
               }
               else
               {
                 rightHatY = map(rightHatY, -32767, -7500, scaledMotorSpeed, 0);
                 rightMotorForward = false;
               }
               
               //Set motor speed based on joystick value, and enable right motor
               rightMotorSpeed = rightHatY * speedScale;  
               rightMotorGo = true;
            }
            else if(secondController &&
               (Xbox.getAnalogHat(RightHatY, 1) > 7500 || 
               Xbox.getAnalogHat(RightHatY, 1) < -7500))
            {
               //Set the rightHatY vlaue to that of the y axis
               rightHatY = Xbox.getAnalogHat(RightHatY, 1);
                
               //scale between 0 and the scaled motor speed, and set direction 
               if ( rightHatY > 0)
               {
                 rightHatY = map(rightHatY, 7500, 32767, 0, scaledMotorSpeed);
                 rightMotorForward = true;
               }
               else
               {
                 rightHatY = map(rightHatY, -32767, -7500, scaledMotorSpeed, 0);
                 rightMotorForward = false;
               }
               
               //Set motor speed based on joystick value, and enable right motor
               rightMotorSpeed = rightHatY * speedScale;  
               rightMotorGo = true;
            }
            else
            {
              //if no right values, then dissable the right motor
              rightMotorGo = false;
            }
            //run the tankdrive Function
            tankDrive();
          }
          else
          {
            //if no xbox buttons pressed then turn the robots motors off
            robotOff();
          }
          break;
        case 1: //Arcade Mode
          //Check to see if the joysticks are moved at all
          if (Xbox.getAnalogHat(LeftHatX, 0) > 7500 || 
              Xbox.getAnalogHat(LeftHatX, 0) < -7500 || 
              Xbox.getAnalogHat(LeftHatY, 0) > 7500 || 
              Xbox.getAnalogHat(LeftHatY, 0) < -7500 || 
              Xbox.getAnalogHat(RightHatX, 0) > 7500 || 
              Xbox.getAnalogHat(RightHatX, 0) < -7500 || 
              Xbox.getAnalogHat(RightHatY, 0) > 7500 || 
              Xbox.getAnalogHat(RightHatY, 0) < -7500 ||
              ( secondController &&
              Xbox.getAnalogHat(LeftHatX, 1) > 7500 || 
              Xbox.getAnalogHat(LeftHatX, 1) < -7500 || 
              Xbox.getAnalogHat(LeftHatY, 1) > 7500 || 
              Xbox.getAnalogHat(LeftHatY, 1) < -7500 || 
              Xbox.getAnalogHat(RightHatX, 1) > 7500 || 
              Xbox.getAnalogHat(RightHatX, 1) < -7500 || 
              Xbox.getAnalogHat(RightHatY, 1) > 7500 || 
              Xbox.getAnalogHat(RightHatY, 1) < -7500))
          {
            //get the xValue of the arctan function by right analog hat Y value
            if (Xbox.getAnalogHat(RightHatY, 0) > 7500 || 
                Xbox.getAnalogHat(RightHatY, 0) < -7500) 
            { 
              //Set the rightHatY vlaue to that of the y axis
              rightHatY = Xbox.getAnalogHat(RightHatY, 0);
           
              //Scale values to a range of -180 to 180    
              if ( rightHatY > 0)
              {
                rightHatY = map(rightHatY, 7500, 32767, 0, 180);
              }
              else
              {
                rightHatY = map(rightHatY, -32767, -7500, -180, 0); 
              }
              //enable the motors
              motorGo = true;
            }
            else if (Xbox.getAnalogHat(RightHatY, 1) > 7500 || 
                Xbox.getAnalogHat(RightHatY, 1) < -7500) 
            { 
              //Set the rightHatY vlaue to that of the y axis
              rightHatY = Xbox.getAnalogHat(RightHatY, 1);
           
              //Scale values to a range of -180 to 180    
              if ( rightHatY > 0)
              {
                rightHatY = map(rightHatY, 7500, 32767, 0, 180);
              }
              else
              {
                rightHatY = map(rightHatY, -32767, -7500, -180, 0); 
              }
              //enable the motors
              motorGo = true;
            }
            else
            {
              //if no value then set rightHatY = to 1
              rightHatY = 1;
            }
            //Right Motor control by the Right analog hat Y value
            if(Xbox.getAnalogHat(RightHatX, 0) > 7500 || 
               Xbox.getAnalogHat(RightHatX, 0) < -7500)
            {
               //Set the rightHatX vlaue to that of the x axis
               rightHatX = Xbox.getAnalogHat(RightHatX, 0);
                
               //Scale values to a range of -180 to 180  
               if ( rightHatX > 0)
               {
                 rightHatX = map(rightHatX, 7500, 32767, 0, 180);
               }
               else
               {
                 rightHatX = map(rightHatX, -32767, -7500, -180, 0);
               }
               //enable the motors
               motorGo = true;
            }
            else if(Xbox.getAnalogHat(RightHatX, 1) > 7500 || 
               Xbox.getAnalogHat(RightHatX, 1) < -7500)
            {
               //Set the rightHatX vlaue to that of the x axis
               rightHatX = Xbox.getAnalogHat(RightHatX, 1);
                
               //Scale values to a range of -180 to 180  
               if ( rightHatX > 0)
               {
                 rightHatX = map(rightHatX, 7500, 32767, 0, 180);
               }
               else
               {
                 rightHatX = map(rightHatX, -32767, -7500, -180, 0);
               }
               //enable the motors
               motorGo = true;
            }
            else
            {
              //if no value then set rightHatX = tp 1
              rightHatX = 1;
            }
                
            //Turn rightHatY and rightHatX into the desired angle in radians
            //atan2 returns value of 0 to pi then -pi to 0 in radians
            desiredHeading = atan2(rightHatY, rightHatX);
            //Take the values returned by atan2 function
            //Returns values of 0-2pi radians
            if(desiredHeading < 0)
            {
              desiredHeading += (2*pi);
              desiredHeading = fmod(desiredHeading, (2*pi));
            }
            //convert radians to degrees
            //returns a value of 0-360 degrees
            desiredHeading = round(desiredHeading*180/pi);
   
            //run arcadeDrive function
            arcadeDrive();
          }
          else
          {
            //if no joystick command given then shut motors off
            robotOff();
          }  
        break;
        case 2: //shape Drawing
        
        break;
        //uncomment the following code to impliment mode 3
        // code for mode 3 goes here 
        //case 3: //unassinged, geex, im getting to it!
        //
        //  break;
      }
      
    }//End If Xbox is connected
    else //If no Xbox controller is connected, then shut off Robot
    {
      robotOff(); //coasts the robot to a stop
    }
  }//End Reciever Connected
}
/////////////////////////////////////
///////////Drive Functions///////////
/////////////////////////////////////
void tankDrive() 
{ 
  //for safty reasons, allow the eStop to inturupt any other commands
  if(Xbox.getButtonPress(L2,0) || 
     Xbox.getButtonPress(R2, 0) || 
     (secondController && 
     (Xbox.getButtonPress(L2,1) || 
     Xbox.getButtonPress(R2, 1))))
  {
    //freeze the robot
    eStop();
  }
  else{
    //Left Motor Control
    if(leftMotorGo)//if enabled
    {
      //turn left motor on
      digitalWrite(leftEnable1, HIGH);
      digitalWrite(leftEnable2, HIGH);
      //digitalWrite(rightEnable1, HIGH);
      //digitalWrite(rightEnable2, HIGH);

    
      //set the left motor speeds for forward and backward
      if(!leftMotorForward)
      {
        analogWrite(leftPWM1, leftMotorSpeed);
        analogWrite(leftPWM2, 0);
      }
      else
      {
        analogWrite(leftPWM1, 0);
        analogWrite(leftPWM2, leftMotorSpeed);
      }
    }
    else
    {
      //If left motor was not enabled, break it
      analogWrite(leftPWM1, 0);
      analogWrite(leftPWM2, 0);
      digitalWrite(leftEnable1, LOW);
      digitalWrite(leftEnable2, LOW);
      //digitalWrite(rightEnable1, LOW);
      //digitalWrite(rightEnable2, LOW);
    }
    //Right Motor Control
    if(rightMotorGo)//if enabled
    {
      //turn right motor on
      //digitalWrite(leftEnable1, HIGH);
      //digitalWrite(leftEnable2, HIGH);
      digitalWrite(rightEnable1, HIGH);
      digitalWrite(rightEnable2, HIGH);
     
      //sets the right motor speeds for forwards and backwards
      if(!rightMotorForward)
      { 
        analogWrite(rightPWM1, rightMotorSpeed);
        analogWrite(rightPWM2, 0);
      }
      else
      {
        analogWrite(rightPWM1, 0);
        analogWrite(rightPWM2, rightMotorSpeed);
      }
    }
    else
    {
      //if right motor is not enabled, break it
      analogWrite(rightPWM1, 0);
      analogWrite(rightPWM2, 0);
      //digitalWrite(leftEnable1, LOW);
      //digitalWrite(leftEnable2, LOW);
      digitalWrite(rightEnable1, LOW);
      digitalWrite(rightEnable2, LOW);
    }
  }
}

void arcadeDrive() 
{ 
  //for safty reasons,allow eBreak to freexe the robot at the highest priority
  if(Xbox.getButtonPress(L2,0) || 
     Xbox.getButtonPress(R2, 0) || 
     (secondController && 
     (Xbox.getButtonPress(L2,1) || 
     Xbox.getButtonPress(R2, 1))))
  {
    //break the robot
    eStop();
  }
  else
  {
    //set motor speed to that of the speedScale
    motorSpeed = scaledMotorSpeed * speedScale;
    
    //take desiredHeading and tell the motor a direction
    //also map the percent for each motor to a proper percentage
    if(desiredHeading <= 90)
    {
      motorForward = true;
      //leftMotorPercent = map(desiredHeading, 0, 90, 50, 100);
      leftMotorPercent = 100;
      rightMotorPercent = map(desiredHeading, 0, 90, 0, 100);
    }
    else if(desiredHeading <= 180)
    {
      motorForward = true;
      leftMotorPercent = map(desiredHeading, 90, 180, 100, 0);
      //rightMotorPercent = map(desiredHeading, 90, 180, 100, 50);
      rightMotorPercent = 100;
    }
    else if(desiredHeading <= 270)
    {
      motorForward = false;
      leftMotorPercent = map(desiredHeading, 180, 270, 0, 100);
      //rightMotorPercent = map(desiredHeading, 180, 270, 50, 100);
      rightMotorPercent = 100;
    }
    else if(desiredHeading <= 360)
    {
      motorForward = false;
      //leftMotorPercent = map(desiredHeading, 270, 360, 100, 50);
      leftMotorPercent = 100;
      rightMotorPercent = map(desiredHeading, 270, 360, 100, 0);
    }
    
    //convert to a percent
    leftMotorPercent /= 100;
    rightMotorPercent /= 100;
    
    if(motorGo)
    {
      //turn on the motors
      digitalWrite(leftEnable1, HIGH);
      digitalWrite(leftEnable2, HIGH);
      digitalWrite(rightEnable1, HIGH);
      digitalWrite(rightEnable2, HIGH);      
      
      //Give the motors the proper values based on the direction
      if(!motorForward)
      {
        analogWrite(leftPWM1, (motorSpeed * leftMotorPercent));
        analogWrite(leftPWM2, 0);
        analogWrite(rightPWM1, (motorSpeed * rightMotorPercent));
        analogWrite(rightPWM2, 0);
      }
      else
      {
        analogWrite(leftPWM1, 0);
        analogWrite(leftPWM2, (motorSpeed * leftMotorPercent));
        analogWrite(rightPWM1, 0);
        analogWrite(rightPWM2, (motorSpeed * rightMotorPercent));
      }
    }
    else
    {
      //If no input then coast
      analogWrite(leftPWM1, 0);
      analogWrite(leftPWM2, 0);
      analogWrite(rightPWM1, 0);
      analogWrite(rightPWM2, 0);
      digitalWrite(leftEnable1, LOW);
      digitalWrite(leftEnable2, LOW);
      digitalWrite(rightEnable1, LOW);
      digitalWrite(rightEnable2, LOW);
    } 
  }
}

//Freexes the motors in their tracks available at anytime the motors are running, for safty reasons
void eStop()
{
  liftMode = 0;
  
  //enable all motors
  digitalWrite(leftEnable1, HIGH);
  digitalWrite(leftEnable2, HIGH);
  digitalWrite(rightEnable1, HIGH);
  digitalWrite(rightEnable2, HIGH);
  
  //Set all motors to speed 0 which will hard break the motors 
  analogWrite(leftPWM1, 0);
  analogWrite(leftPWM2, 0); 
  //left motor
  analogWrite(rightPWM1, 0);
  analogWrite(rightPWM2, 0);
  //right motor
}



//Tells the robot to turn motors off and stops the robot
void robotOff () 
{
  //if the analog triggers are pressed run the eBrake Function
  if(Xbox.getButtonPress(L2,0) || 
     Xbox.getButtonPress(R2, 0) || 
     (secondController && 
     (Xbox.getButtonPress(L2,1) || 
     Xbox.getButtonPress(R2, 1))))
  {
    eStop();
  }
  //otherwise let the robot coast to a stop
  else
  { 
    analogWrite(leftPWM1, 0);
    analogWrite(leftPWM2, 0); 
    //left motor
        
    analogWrite(rightPWM1, 0);
    analogWrite(rightPWM2, 0);
    //right motor
    digitalWrite(leftEnable1, LOW);
    digitalWrite(leftEnable2, LOW);
    digitalWrite(rightEnable1, LOW);
    digitalWrite(rightEnable2, LOW);
  }               
}  

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 7 || pin == 8) 
  { // Timer3 or Timer4
      switch(divisor)
      {
        case 1: 
          mode = 0x01; 
          break;
        case 8: 
          mode = 0x02; 
          break;
        case 32: 
          mode = 0x03; 
          break;
        case 64: 
          mode = 0x03; 
          break;
        case 128: 
          mode = 0x05; 
          break;
        case 256: 
          mode = 0x04; 
          break;
        case 1024: 
          mode = 0x05; 
          break;
        default: 
          return;
    }
    if(pin == 5) 
    { 
      TCCR3B = TCCR3B & 0b11111000 | mode; // Timer3
    } 
    else 
    {
      TCCR4B = TCCR4B & 0b11111000 | mode; // Timer4
    }
  } 
}
   
