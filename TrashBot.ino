#include <XBOXRECV.h> 
#include <math.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

USB Usb;
XBOXRECV Xbox(&Usb);

//define pi
const float pi = 3.14;

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
int PWMPin1 = 7; 
int PWMPin2 = 6;
int enable1 = 22;
int enable2 = 23;
//Right Motor
int PWMPin3 = 5;
int PWMPin4 = 4;
int enable3 = 24;
int enable4 = 25;

//Place holder for the mode the robot is in
//starts at 0(tankDrive)
//Modes: 0(tankDrive), 1(arcadeDrive), 2(shapeDraw), 3(undefined)
int modeSelect = 0;

//stores the ratio of speeds for the robot 0-4
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
float desiredHeading = 0; //desiredHeadig obtained from the joystick
float desiredHeadingX = 0; //place to store the right analog hat X value
float desiredHeadingY = 0; //place to store the right analog hat Y Value
float speedMagnitudeX = 0; //place to store the right analog hat X value
float speedMagnitudeY = 0; //place to store the right analog hat Y value
float speedMagnitude = 0; //place to store the scaled speed value

//Place to store the Analog hat values
float leftHatY; //vlaue from the left hat Y direction
float leftHatX; //vlaue from the left hat X direction
float rightHatY; //vlaue from the Right hat Y direction
float rightHatX; //vlaue from the Right hat X direction

void setup() {
  //Serial.begin(115200);
  //while (!Serial);
  if (Usb.Init() == -1) { //If the usb shiled isn't found then wait
    //Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  //Serial.print(F("\r\nXbox Wireless Receiver Library Started"));
  
  //Set all of the motor pins to be Outputs
  pinMode(PWMPin1, OUTPUT);
  pinMode(PWMPin2, OUTPUT);
  pinMode(PWMPin3, OUTPUT);
  pinMode(PWMPin4, OUTPUT);
  pinMode(enable1, OUTPUT);
  pinMode(enable2, OUTPUT);
  pinMode(enable3, OUTPUT);
  pinMode(enable4, OUTPUT);
 
  //Turn on each individual H-Bridge chips
  digitalWrite(enable1, HIGH);
  digitalWrite(enable2, HIGH);     
  digitalWrite(enable3, HIGH); 
  digitalWrite(enable4, HIGH); 
}

void loop() {
  Usb.Task();
  if (Xbox.XboxReceiverConnected) //If the revciever is connected to the arduino
  {
    if (Xbox.Xbox360Connected[0]) //If the controller is connected, only connect one conroller
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
      //set light based on value of speedScale
      switch(speedScale)
      {
        case 0:
          Xbox.setLedMode(ALTERNATING,0); //sets all lights to alternating
          break;
        case 1:
          Xbox.setLedOn(LED1,0); //turns on LED1
          break;
        case 2:
          Xbox.setLedOn(LED2,0); //turns on LED2
          break;
        case 3:
          Xbox.setLedOn(LED3,0); //turns on LED 3
          break;
        case 4:
          Xbox.setLedOn(LED4,0); //turns on LED4
          break;
      }
      
      //Code to select the desiered mode
      if(Xbox.getButtonClick(A, 0))
      {
        modeSelect = 1; //set to Arcade Mode 
      }
      if(Xbox.getButtonClick(X, 0))
      {
        modeSelect = 0; //Set to Tank Drive Mode 
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
              Xbox.getAnalogHat(RightHatY, 0) < -7500)
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
              Xbox.getAnalogHat(RightHatY, 0) < -7500)
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
                desiredHeadingY = map(rightHatY, 7500, 32767, 0, 180);
                speedMagnitudeY = map(rightHatY, 7500, 32767, 0, scaledMotorSpeed);
              }
              else
              {
                desiredHeadingY = map(rightHatY, -32767, -7500, -180, 0);
                speedMagnitudeY = map(rightHatY, -32767, -7500, scaledMotorSpeed, 0); 
              }
              //enable the motors
              motorGo = true;
            }
            else
            {
              //if no value then set rightHatY = to 1
              desiredHeadingY = 1;
              speedMagnitudeY = 0;
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
                 desiredHeadingX = map(rightHatX, 7500, 32767, 0, 180);
                 speedMagnitudeX = map(rightHatX, 7500, 32767, 0, scaledMotorSpeed);
               }
               else
               {
                 desiredHeadingX = map(rightHatX, -32767, -7500, -180, 0);
                 speedMagnitudeX = map(rightHatX, -32767, -7500, scaledMotorSpeed, 0);
               }
               //enable the motors
               motorGo = true;
            }
            else
            {
              //if no value then set rightHatX = tp 1
              desiredHeadingX = 1;
              speedMagnitudeX = 0;
            }
                
            //Turn rightHatY and rightHatX into the desired angle in radians
            //atan2 returns value of 0 to pi then -pi to 0 in radians
            desiredHeading = atan2(desiredHeadingY, desiredHeadingX);
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
            speedMagnitude = (speedScale * (speedMagnitudeX + speedMagnitudeY));
   
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
          //use D-Pad to scale values on the fly
          if(Xbox.getButtonClick(UP,0))
          {
            straightTime += 100; //value to increase the time the robot goes straight 
          }
          if(Xbox.getButtonClick(DOWN,0))
          {
            if(straightTime > 0)
            {
              straightTime -= 100; //value to decrease the time the robot goes straight 
            }
          }
          //step size for angles of 30 degrees
          if(Xbox.getButtonClick(RIGHT,0))
          {
            if(angle < 360)
            {
              angle += 30; //value to increase the time the robot turns
            }
          }
          if(Xbox.getButtonClick(LEFT,0))
          {
            if(angle > 0)
            {
              angle -= 30; //value to decrease the time the robot turns 
            }
          }
          
          //the amount of time in mS that it takes to turn by angle degrees 
          turnTime = angle * angleconstant;
          //set the currentMillis counter to the current Clock time
          currentMillis = millis();
            //run this statment as long as the function has run less time than the alooted time via straightTime variable
          if ((currentMillis - previousMillis) <= straightTime)
          {
            //Enable motors, set them to forward, and half speed
            rightMotorGo = true; 
            leftMotorGo = true;
            rightMotorForward = true;
            rightMotorSpeed = 125;
            leftMotorForward = true;
            leftMotorSpeed = 125;
            //run the tankDrive Function;
            tankDrive();
          }
          //run this statment as long as the function has surpased straight time, but is less than turnTime + straght Time
          else if ((currentMillis - previousMillis) <= (straightTime + turnTime))
          { 
            //Enalbes Motors, turns left, and half speed
            rightMotorGo = true;
            leftMotorGo = true;
            rightMotorForward = true;
            rightMotorSpeed = 125;
            leftMotorForward = false;
            leftMotorSpeed = 125;
            //run tankDrive Function
            tankDrive();
          }
          else
          {
            //Stop the motors
            rightMotorGo = false;
            leftMotorGo = false;
            //set the previousMillis counter to that of the currentMills counter to run the loop again
            previousMillis = currentMillis;
          }  
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
  if(Xbox.getButtonPress(L2,0) || Xbox.getButtonPress(R2, 0))
  {
    //freeze the robot
    eStop();
  }
  else{
    //Left Motor Control
    if(leftMotorGo)//if enabled
    {
      //turn left motor on
      digitalWrite(enable1, HIGH);
      digitalWrite(enable2, HIGH); 
    
      //set the left motor speeds for forward and backward
      if(!leftMotorForward)
      {
        analogWrite(PWMPin1, leftMotorSpeed);
        analogWrite(PWMPin2, 0);
      }
      else
      {
        analogWrite(PWMPin1, 0);
        analogWrite(PWMPin2, leftMotorSpeed);
      }
    }
    else
    {
      //If left motor was not enabled, break it
      analogWrite(PWMPin1, 0);
      analogWrite(PWMPin2, 0);
      digitalWrite(enable1, LOW);
      digitalWrite(enable2, LOW); 
    }
    //Right Motor Control
    if(rightMotorGo)//if enabled
    {
      //turn right motor on
      digitalWrite(enable3, HIGH);
      digitalWrite(enable4, HIGH); 
     
      //sets the right motor speeds for forwards and backwards
      if(!rightMotorForward)
      { 
        analogWrite(PWMPin3, rightMotorSpeed);
        analogWrite(PWMPin4, 0);
      }
      else
      {
        analogWrite(PWMPin3, 0);
        analogWrite(PWMPin4, rightMotorSpeed);
      }
    }
    else
    {
      //if right motor is not enabled, break it
      analogWrite(PWMPin3, 0);
      analogWrite(PWMPin4, 0);
      digitalWrite(enable3, LOW);
      digitalWrite(enable4, LOW); 
    }
  }
}

void arcadeDrive() 
{ 
  //for safty reasons,allow eBreak to freexe the robot at the highest priority
  if(Xbox.getButtonPress(L2,0) || Xbox.getButtonPress(R2, 0))
  {
    //break the robot
    eStop();
  }
  else
  {
    //take desiredHeading and tell the motor a direction
    //also map the percent for each motor to a proper percentage
    if(desiredHeading <= 90)
    {
      motorForward = true;
      leftMotorPercent = map(desiredHeading, 0, 90, 50, 100);
      rightMotorPercent = map(desiredHeading, 0, 90, 0, 100);
    }
    else if(desiredHeading <= 180)
    {
      motorForward = true;
      leftMotorPercent = map(desiredHeading, 90, 180, 100, 0);
      rightMotorPercent = map(desiredHeading, 90, 180, 100, 50);
    }
    else if(desiredHeading <= 270)
    {
      motorForward = false;
      leftMotorPercent = map(desiredHeading, 180, 270, 0, 100);
      rightMotorPercent = map(desiredHeading, 180, 270, 50, 100);
    }
    else if(desiredHeading <= 360)
    {
      motorForward = false;
      leftMotorPercent = map(desiredHeading, 270, 360, 100, 50);
      rightMotorPercent = map(desiredHeading, 270, 360, 100, 0);
    }
    
    //convert to a percent
    leftMotorPercent /= 100;
    rightMotorPercent /= 100;
    
    if(motorGo)
    {
      //turn on the motors
      digitalWrite(enable1, HIGH);
      digitalWrite(enable2, HIGH);      
      digitalWrite(enable3, HIGH);
      digitalWrite(enable4, HIGH);
      
      //Give the motors the proper values based on the direction
      if(!motorForward)
      {
        analogWrite(PWMPin1, (speedMagnitude * leftMotorPercent));
        analogWrite(PWMPin2, 0);
        analogWrite(PWMPin3, (speedMagnitude * rightMotorPercent));
        analogWrite(PWMPin4, 0);
      }
      else
      {
        analogWrite(PWMPin1, 0);
        analogWrite(PWMPin2, (speedMagnitude * leftMotorPercent));
        analogWrite(PWMPin3, 0);
        analogWrite(PWMPin4, (speedMagnitude * rightMotorPercent));
      }
    }
    else
    {
      //If no input then coast
      analogWrite(PWMPin1, 0);
      analogWrite(PWMPin2, 0);
      analogWrite(PWMPin3, 0);
      analogWrite(PWMPin4, 0);
      digitalWrite(enable1, LOW);
      digitalWrite(enable2, LOW); 
      digitalWrite(enable3, LOW);
      digitalWrite(enable4, LOW);
    } 
  }
}

//Freezes the motors in their tracks available at anytime the motors are running, for safty reasons
void eStop()
{
  //enable all motors
  digitalWrite(enable1, HIGH);
  digitalWrite(enable2, HIGH);
  digitalWrite(enable3, HIGH);
  digitalWrite(enable4, HIGH);
  
  //Set all motors to speed 0 which will hard break the motors 
  analogWrite(PWMPin2, 0);
  analogWrite(PWMPin1, 0); 
  //left motor
  analogWrite(PWMPin4, 0);
  analogWrite(PWMPin3, 0);
  //right motor
}



//Tells the robot to turn motors off and stops the robot
void robotOff () 
{
  //if the analog triggers are pressed run the eBrake Function
  if(Xbox.getButtonPress(L2,0) || Xbox.getButtonPress(R2, 0))
  {
    eStop();
  }
  //otherwise let the robot coast to a stop
  else
  {
    analogWrite(PWMPin2, 0);
    analogWrite(PWMPin1, 0); 
    //left motor
        
    analogWrite(PWMPin4, 0);
    analogWrite(PWMPin3, 0);
    //right motor
    digitalWrite(enable1, LOW);
    digitalWrite(enable2, LOW);
    digitalWrite(enable3, LOW);
    digitalWrite(enable4, LOW);
  }               
}  
   
