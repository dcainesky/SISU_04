/*******************************************************
* _SISU_04
* Version:  04_01
* Start Date: 03-13-22
* Last Rev:  05-05-22 (INCOMPLETE - HCSR04, Buzzer, and LED not working)
* DCC
* 
* Description:
* - Rover utility cart with rocker-bogie suspension inspired 
*  by NASA 'Curiosity' & 'Perseverance' Mars Rovers.
* - 6 wheel drive with 6 wheel steering.
* - Includes IR remote control, an ultrasonic dictance sensor, buzzer,
*  and RGB LED light.
********************************************************
*
********************************************************
DC Motors
___ motorLF = Left Front DC Motor
___ motorRF = Right Front DC Motor
___ motorLM = Left Middle DC Motor
___ motorRM = Right Middle DC Motor
___ motorLR = Left Rear DC Motor
___ motorRR = Right Rear DC Motor
***************
L298N Motor Drivers
___ L298N_F = L298N for Front Motors (located left front of chassis)
___ L298N_M = L298N for Middle Motors (located right front of chassis)
___ L298N_R = L298N for Rear Motors (located middle of chassis)
***************
Steering Servos
___ servoLF = Left Front Steering Servo
___ servoRF = Right Front Steering Servo
___ servoLM = Left Middle Steering Servo
___ servoRM = Right Middle Steering Servo
___ servoLR = Left Rear Steering Servo
___ servoRR = Right Rear Steering Servo
********************************************************/

/*******************************************************
* Include, Declare Constants/Variables, and Initialize:
********************************************************/
 // Include Libraries
 #include <IRremote.h>            // infrared remote control
 #include <Servo.h>               // steering servos
 #include <NewPing.h>             // plan to make use of 'ping' in future

 //  L298N DC Motor Drivers to Arduino MEGA 2560 Pin Assignments
 const int L298N_F_ENA = 2;     // digital PWM I/O pin to control motor 'A' speed - motorLF
 const int L298N_F_ENB = 3;     // digital PWM I/O pin to control motor 'B' speed - motorRF
 const int L298N_F_IN1 = 23;    // digital I/O pin - on/off for motor 'A' lead [OUT1] (motorLF)
 const int L298N_F_IN2 = 25;    // digital I/O pin - on/off for motor 'A' lead [OUT2] (motorLF)
 const int L298N_F_IN3 = 27;    // digital I/O pin - on/off for motor 'B' lead [OUT3] (motorRF)
 const int L298N_F_IN4 = 29;    // digital I/O pin - on/off for motor 'B' lead [OUT4] (motorRF)
 const int L298N_M_ENA = 4;     // digital PWM I/O pin to control motor 'A' speed (motorLM)
 const int L298N_M_ENB = 5;     // digital PWM I/O pin to control motor 'B' speed (motorRM)
 const int L298N_M_IN1 = 33;    // digital I/O pin - on/off for motor 'A' lead [OUT1] (motorLM)
 const int L298N_M_IN2 = 35;    // digital I/O pin - on/off for motor 'A' lead [OUT2] (motorLM)
 const int L298N_M_IN3 = 37;    // digital I/O pin - on/off for motor 'B' lead [OUT3] (motorRM)
 const int L298N_M_IN4 = 39;    // digital I/O pin - on/off for motor 'B' lead [OUT4] (motorRM)
 const int L298N_R_ENA = 6;     // digital PWM I/O pin to control motor 'A' speed (motorLR)
 const int L298N_R_ENB = 7;     // digital PWM I/O pin to control motor 'B' speed (motorRR)
 const int L298N_R_IN1 = 43;    // digital I/O pin - on/off for motor 'A' lead [OUT1] (motorLR)
 const int L298N_R_IN2 = 45;    // digital I/O pin - on/off for motor 'A' lead [OUT2] (motorLR)
 const int L298N_R_IN3 = 47;    // digital I/O pin - on/off for motor 'B' lead [OUT3] (motorRR)
 const int L298N_R_IN4 = 49;    // digital I/O pin - on/off for motor 'B' lead [OUT4] (motorRR)

// DC Motor Speeds
 int MS = 190;                    // low speed (150/255, around 60%)
                                  // mid default speed (190/255, around 75%)
                                  // high speed (230/255, around 90%)

// Steering Servos
// Create Servo Objects to Control the Servos
 Servo servoLF;             // left front
 Servo servoRF;             // right front
 Servo servoLM;             // left middle
 Servo servoRM;             // right middle
 Servo servoLR;             // left rear
 Servo servoRR;             // right rear
 // Define Pins for Servos (on MEGA 2560)
 int servoLF_Pin = 22;          // digital I/O pin for position control
 int servoRF_Pin = 24;          // digital I/O pin for position control
 int servoLM_Pin = 32;          // digital I/O pin for position control
 int servoRM_Pin = 34;          // digital I/O pin for position control
 int servoLR_Pin = 42;          // digital I/O pin for position control
 int servoRR_Pin = 44;          // digital I/O pin for position control
 // Define Variables for Servo Positions
 int servoLF_Pos = 90;          // start with wheels straight ahead
 int servoRF_Pos = 90;
 int servoLM_Pos = 90;
 int servoRM_Pos = 90;
 int servoLR_Pos = 90;
 int servoRR_Pos = 90;

// IR Receiver
const int IR_in = A0;         // variable to store IR receiver signal input pin #
IRrecv irrecv(IR_in);         // create a class object used to receive class
decode_results results;       // create a decoding results class object
unsigned long lastCode;       // define variable to store last IR code received QUESTION
unsigned long key_value=0;    // QUESTION

 // Other
 const int buzzer = 11;     // active buzzer pin - used PWM pin - for future tone control
 const int LED_R = 10;      // PWM pin for mounted RGB-LED red intensity (or include a 220 ohm resistor)
 const int LED_G = 9;       // PWM pin for mounted RGB-LED green intensity (or include a 220 ohm resistor)
 const int LED_B = 8;       // PWM pin for mounted RGB-LED blue intensity (or include a 220 ohm resistor)
 int redValue;              // define variables for LED intensity
 int greenValue;
 int blueValue;
  
// Set up HC-SR04 Variables
#define triggerPin A4 
#define echoPin A5 
long duration, cm, inches;  
int distance;
#define maxDistance 200

// NewPing setup of pins and maximum distance
NewPing sonar(triggerPin, echoPin, maxDistance); 

/*******************************************************
* Initial 'Setup' Code To Run Once:
********************************************************/
void setup()
{
  // Set L298N Connected Pins to output
  pinMode(L298N_F_ENA, OUTPUT);
  pinMode(L298N_F_ENB, OUTPUT);
  pinMode(L298N_F_IN1, OUTPUT);
  pinMode(L298N_F_IN2, OUTPUT);
  pinMode(L298N_F_IN3, OUTPUT);
  pinMode(L298N_F_IN4, OUTPUT);
  pinMode(L298N_M_ENA, OUTPUT);
  pinMode(L298N_M_ENB, OUTPUT);
  pinMode(L298N_M_IN1, OUTPUT);
  pinMode(L298N_M_IN2, OUTPUT);
  pinMode(L298N_M_IN3, OUTPUT);
  pinMode(L298N_M_IN4, OUTPUT);
  pinMode(L298N_R_ENA, OUTPUT);
  pinMode(L298N_R_ENB, OUTPUT);
  pinMode(L298N_R_IN1, OUTPUT);
  pinMode(L298N_R_IN2, OUTPUT);
  pinMode(L298N_R_IN3, OUTPUT);
  pinMode(L298N_R_IN4, OUTPUT);

  // Attach Servos on Servo Pins to Servo Objects
  servoLF.attach(servoLF_Pin);
  servoRF.attach(servoRF_Pin);
  servoLM.attach(servoLM_Pin);
  servoRM.attach(servoRM_Pin);
  servoLR.attach(servoLR_Pin);
  servoRR.attach(servoRR_Pin);

// Set HC-SR04 Pins
pinMode(triggerPin, OUTPUT);
pinMode(echoPin, INPUT);

  // Set IR Receiver Pins and Start
  pinMode(IR_in, INPUT);
  irrecv.enableIRIn();
  decode_results results;
  irrecv.blink13(true);

  // Set Buzzer Pin
  pinMode(buzzer, OUTPUT);

  // Set RGB-LED Pins
  pinMode(LED_R, OUTPUT);          // set pins to "output"
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, LOW);        // default LED to off
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  }

/*******************************************************
 * Main 'Loop' Code To Run Repeatedly:
 *******************************************************/
void loop()
{
   // Trigger HC-SR04 pulse
  digitalWrite(triggerPin, LOW);   // clear trigger by setting it low
  delayMicroseconds(5);
  digitalWrite(triggerPin, HIGH);   // trigger sensor for 10 microseconds
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
 
  // Read the echo pin to get duration in microseconds
  duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance (speed of sound = 343 m/s)
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  inches = (duration/2) / 74;   // Divide by 74 or multiply by 0.0135

  // infrared receiver
  if (irrecv.decode(&results))         // check to see if code received
  {
    if(results.value==0xFFFFFFFF)      // if a repeat then use last code received
    {
      results.value=lastCode;
    }
    switch (results.value)
    {
    case 0xFF10EF:          // receive "<" key
    turnLeft();             // turn left
    break;                  // exit from "switch" statement

    case 0xFF5AA5:          // receive ">" key
    turnRight();            // turn right
    break;

    case 0xFF6897:          // receive "*" key
    doglegLeft();           // turn all servos to left
    break;

    case 0xFF9867:          // receive "0" key
    turnFront();            // turn all servos to forward
    break;

    case 0xFFB04F:          // receive "#" key
    doglegRight();          // turn all servos to right
    break;

    case 0xFF18E7:          // receive up arrow key
    halt();
    delay(100);
    forwardMS();            // run DC motors to move ahead
    break;

    case 0xFF4AB5:          // receive down arrow key
    halt();
    delay(100);
    backwardMS();           // run DC motors to move in reverse
    break;

    case 0xFF38C7:          // receive "OK" key
    halt();                 // stop all motors
    break;

    case 0xFF22DD:          // receive "4" key
    beep();                 // beep and flash red
    flashRed();
    break;

    case 0xFF02FD:          // receive "5" key
    beep();                 // beep and flash green
    flashGreen();
    break;

    case 0xFFC23D:          // receive "6" key
    beep();                 // beep and flash blue
    flashBlue();
    break;
    
    delay(30);              // delay to prevent false readings
    irrecv.resume();        // receive the next value
    }
  }
   // Trigger actions based on HC-SR04 distance if (inches > 1 && inches < 6)
  {
    flashRed();
    beep();
    flashRed();
  }
  if (inches > 6 && inches <12)
  {
    flashYellow();
    beep();
    flashYellow();
  }
  if (inches > 12 && inches < 24)
  {
    flashGreen();
    beep();
    flashGreen();
  }
  if (inches > 24)
  {
    flashBlue();
    beepBeep();
    beepBeep();
    flashBlue();
  }
}

/*****************************************************************
 * Other Functions to be called:
 *****************************************************************/
 void beep()          // sounds buzzer for 200/1000 second
 {
  digitalWrite(buzzer, HIGH);
  delay(200);
  digitalWrite(buzzer, LOW);
 }

 void beepBeep()      // sounds buzzer twice and quicker
 {
 digitalWrite(buzzer, HIGH);
 delay(150);
 digitalWrite(buzzer, LOW);
 delay(150);
 digitalWrite(buzzer, HIGH);
 delay(150);
 digitalWrite(buzzer, LOW);
 }

 void flashRed()            // flash RGB LED red for 200/1000 second
  {
 redValue = 255;
 greenValue = 0;
 blueValue = 0;
 analogWrite(LED_R, redValue);
 analogWrite(LED_G, greenValue);
 analogWrite(LED_B, blueValue);
 delay(200);
 redValue=0;
 analogWrite(LED_R, redValue);
  }

 void flashYellow()         // flash RGB LED yellow for 200/1000 second
  {
 redValue = 255;
 greenValue = 255;
 blueValue = 0;
 analogWrite(LED_R, redValue);
 analogWrite(LED_G, greenValue);
 analogWrite(LED_B, blueValue);
 delay(200);
 redValue = 0;
 greenValue = 0;
 analogWrite(LED_R, redValue);
 analogWrite(LED_G, greenValue);
  }
  
 void flashGreen()          // flash RGB LED green for 200/1000 second
  {
 redValue = 0;
 greenValue = 255;
 blueValue = 0;
 analogWrite(LED_R, redValue);
 analogWrite(LED_G, greenValue);
 analogWrite(LED_B, blueValue);
 delay(200);
 greenValue=0;
 analogWrite(LED_G, greenValue);
  }
  
 void flashBlue()           // flash RGB LED blue for 200/1000 second
  {
 redValue = 0;
 greenValue = 0;
 blueValue = 255;
 analogWrite(LED_R, redValue);
 analogWrite(LED_G, greenValue);
 analogWrite(LED_B, blueValue);
 delay(200);
 blueValue=0;
 analogWrite(LED_B, blueValue);
  }
  
 void msSlow()              // set motor speed MS to slow
 {
  MS = 150;
 }
  
 void msMedium()            // set motor speed MS to medium
 {
  MS = 190;
 }
  
 void msFast()              // set motor speed MS to fast
 {
  MS = 230;
 }
 
 void forwardMS()           // set DC motors to move forward at set "MS" speed
 { 
  digitalWrite(L298N_F_IN1, HIGH);      // turn on motors
  digitalWrite(L298N_F_IN2, LOW);
  digitalWrite(L298N_F_IN3, LOW);
  digitalWrite(L298N_F_IN4, HIGH);
  digitalWrite(L298N_M_IN1, HIGH);
  digitalWrite(L298N_M_IN2, LOW);
  digitalWrite(L298N_M_IN3, LOW);
  digitalWrite(L298N_M_IN4, HIGH);
  digitalWrite(L298N_R_IN1, LOW);
  digitalWrite(L298N_R_IN2, HIGH);
  digitalWrite(L298N_R_IN3, HIGH);
  digitalWrite(L298N_R_IN4, LOW);
  analogWrite(L298N_F_ENA, MS);         // set speed
  analogWrite(L298N_F_ENB, MS);
  analogWrite(L298N_M_ENA, MS);
  analogWrite(L298N_M_ENB, MS);
  analogWrite(L298N_R_ENA, MS);
  analogWrite(L298N_R_ENB, MS);
 }
 
void backwardMS()           // set DC motors to move backward at set "MS" speed
 {
  digitalWrite(L298N_F_IN1, LOW);
  digitalWrite(L298N_F_IN2, HIGH);
  digitalWrite(L298N_F_IN3, HIGH);
  digitalWrite(L298N_F_IN4, LOW);
  digitalWrite(L298N_M_IN1, LOW);
  digitalWrite(L298N_M_IN2, HIGH);
  digitalWrite(L298N_M_IN3, HIGH);
  digitalWrite(L298N_M_IN4, LOW);
  digitalWrite(L298N_R_IN1, HIGH);
  digitalWrite(L298N_R_IN2, LOW);
  digitalWrite(L298N_R_IN3, LOW);
  digitalWrite(L298N_R_IN4, HIGH);
  analogWrite(L298N_F_ENA, MS);
  analogWrite(L298N_F_ENB, MS);
  analogWrite(L298N_M_ENA, MS);
  analogWrite(L298N_M_ENB, MS);
  analogWrite(L298N_R_ENA, MS);
  analogWrite(L298N_R_ENB, MS);
 }
 
void halt()                 // Halt/Stop DC Motors
 {
  analogWrite(L298N_F_ENA, 0);
  analogWrite(L298N_F_ENB, 0);
  analogWrite(L298N_M_ENA, 0);
  analogWrite(L298N_M_ENB, 0);
  analogWrite(L298N_R_ENA, 0);
  analogWrite(L298N_R_ENB, 0);
 }
 
void turnFront()            // Turn All Servos to Straight Ahead
 {
  servoLF.write(90);
  servoRF.write(90);
  servoLM.write(90);
  servoRM.write(90);
  servoLR.write(90);
  servoRR.write(90);
  delay(20);                // allow for servo to get there
 }

 void turnLeft()            // Turn Servos to Make Left Turn
 {
  servoLF.write(60);
  servoRF.write(60);
  servoLM.write(90);
  servoRM.write(90);
  servoLR.write(120);
  servoRR.write(120);
  delay(20);
 }

void turnRight()            // Turn All Servos to MakeRight Turn
 {
  servoLF.write(120);
  servoRF.write(120);
  servoLM.write(90);
  servoRM.write(90);
  servoLR.write(60);
  servoRR.write(60);
  delay(20);
 }
 
 void doglegLeft()          // Turn All Servos to Dogleg Left
 {
  servoLF.write(60);
  servoRF.write(60);
  servoLM.write(60);
  servoRM.write(60);
  servoLR.write(60);
  servoRR.write(60);
  delay(20);
 }

void doglegRight()          // Turn All Servos to Dogleg Right
 {
  servoLF.write(120);
  servoRF.write(120);
  servoLM.write(120);
  servoRM.write(120);
  servoLR.write(120);
  servoRR.write(120);
  delay(20);
 }
