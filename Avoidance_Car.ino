/* Author: Alex DiStasi
 * Date: 11/11/2019
 * Purpose: Create an obstacle avoiding robot. 
 *  Car moves due to two gearmotors attached to two wheels. An ultrasonic sensor is used to detect if there is an object in front of the robot. The ultrasonic sensor is moved by a servo motor to gain more information about surrounding environment.
 *  To view the robot in action, watch here: https://youtu.be/UX64fmhKRaA
 */


#include <Servo.h>
Servo servo;

//initialize the servo motor
const int servoPin = 11; //servo is attached to pin 11

//initializes the ultrasonic sensor
//Trigger pin sends out sound wave. If object is near, wave reflects and is detected by echo pin. Distance calculated by the speed of the process.
const int trigPin = 13; //trigger is attached to pin 13
const int echoPin = 12; //echo is attached to pin 12
  
//initialize motors. Motors control movement of robot. Each motor has two pins that can be controlled.
//rmpc1 = right motor control pin 1. Attached to pin 4
const int rmcp1 = 4;
//rmpc2 = right motor control pin 2. Attached to pin 2
const int rmcp2 = 2;
//rmsc = right motor speed control. Attached to pin 3
const int rmsc = 3;
//lmpc1 = left motor control pin 1. Attached to pin 7
const int lmcp1 = 7;
//rmpc2 = left motor control pin 2. Attached to pin 5
const int lmcp2 = 5;
//lmsc = left motor speed control. Attached to pin 6
const int lmsc = 6;
//Declare variables

//This variable is used to store information about how far an object is from the robot
float distanceFromObject = 0;
//This variable is used to determine the angle the servo should be set to.
int servoAngle = 0;
const int numAnglesToCheck = 4;
int angles_to_check[numAnglesToCheck] = {60, 90, 120, 90};
int distancesFromObject[numAnglesToCheck];
bool objectNear = false;


//Calculates distance of object in meters using ultrasonic sensor
float calculateDistance() {
  //turn on trigger pin for 10 micro seconds and then turn off trigger pin. Used to send sound wave
  digitalWrite(trigPin, HIGH); //Turn on trig pin
  delayMicroseconds(10); //pause for 10 microseconds
  digitalWrite(trigPin, LOW); //turn off trig pin

  //echoLength stores how long it takes sound wave to reach echo sensor. pulseIn() is a function that outputs how long it takes for pin to go from HIGH to LOW. 
  float echoLength = pulseIn(echoPin, HIGH);
  
  //distanceFromObject calculates the distance that the sound wave traveled in inches
  //      Formula found in data sheet for ultrasonic sensor here: https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
  float distanceFromObject = (echoLength / 148);
  Serial.print(distanceFromObject);
  Serial.print("              ");
  return distanceFromObject;
}
/*
Checks the distance of an object at 3 different positions: 60 degrees, 90 degrees, 120 degrees. 
At each position, find out how far robot is from an obstacle at that angle and store that distance value in an array
*/
void checkSurroundings(){
   //Iterate through array angles_to_check and set the servo to turn at that angle. 
   for (int i = 0; i < numAnglesToCheck; i++){
    servo.write(angles_to_check[i]);
    distancesFromObject[i]=calculateDistance();
    delay(1000);
   }
  
}

//Moves motors forward
void moveforward()                       
{
  //set pin 1 of left and right motors to high
  digitalWrite(rmcp1, HIGH);
  digitalWrite(lmcp1, HIGH);
  //set pin 1 of left and right motors to low
  digitalWrite(rmcp2, LOW);      
  digitalWrite(lmcp2, LOW);                   

  //move car forward for 1 seconds and then stop moving
  analogWrite(rmsc, 255);     
  analogWrite(lmsc, 255);     
  delay(1000);   
  analogWrite(rmsc, 0);     
  analogWrite(lmsc, 0);          
}

//stop movement to car
void stopCar()                       
{
  analogWrite(rmsc, 0);     
  analogWrite(lmsc, 0);              
}

//Move car backwards and turn towards the right to move car away from obstacle
void moveBackwards(){
  //set pin 1 of left and right motors to low
  digitalWrite(rmcp1, LOW);
  digitalWrite(lmcp1, LOW);
  //set pin 1 of left and right motors to high
  digitalWrite(rmcp2, HIGH);      
  digitalWrite(lmcp2, HIGH);       
  //move backwards for 1 second and then stop
  analogWrite(rmsc, 250);     
  analogWrite(lmsc, 250);  
  delay(1000);  
  analogWrite(rmsc, 0);     
  analogWrite(lmsc, 0);
  
  //set wheels to turn towards the right, move for half a second, then stop the car
  //set pin 1 of right motor to high and set left motor to low 
  digitalWrite(rmcp1, LOW);
  digitalWrite(lmcp1, HIGH);
  //set pin 1 of left and right motors to high
  digitalWrite(rmcp2, HIGH);      
  digitalWrite(lmcp2, LOW);       
  //move for a half second and then stop
  analogWrite(rmsc, 250);     
  analogWrite(lmsc, 250);  
  delay(500);  
  analogWrite(rmsc, 0);     
  analogWrite(lmsc, 0);
  


  
  
}

//This function is called whenever the program is run or reset
void setup() {
  //turns servo on. Servo is used to direct ultrasonic sensor
  servo.attach (servoPin);
  //move servo to 90 degrees. This is done to center the servo
  servo.write(90);
  
  //set the trigPin as an output
  pinMode(trigPin, OUTPUT);
  //set the echo pin as an input
  pinMode(echoPin, INPUT);
  
  //set the motors as output
  pinMode(rmcp1, OUTPUT);
  pinMode(rmcp2, OUTPUT);
  pinMode(rmsc, OUTPUT);
  pinMode(lmcp1, OUTPUT);
  pinMode(lmcp2, OUTPUT);
  pinMode(lmsc, OUTPUT);

  //view data in real time
  Serial.begin(9600);

}
/*
This function runs code repeatedly.
Have the robot check its surroundings. If there is an object within 15 inches, the car will stop moving, move backwards, and turn towards the right.
If there is no obstacle in its way, the car will move forward.
*/
void loop() {

  checkSurroundings();
  objectNear = false;

  //check distance of each angle to check if there is anything within 15 inches of the robot by iterating through an array of distance values. 
  //If there is something less than 15 inches away, stop the car and set boolean objectNear to True.
  for (int i = 0; i<numAnglesToCheck; i++){
    if (distancesFromObject[i] < 15.00){
      objectNear = true;
      stopCar();
    }
  }

 //If something is less than 15 inches away, move the robot backwards and turn to the right. Otherwise, have the robot move forward
  if (objectNear == true){
    Serial.println("Something is close!");
    //Move the car backwards and turn
    moveBackwards();
  }else{
    Serial.println("It's safe");
    //Move the car forward
    moveforward();
  }
  
  

}
 
