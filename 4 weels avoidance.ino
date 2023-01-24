  //ARDUINO OBSTACLE AVOIDING CAR//
// Before uploading the code you have to install the necessary library//
//AFMotor Library https://learn.adafruit.com/adafruit-motor-shield/library-install //
//NewPing Library https://github.com/livetronic/Arduino-NewPing// 
//Servo Library https://github.com/arduino-libraries/Servo.git //
// To Install the libraries go to sketch >> Include Library >> Add .ZIP File >> Select the Downloaded ZIP files From the Above links //


#include <AFMotor.h>  
#include <NewPing.h>
#include <Servo.h> 



#define TRIG_PIN A0 
#define ECHO_PIN A1 
#define MAX_DISTANCE 200 // Sonar
#define MAX_SPEED 190 // sets speed of DC  motors
//#define MAX_SPEED_OFFSET 20

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);// sonar 


Servo myservo;   

boolean goesForward=false;
int distance = 100;
int speedSet = 0;
///....................IR Section..........................
int IRpin  =  5;    //sets the IR sensor @pin 5
//defines the values
float IRvalue;        
float IRdistance;  

 int IRpin3  =  3;    //sets the IR sensor @pin 3
//defines the values
float IRvalue3;        
float IRdistance3;  

//.......................................................




void setup() {

 pinMode(13,OUTPUT);   //left motors forward
pinMode(12,OUTPUT);   //left motors reverse
pinMode(11,INPUT);   //right motors forward
pinMode(10,INPUT);   //right motors reverse
pinMode(9,OUTPUT);   //Led
Serial.begin(9600);

  myservo.attach(9);  
  myservo.write(115); 
  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  pinMode (IRpin,INPUT); 
  pinMode (IRpin3,INPUT); 
}

void loop() {


  
 int distanceR = 0;
 int IRdistance  =0;
 int IRdistance3 =0;
 int distanceL =  0;
 delay(40);

//IR section..............................

 IRvalue = analogRead(IRpin);  //sets the IRvalue to read the IRpin 
 
  IRdistance = 2076/(IRvalue -11); //converts the distance to cm
 
  delay(100);  //delay of 100 milliseconds for every reading
  
IRvalue3 = analogRead(IRpin3);  //sets the IRvalue to read the IRpin 
 
  IRdistance3 = 2076/(IRvalue3 -11); //converts the distance to cm

 //........................................................


 if (IRdistance<4)
 {
 shortAvoidRight();  
 }

 if (IRdistance3<4)
 {
  shortAvoidLeft();  
 }

 
 //Sonar Radar section.......    ...................................................

  if(distance<=45)
 {
  moveStop();
  delay(100);
  moveBackward();
  delay(100);
  moveStop();
  delay(200);
  distanceR = lookRight();
  delay(200);
  distanceL = lookLeft();
  delay(200);
 
  if(distanceR>=distanceL)
  {
    turnRight();
    delay (200);
    moveStop();

  // Serial.print ("Go Right");
    
  }else
  {
    turnLeft();
    delay (200);
    moveStop();
  }
 }else
 {
  moveForward();
 }
 distance = readPing();
}

int lookRight()
{
    myservo.write(50); 
    delay(500);
    int distance = readPing();
    delay(100);
    myservo.write(115); 
    return distance;
}

int lookLeft() 
{
    myservo.write(170); 
    delay(500);
    int distance = readPing();
    delay(100);
    myservo.write(115); 
    return distance;
    delay(100);
}



// Next function returns the real distance in cm sonar measured ...............................................................

int readPing() { 
  delay(70);
  int cm = sonar.ping_cm();
  if(cm==0)
  {
    cm = 250;
  }
  return cm;
}

//Muvement and avoidance functions................................................................................................

void moveStop() {
   analogWrite(10,0);
   analogWrite(11,0);
  } 
  
void moveForward() {

 if(!goesForward)
  {
    goesForward=true;
  digitalWrite(13,LOW);// pin 13 si pin 12 in nivel logig inalt va actiona motoarele pentru mers inainte.
  digitalWrite(12,LOW);
  analogWrite(10,255);  // pinii 10 si 11 regleaza viteza motoarelor.Vor trimite semnal PWM de la 0 la 100 catre pinii EN A si EN B al lui L298.
  analogWrite(11,255);
 
  }
}

void moveBackward() {
    goesForward=false;
  digitalWrite(13,HIGH);
  digitalWrite(12,HIGH);
  analogWrite(10,255);
  analogWrite(11,255); 

 
}  

void turnRight() {
  digitalWrite(13,HIGH);
  digitalWrite(12,LOW);
  analogWrite(10,255);  // pinii 10 si 11 regleaza viteza motoarelor.Vor trimite semnal PWM de la 0 la 100 catre pinii EN A si EN B al lui L298.
  analogWrite(11,255); 

} 
 
void turnLeft() {
   digitalWrite(13,LOW);
  digitalWrite(12,HIGH);
  analogWrite(10,255);  // pinii 10 si 11 regleaza viteza motoarelor.Vor trimite semnal PWM de la 0 la 100 catre pinii EN A si EN B al lui L298.
  analogWrite(11,255); 
 
}  


void shortAvoidRight(){
   digitalWrite(13,HIGH);
  digitalWrite(12,LOW);
  analogWrite(10,255);  // pinii 10 si 11 regleaza viteza motoarelor.Vor trimite semnal PWM de la 0 la 100 catre pinii EN A si EN B al lui L298.
  analogWrite(11,255); 
  Serial.print ("Short Turn Right ");
  delay(300);
   analogWrite(10,0); //stop motors
   analogWrite(11,0);
   delay(200);  
  digitalWrite(13,LOW);// pin 13 si pin 12 in nivel logig inalt va actiona motoarele pentru mers inainte.
  digitalWrite(12,LOW);
  analogWrite(10,255);  // pinii 10 si 11 regleaza viteza motoarelor.Vor trimite semnal PWM de la 0 la 100 catre pinii EN A si EN B al lui L298.
  analogWrite(11,255);
}

void shortAvoidLeft(){
   digitalWrite(13,LOW);
  digitalWrite(12,HIGH);
  analogWrite(10,255);  // pinii 10 si 11 regleaza viteza motoarelor.Vor trimite semnal PWM de la 0 la 100 catre pinii EN A si EN B al lui L298.
  analogWrite(11,255); 
  Serial.print (" Short Turn Left "); 
   delay(300);  
   analogWrite(10,0); //stop motors
   analogWrite(11,0);
   delay(200);  
  digitalWrite(13,LOW);// pin 13 si pin 12 in nivel logig inalt va actiona motoarele pentru mers inainte.
  digitalWrite(12,LOW);
  analogWrite(10,255);  // pinii 10 si 11 regleaza viteza motoarelor.Vor trimite semnal PWM de la 0 la 100 catre pinii EN A si EN B al lui L298.
  analogWrite(11,255);
  Serial.print (" Go Forward  ");  
}
