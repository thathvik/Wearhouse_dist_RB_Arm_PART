#include <Servo.h>

Servo left;  // create servo object to control a servo
Servo right;
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

const int GNND = 4;
const int GNDD = 35;
const int echo = 37;
const int trig = 39;
const int VCCC = 41;

float invcmCosnt = (2*1000000)/(100*344.8); //cmDist=rawTime/invcmCosnt

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  left.attach(3);  // attaches the servo on pin 9 to the servo object
  right.attach(5);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);  
  pinMode(GNND, OUTPUT);
  pinMode(GNDD, OUTPUT);
  pinMode(VCCC, OUTPUT);
  digitalWrite(VCCC, HIGH);
  digitalWrite(GNND, LOW);
  digitalWrite(GNDD, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  left.write(114);
  right.write(74);

  // tell servo to go to position in variable 'pos'
}


void loop() {
  float rawTime, inDist, inDist1, cmDist, cmDist1; //declare variables
  /* Ping))) produces chirp when a HIGH pulse of 2us (min) or longer duration (5us typical) is
  applied to Sig pin. To have a clarn HIGH pulse, give a low and then high */
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);
  rawTime = pulseIn(echo, HIGH); //measured in u-seconds
  cmDist = 100;
  while(cmDist > 4){
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(5);
    digitalWrite(trig, LOW);
  /* Now same Sign pin is to be used to monitor if the return echo has been received. Essentially,
  Sig pin remains HIGH until a return echo is received and at that time it goes LOW. So measure
  the time duration for HIGH at the Sig pin */
    rawTime = pulseIn(echo, HIGH); //measured in u-seconds
    cmDist = rawTime/invcmCosnt;  
    Serial.println(cmDist);
  }
  Serial.println("Out");
  Serial3.println("s");
  left.write(94);
  right.write(94);
  delay(1000);
  left.write(114); 
  right.write(114);
  delay(1700);
  Serial.println("Turned");
  left.write(94);
  right.write(94);
  Serial.println("Stopped");
   while(1){
      if(Serial3.read()=='f'){
        break;
        }
    }
  left.write(114); 
  right.write(74);   // tell servo to go to position in variable 'pos'
  delay(2500);
  left.write(94);
  right.write(94);
  while(1){
  }
}
