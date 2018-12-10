#include <stdio.h>
#define LWIEL 4          // zet het linker wiel op een pinout
#define RWIEL 5          // zet het rechter wiel op een pinout
#define STP 1500         // Stop commando servosturing
#define SERVOMAX 2500   // 
#define SERVOMIN 500    //
#define LEVEL1  100         // maakt LEVEL1 ... zodat de leider boven LEVEL1 wordt gezien door sensoren
#define LEVEL2  100      // maakt LEVEL2 ... zodat laser wordt gedetecteerd boven LEVEL2

//**********************************************************

//**********************************************************
int sensorWaarde;
int currentServoPin;
int currentInput;
bool waitingForInt = 0;
int input = 1500;
int reflectpin_A = A0;  // sensor leidervolger rechts voorop
int reflectpin_B = A1;  // sensor leidervolger links voorop
int reflectpin_C = A2;  // sensor laser detectie "totaal"
int reflectpin_D = A3;  // sensor laser detectie voorop
//**********************************************************
void setup() {
  
  
//**********************************************************  
cli();//stop interrupts
//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 780;// = (16*10^6) / (20*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
sei();//allow interrupts
 //********************************************************** 
  
   aansturenServo(3, 1000); // test voor de Servo

  
}
//**********************************************************
void loop() {
 
}
//**********************************************************
void aansturenServo(int servopin, int input){
  waitingForInt = 0;
  pinMode(servopin, OUTPUT);
  Serial.begin(9600);
  digitalWrite(servopin, HIGH);
  delayMicroseconds(input);
  digitalWrite(servopin, LOW);
  currentServoPin = servopin;
  currentInput = input;  
  waitingForInt = 1;
}
//**********************************************************
int sensorWaarden(int sensorWaarde){
  int sensorwaarden_A = analogRead(reflectpin_A);
  int sensorwaarden_B = analogRead(reflectpin_B);
  char a,b;
  
  if(sensorwaarden_A > LEVEL1){a=0x01;}
  if(sensorwaarden_B > LEVEL1){b=0x02;}
//DEBUG printf("%d+%d",a,b);
return a+b;
}

//**********************************************************
void keuzeAansturing(){
  if(sensorWaarde=0x01){rechts();}
  if(sensorWaarde=0x02){links();}
  if(sensorWaarde=0x04){rechtdoor();}
}
//***********************************************************
void rechts(){
  aansturenServo(LWIEL, SERVOMIN);//aansturing linkerwiel
  aansturenServo(RWIEL, STP);//aansturing rechterwiel
}
//**********************************************************
void links(){
  aansturenServo(LWIEL, STP);//aansturing linkerwiel
  aansturenServo(RWIEL, SERVOMAX);//aansturing rechterwiel 
}
//**********************************************************
void rechtdoor(){
  aansturenServo(LWIEL, SERVOMIN);//aansturing linkerwiel
  aansturenServo(RWIEL, SERVOMAX);//aansturing rechterwiel
}
//**********************************************************
void laserDetectie(){
  int sensorWaarden_C = analogRead(reflectpin_C);

  if(sensorWaarden_C > LEVEL2){draaiRichtingLaser();} 
}
//*********************************************************
void draaiRichtingLaser(void){
  int sensorWaarden_D = analogRead(reflectpin_D);

  if(sensorWaarden_D < LEVEL2){rechts();}
  else if(sensorWaarden_D > LEVEL2){rechtdoor();}
}
//*********************************************************
ISR(TIMER1_COMPA_vect){  //Deze functie word geactiveerd door de interrupt, verander de 1 naar 0 voor timer0 and 2 voor timer2
  
   if (waitingForInt = 1){
    aansturenServo(currentServoPin, currentInput);
    }
  }
