#define LWIEL 4         // zet het linker wiel op een pinout
#define RWIEL 5         // zet het rechter wiel op een pinout
#define STP 1500        // Stop commando servosturing
#define SERVOMAX 2500   // 
#define SERVOMIN 500    //
#define LEVEL1  100     // maakt LEVEL1 ... zodat de leider boven LEVEL1 wordt gezien door sensoren
#define LEVEL2  850     // maakt LEVEL2 850 zodat laser wordt gedetecteerd boven LEVEL2
#define LEVEL3  100     // waarde waarvoor de sensor de laser ziet
//******************************************************************
int sensorWaarde;
int currentServoPin;
int currentInput;
bool waitingForInt = 0;
int input = 1500;
int reflectpin_A = A0;  // sensor leidervolger rechts voorop
int reflectpin_B = A1;  // sensor leidervolger links voorop
int reflectpin_C = A2;  // sensor leidervolger voorop
int reflectpin_D = A3;  // sensor laser detectie zijkant rechts
int reflectpin_E = A4;  // sensor laser detectie zijkant links
int reflectpin_F = A5;  // afstandssensor
const int trigPin = 8;
const int echoPin = 9;
//******************************************************************
void setup() {
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
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600);
}
//**********************************************************
void loop() {
  void afstandsSensor();
  void laserDetectie();
  void sensorwaarden();
  void keuzeAansturing();
}
//**********************************************************
void afstandsSensor(int getDistance){
   while(getDistance<3){staStil();}
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
int sensorWaarden(){
  int sensorwaarden_A = analogRead(reflectpin_A);
  int sensorwaarden_B = analogRead(reflectpin_B);
  int sensorwaarden_C = analogRead(reflectpin_C);
  char a;
  char b;
  char c;
  char sensorwaarden = a+b+c;
    
  if(sensorwaarden_A > LEVEL1){a=0x01;}
  if(sensorwaarden_B > LEVEL1){b=0x02;}
  if (sensorwaarden_C > LEVEL1){c=0x04;}
//DEBUG printf("%d",sensorwaarden);
return sensorwaarden;
}

//**********************************************************
void keuzeAansturing(int sensorwaarden){
  if(sensorwaarden=0x01){rechts();}
  if(sensorwaarden=0x02){links();}
  if(sensorwaarden=0x04){rechtdoor();}
  if(sensorwaarden=0x05){rechts();}
  if(sensorwaarden=0x06){links();}
  if(sensorwaarden=0x07){rechtdoor();}
  else{draaiOmAs();}
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
void draaiOmAs(){
  aansturenServo(LWIEL, SERVOMIN);
  aansturenServo(RWIEL, SERVOMIN);
}
//***********************************************************
void staStil(){
  aansturenServo(LWIEL, STP);
  aansturenServo(RWIEL, STP);
}
//************************************************************
int getDistance()
{
  //Making sure there is nothing on the trigpin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  //Setting the signal on the trigger pin to HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH); //Reading the incoming pulse from the echopin
  
  int distance = duration*0.034/2; //berekenen afstand

  return distance;
}
//************************************************************
void laserDetectie(){
  int sensorWaarden_D = analogRead(reflectpin_D);
  int sensorWaarden_E = analogRead(reflectpin_E);
  int sensorWaarden_C = analogRead(reflectpin_C);
  int Laser_C = 0;  //aanpassen zodat sensor laser ziet

  if(sensorWaarden_D > LEVEL2){
    if(sensorWaarden_C < LEVEL3){rechts();}
    else if(sensorWaarden_C > LEVEL2){rechtdoor();} 
  }
 if(sensorWaarden_E > LEVEL2){
    if(Laser_C < LEVEL3){links();}
    else if(sensorWaarden_C > LEVEL2){rechtdoor();} 
 }
}
//*********************************************************
ISR(TIMER1_COMPA_vect){  //Deze functie word geactiveerd door de interrupt, verander de 1 naar 0 voor timer0 and 2 voor timer2
  
   if (waitingForInt = 1){
    aansturenServo(currentServoPin, currentInput);
    }
  }
