#define LWIEL 10
#define RWIEL 11
#define trigPin 4
#define echoPin 5
int reflectpin_A = A0;  // Links
int reflectpin_B = A1;  // Midden
int reflectpin_C = A2;  // Rechts
int reflectpin_D = A3;  // Afstandssensor
int reflectpin_E = A4;  // Laser links 
int reflectpin_F = A5;  // Laser rechts


int LeftWheelValue;
int RightWheelValue;

float start;
float vergelijk;

int sensorWaarden_A = 0;
int sensorWaarden_B = 0;
int sensorWaarden_C = 0;
int sensorWaarden_D = 0;
int sensorWaarden_E = 0;
int sensorWaarden_F = 0;
//***************************************************************************************************************************************************************************************************************
void setup() {
pinMode(reflectpin_A, INPUT);
pinMode(reflectpin_B, INPUT);
pinMode(reflectpin_C, INPUT);
pinMode(reflectpin_D, INPUT);
pinMode(reflectpin_E, INPUT);
pinMode(reflectpin_F, INPUT);

pinMode(13, OUTPUT);
pinMode(12, OUTPUT);
pinMode(9, OUTPUT);
pinMode(LWIEL, OUTPUT);
pinMode(RWIEL, OUTPUT);

Serial.begin(9600);
start = millis();
}
//**************************************************************************************************************************************************************************************************************
void loop() {
  

keuzeSturen(sensorWaarden());
}
//**************************************************************************************************************************************************************************************************************
int sensorWaarden(void){
  char returnValue = 0;
  char A,B,C = 0;
  sensorWaarden_A = analogRead(reflectpin_A);
  sensorWaarden_B = analogRead(reflectpin_B);
  sensorWaarden_C = analogRead(reflectpin_C);
  /*Serial.print(sensorWaarden_B);
  Serial.print(",");
  Serial.print(sensorWaarden_C);
  Serial.print(",");
  Serial.println(sensorWaarden_A);*/
    
  if(sensorWaarden_A > 600){digitalWrite(13, HIGH); A = 0x02;} else{digitalWrite(13,LOW); A = 0x00;}
  if(sensorWaarden_B > 600){digitalWrite(12, HIGH); B = 0x04;}  else{digitalWrite(12, LOW); B = 0x00;}
  if(sensorWaarden_C > 600){digitalWrite(9, HIGH); C = 0x01;} else{digitalWrite(9, LOW); C = 0x00;}
  returnValue = A+B+C; 
  Serial.println(returnValue);
return returnValue;
}
//***************************************************************************************************************************************************************************************************************
void keuzeSturen(int sensorWaarden){
  switch(sensorWaarden){
    case 0x00: 
    vergelijk = millis();
      if((vergelijk - start)>750){
        float verschil=vergelijk-start;
        start=millis(); 
        while((sensorWaarden_A < 600) && (sensorWaarden_B < 600) && (sensorWaarden_C < 600)) {
          draai();
          sensorWaarden_A = analogRead(reflectpin_A);
          sensorWaarden_B = analogRead(reflectpin_B);
          sensorWaarden_C = analogRead(reflectpin_C);
          }
        }
      Serial.println("Geen een");
    break;
    case 0x01:
      rechts();
      Serial.println("Rechter");
    break;
    case 0x02:
      links();
      Serial.println("Linker");
    break;
    case 0x03:
      //error
      Serial.println("Rechter en Linker");
    break;
    case 0x04:
      rechtdoor();
      Serial.println("Middelste");
    break;
    case 0x05:
      rechts();
      Serial.println("Middelste en rechter");
    break;
    case 0x06:
      links();
      Serial.println("Middelste en linker");
    break;
    case 0x07:
      rechtdoor();
      Serial.println("Allemaal");
    break; 
    default :
      //error 
      Serial.println("ERROR");
  }
}
//***************************************************************************************************************************************************************************************************************
/*
int getDistance(){  
  //Making sure there is nothing on the trigpin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  //Setting the signal on the trigger pin to HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //Reading the incoming pulse from the echopin
  long duration = pulseIn(echoPin, HIGH);

  //Calculating the distance
  int distance = duration*0.034/2;
  if (distance > 100){
    return 0;}
  return distance;
*/
//***************************************************************************************************************************************************************************************************************
/*
int laserDetectie(){
  int sensorWaarden_E = analogRead(reflectpin_E);
  int sensorWaarden_F = analogRead(reflectpin_F);

  if(sensorWaarden_E > LEVEL2){
    
  }
}
*/
//***************************************************************************************************************************************************************************************************************
/*
void initInterrupt(){
  TCCR1A = 0;

  TCCR1B |= ~(1 << CS12);
  TCCR1B &= (1 << CS11);
  TCCR1B &= ~(1 << CS10);

  TCNT1 = t1_load;
  OCR1A = t1_comp;

  TIMSK1 = (1 << OCIE1A);

  sei();
}

//Deze functie word geactiveerd door de interrupt, verander de 1 naar 0 voor timer0 and 2 voor timer2
ISR(TIMER1_COMPA_vect)
{  
  TCNT1 = t1_load;

    
    aansturenServo(LWIEL, LeftWheelValue);
    aansturenServo(RWIEL, RightWheelValue);
    //distance = getDistance();
 
}
*/
//***************************************************************************************************************************************************************************************************************
void aansturenServo(int servopin,  float input)
{  
  digitalWrite(servopin, HIGH);
  delayMicroseconds(input);
  digitalWrite(servopin, LOW);

}
//***************************************************************************************************************************************************************************************************************
void links(){
  aansturenServo(LWIEL, 2000);
  aansturenServo(RWIEL, 1500);
}
//**************************************************************************************************************************************************************************************************************
void rechts(){
  aansturenServo(LWIEL, 1500);
  aansturenServo(RWIEL, 1000);
}
//**************************************************************************************************************************************************************************************************************
void stilstand(){
  aansturenServo(LWIEL, 1500);
  aansturenServo(RWIEL, 1500);
}
//***************************************************************************************************************************************************************************************************************
void draai(){
  aansturenServo(LWIEL, 1000);
  aansturenServo(RWIEL, 1000);
}
//***************************************************************************************************************************************************************************************************************
void rechtdoor(){
  aansturenServo(LWIEL, 2000);
  aansturenServo(RWIEL, 1000);
}
