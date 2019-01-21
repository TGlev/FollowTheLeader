#include <PID_v1.h>
#define LWIEL 10
#define RWIEL 11
#define trigPin 4
#define echoPin 5

#define STOP 1500
#define SERVOMAX 2500
#define SERVOMIN 500

int LeftWheelValue = 1400;
int RightWheelValue = 1600;

const uint16_t t1_load = 0;
const uint16_t t1_comp = 40000;

double setPoint = 15; //Desired distance in cm 
double distance; // Input for PID
double PIDOutput;
double filteredOutput;
double Kp = 20;
double Ki = 10;
double Kd = 5;

const int numReadings = 50;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
double average = 0;                // the average

PID myPID(&average, &PIDOutput, &setPoint, Kp, Ki, Kd, DIRECT);

int LeftSensor = A0;  // Links
int MidSensor = A1;  // Midden
int RightSensor = A2;  // Rechts

float start;
float vergelijk;

int LeftSensorIR;
int MidSensorIR;
int RightSensorIR;

void setup() 
{
  pinMode(LWIEL, OUTPUT);
  pinMode(RWIEL, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  initAverage();
  initInterrupt();
  Serial.begin(9600);
  myPID.SetMode(AUTOMATIC); //turn on PID
  myPID.SetTunings(Kp,Ki,Kd);//set gain

  pinMode(LeftSensor, INPUT);
  pinMode(MidSensor, INPUT);
  pinMode(RightSensor, INPUT);
  
  Serial.println("Started up.");
}

void loop() 
{
 keuzeSturen(sensorWaarden());
}

void calculateWheels()
{
  if(average < 7)
  {
    achteruit();
  }
  else
  {
    myPID.Compute();
    filteredOutput = map(PIDOutput, 0, 255, 0, 100);
    PID2Servo();
  }
}

int sensorWaarden()
{
  char returnValue = 0;
  char A,B,C = 0;
  
  LeftSensorIR = analogRead(LeftSensor);
  MidSensorIR = analogRead(MidSensor);
  RightSensorIR = analogRead(RightSensor);
    
  if(LeftSensorIR > 600){A = 0x02;} else{A = 0x00;}
  if(MidSensorIR > 600){B = 0x04;}  else{B = 0x00;}
  if(RightSensorIR > 600){C = 0x01;} else{C = 0x00;}
  returnValue = A+B+C; 
  return returnValue;
}

void keuzeSturen(int sensorWaarden)
{
  switch(sensorWaarden){
    case 0x00: 
    vergelijk = millis();
      if((vergelijk - start)>750){
        float verschil=vergelijk-start;
        start=millis(); 
        while((LeftSensorIR < 600) && (MidSensorIR < 600) && (RightSensorIR < 600)) 
        {
          draai();
          LeftSensorIR = analogRead(LeftSensor);
          MidSensorIR = analogRead(MidSensor);
          RightSensorIR = analogRead(RightSensor);
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
      calculateWheels();
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
      calculateWheels();
      Serial.println("Allemaal");
    break; 
    default :
      //error 
      Serial.println("ERROR");
  }
}

void PID2Servo()
{
  LeftWheelValue = 1400+filteredOutput;
  RightWheelValue = 1600-filteredOutput;
}

void aansturenServo(int servopin,  float input)
{  
  digitalWrite(servopin, HIGH);
  delayMicroseconds(input);
  digitalWrite(servopin, LOW);
}

int getDistance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);

  int calculated = duration*0.034/2;
  if (calculated > 100){return 100;}
  
  distance = calculated;
  getAverage();
}

void initInterrupt()
{
  TCCR1A = 0;

  TCCR1B |= ~(1 << CS12);
  TCCR1B &= (1 << CS11);
  TCCR1B &= ~(1 << CS10);

  TCNT1 = t1_load;
  OCR1A = t1_comp;

  TIMSK1 = (1 << OCIE1A);

  sei();
}


void initAverage()
{
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;
}

void getAverage()
{
  total = total - readings[readIndex];
  readings[readIndex] = distance;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;

  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  average = total / numReadings;
  delay(2);
}

ISR(TIMER1_COMPA_vect)
{  
  TCNT1 = t1_load;
  aansturenServo(LWIEL, LeftWheelValue);
  aansturenServo(RWIEL, RightWheelValue);
  distance = getDistance();
 
}

void links()
{
  LeftWheelValue = 1600;
  RightWheelValue = 1500;
}

void rechts()
{
  LeftWheelValue = 1500;
  RightWheelValue = 1400;
}

void stilstand()
{
  LeftWheelValue = 1500;
  RightWheelValue = 1500;
}

void draai()
{
  LeftWheelValue = 1400;
  RightWheelValue = 1400;
}

void rechtdoor()
{
  LeftWheelValue = 1400;
  RightWheelValue = 1600;
}

void achteruit()
{
  LeftWheelValue = 1600;
  RightWheelValue = 1400;
}
