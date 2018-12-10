int input = 1500;
int servopin = 4;

void servoAansturingDoorOwenDeGeweldige(int servopin, int input) //DEZE SHIZZLE MOET GELOOPT WORDEN JUNG
{
    pinMode(servopin, OUTPUT);
  Serial.begin(9600);
    digitalWrite(servopin, HIGH);
  delayMicroseconds(input);
  digitalWrite(servopin, LOW);
  delay(20);
  
 
}
