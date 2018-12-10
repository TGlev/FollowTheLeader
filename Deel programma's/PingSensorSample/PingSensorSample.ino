const int trigPin = 8;
const int echoPin = 9;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600);
}

void loop() {
  Serial.println(getDistance());
  delay(500);
}

int getDistance()
{

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

  return distance;
}
