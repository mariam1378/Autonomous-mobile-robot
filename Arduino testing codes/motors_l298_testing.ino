//variables
int motor1pin1 = 4;
int motor1pin2 = 5;
int motor1speed = 6;

int motor2pin1 = 7;
int motor2pin2 = 8;
int motor2speed = 9;

//Setup code
void setup() {
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(motor1speed, OUTPUT);
  pinMode(motor2speed, OUTPUT);
}

//Main code
void loop() {
  // set motor speed   
  analogWrite(motor1speed, 255);
  analogWrite(motor2speed, 255);

  //same direction forward
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  delay(1000);

  //same direction Backward
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  delay(1000);
}
