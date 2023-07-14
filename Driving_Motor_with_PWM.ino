#define ENCA 2      // yellow
#define ENCB 3      // green
#define PWM 5
#define INA 4
#define INB 7

int pos = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder, RISING);   // Setting up the interrupt (interrupt number, function, trigger type)

}

void loop() {
  // put your main code here, to run repeatedly:
  setMotor(1,25,PWM,INA,INB);
  delay(200);
  Serial.println(pos);
  setMotor(-1,25,PWM,INA,INB);
  delay(400);
  Serial.println(pos);
  setMotor(0,25,PWM,INA,INB);
  delay(200);
  Serial.println(pos);
}

void setMotor(int dir, int pwmVal, int pwm, int inA, int inB){
  analogWrite(pwm,pwmVal);
  // ccw is +1 and respective inA and inB values are from the truthtable
  if (dir == 1) {
    digitalWrite(inA,LOW);
    digitalWrite(inB,HIGH);
  } else if (dir == -1) {
    digitalWrite(inA,HIGH);
    digitalWrite(inB,LOW);
  } else {
    digitalWrite(inA,LOW);
    digitalWrite(inB,LOW);
  }
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b>0){
    pos++;                  // If signal B is already high -> clockwise rotation (assumed)
  } else {
    pos--;                  // If signal B is low -> counter-clockwise rotation (assumed)
  }
}
