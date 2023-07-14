#define ENCA 2      // yellow
#define ENCB 3      // green

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
  Serial.println(pos);      // Print position to debug code and to view the position output
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b>0){
    pos++;                  // If signal B is already high -> clockwise rotation
  } else {
    pos--;                  // If signal B is low -> counter-clockwise rotation
  }
}
