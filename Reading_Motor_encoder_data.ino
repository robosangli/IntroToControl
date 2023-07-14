#define ENCA 2      // yellow
#define ENCB 3      // green
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  int a = digitalRead(ENCA);
  int b = digitalRead(ENCB);
  Serial.print(a*10);         // scaling up by 10
  Serial.print(" ");
  Serial.print(b*10);         // scaling up by 10
  Serial.println();

}
