// Global variables
unsigned long frequency = 0.5;                // unsigned long for storing time
unsigned long timePeriod = 1000/frequency;    // changing unit from s to ms
unsigned long delayTime = 100;

int ledState = LOW;                           // initial state of LED

unsigned long prevMillis = 0;                 // to store time-stamp of previous update to LED

void setup() {
  // put your setup code here, to run once:
  // initializing digital pin LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currMillis = millis();
  if (currMillis - prevMillis == delayTime) {
    prevMillis = currMillis;

    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    digitalWrite(LED_BUILTIN, ledState);      // update state of LED
  }
}
