#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <PID_v1.h>
#include <TaskScheduler.h>
#include <TaskSchedulerDeclarations.h>
#include <TaskSchedulerSleepMethods.h>

#define ENCA 2      // yellow
#define ENCB 3      // green
#define PWM 5
#define INA 4
#define INB 7

Scheduler runner;

int pos = 0;

double input_angle, output_voltage, desired_angle;  //defines the input angle fed into the PID, the output voltage (pwm signal) to the driver, and the desired angle that is manually set.
double Kp = 10.0; //Since we are only dealing with the proportional part of the PID calculation (for now), Kp can be manually changed for a large or small gain.
double Ki = 1.0;  //Keep this value constant
double Kd = 1.0;  //Keep this value constant 
Encoder myEnc(ENCA, ENCB);
PID position_pid(&input_angle, &output_voltage, &desired_angle, Kp,Ki,Kd,P_ON_M,DIRECT);

void position_control() {
  long newPosition = myEnc.read();  //Determines the position of the motor as an integer.
  input_angle = (360/751.8)*newPosition;  //Converts the integer value from the encoder to readable angle values to be fed into the PID calculation. 
  position_pid.Compute();  //PID calculation
  analogWrite(5, output_voltage);  //Takes the output pwm voltage and sends it to the pwm driver pin.
  Serial.print(input_angle);  //Input angle and output voltage on serial monitor
  Serial.print("  ");
  Serial.println(output_voltage);
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


Task read_input(5, TASK_FOREVER, &position_control);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  // attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder, RISING);   // Setting up the interrupt (interrupt number, function, trigger type)

  runner.addTask(read_input);
  Serial.println("Added Task");
  read_input.enable();
  Serial.println("Enabled Task");
  desired_angle = 180;
  position_pid.SetMode(AUTOMATIC);
  pinMode(INA, OUTPUT); 
  pinMode(INB, OUTPUT);
  pinMode(PWM, OUTPUT);
  //digitalWrite(INA, HIGH);
  //digitalWrite(INB, LOW);
  //analogWrite(PWM, 63);
  Serial.println("Input Angle and Output Voltage:");

}

void loop() {
  // put your main code here, to run repeatedly:
  runner.execute();
  
  setMotor(1,25,PWM,INA,INB);
  delay(200);
   Serial.println(input_angle);
  setMotor(-1,25,PWM,INA,INB);
  delay(400);
   Serial.println(input_angle);
  setMotor(0,25,PWM,INA,INB);
  delay(200);
   Serial.println(input_angle);
}
