



#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <PID_v1.h>
#include <TaskScheduler.h>
#include <TaskSchedulerDeclarations.h>
#include <TaskSchedulerSleepMethods.h>

#define ENC_A 2 //Channel A on the motor
#define ENC_B 3 //Channel B on the motor
#define PWM_PIN 5 //Output pin going to driver pwm input
#define IN_A 7 //Output pin going to driver INA input
#define IN_B 6 //Output pin going to driver INB input
Scheduler runner;

double input_angle, output_voltage, desired_angle;  //defines the input angle fed into the PID, the output voltage (pwm signal) to the driver, and the desired angle that is manually set.
double Kp = 1.0; //Since we are only dealing with the proportional part of the PID calculation (for now), Kp can be manually changed for a large or small gain.
double Ki = 1.0;  //Keep this value constant
double Kd = 1.0;  //Keep this value constant 

Encoder myEnc(ENC_A, ENC_B);
PID position_pid(&input_angle, &output_voltage, &desired_angle, Kp,Ki,Kd,P_ON_E,DIRECT);  //If the motor isn't responding smoothly enough, switch P_ON_E to P_ON_M.

void position_control() {
  long newPosition = myEnc.read();  //Determines the position of the motor as an integer.
  input_angle = (360/751.8)*newPosition;  //Converts the integer value from the encoder to readable angle values to be fed into the PID calculation. 
  if (input_angle > desired_angle){
    digitalWrite(IN_A, LOW);
    digitalWrite(IN_B, HIGH);
    input_angle = desired_angle - (input_angle-desired_angle);
  }
  else{
    digitalWrite(IN_A, HIGH);
    digitalWrite(IN_B, LOW);
  }
  position_pid.Compute();   //PID calculation
  analogWrite(5, output_voltage);  //Takes the output pwm voltage and sends it to the pwm driver pin.
  Serial.print(desired_angle-input_angle);  //Error and output voltage on serial monitor
  Serial.print("  ");
  Serial.println(output_voltage);
}


Task read_input(5, TASK_FOREVER, &position_control);

void setup() {
  Serial.begin(115200);

  runner.addTask(read_input);
  Serial.println("Added Task");
  read_input.enable();
  Serial.println("Enabled Task");
  desired_angle = 180; //Desired angle is manually set.
  position_pid.SetMode(AUTOMATIC);
  pinMode(IN_A, OUTPUT); 
  pinMode(IN_B, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(IN_A, HIGH);
  digitalWrite(IN_B, LOW);
  analogWrite(PWM_PIN, 63);
  Serial.println("Error and Output Voltage:");
}



void loop() {
  runner.execute();
}
