#include <Arduino.h>
#include <esp_timer.h>

esp_timer_handle_t timer;


const int DEBUG = 0;
const int delaytime = 5;

/*         PID         */
const float Kp    = 10; //10
const float Kd    = 1;  //1
const float Ki    = 0.5; //.5

const float Kp_motor = 2.1;   //2.1
const float Kd_motor = 0.5;   //.5
const float Ki_motor = 0.0;

/*       END OF PID      */


const float smallDeviation = .2;  

float P, I, D, PID;
float P_motor, I_motor, D_motor, PID_motor;

float lsp, rsp;

float error = 0, lastError = 0;
float error_motor = 0, lastError_motor = 0;

float max_windup = 50;
float max_windup_motor = 10;

void timer_callback(void* arg);
/*       END OF PID      */

/*     MOTORS       */
const int pin_MotorR_1 = 27;    
const int pin_MotorR_2 = 26;  

const int pin_MotorL_1 = 32;
const int pin_MotorL_2 = 33;

const int MotorR_1 = 0;
const int MotorR_2 = 1;

const int MotorL_1 = 2;
const int MotorL_2 = 3;

const int Motor_enable = 25;

const int resolution = 12;
const float maxpwm = 4095 - 1;

const float MAX_PERCENT_R = 100;
const float MAX_PERCENT_L = 100;

volatile float BaseSpeedR = 100;
volatile float BaseSpeedL = 100;

volatile float Percent_R = BaseSpeedR;
volatile float Percent_L = BaseSpeedL;

const int ENC_R_pin = 13;
const int ENC_L_pin = 14;

int R_Encoder = 0, L_Encoder = 0;

/*    END OF MOTORS   */


/*     IR SENSOR       */

const int IR_0 = 21;
const int IR_1 = 19;
const int IR_2 = 18;
const int IR_3 = 16;
const int IR_4 = 17;

int readings[5] = {1, 1, 0, 1, 1};

const float bigDeviation = .30;


/*    END OF IR SENSOR   */

/*         LDR           */

const int LDR_0 = 5;
const int LDR_1 = 2;
const int LDR_2 = 0;

const int LDR_Go = 90000;

int LDR_value[3] = {0,0,0};

/*       END OF LDR      */

int counter;


/*         Function declaration         */
void set_pwm(float pwm, char motor);
int percent_to_pwm(float percent);
void read_IR();
int control_if();
void control_PID();
void calc_PID();
void motor_PID();
void go();
void checkEnd();



// Define interrupt service routines (ISRs)
void IRAM_ATTR Right_Encoder_Int() 
{
  R_Encoder++;
}

void IRAM_ATTR Left_Encoder_Int() 
{
  L_Encoder++;
}



void setup() 
{
Serial.begin(115200);

    // Configure the timer
  esp_timer_create_args_t timer_args = {
    .callback = &timer_callback,
    .name = "periodic_timer"
  };

  esp_timer_create(&timer_args, &timer);

  // Start the timer (10ms = 10000 microseconds)
  esp_timer_start_periodic(timer, delaytime * 1000);

  // Configure interrupt pins as inputs
  pinMode(ENC_R_pin, INPUT_PULLUP); // Or INPUT, INPUT_PULLDOWN, depending on your circuit
  pinMode(ENC_L_pin, INPUT_PULLUP); // Or INPUT, INPUT_PULLDOWN, depending on your circuit

  // Attach interrupts to the pins
  attachInterrupt(digitalPinToInterrupt(ENC_R_pin), Right_Encoder_Int, FALLING); // Or RISING, CHANGE, LOW, HIGH, depending on your needs.
  attachInterrupt(digitalPinToInterrupt(ENC_L_pin), Left_Encoder_Int , FALLING); // Or RISING, CHANGE, LOW, HIGH, depending on your needs.

  //MOTORS
  
  pinMode(pin_MotorL_1, OUTPUT);
  pinMode(pin_MotorL_2, OUTPUT);
  pinMode(pin_MotorR_1, OUTPUT);
  pinMode(pin_MotorR_2, OUTPUT);
  pinMode(Motor_enable, OUTPUT);


  ledcAttachChannel(pin_MotorR_1, 500, resolution, MotorR_1);
  ledcAttachChannel(pin_MotorR_2, 500, resolution, MotorR_2);
  ledcAttachChannel(pin_MotorL_1, 500, resolution, MotorL_1);
  ledcAttachChannel(pin_MotorL_2, 500, resolution, MotorL_2);

  //Initialize motors at 
  set_pwm(0, 'L');
  set_pwm(0, 'R');


  digitalWrite(Motor_enable, 1);

  //END OF MOTORS


  //IR SENSOR
  pinMode(IR_0, INPUT_PULLUP);
  pinMode(IR_1, INPUT_PULLUP);
  pinMode(IR_2, INPUT_PULLUP);
  pinMode(IR_3, INPUT_PULLUP);
  pinMode(IR_4, INPUT_PULLUP);

  //END OF IR SENSOR


  //LDR

  pinMode(LDR_0, OUTPUT);
  pinMode(LDR_1, INPUT);
  pinMode(LDR_2, OUTPUT);

  digitalWrite(LDR_0, 1);
  digitalWrite(LDR_0, 0);
  
  //END OF LDR



}

void loop() 
{
  /*
  go();

  read_IR();

  if( control() == 0) delay(100000);

  if (DEBUG)
  {
  Serial.print("R0: ");
  Serial.print(readings[0]);
  Serial.print(" R1: ");
  Serial.print(readings[1]);
  Serial.print(" R2: ");
  Serial.print(readings[2]);
  Serial.print(" R3: ");
  Serial.print(readings[3]);
  Serial.print(" R4: ");
  Serial.println(readings[4]);
  }

  delay(delaytime);
  */



}

void timer_callback(void* arg) 
{
  counter ++;
  
  read_IR();

  motor_PID();
  calc_PID();
  
  control_PID();
  
  if(counter > 100) checkEnd();
  
  Serial.print("PcR: ");
  Serial.print(Percent_R);
  Serial.print("  PcL: ");
  Serial.print(Percent_L);
  Serial.print("  PID: ");
  Serial.println(PID + PID_motor);

  Serial.print("  ");
  Serial.print(readings[0]);
  Serial.print("  ");
  Serial.print(readings[1]);
  Serial.print("  ");
  Serial.print(readings[2]);
  Serial.print("  ");
  Serial.print(readings[3]);
  Serial.print("  ");
  Serial.println(readings[4]);


}

/*
percent 0 to 100
dir F or B
motor R or L
*/
void set_pwm(float percent, char motor)
{
  
  if (percent > MAX_PERCENT_R and motor == 'R') percent = MAX_PERCENT_R;
  if (percent > MAX_PERCENT_L and motor == 'L') percent = MAX_PERCENT_L;

  if (percent < -MAX_PERCENT_R and motor == 'R') percent = -MAX_PERCENT_R;
  if (percent < -MAX_PERCENT_L and motor == 'L') percent = -MAX_PERCENT_L;


  int pwm = percent_to_pwm(percent);
  
  if (DEBUG)
  {
    Serial.print("Setting");
    Serial.print(motor);
    Serial.print(" at pc: ");
    Serial.print(percent);
    Serial.print(" pwm: ");
    Serial.println(pwm);
  }


  if(motor == 'R' )
  {
    if (percent >= 0)
    {
      ledcWrite(pin_MotorR_1 , 0);
      ledcWrite(pin_MotorR_2 , pwm);  
    }

    if (percent < 0)
    {
      ledcWrite(pin_MotorR_1 , -pwm);
      ledcWrite(pin_MotorR_2 , 0);  
    }
    
  }

  if(motor == 'L' )
  {
    if (percent >= 0)
    {
      ledcWrite(pin_MotorL_1 , 0);
      ledcWrite(pin_MotorL_2 , pwm);  
    }

    if (percent < 0)
    {
      ledcWrite(pin_MotorL_1 , -pwm);
      ledcWrite(pin_MotorL_2 , 0);  
    }
    
  }

}

int percent_to_pwm(float percent)
{
  int pwm = percent * maxpwm / 100;
  if (DEBUG)
  {
  Serial.print("PWM: ");
  Serial.println(pwm);
  }
  return pwm;
}

void read_IR()
{

  readings[0] = digitalRead(IR_0); ;
  readings[1] = digitalRead(IR_1); ;
  readings[2] = digitalRead(IR_2); ;
  readings[3] = digitalRead(IR_3); ;
  readings[4] = digitalRead(IR_4); ;

  if(DEBUG)
  {
    Serial.print  (readings[0]);
    Serial.print  (" ");
    Serial.print  (readings[1]);
    Serial.print  (" ");
    Serial.print  (readings[2]);
    Serial.print  (" ");
    Serial.print  (readings[3]);
    Serial.print  (" ");
    Serial.println(readings[4]);
  }

}

void calc_PID()
{

  error = (readings[0] * (-2) + readings[1] * (-1) + readings[2] * (0) + readings[3] * (1) + readings[4] * (2));

  P = error;
  I = I + error;
  D = lastError - error;
  lastError = error;
  if(I > max_windup) I = max_windup;
  if(I < -max_windup) I = -max_windup;


  PID = P * Kp + I * Ki + D * Kd;

}

void control_PID()
{
  
  Percent_L = BaseSpeedL - PID - PID_motor;
  Percent_R = BaseSpeedR + PID + PID_motor;

  Percent_L = constrain(Percent_L, 0, MAX_PERCENT_L);
  Percent_R = constrain(Percent_R, 0, MAX_PERCENT_R);

  set_pwm(Percent_L, 'L' );
  set_pwm(Percent_R, 'R' );


}

void motor_PID()
{
  error_motor = L_Encoder - R_Encoder + PID;

  P_motor = error_motor;
  I_motor = I_motor + error_motor;
  D_motor = lastError_motor - error_motor;
  lastError_motor = error_motor;
  if(I > max_windup_motor) I = max_windup_motor;
  if(I < -max_windup_motor) I = -max_windup_motor;

  PID_motor = P_motor * Kp_motor + I_motor * Ki_motor + D_motor * Kd_motor;

}

void go()
{
  int LDR_value = 0;

  while(1)
  {

    LDR_value = analogRead(LDR_1);

    if (DEBUG) 
    {
    Serial.print("LDR: ");
    Serial.println(LDR_value);
    }

    if (LDR_value > LDR_Go){
      digitalWrite(Motor_enable, 1);
      break;
    } 

  }

}


void checkEnd()
{
  int count = 0;
  for(int i = 0; i < 5 ; i++) if (readings[i] == 0) count ++;

  if (count == 5)
  {
  set_pwm(0, 'R');
  set_pwm(0, 'L');
  delay(100000);
  }


}
