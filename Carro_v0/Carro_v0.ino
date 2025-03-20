
/*     MOTORS       */
const int pin_MotorR_1 = 5;    
const int pin_MotorR_2 = 19;  

const int pin_MotorL_1 = 16;
const int pin_MotorL_2 = 17;

const int MotorR_1 = 0;
const int MotorR_2 = 1;

const int MotorL_1 = 2;
const int MotorL_2 = 3;

const int Motor_enable = 4;

const int resolution = 10;
const float maxpwm = 1024 - 1;

volatile float Percent_R = 0;
volatile float Percent_L = 0;

const float MAX_PERCENT_R = 100;
const float MAX_PERCENT_L = 100;
/*    END OF MOTORS   */


/*     IR SENSOR       */

const int IR_0 = 12;
const int IR_1 = 14;
const int IR_2 = 27;
const int IR_3 = 26;
const int IR_4 = 25;

const int IR_enable = 33;

int readings[5] = {0, 0, 0, 0, 0};


/*    END OF IR SENSOR   */
const int delaytime = 200;

void set_pwm(float pwm, char dir, char motor);
int percent_to_pwm(float percent);
void read_IR();

/*
void IRAM_ATTR isr() {
	button1.numberKeyPresses++;
	button1.pressed = true;
}
*/

void setup() {

  //MOTORS
  
  pinMode(pin_MotorL_1, OUTPUT);
  pinMode(pin_MotorL_2, OUTPUT);
  pinMode(pin_MotorR_1, OUTPUT);
  pinMode(pin_MotorR_2, OUTPUT);
  pinMode(Motor_enable, OUTPUT);


  ledcAttachChannel(pin_MotorR_1, 2000, resolution, MotorR_1);
  ledcAttachChannel(pin_MotorR_2, 2000, resolution, MotorR_2);
  ledcAttachChannel(pin_MotorL_1, 2000, resolution, MotorL_1);
  ledcAttachChannel(pin_MotorL_2, 2000, resolution, MotorL_2);

  digitalWrite(Motor_enable, HIGH);

  //Initialize motors at 0
  set_pwm(Percent_L, 'F', 'L');
  set_pwm(Percent_R, 'F', 'R');

  //END OF MOTORS



  //IR SENSOR
  pinMode(IR_0, INPUT_PULLUP);
  pinMode(IR_1, INPUT_PULLUP);
  pinMode(IR_2, INPUT_PULLUP);
  pinMode(IR_3, INPUT_PULLUP);
  pinMode(IR_4, INPUT_PULLUP);
  pinMode(IR_enable, OUTPUT);

  digitalWrite(IR_enable, HIGH);
  //END OF IR SENSOR

//

Serial.begin(115200);
}

void loop() 
{

set_pwm(98.5, 'F', 'R');
set_pwm(98.2, 'B', 'L');

}


/*
percent 0 to 100
dir F or B
motor R or L
*/
void set_pwm(float percent, char dir, char motor){
  

  int pwm = percent_to_pwm(percent);

  if( dir == 'F' and motor == 'R' )
  {
    ledcWrite(pin_MotorR_1 , pwm);
    ledcWrite(pin_MotorR_2 , 0);
  }

  if( dir == 'F' and motor == 'L' )
  {
    ledcWrite(pin_MotorL_1 , pwm);
    ledcWrite(pin_MotorL_2 , 0);
  }

  if( dir == 'B' and motor == 'R' )
  {
    ledcWrite(pin_MotorR_1 , 0);
    ledcWrite(pin_MotorR_2 , pwm);
  }

  if( dir == 'B' and motor == 'L' )
  {
    ledcWrite(pin_MotorL_1 , 0);
    ledcWrite(pin_MotorL_2 , pwm);
  }
}


int percent_to_pwm(float percent){
  int pwm = percent * maxpwm / 100;
  Serial.print("PWM: ");
  Serial.println(pwm);
  return pwm;
}

void read_IR()
{

  readings[0] = digitalRead(IR_0);
  readings[1] = digitalRead(IR_1);
  readings[2] = digitalRead(IR_2);
  readings[3] = digitalRead(IR_3);
  readings[4] = digitalRead(IR_4);

}

void control()
{
  if (readings[0] == 1)
  {
    if (Percent_R > 0)
    {
      Percent_R = Percent_R - 5;
      set_pwm(Percent_R, 'F', 'R');
    }
    if (Percent_L < MAX_PERCENT_L)
    {
      Percent_L = Percent_L + 5;
      set_pwm(Percent_L, 'F', 'L');
    }
  }

  if (readings[1] == 1)
  {
    if (Percent_R > 0)
    { 
      Percent_R = Percent_R - 2;
      set_pwm(Percent_R, 'F', 'R');
    }
    if (Percent_L < MAX_PERCENT_L)
    {
      Percent_L = Percent_L + 2;
      set_pwm(Percent_L, 'F', 'L');
    }
  }

  if (readings[2] == 1)
  {
    if (Percent_R < MAX_PERCENT_R)
    {
      Percent_R = Percent_R + 5;
      set_pwm(Percent_R, 'F', 'R');
    }

    if (Percent_L < MAX_PERCENT_L)
    {
      Percent_L = Percent_L + 5;
      set_pwm(Percent_L, 'F', 'L');
    }
  }

  if (readings[3] == 1)
  {
    if (Percent_R < MAX_PERCENT_R)
    {
      Percent_R = Percent_R + 2;
      set_pwm(Percent_R, 'F', 'R');
    }
    if (Percent_L > 0)
    {
      Percent_L = Percent_L - 2;
      set_pwm(Percent_L, 'F', 'L');
    }
  }

  if (readings[4] == 1)
  {
    if (Percent_R < MAX_PERCENT_R)
    {
      Percent_R = Percent_R + 5;
      set_pwm(Percent_R, 'F', 'R');
    }
    if (Percent_L > 0)
    {
      Percent_L = Percent_L - 5;
      set_pwm(Percent_L, 'F', 'L');
    }
  }

  if (readings[0] == 1 and readings[1] == 1 and readings[2] == 1 and readings[3] == 1 and readings[4] == 1)
  {
    Percent_R = 0;
    Percent_L = 0;
    set_pwm(Percent_R, 'F', 'R');
    set_pwm(Percent_L, 'F', 'L');
  }

}