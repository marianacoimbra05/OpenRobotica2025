const int ReadPin=14;
const int WritePin=12;

const int MotorR_1 = 16;    
const int MotorR_2 = 17;  

const int MotorL_1 = 18;
const int MotorL_2 = 19;

//Interrupts
struct Button {
	const uint8_t PIN;
	uint32_t numberKeyPresses;
	bool pressed;
};

Button button1 = {18, 0, false};

void IRAM_ATTR isr() {
	button1.numberKeyPresses++;
	button1.pressed = true;
}

void set_pwm(int pwm, char dir, char motor){
  //pwm 0 a 255
  //dir F or B
  //motor R or L

  if( dir == 'F' and motor == 'R' )
  {
    analogWrite(MotorR_1 , pwm);
    analogWrite(MotorR_2 , 0);
  }

  if( dir == 'F' and motor == 'L' )
  {
    analogWrite(MotorL_1 , pwm);
    analogWrite(MotorL_2 , 0);
  }

  if( dir == 'B' and motor == 'R' )
  {
    analogWrite(MotorR_1 , 0);
    analogWrite(MotorR_2 , pwm);
  }

  if( dir == 'B' and motor == 'L' )
  {
    analogWrite(MotorL_1 , 0);
    analogWrite(MotorL_2 , pwm);
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(WritePin, OUTPUT);
  pinMode(ReadPin, INPUT_PULLDOWN);

  pinMode(MotorR_1, OUTPUT);
  pinMode(MotorR_2, OUTPUT);

  pinMode(MotorL_1, OUTPUT);
  pinMode(MotorL_2, OUTPUT);

  Serial.begin(115200);

  //initialize motors at 0
  set_pwm(0, 'F', 'L');
  set_pwm(0, 'F', 'R');



//Interrupts
pinMode(button1.PIN, INPUT_PULLUP);
attachInterrupt(button1.PIN, isr, FALLING);
//
}

void loop() {
  // put your main code here, to run repeatedly:

/*  
  digitalWrite(WritePin, HIGH);
  //void analogWrite(2, 180);
  int a=digitalRead(ReadPin);

  if (a==1){
    Serial.println("Hello World");
  }
  else 
    Serial.println("Not Hello World");
  a=0;
*/

  set_pwm(0, 'F', 'L');
  set_pwm(0, 'F', 'R');


  delay(5000);

  set_pwm(255, 'F', 'L');
  set_pwm(255, 'F', 'R');

  delay(5000);

  set_pwm(0, 'B', 'L');
  set_pwm(0, 'B', 'R');

  delay(5000);

  set_pwm(255, 'B', 'L');
  set_pwm(255, 'B', 'R');

  delay(5000);

  //void analogWriteResolution(uint8_t pin, uint8_t resolution);
  //void analogWriteFrequency(uint8_t pin, uint32_t freq);
/*
	if (button1.pressed) {
		Serial.printf("Button has been pressed %u times\n", button1.numberKeyPresses);
		button1.pressed = false;
	}
*/
}
