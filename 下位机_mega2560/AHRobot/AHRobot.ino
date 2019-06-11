
#include "Configuration.h"
#include "Definitions.h"   
void setup() 
{ 
  // STEPPER MOTOR PINS (SAME AS RAMPS 1.4)
  // X MOTOR
  //     X-STEP: A0    (PF0)
  //     X-DIR:  A1    (PF1)
  //     X-ENABLE: D38 (PD7)
  // Y MOTOR (Y-LEFT)
  //     Y-STEP: A6    (PF6)
  //     Y-DIR:  A7    (PF7)
  //     Y-ENABLE: A2  (PF2)
  // Z MOTOR (Y-RIGHT)
  //     Z-STEP: D46   (PL3)
  //     Z-DIR:  D48   (PL1)
  //     Z-ENABLE: A8  (PK0)

  // STEPPER PINS 
  // X_AXIS
  pinMode(38,OUTPUT);  // ENABLE MOTOR
  pinMode(A0,OUTPUT);  // STEP MOTOR
  pinMode(A1,OUTPUT);  // DIR MOTOR
  // Y_AXIS (Y-LEFT)
  pinMode(A2,OUTPUT);  // ENABLE MOTOR
  pinMode(A6,OUTPUT);  // STEP MOTOR
  pinMode(A7,OUTPUT);  // DIR MOTOR
  // Z_AXIS (Y-RIGHT)
  pinMode(A8,OUTPUT);  // ENABLE MOTOR
  pinMode(46,OUTPUT);  // STEP MOTOR
  pinMode(48,OUTPUT);  // DIR MOTOR 

  pinMode(A3,OUTPUT);  // DEBUG PIN FOR OSCILLOSCOPE TIME MEASURES

  pinMode(19,INPUT);  // RX1 Serial Port 1
  pinMode(18,OUTPUT); // TX1

  //FANS and LEDS
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(13,OUTPUT);

  // Disable Motors
  digitalWrite(38,HIGH);
  digitalWrite(A2,HIGH);
  digitalWrite(A8,HIGH);

  Serial.begin(115200);
  Serial.println("AHR Robot version 1.05");
  delay(100);
  Serial.println("Initializing robot...");
  delay(100);

  cameraProcessInit();  // Camera variable initializations

  //LED blink
  for (uint8_t k=0;k<4;k++)
  {
    digitalWrite(13,HIGH);
    delay(300);
    digitalWrite(13,LOW);
    delay(300);
  } 

  // We use TIMER 1 for stepper motor X AXIS and Timer 3 for Y AXIS
  // STEPPER MOTORS INITIALIZATION
  // TIMER1 CTC MODE
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11); 
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0); 
  TCCR1A &= ~(3<<COM1B0); 

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

  OCR1A = ZERO_SPEED;   // Motor stopped
  dir_x = 0;
  TCNT1 = 0;

  // We use TIMER 3 for stepper motor Y AXIS 
  // STEPPER MOTORS INITIALIZATION
  // TIMER3 CTC MODE
  TCCR3B &= ~(1<<WGM13);
  TCCR3B |=  (1<<WGM12);
  TCCR3A &= ~(1<<WGM11); 
  TCCR3A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR3A &= ~(3<<COM1A0); 
  TCCR3A &= ~(3<<COM1B0); 

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR3B = (TCCR3B & ~(0x07<<CS10)) | (2<<CS10);

  OCR3A = ZERO_SPEED;   // Motor stopped
  dir_y = 0;
  TCNT3 = 0;

  delay(100);

  Serial.println("Initializing Stepper motors...");
  delay(100);
  TIMSK1 |= (1<<OCIE1A);  // Enable Timer1 interrupt
  TIMSK3 |= (1<<OCIE1A);  // Enable Timer1 interrupt

  // Enable steppers
  digitalWrite(38,LOW);   // X-axis
  digitalWrite(A2,LOW);   // Y-axis left
  digitalWrite(A8,LOW);   // Z-axis (Y-axis right)

  // Output parameters
  Serial.print("Max_acceleration_x: ");
  Serial.println(max_acceleration_x);
  Serial.print("Max_acceleration_y: ");
  Serial.println(max_acceleration_y);
  Serial.print("Max speed X: ");
  Serial.println(MAX_SPEED_X);
  Serial.print("Max speed Y: ");
  Serial.println(MAX_SPEED_Y);
  Serial.println("Moving to initial position...");
  Serial.println("Ready!!");
  delay(100);

  // Enable Air Hockey FANS
  //digitalWrite(9,HIGH);
  //digitalWrite(10,HIGH);
  analogWrite(9, FAN1_SPEED);
  analogWrite(10, FAN2_SPEED);

  position_x = ROBOT_CENTER_X*X_AXIS_STEPS_PER_UNIT;
  position_y = ROBOT_DEFENSE_POSITION*Y_AXIS_STEPS_PER_UNIT;

  timer_old = micros();
  timer_packet_old = timer_old;
  micros_old = timer_old;
}

void loop()
{
	//int dt;
	debug_counter++;
	timer_value = micros();//��ȡ����ʱ��
	if ((timer_value-timer_old)>=1000)  // 1Khzѭ��
	{
		//dt = (timer_value-timer_old)/1000;
		timer_old = timer_value;

		loop_counter++;//ѭ������+1

		packetRead();//��ȡ����
		if (newPacket)//���յ�����Ϣ
		{
		//	dt = (timer_value-timer_packet_old)/1000.0;
		//	dt = 16;  //60 Hz = 16.66ms
			newPacket=0;

			robot_status = (uint8_t)puckSize;
			predict_x = (puckPixX - 51) / 1.15528;
			predict_y = (puckPixY - 13) / 1.15385 + 20;

			missingStepsDetection(robotPixX, robotPixY);//��ⲽ����ʧλ�Ʋ�У��
		}

		robotStrategy();//���ԣ�������Ӧִ�еĶ���

		positionControl();//λ�ÿ������
	}
}





