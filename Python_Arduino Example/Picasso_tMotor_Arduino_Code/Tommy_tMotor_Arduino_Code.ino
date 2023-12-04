//Required Files Motor_Control_Tmotor.cpp, Motor_Control_Tmotor.h, Serial_Isra.cpp, Serial_Isra.h

#include <FlexCAN_T4.h>
#include "Motor_Control_Tmotor.h"
#include "Serial_Isra.h"

/*Canbus Setup*/
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
CAN_message_t msgR;
/*Canbus Setup*/

/*Motor Setup*/
int Motor_ID = 1;
int CAN_ID = 3;
double torque_command = 0;
double velocity_command = 0;
double position_command = 0;

float p_des = 0;
float v_des = 0;
float kp = 0;
float kd = 0;
float t_ff = 0;

Motor_Control_Tmotor m1(Motor_ID, CAN_ID);
/*Motor Setup*/

/*Time Control*/
unsigned long Delta_T1 = 4000;
unsigned long t_pr1;
double HZ = 1.0 / (Delta_T1 / 100000.0);
double current_time = 0;
unsigned long beginning = 0;
unsigned long input_delay_control = 2000;
/*Time Control*/

/*Isra Serial Class Setup*/
Serial_Isra Serial_Isra;
//Check Serial_Isra.h to ensure connection to correct serial port on PCB
/*Isra Serial Class Setup*/

/*Serial Send/Recieve*/
size_t Send_Length = 11;
char Send[11] = {0xaa, 0xbb, 0x33, 0x33, 0xcc, 0x33, 0x33, 0xdd, 0x33, 0x33, 0xee};
char SerialData2[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x33};
/*Serial Send/Recieve*/

/*Motor Send/Receive Variables*/
uint16_t position_int = 0x0000;
uint16_t speed_int = 0x0000;
uint16_t torque_int = 0x0000;
uint16_t Torque_Command_Uint = 0x0000;
float Torque_Command = 0;
/*Motor Send/Recieve Variables*/

void setup() {
  delay(3000);
  Serial.begin(115200);
  Serial6.begin(115200);
  Serial_Isra.INIT();
  initial_CAN();
  beginning = micros();
  Serial.print("Serial HZ: ");
  Serial.println(HZ);
  Serial.println("SETUP DONE");
  delay(100);
}

void loop() {

  current_time = micros() - beginning;

  //Receive Current Command
  Serial_Isra.READ2();
  Torque_Command_Uint = (Serial_Isra.SerialData2[4] << 8 | Serial_Isra.SerialData2[3]);
  Torque_Command = Serial_Isra.uint_to_float(Torque_Command_Uint, -10, 10, 16);

  //Send Current Command to Motor
  m1.send_cmd( 0, 0, 0, 0, Torque_Command);

  if (current_time - t_pr1 > Delta_T1) {
    t_pr1 = current_time;

    receive_CAN_data();
    Wait(input_delay_control);

    float Position = m1.pos;
    float Speed = m1.spe;
    float Torque = m1.torque;

    position_int = Serial_Isra.float_to_uint(Position, 0, 360, 16);
    speed_int = Serial_Isra.float_to_uint(Speed, -25, 25, 16);
    torque_int = Serial_Isra.float_to_uint(Torque, -10, 10, 16);

    //Assign Motor Variables to Message
    Send[2] = torque_int >> 8;
    Send[3] = torque_int & 0xFF;
    Send[5] = speed_int >> 8;
    Send[6] = speed_int & 0xFF;
    Send[8] = position_int >> 8;
    Send[9] = position_int & 0xFF;

    //Send Message
    Serial_Isra.WRITE(Send, Send_Length);

    

    //Print Statements for Debug
    Serial.print("Torque Command:  ");
    Serial.print(Torque_Command);
    Serial.print(" Torque:  ");
    Serial.print(Torque);
    Serial.print(" Position: ");
    Serial.print(Position);
    Serial.print(" Speed: ");
    Serial.println(Speed);
    Serial.println();
    //Print Statements for Debug

  }
}

void initial_CAN()
{
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(400);
  Serial.println("Can bus setup done...");
  delay(200);
  m1.initial_CAN();
  m1.exit_control_mode();
  delay(200);
  m1.exit_control_mode();
  delay(1000);
  m1.enter_control_mode();
  delay(200);
  receive_CAN_data();
  delay(200);
  m1.set_origin();
  delay(200);
  receive_CAN_data();
  delay(2);
  m1.set_origin();
  delay(2);
  m1.set_origin();
  delay(2);
}

void receive_CAN_data()
{
  if (Can3.read(msgR))
  {
    Can3.read(msgR);
    int id = msgR.buf[0];
    Serial.print(msgR.id, HEX );
    if (id == Motor_ID)
    {
      m1.unpack_reply(msgR);
    }
  }
}

void Wait(unsigned long delay_control)
{
  unsigned long Time_start = micros();
  unsigned long Time_Delta = delay_control;
  unsigned long Time_Control = 0;

  do {
    Time_Control = micros() - Time_start;
  }
  while (Time_Control < Time_Delta);

}

void Position_Control_Example()
{
  double t = millis() / 1000.0;
  position_command = 45 * sin(t/2);
  p_des = position_command * PI / 180;
  v_des = 0; //dont change this
  kp = 200; //max 450 min 0
  kd = 3; //max 5 min 0
  t_ff = 0; //dont change this
  m1.send_cmd( p_des, v_des, kp, kd, t_ff);
  receive_CAN_data();
  //Serial.print(position_command);
  double v1 = 90;
  double v2 = -90;
  Serial.print(v1);
  Serial.print("   ");
  Serial.print(v2);
  Serial.print("   ");
  Serial.print(m1.pos * 180 / PI);
  Serial.print("   ");
  Serial.print(position_command);
  Serial.println("   ");
  //Serial.println(m1.temp);
}
