
#include <SPI.h>

#define RPM_TO_RADS 0.104719
//buffer sizes..note that we add padding to make it 4 byte aligned
#define SERIAL_BUF_SZ 12
#define SERIAL_RD_BUF_SZ 12
#define OFF_TICK_SPACING 50000
//delay in ms before changing the direction of a wheel
#define DIR_CHANGE_DELAY 50

// ADDR:0b0000 + CMD:0b00 + UNUSED:0b00
uint8_t POT_WRITE_CMD = 0b00000000; 
uint16_t MAX_WIPER_STEPS = 129;
int MOTOR_MAX = 85;
int MOTOR_MIN = 0;

int SPI_SS = 10; // SS pin for duemilanove
int CS1 = A5;  // any GPIO can be used here
int CS2 = A4;  // any GPIO can be used here

// for reversing the direction of the motor
int REVERSE_PIN1 = A3;
int REVERSE_PIN2 = A2;

//encodeer
int ENC1 = 3;
int ENC2 = 2;
volatile unsigned long enc1_tick = 0;
volatile unsigned long enc2_tick = 0;

//Current tick spacing. Give it a very slow initial value to kick off the movement
volatile uint32_t curr_m1 = OFF_TICK_SPACING;
volatile uint32_t curr_m2 = OFF_TICK_SPACING;

//we don't want to use the first tick to calc tick spacing so track that
volatile bool done_1tick_m1 = 0;
volatile bool done_1tick_m2 = 0;

byte set_m1_dir = 1;
byte set_m2_dir = 1;
uint32_t set_m1 = OFF_TICK_SPACING;
uint32_t set_m2 = OFF_TICK_SPACING;

//control loop
unsigned long control_time = 0;
int32_t ctrl_cmd_1 = 0;
int32_t ctrl_cmd_2 = 0;
int32_t error1 = 0;
int32_t error2 = 0;
int32_t error1_accum = 0;
int32_t error2_accum = 0;
bool active = 0; //only kick things off after first cmd

//Control constants. Dealing with inverse to avoid floats
//int P_INV = -10;
//int I_INV = -60;
int P_INV = -1600;
int I_INV = -9600;

//serialize data
byte serial_buf[SERIAL_BUF_SZ];
byte serial_rd_buf[SERIAL_RD_BUF_SZ];

void setup() {
  // set the SS to OUTPUT so that we are in Master mode
  pinMode(SPI_SS, OUTPUT);

  // disable comm with the motors
  pinMode(CS1, OUTPUT);
  digitalWrite(CS1, HIGH);
  
  pinMode(CS2, OUTPUT);
  digitalWrite(CS2, HIGH);  

  //init pin for reverse
  pinMode(REVERSE_PIN1, OUTPUT);  
  pinMode(REVERSE_PIN2, OUTPUT);
  //digitalWrite(REVERSE_PIN1, HIGH);
  //digitalWrite(REVERSE_PIN2, HIGH);  

  // set parameters for SPI
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0); 

  //turn off the motors
  send_cmd(0, 0, 1, 1);

  //start serial comm with host  
  Serial.begin(115200);  

  //encoder ticks
  pinMode(ENC1, INPUT);
  pinMode(ENC2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1), edge1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2), edge2, CHANGE);  
}

void edge1()
{
  if (micros() - enc1_tick > 1000)
  {
    if (done_1tick_m1)
      curr_m1 = micros() - enc1_tick; 
    else
       done_1tick_m1 = 1;   
    enc1_tick = micros();
  }
}

void edge2()
{
  if (micros() - enc2_tick > 1000)
  {
    if (done_1tick_m2)
      curr_m2 = micros() - enc2_tick;
    else
       done_1tick_m2 = 1;
    enc2_tick = micros();
  }
}

void send_cmd(int motor1, int motor2, bool motor1_fwd, bool motor2_fwd)
{  
  static bool last_motor1_fwd;
  static bool last_motor2_fwd;
  //check limits
  motor1 = motor1 <= MOTOR_MAX ? motor1 : MOTOR_MAX;
  motor1 = motor1 >= MOTOR_MIN ? motor1 : MOTOR_MIN;
  motor2 = motor2 <= MOTOR_MAX ? motor2 : MOTOR_MAX;
  motor2 = motor2 >= MOTOR_MIN ? motor2 : MOTOR_MIN;

  //if we are switching the dir of any motor se it to zero then delay
  if ( last_motor1_fwd != motor1_fwd || last_motor2_fwd != motor2_fwd)
  {
    if(last_motor1_fwd != motor1_fwd)
    {
      //turn off motor 1
      digitalWrite(CS1, LOW);
      SPI.transfer(POT_WRITE_CMD);
      SPI.transfer(MOTOR_MAX);  //this turns it off since values are flipped
      digitalWrite(CS1, HIGH);      
    }
    if(last_motor2_fwd != motor2_fwd)
    {
      //turn off motor 2
      digitalWrite(CS2, LOW);
      SPI.transfer(POT_WRITE_CMD);
      SPI.transfer(MOTOR_MAX);  //this turns it off since values are flipped
      digitalWrite(CS2, HIGH);      
    }
    last_motor1_fwd = motor1_fwd;
    last_motor2_fwd = motor2_fwd;
    delay(DIR_CHANGE_DELAY);

    digitalWrite(REVERSE_PIN1, motor1_fwd ? HIGH : LOW);  
    digitalWrite(REVERSE_PIN2, motor2_fwd ? HIGH : LOW);
    delay(DIR_CHANGE_DELAY);
  } 
  else
  {
    //set the direction
    digitalWrite(REVERSE_PIN1, motor1_fwd ? HIGH : LOW);  
    digitalWrite(REVERSE_PIN2, motor2_fwd ? HIGH : LOW);    
  } 

  //the connections are flipped so reverse the cmd
  motor1 = MOTOR_MAX - motor1;
  motor2 = MOTOR_MAX - motor2;

  //set motor 1
  digitalWrite(CS1, LOW);
  SPI.transfer(POT_WRITE_CMD);
  SPI.transfer(motor1);
  digitalWrite(CS1, HIGH);

  //set motor 2
  digitalWrite(CS2, LOW);
  SPI.transfer(POT_WRITE_CMD);
  SPI.transfer(motor2); 
  digitalWrite(CS2, HIGH);
}

void loop() 
{
  //delay(1000);
  //send_cmd(0, 0, 1, 1);/*
  if(active && millis() >=  control_time)
  {
    // calc the new cmd signals
    error1 = set_m1 - curr_m1;
    error2 = set_m2 - curr_m2;
    error1_accum += error1;
    error2_accum += error2;    

    ctrl_cmd_1 = (error1/P_INV) + (error1_accum/I_INV);
    ctrl_cmd_2 = (error2/P_INV) + (error2_accum/I_INV);

    //check limits
    ctrl_cmd_1 = ctrl_cmd_1 <= MOTOR_MAX ? ctrl_cmd_1 : MOTOR_MAX;
    ctrl_cmd_1 = ctrl_cmd_1 >= MOTOR_MIN ? ctrl_cmd_1 : MOTOR_MIN;
    ctrl_cmd_2 = ctrl_cmd_2 <= MOTOR_MAX ? ctrl_cmd_2 : MOTOR_MAX;
    ctrl_cmd_2 = ctrl_cmd_2 >= MOTOR_MIN ? ctrl_cmd_2 : MOTOR_MIN;
    send_cmd(ctrl_cmd_1, ctrl_cmd_2, set_m1_dir, set_m2_dir); 

    //serialize and send to host
    serial_buf[0] = 0;
    serial_buf[1] = (uint8_t)ctrl_cmd_1;
    serial_buf[2] = 0;
    serial_buf[3] = (uint8_t)ctrl_cmd_2;
    serial_buf[4] = (curr_m1 >> 24) & 0xFF;
    serial_buf[5] = (curr_m1 >> 16) & 0xFF;
    serial_buf[6] = (curr_m1 >> 8) & 0xFF;
    serial_buf[7] = curr_m1 & 0xFF;    
    serial_buf[8] = (curr_m2 >> 24) & 0xFF;
    serial_buf[9] = (curr_m2 >> 16) & 0xFF;
    serial_buf[10] = (curr_m2 >> 8) & 0xFF;
    serial_buf[11] = curr_m2 & 0xFF; 
    Serial.write(serial_buf, SERIAL_BUF_SZ);    

    control_time = millis() + 20;
  }

  // check for command data
  if (Serial.available())
  {
    Serial.readBytes(serial_rd_buf, SERIAL_RD_BUF_SZ);
    set_m1_dir = serial_rd_buf[1];
    set_m2_dir = serial_rd_buf[3];
    set_m1 = (serial_rd_buf[4] << 24) | (serial_rd_buf[5] << 16) |
             (serial_rd_buf[6] << 8) |  serial_rd_buf[7];
    set_m2 = (serial_rd_buf[8] << 24) | (serial_rd_buf[9] << 16) |
             (serial_rd_buf[10] << 8) |  serial_rd_buf[11];

    if( set_m1 == 0 && set_m2 == 0)
    {
      active = 0;
      ctrl_cmd_1 = 0;
      ctrl_cmd_2 = 0;
      curr_m1 = OFF_TICK_SPACING;
      curr_m2 = OFF_TICK_SPACING;      
      error1_accum = 0;
      error2_accum = 0;
      send_cmd(ctrl_cmd_1, ctrl_cmd_2, set_m1_dir, set_m2_dir);
    }
    else if( active == 0 && (set_m1 != 0 || set_m2 != 0))
    {
      active = 1;

      //reset tick count for new session
      curr_m1 = OFF_TICK_SPACING;
      curr_m2 = OFF_TICK_SPACING;
      done_1tick_m1 = 0;
      done_1tick_m2 = 0;
    }

    /*loopback test
    ctrl_cmd_1 = set_m1_dir;
    ctrl_cmd_2 = set_m2_dir;
    curr_m1 = set_m1;
    curr_m2 = set_m2;*/   
    
  }//*/
}


