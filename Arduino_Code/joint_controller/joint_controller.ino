// For EE543 project, the amount of joints is 4
/*This version of firmware is for the arduino that will reset when the serial port is connected*/
#define JOINT_NUM 4

// Define communication protocal
#define INIT_UNO 'I'
#define ACK_UNO 'A' 
#define SIG_PC 'S'
char dump;

// Define state machine for serial communication
enum serial_states{IDLE, INIT, READY, STREAM, TIMEOUT};
enum serial_states serial_state;
// Define flag for event
bool PC_signal_flag = false;
bool serial_timeout_flag = false;

// Debug pin
// hook on the oscilloscope to confirm the frequency
const int debugPin_idle = 2;
const int debugPin_init = 3;
const int debugPin_ready = 4;
const int debugPin_stream = 5;
const int debugPin_timeout = 6;

void debug_pin_init()
{
  // set the digital pin as output:
  pinMode(debugPin_idle, OUTPUT);
  pinMode(debugPin_init, OUTPUT);
  pinMode(debugPin_ready, OUTPUT);
  pinMode(debugPin_stream, OUTPUT);
  pinMode(debugPin_timeout, OUTPUT);
  digitalWrite(debugPin_idle, 0);
  digitalWrite(debugPin_init, 0);
  digitalWrite(debugPin_ready, 0);
  digitalWrite(debugPin_stream, 0);
  digitalWrite(debugPin_timeout, 0);
}

void debug_pin_state()
{

  switch(serial_state)
  {
    case IDLE:
      digitalWrite(debugPin_idle, 1);
      digitalWrite(debugPin_init, 0);
      digitalWrite(debugPin_ready, 0);
      digitalWrite(debugPin_stream, 0);
      break;

    case INIT:
      digitalWrite(debugPin_idle, 0);
      digitalWrite(debugPin_init, 1);
      digitalWrite(debugPin_ready, 0);
      digitalWrite(debugPin_stream, 0);
      break;

    case READY:
      digitalWrite(debugPin_idle, 0);
      digitalWrite(debugPin_init, 0);
      digitalWrite(debugPin_ready, 1);
      digitalWrite(debugPin_stream, 0);
      break;

    case STREAM:
      digitalWrite(debugPin_idle, 0);
      digitalWrite(debugPin_init, 0);
      digitalWrite(debugPin_ready, 0);
      digitalWrite(debugPin_stream, 1);
      break;
  }
}

//include necessary library
// #include <wire.h>
#include <Adafruit_PWMServoDriver.h>

//create servo board object
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//servo setting parameters
// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  70 //-90 for mg996R// This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  440 //+90 for mg996R// This is the 'maximum' pulse length count (out of 4096)
// #define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
// #define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
// #define USMIN  150
// #define USMAX  600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
// uint8_t servonum = 0;

//data array that store the Joint pose
uint16_t joint_poses[JOINT_NUM];
unsigned char buff[JOINT_NUM*2]; //raw data receive from serial port

void timer_setup(){
  cli();                      //stop interrupts for till we make the settings
  /*1. First we reset the control register to amke sure we start with everything disabled.*/
  TCCR1A = 0;                 // Reset entire TCCR1A to 0 
  TCCR1B = 0;                 // Reset entire TCCR1B to 0
 
  /*2. We set the prescalar to the desired value by changing the CS10 CS12 and CS12 bits. */  
  TCCR1B |= B00000100;        //Set CS12 to 1 so we get prescalar 256  
  
  // /*3. We enable compare match mode on register A*/
  // TIMSK1 |= B00000010;        //Set OCIE1A to 1 so we enable compare match A 
  
  /*4. Set the value of register A to 31250*/
  OCR1A = 31250;             //Finally we set compare register A to this value  

  sei();                     //enable the intrrupt, so that the serial port can receive message
}
void timer_int_en(){
  TIMSK1 |= B00000010;        //Set OCIE1A to 1 so we enable compare match A 
}
void timer_int_dis(){
  TIMSK1 &= B11111101;        //Reset OCIE1A to 1 so we disable compare match A 
}
void timer_reset(){
  //reset counter
  TCNT1 &= 0; // clear the counter value
}
void timer_en(){
  //re-enable timer
  TCCR1B |= B00000100; //set the prescale to 256
}
void timer_dis(){
  //disable timer
  TCCR1B = 0; // disconnect the counter from clock source
}

void setup() {
  // put your setup code here, to run once:
  //configure the serial port to communicate with PC
  Serial.begin(115200);
  // Serial.print("Controller begin");
  // Serial.println();

  //set the state machine to INIT
  serial_state = IDLE;

  // set the digital pin as output:
  debug_pin_init();

  //configure timer
  timer_setup();

  //configure the PCA9865 serboard
  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

void loop() {
  debug_pin_state();
  //state transition
  switch(serial_state)
  {
    case IDLE:
      //transition the state to init state
      serial_state = INIT;
      break;
    case INIT:
      serial_state = READY;
      break;
    case READY:
      //if receive signal from PC, send state to Stream
      if(PC_signal_flag)
      {
        serial_state = STREAM;
        //reset signal flag
        PC_signal_flag = false;
      }
      else
      {
        //stay in the same state otherwise
        serial_state = READY;
      }
      break;
    case STREAM:
      // if the time out flag is true, send state to TIMEOUT
      if(serial_timeout_flag)
      {
        serial_state = TIMEOUT;
        //reset timeout flag
        serial_timeout_flag = false;
      }
      else
      {
        //stay at the same state
        serial_state = STREAM;
      }
      break;
    case TIMEOUT:
      // send the state back to idle
      serial_state = IDLE;
      break;
  }  
  //state behavior
  switch(serial_state)
  {
    case IDLE:
      // do nothing
      break;
    case INIT:
      // Serial.print("INIT");
      // Serial.println();
      // clear the RX buffer
      while(Serial.available()>0){
        // read the port until it's empty
        dump = Serial.read();
      }
      //flush the output buffer
      Serial.flush();
      // disable the timer
      timer_dis();
      // reset the timer counter
      timer_reset();

      //sent out init signal
      Serial.write(INIT_UNO);
      
      break;
    case READY:
      // Serial.print("READY");
      // Serial.println();
      // polling for the signal byte
      if (Serial.available())
      {
        if(Serial.read() == SIG_PC)
        {
          // set the signal flag
          PC_signal_flag = true;
          // send out acknowledge byte when receive the signal
          Serial.print(ACK_UNO);
          // enable timer interrupt
          // timer_int_en();
          // enable timer
          // timer_en();
        }
      }
      
      break;
    case STREAM:
      // Serial.print("STREAM");
      // Serial.println();
      // receive 8 bytes at a time
      // PC will send joint pose command at 50 Hz
      // data format: each joint position is a uint16, will be sent in two bytes,
      // format will be JP1_H,JP1_L...., unit: length count(I don't want arduino to handle floating point number)

      while(Serial.available() < (JOINT_NUM+1)*2) //"+1" here means the gripper command
      {
        //wait
      }
      Serial.readBytes(buff, (JOINT_NUM+1)*2);
      // digitalWrite(debugPin_read_serial, 0);
      // update servo command
      // digitalWrite(debugPin_update_servo, 1);

      //start compose the joint pose from the receive data
      for(int i = 0; i < (JOINT_NUM+1); i++)
      {
        joint_poses[i] = ((((uint16_t)(buff[2*i])) << 8u) | (uint16_t)(buff[2*i+1]));
        // Serial.print(joint_poses[i]);
      }
      //update the servo command
      for(int i = 0; i < (JOINT_NUM+1); i++)
      {
        
        pwm.setPWM(i, 0, joint_poses[i]);
        // pwm.setPWM(i, 0, 598);
      }
      // digitalWrite(debugPin_update_servo, 0);
      //make sure the input buffer is cleared
      while(Serial.available()>0){
        // read the port until it's empty
        dump = Serial.read();
      }
      // send out acknowledge byte
      Serial.print(ACK_UNO);
      break;
    case TIMEOUT:
      // Serial.print("TIMEOUT");
      // Serial.println();
      // disable timer interrupt
      timer_int_dis();
      break;
  }

}

//With the settings above, this IRS will trigger each 500ms.
ISR(TIMER1_COMPA_vect){
  TCNT1  = 0;                  //First, set the timer back to 0 so it resets for next interrupt
  //set the timeout flag
  digitalWrite(debugPin_timeout, 1);
  serial_timeout_flag = true;
}
