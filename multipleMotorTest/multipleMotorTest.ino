// Robert Robotics
// Be careful and have an E-STOP
// This works on my motor, and may work on yours, but no promises.

// Description: Runs demo shown at the end of the video here: https://www.youtube.com/watch?v=Mhxz2Bj2RXA&lc=UgxFZhySNMnKgtVloIN4AaABAg

#include "math_ops.h" // https://os.mbed.com/users/benkatz/code/CanMasterTest/ (slightly modified for Teensy use)
#include <FlexCAN.h>  // https://github.com/collin80/FlexCAN_Library

#define DT                  .01f             // Control loop period

/// Value Limits ///
// These values are imported directly from Ben Katz's Can Master Test script
#define P_MIN -95.5f        // Radians
#define P_MAX 95.5f
#define V_MIN -45.0f       // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f        // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f        // N-m/rad/s
#define KD_MAX 5.0f
#define I_MIN -18.0f
#define I_MAX 18.0f

// Setup FlexCAN messages
CAN_message_t txMsg;
CAN_message_t rxMsg;

// Potentiometer input pin and float value
const int potPin = A17;
float potVal = 0;

// Initialize desired position, velocity, kp, kd, and feed forward torque
float i_ff = 0;
float p_des = 0;
float v_des = 0;
float kp = 0;
float kd = 0;

// Count loops
int loop_counter = 0;

// Using a quick averaging method on potentiometer measurements to smooth potentiometer readout
float average = 0;

// Put control code here.
void control()
{
  // Increase counter value
  float t = DT * loop_counter;

  // You can experiment with the kd, kp, and p_des values to see the response of the motor
//  p_des = 0;
//  kd = 0.0f;
//  kp = average;

  // Here are some decent low and slow values for jogging the motor position:

   p_des = 2* sin(1.5*t); // generates sine wave based on current time step
//   p_des = average; // if you uncomment this one you are now controlling the motor position based on the potentiometer position

  // Kp and kd values which are good for these p_des values
  
    kd = .5f;
    kp = 2.0f; // you can probably make this larger to get a quicker response
  

  pack_cmd();
  loop_counter++;     // Increment loop counter
}



// Set up some more stuff
float position, velocity, current;

// Potentiometer sliding window array for averages and other stuff
const int numReadings = 10;
float readings[numReadings];
int readIndex = 0;
float total = 0;


// Your setup code goes here :)
void setup() {
  pinMode(potPin, INPUT);

  // Motor will not be enabled until serial monitor has started
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Serial begun");

  // Zero potentiometer readings/average
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  Can0.begin(1000000); // begin CAN at 1MBps

  // Functions to zero, disable, and enable motor
  disable_motor();
  delay(3000);
  zero_motor();
  delay(1);
  enable_motor();
  delay(1000);

  // Functions to pack and unpack CAN messages
  pack_cmd();
  delay(1000);
  unpack_reply();
}


void loop() {
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(potPin);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;

  average = map(average, 0, 1023, 6.28, 0);
  // send it to the computer as ASCII digits

  // Print value to Serial monitor
  Serial.println(average);

  // Execute control code
  control();
  delay(1);
  unpack_reply();
  delay(9); // delay 10ms to meet time step set in control function

  // Secondary way of disabling motor, if serial window closes, send disable command
  if (!Serial) {
    disable_motor();
  }

}

void pack_cmd() {
  /// limit data to be within bounds ///
  p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
  v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
  kp = fminf(fmaxf(KP_MIN, kp), KP_MAX);
  kd = fminf(fmaxf(KD_MIN, kd), KD_MAX);
  i_ff = fminf(fmaxf(I_MIN, i_ff), I_MAX);
  /// convert floats to unsigned ints ///
  int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  int t_int = float_to_uint(i_ff, I_MIN, I_MAX, 12);
  /// pack ints into the can buffer ///
  txMsg.id = 0x01;
  txMsg.len = 8;
  txMsg.buf[0] = p_int >> 8;
  txMsg.buf[1] = p_int & 0xFF;
  txMsg.buf[2] = v_int >> 4;
  txMsg.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  txMsg.buf[4] = kp_int & 0xFF;
  txMsg.buf[5] = kd_int >> 4;
  txMsg.buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  txMsg.buf[7] = t_int & 0xff;
  Can0.write(txMsg);

    txMsg.id = 0x02;
  txMsg.len = 8;
  txMsg.buf[0] = p_int >> 8;
  txMsg.buf[1] = p_int & 0xFF;
  txMsg.buf[2] = v_int >> 4;
  txMsg.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  txMsg.buf[4] = kp_int & 0xFF;
  txMsg.buf[5] = kd_int >> 4;
  txMsg.buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  txMsg.buf[7] = t_int & 0xff;
  Can0.write(txMsg);

    txMsg.id = 0x03;
  txMsg.len = 8;
  txMsg.buf[0] = p_int >> 8;
  txMsg.buf[1] = p_int & 0xFF;
  txMsg.buf[2] = v_int >> 4;
  txMsg.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  txMsg.buf[4] = kp_int & 0xFF;
  txMsg.buf[5] = kd_int >> 4;
  txMsg.buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  txMsg.buf[7] = t_int & 0xff;
  Can0.write(txMsg);
}

void unpack_reply() {
  rxMsg.len = 6;
  Can0.read(rxMsg);
  int id = rxMsg.buf[0];
  int p_int = (rxMsg.buf[1] << 8) | rxMsg.buf[2];
  int v_int = (rxMsg.buf[3] << 4) | (rxMsg.buf[4] >> 4);
  int i_int = ((rxMsg.buf[4] & 0xF) << 8) | rxMsg.buf[5];
  /// convert unsigned ints to floats ///
  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);

  position = p;
  velocity = v;
  current = i;

  // Uncomment if you want position and current values printed to monitor
  /*
    Serial.print("Position: ");
    Serial.println(position);

    Serial.print("Velocity: ");
    Serial.println(velocity);

    Serial.print("Current: ");
    Serial.println(current);
    Serial.println();
    Serial.println();
  */
}


void enable_motor() {
  txMsg.id = 0x01;
  txMsg.len = 8;
  txMsg.buf[0] = 0xFF;
  txMsg.buf[1] = 0xFF;
  txMsg.buf[2] = 0xFF;
  txMsg.buf[3] = 0xFF;
  txMsg.buf[4] = 0xFF;
  txMsg.buf[5] = 0xFF;
  txMsg.buf[6] = 0xFF;
  txMsg.buf[7] = 0xFC;
  Can0.write(txMsg);

    txMsg.id = 0x02;
  txMsg.len = 8;
  txMsg.buf[0] = 0xFF;
  txMsg.buf[1] = 0xFF;
  txMsg.buf[2] = 0xFF;
  txMsg.buf[3] = 0xFF;
  txMsg.buf[4] = 0xFF;
  txMsg.buf[5] = 0xFF;
  txMsg.buf[6] = 0xFF;
  txMsg.buf[7] = 0xFC;
  Can0.write(txMsg);

    txMsg.id = 0x03;
  txMsg.len = 8;
  txMsg.buf[0] = 0xFF;
  txMsg.buf[1] = 0xFF;
  txMsg.buf[2] = 0xFF;
  txMsg.buf[3] = 0xFF;
  txMsg.buf[4] = 0xFF;
  txMsg.buf[5] = 0xFF;
  txMsg.buf[6] = 0xFF;
  txMsg.buf[7] = 0xFC;
  Can0.write(txMsg);
}

void disable_motor() {
  txMsg.id = 0x01;
  txMsg.len = 8;
  txMsg.buf[0] = 0xFF;
  txMsg.buf[1] = 0xFF;
  txMsg.buf[2] = 0xFF;
  txMsg.buf[3] = 0xFF;
  txMsg.buf[4] = 0xFF;
  txMsg.buf[5] = 0xFF;
  txMsg.buf[6] = 0xFF;
  txMsg.buf[7] = 0xFD;
  Can0.write(txMsg);

    txMsg.id = 0x02;
  txMsg.len = 8;
  txMsg.buf[0] = 0xFF;
  txMsg.buf[1] = 0xFF;
  txMsg.buf[2] = 0xFF;
  txMsg.buf[3] = 0xFF;
  txMsg.buf[4] = 0xFF;
  txMsg.buf[5] = 0xFF;
  txMsg.buf[6] = 0xFF;
  txMsg.buf[7] = 0xFD;
  Can0.write(txMsg);

    txMsg.id = 0x03;
  txMsg.len = 8;
  txMsg.buf[0] = 0xFF;
  txMsg.buf[1] = 0xFF;
  txMsg.buf[2] = 0xFF;
  txMsg.buf[3] = 0xFF;
  txMsg.buf[4] = 0xFF;
  txMsg.buf[5] = 0xFF;
  txMsg.buf[6] = 0xFF;
  txMsg.buf[7] = 0xFD;
  Can0.write(txMsg);
}

void zero_motor() {
  txMsg.id = 0x01;
  txMsg.len = 8;
  txMsg.buf[0] = 0xFF;
  txMsg.buf[1] = 0xFF;
  txMsg.buf[2] = 0xFF;
  txMsg.buf[3] = 0xFF;
  txMsg.buf[4] = 0xFF;
  txMsg.buf[5] = 0xFF;
  txMsg.buf[6] = 0xFF;
  txMsg.buf[7] = 0xFE;
  Can0.write(txMsg);

    txMsg.id = 0x02;
  txMsg.len = 8;
  txMsg.buf[0] = 0xFF;
  txMsg.buf[1] = 0xFF;
  txMsg.buf[2] = 0xFF;
  txMsg.buf[3] = 0xFF;
  txMsg.buf[4] = 0xFF;
  txMsg.buf[5] = 0xFF;
  txMsg.buf[6] = 0xFF;
  txMsg.buf[7] = 0xFE;
  Can0.write(txMsg);

    txMsg.id = 0x03;
  txMsg.len = 8;
  txMsg.buf[0] = 0xFF;
  txMsg.buf[1] = 0xFF;
  txMsg.buf[2] = 0xFF;
  txMsg.buf[3] = 0xFF;
  txMsg.buf[4] = 0xFF;
  txMsg.buf[5] = 0xFF;
  txMsg.buf[6] = 0xFF;
  txMsg.buf[7] = 0xFE;
  Can0.write(txMsg);
}
