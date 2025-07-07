/*
The SimpleFOC library is used to control the BLDC motor: https://docs.simplefoc.com/ 
Documentation for the VBCore module: https://voltbro.gitbook.io/vbcores 
Documentation for the motor driver: https://voltbro.gitbook.io/vbcores/vbcores-hardware/bldc-draiver-30a
*/

#include <VBCoreG4_arduino_system.h>
#include <Wire.h>
#include <AS5600.h>
#include <SimpleFOC.h>

#define PinSDA PB_7_ALT1 // SDA pin of the I2C bus on the motor driver
#define PinSCL PC6       // SCL pin of the I2C bus on the motor driver
#define PinNSLEEP PB3    // Pin for enabling/disabling the motor driver

// State Feedback Constants
#define K_PA -53.53  // k1 - pendulum arm angle
#define K_PV -7.33   // k2 - pendulum arm angular velocity
#define K_MV -0.043  // k3 - reaction wheel angular velocity
#define K_SWING_UP 35.0 // k -Coefficient for energy-based swing-up and braking
#define EPSILON 0.16;   // Tolerance for switching control modes

// Two hardware timers will call control functions periodically
HardwareTimer *timer_move = new HardwareTimer(TIM7);
HardwareTimer *timer_foc = new HardwareTimer(TIM2);

float target_angle, pendulum_angle, prev_angle, motor_angle, motor_velocity, pendulum_vel;
int stop_flag = 0, swing_up_flag = 0;

float offset;
float angle_tmp;
float u = 0.0;

// Feedback Coefficients
float k_pa = K_PA; // pendulum arm angle
float k_pv = K_PV; // pendulum arm angular velocity
float k_mv = K_MV; // reaction wheel angular velocity
float k_swing_up = K_SWING_UP; // Coefficient for energy swing-up and braking

// Physical parameters of the system
float M = 0.1975; //0.014;      // Mass of the pendulum
float l = 0.06;       // Length of the pendulum
float m = 0.2545; //0.063;      // Mass of the motor + flywheel
float J = M*l*l/3;    // Moment of inertia of the pendulum about its base
float Jmr = 0.0003385;// Combined moment of inertia of motor and flywheel
float g = 9.81;       // Gravitational acceleration

float E_ref = (M/2+m)*g*l; // Desired energy level (upright equilibrium position)
float E;
float epsilon = EPSILON;   // Tolerance for switching control modes

float c_vel = 0.5*(J+m*l*l+Jmr); // Constant used in energy calculation

SPIClass SPI_3(PC12, PC11, PC10); // SPI pins on motor driver: PC11 - MISO, PC12 - MOSI, PC10 - SCK

MagneticSensorSPI motor_sensor = MagneticSensorSPI(PA15, 14, 0x3FFF); // Motor sensor over SPI
AMS_5600 pendulum_sensor; // Pendulum sensor over I2C

// Current sensor: ACS711KEXLT-31AB-T
// Connected to pins PC1, PC2, PC3 corresponding to I_A, I_B, I_C
float sensitivity = 45.0;
InlineCurrentSense current_sense = InlineCurrentSense(sensitivity, PC1, PC2, PC3);  

// Create motor object: 7 pole pairs, 12.2 Ohm resistance
BLDCMotor motor = BLDCMotor(7, 12.2);

// Create driver object with 3-phase PWM pins
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10);

void setup() {
  Wire.setSDA(PinSDA);
  Wire.setSCL(PinSCL);
  Wire.begin();

  pinMode(PB5, INPUT);         // NFAULT pin on the driver
  pinMode(PinNSLEEP, OUTPUT);  // nSLEEP pin: driver enable
  pinMode(LED2, OUTPUT);       // User LED pin
  digitalWrite(PinNSLEEP, HIGH); // Enable the driver
  pinMode(USR_BTN, INPUT_PULLUP); // User button (active LOW)

  // Configure low-side control pins (INLA, INLB, INLC): PB13, PB14, PB15
  // Enable all low-side switches before driver init
  pinMode(PB15, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB13, OUTPUT);
  digitalWrite(PB15, HIGH);
  digitalWrite(PB14, HIGH);
  digitalWrite(PB13, HIGH);

  // Initialize motor driver using SimpleFOC
  driver.voltage_power_supply = 16;
  driver.init();
  driver.enable();
  motor.linkDriver(&driver);

  // Initialize current sensing using SimpleFOC
  current_sense.init();
  current_sense.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);

  // Motor setup with SimpleFOC
  motor_sensor.init(&SPI_3);
  motor.linkSensor(&motor_sensor);
  motor.torque_controller = TorqueControlType::voltage; 
  motor.controller = MotionControlType::torque;

  motor.current_limit = 50;
  motor.voltage_limit = 16;
  motor.velocity_limit = 5000; 
  motor.init();
  motor.initFOC();
  motor.target = 0;

  // Check pendulum sensor connection
  if(pendulum_sensor.detectMagnet() == 0 ){
    while(1){
        if(pendulum_sensor.detectMagnet() == 1 ){
            Serial.print("Current Magnitude: ");
            Serial.println(pendulum_sensor.getMagnitude());
            break;
        }
        else{
            Serial.println("Can not detect magnet");
        }
        delay(1000);
    }
  }

  offset = pendulum_sensor.getRawAngle();
  pendulum_angle = pendulum_sensor.getRawAngle() - offset;
  prev_angle = pendulum_angle;
 
  timer_move->pause();
  timer_move->setOverflow(2000, HERTZ_FORMAT); 
  timer_move->attachInterrupt(move); // Called at 2kHz for motion control
  timer_move->refresh();
  timer_move->resume();

  timer_foc->pause();
  timer_foc->setOverflow(5000, HERTZ_FORMAT);
  timer_foc->attachInterrupt(FOC_func); // Called at 5kHz for low-level torque control
  timer_foc->refresh();
  timer_foc->resume();

  delay(1000);
}

// Function to calculate control voltage for the motor
void control(){
  calc_angle_vel(); // Get angles and velocities from sensors
  E = c_vel*sq(pendulum_vel)+ E_ref*cos(pendulum_angle); // Compute pendulum energy

  if (swing_up_flag){ // If user activated the pendulum
    if (abs(E_ref - E) < epsilon) {
      // Stabilization around unstable equilibrium
      u = -(k_pa*pendulum_angle + k_pv*pendulum_vel + k_mv*motor_velocity); 
    }
    else {
      // Energy-based swing-up control
      u = k_swing_up * (E_ref - E) * sign(pendulum_vel * cos(pendulum_angle));// sign(pendulum_vel);  //
    }
  }
  else if(stop_flag) {
    // Braking mode
    u = k_swing_up * (-E_ref - E) * sign(pendulum_vel * cos(pendulum_angle));
  }

  // Limit control voltage
  if (u > 12) u = 12;
  else if (u < -12) u = -12;
  
  motor.target = -u; // Apply control voltage to the motor
}

// Function to compute pendulum and motor angles and velocities
void calc_angle_vel(){
  angle_tmp = pendulum_sensor.getRawAngle()-offset;
  angle_tmp = 2048 - angle_tmp;
  pendulum_angle = (PI*angle_tmp)/2048; // Convert to range [-π, π]
  if (pendulum_angle > PI){
    angle_tmp = fmod(pendulum_angle, PI);
    pendulum_angle = angle_tmp - PI;
  }
  pendulum_vel = (pendulum_angle - prev_angle)*1000; // Angular velocity (rad/s), called at ~1kHz
  prev_angle = pendulum_angle;
  motor_velocity =  -motor.shaft_velocity; // Motor angular velocity via SimpleFOC
  motor_angle = -motor.shaft_angle;        // Motor angle via SimpleFOC
}

// Return sign of input value (-1, 0, or 1)
int sign(float val){
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

// Stop the pendulum: disable control, set voltage to 0
void stop(){
  u = 0;
  stop_flag = 1;
  swing_up_flag = 0;
  motor.move(0);
  delay(200);
}

// Start the pendulum: enable swing-up control
void start(){
  stop_flag = 0;
  swing_up_flag = 1;
  motor.move(1);
  delay(500);
}

// Main loop: handles button state and calls control logic
// LED ON: pendulum is active and trying to balance
// LED OFF: pendulum is stopped or braking
void loop() {
  int buttonState = digitalRead(USR_BTN);
  if(buttonState == 0){ // Button pressed: toggle start/stop
    if(swing_up_flag == 1){
      digitalWrite(LED2, LOW);
      stop();
    }
    else{
      digitalWrite(LED2, HIGH);
      start();
    }
  }
  control();
  delay(1);
}

// Function running low-level torque control loop (5kHz)
void FOC_func(){
  motor.loopFOC();
}

// Function running motion control (2kHz), calls motor.move() using motor.target
void move(){
  motor.move(); 
}
