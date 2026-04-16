/*
Serial-driven firmware for the reaction wheel inverted pendulum.
The hardware setup and control laws are derived from the practicum sketch,
but user interaction is handled exclusively through the serial protocol.
*/

#include <VBCoreG4_arduino_system.h>
#include <Wire.h>
#include <AS5600.h>
#include <SimpleFOC.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PinSDA PB_7_ALT1
#define PinSCL PC6
#define PinNSLEEP PB3

#define K_PA 53.53f
#define K_PV 7.33f
#define K_MV 0.043f
#define K_SWING_UP 35.0f
#define EPSILON 0.16f

static const unsigned long CONTROL_PERIOD_US = 1000;
static const unsigned long TELEMETRY_PERIOD_US = 5000;
static const float TELEMETRY_VELOCITY_ALPHA = 0.02f;
static const float TELEMETRY_VOLTAGE_ALPHA = 0.1f;
static const unsigned long DRIVER_RESET_PULSE_US = 50;
static const unsigned long DRIVER_WAKE_DELAY_US = 3000;
static const unsigned long DRIVER_RESET_RETRY_US = 50000;
static const size_t COMMAND_BUFFER_SIZE = 96;
static const uint8_t MAX_TOKENS = 8;
static const float TWO_PI_F = 2.0f * PI;

enum ControlMode : uint8_t {
  MODE_STOPPED = 0,
  MODE_GO_UP,
  MODE_GO_DOWN,
  MODE_READ_ONLY
};

struct MotorSnapshot {
  float shaft_velocity;
  float shaft_angle;
};

HardwareTimer *timer_move = new HardwareTimer(TIM7);
HardwareTimer *timer_foc = new HardwareTimer(TIM2);

SPIClass SPI_3(PC12, PC11, PC10);
MagneticSensorSPI motor_sensor = MagneticSensorSPI(PA15, 14, 0x3FFF);
AMS_5600 pendulum_sensor;

float sensitivity = 45.0f;
InlineCurrentSense current_sense = InlineCurrentSense(sensitivity, PC1, PC2, PC3);
BLDCMotor motor = BLDCMotor(7, 12.2);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10);

float pendulum_angle = 0.0f;
float prev_angle = 0.0f;
float motor_angle = 0.0f;
float motor_velocity = 0.0f;
float motor_velocity_telemetry = 0.0f;
float pendulum_vel = 0.0f;
float offset = 0.0f;
float u = 0.0f;
float u_telemetry = 0.0f;

float k_pa = K_PA;
float k_pv = K_PV;
float k_mv = K_MV;
float k_swing_up = K_SWING_UP;

float M = 0.1975f;
float l = 0.088f;
float m = 0.2545f;
float J = M * l * l / 12.0f;
float Jmr = 0.0003385f;
float g = 9.81f;

float E_ref = (M / 2.0f + m) * g * l;
float E = 0.0f;
float epsilon = EPSILON;
float c_vel = 0.5f * (J + m * l * l + Jmr);

ControlMode control_mode = MODE_STOPPED;
bool pendulum_sensor_ready = false;
bool motor_initialized = false;
char command_buffer[COMMAND_BUFFER_SIZE];
size_t command_length = 0;
bool command_overflow = false;
unsigned long last_control_tick_us = 0;
unsigned long last_telemetry_tick_us = 0;
unsigned long stream_start_us = 0;
unsigned long last_state_sample_us = 0;
unsigned long last_driver_reset_attempt_us = 0;

void FOC_func();
void move();
void forceMotorIdle();

int sign_value(float val) {
  if (val < 0.0f) {
    return -1;
  }
  else {
    return 1;
  }
  // return 0;
}

void setDriverSleep(bool enabled) {
  digitalWrite(PinNSLEEP, enabled ? HIGH : LOW);
}

int readDriverFaultPin() {
  return digitalRead(PB5);
}

bool resetDriverAfterFault() {
  forceMotorIdle();

  timer_move->pause();
  timer_foc->pause();

  setDriverSleep(false);
  delayMicroseconds(DRIVER_RESET_PULSE_US);
  setDriverSleep(true);
  delayMicroseconds(DRIVER_WAKE_DELAY_US);

  timer_move->refresh();
  timer_foc->refresh();
  timer_foc->resume();
  timer_move->resume();

  const unsigned long now_us = micros();
  last_control_tick_us = now_us;
  last_telemetry_tick_us = now_us;
  last_state_sample_us = now_us;
  motor_velocity_telemetry = 0.0f;
  u_telemetry = 0.0f;

  return readDriverFaultPin() != LOW;
}

float wrapAnglePi(float angle) {
  return remainderf(angle, TWO_PI_F);
}

MotorSnapshot readMotorSnapshot() {
  MotorSnapshot snapshot;
  noInterrupts();
  snapshot.shaft_velocity = motor.shaft_velocity;
  snapshot.shaft_angle = motor.shaft_angle;
  interrupts();
  return snapshot;
}

void setMotorTargetAtomic(float target) {
  noInterrupts();
  motor.target = target;
  interrupts();
}

float readPendulumAngle() {
  const float raw_delta = 2048.0f - (pendulum_sensor.getRawAngle() - offset);
  return wrapAnglePi((PI * raw_delta) / 2048.0f);
}

void updateLedState() {
  digitalWrite(LED2, control_mode == MODE_STOPPED ? LOW : HIGH);
}

void forceMotorIdle() {
  u = 0.0f;
  setMotorTargetAtomic(0.0f);
}

void setControlMode(ControlMode new_mode) {
  control_mode = new_mode;
  if (new_mode == MODE_STOPPED) {
    forceMotorIdle();
  } else if (new_mode == MODE_READ_ONLY) {
    forceMotorIdle();
    stream_start_us = micros();
    last_telemetry_tick_us = stream_start_us;
  } else {
    stream_start_us = micros();
    last_telemetry_tick_us = stream_start_us;
  }
  updateLedState();
}

bool initializePendulumReference() {
  if (pendulum_sensor.detectMagnet() == 0) {
    pendulum_sensor_ready = false;
    setDriverSleep(false);
    forceMotorIdle();
    return false;
  }

  offset = pendulum_sensor.getRawAngle();
  pendulum_angle = readPendulumAngle();
  prev_angle = pendulum_angle;
  pendulum_vel = 0.0f;
  motor_velocity_telemetry = 0.0f;
  u_telemetry = 0.0f;
  last_state_sample_us = micros();
  pendulum_sensor_ready = true;
  setDriverSleep(true);
  return true;
}

void calc_angle_vel(unsigned long now_us) {
  pendulum_angle = readPendulumAngle();

  unsigned long dt_us = (unsigned long)(now_us - last_state_sample_us);
  if (dt_us == 0) {
    dt_us = CONTROL_PERIOD_US;
  }

  const float delta_angle = wrapAnglePi(pendulum_angle - prev_angle);
  pendulum_vel = delta_angle * (1000000.0f / (float)dt_us);
  prev_angle = pendulum_angle;
  last_state_sample_us = now_us;

  // Timer ISRs update SimpleFOC state, so main loop works with an atomic snapshot.
  const MotorSnapshot motor_snapshot = readMotorSnapshot();
  motor_velocity = -motor_snapshot.shaft_velocity;
  motor_angle = -motor_snapshot.shaft_angle;
  motor_velocity_telemetry += TELEMETRY_VELOCITY_ALPHA *
      (motor_velocity - motor_velocity_telemetry);
  u_telemetry += TELEMETRY_VOLTAGE_ALPHA * (u - u_telemetry);
}

void updateControlStep(unsigned long now_us) {
  if (!pendulum_sensor_ready) {
    forceMotorIdle();
    return;
  }

  calc_angle_vel(now_us);
  E = c_vel * sq(pendulum_vel) + E_ref * cos(pendulum_angle);

  if (control_mode == MODE_GO_UP) {
    if (fabsf(E_ref - E) < epsilon) {
      u = -(k_pa * pendulum_angle + k_pv * pendulum_vel + k_mv * motor_velocity);
    } else {
      u = -k_swing_up * (E_ref - E) * sign_value(pendulum_vel * cos(pendulum_angle));
    }
    if (u > 12.0f) {
      u = 12.0f;
    } else if (u < -12.0f) {
      u = -12.0f;
    }
    setMotorTargetAtomic(u);
  } else if (control_mode == MODE_GO_DOWN) {
    u = -k_swing_up * (-E_ref - E) * sign_value(pendulum_vel * cos(pendulum_angle));
    if (u > 12.0f) {
      u = 12.0f;
    } else if (u < -12.0f) {
      u = -12.0f;
    }
    setMotorTargetAtomic(u);
  } else {
    forceMotorIdle();
  }
}

void printFloat(float value) {
  Serial.print(value, 6);
}

void printError(const char *code) {
  Serial.print("#ERR ");
  Serial.println(code);
}

void printTelemetry(unsigned long now_us) {
  // Serial.print("#PB5 ");
  // Serial.print((unsigned long)((now_us - stream_start_us) / 1000UL));
  // Serial.print(' ');
  // Serial.println(readDriverFaultPin());

  Serial.print("#OUT ");
  Serial.print((unsigned long)((now_us - stream_start_us) / 1000UL));
  Serial.print(' ');
  printFloat(pendulum_angle);
  Serial.print(' ');
  printFloat(pendulum_vel);
  Serial.print(' ');
  printFloat(motor_velocity_telemetry);
  Serial.print(' ');
  printFloat(E);
  Serial.print(' ');
  printFloat(E_ref);
  Serial.print(' ');
  printFloat(u_telemetry);
  Serial.println();
}

int tokenizeArguments(char *line, char *tokens[], int max_tokens) {
  int count = 0;
  char *cursor = line;
  while (*cursor != '\0') {
    while (*cursor == ' ') {
      ++cursor;
    }
    if (*cursor == '\0') {
      break;
    }
    if (count >= max_tokens) {
      return -1;
    }
    tokens[count++] = cursor;
    while (*cursor != '\0' && *cursor != ' ') {
      ++cursor;
    }
    if (*cursor == '\0') {
      break;
    }
    *cursor = '\0';
    ++cursor;
  }
  return count;
}

bool parseFloatToken(const char *token, float &value) {
  char *end_ptr = nullptr;
  value = strtof(token, &end_ptr);
  return end_ptr != token && *end_ptr == '\0';
}

bool ensureSensorForMotion() {
  if (pendulum_sensor_ready) {
    return true;
  }
  return initializePendulumReference();
}

void handleSetK(char *tokens[], int token_count) {
  if (token_count != 5 && token_count != 6) {
    printError("BAD_ARGS");
    return;
  }

  float new_k1 = 0.0f;
  float new_k2 = 0.0f;
  float new_k3 = 0.0f;
  float new_k = 0.0f;
  float new_epsilon = epsilon;
  if (!parseFloatToken(tokens[1], new_k1) ||
      !parseFloatToken(tokens[2], new_k2) ||
      !parseFloatToken(tokens[3], new_k3) ||
      !parseFloatToken(tokens[4], new_k) ||
      (token_count == 6 && !parseFloatToken(tokens[5], new_epsilon))) {
    printError("BAD_NUMBER");
    return;
  }

  setControlMode(MODE_STOPPED);
  k_pa = new_k1;
  k_pv = new_k2;
  k_mv = new_k3;
  k_swing_up = new_k;
  epsilon = new_epsilon;
  Serial.println("#SETK OK");
}

void handleGetK() {
  setControlMode(MODE_STOPPED);
  Serial.println("#GETK OK");
  printFloat(k_pa);
  Serial.print(' ');
  printFloat(k_pv);
  Serial.print(' ');
  printFloat(k_mv);
  Serial.print(' ');
  printFloat(k_swing_up);
  Serial.print(' ');
  printFloat(epsilon);
  Serial.println();
}

void handleStatus() {
  setControlMode(MODE_STOPPED);
  Serial.println("#STATUS OK");
  Serial.print("#RWIP READY=");
  Serial.println(motor_initialized ? 1 : 0);
}

void handleGoUp() {
  if (!ensureSensorForMotion()) {
    printError("SENSOR");
    return;
  }
  setControlMode(MODE_GO_UP);
  Serial.println("#GO_UP OK");
}

void handleGoDown() {
  if (!ensureSensorForMotion()) {
    printError("SENSOR");
    return;
  }
  setControlMode(MODE_GO_DOWN);
  Serial.println("#GO_DOWN OK");
}

void handleReadOnly() {
  if (!ensureSensorForMotion()) {
    printError("SENSOR");
    return;
  }
  setControlMode(MODE_READ_ONLY);
  Serial.println("#READ OK");
}

void handleStop() {
  setControlMode(MODE_STOPPED);
  Serial.println("#STOP OK");
}

void processCommand(char *line) {
  char *tokens[MAX_TOKENS];
  int token_count = tokenizeArguments(line, tokens, MAX_TOKENS);
  if (token_count < 0) {
    printError("BAD_ARGS");
    return;
  }
  if (token_count == 0) {
    return;
  }

  if (strcmp(tokens[0], "#SETK") == 0) {
    handleSetK(tokens, token_count);
  } else if (strcmp(tokens[0], "#GETK") == 0) {
    if (token_count != 1) {
      printError("BAD_ARGS");
      return;
    }
    handleGetK();
  } else if (strcmp(tokens[0], "#GO_UP") == 0) {
    if (token_count != 1) {
      printError("BAD_ARGS");
      return;
    }
    handleGoUp();
  } else if (strcmp(tokens[0], "#GO_DOWN") == 0) {
    if (token_count != 1) {
      printError("BAD_ARGS");
      return;
    }
    handleGoDown();
  } else if (strcmp(tokens[0], "#READ") == 0) {
    if (token_count != 1) {
      printError("BAD_ARGS");
      return;
    }
    handleReadOnly();
  } else if (strcmp(tokens[0], "#STOP") == 0) {
    if (token_count != 1) {
      printError("BAD_ARGS");
      return;
    }
    handleStop();
  } else if (strcmp(tokens[0], "#STATUS") == 0) {
    if (token_count != 1) {
      printError("BAD_ARGS");
      return;
    }
    handleStatus();
  } else {
    printError("BAD_CMD");
  }
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r' || incoming == '\n') {
      if (command_overflow) {
        command_length = 0;
        command_overflow = false;
        printError("BAD_CMD");
      } else if (command_length > 0) {
        command_buffer[command_length] = '\0';
        processCommand(command_buffer);
        command_length = 0;
      }
      continue;
    }

    if (command_length < COMMAND_BUFFER_SIZE - 1) {
      command_buffer[command_length++] = incoming;
    } else {
      command_overflow = true;
    }
  }
}

void setup() {
  Serial.begin(500000);

  Wire.setSDA(PinSDA);
  Wire.setSCL(PinSCL);
  Wire.begin();

  pinMode(PB5, INPUT_PULLUP);
  pinMode(PinNSLEEP, OUTPUT);
  pinMode(LED2, OUTPUT);

  pinMode(PB15, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB13, OUTPUT);
  digitalWrite(PB15, HIGH);
  digitalWrite(PB14, HIGH);
  digitalWrite(PB13, HIGH);

  setDriverSleep(true);
  updateLedState();

  driver.voltage_power_supply = 12;
  driver.init();
  driver.enable();
  motor.linkDriver(&driver);

  current_sense.init();
  current_sense.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);

  motor_sensor.init(&SPI_3);
  motor.linkSensor(&motor_sensor);
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  motor.current_limit = 50;
  motor.voltage_limit = 12;
  motor.velocity_limit = 5000;
  motor.init();
  motor.initFOC();
  motor_initialized = true;
  forceMotorIdle();

  initializePendulumReference();

  timer_move->pause();
  timer_move->setOverflow(2000, HERTZ_FORMAT);
  timer_move->attachInterrupt(move);
  timer_move->refresh();
  timer_move->resume();

  timer_foc->pause();
  timer_foc->setOverflow(5000, HERTZ_FORMAT);
  timer_foc->attachInterrupt(FOC_func);
  timer_foc->refresh();
  timer_foc->resume();

  unsigned long now_us = micros();
  last_control_tick_us = now_us;
  last_telemetry_tick_us = now_us;
  stream_start_us = now_us;
  last_state_sample_us = now_us;

  Serial.println("Ready");
}

void loop() {
  pollSerialCommands();

  unsigned long now_us = micros();
  bool driver_fault_active = (readDriverFaultPin() == LOW);
  if (driver_fault_active) {
    if ((unsigned long)(now_us - last_driver_reset_attempt_us) >= DRIVER_RESET_RETRY_US) {
      last_driver_reset_attempt_us = now_us;
      resetDriverAfterFault();
      driver_fault_active = (readDriverFaultPin() == LOW);
    }
    forceMotorIdle();
  }

  if (!driver_fault_active &&
      (unsigned long)(now_us - last_control_tick_us) >= CONTROL_PERIOD_US) {
    if ((unsigned long)(now_us - last_control_tick_us) > (CONTROL_PERIOD_US * 4UL)) {
      last_control_tick_us = now_us;
    } else {
      last_control_tick_us += CONTROL_PERIOD_US;
    }
    updateControlStep(now_us);
  }

  if (control_mode != MODE_STOPPED &&
      (unsigned long)(now_us - last_telemetry_tick_us) >= TELEMETRY_PERIOD_US &&
      pendulum_sensor_ready) {
    last_telemetry_tick_us = now_us;
    printTelemetry(now_us);
  }
}

void FOC_func() {
  motor.loopFOC();
}

void move() {
  motor.move();
}
