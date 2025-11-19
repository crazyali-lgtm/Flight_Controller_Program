///////////////////////////////////////////////////////////////////////////////////////
// Terms of use & Safety note – unverändert
///////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <EEPROM.h>

////////////////////////////////////////////////////////////////////////////////////////////////////
// PID gain and limit settings – unverändert (gerne später tunen)
////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;
float pid_i_gain_roll = 0.04;
float pid_d_gain_roll = 18.0;
int   pid_max_roll     = 400;

float pid_p_gain_pitch = pid_p_gain_roll;
float pid_i_gain_pitch = pid_i_gain_roll;
float pid_d_gain_pitch = pid_d_gain_roll;
int   pid_max_pitch    = pid_max_roll;

float pid_p_gain_yaw = 4.0;
float pid_i_gain_yaw = 0.02;
float pid_d_gain_yaw = 0.0;
int   pid_max_yaw    = 400;

boolean auto_level = true;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Globale Variablen
////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4; // (nicht mehr genutzt)
byte eeprom_data[36];
byte highByte, lowByte;

volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;

// *** geändert: Wir nutzen PPM → array größer + volatile
volatile int receiver_input[9];  // [1..8] üblich

int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start, gyro_address;
int temperature;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;

long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4;
unsigned long esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll,  pid_roll_setpoint,  gyro_roll_input,  pid_output_roll,  pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw,   pid_yaw_setpoint,   gyro_yaw_input,   pid_output_yaw,   pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;

////////////////////////////////////////////////////////////////////////////////////////////////////
// *** neu: Hardware-Mapping für euer Layout
////////////////////////////////////////////////////////////////////////////////////////////////////
// ESC-Ausgänge:
#define ESC1_PORTD_BIT 3   // D3  -> Motor 1 (front-right CCW)
#define ESC2_PORTD_BIT 5   // D5  -> Motor 2 (rear-right  CW)
#define ESC3_PORTD_BIT 6   // D6  -> Motor 3 (rear-left  CCW)
#define ESC4_PORTB_BIT 1   // D9  -> Motor 4 (front-left CW)
// LED bleibt D12 (PB4)

// PPM-Eingang:
#define PPM_PIN 2          // D2 (INT0)

// *** neu: PPM-Decoder-Zustand
volatile unsigned long ppm_last = 0;
volatile byte ppm_ch = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup
////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  //Serial.begin(57600);

  for(start = 0; start <= 35; start++) eeprom_data[start] = EEPROM.read(start);
  start = 0;
  gyro_address = eeprom_data[32];

  Wire.begin();
  TWBR = 12; // 400kHz I2C

  // *** geändert: ESC-Pins passend setzen (D3,D5,D6 als Ausgang / PORTD; D9 auf PORTB)
  DDRD |= (1<<ESC1_PORTD_BIT) | (1<<ESC2_PORTD_BIT) | (1<<ESC3_PORTD_BIT);
  DDRB |= (1<<4) | (1<<5) | (1<<ESC4_PORTB_BIT); // D12 LED (PB4), D13 (PB5) wie original, plus D9 (PB1)

  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  // *** geändert: KEINE PCINT auf D8..D11 mehr – wir benutzen PPM an D2 (INT0)
  pinMode(PPM_PIN, INPUT);                         // bei Open-Collector ggf. INPUT_PULLUP
  EICRA |= (1<<ISC01) | (1<<ISC00);               // INT0 auf Rising Edge
  EIMSK |= (1<<INT0);                             // INT0 aktivieren

  // Setup-Checks
  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B') delay(10);
  if(eeprom_data[31] == 2 || eeprom_data[31] == 3) delay(10); // MPU-6050 erforderlich

  set_gyro_registers();

  // 5s Wartezeit: ESCs erhalten 1000µs, damit sie ruhig sind
  for (cal_int = 0; cal_int < 1250 ; cal_int ++){
    esc_1 = esc_2 = esc_3 = esc_4 = 1000;
    esc_send_pulse_blocking(esc_1, esc_2, esc_3, esc_4); // *** geändert: über neue Routine
    delayMicroseconds(3000);
  }

  // Gyro-Kalibrierung
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){
    if(cal_int % 15 == 0) digitalWrite(12, !digitalRead(12));
    gyro_signalen();
    gyro_axis_cal[1] += gyro_axis[1];
    gyro_axis_cal[2] += gyro_axis[2];
    gyro_axis_cal[3] += gyro_axis[3];

    // ESCs mit 1000µs leise halten
    esc_1 = esc_2 = esc_3 = esc_4 = 1000;
    esc_send_pulse_blocking(esc_1, esc_2, esc_3, esc_4);

    delay(3);
  }
  gyro_axis_cal[1] /= 2000;
  gyro_axis_cal[2] /= 2000;
  gyro_axis_cal[3] /= 2000;

  // *** entfernt: PCICR/PCMSK (wir nutzen PPM)

  // Warten bis RX aktiv (Gas unten, Yaw mittig/hoch genug)
  while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400){
    receiver_input_channel_3 = convert_receiver_channel(3);
    receiver_input_channel_4 = convert_receiver_channel(4);
    start++;

    // 1000µs Haltepuls
    esc_1 = esc_2 = esc_3 = esc_4 = 1000;
    esc_send_pulse_blocking(esc_1, esc_2, esc_3, esc_4);

    delay(3);
    if(start == 125){ digitalWrite(12, !digitalRead(12)); start = 0; }
  }
  start = 0;

  // Batteriespannung grob initialisieren
  battery_voltage = (analogRead(0) + 65) * 1.2317;

  loop_timer = micros();
  digitalWrite(12, LOW);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Hauptschleife
////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){

  // Gyro-Rate-Filter für PID
  gyro_roll_input  = (gyro_roll_input  * 0.7) + ((gyro_roll  / 65.5) * 0.3);
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);
  gyro_yaw_input   = (gyro_yaw_input   * 0.7) + ((gyro_yaw   / 65.5) * 0.3);

  // IMU: Winkelintegration + Acc-Driftkorrektur
  angle_pitch += gyro_pitch * 0.0000611;
  angle_roll  += gyro_roll  * 0.0000611;

  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);
  angle_roll  += angle_pitch * sin(gyro_yaw * 0.000001066);

  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
  if(abs(acc_y) < acc_total_vector) angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;
  if(abs(acc_x) < acc_total_vector) angle_roll_acc  = asin((float)acc_x/acc_total_vector)* -57.296;

  // evtl. Fein-Offset
  angle_pitch_acc -= 0.0;
  angle_roll_acc  -= 0.0;

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
  angle_roll  = angle_roll  * 0.9996 + angle_roll_acc  * 0.0004;

  pitch_level_adjust = auto_level ? angle_pitch * 15 : 0;
  roll_level_adjust  = auto_level ? angle_roll  * 15 : 0;

  // Arm/Disarm-Logik
  if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050) start = 1;
  if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
    start = 2;
    angle_pitch = angle_pitch_acc;
    angle_roll  = angle_roll_acc;
    gyro_angles_set = true;

    pid_i_mem_roll = pid_last_roll_d_error = 0;
    pid_i_mem_pitch = pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = pid_last_yaw_d_error = 0;
  }
  if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950) start = 0;

  // Setpoints
  pid_roll_setpoint = 0;
  if(receiver_input_channel_1 > 1508)      pid_roll_setpoint = receiver_input_channel_1 - 1508;
  else if(receiver_input_channel_1 < 1492) pid_roll_setpoint = receiver_input_channel_1 - 1492;
  pid_roll_setpoint -= roll_level_adjust;
  pid_roll_setpoint /= 3.0;

  pid_pitch_setpoint = 0;
  if(receiver_input_channel_2 > 1508)      pid_pitch_setpoint = receiver_input_channel_2 - 1508;
  else if(receiver_input_channel_2 < 1492) pid_pitch_setpoint = receiver_input_channel_2 - 1492;
  pid_pitch_setpoint -= pitch_level_adjust;
  pid_pitch_setpoint /= 3.0;

  pid_yaw_setpoint = 0;
  if(receiver_input_channel_3 > 1050){
    if(receiver_input_channel_4 > 1508)      pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
    else if(receiver_input_channel_4 < 1492) pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
  }

  calculate_pid();

  // Batteriespannung (einfaches Komplementärfilter)
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;
  if(battery_voltage < 1000 && battery_voltage > 600) digitalWrite(12, HIGH);

  throttle = receiver_input_channel_3;

  if (start == 2){
    if (throttle > 1800) throttle = 1800;
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; // FR (CCW)  -> D3
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; // RR (CW)   -> D5
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; // RL (CCW)  -> D6
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; // FL (CW)   -> D9

    if (battery_voltage < 1240 && battery_voltage > 800){
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);
    }

    if (esc_1 < 1100) esc_1 = 1100;
    if (esc_2 < 1100) esc_2 = 1100;
    if (esc_3 < 1100) esc_3 = 1100;
    if (esc_4 < 1100) esc_4 = 1100;

    if (esc_1 > 2000) esc_1 = 2000;
    if (esc_2 > 2000) esc_2 = 2000;
    if (esc_3 > 2000) esc_3 = 2000;
    if (esc_4 > 2000) esc_4 = 2000;
  }
  else{
    esc_1 = esc_2 = esc_3 = esc_4 = 1000;
  }

  // ***** Loop-Timing 4ms (250Hz) *****
  if(micros() - loop_timer > 4050) digitalWrite(12, HIGH);
  while(micros() - loop_timer < 4000);
  loop_timer = micros();

  // *** geändert: ESC-Impulse auf D3/D5/D6/D9 erzeugen und währenddessen IMU lesen ***
  unsigned long t0 = loop_timer;

  // High setzen
  PORTD |= (1<<ESC1_PORTD_BIT) | (1<<ESC2_PORTD_BIT) | (1<<ESC3_PORTD_BIT);
  PORTB |= (1<<ESC4_PORTB_BIT);

  // Abschaltzeiten berechnen
  unsigned long t1 = t0 + esc_1;
  unsigned long t2 = t0 + esc_2;
  unsigned long t3 = t0 + esc_3;
  unsigned long t4 = t0 + esc_4;

  // Während der ersten ~1000µs ist Luft → IMU lesen
  gyro_signalen();

  // Pending-Masken
  uint8_t pendD = (1<<ESC1_PORTD_BIT) | (1<<ESC2_PORTD_BIT) | (1<<ESC3_PORTD_BIT);
  uint8_t pendB = (1<<ESC4_PORTB_BIT);

  while(pendD || pendB){
    unsigned long now = micros();
    if((pendD & (1<<ESC1_PORTD_BIT)) && now >= t1){ PORTD &= ~(1<<ESC1_PORTD_BIT); pendD &= ~(1<<ESC1_PORTD_BIT); }
    if((pendD & (1<<ESC2_PORTD_BIT)) && now >= t2){ PORTD &= ~(1<<ESC2_PORTD_BIT); pendD &= ~(1<<ESC2_PORTD_BIT); }
    if((pendD & (1<<ESC3_PORTD_BIT)) && now >= t3){ PORTD &= ~(1<<ESC3_PORTD_BIT); pendD &= ~(1<<ESC3_PORTD_BIT); }
    if((pendB & (1<<ESC4_PORTB_BIT)) && now >= t4){ PORTB &= ~(1<<ESC4_PORTB_BIT); pendB &= ~(1<<ESC4_PORTB_BIT); }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// *** neu: PPM-Decoder auf INT0 (D2) – füllt receiver_input[1..]
////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(INT0_vect){
  unsigned long now = micros();
  unsigned int dt = (unsigned int)(now - ppm_last);
  ppm_last = now;

  if(dt > 3000 && dt < 10000){
    ppm_ch = 1; // Sync → neues Frame
  } else {
    if(ppm_ch >= 1 && ppm_ch <= 8){
      receiver_input[ppm_ch] = dt;
      ppm_ch++;
    }
  }
}

// *** ALT: PCINT0_vect (D8..D11) entfällt komplett ***

////////////////////////////////////////////////////////////////////////////////////////////////////
// Gyro/Accel lesen (wie original, nur dass RX-Werte aus PPM stammen)
////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(){
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address,14);

    // RC-Kanäle konvertieren (aus PPM-roh → 1000..2000µs skaliert)
    receiver_input_channel_1 = convert_receiver_channel(1);
    receiver_input_channel_2 = convert_receiver_channel(2);
    receiver_input_channel_3 = convert_receiver_channel(3);
    receiver_input_channel_4 = convert_receiver_channel(4);

    while(Wire.available() < 14);
    acc_axis[1] = Wire.read()<<8|Wire.read();
    acc_axis[2] = Wire.read()<<8|Wire.read();
    acc_axis[3] = Wire.read()<<8|Wire.read();
    temperature = Wire.read()<<8|Wire.read();
    gyro_axis[1] = Wire.read()<<8|Wire.read();
    gyro_axis[2] = Wire.read()<<8|Wire.read();
    gyro_axis[3] = Wire.read()<<8|Wire.read();
  }

  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];
    gyro_axis[2] -= gyro_axis_cal[2];
    gyro_axis[3] -= gyro_axis_cal[3];
  }
  gyro_roll  = gyro_axis[eeprom_data[28] & 0b00000011];
  if(eeprom_data[28] & 0b10000000) gyro_roll *= -1;
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];
  if(eeprom_data[29] & 0b10000000) gyro_pitch *= -1;
  gyro_yaw   = gyro_axis[eeprom_data[30] & 0b00000011];
  if(eeprom_data[30] & 0b10000000) gyro_yaw   *= -1;

  acc_x = acc_axis[eeprom_data[29] & 0b00000011];
  if(eeprom_data[29] & 0b10000000) acc_x *= -1;
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];
  if(eeprom_data[28] & 0b10000000) acc_y *= -1;
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];
  if(eeprom_data[30] & 0b10000000) acc_z *= -1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// PID – unverändert
////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(){
  // Roll
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)      pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < -pid_max_roll) pid_i_mem_roll = -pid_max_roll;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll
                  + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)       pid_output_roll = pid_max_roll;
  else if(pid_output_roll < -pid_max_roll) pid_output_roll = -pid_max_roll;

  pid_last_roll_d_error = pid_error_temp;

  // Pitch
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)       pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < -pid_max_pitch) pid_i_mem_pitch = -pid_max_pitch;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch
                   + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)       pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < -pid_max_pitch) pid_output_pitch = -pid_max_pitch;

  pid_last_pitch_d_error = pid_error_temp;

  // Yaw
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)       pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < -pid_max_yaw) pid_i_mem_yaw = -pid_max_yaw;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw
                 + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)       pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < -pid_max_yaw) pid_output_yaw = -pid_max_yaw;

  pid_last_yaw_d_error = pid_error_temp;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// RX-Kanalnormierung – unverändert 
////////////////////////////////////////////////////////////////////////////////////////////////////
int convert_receiver_channel(byte function){
  byte channel, reverse;
  int low, center, high, actual;
  int difference;

  channel = eeprom_data[function + 23] & 0b00000111;
  reverse = (eeprom_data[function + 23] & 0b10000000) ? 1 : 0;

  actual = receiver_input[channel];
  low    = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2];
  high   = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];

  if(actual < center){
    if(actual < low) actual = low;
    difference = ((long)(center - actual) * 500L) / (center - low);
    return reverse ? 1500 + difference : 1500 - difference;
  }
  else if(actual > center){
    if(actual > high) actual = high;
    difference = ((long)(actual - center) * 500L) / (high - center);
    return reverse ? 1500 - difference : 1500 + difference;
  }
  else return 1500;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// MPU-6050 Register – unverändert
////////////////////////////////////////////////////////////////////////////////////////////////////
void set_gyro_registers(){
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(gyro_address); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
    Wire.beginTransmission(gyro_address); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();

    Wire.beginTransmission(gyro_address); Wire.write(0x1B); Wire.endTransmission();
    Wire.requestFrom(gyro_address, 1); while(Wire.available() < 1);
    if(Wire.read() != 0x08){ digitalWrite(12,HIGH); while(1) delay(10); }

    Wire.beginTransmission(gyro_address); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// *** neu: kleine Helferfunktion – 4 Pulsbreiten gleichzeitig senden (D3/D5/D6/D9)
////////////////////////////////////////////////////////////////////////////////////////////////////
void esc_send_pulse_blocking(int us1, int us2, int us3, int us4){
  unsigned long t0 = micros();

  PORTD |= (1<<ESC1_PORTD_BIT) | (1<<ESC2_PORTD_BIT) | (1<<ESC3_PORTD_BIT);
  PORTB |= (1<<ESC4_PORTB_BIT);

  unsigned long t1 = t0 + us1;
  unsigned long t2 = t0 + us2;
  unsigned long t3 = t0 + us3;
  unsigned long t4 = t0 + us4;

  uint8_t pendD = (1<<ESC1_PORTD_BIT) | (1<<ESC2_PORTD_BIT) | (1<<ESC3_PORTD_BIT);
  uint8_t pendB = (1<<ESC4_PORTB_BIT);

  while(pendD || pendB){
    unsigned long now = micros();
    if((pendD & (1<<ESC1_PORTD_BIT)) && now >= t1){ PORTD &= ~(1<<ESC1_PORTD_BIT); pendD &= ~(1<<ESC1_PORTD_BIT); }
    if((pendD & (1<<ESC2_PORTD_BIT)) && now >= t2){ PORTD &= ~(1<<ESC2_PORTD_BIT); pendD &= ~(1<<ESC2_PORTD_BIT); }
    if((pendD & (1<<ESC3_PORTD_BIT)) && now >= t3){ PORTD &= ~(1<<ESC3_PORTD_BIT); pendD &= ~(1<<ESC3_PORTD_BIT); }
    if((pendB & (1<<ESC4_PORTB_BIT)) && now >= t4){ PORTB &= ~(1<<ESC4_PORTB_BIT); pendB &= ~(1<<ESC4_PORTB_BIT); }
  }
}
