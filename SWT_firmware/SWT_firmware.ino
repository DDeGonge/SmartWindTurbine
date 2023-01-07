#include "pins.h"
#include "gparse.h"
#include <Arduino.h>
#include <Tlv493d.h>
#include <Wire.h>

#define SENSORFREQ 300 // 300 1000
#define CYCLEFREQ 300 // 300 1000

// General settings
int wind_station_samplerate_hz = 10;
int motor_adjust_min_delay_s = 3;
float base_motor_drivetime_ms = 1500;
float wind_dir_thresh = 0.3;
float collective_thresh = 0.2;
float coll_motor_drivetime_ms = 150;

// Anemometer settings
float an_zmax = 20;
float an_ymax = 5;
float anemometer_lpf = 0.3;
float an_maxspeeeed = 15;
float wvane_lpf = 2.0;
float wvane_zeropos = 0.35;
float coll_lpf = 2.0;
float coll_ymin = -5.0;
float coll_ymax = -33.0;

// Tracking vars
volatile float anemometer_speed_rps = 0;
volatile long anemometer_last_time = 0;

void setup() {
  Serial.begin(250000);

  // configure gpio
  pinMode(HS_S, INPUT);
  pinMode(HS_N, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SDA, OUTPUT);
  pinMode(M0_EN0, OUTPUT);
  pinMode(M0_EN1, OUTPUT);
  pinMode(M0_SLEEP, OUTPUT);
  pinMode(M1_EN0, OUTPUT);
  pinMode(M1_EN1, OUTPUT);
  pinMode(M1_SLEEP, OUTPUT);
  pinMode(TLV0_PWR, OUTPUT);
  pinMode(TLV1_PWR, OUTPUT);

  digitalWrite(M0_EN0, LOW);
  digitalWrite(M0_EN1, LOW);
  digitalWrite(M0_SLEEP, LOW);
  digitalWrite(M1_EN0, LOW);
  digitalWrite(M1_EN1, LOW);
  digitalWrite(M1_SLEEP, LOW);
  digitalWrite(TLV0_PWR, LOW);
  digitalWrite(TLV1_PWR, LOW);

  delay(100);

//  while(!Serial);
  Serial.println("INITIALIZING");

  // Boot tlv sensors individually to control addresses
  uint16_t stabilize_ms = 20;
  digitalWrite(SDA, LOW);
  delay(stabilize_ms);
  digitalWrite(TLV0_PWR, HIGH);
  delay(stabilize_ms);
  digitalWrite(SDA, HIGH);
  delay(stabilize_ms);
  digitalWrite(TLV1_PWR, HIGH);
  delay(stabilize_ms);

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(HS_N), anemometer_int, FALLING);

  // Blink LEDs to indicate it is booted
  for(uint8_t i=0; i < 3; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }

  // DEBUG MODE
//  manual_drive_motors();
}

void loop() {
  // Configure tlv hall sensors
  Wire.setClock(1000000);
  Tlv493d mag_wvane = Tlv493d();
  Tlv493d mag_collective = Tlv493d();

  mag_collective.begin(Wire, TLV493D_ADDRESS1, false);
  mag_collective.setAccessMode(mag_collective.ULTRALOWPOWERMODE);
  mag_collective.disableTemp();
  
  mag_wvane.begin(Wire, TLV493D_ADDRESS2, false);
  mag_wvane.setAccessMode(mag_wvane.ULTRALOWPOWERMODE);
  mag_wvane.disableTemp();

  Serial.println("SENSORS STARTED");

  float windvel = 0;
  float winddir = 0;
  float col_percent = 0;
  int windsamples = wind_station_samplerate_hz * motor_adjust_min_delay_s;
  int winddelay = 1000 / wind_station_samplerate_hz;
  float lpf_wvane = min(1.0, (wvane_lpf * winddelay) / 1000);

//  while(true)
//  {
//    drive_motor(M0_EN0, M0_EN1, M0_SLEEP, true, 1000);
//    drive_motor(M0_EN0, M0_EN1, M0_SLEEP, false, 1000);
//  }

  // control loop
  while(true)
  {
    // Get wind stats and collective pos
    for (uint16_t i = 0; i < 2; i++)
    {
      winddir += lpf_wvane * (get_wind_dir(mag_wvane) - winddir);
      delay(winddelay);
    }

    // Drive base rotation if needed
//    if (winddir < -wind_dir_thresh)
//    {
//      drive_motor(M0_EN0, M0_EN1, M0_SLEEP, true, base_motor_drivetime_ms);
//    }
//    else if (winddir > wind_dir_thresh)
//    {
//      drive_motor(M0_EN0, M0_EN1, M0_SLEEP, false, base_motor_drivetime_ms);
//    }

    // Drive collective pitch if needed
    windvel = get_wind_vel();
    col_percent = get_collective_percent(mag_collective);
    float col_target = min(1.0, windvel / an_maxspeeeed);
//    col_target = 0.7;
//    col_target = (col_target * 0.9) + 0.05;  // shift to avoid the edges idk it's fine
    float col_error = col_target - col_percent;
//    if (col_error < -collective_thresh)
//    {
//      drive_motor(M1_EN0, M1_EN1, M1_SLEEP, false, coll_motor_drivetime_ms);
//    }
//    else if (col_error > collective_thresh)
//    {
//      drive_motor(M1_EN0, M1_EN1, M1_SLEEP, true, coll_motor_drivetime_ms);
//    }
    Serial.print("windDir: ");
    Serial.print(winddir);
    Serial.print("\twindVel: ");
    Serial.println(windvel);
//    Serial.print("\tcollectiveTarget: ");
//    Serial.print(col_target);
//    Serial.print("\tcollectiveNow: ");
//    Serial.println(col_percent);
  }


  // debuggin mode from here on out
//  Serial.println("~ ~ ~ Manual drive commands ~ ~ ~");
//  Serial.println("Base motor positive - b0");
//  Serial.println("Base motor negative - b1");
//  Serial.println("Collective motor positive - c0");
//  Serial.println("Collective motor negative - c1");
//  Serial.println("Time arg - tNNNN. in ms. Default is 1000.");
//  Serial.println("ex. b1 t1500");
//
//  delay(2000);
//
//  char serial_data[MAX_MSG_LEN];
//  clear_data(serial_data);
//  char base_cmd, char_value;
//  int32_t base_value, int_value;
//  float float_value;
//
//  while (true)
//  {
//    windvel = get_wind_vel();
//    winddir += lpf_wvane * (get_wind_dir(mag_wvane) - winddir);
//    delay(winddelay);
//    Serial.print("Wind vel: ");
//    Serial.print(windvel);
//    Serial.print("\tWind dir: ");
//    Serial.println(winddir);
//
//    if (respondToSerial(serial_data)) 
//    {
//      vector<string> args;
//      parse_inputs(serial_data, args);
//      parse_int(args[0], base_cmd, base_value);
//      gcode_command_floats gcode(args);
//
//      switch (tolower(base_cmd)) 
//      {
//        case 'b': {
//          switch (base_value) 
//          {
//            case 0: {
//              float drivetime = gcode.com_exists('t') ? gcode.fetch('t') : 1000.0;
//              Serial.print("B POS ");
//              Serial.println(drivetime);
//              drive_motor(M0_EN0, M0_EN1, M0_SLEEP, true, drivetime);
//              break;
//            }
//            case 1: {
//              float drivetime = gcode.com_exists('t') ? gcode.fetch('t') : 1000.0;
//              Serial.print("B NEG ");
//              Serial.println(drivetime);
//              drive_motor(M0_EN0, M0_EN1, M0_SLEEP, false, drivetime);
//              break;
//            }
//          }
//          break;
//        }
//        case 'c': {
//          switch (base_value) 
//          {
//            case 0: {
//              float drivetime = gcode.com_exists('t') ? gcode.fetch('t') : 1000.0; 
//              Serial.print("C POS ");
//              Serial.println(drivetime);
//              drive_motor(M1_EN0, M1_EN1, M1_SLEEP, true, drivetime);
//              break;
//            }
//            case 1: {
//              float drivetime = gcode.com_exists('t') ? gcode.fetch('t') : 1000.0;
//              Serial.print("C NEG ");
//              Serial.println(drivetime);
//              drive_motor(M1_EN0, M1_EN1, M1_SLEEP, false, drivetime);
//              break;
//            }
//          }
//          break;
//        }
//      }
//      Serial.println("ok");
//      clear_data(serial_data);
//    }
//  }
}

void drive_motor(int pin_en0, int pin_en1, int pin_sleep, bool reverse, int time_ms)
{
  digitalWrite(pin_sleep, HIGH);
  if (reverse)
  {
    digitalWrite(pin_en0, HIGH);
    digitalWrite(pin_en1, LOW);
  }
  else
  {
    digitalWrite(pin_en0, LOW);
    digitalWrite(pin_en1, HIGH);
  }
  delay(time_ms);
  digitalWrite(pin_sleep, LOW);
}

void manual_drive_motors()
{
  Serial.println("~ ~ ~ Manual drive commands ~ ~ ~");
  Serial.println("Base motor positive - b0");
  Serial.println("Base motor negative - b1");
  Serial.println("Collective motor positive - c0");
  Serial.println("Collective motor negative - c1");
  Serial.println("Time arg - tNNNN. in ms. Default is 1000.");
  Serial.println("ex. b1 t1500");

  char serial_data[MAX_MSG_LEN];
  clear_data(serial_data);
  char base_cmd, char_value;
  int32_t base_value, int_value;
  float float_value;
  while (true)
  {
    if (respondToSerial(serial_data)) 
    {
      vector<string> args;
      parse_inputs(serial_data, args);
      parse_int(args[0], base_cmd, base_value);
      gcode_command_floats gcode(args);

      switch (tolower(base_cmd)) 
      {
        case 'b': {
          switch (base_value) 
          {
            case 0: {
              float drivetime = gcode.com_exists('t') ? gcode.fetch('t') : 1000.0;
              Serial.print("B POS ");
              Serial.println(drivetime);
              drive_motor(M0_EN0, M0_EN1, M0_SLEEP, true, drivetime);
              break;
            }
            case 1: {
              float drivetime = gcode.com_exists('t') ? gcode.fetch('t') : 1000.0;
              Serial.print("B NEG ");
              Serial.println(drivetime);
              drive_motor(M0_EN0, M0_EN1, M0_SLEEP, false, drivetime);
              break;
            }
          }
          break;
        }
        case 'c': {
          switch (base_value) 
          {
            case 0: {
              float drivetime = gcode.com_exists('t') ? gcode.fetch('t') : 1000.0; 
              Serial.print("C POS ");
              Serial.println(drivetime);
              drive_motor(M1_EN0, M1_EN1, M1_SLEEP, true, drivetime);
              break;
            }
            case 1: {
              float drivetime = gcode.com_exists('t') ? gcode.fetch('t') : 1000.0;
              Serial.print("C NEG ");
              Serial.println(drivetime);
              drive_motor(M1_EN0, M1_EN1, M1_SLEEP, false, drivetime);
              break;
            }
          }
          break;
        }
      }
      Serial.println("ok");
      clear_data(serial_data);
    }
  }
}

float get_wind_vel()
{
  return anemometer_speed_rps * 5;
}

float get_wind_dir(Tlv493d MagSensor)
{
  MagSensor.updateData();
  float xmag = MagSensor.getX(); // an_zmax;
  float ymag = MagSensor.getY(); // an_ymax;
  float winddir = atan2(xmag, ymag);
  winddir -= wvane_zeropos;
  winddir = winddir < -3.14 ? winddir + 6.28 : winddir;
  winddir = winddir > 3.14 ? winddir - 6.28 : winddir;
  return winddir;
}

float get_collective_percent(Tlv493d MagSensor)
{
  MagSensor.updateData();
  float ymag = MagSensor.getY();
  float coll_delta = coll_ymax - coll_ymin;
//  Serial.print("ymag: ");
//  Serial.println(ymag);
  if (coll_delta < 0)
  {
    if (ymag <= coll_ymax) return 1.0;
    if (ymag >= coll_ymin) return 0.0;
  }
  else
  {
    if (ymag >= coll_ymax) return 1.0;
    if (ymag <= coll_ymin) return 0.0;
  }
  return (ymag - coll_ymin) / coll_delta;
}

void print_mag(Tlv493d MagSensor)
{
  MagSensor.updateData();
  Serial.print("X:");
  Serial.print(MagSensor.getX());
  Serial.print(",Y:");
  Serial.print(MagSensor.getY());
  Serial.print(",Z:");
  Serial.println(MagSensor.getZ());
}

void anemometer_int()
{
  long this_time = micros();
  long time_delta = this_time - anemometer_last_time;
  float new_vel_rps = 1000000 / time_delta;
  float lpf_mult = min(1.0, (anemometer_lpf * time_delta) / 1000000);
  anemometer_speed_rps += lpf_mult * (new_vel_rps - anemometer_speed_rps);
  anemometer_last_time = this_time;
}
