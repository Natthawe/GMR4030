#define CONNECT_SERVER true
#define CONNECT_A1454 true

//Robot Motion Config =====================
#define POS_ONE_BLOCK 23875 //24060  //24384 //7519/5*16=24061
#define POS_TURN_90  10500    //12320    //10432 //(3250/5)*16 = 10400 //new 3207/5*16=10262
#define POS_TURN_180 20600 //24640 //20800   //24040   //20736 //20864//(3240*2)/5*16 = 20736 6d1d6d1d
#define POS_CHARGING 3000
#define SPEED_ACCEL 250  //500
#define TURN_MAX_SPEED 20000
#define MAX_SPEED 40000
#define MIN_SPEED 1200
//===========================================
#define EMER_SW_GPIO 39

TaskHandle_t userTask;

#include <SoftwareSerial.h>
EspSoftwareSerial::UART PortRFID;
const unsigned char SL031CMD_SelectCard[] = { 0xBA, 0x02, 0x01, 0xB9 };
const unsigned char SL031CMD_LoginSector0[] = { 0xBA, 0x0A, 0x02, 0x00, 0xAA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x18 };
const unsigned char SL031CMD_ReadBlock1[] = { 0xBA, 0x03, 0x03, 0x01, 0xBB };
const unsigned char SL031CMD_WriteBlock1[] = { 0xBA, 0x13, 0x04, 0x01, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA5 };

unsigned char chargeState = 0;
unsigned char last_chargeState = -1;

int vbatt = 0;
int offset_vbatt = 0;

char robotname[16] = { 0 };
unsigned char pos[3] = {0,0,0}; 
char last_pos[3] = { 1, 2, 0 };  //x,y,dir
int _isreadypath = 1;
uint32_t chipId = 0;
void _getChipId() {
  for (int i = 0; i < 17; i = i + 8) chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  //sprintf(robotname, "ROBOT1:%06X", chipId);
  switch(chipId)
  {
    case 0xD228A8: sprintf(robotname, "ROBOT1:%06X", chipId); offset_vbatt = 160; break;
    case 0xA507FC: sprintf(robotname, "ROBOT2:%06X", chipId); offset_vbatt = 0; break;
    case 0x6D4F44: sprintf(robotname, "ROBOT3:%06X", chipId); offset_vbatt = 170; break;
    case 0xA4E784: sprintf(robotname, "ROBOT4:%06X", chipId); offset_vbatt = 90; break;
  }
  //sprintf(robotname,"%x",chipId);
}

char server[] = "10.141.80.22";
//char server[] = "10.1.10.254";

#include "EEPROM.h"
#define EEPROM_SIZE 64

//WiFi And Telnet Debug Config ==============
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiUdp.h>
#include <WiFiAP.h>

const char* ssid = "GroupMaker";
const char* password = "0815935599";

// const char* ssid = "Pandora_MFG_OT" ;
// const char* password = "P@ndoraOT";

//WiFiClient client;

#include <SoftwareSerial.h>

void _WiFi_Init() 
{
  //WiFi.softAP(ssid, password);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("WL0 IP address: ");
  Serial.println(WiFi.localIP());
}
//===========================================
//Ethernet W5500 Config =====================
#include <SPI.h>
#include <Ethernet.h>
#define W5500_CS_GPIO 15

EthernetClient client;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
void _W5500_Init() {
  pinMode(W5500_CS_GPIO, OUTPUT);
  digitalWrite(W5500_CS_GPIO, HIGH);
  Ethernet.init(W5500_CS_GPIO);

  Serial.println("Trying to get an IP address using DHCP");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
    // initialize the Ethernet device not using DHCP:
    //Ethernet.begin(mac, ip, myDns, gateway, subnet);
  }
  // print your local IP address:
  Serial.print("Eth IP address: ");
  Serial.println(Ethernet.localIP());
}
void _W5500_LinkStatus() {
}
//===========================================
//OTA And Telnet Config =====================
#include <ArduinoOTA.h>
#include <TelnetStream.h>
void _OTATelnet_Init() {
  ArduinoOTA.setHostname(robotname);
  ArduinoOTA.setPassword("zzzzzzzz");
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else  // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });


  ArduinoOTA.begin();
  TelnetStream.begin();
}
//Sensor A1454 Config =======================
//===========================================
#include <Wire.h>
#define A1454_TEMPOUT (0x1D)
#define A1454_OUTPUT (0x1F)
#define A1454_SLEEP (0x20)
#define A1454_ACCESS_ADDRESS (0x24)
#define A1454_ACCESS_CODE (0x2C413534)
unsigned char sensor[4];
unsigned char sensor_index = 0;
void _A1454_Init() {
  A1454_begin(0x64);
  A1454_begin(0x66);
  A1454_begin(0x67);
  A1454_begin(0x69);
}
//===========================================
//TMC5160PRO Config =========================
#include <TMCStepper.h>
#define TMC5160_CS_L 12
#define TMC5160_CS_R 13
#define TMC5160_EN 14
#define R_SENSE 0.075f
#define STALL_VALUE 15  // [-64..63]
TMC5160Stepper driverL = TMC5160Stepper(TMC5160_CS_L, R_SENSE);
TMC5160Stepper driverR = TMC5160Stepper(TMC5160_CS_R, R_SENSE);
void _TMC5160_Init() {
  pinMode(TMC5160_CS_L, OUTPUT);
  pinMode(TMC5160_CS_R, OUTPUT);
  pinMode(TMC5160_EN, OUTPUT);
  digitalWrite(TMC5160_EN, HIGH);
  digitalWrite(TMC5160_CS_L, HIGH);
  digitalWrite(TMC5160_CS_R, HIGH);
  SPI.begin();
  delay(100);
  Serial.println("Init DriveL");
  driverL.begin();  // Initiate pins and registeries
  driverL.toff(4);
  driverL.rms_current(2400, 0.1);  // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driverL.en_pwm_mode(1);          // Enable extremely quiet stepping
  driverL.pwm_autoscale(1);
  driverL.microsteps(16);
  driverL.TCOOLTHRS(0xFFFFF);  // 20bit max
  driverL.THIGH(0);
  driverL.semin(5);
  driverL.semax(2);
  driverL.sedn(0b01);
  driverL.sgt(STALL_VALUE);
  driverL.shaft(0);
  Serial.print("DRVL_STATUS=0b");
  Serial.println(driverL.DRV_STATUS(), BIN);
  delay(10);
  Serial.println("Init DriveR");
  driverR.begin();  // Initiate pins and registeries
  driverR.toff(4);
  driverR.rms_current(2400, 0.1);  // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driverR.en_pwm_mode(1);          // Enable extremely quiet stepping
  driverR.pwm_autoscale(1);
  driverR.microsteps(16);
  driverR.TCOOLTHRS(0xFFFFF);  // 20bit max
  driverR.THIGH(0);
  driverR.semin(5);
  driverR.semax(2);
  driverR.sedn(0b01);
  driverR.sgt(STALL_VALUE);
  driverR.shaft(1);
  Serial.print("DRVR_STATUS=0b");
  Serial.println(driverR.DRV_STATUS(), BIN);
}
unsigned int _TMC5160_Status(unsigned char n) {
  unsigned int status = 0;
  if (n == 0) {
    TelnetStream.print("DRV0_STATUS=0b");
    status = driverL.DRV_STATUS();
    TelnetStream.println(status, BIN);
  } else if (n == 1) {
    TelnetStream.print("DRV1_STATUS=0b");
    status = driverR.DRV_STATUS();
    TelnetStream.println(status, BIN);
  }
  return status;
}
//===========================================
//Motion Control Loop And GPIO Config ============
#include "ESP32_FastPWM.h"
#include "FastInterruptEncoder.h"
#define MOTOR_LEFT_PWM_GPIO 33
#define MOTOR_LEFT_DIR_GPIO 32
#define MOTOR_RIGHT_PWM_GPIO 27
#define MOTOR_RIGHT_DIR_GPIO 26
#define MOTOR_ENCODER_GPIO 25
volatile unsigned long pos_sp = 0, pos_pv = 0;
float error = 0.0;
float kp = 0.7;
float kp_turn = 1.5;
float pid_output;
volatile unsigned int speed_pv = 0, speed_sp = 0;
int diff_speed = 0;
Encoder _enc(MOTOR_ENCODER_GPIO, NULL, SINGLE, 0);
int PWM_resolution = 10.0f;
uint32_t PWM_Pins[] = { MOTOR_LEFT_PWM_GPIO, MOTOR_RIGHT_PWM_GPIO };
uint32_t PWM_chan[] = { 2, 4 };
float dutyCycle[] = { 0.0f, 0.0f };
float frequency[] = { 1000.0f, 1000.0f };
ESP32_FAST_PWM* PWM_Instance[2];
void _MotionControl_Init() {
  //pinMode(35, INPUT_PULLDOWN);
  //digitalWrite(35, HIGH);
  pinMode(MOTOR_LEFT_PWM_GPIO, OUTPUT);
  pinMode(MOTOR_LEFT_DIR_GPIO, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM_GPIO, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR_GPIO, OUTPUT);
  pinMode(MOTOR_ENCODER_GPIO, OUTPUT);
  //pinMode(ALARM_LEFT, INPUT_PULLUP);
  //pinMode(ALARM_RIGHT, INPUT_PULLUP);
  digitalWrite(MOTOR_ENCODER_GPIO, LOW);

  _enc.init();

  PWM_Instance[0] = new ESP32_FAST_PWM(PWM_Pins[0], frequency[0], dutyCycle[0], PWM_chan[0], PWM_resolution);
  PWM_Instance[1] = new ESP32_FAST_PWM(PWM_Pins[1], frequency[1], dutyCycle[1], PWM_chan[1], PWM_resolution);
  PWM_Instance[0]->setPWM();
  PWM_Instance[1]->setPWM();
}
void _setSpeed(unsigned int spdL, unsigned int spdR) {
  int sL, sR;
  if (spdL > 0 && spdL < MIN_SPEED) spdL = MIN_SPEED;
  if (spdR > 0 && spdR < MIN_SPEED) spdR = MIN_SPEED;
  int _temp = (spdL / 100);
  sL = _temp * 100;
  _temp = (spdR / 100);
  sR = _temp * 100;
  if (sL > 0) PWM_Instance[0]->setPWM(PWM_Pins[0], sL, 25.0f);//25.0
  else PWM_Instance[0]->setPWM();
  if (sR > 0) PWM_Instance[1]->setPWM(PWM_Pins[1], sR, 25.0f);
  else PWM_Instance[1]->setPWM();
}
//===========================================
//===========================================
//===========================================

//Slave MCU =================================
// void _sMCU_Init() {
//   Serial2.begin(115200);
// }
// void _sMCU_Loop() {
// }
// void _playSound_playLED(String s) {
//   TelnetStream.println("PlaySound and PlayLed");
//   Serial2.println("1");
// }
//===========================================
//===========================================
//===========================================



unsigned int _motion_run = 0;    //0=Done
unsigned int _motion_block = 0;  //0=Done
unsigned int _last_motion_block = 0;
unsigned int _motion_block_sp = 0;  //0=Done
unsigned int _isRunning = false;
unsigned int pos_x = 1, pos_y = 3, pos_dir = 0;
volatile unsigned char _reqUpdate = 0;
unsigned char _sensor_detect = 0;
unsigned char rfid_sensor = 0;
unsigned char last_rfid_sensor = 0;
unsigned char rfid_buffer_index=0;
unsigned char rfid_data_detect=0;

//String robot_name = "A";
String route_buffer = "";
unsigned int route_buffer_index = 0;
unsigned char detect_data = 0;
String str_recv = "";
String str_head = "";
String str_val = "";

unsigned int last_block;
unsigned long previousMillis = 0;  // will store last time LED was updated
unsigned int task_500ms = 0;
unsigned char _cmd_detect = 0;
String _cmd_data = "";
unsigned char statusEmer = 0;
unsigned char last_statusEmer = -1;

volatile unsigned char init_position = 0;
volatile unsigned char init_pos_step = 0;

void _gotoBlock(unsigned char n) {
  digitalWrite(MOTOR_LEFT_DIR_GPIO, LOW);
  digitalWrite(MOTOR_RIGHT_DIR_GPIO, LOW);
  _enc.resetTicks();
  pos_sp = POS_ONE_BLOCK * n;
  _motion_block_sp = n;
  _sensor_detect = 0;
  _motion_block = 0;
  _motion_run = 1;  //Reset Flag and Start Motion Control
}
void _turnR90() {
  digitalWrite(MOTOR_LEFT_DIR_GPIO, LOW);
  digitalWrite(MOTOR_RIGHT_DIR_GPIO, HIGH);
  _enc.resetTicks();
  pos_sp = POS_TURN_90;
  diff_speed = 0;
  _sensor_detect = 0;
  _motion_block = 0;
  _motion_block_sp = 1;
  _motion_run = 2;  //Reset Flag and Start Motion Control
}
void _turnL90() {
  digitalWrite(MOTOR_LEFT_DIR_GPIO, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR_GPIO, LOW);
  _enc.resetTicks();
  pos_sp = POS_TURN_90;
  diff_speed = 0;
  _sensor_detect = 0;
  _motion_block = 0;
  _motion_block_sp = 1;
  _motion_run = 3;  //Reset Flag = Start Motion
}
void _turnR180() {
  digitalWrite(MOTOR_LEFT_DIR_GPIO, LOW);
  digitalWrite(MOTOR_RIGHT_DIR_GPIO, HIGH);
  _enc.resetTicks();
  pos_sp = POS_TURN_180;
  diff_speed = 0;
  _sensor_detect = 0;
  _motion_block = 0;
  _motion_block_sp = 1;
  _motion_run = 4;  //Reset Flag and Start Motion Control
}

void _turnL180() {
  digitalWrite(MOTOR_LEFT_DIR_GPIO, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR_GPIO, LOW);
  _enc.resetTicks();
  pos_sp = POS_TURN_180;
  diff_speed = 0;
  _sensor_detect = 0;
  _motion_block = 0;
  _motion_block_sp = 1;
  _motion_run = 4;  //Reset Flag and Start Motion Control
}

void _turnOnCharging() {
  digitalWrite(MOTOR_LEFT_DIR_GPIO, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR_GPIO, HIGH);
  _enc.resetTicks();
  pos_sp = POS_CHARGING;
  diff_speed = 0;
  _sensor_detect = 0;
  // _motion_block = 0;
  // _motion_block_sp = 1;
  _motion_run = 5;  //Reset Flag and Start Motion Control
}

void _turnOffCharging() {
  digitalWrite(MOTOR_LEFT_DIR_GPIO, LOW);
  digitalWrite(MOTOR_RIGHT_DIR_GPIO, LOW);
  _enc.resetTicks();
  pos_sp = POS_CHARGING;
  diff_speed = 0;
  _sensor_detect = 0;
  // _motion_block = 0;
  // _motion_block_sp = 1;
  _motion_run = 5;  //Reset Flag and Start Motion Control
}

// void _tracking() {
//   if (sensor[0] && sensor[1] && sensor[2] && sensor[3]) {
//     diff_speed = 0;
//   }
//   if (!sensor[0] && sensor[1] && sensor[2] && !sensor[3]) {
//     diff_speed = 0;
//   }
//   if (!sensor[0] && sensor[1] && !sensor[2] && !sensor[3]) {
//     diff_speed = -20;
//   }
//   if (sensor[0] && sensor[1] && !sensor[2] && !sensor[3]) {
//     diff_speed = -30;
//   }
//   if (sensor[0] && !sensor[1] && !sensor[2] && !sensor[3]) {
//     diff_speed = -50;
//   }
//   if (!sensor[0] && !sensor[1] && sensor[2] && !sensor[3]) {
//     diff_speed = 20;
//   }
//   if (!sensor[0] && !sensor[1] && sensor[2] && sensor[3]) {
//     diff_speed = 30;
//   }
//   if (!sensor[0] && !sensor[1] && !sensor[2] && sensor[3]) {
//     diff_speed = 50;
//   }
// }

void _tracking() {
  if (sensor[0] && sensor[1] && sensor[2] && sensor[3]) {
    diff_speed = 0;
  }
  if (!sensor[0] && sensor[1] && sensor[2] && !sensor[3]) {
    diff_speed = 0;
  }
  if (!sensor[0] && sensor[1] && !sensor[2] && !sensor[3]) {
    diff_speed = -35;//-25
  }
  if (sensor[0] && sensor[1] && !sensor[2] && !sensor[3]) {
    diff_speed = -50;//-40
  }
  if (sensor[0] && !sensor[1] && !sensor[2] && !sensor[3]) {
    diff_speed = -80;//-100
  }
  if (!sensor[0] && !sensor[1] && sensor[2] && !sensor[3]) {
    diff_speed = 35;
  }
  if (!sensor[0] && !sensor[1] && sensor[2] && sensor[3]) {
    diff_speed = 50;
  }
  if (!sensor[0] && !sensor[1] && !sensor[2] && sensor[3]) {
    diff_speed = 80;
  }
}


void MotionControl_Loop() {
  _enc.loop();
  pos_pv = _enc.getTicks();
  if (_motion_run) 
  {
    error = pos_sp - pos_pv;
    if(error > 800000.0)error = 800000;

    //pid_output = kp * error;
    if(_motion_run >= 2)
    {
      pid_output = kp_turn * error;
    }
    else //=1
    {
      pid_output = kp * error;
    }

    if (_motion_run >= 2) 
    {
      if (pid_output > TURN_MAX_SPEED) pid_output = TURN_MAX_SPEED;
    }
    else if (pid_output > MAX_SPEED) pid_output = MAX_SPEED;

    if (pid_output < 0) pid_output = 0;
    speed_sp = pid_output;
    if (speed_pv < speed_sp) speed_pv += SPEED_ACCEL;  //Accel
    if (speed_pv > speed_sp) speed_pv = speed_sp;      //PID Decel

    if (_motion_run == 1) _tracking();

    // if (diff_speed == 0) _setSpeed(speed_pv, speed_pv);
    // else 
    // {
    //   float _diff = pid_output * abs(diff_speed) / 1000;
    //   if (diff_speed > 0) _setSpeed(speed_pv, speed_pv - _diff);
    //   else if (diff_speed < 0) _setSpeed(speed_pv - _diff, speed_pv);
    // }
    if (diff_speed == 0) _setSpeed(speed_pv, speed_pv);
    else 
    {
      float _diff = pid_output * abs(diff_speed) / 1000;
      if (diff_speed > 0) _setSpeed(speed_pv + _diff, speed_pv );
      else if (diff_speed < 0) _setSpeed(speed_pv, speed_pv + _diff);
    }

  } else _setSpeed(0, 0);

  if (_motion_run == 1) 
  { 
    if (pos_pv >= pos_sp - (POS_ONE_BLOCK / 2)) 
    {
      //if (sensor[0] && sensor[1] && sensor[2] && sensor[3] && _sensor_detect == 0) 
      if (sensor[0]  && sensor[3] && _sensor_detect == 0) 
      {
        
        pos_sp = pos_pv + 5300;
        //pos_sp = pos_pv + 5500;
        diff_speed = 0;
        _sensor_detect = 1;
        TelnetStream.println("SLOW BLOCK==============");
        TelnetStream.print(sensor[0]);
        TelnetStream.print(" ");
        TelnetStream.print(sensor[1]);
        TelnetStream.print(" ");
        TelnetStream.print(sensor[2]);
        TelnetStream.print(" ");
        TelnetStream.print(sensor[3]);
        TelnetStream.print(" ");
        TelnetStream.print(pos_sp);
        TelnetStream.print(" ");
        TelnetStream.println(pos_pv);
        TelnetStream.println("=======================");
      }
    }
  }

  if (_motion_run == 2 || _motion_run == 3 || _motion_run == 4) {
    if (pos_pv >= pos_sp * 0.8)  //80%
    {
      if (sensor[1] && sensor[2] && _sensor_detect == 0) {
        //pos_sp = pos_pv + 425;
        diff_speed = 0;
        pos_sp = pos_pv + 600;
        _sensor_detect = 1;
        TelnetStream.println("TURN SENSOR =======================");
        TelnetStream.print(sensor[0]);
        TelnetStream.print(" ");
        TelnetStream.print(sensor[1]);
        TelnetStream.print(" ");
        TelnetStream.print(sensor[2]);
        TelnetStream.print(" ");
        TelnetStream.print(sensor[3]);
        TelnetStream.print(" ");
        TelnetStream.print(pos_sp);
        TelnetStream.print(" ");
        TelnetStream.print(pos_pv);
        TelnetStream.print(" ");
        TelnetStream.println(_motion_run);
        TelnetStream.println("=======================");
      }
    }
  }



  switch (_motion_run) 
  {
    case 1:
      if (CONNECT_SERVER)
      {
        if (_isRunning == 0 && _motion_block < _motion_block_sp - 1) 
        {
          pos_sp = POS_ONE_BLOCK * (_motion_block + 2);
          _motion_block_sp = _motion_block + 2;
        }
      }
      if (_motion_block >= _motion_block_sp - 1) 
      {
        if (pos_pv >= pos_sp) _motion_block++;
        if (_motion_block >= _motion_block_sp) 
        {
          route_buffer = "";
          route_buffer_index = 0;
          // _reqUpdate = 1;
        }
      } 
      else 
      {
        _motion_block = pos_pv / (POS_ONE_BLOCK);
      }
      break;

    case 2:
      if (pos_pv >= pos_sp) _motion_block++;
      break;
    case 3:
      if (pos_pv >= pos_sp) _motion_block++;
      break;
    case 4:
      if (pos_pv >= pos_sp) _motion_block++;
      break;
  }

  if (_motion_block != _last_motion_block) {
    TelnetStream.print("Block ");
    TelnetStream.print(_motion_block);
    TelnetStream.print(" / ");
    TelnetStream.println(_motion_block_sp);
    _last_motion_block = _motion_block;

    if (_motion_block != 0) {
      switch (_motion_run) {
        case 1:
          switch (pos_dir) {
            case 0: pos_y++; break;
            case 1: pos_x++; break;
            case 2: pos_y--; break;
            case 3: pos_x--; break;
          }
          break;
        case 2:
          switch (pos_dir) {
            case 0: pos_dir = 1; break;
            case 1: pos_dir = 2; break;
            case 2: pos_dir = 3; break;
            case 3: pos_dir = 0; break;
          }
          break;
        case 3:
          switch (pos_dir) {
            case 0: pos_dir = 3; break;
            case 1: pos_dir = 0; break;
            case 2: pos_dir = 1; break;
            case 3: pos_dir = 2; break;
          }
          break;
        case 4:
          switch (pos_dir) {
            case 0: pos_dir = 2; break;
            case 1: pos_dir = 3; break;
            case 2: pos_dir = 0; break;
            case 3: pos_dir = 1; break;
          }
          break;
      }
    }

    _reqUpdate = 1;
    //_motion_run=0;//Done
  }

  if (pos_pv >= pos_sp) 
  {
    _motion_run = 0;  //Done
  }
}

const TickType_t xDelay1000ms = pdMS_TO_TICKS(1000); 
void userTaskFunc(void *parameter) 
{
  (void)parameter;
  uint32_t count = 0;
  for (;;) 
  {
    vTaskDelay(xDelay1000ms);
    if(init_position==1)httpRequestInit();

    if(_reqUpdate>=1)
    {
      _reqUpdate=0;
      httpRequest();
    }
  }
}

void setup() {
  EEPROM.begin(EEPROM_SIZE);
  pos_x = EEPROM.read(0);
  pos_y = EEPROM.read(1);
  pos_dir = EEPROM.read(2);

  pinMode(36,INPUT_PULLUP);

  _getChipId();

  pinMode(EMER_SW_GPIO, INPUT_PULLUP);

  Serial.begin(115200);

  Serial2.begin(115200,SERIAL_8N1, 16, 17); //robot 2-3-4

  // Serial2.begin(115200,SERIAL_8N1, 17, 16); //robot 1

  
  pinMode(4,INPUT_PULLUP);//RFID IN
  //_sMCU_Init();

  //delay(2000);

  _W5500_Init();

  _WiFi_Init();

  _OTATelnet_Init();

  xTaskCreatePinnedToCore(
    userTaskFunc,   // Taskentry function
    "UserTAsk",     // Task name
    4096,           // Stack size (in words)
    NULL,           // Task parameter
    2,              // Task priority (higher than the main task)
    &userTask,      // Task handle
    0               // Core ID (either 0 or 1)
  );

  _TMC5160_Init();

  _A1454_Init();

  _MotionControl_Init();

  _setSpeed(0, 0);

  delay(2000);

  _setSpeed(0, 0);
}


void init_position_proc() //alignment
{
  if(init_position == 0 && _motion_run == 0 && init_pos_step <= 5 && statusEmer == 0)
  {
    switch(init_pos_step)
    {
      case 0: _turnR90(); init_pos_step++; break;
      case 1: //delay(1000); 
      init_pos_step++; break;
      case 2: _turnR90(); init_pos_step++; break;
      case 3: //delay(1000); 
      init_pos_step++; break;
      case 4: _turnR90(); init_pos_step++; break;
      case 5: delay(2000);
      init_pos_step++; break;
    }
  }

}

void loop() 
{
  ArduinoOTA.handle();

  //Emer active LOW
  if (digitalRead(EMER_SW_GPIO) == LOW)
  {
    if(statusEmer==1)delay(2000);
    digitalWrite(TMC5160_EN, LOW);
    statusEmer=0;
  }
  else
  {
    digitalWrite(TMC5160_EN, HIGH);
    statusEmer=1;
    route_buffer="";
    route_buffer_index=0;

  } 
  if(statusEmer != last_statusEmer)
  {
    last_statusEmer = statusEmer;
    if(statusEmer)
    {
      
      init_position = 0;
      init_pos_step = 0;
    }
  }

  init_position_proc();

  if (_isRunning == 1 && _motion_run == 0 && init_position == 2) 
  {
    if (route_buffer_index < route_buffer.length()) 
    {
      char c = route_buffer[route_buffer_index++];
      TelnetStream.print("RUN->");
      TelnetStream.print(c);
      TelnetStream.print(" ");
      int b = 1;
      //for (int i = 1; i < route_buffer.length(); i++) 
      int max_b = route_buffer.length();
      if(max_b >= 39) max_b=39;
      // for (int i = 1; i < route_buffer.length(); i++) 
      for (int i = 1; i < max_b; i++) 
      {
        if (route_buffer[i] == c) 
        {
          b++;
        } 
        else 
        {
          i = 9999;
          break;
        }
      }
      switch (c) 
      {
        case 'W': _gotoBlock(b); break;
        case 'E': _gotoBlock(b); break;
        case 'N': _gotoBlock(b); break;
        case 'S': _gotoBlock(b); break;
        case 'R': _turnR90(); break;
        case 'L': _turnL90(); break;
        case 'T': _turnR180(); break;
        case '\n': break;
        case '\r':
          break;
          //default:run_mode_flag=0;  break;
      }
      //      if(route_buffer_index >= route_buffer.length())
      //      {
      //        route_buffer="";
      //        route_buffer_index=0;
      //      }
    }
  }

  if ((TelnetStream.available() || Serial.available()) && _motion_run == 0) 
  {
    char c;
    if (TelnetStream.available()) c = TelnetStream.read();
    if (Serial.available()) c = Serial.read();
    //Serial.println(c);
    if (c == '{' && _cmd_detect == 0) {
      _cmd_detect = 1;
      _cmd_data = "";
      //TelnetStream.println("CMD DETECT");
    }
    if (_cmd_detect) {
      _cmd_data += c;

      if (c == '}')  //End cmd
      {
        //TelnetStream.println("CMD END");
        _cmd_data.replace("{", "");
        _cmd_data.replace("}", "");
        _cmd_data.replace("[", "");
        _cmd_data.replace("]", "");
        int inx = _cmd_data.indexOf(':');
        String _head = _cmd_data.substring(0, inx);
        String _val = _cmd_data.substring(inx + 1, _cmd_data.length());
        //TelnetStream.println(_head);
        //TelnetStream.println(_val);
        if (_head == "setmaxspeed") {
        }
        if (_head == "setminspeed") {
        }
        if (_head == "setpos") {
          String _data[10];
          int _data_count = 0;
          while (_val.length() > 0) {
            int index = _val.indexOf(',');
            if (index == -1)  // No space found
            {
              _data[_data_count++] = _val;
              break;
            } else {
              _data[_data_count++] = _val.substring(0, index);
              _val = _val.substring(index + 1);
            }
          }
          if (_data_count == 3) {
            int temp = _data[0].toInt();
            if (temp >= 0 && temp <= 100) pos_x = temp;
            temp = _data[1].toInt();
            if (temp >= 0 && temp <= 100) pos_y = temp;
            temp = _data[2].toInt();
            if (temp >= 0 && temp <= 3) pos_dir = temp;

            EEPROM.write(0, pos_x);
            EEPROM.write(1, pos_y);
            EEPROM.write(2, pos_dir);
            EEPROM.commit();

            // TelnetStream.print("Pos_X=");
            // TelnetStream.println(pos_x);
            // TelnetStream.print("Pos_Y=");
            // TelnetStream.println(pos_y);
            // TelnetStream.print("Pos_DIR=");
            // TelnetStream.println(pos_dir);
            //   TelnetStream.print(": \"");
            // for (int i = 0; i < _data_count; i++)
            // {
            //   TelnetStream.print(i);
            //   TelnetStream.print(": \"");
            //   TelnetStream.print(_data[i]);
            //   TelnetStream.println("\"");
            // }
          }
          // for (int i = 0; i < _data_count; i++)
          // {
          //   TelnetStream.print(i);
          //   TelnetStream.print(": \"");
          //   TelnetStream.print(_data[i]);
          //   TelnetStream.println("\"");
          // }
        }
        _cmd_detect = 0;
        _cmd_data = "";
      }

    } else if (!_cmd_detect) {
      switch (c) {
        case '1': _gotoBlock(1); break;
        case '2': _gotoBlock(2); break;
        case '3': _gotoBlock(3); break;
        case '4': _gotoBlock(4); break;
        case '5': _gotoBlock(5); break;
        case '6': _gotoBlock(6); break;
        case '7': _gotoBlock(7); break;
        case '8': _gotoBlock(8); break;
        case 'd': _turnR90(); break;
        case 'a': _turnL90(); break;
        case 'e': _turnR180(); break;
        case 'q': _turnL180(); break;
        case 'c': _turnOnCharging(); break;
        case 'v': _turnOffCharging(); break;
      }
    }
  }
  
  unsigned long currentMillis = millis();

 // _enc.loop();

  if (currentMillis - previousMillis >= 5) 
  {
    previousMillis = currentMillis;

    MotionControl_Loop();
    

    read_Sensor(sensor_index++);//0 1 2 3 
    if (sensor_index >= 4) sensor_index = 0;

    if (task_500ms++ >= 100) 
    {
      task_500ms = 0;
      if (_motion_run == 0) _reqUpdate = 1;

      if(vbatt==0)vbatt = analogRead(36);
      else
      {
         vbatt += analogRead(36);
         vbatt /= 2;
      }

      TelnetStream.print(sensor[0]);
      TelnetStream.print(" ");
      TelnetStream.print(sensor[1]);
      TelnetStream.print(" ");
      TelnetStream.print(sensor[2]);
      TelnetStream.print(" ");
      TelnetStream.print(sensor[3]);

      TelnetStream.print(" ");
      TelnetStream.print(pos_x);
      TelnetStream.print(",");
      TelnetStream.print(pos_y);
      TelnetStream.print(",");
      TelnetStream.print(pos_dir);
      TelnetStream.print(" ");
      TelnetStream.print(rfid_sensor);
      TelnetStream.print(" ");
      
      // TelnetStream.print(vbatt + 110); //robot1
      // TelnetStream.print(vbatt - 90); //robot2
      // TelnetStream.print(vbatt + 60); //robot3
      // TelnetStream.print(vbatt); //robot4
      TelnetStream.print(vbatt + offset_vbatt);
      TelnetStream.print (" ");
      TelnetStream.print(init_position);
      TelnetStream.print (" ");
      TelnetStream.print(_motion_run);
      TelnetStream.print (" ");
      TelnetStream.print(statusEmer);
      TelnetStream.print(" ");
      TelnetStream.print(chargeState);
      TelnetStream.println (" ");      
      

      //if(init_position==1)httpRequestInit();


      if (pos_x != last_pos[0] || pos_y != last_pos[1] || pos_dir != last_pos[2]) 
      {
        last_pos[0] = pos_x;
        last_pos[1] = pos_y;
        last_pos[2] = pos_dir;
        EEPROM.write(0, pos_x);
        EEPROM.write(1, pos_y);
        EEPROM.write(2, pos_dir);
        EEPROM.commit();
      }
    }
  }
  // if (_reqUpdate) {
  //   httpRequest();
  //   _reqUpdate = 0;
  // }
  ethProcess();

  if(_motion_run==0)
    if(digitalRead(4) == LOW)rfid_sensor = 1; else rfid_sensor = 0;

  if(rfid_sensor != last_rfid_sensor)
  {
    last_rfid_sensor = rfid_sensor;
    if(rfid_sensor)
    {
      Serial2.write(SL031CMD_SelectCard, sizeof(SL031CMD_SelectCard));
      delay(100);
      Serial2.write(SL031CMD_LoginSector0, sizeof(SL031CMD_LoginSector0));
      delay(100);
      Serial2.write(SL031CMD_ReadBlock1, sizeof(SL031CMD_ReadBlock1));
      delay(100);
    }
  }

  while (Serial2.available()) 
  {
    char c = Serial2.read();
    //TelnetStream.print(c,HEX);
    //TelnetStream.print(" ");
    if(c==0xBD)
    {
      rfid_data_detect=1;
      rfid_buffer_index=0;
    }
    if(rfid_data_detect==1)
    {
      if(rfid_buffer_index==1 && c != 0x13){rfid_data_detect=0;}
      if(rfid_buffer_index==2 && c != 0x03){rfid_data_detect=0;}
      if(rfid_buffer_index==3 && c != 0x00){rfid_data_detect=0;}
      if(rfid_buffer_index==4){pos[0]=c; pos[2]=1;}
      if(rfid_buffer_index==5){pos[1]=c; pos[2]=1;}
      if(rfid_buffer_index==6){if(c==0x00)pos[2]=1; else if(c==0x01)pos[2]=2;}
      if(rfid_buffer_index>=6)
      {
        rfid_data_detect=0;
        TelnetStream.print("pos = ");
        TelnetStream.print(pos[0]);
        TelnetStream.print(" , ");
        TelnetStream.print(pos[1]);
        TelnetStream.print(" , ");
        TelnetStream.println(pos[2]);
        pos_x = pos[0];
        pos_y = pos[1];
        pos_dir = pos[2];

        if(init_position==0)
        {
          init_position=1;
          httpRequestInit();
        }
      }
      rfid_buffer_index++;
    }
  }
}


void ethProcess() {
  if (CONNECT_SERVER) {
    if (client.available()) {
      char c = client.read();
      //if(c!='"')Serial.write(c);
      if (c == '{') {
        detect_data = 1;
        str_recv = "";
      }
      if (detect_data) {
        if (c != '"') str_recv += c;
        if (c == '}') {
          detect_data = 0;
          //Serial.println(str_recv);
          int inx = str_recv.indexOf(':');
          if (inx > 3) {
            str_head = str_recv.substring(1, inx);
            str_val = str_recv.substring(inx + 1, str_recv.length() - 1);
            String str_cmd = "";
            if (str_head == "actions") {
              TelnetStream.print(str_head);
              TelnetStream.print(":");
              //TelnetStream.print(str_val);
              //TelnetStream.print(":");
              TelnetStream.println(route_process(str_val));

              if (route_buffer == "") 
              {
                route_buffer = route_process(str_val);
                route_buffer_index = 0;
              }
            }
            if (str_head == "isRunning") {
              //Serial.println(str_head);
              //Serial.println(str_val);
              if (str_val == "2") _isRunning = 2;
              if (str_val == "1") {
                _isRunning = 1;
                route_buffer = "";
                route_buffer_index = 0;
              }
              if (str_val == "0") {
                _isRunning = 0;
              }
            }
            if(str_head == "stateToCharge") 
            {
              if (str_val == "1") 
              {
                chargeState=1;
              }
              else
              {
                chargeState=0;  
              }
              if(chargeState != last_chargeState)
              {
                last_chargeState = chargeState;
                if(chargeState)
                {
                    _turnOnCharging();
                }
                else
                {
                    _turnOffCharging();
                }
              }
            }
          }
        }
      }
    }
  }
}

void httpRequest() {
  if (CONNECT_SERVER) {
    // close any connection before send a new request.
    // This will free the socket on the WiFi shield
    client.stop();
    // if there's a successful connection:
    if (client.connect(server, 8003)) {
      //Serial.println("");
      //Serial.println("connecting...");
      // send the HTTP GET request:
      if (_motion_block_sp >= 2 && _motion_block < _motion_block_sp) _isreadypath = 0;
      else _isreadypath = 1;
      String p = "robotname=" + String(robotname) + "&x=" + String(pos_x) + "&y=" + String(pos_y) + "&dir=" + String(pos_dir) + "&isreadypath=" + String(_isreadypath);
      TelnetStream.println(p);
      // client.println("GET /api/robots/controllers/fleet?" + p + "&btn1=0&btn2=0&btn3=0&battery=4000 HTTP/1.1");
      // client.println("GET /api/robots/controllers/fleet?" + p + "&btn1=0&btn2=0&btn3=0&battery=" + String(vbatt + 110) + " HTTP/1.1"); //robot1
      // client.println("GET /api/robots/controllers/fleet?" + p + "&btn1=0&btn2=0&btn3=0&battery=" + String(vbatt - 90) + " HTTP/1.1"); //robot2
      // client.println("GET /api/robots/controllers/fleet?" + p + "&btn1=0&btn2=0&btn3=0&battery=" + String(vbatt + 60) + " HTTP/1.1"); //robot3
      // client.println("GET /api/robots/controllers/fleet?" + p + "&btn1=0&btn2=0&btn3=0&battery=" + String(vbatt) + " HTTP/1.1"); //robot4
      client.println("GET /api/robots/controllers/fleet?" + p + "&btn1=0&btn2=0&btn3=0&battery=" + String(vbatt + offset_vbatt) + " HTTP/1.1");
      client.print("Host: ");
      client.println(server);
      client.println("Connection: close");
      client.println();

    } else {
      // if you couldn't make a connection:
      Serial.println("connection failed");
    }
  }
}

void httpRequestInit() {
  if (CONNECT_SERVER) {
    // close any connection before send a new request.
    // This will free the socket on the WiFi shield
    client.stop();
    // if there's a successful connection:
    if (client.connect(server, 8003)) 
    {

      String p = "robotname=" + String(robotname) ;
      TelnetStream.println(p);
      // client.println("GET /api/robots/controllers/fleet?" + p + "&btn1=0&btn2=0&btn3=0&battery=4000 HTTP/1.1");
      client.println("GET /api/robots/controllers/fleet/disconnect?"+ p + " HTTP/1.1");
      client.print("Host: ");
      client.println(server);
      client.println("Connection: close");
      client.println();
      init_position = 2;
    } 
    else 
    {
      // if you couldn't make a connection:
      Serial.println("connection failed");
    }
  }
}


String route_process(String data) 
{
  String cmd = "";
  for (int i = 0; i < data.length(); i++) 
  {
    if (i == 0) 
    {
      char _temp = route_dir_action(pos_dir, data[i]);
      if (_temp != NULL) cmd += _temp;
      if (data[i] != '0' && data[i] != '1' && data[i] != '2' && data[i] != '3' && data[i] != '-') cmd += data[i];
    } 
    else 
    {
      char _temp = route_dir_action(data[i - 1], data[i]);
      if (_temp != NULL) cmd += _temp;
      if (data[i] != '0' && data[i] != '1' && data[i] != '2' && data[i] != '3' && data[i] != '-') cmd += data[i];
    }
  }
  //Serial.println(cmd);
  //Serial.println(compress_block(cmd));
  return cmd;
}

char route_dir_action(char curr_dir, char next_block) {
  //Between Position
  if ((curr_dir == 'N' || curr_dir == 0) && next_block == 'W') return 'L';
  if ((curr_dir == 'N' || curr_dir == 0) && next_block == 'E') return 'R';
  if ((curr_dir == 'N' || curr_dir == 0) && next_block == 'S') return 'T';
  if ((curr_dir == 'E' || curr_dir == 1) && next_block == 'N') return 'L';
  if ((curr_dir == 'E' || curr_dir == 1) && next_block == 'S') return 'R';
  if ((curr_dir == 'E' || curr_dir == 1) && next_block == 'W') return 'T';
  if ((curr_dir == 'S' || curr_dir == 2) && next_block == 'E') return 'L';
  if ((curr_dir == 'S' || curr_dir == 2) && next_block == 'W') return 'R';
  if ((curr_dir == 'S' || curr_dir == 2) && next_block == 'N') return 'T';
  if ((curr_dir == 'W' || curr_dir == 3) && next_block == 'S') return 'L';
  if ((curr_dir == 'W' || curr_dir == 3) && next_block == 'N') return 'R';
  if ((curr_dir == 'W' || curr_dir == 3) && next_block == 'E') return 'T';
  //END Position Direction
  if ((curr_dir == 'N' || curr_dir == 0) && next_block == '3') return 'L';
  if ((curr_dir == 'N' || curr_dir == 0) && next_block == '1') return 'R';
  if ((curr_dir == 'N' || curr_dir == 0) && next_block == '2') return 'T';
  if ((curr_dir == 'E' || curr_dir == 1) && next_block == '0') return 'L';
  if ((curr_dir == 'E' || curr_dir == 1) && next_block == '2') return 'R';
  if ((curr_dir == 'E' || curr_dir == 1) && next_block == '3') return 'T';
  if ((curr_dir == 'S' || curr_dir == 2) && next_block == '1') return 'L';
  if ((curr_dir == 'S' || curr_dir == 2) && next_block == '3') return 'R';
  if ((curr_dir == 'S' || curr_dir == 2) && next_block == '0') return 'T';
  if ((curr_dir == 'W' || curr_dir == 3) && next_block == '2') return 'L';
  if ((curr_dir == 'W' || curr_dir == 3) && next_block == '0') return 'R';
  if ((curr_dir == 'W' || curr_dir == 3) && next_block == '1') return 'T';

  return NULL;
}


void read_Sensor(unsigned char n) {
  if (CONNECT_A1454) {
    switch (n) {
      case 0:
        if (abs(A1454_readMag(0x69)) > 60) sensor[0] = 1;  //intersection path
        else sensor[0] = 0;
        break;
      case 1:
        if (abs(A1454_readMag(0x67)) > 60) sensor[1] = 1;  //direct path
        else sensor[1] = 0;
        break;
      case 2:
        if (abs(A1454_readMag(0x66)) > 60) sensor[2] = 1;  //direct path
        else sensor[2] = 0;
        break;
      case 3:
        if (abs(A1454_readMag(0x64)) > 60) sensor[3] = 1;  //intersection path
        else sensor[3] = 0;
        break;
    }
  }
  // Serial.print(sensor[0]);
  // Serial.print(sensor[1]);
  // Serial.print(sensor[2]);
  // Serial.println(sensor[3]);
}
//Begins I2C Communication
void A1454_begin(unsigned char A1454_ADDRESS) {
  Wire.setClock(400000);
  Wire.begin();
  A1454_access(A1454_ADDRESS);
  A1454_wake(A1454_ADDRESS);
}
int16_t A1454_readMag(unsigned char A1454_ADDRESS) {
  uint8_t rawData[4] = { 0, 0, 0, 0 };

  Wire.beginTransmission(A1454_ADDRESS);
  Wire.write(A1454_OUTPUT);
  Wire.endTransmission();

  Wire.requestFrom(A1454_ADDRESS, 4);
  Wire.read();
  Wire.read();
  rawData[2] = Wire.read();
  rawData[3] = Wire.read();

  int16_t destination = 0;
  destination = (int16_t)(((int16_t)(rawData[2] & 0x0F) << 12) | rawData[3] << 4);
  destination = destination >> 4;

  return destination;
}
//Wake the sensor from sleep mode, wait for it to stabilize
void A1454_wake(unsigned char A1454_ADDRESS) {
  Wire.beginTransmission(A1454_ADDRESS);
  Wire.write(A1454_SLEEP);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
  //delay(1);
}
//Write ACCESS codes to write to allow write to registers?
void A1454_access(unsigned char A1454_ADDRESS) {
  Wire.beginTransmission(A1454_ADDRESS);
  Wire.write(A1454_ACCESS_ADDRESS);
  Wire.write(A1454_ACCESS_CODE);
  Wire.endTransmission();
}
