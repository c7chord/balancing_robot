//Todo

// 라이브러리
#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"
#include "I2Cdev.h"

// define
#define RX_PIN 2
#define TX_PIN 4
#define ON 1
#define OFF 0
// 변수

// 로봇 상태 관련
int dt = 0;
int state = OFF;
int state_count = 0; // on/off 변수

// 제어 관련
float err = 0;
float pre_err = 0;

float pos_p = 0;
float pos_i = 0;
float pos_d = 0;

float pos_desired = 0;
float pos_kp = 0.2; // 0.2;
float pos_ki = 0.0025; // 0.0025;
float pos_kd = 3.2; // 3.2; 
 ///Kp = 0.2; // 0.14;//0.6*Ku; 
 //   Ki = 0.035; // 0.03; //1.2*Ku/Tu; // 적분항 (가속기)
   // igain = 1000;// 4000; // 700; 그냥, ki, i gain이 최대 속도(30)이가 넘게끔
   // Kd = 5; // 5; //0.075*Ku*Tu; // 미분항 (감소기)
   // K = 1; // 1;

float vel_p = 0;
float vel_i = 0;
float vel_d = 0;

float vel_desired = 0;
float vel_measured_ = 0;
float vel_kp = 0; // -0.15;// -1;//0.1;// 0.1;
float vel_ki = -0.001; // -0.005;// 0.0005; // 0.0001;
float vel_kd = 0; // -0.1;

// 그나마 되는 거 : -0.15, 0, 0


float K = 1;
float K_Ang = 1;
float LinOutput = 0;
float AngInput = 0;
float AngOutput = 0;
float LeftOutput=0;
float RightOutput=0;

// 로봇 설정값 관련
float range = 90; // 앞뒤 허용 범위
float pitch;

// 사용자 input 관련
float K_ds4 = 0.5;
float accel = 0;
float brake = 0;

// dt 함수 관련
unsigned long t, p_t;
unsigned long previousMillis = 0;

// 모터 관련
static const uint8_t MOTOR_HW_ID_1 = 0x01; // left wheel
static const uint8_t MOTOR_HW_ID_2 = 0x7f; // right wheel
static const uint8_t MASTER_CAN_ID = 0x00; // ESP32
bool driver_installed = false;

// mpu5060 관련
sensors_event_t a, g, temp;

// class 
ControllerPtr myControllers[BP32_MAX_GAMEPADS];  // 듀얼쇼크
XiaomiCyberGearDriver motor1(MOTOR_HW_ID_1, MASTER_CAN_ID);  // 모터1
XiaomiCyberGearDriver motor2(MOTOR_HW_ID_2, MASTER_CAN_ID);  // 모터2
Adafruit_MPU6050 mpu; // mpu5060

/////////////// 다른 함수 ///////////////////

// bluepad32
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}
void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}
void dumpGamepad(ControllerPtr ctl) {
  // 프린트 단
  // Serial.printf(
  //     "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
  //     "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
  //     ctl->index(),        // Controller Index
  //     ctl->dpad(),         // 왼쪽 4개 : NULL:0x00 up:0x01, down:0x02, left:0x08, up:0x04
  //     ctl->buttons(),      // 오른쪽 4개 : NULL:0x0000 세모:0x0008 네모:0x0004 동그라미:0x0002 엑스: 0x0001 (진동 상태에 대한 탐구 필요. (0x004?)) 그리고... 막 R2 누그로 하면 변함

  //     ctl->axisX(),        // (-511 - 512) left X Axis
  //     ctl->axisY(),        // (-511 - 512) left Y axis
  //     // 왼쪽 조이스틱. 기기 자체의 캘리브레이션 오류 있음, 중앙값에 대한 Hysterisis 존재.
  //     // 흰색 : x축 IDLE -> -24~-16 / y축 IDLE -> -16 ~ 4 / 좌측 최대 -> -508 / 우측 최대 -> 512 / 위 최대 -> -508 / 아래 최대 : 512 /
  //     // 빨간색 :
  //     ctl->axisRX(),       // (-511 - 512) right X axis
  //     ctl->axisRY(),       // (-511 - 512) right Y axis
  //     // 오른쪽 조이스틱. 기기 자체의 캘리브레이션 오류 있음, 중앙값에 대한 Hysterisis 존재.
  //     // 흰색 : x축 IDLE -> -8~--4 / y축 IDLE -> 0 ~ 8 / 좌측 최대 -> -508 / 우측 최대 -> 512 / 위 최대 -> -508 / 아래 최대 : 512 /
  //     // 빨간색 :
  //     ctl->brake(),        // L2버튼. 빨간색 : (0~400 - 1020): brake button
  //     ctl->throttle(),     // R2버튼. (0-1020)
  //     ctl->miscButtons(),  // NULL : 0x00 / share : 0x02 / options : 0x04

  //     ctl->gyroX(),        // Gyro X
  //     ctl->gyroY(),        // Gyro Y
  //     ctl->gyroZ(),        // Gyro Z
  //     ctl->accelX(),       // Accelerometer X
  //     ctl->accelY(),       // Accelerometer Y
  // );
}
void dumpMouse(ControllerPtr ctl) {
  Serial.printf("idx=%d, buttons: 0x%04x, scrollWheel=0x%04x, delta X: %4d, delta Y: %4d\n",
                ctl->index(),        // Controller Index
                ctl->buttons(),      // bitmask of pressed buttons
                ctl->scrollWheel(),  // Scroll Wheel
                ctl->deltaX(),       // (-511 - 512) left X Axis
                ctl->deltaY()        // (-511 - 512) left Y axis
  );
}
void dumpKeyboard(ControllerPtr ctl) {
  static const char* key_names[] = {
    // clang-format off
        // To avoid having too much noise in this file, only a few keys are mapped to strings.
        // Starts with "A", which is offset 4.
        "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V",
        "W", "X", "Y", "Z", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",
        // Special keys
        "Enter", "Escape", "Backspace", "Tab", "Spacebar", "Underscore", "Equal", "OpenBracket", "CloseBracket",
        "Backslash", "Tilde", "SemiColon", "Quote", "GraveAccent", "Comma", "Dot", "Slash", "CapsLock",
        // Function keys
        "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12",
        // Cursors and others
        "PrintScreen", "ScrollLock", "Pause", "Insert", "Home", "PageUp", "Delete", "End", "PageDown",
        "RightArrow", "LeftArrow", "DownArrow", "UpArrow",
    // clang-format on
  };
  static const char* modifier_names[] = {
    // clang-format off
        // From 0xe0 to 0xe7
        "Left Control", "Left Shift", "Left Alt", "Left Meta",
        "Right Control", "Right Shift", "Right Alt", "Right Meta",
    // clang-format on
  };
  Serial.printf("idx=%d, Pressed keys: ", ctl->index());
  for (int key = Keyboard_A; key <= Keyboard_UpArrow; key++) {
    if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
      const char* keyName = key_names[key - 4];
      Serial.printf("%s,", keyName);
    }
  }
  for (int key = Keyboard_LeftControl; key <= Keyboard_RightMeta; key++) {
    if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
      const char* keyName = modifier_names[key - 0xe0];
      Serial.printf("%s,", keyName);
    }
  }
  Console.printf("\n");
}
void dumpBalanceBoard(ControllerPtr ctl) {
  Serial.printf("idx=%d,  TL=%u, TR=%u, BL=%u, BR=%u, temperature=%d\n",
                ctl->index(),        // Controller Index
                ctl->topLeft(),      // top-left scale
                ctl->topRight(),     // top-right scale
                ctl->bottomLeft(),   // bottom-left scale
                ctl->bottomRight(),  // bottom-right scale
                ctl->temperature()   // temperature: used to adjust the scale value's precision
  );
}
void processGamepad(ControllerPtr ctl) {
  // There are different ways to query whether a button is pressed.
  // By query (정보 요청) each button individually:
  //  a(), b(), x(), y(), l1(), etc...
  if (ctl->a()) {
    static int colorIdx = 0;
    // Some gamepads like DS4 and DualSense support changing the color LED.
    // It is possible to change it by calling:
    switch (colorIdx % 3) {
      case 0:
        // Red
        ctl->setColorLED(255, 0, 0);
        break;
      case 1:
        // Green
        ctl->setColorLED(0, 255, 0);
        break;
      case 2:
        // Blue
        ctl->setColorLED(0, 0, 255);
        break;
    }
    colorIdx++;
  }

  if (ctl->b()) {
    // Turn on the 4 LED. Each bit represents one LED.
    static int led = 0;
    led++;
    // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
    // support changing the "Player LEDs": those 4 LEDs that usually indicate
    // the "gamepad seat".
    // It is possible to change them by calling:
    ctl->setPlayerLEDs(led & 0x0f);
  }

  if (ctl->x()) {
    // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S, Stadia support rumble.
    // It is possible to set it by calling:
    // Some controllers have two motors: "strong motor", "weak motor".
    // It is possible to control them independently.
    ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
                        0x40 /* strongMagnitude */);
  }

  // Another way to query controller data is by getting the buttons() function.
  // See how the different "dump*" functions dump the Controller info.
  dumpGamepad(ctl);  // status serial monitor print function
}
void processMouse(ControllerPtr ctl) {
  // This is just an example.
  if (ctl->scrollWheel() > 0) {
    // Do Something
  } else if (ctl->scrollWheel() < 0) {
    // Do something else
  }

  // See "dumpMouse" for possible things to query.
  dumpMouse(ctl);
}
void processKeyboard(ControllerPtr ctl) {
  if (!ctl->isAnyKeyPressed())
    return;

  // This is just an example.
  if (ctl->isKeyPressed(Keyboard_A)) {
    // Do Something
    Serial.println("Key 'A' pressed");
  }

  // Don't do "else" here.
  // Multiple keys can be pressed at the same time.
  if (ctl->isKeyPressed(Keyboard_LeftShift)) {
    // Do something else
    Serial.println("Key 'LEFT SHIFT' pressed");
  }

  // Don't do "else" here.
  // Multiple keys can be pressed at the same time.
  if (ctl->isKeyPressed(Keyboard_LeftArrow)) {
    // Do something else
    Serial.println("Key 'Left Arrow' pressed");
  }

  // See "dumpKeyboard" for possible things to query.
  dumpKeyboard(ctl);
}
void processBalanceBoard(ControllerPtr ctl) {
  // This is just an example.
  if (ctl->topLeft() > 10000) {
    // Do Something
  }

  // See "dumpBalanceBoard" for possible things to query.
  dumpBalanceBoard(ctl);
}
void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else if (myController->isMouse()) {
        processMouse(myController);
      } else if (myController->isKeyboard()) {
        processKeyboard(myController);
      } else if (myController->isBalanceBoard()) {
        processBalanceBoard(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// setup 관련
void setup_dualshock4() {
  Serial.printf("[SETUP] ESP32 BluePad Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("[SETUP] Bluetooth Address : %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
  Serial.println("[SETUP]Connect your Dualshock4");
  //while((myControllers[0] == nullptr)){ } // input이 들어올때까지 멈추는 함수
  //Serial.println("[SETUP]Dualshock4 Connected");
}
void setup_xiaomigear() {
  // Initialize CAN bus (TWAI) using the first motor object
  motor1.init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true);
  delay(1000);

  // --- Motor #1 (left) setup ---
  //motor1.init_motor(MODE_POSITION);
  motor1.init_motor(MODE_SPEED);
  motor1.set_limit_speed(V_MAX);
  motor1.set_limit_current(5.0f);
  motor1.set_speed_ref(0.0f);
  //motor1.set_speed_ki(motor_Ki);
  //motor1.set_speed_kp(motor_Kp);

  //motor2.init_twai(RX_PIN, TX_PIN, /*serial_debug=*/false);
  //delay(1000);
  //motor2.init_motor(MODE_POSITION);
  motor2.init_motor(MODE_SPEED);
  motor2.set_limit_speed(V_MAX);
  motor2.set_limit_current(5.0f);
  motor2.set_speed_ref(0.0f);
  //motor2.set_speed_ki(motor_Ki);
  //motor2.set_speed_kp(motor_Kp);

  driver_installed = true;
  delay(1000);
  Serial.println("[SETUP] Xiaomi CyberGear Setup Successful");
}
void setup_MPU6050() {
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("[SETUP] [!] MPU6050 Connection Fail. Please Restart");
    while (!mpu.begin())
    {
      break;
    }
  }
  Serial.println("[SETUP] MPU6050 Connection Successful");

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("[SETUP] MPU6050 Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  delay(1000); // 센서 안정화를 위한 1000초.
}

// print 관련
void print_angle() {
  // Serial.print("pitching_before : ");
  //Serial.print(prepitch);
  Serial.print(" pitch : ");
  Serial.print(pitch);
}
void print_dt() {
  Serial.print(" dt = ");
  Serial.print(dt);
}
void print_state() {
  if (state==OFF) {
    Serial.print("Robot State : ");
    Serial.print(state);
    Serial.println("please press L2 & R2");
  }
  
  if (state==ON) {
    // Serial.print("Robot State : ");
    // Serial.print(state);
    Serial.print("[LinInput, AngInput] : [");
    Serial.print(vel_desired);
    Serial.print(", ");
    Serial.print(AngInput);
    Serial.print("]");
  }
}
void print_motor() {
  Serial.print("linear term : ");
  //Serial.print(kp*err);
  Serial.print(", integral term : ");
  //Serial.print(ki*err*dt);
  Serial.print(", differential term : ");
  //Serial.print(Kd*(err-pre_err)/dt);
  Serial.print("[LinOutput, AngOutput] : [");
  Serial.print(LinOutput);
  Serial.print(", ");
  Serial.print(AngOutput);
  Serial.println("]");
}

// 오류 체크
bool problem() {
  bool result = false;
  
  // 샤오미 기어
  if (!driver_installed) {
    Serial.print("Driver not installed yet...");
    delay(1000);
    result = true;
  }
  return result;
}

// get 함수
void get_keyboard() {
  
  // Serial.println("[Speed menu]type [leftSpeed],[rightSpeed]");
  if(Serial.available()){
    String input = Serial.readStringUntil('\n');
    if(input == "on")
    {
      motor1.enable_motor();
      motor2.enable_motor();
    }
    else if(input == "off")
    {
      motor1.stop_motor();
      motor2.stop_motor();      
    }
    else {
      input.trim();
      // Look for the comma that separates [leftSpeed],[rightSpeed]
      int commaPos = input.indexOf(',');
      vel_desired = input.substring(0, commaPos).toFloat();
      AngInput = input.substring(commaPos + 1).toFloat();
    }
   } // input이 들어올때까지 멈추는 함수 
}

void get_ds4() {
  if (myControllers[0] == nullptr) {
    return;
  }  
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();
  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  // vTaskDelay(1);
  delay(1);

  accel = V_MAX*((float)myControllers[0]->throttle())/1020;
  brake = V_MAX*(float)myControllers[0]->brake()/1020;
  vel_desired = K_ds4 * (brake - accel);
  AngInput = K_ds4 * V_MAX*(float)myControllers[0]->axisRX()/1020;
}

// loop 선택 함수
void loop_keyboard() {
  
  // 에러 체크
  if(problem()) {
     return;
   }

  // 시동
  if(state == OFF) {
    state = ON;
    delay(100);
    motor1.enable_motor();
    motor2.enable_motor();
    pos_i = 0;
    vel_i = 0;
  }
  
  // 사용자 입력
  get_keyboard(); 
  
  // 제어
  control();  

  // print_state(); // 로봇 상태 보여줌 (input)
  // print_angle(); // angle print  
  // print_motor(); // 모터 상태 보여줌 (output)
}

void loop_ds4() {
  
  // problem 없으면 ㄲ 
  if(problem()) {
    return;
  }
  
  // OFF control
  get_ds4();
  print_state(); // 로봇 상태 보여줌
  while(on())
  {   
    get_ds4(); // 병진, 회전 input을 듀얼쇼크로 맥이기

    control(); // ref를 기반으로 모터 속도 주기 
    
    // print_state(); // 로봇 상태 보여줌 (input)
    // print_angle(); // angle print
    // print_motor(); // 모터 상태 보여줌 (output)
  }
}

// 스위치 및 기타 함수
void turn_on() {

  if (myControllers[0] == nullptr) {
    return;
  }
  if (myControllers[0]->brake() == 1020 && myControllers[0]->throttle() == 1020) {
    state_count++;
  }
  if ((state_count >= 10) && (myControllers[0]->brake() <= 1000 && myControllers[0]->throttle() <= 1000)) {
    {
      delay(100);
      motor1.enable_motor();
      motor2.enable_motor();
      state_count = 0;
      state = ON;
    }
  }
}

void turn_off() {
  if (myControllers[0] == nullptr) {
    return;
  }
  if (myControllers[0]->brake() == 1020 && myControllers[0]->throttle() == 1020) {
    state_count++;
  }
  if ((state_count >= 10) && (myControllers[0]->brake() <= 1000 && myControllers[0]->throttle() == 0)) {
    delay(100);  // bluetooth와 CAN통신이 겹쳐서 stop_moter가 안 먹힐 거라고 생각해서 넣은 딜레이인데, 실제로 그게 먹힘.
    motor1.stop_motor();
    motor2.stop_motor();
    state_count = 0;
    state = OFF;
  }
}

int on() {
 if (state == OFF) {
    turn_on();
  }
  if (state == ON) {
    turn_off(); 
  }
  return state;
} 

float get_dt() {
  unsigned long currentMillis = millis();
  unsigned long dt_ = currentMillis - previousMillis;
  previousMillis = currentMillis;
  return dt_;
}

// 제어 관련
float pos_measured() {
  mpu.getEvent(&a, &g, &temp);
  delay(1);
  pitch = (atan2(a.acceleration.y, a.acceleration.z) * 180 / PI);
  
  // print
  Serial.print("pos_measured = ");
  Serial.print(pitch);
  Serial.print(" || ");
  return pitch;
}

float vel_measured() {
  motor1.request_status();
    twai_message_t message1;
    while (twai_receive(&message1, pdMS_TO_TICKS(10)) == ESP_OK) {
      motor1.process_message(message1);
    }  
  float result = motor1.get_status().speed;
  Serial.print("vel_measured = ");
  Serial.print(result);
  Serial.print(" || ");
  return result;
}

float PID(float *desired, float measured, float kp, float ki, float kd, float i_max, float *p, float *i, float *d) {
  
  pre_err = err;
  err = *desired - measured;

  *p = kp*err; 
  *i += ki*err*dt;
  if (*i > i_max) {
    *i = i_max;
  }
  if (*i < -i_max) {
    *i = - i_max;
  }
  *d = kd*(err - pre_err)/dt;

  return *p + *i + *d;
}

void control() {
  // 각도 정지
  if (!(pitch > pos_desired - range && pitch < pos_desired + range))
  {
    motor1.set_speed_ref(0);
    motor2.set_speed_ref(0);
    return;
  }

  // calculate motor speed with pid 
  dt = get_dt();
  Serial.print("vel_desired = ");
  Serial.print(vel_desired);
  Serial.print(" || ");
  pos_desired = PID(&vel_desired,vel_measured(),vel_kp, vel_ki, vel_kd, range,&vel_p,&vel_i,&vel_d);
  Serial.print("pos_desired = ");
  Serial.print(pos_desired);
  Serial.print(" || ");
  LinOutput = PID(&pos_desired,pos_measured(),pos_kp, pos_ki, pos_kd, 0.5*V_MAX, &pos_p, &pos_i, &pos_d); // 0.5V_MAX보다 밀었을 때 pos_kd가 기능하는 일은 없음. 
  
  Serial.print("[dt, p, i, d, result] = ");
  Serial.print("[");
  Serial.print(dt);
  Serial.print(", ");
  Serial.print(pos_p);
  Serial.print(", ");
  Serial.print(pos_i);
  Serial.print(", ");
  Serial.print(pos_d);
  Serial.print(", ");
  Serial.print(LinOutput);
  Serial.println("]");

  AngOutput = K_Ang * AngInput;
  LeftOutput = K*(LinOutput + AngOutput);
  RightOutput = K*(-LinOutput + AngOutput);

  // constrain motor speed
  LeftOutput = constrain(LeftOutput, V_MIN, V_MAX);
  RightOutput = constrain(RightOutput, V_MIN, V_MAX);
  
  // send motor speed
  motor1.set_speed_ref(LeftOutput);
  motor2.set_speed_ref(RightOutput);

}


//////////////MAIN 함수//////////////////

void setup() {
  delay(100);  // Fuck slow ESP32 which uses CH340 chipset.
  setup_xiaomigear(); // 사이버기어 부팅
  setup_MPU6050(); // IMU 부팅
  setup_dualshock4(); // 외부 컨트롤러 부팅
}

void loop() {
  // loop_ds4();
  loop_keyboard();

}
