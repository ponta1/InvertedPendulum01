// MilkSegway 2022/12/23
// ジャイロ MPU6050版
//
// Arduino接続
//   ジャイロ: A4（SDA）・A5（SCL）
//   ロータリーエンコーダー: D2 (割り込み0)
//   XBee: D3(RX), D4(TX)
//   モータードライバー(MP4212) 2系統
//     M1:5,6,7,8  M2:9,10,11,12 (5,6,9,10はPWMが使えるポートとする)

#include <Wire.h>

//#define mySerial Serial
#ifndef mySerial
  #include <SoftwareSerial.h>
  SoftwareSerial mySerial(3, 4);    // RX,TX XBee通信(57600bps)
#endif

int MOTOR_PIN[][4] = {
  {5, 6, 7, 8},     // MOTOR1
  {9, 10, 11, 12}   // MOTOR2
};

float K_GYRO = 1.0;      // 角速度のゲイン
float K_DEG  = 0.11;     // 角度の誤差のゲイン
float K_DEG_SUM = 0.2;   // 角度の誤差の積算のゲイン
float K_SPEED = 2.5;     // 回転速度のゲイン
float K_ALL = 1.00;      // 全体のゲイン

#define PI 3.141592653589793

float gyro = 0;     // 角速度
float deg = 0;      // 角速度の積算（=角度）
float deg_sum = 0;  // 角度の積算

float score = 0;    // スコア 小さいほど良い (揺れが少ないと小さくなる)

#define STEP_TIME 1250  // μsec (800MHz)
//#define STEP_TIME 2500  // μsec (400MHz)

#define BASE_COUNT (4 * 1000000L / STEP_TIME)  // 基準を計測するループ回数 4秒
int base_count = 0;

float base_gyro = 0;  // ジャイロの基準
//float base_acc = 0;   // 加速度の基準

float v_gyro = 0;     // ジャイロ ふらつき
float v_acc = 0;      // 加速度 ふらつき

volatile float motor_power = 0;     // モーターのパワー

float re_speed = 0;   // 移動スピード

// シリアル通信 間引き用のカウンタ -> 遅いので頻度が多いと安定しなくなる
#define SERIAL_COUNT (2 * 1000000L / STEP_TIME)   // 2秒に1回
int serialCount= 0;

unsigned long pre_time = 0;

// スピード(ロータリーエンコーダー)
volatile int re_count = 0;  // 1周期のカウンタ

// ロータリーエンコーダーカウント割り込み処理関数
void RotaryEncoderCount() {
  if (motor_power < 0) {
    re_count--;
  } else {
    re_count++;
  }
}

// id: motor id(0 or 1)
// spd : +- 0 - 255
void ctrl_motor(int id, int spd) 
{
  if(spd > 255)  spd = 255;
  if(spd < -255) spd = -255;

  if (spd == 0) {
    digitalWrite(MOTOR_PIN[id][0], 0); // ON
    digitalWrite(MOTOR_PIN[id][1], 0); // ON
    digitalWrite(MOTOR_PIN[id][2], 0); // OFF
    digitalWrite(MOTOR_PIN[id][3], 0); // OFF
  }
  else if (spd > 0) {
    analogWrite(MOTOR_PIN[id][0], 255 - spd); // ON(PWM)
    digitalWrite(MOTOR_PIN[id][1], 1); // OFF
    digitalWrite(MOTOR_PIN[id][2], 0); // OFF
    digitalWrite(MOTOR_PIN[id][3], 1); // ON
  }
  else {
    spd = -spd;
    digitalWrite(MOTOR_PIN[id][0], 1); // OFF
    analogWrite(MOTOR_PIN[id][1], 255 - spd); // ON(PWM)
    digitalWrite(MOTOR_PIN[id][2], 1); // ON
    digitalWrite(MOTOR_PIN[id][3], 0); // OFF
  }
}

void init_motor(int id) 
{
  pinMode(MOTOR_PIN[id][0], OUTPUT);
  pinMode(MOTOR_PIN[id][1], OUTPUT);
  pinMode(MOTOR_PIN[id][2], OUTPUT);
  pinMode(MOTOR_PIN[id][3], OUTPUT);
}

float get_serial_float() 
{
  float r;
  int sign = mySerial.read();
  r  = (mySerial.read() - '0') * 10.0;
  r += (mySerial.read() - '0');
  r += (mySerial.read() - '0') / 10.0;
  r += (mySerial.read() - '0') / 100.0;
  if (sign == '-') r = -r;
  mySerial.read();  // スペース

  return r;
}

void setup() 
{
  // モーター初期化
  init_motor(0);
  init_motor(1);
  ctrl_motor(0, 0);
  ctrl_motor(1, 0);

  mySerial.begin(57600);
  mySerial.println("MilkSegway setup start");
    
  // LED
  pinMode(13, OUTPUT);

  // ロータリーエンコーダー
  pinMode(2, INPUT);

  // MPU6050初期化  
  Wire.begin();
  Wire.setClock(400000L);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Pin2が変化したときに割り込み
  attachInterrupt(0, RotaryEncoderCount, CHANGE);

  delay(5000);  // XBeeの接続待ち
  mySerial.println("MilkSegway setup complete");
}

int f1 = 0;
int f2 = 0;

void loop()
{
  // 時間待ちループ
  int wait_count = 0;
  unsigned long time = micros();
  while (time - pre_time <= STEP_TIME) {
    time = micros();
    wait_count++;
  }
  pre_time = time;

  // MPU6050から取得
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  while (Wire.available() < 14);
  
  int16_t axRaw = Wire.read() << 8 | Wire.read();
  int16_t ayRaw = Wire.read() << 8 | Wire.read();
  int16_t azRaw = Wire.read() << 8 | Wire.read();
  int16_t temperature = Wire.read() << 8 | Wire.read();
  int16_t gxRaw = Wire.read() << 8 | Wire.read();
  int16_t gyRaw = Wire.read() << 8 | Wire.read();
  int16_t gzRaw = Wire.read() << 8 | Wire.read();

  // 加速度センサーから算出した角度
  float acc_deg = -(atan2(azRaw, ayRaw) / PI * 180.0 + 12.80) * (1000000L / STEP_TIME); // 12.80 : もともと少し傾いている

  // 直立している時の基準値が収束するまで待つ
  if (base_count < BASE_COUNT) {
    // ジャイロの基準の更新
    base_gyro += gxRaw;
    //base_acc += azRaw;
    base_count++;
    return;
  }
  else if (base_count == BASE_COUNT) {
    base_gyro /= base_count;
    //base_acc /= base_count;
    deg = acc_deg; // 傾きの初期値
    base_count++;
    mySerial.println("MilkSegway Start");
  }

  // ジャイロ(角速度)
  gyro = (gxRaw - base_gyro) / 131.0; // (度/s)に変換

  // 加速度
  //float raw_acc = azRaw - base_acc;

  v_gyro = v_gyro * 0.95 + ((gyro < 0) ? -gyro: gyro) * 100 * 0.05;   // ジャイロ ふらつき
  //v_acc = v_acc * 0.95 + ((raw_acc < 0) ? -raw_acc: raw_acc) * 0.05;  // 加速度 ふらつき
  //v_acc = v_acc * 0.95 + ((acc_deg < 0) ? -acc_deg: acc_deg) * 0.05;  // 加速度 ふらつき

  deg = deg + gyro; // 角速度の積算=角度
  deg_sum = deg_sum + deg; // 角度の積算

  // ふらついていない -> 角度と速度を補正
  if (v_gyro < 30) {
    deg = deg * 0.99 + acc_deg * 0.01;
    deg_sum = deg_sum * 0.95;
    re_speed = re_speed * 0.95;
    f1++;
  }
  else if (v_gyro < 60) {
    deg = deg * 0.995 + acc_deg * 0.005;
    deg_sum = deg_sum * 0.97;
    re_speed = re_speed * 0.97;
    f2++;
  }

  // ロータリーエンコーダー
  re_speed = re_speed + re_count;     // 速度
  re_count = 0;

  // モーターへの出力を計算(PID制御)
  float power = ((K_GYRO * gyro) +        // D
                 (K_DEG * deg) +          // P
                 (K_DEG_SUM * 0.001 * deg_sum) +  // I
                 (K_SPEED * re_speed)
                 ) * K_ALL;

  motor_power = motor_power * 0.5 + power * 0.5;  // 移動平均

  // 評価 : この値が小さくなるようにパラメータを自動調整出来ないか?
  score = score * 0.999 + ((gyro > 0)? gyro: -gyro) * 0.001;  // 移動平均

  // モータースピードの計算
  int spd = (int)motor_power;

  // 破たんした（倒れた）場合は止める
  if(spd > 1000 || spd < -1000)  spd = 0;

  // 使用したモーターの特性で20以下は動かないので補正する
  if (spd > 0) spd += 20;
  if (spd < 0) spd -= 20;

  // モータの最大速度を超えないように制限
  if(spd > 255)  spd = 255;
  if(spd < -255) spd = -255;

  ctrl_motor(0, spd);
  ctrl_motor(1, spd);

  serialCount++;
  if (serialCount >= SERIAL_COUNT) {  
    // 表示用
    int disp_gyro = (int)(K_GYRO * gyro * K_ALL);
    int disp_deg = (int)(K_DEG * deg * K_ALL);
    int disp_deg_sum = (int)(K_DEG_SUM * deg_sum * 0.001 * K_ALL);
    int disp_re_speed = (int)(K_SPEED * re_speed * K_ALL);
    int disp_spd = motor_power;
  
    char buf_gyro[10];
    dtostrf(gyro, 7, 3, buf_gyro);
    char buf_deg[10];
    dtostrf(deg, 6, 1, buf_deg);
    char buf_deg_sum[10];
    dtostrf(deg_sum * 0.001, 5, 0, buf_deg_sum);
    char buf_re_speed[10];
    dtostrf(re_speed, 4, 0, buf_re_speed);

    char buf[128];
    sprintf(buf, "S:%3d G:%s/%3d DEG:%s/%3d SUM:%s/%3d RE:%s/%3d C:%2d VG:%3d A:%5d f:%3d/%3d SPD:%4d", 
      (int)(score * 100), buf_gyro, disp_gyro, buf_deg, disp_deg, buf_deg_sum, disp_deg_sum, buf_re_speed, disp_re_speed, 
      wait_count, (int)v_gyro, (int)(acc_deg), f1, f2, disp_spd);
    mySerial.println(buf);  // 送信完了まで待つので遅い
    
    f1 = f2 = 0;
    serialCount = 0;
  }

  // LED点滅
  if (serialCount == 0)  
    digitalWrite(13, HIGH);
  if (serialCount == SERIAL_COUNT / 2)  
    digitalWrite(13, LOW);

  // シリアル受信
  if (mySerial.available() >= 50) {
    int command = mySerial.read();
    if (command == 'P') {
      mySerial.println("Param command received");
      float dummy  = get_serial_float();
      K_GYRO       = get_serial_float();
      K_DEG        = get_serial_float();
      K_DEG_SUM    = get_serial_float();
      dummy        = get_serial_float();
      K_SPEED      = get_serial_float();
      dummy        = get_serial_float();
      K_ALL        = get_serial_float();

      mySerial.read();  // 'E'
    }
    else if (command == 'R') {
      mySerial.println("Reset command received");
      // 'E'が来るまでループ
      int r = command;
      while (r != 'E') {
        r = mySerial.read();
      }
      deg = 0;
      deg_sum = 0;
      re_speed = 0;
    }
    else {
      mySerial.println("Command receive error");
      // 'E'が来るまでループ
      char buf[100];
      int i = 0;
      int r = command;
      buf[i++] = r;
      while (r != 'E') {
        r = mySerial.read();
        buf[i++] = r;
      }
      buf[i++] = 0;
      mySerial.println(buf);
    }
  }
}

