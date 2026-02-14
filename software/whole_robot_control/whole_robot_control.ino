#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

const char *ssid = "your_ssid";         // The router name 
const char *password = "your_password"; // The router password

//------------ ADHESION MECHANISM INIT --------------

Servo esc;                    // Initializing ECS object
int escPin = 33;              // and its connection pin
int max_speed = 1800;         // and PWM value in range 1000 - 2000

//------------ DRIVE INIT --------------

const int PWM_motor_R = 17;   // Right motor PWM pin
const int PWM_motor_L = 16;   // Left motor PWM pin

const int ENCODER_L = 25;     // Right motor encoder pin
const int ENCODER_R = 26;     // Left motor encoder pin

// Variables
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;
long prevEncL = 0;
long prevEncR = 0;

// Encoders interrupts
void IRAM_ATTR encoderLeftISR() {
  encoderLeftCount++;
}

void IRAM_ATTR encoderRightISR() {
  encoderRightCount++;
}


//------------ IMU INIT --------------

Adafruit_MPU6050 mpu;    // MPU6050 IMU object

// Trajectory params 
const float R = 0.07;  // Wheel radius [m] 
const float WHEEL_BASE = 0.29;  // Wheel track [m] (the distance between wheels)
const int ENCODER_PPR = 270;  // Encoder impulses per rotation (depend on the motor) 

float x = 0.0;      // X position [m]
float y = 0.0;      // Y position [m]
float theta = 0.0;  // orientation [rad]

unsigned long lastTime = 0;
float gyroBias = 0.0;


//------------ WiFi INIT --------------

NetworkServer server(80);     // ESP Server

// UDP
WiFiUDP udp;
const int udpPort = 8888;
const int udpSendPort = 8889;

bool isRecording = false;
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 100; // 10 Hz

IPAddress computerIP;
// Sensor events - make global for UDP access
sensors_event_t a, g, temp;

// Timing variables
unsigned long lastOdometryUpdate = 0;
unsigned long lastPrint = 0;

//----------- Gyroscope Calibration -------------

float calibrateGyro() {
  Serial.println("Calibrating gyro... Keep robot still!");
  float sum = 0;
  sensors_event_t a, g, t;

  for (int i = 0; i < 1000; i++) {
    mpu.getEvent(&a, &g, &t);
    sum += a.acceleration.z;
    delay(2);
  }
  
  float bias = sum / 1000.0;
  Serial.print("Gyro bias: ");
  Serial.print(bias, 6);
  Serial.println(" rad/s");
  return bias;
}


// ----------------------------- ESP SET UP ------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  //ESC setup
  esc.setPeriodHertz(50);
  esc.attach(escPin, 1000, 2000);
  esc.writeMicroseconds(1000);
  delay(2000);

  delay(1000);
  Serial.print("ESC OK!");
  // Inicjalizacja I2C
  Wire.begin();
  // MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 nie znaleziony!");
    while (1) delay(10);
  }
  Serial.println("MPU6050 OK");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  delay(100);
  gyroBias = calibrateGyro();



  // PWM Config
  ledcAttach(PWM_motor_R, 20000, 8);
  ledcAttach(PWM_motor_L, 20000, 8);
  ledcWrite(PWM_motor_R, 255);
  ledcWrite(PWM_motor_L, 255);
  
  // Encoders
  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(ENCODER_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), encoderRightISR, RISING);

  // WiFi
  Serial.print("Laczenie WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi OK!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("Serwer gotowy\n");
  
  // Start UDP
  udp.begin(udpPort);
  Serial.println("UDP uruchomiony na porcie " + String(udpPort));
  Serial.println("Czekam na komendy z Pythona...");

  lastTime = micros();
}


void updateOdometry() {

  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6;
  lastTime = now;

  // Encoders measurement
  long encL = encoderLeftCount;
  long encR = encoderRightCount;

  long dEncL = encL - prevEncL;
  long dEncR = encR - prevEncR;

  prevEncL = encL;
  prevEncR = encR;

  // Distance calculation
  float sL = dEncL * (2 * PI * R) / 270;
  float sR = dEncR * (2 * PI * R) / 270;
  float s  = (sL + sR) / 2.0;     

  // Angular veocity calculation
  mpu.getEvent(&a, &g, &temp);
  float gyroZ = (g.gyro.z - gyroBias);
  float omega = gyroZ;   // rad/s

  float thetaMid = theta + omega * dt * 0.5;
  theta += omega * dt;


  // Angle normalization [-π, π]
  while (theta > PI) theta -= 2.0 * PI;
  while (theta < -PI) theta += 2.0 * PI;

  x += s * cos(thetaMid);
  y += s * sin(thetaMid);
}

// Serial info about measurements
void printOdometry() {
  Serial.print("X: "); Serial.print(x, 4);
  Serial.print(" m | Y: "); Serial.print(y, 4);
  Serial.print(" m | Theta: "); Serial.print(theta * 180.0 / PI, 2);
  Serial.println(" deg");
}


void handleUDPMeasurements() {

  // ===== UDP commends  =====
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char buf[50];
    int len = udp.read(buf, 50);
    if (len > 0) buf[len] = 0;

    computerIP = udp.remoteIP();
    String cmd = String(buf);
    cmd.trim();

    if (cmd == "START") {
      encoderLeftCount = 0;
      encoderRightCount = 0;
      prevEncL = 0;
      prevEncR = 0;
      x = 0.0;
      y = 0.0;
      theta = 0.0;
      lastTime = micros();

      isRecording = true;
      Serial.println("UDP: START POMIAROW");

      udp.beginPacket(computerIP, udpSendPort);
      udp.print("STARTED");
      udp.endPacket();
    }

    else if (cmd == "STOP") {
      isRecording = false;
      Serial.println("UDP: STOP POMIAROW");

      udp.beginPacket(computerIP, udpSendPort);
      udp.print("STOPPED");
      udp.endPacket();
    }
  }

  // ===== Save to CSV =====
  if (isRecording && millis() - lastSendTime >= sendInterval) {
    lastSendTime = millis();

    updateOdometry();

    String csv =
      String(millis()) + "," +
      // String(a.acceleration.x, 4) + "," +
      // String(a.acceleration.y, 4) + "," +
      // String(a.acceleration.z, 4) + "," +
      String(x*3, 4) + "," +
      String(y*3, 4) + "," +
      String(theta, 6) + "," +
      String(encoderLeftCount) + "," +
      String(encoderRightCount);

    udp.beginPacket(computerIP, udpSendPort);
    udp.print(csv);
    udp.endPacket();
  }
}


// Main loop for handling TCP commands
void loop() {
    NetworkClient client = server.accept();

    if (client) {
      Serial.println("Klient polaczony");
      
      while (client.connected()) {
        if (client.available()) {
          String cmd = client.readStringUntil('\n');
          cmd.trim();
          Serial.println();

          if (cmd == "M") {
            for (int speed = 1000; speed <= max_speed; speed += 10) {     // Ramp propeller motor acceleration
              esc.writeMicroseconds(speed);
              delay(20);
            }
            delay(1000);
          }
          else if (cmd == "N") {
            for (int speed = max_speed; speed >= 1000; speed -= 10) {
              esc.writeMicroseconds(speed);
              delay(20);
            }
          }
          else if (cmd == "W") {
            ledcWrite(PWM_motor_R, 72);       // Each case needs setting 8-bit PWM value in range 0 (max speed) - 255 (zero speed)
            ledcWrite(PWM_motor_L, 64);
            Serial.println("FORWARD");
            
          }
          else if (cmd == "S" || cmd == "STOP") {
            ledcWrite(PWM_motor_R, 144);
            ledcWrite(PWM_motor_L, 128);
            Serial.println("STOP");
            
          }
          else if (cmd == "A") {
            ledcWrite(PWM_motor_R, 96);
            ledcWrite(PWM_motor_L, 144);
            Serial.println("LEFT");
            
          }
          else if (cmd == "D") {
            ledcWrite(PWM_motor_R, 144);
            ledcWrite(PWM_motor_L, 96);
            client.println("RIGHT");
          }
          else {
            Serial.println("NIEZNANA KOMENDA!");
          }
          client.flush();
        }

        handleUDPMeasurements();
      }
      
      client.stop();
      Serial.println("Klient rozlaczony\n");
    }
    
  handleUDPMeasurements();
}



