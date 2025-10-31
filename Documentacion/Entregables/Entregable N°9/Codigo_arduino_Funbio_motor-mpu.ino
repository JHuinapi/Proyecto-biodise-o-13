#include <Wire.h>
#include <MPU9250.h>

// ------------------ IMU ------------------
MPU9250 mpu;                 // Librería tipo hideakitai
const uint8_t MPU_ADDR = 0x68; // 0x69 si AD0=HIGH
const unsigned long IMU_PERIOD_MS = 100; // cada 100 ms
unsigned long tIMU = 0;

// ------------------ RF (CH4 D0–D3) ------------------
// Usamos SOLO D0 y D1 como botones en pines 2 y 3
const byte BTN_FWD = 2;  // D0 del receptor
const byte BTN_REV = 3;  // D1 del receptor
const bool ACTIVE_HIGH = true;   
const unsigned long DEBOUNCE_MS = 50;
bool lastFwd = false, lastRev = false;
unsigned long tDebounce = 0;

// ------------------ L298N ------------------
const byte ENA = 11 ;  // PWM canal A
const byte IN1 = 10;
const byte IN2 = 9;
const byte ENB = 6;   // PWM canal B
const byte IN3 = 4;
const byte IN4 = 5;

const byte SPEED_PWM = 180;
const unsigned long RUN_MS = 2000; 

bool running = false;
int dir = 0; // 1 = adelante, -1 = atrás, 0 = parado
unsigned long tEnd = 0;

// ------------------ Helpers ------------------
inline bool btnRead(byte pin) {
  bool v = digitalRead(pin);
  return ACTIVE_HIGH ? v : !v;
}

void startRun(int d) {
  dir = d;
  if (dir == 1) {
    // Adelante: ambos motores mismo sentido
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    // Atrás
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  }
  analogWrite(ENA, SPEED_PWM);
  analogWrite(ENB, SPEED_PWM);
  running = true;
  tEnd = millis() + RUN_MS;
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  // Freno activo (ambos LOW)
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  running = false;
  dir = 0;
}

// ------------------ Setup ------------------
void setup() {
  Serial.begin(115200);

  // RF botones
  if (ACTIVE_HIGH) {
    pinMode(BTN_FWD, INPUT);
    pinMode(BTN_REV, INPUT);
  } else {
    pinMode(BTN_FWD, INPUT_PULLUP);
    pinMode(BTN_REV, INPUT_PULLUP);
  }

  // L298N
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  // IMU
  Wire.begin();
  delay(50);
  if (!mpu.setup(MPU_ADDR)) {
    Serial.println(F("Error al iniciar MPU9250 (revisa direccion 0x68/0x69 y cableado)"));
  } else {
    Serial.println(F("MPU9250 listo"));
  }

  Serial.println(F("Sistema listo."));
}

// ------------------ Loop ------------------
void loop() {
  // --------- Lectura de botones y control no bloqueante ----------
  bool fwd = btnRead(BTN_FWD);
  bool rev = btnRead(BTN_REV);

  bool riseFwd = (fwd && !lastFwd);
  bool riseRev = (rev && !lastRev);

  unsigned long now = millis();

  if (!running && (now - tDebounce > DEBOUNCE_MS)) {
    if (riseFwd && !rev) {
      startRun(+1);
      tDebounce = now;
    } else if (riseRev && !fwd) {
      startRun(-1);
      tDebounce = now;
    }
  }

  if (running && now >= tEnd) {
    stopMotors();
  }

  lastFwd = fwd; lastRev = rev;

  // --------- IMU: leer e imprimir cada IMU_PERIOD_MS ----------
  if (now - tIMU >= IMU_PERIOD_MS) {
    tIMU = now;
    if (mpu.update()) {
      float ax = mpu.getAccX(); // en g
      float ay = mpu.getAccY();
      float az = mpu.getAccZ();

      // Estimación simple de ángulos con solo acelerómetro:
      float roll  = atan2(ay, az) * 180.0 / PI;                      // rotación alrededor de X
      float pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;    // rotación alrededor de Y

      Serial.print(F("acc[g]: "));
      Serial.print(ax, 3); Serial.print(' ');
      Serial.print(ay, 3); Serial.print(' ');
      Serial.print(az, 3);
      Serial.print(F(" | roll: "));
      Serial.print(roll, 1);
      Serial.print(F(" deg | pitch: "));
      Serial.println(pitch, 1);
    }
  }
}