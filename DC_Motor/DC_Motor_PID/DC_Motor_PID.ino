// Pins
#define ENCA 3 // Green
#define ENCB 2 //Yellow
#define PWM 4
#define IN1 5
#define IN2 6

// Globals
long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;
float vtFiltered = 0; // Referência via serial

float eintegral = 0;
float eprev = 0;
int pwr = 0;
float e = 0; // Sinal de erro na realimentação

//Ajuste os ganhos
float kp = 0.32;
float ki = 0.8;
float kd = 0.2;

float e_integral_max = 600; // Limite para o erro integral

void setup() {
  Serial.begin(115200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
  // Verificar dados recebidos pela serial
  if(Serial.available() > 0){
    String input = Serial.readStringUntil('\n');
    vtFiltered = input.toFloat();
  }

  int pos = 0;
  float velocity2 = 0;

  // Obter posição e velocidade
  getEncoderData(pos, velocity2);

  // Calcular velocidade
  float deltaT = 0;
  float v1 = computeVelocity(pos, deltaT);

  // Filtrar sinais de velocidade
  filterVelocities(v1, velocity2);

  // Calcular o sinal de controle PID
  float u = computePID(vtFiltered, v1Filt, deltaT);

  // Configurar o motor
  controlMotor(u);

  // Saída para depuração
  debugOutput(vtFiltered, v1Filt, v1, e, pwr);

  delay(50);
}

// Função para obter posição e velocidade do encoder (mantida)
void getEncoderData(int &pos, float &velocity2) {
  noInterrupts();
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts();
}

// Função para calcular velocidade (mantida)
float computeVelocity(int pos, float &deltaT) {
  long currT = micros();
  deltaT = ((float)(currT - prevT)) / 1.0e6;
  float velocity = (pos - posPrev) / deltaT;
  posPrev = pos;
  prevT = currT;
  return velocity / 600.0 * 60.0;
}

// Função para filtrar velocidades (mantida)
void filterVelocities(float v1, float v2) {
  v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Prev = v1;
  v2Filt = 0.854 * v2Filt + 0.0728 * v2 + 0.0728 * v2Prev;
  v2Prev = v2;
}

// Função PID (mantida)
float computePID(float vt1, float v1Filt, float deltaT) {
  e = vt1 - v1Filt;

  eintegral += e * deltaT;
  eintegral = constrain(eintegral, -e_integral_max, e_integral_max);
  
  float ederiv = (e - eprev) / deltaT;
  eprev = e;

  return kp * e + ki * eintegral + kd * ederiv;
}

// Controle do motor (mantida)
void controlMotor(float u) {
  int dir = (u < 0) ? -1 : 1;
  pwr = constrain((int)fabs(u), 0, 255);
  setMotor(dir, pwr, PWM, IN1, IN2);
}

// Saída de depuração (ajustada)
void debugOutput(float vt1, float v1Filt, float v1, float erro, int pwm) {
  Serial.print(vt1);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.print(" ");
  Serial.print(pwm);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  digitalWrite(in1, dir == 1 ? HIGH : LOW);
  digitalWrite(in2, dir == -1 ? HIGH : LOW);
}

void readEncoder() {
  int b = digitalRead(ENCB);
  pos_i += (b > 0) ? 1 : -1;
  
  long currT = micros();
  float deltaT = ((float)(currT - prevT_i)) / 1.0e6;
  velocity_i = ((b > 0) ? 1 : -1) / deltaT;
  prevT_i = currT;
}