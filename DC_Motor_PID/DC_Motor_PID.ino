// Pins
#define ENCA 3 // Green
#define ENCB 2 //Yellow
#define PWM 4
#define IN1 5
#define IN2 6
#define POT A0

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
static float vtFiltered = 200;


float eintegral = 0;
float eprev = 0;
int pwr = 0;
float e = 0; // Sinal de erro na realimentação

//Ajuste os ganhos
float kp = 0.32; //0.5;
float ki = 0.8; //0.5;
float kd = 0.2; //0.5

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
  int pos = 0;
  float velocity2 = 0;

  // Obter posição e velocidade
  getEncoderData(pos, velocity2);

  // Calcular velocidade
  float deltaT = 0;
  float v1 = computeVelocity(pos, deltaT);

  // Filtrar sinais de velocidade
  filterVelocities(v1, velocity2);
/*
  // Ler o valor desejado do potenciômetro
  float vt = readTargetSpeed();
  vtFiltered = 0.9 * vtFiltered + 0.1 * vt;  // Filtragem simples da referência
*/
  // Calcular o sinal de controle PID
  float u = computePID(vtFiltered, v1Filt, deltaT);

  // Configurar o motor
  controlMotor(u);

  // Saída para depuração
  debugOutput(vtFiltered, v1Filt, v1, e, pwr);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  delay(25);
}

// Função para obter posição e velocidade do encoder
void getEncoderData(int &pos, float &velocity2) {
  noInterrupts(); // Desativar interrupções ao ler dados
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts(); // Reativar interrupções
}

// Função para calcular velocidade com base na posição
float computeVelocity(int pos, float &deltaT) {
  long currT = micros();
  deltaT = ((float)(currT - prevT)) / 1.0e6;
  float velocity = (pos - posPrev) / deltaT;
  posPrev = pos;
  prevT = currT;
  return velocity / 600.0 * 60.0; // Converter para RPM
}

// Função para filtrar velocidades
void filterVelocities(float v1, float v2) {
  v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Prev = v1;
  v2Filt = 0.854 * v2Filt + 0.0728 * v2 + 0.0728 * v2Prev;
  v2Prev = v2;
}

// Função para ler o valor alvo do potenciômetro
float readTargetSpeed() {
  int pot = analogRead(POT);
  int pot_scaled = map(pot, 0, 1023, 0, 285);
  return pot_scaled;
}

// Função para calcular o sinal de controle PID
float computePID(float vt1, float v1Filt, float deltaT) {
  e = vt1 - v1Filt;

  // Atualizar erro integral com limite
  eintegral += e * deltaT;
  if (eintegral > e_integral_max) {
    eintegral = e_integral_max;
  } else if (eintegral < -e_integral_max) {
    eintegral = -e_integral_max;
  }

  // Calcular erro derivativo
  float ederiv = (e - eprev) / deltaT;
  eprev = e;

  // Calcular o sinal PID
  return kp * e + ki * eintegral + kd * ederiv;
}

// Função para configurar o motor
void controlMotor(float u) {
  int dir = (u < 0) ? -1 : 1;
  pwr = (int)fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }
  setMotor(dir, pwr, PWM, IN1, IN2);
}

// Função para saída de depuração
void debugOutput(float vt1, float v1Filt, float v1, float erro, int pwm) {
  Serial.print(vt1);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.print(" ");
  Serial.print(pwm);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal); // Velocidade do motor
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  int b = digitalRead(ENCB);
  int increment = (b > 0) ? 1 : -1;
  pos_i += increment;

  long currT = micros();
  float deltaT = ((float)(currT - prevT_i)) / 1.0e6;
  velocity_i = increment / deltaT;
  prevT_i = currT;
}
