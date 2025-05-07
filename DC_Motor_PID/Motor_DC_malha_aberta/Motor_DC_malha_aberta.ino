// Pins
#define ENCA 3
#define ENCB 2
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

int pwr = 0;
bool motorLigado = false; // Estado do motor (ligado/desligado)

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
  // Verifica se há dados disponíveis na serial (comando do MATLAB)
  if (Serial.available() > 0) {
    char comando = Serial.read(); // Lê o comando
    if (comando == '1') {
      motorLigado = true; // Liga o motor
    } else if (comando == '0') {
      motorLigado = false; // Desliga o motor
      setMotor(0, 0, PWM, IN1, IN2); // Para o motor
    }
  }

  // Se o motor estiver ligado, aplica o sinal de controle
  if (motorLigado) {
    int pos = 0;
    float velocity2 = 0;

    // Obter posição e velocidade
    getEncoderData(pos, velocity2);

    // Calcular velocidade
    float deltaT = 0;
    float v1 = computeVelocity(pos, deltaT);

    // Filtrar sinais de velocidade
    filterVelocities(v1, velocity2);

    // Definir a velocidade de referência
    float vt = 200; // Velocidade desejada (RPM)

    // Configurar o motor
    controlMotor(vt);

    // Saída para depuração
    debugOutput(vt, v1Filt, pwr);
  }

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  delay(50);
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
void debugOutput(float vt1, float v1Filt, int pwm) {
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