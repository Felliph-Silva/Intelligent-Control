// Pinos
#define ENCA 2 // Verde
#define ENCB 3 // Amarelo
#define PWM 4
#define IN1 5
#define IN2 6

// Encoder
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

// Variáveis globais
int pwr = 0;
int direcao = 1;
float v_target= 100;
long prevT = 0;
int posPrev = 0;

// Estado do motor
bool motorAtivo = false;

void setup() {
  Serial.begin(115200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  float referenciaPWM = (v_target * 255) / 220; //Conversão de RPM para PWM, 220 é a rotação máxima do motor

  referenciaPWM = constrain(referenciaPWM, -255, 255);
  direcao = (referenciaPWM < 0) ? -1 : 1;

  pwr = abs((int)referenciaPWM);

  prevT = micros();
}

void loop() {
  // Leitura de comando Serial para ligar/desligar o motor
  if (Serial.available()) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();
    if (comando == "1") {
      motorAtivo = true;
    } else if (comando == "0") {
      motorAtivo = false;
    }
  }

  // Leitura do encoder e cálculo da velocidade
  int pos;
  float vEncoder;
  getEncoderData(pos, vEncoder);

  float deltaT;
  float velocidadeCalculada = computeVelocity(pos, deltaT);
  
  if (motorAtivo) {
    setMotor(direcao, pwr, PWM, IN1, IN2);
  } else {
    setMotor(0, 0, PWM, IN1, IN2);
  }

  // Envio dos dados para análise
  debugOutput(v_target, velocidadeCalculada, (motorAtivo ? pwr * direcao : 0));

  delay(50);
}

void debugOutput(float referencia, float velocidadeLida, int pwmAplicado) {
  Serial.print(referencia);
  Serial.print(" ");
  Serial.print(velocidadeLida);
  Serial.print(" ");
  Serial.println(pwmAplicado);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
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

void getEncoderData(int &pos, float &velocity2) {
  noInterrupts();
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts();
}

float computeVelocity(int pos, float &deltaT) {
  long currT = micros();
  deltaT = ((float)(currT - prevT)) / 1.0e6;
  float velocity = (pos - posPrev) / deltaT;
  posPrev = pos;
  prevT = currT;
  return velocity / 600.0 * 60.0; // Conversão para RPM
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
