// Pinos
#define ENCA 3 // Verde
#define ENCB 2 // Amarelo
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
float referenciaPWM = 100; // <- PWM fixo aqui
float velocidadeFiltrada = 0;
float velocidadeAnterior = 0;
long prevT = 0;
int posPrev = 0;

void setup() {
  Serial.begin(115200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  // Define valores iniciais de PWM e direção
  referenciaPWM = constrain(referenciaPWM, -255, 255);
  direcao = (referenciaPWM < 0) ? -1 : 1;
  pwr = abs((int)referenciaPWM);
}

void loop() {
  // Calcular velocidade estimada
  int pos;
  float vEncoder;
  getEncoderData(pos, vEncoder);

  float deltaT;
  float velocidadeCalculada = computeVelocity(pos, deltaT);

  // Filtragem simples
  velocidadeFiltrada = 0.854 * velocidadeFiltrada + 0.0728 * velocidadeCalculada + 0.0728 * velocidadeAnterior;
  velocidadeAnterior = velocidadeCalculada;

  // Aplicar PWM diretamente (malha aberta)
  setMotor(direcao, pwr, PWM, IN1, IN2);

  // Enviar dados para análise (Serial)
  debugOutput(referenciaPWM, velocidadeFiltrada, pwr * direcao);

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
