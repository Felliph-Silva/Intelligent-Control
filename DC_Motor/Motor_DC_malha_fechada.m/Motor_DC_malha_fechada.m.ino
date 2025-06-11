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
float referenciaVelocidade = 100.0; // Referência em RPM
float velocidadeFiltrada = 0;
float velocidadeAnterior = 0;
long prevT = 0;
int posPrev = 0;

// Controle
float Kp = 1.0; // Ganho proporcional
int pwmSinal = 0;

// Controle de estado do motor
bool motorAtivo = false;

void setup() {
  Serial.begin(115200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  prevT = micros();
}

void loop() {
  // Checa entrada serial para ativar/desativar motor
  if (Serial.available()) {
    char comando = Serial.read();
    if (comando == '0') {
      motorAtivo = false;
    } else if (comando == '1') {
      motorAtivo = true;
    }
  }

  // Leitura do encoder e velocidade
  int pos;
  float vEncoder;
  getEncoderData(pos, vEncoder);

  float deltaT;
  float velocidadeCalculada = computeVelocity(pos, deltaT);

  // Filtragem da velocidade
  velocidadeFiltrada = 0.854 * velocidadeFiltrada + 0.0728 * velocidadeCalculada + 0.0728 * velocidadeAnterior;
  velocidadeAnterior = velocidadeCalculada;

  if (motorAtivo) {
    // CONTROLE EM MALHA FECHADA
    float erro = referenciaVelocidade - velocidadeFiltrada;
    pwmSinal = Kp * erro;
    pwmSinal = constrain(pwmSinal, -255, 255);

    int direcao = (pwmSinal >= 0) ? 1 : -1;
    int pwr = abs(pwmSinal);

    setMotor(direcao, pwr, PWM, IN1, IN2);
  } else {
    setMotor(0, 0, PWM, IN1, IN2); // Desliga motor
    pwmSinal = 0;
  }

  // Envia dados para serial
  debugOutput(referenciaVelocidade, velocidadeFiltrada, pwmSinal);

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
