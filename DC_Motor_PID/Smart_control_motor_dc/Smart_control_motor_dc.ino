// Pins
#define ENCA 5
#define ENCB 6
#define PWM 2
#define IN1 3
#define IN2 4


// Globals
long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;
int vref = 280;

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

  // Saída para depuração
  debugOutput(v1, vref);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  setMotor(1, 255, PWM, IN1, IN2);
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

// Função para saída de depuração
void debugOutput(float v1, float Vref) {
  Serial.print("Velocidade medida pelo encoder: ");
  Serial.print(v1);
  Serial.print(" RPM");
  Serial.print("  Velocidade referencia: ");
  Serial.print(Vref);
  Serial.print(" RPM");
  Serial.println("");
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
