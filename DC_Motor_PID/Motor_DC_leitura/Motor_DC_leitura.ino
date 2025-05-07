// Pins
#define ENCA 2
#define ENCB 3
#define PWM 4
#define IN1 5
#define IN2 6

// Globals
long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;
int vref = 0; // Velocidade de referência (em RPM)
int pwmValue = 0; // Valor PWM (0-255)
int dir = 1; // Direção (1 para frente, -1 para trás)

void setup() {
  Serial.begin(115200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  
  Serial.println("Digite a velocidade de referencia (RPM) no Serial Monitor:");
  Serial.println("Valores positivos para uma direcao, negativos para a direcao oposta");
}

void loop() {
  // Verificar se há dados disponíveis no Serial
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    vref = input.toInt();
    
    // Determinar direção e converter para PWM absoluto
    if (vref > 0) {
      dir = 1;
      pwmValue = map(abs(vref), 0, 280, 0, 255);
    } else if (vref < 0) {
      dir = -1;
      pwmValue = map(abs(vref), 0, 280, 0, 255);
    } else {
      dir = 0;
      pwmValue = 0;
    }
    
    // Limitar o PWM entre 0 e 255
    pwmValue = constrain(pwmValue, 0, 255);
    
    Serial.print("Velocidade de referencia definida para: ");
    Serial.print(vref);
    Serial.print(" RPM | PWM: ");
    Serial.print(pwmValue);
    Serial.print(" | Direcao: ");
    Serial.println(dir == 1 ? "Frente" : (dir == -1 ? "Tras" : "Parado"));
  }

  int pos = 0;
  float velocity2 = 0;

  // Obter posição e velocidade
  getEncoderData(pos, velocity2);

  // Calcular velocidade
  float deltaT = 0;
  float v1 = computeVelocity(pos, deltaT);

  // Saída para depuração
  debugOutput(v1, vref);

  // Controlar o motor com os valores recebidos
  setMotor(dir, pwmValue, PWM, IN1, IN2);
  
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
  Serial.print("Velocidade medida: ");
  Serial.print(v1);
  Serial.print(" RPM | Velocidade referencia: ");
  Serial.print(Vref);
  Serial.print(" RPM | PWM: ");
  Serial.print(pwmValue);
  Serial.print(" | Direcao: ");
  Serial.println(dir == 1 ? "Frente" : (dir == -1 ? "Tras" : "Parado"));
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