// Pins (sem alterações)
#define ENCA 2 // Green
#define ENCB 3 //Yellow
#define PWM 4
#define IN1 5
#define IN2 6

// Globals (algumas novas para autotune)
long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
// float v2Filt = 0; // v2Filt e v2Prev não parecem ser usados ativamente no controle PID
// float v2Prev = 0;
float v_target = 0; // Referência via serial (para PID normal)
float autotune_setpoint = 0; // Setpoint em torno do qual o relé vai oscilar

float eintegral = 0;
float eprev = 0;
int pwr = 0;
float e = 0; // Sinal de erro na realimentação

// Ganhos PID (serão ajustados pelo autotune)
float kp = 0.32;
float ki = 0.8;
float kd = 0.2;

float e_integral_max = 600; // Limite para o erro integral

// Estado do motor
bool motorAtivo = false;

// --- Variáveis para Autotune por Relé ---
enum AutotuneState { IDLE, RUNNING, DONE_TUNING };
AutotuneState autotune_state = IDLE;

float relay_output_amplitude = 150; // Amplitude do relé (valor de PWM). Ajuste conforme necessário.
                                    // Deve ser grande o suficiente para causar oscilação, mas não excessivo.
unsigned long last_crossing_time_us = 0;
float last_velocity_reading = 0;

float cycle_peaks[5];     // Armazena os picos de velocidade de cada ciclo
float cycle_valleys[5];   // Armazena os vales de velocidade de cada ciclo
unsigned long cycle_periods_us[5]; // Armazena os períodos de cada ciclo
int cycle_count = 0;      // Contador de ciclos de oscilação detectados
const int NUM_CYCLES_FOR_AVG = 3; // Número de ciclos estáveis para calcular a média
const int CYCLES_TO_SKIP = 2;     // Número de ciclos iniciais para ignorar (para estabilização)

unsigned long autotune_start_time_ms = 0;
const unsigned long AUTOTUNE_TIMEOUT_MS = 60000; // Timeout de 60 segundos para o autotune

void setup() {
  Serial.begin(115200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  Serial.println("Setup completo. Envie '1' para ligar, '0' para desligar, 'TUNE valor_setpoint valor_amplitude_rele' para iniciar autotune.");
  Serial.println("Exemplo TUNE: TUNE 50 150 (Setpoint 50, Amplitude PWM 150)");
}

void loop() {
  // Leitura de comando Serial
  if (Serial.available()) {
    handleSerialInput();
  }

  int pos = 0;
  float velocity2_dummy = 0; // velocity2 não é usada no PID, mas getEncoderData a retorna

  // Obter posição e velocidade
  getEncoderData(pos, velocity2_dummy);

  // Calcular velocidade
  float deltaT_micros = 0; // Renomeado para evitar conflito com deltaT em computePID
  float current_velocity_raw = computeRawVelocity(pos, deltaT_micros);

  // Filtrar sinal de velocidade
  filterVelocities(current_velocity_raw); // Modificado para aceitar apenas v1

  if (autotune_state == RUNNING) {
    runAutotuneRelayCycle(v1Filt, deltaT_micros / 1.0e6); // Passa deltaT em segundos
  } else if (motorAtivo) {
    // Controle PID do motor
    float u = computePID(v_target, v1Filt, deltaT_micros / 1.0e6); // Passa deltaT em segundos
    controlMotor(u);
    // Saída para depuração PID
    debugOutput(v_target, v1Filt, pwr);
  } else {
    setMotor(0, 0, PWM, IN1, IN2); // Desliga motor
    eintegral = 0; // Reseta integral quando motor está desligado
    eprev = 0;
  }
  
  // Delay pequeno para estabilidade do loop e leitura serial
  // O delay original de 50ms pode ser muito longo para um controle rápido ou autotune preciso.
  // Ajuste conforme necessário. Para autotune, um loop mais rápido pode ser melhor.
  delay(10); // Reduzido para permitir detecção mais rápida de oscilações
}

void handleSerialInput() {
  String input_str = Serial.readStringUntil('\n');
  input_str.trim();

  if (input_str.startsWith("TUNE")) {
    // Formato esperado: "TUNE setpoint amplitude_rele"
    // Ex: "TUNE 50 150"
    int first_space = input_str.indexOf(' ');
    int second_space = input_str.indexOf(' ', first_space + 1);
    if (first_space > 0 && second_space > first_space) {
        String sp_str = input_str.substring(first_space + 1, second_space);
        String amp_str = input_str.substring(second_space + 1);
        float sp = sp_str.toFloat();
        float amp = amp_str.toFloat();

        if (amp > 20 && amp <= 255) { // Amplitude mínima para garantir movimento
            autotune_setpoint = sp;
            relay_output_amplitude = amp;
            startAutotune();
        } else {
            Serial.println("Amplitude do rele invalida (20-255).");
        }
    } else {
        Serial.println("Formato TUNE: TUNE valor_setpoint valor_amplitude_rele");
    }
  } else if (input_str == "1") {
    if (autotune_state != IDLE && autotune_state != DONE_TUNING) {
        Serial.println("Autotune em progresso. Pare antes de ligar o PID.");
    } else {
        motorAtivo = true;
        autotune_state = IDLE; // Garante que saiu do modo autotune
        eintegral = 0; // Reseta integral ao (re)ligar
        eprev = v1Filt; // Define eprev para a velocidade atual para evitar salto no derivativo
        Serial.println("Motor ATIVADO (Modo PID).");
    }
  } else if (input_str == "0") {
    motorAtivo = false;
    if (autotune_state == RUNNING) {
        Serial.println("Autotune interrompido.");
    }
    autotune_state = IDLE;
    setMotor(0, 0, PWM, IN1, IN2); // Garante que o motor pare
    Serial.println("Motor DESATIVADO.");
  } else {
    // Se não estiver em autotune e o motor estiver ativo, interpreta como novo v_target
    if (autotune_state == IDLE && motorAtivo) {
        float new_target = input_str.toFloat();
        if (new_target != v_target) {
            v_target = new_target;
            eintegral = 0; // Reseta integral ao mudar setpoint para evitar windup
            eprev = v1Filt; // Atualiza eprev
            Serial.print("Novo Setpoint PID: ");
            Serial.println(v_target);
        }
    }
  }
}


// Função para obter posição e velocidade do encoder (mantida)
void getEncoderData(int &pos, float &velocity2) {
  noInterrupts();
  pos = pos_i;
  // velocity2 = velocity_i; // velocity_i não é mais atualizada na ISR da forma antiga
  interrupts();
  // velocity_i é agora apenas um contador de pulsos na ISR,
  // o cálculo de velocidade é feito em computeRawVelocity
}

// Função para calcular velocidade (modificada para retornar deltaT em micros)
float computeRawVelocity(int pos, float &deltaT_micros_out) {
  long currT_micros = micros();
  deltaT_micros_out = (float)(currT_micros - prevT);
  
  float velocity_pps; // Pulsos por segundo
  if (deltaT_micros_out > 0) {
    velocity_pps = (float)(pos - posPrev) / (deltaT_micros_out / 1.0e6);
  } else {
    velocity_pps = 0;
  }
  
  posPrev = pos;
  prevT = currT_micros;
  
  // Converter pulsos por segundo para a unidade desejada (ex: RPM)
  // Se 600 pulsos por revolução: RPM = (velocity_pps / 600) * 60
  // Ajuste este fator conforme a sua unidade de velocidade alvo e contagem do encoder
  return velocity_pps / 600.0 * 60.0; // Exemplo: RPM se 600 PPR
}


// Função para filtrar velocidades (modificada para aceitar e filtrar apenas v1)
void filterVelocities(float v_raw) {
  v1Filt = 0.854 * v1Filt + 0.0728 * v_raw + 0.0728 * v1Prev;
  v1Prev = v_raw;
}

// Função PID (deltaT agora é passado em segundos)
float computePID(float vt, float current_v, float deltaT_s) {
  if (deltaT_s <= 0) return kp * e; // Evita divisão por zero se deltaT for inválido

  e = vt - current_v;

  eintegral += e * deltaT_s;
  eintegral = constrain(eintegral, -e_integral_max, e_integral_max);
  
  float ederiv = (e - eprev) / deltaT_s;
  eprev = e;

  return kp * e + ki * eintegral + kd * ederiv;
}

// Controle do motor (mantida)
void controlMotor(float u) {
  int dir = (u < 0) ? -1 : 1;
  pwr = constrain((int)fabs(u), 0, 255);
  setMotor(dir, pwr, PWM, IN1, IN2);
}

// Saída de depuração (ajustada para autotune e PID)
void debugOutput(float target_val, float current_val_filt, int current_pwm) {
  Serial.print(target_val);
  Serial.print(" ");
  Serial.print(current_val_filt);
  Serial.print(" ");
  Serial.print(current_pwm);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwmPin, int in1Pin, int in2Pin) {
  analogWrite(pwmPin, pwmVal);
  if (dir == 1) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (dir == -1) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else { // dir == 0 (parar)
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}

// ISR do Encoder (simplificada, apenas conta posições)
void readEncoder() {
  int b = digitalRead(ENCB);
  pos_i += (b > 0) ? 1 : -1;
  // Cálculo de velocidade removido daqui para ser feito no loop principal
  // para maior precisão de deltaT e evitar float em ISR.
}

// --- Funções de Autotune por Relé ---

void startAutotune() {
  Serial.println("--- Iniciando Autotune por Rele ---");
  Serial.print("Setpoint para oscilacao: "); Serial.println(autotune_setpoint);
  Serial.print("Amplitude do rele (PWM): "); Serial.println(relay_output_amplitude);

  motorAtivo = true; // Autotune precisa do motor ativo
  autotune_state = RUNNING;
  cycle_count = 0;
  
  // Reseta dados de ciclos anteriores
  for(int i=0; i<NUM_CYCLES_FOR_AVG; ++i) {
    cycle_peaks[i] = 0;
    cycle_valleys[i] = 0;
    cycle_periods_us[i] = 0;
  }
  
  last_crossing_time_us = micros();
  last_velocity_reading = v1Filt; // Usa a velocidade filtrada atual como referência inicial
  autotune_start_time_ms = millis();

  // Reseta o integrador e erro previo do PID para quando voltar ao modo PID
  eintegral = 0;
  eprev = 0; 
  
  Serial.println("Autotune: Aguardando estabilizacao e oscilacoes...");
}

void runAutotuneRelayCycle(float current_velocity, float deltaT_s) {
  if (millis() - autotune_start_time_ms > AUTOTUNE_TIMEOUT_MS) {
    Serial.println("Autotune TIMEOUT!");
    finishAutotune(false); // Finaliza sem sucesso
    return;
  }

  float error_autotune = autotune_setpoint - current_velocity;
  int relay_pwm_signal;

  // Lógica do Relé Simples (sem histerese por enquanto para o método clássico)
  // Se a velocidade está abaixo do setpoint (erro positivo), aplica PWM positivo.
  // Se a velocidade está acima do setpoint (erro negativo), aplica PWM negativo.
  if (error_autotune > 0) {
    relay_pwm_signal = relay_output_amplitude;
  } else {
    relay_pwm_signal = -relay_output_amplitude;
  }
  controlMotor(relay_pwm_signal); // Aplica o sinal do relé

  // Detecção de cruzamento pelo setpoint
  // Cruzou de baixo para cima
  bool crossed_up = (last_velocity_reading < autotune_setpoint && current_velocity >= autotune_setpoint);
  // Cruzou de cima para baixo
  bool crossed_down = (last_velocity_reading > autotune_setpoint && current_velocity <= autotune_setpoint);

  unsigned long now_us = micros();

  // Tenta identificar um ciclo completo (pico-vale-pico ou vale-pico-vale)
  // Esta é uma forma simplificada: mede período entre cruzamentos na mesma direção
  // e amplitude entre o pico e o vale dentro desse período.
  
  // Variáveis static para rastrear picos/vales dentro de um semi-ciclo
  static float current_cycle_peak = -1e9; // Ou autotune_setpoint;
  static float current_cycle_valley = 1e9;  // Ou autotune_setpoint;
  static unsigned long time_of_last_up_crossing = 0;
  static bool waiting_for_peak_after_up_crossing = false;
  static bool waiting_for_valley_after_down_crossing = false;


  if (crossed_up) {
    if (time_of_last_up_crossing > 0 && waiting_for_valley_after_down_crossing) { // Completou um ciclo: subiu -> (atingiu pico) -> desceu -> (atingiu vale) -> subiu de novo
      unsigned long period_us = now_us - time_of_last_up_crossing;
      float amplitude_pv = current_cycle_peak - current_cycle_valley; // Amplitude pico-a-vale

      if (cycle_count >= CYCLES_TO_SKIP) {
        int idx = cycle_count - CYCLES_TO_SKIP;
        if (idx < NUM_CYCLES_FOR_AVG) {
          cycle_periods_us[idx] = period_us;
          // A amplitude 'a' na fórmula de Ziegler-Nichols é a amplitude da oscilação da saída (metade do pico-a-vale)
          cycle_peaks[idx] = current_cycle_peak; // Usado para calcular 'a'
          cycle_valleys[idx] = current_cycle_valley; // Usado para calcular 'a'
          Serial.print("Ciclo "); Serial.print(idx+1); Serial.print(": T_us="); Serial.print(period_us);
          Serial.print(", Pico="); Serial.print(current_cycle_peak); Serial.print(", Vale="); Serial.println(current_cycle_valley);
        }
      }
      cycle_count++;
    }
    time_of_last_up_crossing = now_us;
    current_cycle_peak = current_velocity; // Reset, o pico será daqui para frente
    waiting_for_peak_after_up_crossing = true;
    waiting_for_valley_after_down_crossing = false; // Não mais esperando por vale
  } else if (crossed_down) {
    // Registra que cruzou para baixo, agora vamos procurar o vale.
    current_cycle_valley = current_velocity; // Reset, o vale será daqui para frente
    waiting_for_valley_after_down_crossing = true;
    waiting_for_peak_after_up_crossing = false; // Não mais esperando por pico
  }

  // Atualiza pico/vale do semi-ciclo atual
  if (waiting_for_peak_after_up_crossing && current_velocity > current_cycle_peak) {
    current_cycle_peak = current_velocity;
  }
  if (waiting_for_valley_after_down_crossing && current_velocity < current_cycle_valley) {
    current_cycle_valley = current_velocity;
  }

  last_velocity_reading = current_velocity;

  // Verifica se coletou ciclos suficientes
  if (cycle_count >= (NUM_CYCLES_FOR_AVG + CYCLES_TO_SKIP)) {
    finishAutotune(true); // Finaliza com sucesso
  }
  
  // Debug do autotune
  if((now_us / 100000) % 5 == 0){ // Imprime a cada 0.5s aproximadamente
      Serial.print("AT: sp="); Serial.print(autotune_setpoint);
      Serial.print(", vFilt="); Serial.print(current_velocity);
      Serial.print(", err="); Serial.print(error_autotune);
      Serial.print(", relayOut="); Serial.println(relay_pwm_signal);
  }
}

void finishAutotune(bool success) {
  setMotor(0, 0, PWM, IN1, IN2); // Para o motor ao final do autotune
  motorAtivo = false; // Para o motor para segurança, usuário deve religar
  autotune_state = DONE_TUNING;

  if (!success) {
    Serial.println("--- Autotune Falhou ou Interrompido ---");
    Serial.println("Ganhos PID nao alterados.");
    return;
  }

  Serial.println("--- Autotune: Calculando Ganhos ---");
  float sum_Tu_s = 0;
  float sum_amplitude_a = 0; // Amplitude 'a' = (Pico - Vale) / 2
  int valid_cycles_for_avg = 0;

  for (int i = 0; i < NUM_CYCLES_FOR_AVG; i++) {
    if (cycle_periods_us[i] > 0 && (cycle_peaks[i] > cycle_valleys[i])) { // Verifica dados válidos
      sum_Tu_s += (float)cycle_periods_us[i] / 1.0e6;
      sum_amplitude_a += (cycle_peaks[i] - cycle_valleys[i]) / 2.0;
      valid_cycles_for_avg++;
    }
  }

  if (valid_cycles_for_avg < 1) {
    Serial.println("Nao foi possivel obter medicoes validas de ciclo.");
    Serial.println("Ganhos PID nao alterados.");
    return;
  }

  float avg_Tu_s = sum_Tu_s / valid_cycles_for_avg;
  float avg_a = sum_amplitude_a / valid_cycles_for_avg;

  Serial.print("Periodo Medio de Oscilacao (Tu): "); Serial.print(avg_Tu_s, 4); Serial.println(" s");
  Serial.print("Amplitude Media da Saida (a): "); Serial.print(avg_a, 4); Serial.println(" (unidade de velocidade)");

  if (avg_a < 0.01) { // Evita divisão por zero ou ganhos excessivos se não houve oscilação significativa
    Serial.println("Amplitude de oscilacao muito pequena. Nao foi possivel calcular ganhos.");
    return;
  }
  if (avg_Tu_s < 0.01) {
    Serial.println("Periodo de oscilacao muito pequeno. Nao foi possivel calcular ganhos.");
    return;
  }


  // Ganho Crítico Ku
  // h é a amplitude da saída do relé (relay_output_amplitude em termos de PWM)
  // No entanto, a fórmula de Ku espera que 'h' e 'a' estejam em unidades consistentes
  // Se 'a' é a amplitude da velocidade, 'h' deveria ser a mudança na velocidade causada pelo relé.
  // Uma abordagem mais comum é usar a amplitude do relé (PWM) diretamente e 'a' como amplitude da saída.
  float Ku = (4.0 * relay_output_amplitude) / (PI * avg_a); // PI é uma constante definida no Arduino
  Serial.print("Ganho Critico (Ku): "); Serial.println(Ku, 4);

  // Ganhos PID Clássico de Ziegler-Nichols
  float new_kp = 0.6 * Ku;
  float Ti = avg_Tu_s / 2.0;
  float Td = avg_Tu_s / 8.0;

  float new_ki = new_kp / Ti; // Se Ti for zero, ki será infinito. avg_Tu_s já foi verificado.
  float new_kd = new_kp * Td;

  Serial.println("--- Novos Ganhos PID Calculados ---");
  Serial.print("Kp: "); Serial.println(new_kp, 4);
  Serial.print("Ki: "); Serial.println(new_ki, 4);
  Serial.print("Kd: "); Serial.println(new_kd, 4);

  // Atualiza os ganhos globais
  kp = new_kp;
  ki = new_ki;
  kd = new_kd;
  
  eintegral = 0; // Reseta o integrador para o novo PID
  eprev = v1Filt;  // Define o erro previo para a velocidade atual

  Serial.println("Ganhos PID atualizados. Envie '1' para ativar o motor com os novos ganhos.");
}