#include <QTRSensors.h>

// --- Pinos do Motor ---
#define PINO_IN1 18  // Pino PWM responsável pelo Motor 1
#define PINO_IN3 19  // Pino PWM responsável pelo Motor 2
// (Nota: Verifique se sua ponte-H precisa de pinos IN2 e IN4 para direção)

// --- Configuração dos Sensores ---
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t EmitterPin = 10; // NOVO: Pino de controle do LED IR

// ATUALIZADO: Construtor moderno da QTR
QTRSensors qtr;

// --- Configurações do Robô ---
int VELOCIDADE_BASE = 150; // Velocidade base (0-255)
// (O seu 'velocidade = 50' era muito baixo, ajuste conforme necessário)

// --- Constantes PID (AJUSTE ESTES VALORES!) ---
// Comece com Kp, depois adicione Kd, e por último Ki se precisar
double Kp = 0.1;  // Ganho Proporcional (Reage ao erro atual)
double Ki = 0.0;  // Ganho Integral (Corrige erros persistentes)
double Kd = 0.05; // Ganho Derivativo (Prevê erros futuros, amortece)

// --- Variáveis do PID ---
double setpoint = 3500;   // Objetivo: 3500 (centro de 8 sensores)
double termoIntegral = 0;
double erroAnterior = 0;
unsigned long tempoAnterior = 0;

// --- Limites do PID (Anti-Windup e Saturação) ---
double MAX_INTEGRAL = 200; // Limite do termo integral
double MIN_INTEGRAL = -200;
int MAX_OUTPUT = 255;      // Limite do PWM (não mude)
int MIN_OUTPUT = 0;        // Limite do PWM (não mude)


void setup() {
  Serial.begin(9600);
  delay(500);

  pinMode(PINO_IN1, OUTPUT);
  pinMode(PINO_IN3, OUTPUT);

  // NOVO: Configuração dos pinos dos sensores (sintaxe moderna)
  qtr.setSensorPins((const uint8_t[]){2, 0, 4, 5, 6, 7, 8, 9}, SensorCount);
  qtr.setEmitterPin(EmitterPin);
  qtr.setTimeout(2000);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // LED aceso durante calibração

  // Calibração
  Serial.println("Calibrando... Mova o robô sobre a linha.");
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(5);
  }
  digitalWrite(LED_BUILTIN, LOW); // calibração terminada

  // Mostrar valores calibrados (sintaxe atualizada)
  Serial.println("Calibracao minima:");
  for (uint8_t i = 0; i < SensorCount; i++) {
    // ATUALIZADO: 'calibrationOn.minimum'
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.println("Calibracao maxima:");
  for (uint8_t i = 0; i < SensorCount; i++) {
    // ATUALIZADO: 'calibrationOn.maximum'
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  // NOVO: Inicia o cronômetro do PID
  tempoAnterior = millis();
}

void loop() {
  
  // ATUALIZADO: Use 'readLineBlack' (para linha preta)
  // ou 'readLineWhite' (para linha branca)
  unsigned int position = qtr.readLineBlack(sensorValues);

  // (Descomente para debug, mas saiba que Serial.print deixa o loop lento)
  /*
  Serial.print("Posicao: ");
  Serial.println(position);
  */

  // #################################################
  // ##            INÍCIO DO CÁLCULO PID            ##
  // #################################################
  // Esta seção substitui seu cálculo 'pound' e 'correcao'

  // 1. Cálculo do Tempo (dt)
  unsigned long agora = millis();
  double dt = (agora - tempoAnterior) / 1000.0; // dt em segundos

  // 2. Cálculo do Erro
  // 'position' (0-7000) é o nosso 'input'
  // 'setpoint' (3500) é o centro
  double erro = setpoint - position;

  // 3. Termo P
  double termoP = Kp * erro;

  // 4. Termo I (com Anti-Windup)
  termoIntegral += erro * dt;
  if (termoIntegral > MAX_INTEGRAL) {
    termoIntegral = MAX_INTEGRAL;
  } else if (termoIntegral < MIN_INTEGRAL) {
    termoIntegral = MIN_INTEGRAL;
  }

  // 5. Termo D (Cuidado com dt=0 na primeira volta)
  double derivada = 0;
  if (dt > 0) {
    derivada = (erro - erroAnterior) / dt;
  }
  double termoD = Kd * derivada;

  // 6. Saída Final (Correção)
  // O 'output' é a *correção* que vamos aplicar aos motores
  double output = termoP + (Ki * termoIntegral) + termoD;

  // 7. Atualização das Memórias
  erroAnterior = erro;
  tempoAnterior = agora;

  // #################################################
  // ##           FIM DO CÁLCULO PID                ##
  // #################################################

  // --- Controle dos Motores ---

  // Calcula a velocidade de cada motor
  // A 'output' (correção) é somada em um lado e subtraída do outro
  int velEsquerda = VELOCIDADE_BASE + output;
  int velDireita  = VELOCIDADE_BASE - output;

  // Saturação da Saída (limita a velocidade final nos limites do PWM)
  velEsquerda = constrain(velEsquerda, MIN_OUTPUT, MAX_OUTPUT);
  velDireita  = constrain(velDireita, MIN_OUTPUT, MAX_OUTPUT);

  // Aplica a saída
  // (ATENÇÃO: Seu código original fazia 'velEsquerda-100', o que
  // provavelmente estava errado. Esta é a forma padrão.)
  analogWrite(PINO_IN1, velEsquerda);
  analogWrite(PINO_IN3, velDireita);
  
  // REMOVIDO: O 'delay(50)' foi removido.
  // O PID funciona melhor calculando o 'dt' real.
}
