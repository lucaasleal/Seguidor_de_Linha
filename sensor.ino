#include <QTRSensors.h>

#define PINO_IN1 18  // Pino responsável pelo controle no sentido horário - M1
#define PINO_IN3 19  // Pino responsável pelo controle no sentido horário - M2

const uint8_t SensorCount = 8;
unsigned int sensorValues[SensorCount];

// pinos dos sensores (ajuste conforme seu hardware)
QTRSensorsRC qtr((unsigned char[]){2, 0, 4, 5, 6, 7, 8, 9}, SensorCount, 2000, 10);



void setup() {
  Serial.begin(9600);
  delay(500);

  pinMode(PINO_IN1, OUTPUT);
  pinMode(PINO_IN3, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // LED aceso durante calibração

  // Calibração — mova o sensor sobre o preto e o branco
  Serial.println("Calibrando...");
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(5);
  }

  digitalWrite(LED_BUILTIN, LOW); // calibração terminada

  // Mostrar valores mínimos e máximos calibrados
  Serial.println("Calibração mínima:");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.println("Calibração máxima:");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop() {
  // Lê posição da linha (0–7000)
  unsigned int position = qtr.readLine(sensorValues);

  // imprime os valores dos 8 sensores corretamente
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print("Sensor ");
    Serial.print(i);          // <-- aqui é o índice correto!
    Serial.print(": ");
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println("\n");

  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorValues[i] = !(sensorValues[i]<900);
  }
  
  int pound = 0;
  for(int i=1; i<5; i++){
    pound -= sensorValues[i-1]* i;
  }
  for(int i=1; i<5; i++){
    pound += sensorValues[i+3]* i;
  }

  int velocidade = 50;
  int ganho = 50;
  int correcao = pound * ganho;

  int velEsquerda = velocidade - correcao;
  int velDireita  = velocidade + correcao;

  velEsquerda = constrain(velEsquerda, 0, 255);
  velDireita  = constrain(velDireita, 0, 255);

  analogWrite(PINO_IN1, velEsquerda-100);
  analogWrite(PINO_IN3, velDireita+100);
  
  delay(50);
}
