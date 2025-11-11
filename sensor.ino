#include <QTRSensors.h>

const uint8_t SensorCount = 8;
unsigned int sensorValues[SensorCount];

// pinos dos sensores (ajuste conforme seu hardware)
QTRSensorsRC qtr((unsigned char[]){2, 0, 4, 5, 6, 7, 8, 9}, SensorCount, 2000, 10);

void setup() {
  Serial.begin(9600);
  delay(500);

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

  Serial.print(" | Posicao: ");
  Serial.println(position);

  delay(1000);
}
