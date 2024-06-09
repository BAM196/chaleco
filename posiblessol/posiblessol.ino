#include <TinyGPSPlus.h>
#include <PulseSensorPlayground.h>
#include <SoftwareSerial.h>

#define BUZZER_PIN 9

SoftwareSerial espSerial(4, 5); // Pines 4 y 5 para ESP32
SoftwareSerial gpsSerial(3, 2); // Pines 3 y 2 para GPS

const int PulseWire = A0; // Sensor de pulso en el pin A0
int Threshold = 550;

PulseSensorPlayground pulseSensor;

static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;

// Variables para oxímetro
int sensorInfrarrojo = A1; // Fototransistor infrarrojo
int sensorRojo = A2;       // Fototransistor rojo
unsigned int intensidad_infrarrojo; // Intensidad reflejada HbO2 (oxihemoglobina)
unsigned int intensidad_rojo;       // Intensidad reflejada Hb (hemoglobina)
const int numReadings = 100;
int readings[numReadings];
int indice = 0;
int total = 0; // Total
float average = 0; // Promedio

//gps
  float latitud = 0;
  float longitud = 0;
  float altitud = 0;
  float velocidad = 0;

void setup() {
  Serial.begin(9600);       // Monitor serial
  espSerial.begin(115200);  // Comunicación con ESP32
  gpsSerial.begin(GPSBaud); // Comunicación con GPS

  pulseSensor.analogInput(PulseWire);
  pulseSensor.setThreshold(Threshold);
  pulseSensor.begin();

  pinMode(BUZZER_PIN, OUTPUT);

  Serial.print(F("Testing TinyGPSPlus library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());

  // Inicializar el arreglo de lecturas del oxímetro
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;
}

void loop() {
  // Leer datos del GPS
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  latitud = gps.location.lat()
  longitud = gps.location.lng()
  altitud = gps.altitude.meters()
  velocidad = gps.speed.kmph()
  // Procesar y enviar datos del sensor de pulso
  if (pulseSensor.sawStartOfBeat()) {
    int BPM = pulseSensor.getBeatsPerMinute();
    Serial.print("BPM: ");
    Serial.println(BPM);
    espSerial.println(BPM);
  }

  // Verificar y procesar datos del GPS
  if (gps.charsProcessed() < 10 && millis() > 5000) {
    Serial.println(F("No GPS data received: check wiring"));
    //tone(BUZZER_PIN, 1000);
    //delay(1000);
    //noTone(BUZZER_PIN);
    //delay(1000);
  }

  // Procesar y enviar datos del oxímetro
  procesarOximetro();
  // Asegurar que los datos del GPS se procesan adecuadamente
  smartDelay(1000);
}

void enviarDatosGPS() {
  if (gps.location.isValid()) {
    espSerial.println(longitud, 6);
    espSerial.println(latitud, 6);
    espSerial.println(altitud, 6);
    espSerial.println(velocidad, 6); 
    //analizar como agregar fecha y hora
  } else {
    Serial.println(F("No valid GPS data"));
  }
}

void procesarOximetro() {
  int valorSensorInfrarrojo = analogRead(sensorInfrarrojo);
  int valorSensorRojo = analogRead(sensorRojo);

  intensidad_infrarrojo = valorSensorInfrarrojo;
  intensidad_rojo = valorSensorRojo;

  float spo2total = ((float)intensidad_rojo + (float)intensidad_infrarrojo);
  float spo2 = ((float)intensidad_infrarrojo / spo2total) * 100.0 + 47.0;

  total = total - readings[indice];
  readings[indice] = spo2;
  total = total + readings[indice];
  indice = (indice + 1) % numReadings;
  average = total / numReadings;

  Serial.print("SpO2: ");
  Serial.print(spo2);
  Serial.print(", Promedio: ");
  Serial.println(average);

  espSerial.print("SpO2: ");
  espSerial.print(spo2);
  espSerial.print(", Promedio: ");
  espSerial.println(average);
}

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}
