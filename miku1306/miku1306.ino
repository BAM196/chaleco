#include <TinyGPSPlus.h>
#include <PulseSensorPlayground.h>
#include <SoftwareSerial.h>

#define BUZZER_PIN 9

String mensaje;

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
unsigned int intensidad_infrarrojo; //(oxihemoglobina)
unsigned int intensidad_rojo;       //(hemoglobina)
const int numReadings = 100;
int readings[numReadings];
int indice = 0;
int total = 0; // Total
float promedio = 0; // Promedio

//gps
  String latitud;
  String longitud;
  String velocidad;
  int year, mes, dia, hora, min, sec;
//Pulso
  int BPM = NULL;


void setup() {
  Serial.begin(9600);       // Monitor serial
  espSerial.begin(115200);  // Comunicación con ESP32
  gpsSerial.begin(GPSBaud); // Comunicación con GPS

  pulseSensor.analogInput(PulseWire);
  pulseSensor.setThreshold(Threshold);
  pulseSensor.begin();

  pinMode(BUZZER_PIN, OUTPUT);

 // Serial.print(F("Testing TinyGPSPlus library v. "));
 // Serial.println(TinyGPSPlus::libraryVersion());

  // Inicializar el arreglo de lecturas del oxímetro
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;
}
void loop() {
  MedirPulso();
  MedirGps();
  MedirO2();
  EnviarDatos();
}

void MedirPulso(){
//if (pulseSensor.sawStartOfBeat()) {             //ocurrio una pulsación? //se puede eliminar
BPM = pulseSensor.getBeatsPerMinute();     // función interna del sensor que entrega BPM como un int 
 //Serial.print("BPM: ");                       //BPM al monitor serial
 //Serial.println(BPM);                        //BPM al monitor serial
  sumaData(String(BPM));
//}
/*else{
  BPM = NULL; 
}*/
}
void MedirGps(){
  //newInt(gps.satellites.value(), gps.satellites.isValid(), 5); //satelites
  latitud = newFloat(gps.location.lat(), gps.location.isValid(), 11, 6);//latitud
  longitud = newFloat(gps.location.lng(), gps.location.isValid(), 12, 6);//longitud
  printDateTime(gps.date, gps.time);//fecha actual
  velocidad = newFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);//velocidad

  sumaData(String(latitud));
  sumaData(String(longitud));
  sumaData(String(velocidad));
  //sumaData();
}
void MedirO2(){

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
  promedio = total / numReadings;

  //Serial.print("SpO2: ");
  //Serial.print(spo2);
  sumaData(String(spo2));
  //Serial.print("Promedio: ");
  ///Serial.println(promedio);
  
  // sumaData(String(promedio));
}
void EnviarDatos(){
/*espSerial.println(BPM);
  espSerial.println(latitud);
  espSerial.println(longitud);
  espSerial.println(velocidad);
  espSerial.println(year);
  espSerial.println(mes);
  espSerial.println(dia);
  espSerial.println(hora);
  espSerial.println(min);
  espSerial.println(sec);*/
  espSerial.println(mensaje);
  Serial.println(mensaje);
  Serial.println("Datos enviados");
  mensaje = "";
 delay(1000);
 //enviar todos los datos en string.;.;. 
 //definir separador  y final
}
//funciones para imprimir valores incluidas en la libreria tinygps
static String newInt(unsigned long val, bool valid, int len)// imprimirt int (integrada de la libreria)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
//  Serial.print(sz);
  return(sz);
  smartDelay(0);
}


static String newFloat(float val, bool valid, int len, int prec)// imprimirt float (integrada de la libreria)
{
  String respuesta;
  if (!valid)
  {
  respuesta = "*****************";
    return (respuesta);}
  else
  {
  //Serial.print(val, prec);
  //int vi = abs((int)val);
  //int flen = prec + (val < 0.0 ? 2 : 1); // . and -
  //flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
  respuesta = String(val,prec);
  return(respuesta);
  smartDelay(0);
}}

static String printDateTime(TinyGPSDate &d, TinyGPSTime &t) // imprimirt fecha y tiempo (integrada de la libreria)
{
  if (!d.isValid())
  {
   // Serial.print(F("********** "));
  String respuesta = "********** ";
  return (respuesta);
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
  //Serial.print(sz);
  return(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  //printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void smartDelay(unsigned long ms) // asigna delay (integrada de la libreria)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}

static void sumaData(String data) {
  if (mensaje.length()>0) {
    mensaje += ",";
  }
  mensaje += data;
}