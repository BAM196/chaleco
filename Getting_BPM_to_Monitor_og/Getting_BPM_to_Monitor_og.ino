#include <TinyGPSPlus.h>              //          //gps library
#include <PulseSensorPlayground.h>               // Includes the PulseSensorPlayground Library.   
#include <SoftwareSerial.h>                     //para el esp32
#define BUZZER_PIN 9                            //buzzer pin
SoftwareSerial espSerial(4, 5);                 //declara los pines 2 y 3 como //RX,TX
const int PulseWire = 0;                        //SENSOR DE PULSO EN EL PIN A0
//const int LED = LED_BUILTIN;                    // The on-board Arduino LED, close to PIN 13. //led en el arduino board
int Threshold = 550;                            // Determine which Signal to "count as a beat" and which to ignore.
                                               // Use the "Gettting Started Project" to fine-tune Threshold Value beyond default setting.
                                              // Otherwise leave the default "550" value. 
PulseSensorPlayground pulseSensor;            // Instancia de PulseSensorPlayground

static const int RXPin = 3, TXPin = 2; //     //pines para gps
static const uint32_t GPSBaud = 9600;//       //baund para el gps

TinyGPSPlus gps; // // gps
SoftwareSerial gpsSerial(RXPin, TXPin);
//variables oximetrO
float spo2=0;   //%spo2
float spo2total=0; //sumatoria de ambas señales 
//Sensores Fototransistores//
int sensor=A1;    //Fototransistor 2pin
int valorSensor;
int sensor2=A2;  //Fototransistor 3pin  
int valorSensor2;
unsigned int   intensidad_infrarrojo;  // intensidad reflejada HbO2 (oxihemoglobina)
unsigned int   intensidad_rojo;        //intensidad refleada   Hb (hemoglobina)
const int numReadings = 100;
int readings[numReadings];  
int indice = 0;  
int total = 0; // Total 
float average = 0; // Promedio 

// The serial connection to the GPS device
//SoftwareSerial ss(RXPin, TXPin);  //       ////gps

void setup() {

  bool Server = false; //variable para ver el estado de la comunicación con el servidor
 

  Serial.begin(9600);                         // For Serial Monitor
  espSerial.begin(115200);                    //para el esp32
  gpsSerial.begin(GPSBaud); //                     //
  pulseSensor.analogInput(PulseWire);   
  //pulseSensor.blinkOnPulse(LED);              //auto-magically blink Arduino's LED with heartbeat.
  pulseSensor.setThreshold(Threshold);
  pulseSensor.begin();                        //inicializar el sensor

  pinMode(BUZZER_PIN, OUTPUT); //pin de buzzer
  
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion()); ///////gps 
//OXIMETRO****************************
for (int thisReading = 0; thisReading < numReadings; thisReading++) 
readings[thisReading] = 0; 
}
void loop() {
    
  //SENSOR DE PULSO*************************************
if (pulseSensor.sawStartOfBeat()) {             //ocurrio una pulsación?
int BPM = pulseSensor.getBeatsPerMinute();     // función interna del sensor que entrega BPM como un int 
 Serial.print("BPM: ");                       //BPM al monitor serial
 Serial.println(BPM);                        //BPM al monitor serial
espSerial.println(BPM);                     //BPM al esp32
}
  delay(200);
//GPS *******************************************************************
  if (millis() > 5000 && gps.charsProcessed() < 10){//gps si no hay señal imprime
    Serial.println(F("No GPS data received: check wiring"));
    
    tone(BUZZER_PIN, 1000);
    delay(1000);
    noTone(BUZZER_PIN);
    delay(1000);

    } 
            printInt(gps.satellites.value(), gps.satellites.isValid(), 5); //satelites
            
            printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);//?
            
            printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);//latitud
            //espSerial.println(gps.location.lat());
            printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);//longitud
            //espSerial.println(gps.location.lng);
            printInt(gps.location.age(), gps.location.isValid(), 5);//tiempo desde la ultima ubicación
            //////////////////////////////////////
            printDateTime(gps.date, gps.time);//fecha actual
            //espSerial.println(gps.date);
            printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);// altitud
            //espSerial.println(gps.altitude.meters());
            printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);//curso
            //espSerial.println(gps.course.deg);
            printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);//velocidad
            //espSerial.println(gps.speed.kmph);
            printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);//cardinalidad N S E O facing direction
            //espSerial.println(gps.course.deg);
    smartDelay(1000);
//OXIMETRO************************************************************
 //lectura de Fototransistores
valorSensor=analogRead(sensor); //lee el fotoreceptor 1
intensidad_infrarrojo=valorSensor;// asigna el valor ala la var infrarroja

valorSensor2=analogRead(sensor2);//lee el fotoreceptor 2 (3pines)
intensidad_rojo=valorSensor2;//asigna el valor  ala var rojo

 // calculo de % de saturacion de Oxigeno basada en la Ley de Beer-Lambert
 spo2total=((float)intensidad_rojo+(float)intensidad_infrarrojo);
 spo2=((float)intensidad_infrarrojo/spo2total);
 spo2=100.0*spo2;
 spo2=spo2+47;
total= total - readings[indice]; 
readings[indice] = spo2;  
total= total + readings[indice];  
indice = indice + 1;  
if (indice >= numReadings) 
indice = 0;  
average = total / numReadings;  
Serial.print(spo2); 
Serial.print(","); 
Serial.println(average, DEC); 
delayMicroseconds(1000); 

}//loop

//funciones para imprimir valores incluidas en la libreria tinygps
static void printInt(unsigned long val, bool valid, int len)// imprimirt int (integrada de la libreria)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printFloat(float val, bool valid, int len, int prec)// imprimirt float (integrada de la libreria)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t) // imprimirt fecha y tiempo (integrada de la libreria)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
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

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len) // imprimirt string (integrada de la libreria)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
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

  
