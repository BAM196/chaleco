#include <HardwareSerial.h>

HardwareSerial SerialPort(1); // Instanciamos un objeto de clase HardwareSerial que nos permite habilitar un canal de comunicación serial UART1, RX: GPIO4, TX: GPIO2 (Pin D2 es transmisor y Pin D4 es receptor)
String receivedMessage;

#define RXD2 16
#define TXD2 17


void setup() {

  Serial.begin(9600);
  delay(1000);
  Serial.println("Iniciando comunicación serial...");
  delay(1000);
  SerialPort.begin(115200, SERIAL_8N1, RXD2, TXD2); // Velocidad en baudios: 9600, Configuración: SERIAL_8N1, RX pin: GPIO4, TX pin: GPIO2
  delay(1000);
  Serial.println("UART1 iniciado a 115200 baudios");
  delay(1000);
}
void loop() {
  Serial.println("en loop");
  delay(1000);
  while (SerialPort.available() > 0) {
    char receivedChar = SerialPort.read(); //Recordar que por cada trama se recibe un caracter
    if (receivedChar == '\n') {
      Serial.println(receivedMessage);  //Imprimimos el mensaje si el caracter leído es '\n' (un salto de línea). En este caso, Arduino está mandando mensajes con un salto de línea al final de cada uno de ellos. Este salto de línea nos permite identificar cuando acaba un mensaje y comienza otro.
      receivedMessage = "";  // Liberamos el contenido de la variable para preparar una nueva iteración de loop()
    } else {
      receivedMessage += receivedChar;  // Concatenamos el caracter recibido al string que almacena el mensaje si el caracter recibido es distinto a '\n'
    }
  }
}