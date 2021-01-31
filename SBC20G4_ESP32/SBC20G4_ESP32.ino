#include "WiFi.h"       
#include <ThingsBoard.h>    
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Max72xxPanel.h>    //  max7219 library

// Helper macro to calculate array size
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

// WiFi access point
#define WIFI_AP_NAME        "SBC"
// WiFi password
#define WIFI_PASSWORD       "sbc$2020"

// See https://thingsboard.io/docs/getting-started-guides/helloworld/ 
// to understand how to obtain an access token
#define TOKEN               "KRgn1MQmVZgKjzGdq0eo"
// ThingsBoard server instance.
#define THINGSBOARD_SERVER  "iot.etsisi.upm.es"

// Baud rate for debug serial
#define SERIAL_DEBUG_BAUD    115200

// Initialize ThingsBoard client
WiFiClient espClient;
// Initialize ThingsBoard instance
ThingsBoard tb(espClient);
// the Wifi radio's status
int status = WL_IDLE_STATUS;
//----------------------------------------------------------LED MATRIX
#define timeScreen 10
unsigned long nowScreen = millis();
unsigned long lastScreen = 0;
int pinCS = 16; // Attach CS to this pin, DIN to MOSI and CLK to SCK 
Max72xxPanel matrix = Max72xxPanel(pinCS, 1, 1);
boolean start = false;
TaskHandle_t Matrix_LED; //Declaracion de una Tarea para procesamiento paralelo
//----------------------------------------------------------
//----------------------------------------------------------SVM30 CO2 TEMP HUMEDAD
#define DELAY 10
/* define driver debug
 * false : no messages
 * true : display driver debug messages
 */
#define DEBUG false
void read_values();
#include <svm30.h>
SVM30 svm; // create instance
//----------------------------------------------------------

//----------------------------------------------------------AIR QUALITY
#include "Air_Quality_Sensor.h"
AirQualitySensor sensor(35);
const int RedLed = 14;
int calidadAire = 0;
//----------------------------------------------------------

//----------------------------------------------------------LOUD NOISES
#define timeSeconds 3

// Set GPIOs for LED and PIR Motion Sensor
const int GreenLed = 33;
const int Sound = 34;
//const int AnalogSound = ???;//ADC2_CH2
int val = 0;
//----------------------------------------------------------

//----------------------------------------------------------DUST COUGH
const int dust = 17;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 3000;//sampe 30s ;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;
//----------------------------------------------------------

//----------------------------------------------------------MOTION SENSOR
const int BlueLed = 15;
const int motionSensor = 12;

// Timer: Auxiliary variables
unsigned long now = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;
boolean onoff = false;
int contador = 0;
boolean presencia = false;

// Checks if motion was detected, sets LED HIGH and starts a timer
void IRAM_ATTR detectsMovement() 
{
    Serial.println("MOTION DETECTED!!!\n");
    digitalWrite(BlueLed, HIGH);
    onoff = true;
    presencia = true;
    startTimer = true;
    lastTrigger = millis();
}
//----------------------------------------------------------
// Main application loop delay
int quant = 20;


// Setup an application
void setup() {//-----------------------------------------------------------------SETUP---------------------------------------------
  // Initialize serial for debugging
  Serial.begin(SERIAL_DEBUG_BAUD);
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  InitWiFi();

  // Pinconfig
  Serial.begin(115200);
  //----------------------------------------------------------LED MATRIX
  matrix.setIntensity(3); // Set brightness between 0 and 15
  xTaskCreatePinnedToCore(
                    func_Matrix_LED,  // Task function.
                    "LED_Matrix",     // name of task. 
                    10000,            // Stack size of task 
                    NULL,             // parameter of the task 
                    1,                // priority of the task 
                    &Matrix_LED,      // Task handle to keep track of created task 
                    0);               // pin task to core 0               
  delay(500);
  //----------------------------------------------------------
  
  while (!Serial);
  delay(3000);
  int powerUP = 30;
  Serial.println(String(powerUP)+" Seconds cooldown so that all the sensores");
  Serial.println("are powered up before monitoring begins.");
  delay(1000);
  for(int i = powerUP; i>0; i--){
    Serial.print(i); Serial.print(", ");
    delay(1000);
  }
  Serial.println("0");
  delay(1000);
  
  //----------------------------------------------------------AIR QUALITY
  if (sensor.init()) {
    Serial.println("Air Quality Sensor READY.");
  } else {
    Serial.println("Air Quality Sensor ERROR! Restarting...");
    //ESP.restart();
  }
  
  pinMode(RedLed, OUTPUT);           // define RED LED as output interface
  digitalWrite(RedLed, LOW);
  
  delay(1000);
  //----------------------------------------------------------
  
  //----------------------------------------------------------SVM30
  // enable debug messages
  svm.EnableDebugging(DEBUG);
  svm.begin();
  
  // try to detect SVM30 sensors
  if (svm.probe() == false) {
      Serial.println((char *) "Could not detect SVM30 sensors"); 
      ESP.restart();
  }
  else Serial.println(F("SVM30 READY"));

  //----------------------------------------------------------
  
  //----------------------------------------------------------LOUD NOISES
  
  pinMode (Sound, INPUT) ;  // define Sound sensor as input interface
  
  pinMode (GreenLed, OUTPUT) ;       // define GREEN LED as output interface
  digitalWrite(GreenLed, LOW);
  
  Serial.println("Loudness Sensor READY.");
  delay(1000);
  //----------------------------------------------------------
  
  //----------------------------------------------------------DUST COUGH
  //Serial.begin(115200);
  pinMode(dust,INPUT);
  starttime = millis();//get the current time;
  
  Serial.println("Dust Sensor READY.");
  delay(1000);
  //----------------------------------------------------------
  
  //----------------------------------------------------------MOTION SENSOR
  pinMode(motionSensor, INPUT);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);

  // Set LED to LOW
  pinMode(BlueLed, OUTPUT);           // define BLUE LED as output interface
  digitalWrite(BlueLed, LOW);
  
  Serial.println("Motion Sensor READY.");
  delay(1000);
  //----------------------------------------------------------
  Serial.println("Monitoring STARTO.");
}

//----------------------------------------------------------------LOOP--------------------------------------
void loop() {
  delay(1000);
  
  // Reconnect to WiFi, if needed
  if (WiFi.status() != WL_CONNECTED) {
    reconnect();
    return;
  }

  // Reconnect to ThingsBoard, if needed
  if (!tb.connected()) {
//    subscribed = false;

    // Connect to the ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
      Serial.println("Failed to connect");
      return;
    }
  }
    
  duration = pulseIn(dust, LOW);
  lowpulseoccupancy = lowpulseoccupancy+duration;

  if(start == false) start = true; //control when matrix task starts

  if(presencia){
  
  if ((millis()-starttime) > sampletime_ms) 
      { //if the sample time == 3s
        Serial.println("Sending data...");

  //----------------------------------------------------------AIR QUALITY
    
    int quality = sensor.slope();
    if(sensor.getValue() < 1200){
      Serial.println("AIR QUALITY SENSOR");
      tb.sendTelemetryFloat("Air Quality", sensor.getValue());
      
      Serial.print("Sensor value: ");
      Serial.println(sensor.getValue());
  
      if (quality == AirQualitySensor::FORCE_SIGNAL) {
          Serial.println("High pollution! Force signal active.");
          digitalWrite(RedLed, HIGH);
          calidadAire = 3;
      } else if (quality == AirQualitySensor::HIGH_POLLUTION) {
          Serial.println("High pollution!");
          digitalWrite(RedLed, HIGH);
          calidadAire = 2;
      } else if (quality == AirQualitySensor::LOW_POLLUTION) {
          Serial.println("Low pollution!");
          digitalWrite(RedLed, LOW);
          calidadAire = 1;
      } else if (quality == AirQualitySensor::FRESH_AIR) {
          Serial.println("Fresh air.");
          digitalWrite(RedLed, LOW);
          calidadAire = 0;
      }
      
      Serial.println();
    }
    //----------------------------------------------------------
    //----------------------------------------------------------SVM30
    read_values();
    //----------------------------------------------------------
    //----------------------------------------------------------LOUD NOISES
    val = analogRead(Sound);
    if(val < 4096){ //Control de errores
    if(val > 0){    //cuando el sensor no funciona correctamente
      Serial.println("LOUD NOISES SENSOR");
      tb.sendTelemetryFloat("Loudness", val);
      
      if (val > 500) //Valor arbitrario
      {
        digitalWrite (GreenLed, HIGH);
        Serial.println("LOUD NOISES!!!");
        Serial.println(val);
      }
      else
      {
        digitalWrite (GreenLed, LOW);
        Serial.println("THE SOUND OF SILENCE");
        Serial.println(val);
      }
      Serial.println();
    }
    }
    tb.sendTelemetryFloat("Loudness", val);
    //----------------------------------------------------------
    //----------------------------------------------------------DUST COUGH

        ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
        concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve

        if(concentration > 0.63){ // prevencion de envio de valores de error
          Serial.println("DUST SENSOR");
          Serial.print(lowpulseoccupancy);
          tb.sendTelemetryFloat("Dust Low Pulse Occupancy", lowpulseoccupancy);
          Serial.print(",");
          Serial.print(ratio);
          tb.sendTelemetryFloat("Dust Ratio", ratio);
          Serial.print(",");
          Serial.println(concentration);
          tb.sendTelemetryFloat("Dust Concentration", concentration);
          lowpulseoccupancy = 0;
          starttime = millis();
          Serial.println();
        }
    }
    
  //----------------------------------------------------------
  //----------------------------------------------------------MOTION SENSOR
    // Current time
    now = millis();
    // Turn off the LED after the number of seconds defined in the timeSeconds variable
    if(startTimer && (now - lastTrigger > (timeSeconds*3000))) { 
      Serial.println("Motion stopped...");
      digitalWrite(BlueLed, LOW);
      startTimer = false;
      onoff = false;
      if(startTimer && (now - lastTrigger > (60*1000*30)))  //media hora
        presencia = false;
    }
    tb.sendTelemetryFloat("Motion Sensor", onoff);
  //----------------------------------------------------------
  // Process messages
  tb.loop();
}
}

void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

void reconnect() {
  // Loop until we're reconnected
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
  }
}

//---------------------------------------------------SVM30
struct svm_values v;
void read_values() {
  
  if (! svm.GetValues(&v))
      Serial.print((char *) "Error during reading SVM30 values");
else{
  Serial.print(F("CO2 equivalent : "));
  tb.sendTelemetryFloat("CO2", (float)v.CO2eq);
  Serial.print(v.CO2eq);

  Serial.print(F(", Humidity : "));
  tb.sendTelemetryFloat("Humedad", (float)v.humidity/1000);
  Serial.print((float) v.humidity/1000);
  
  Serial.print(F(", temperature : "));
  tb.sendTelemetryFloat("Temperatura", (float)v.temperature/1000);
  Serial.println((float) v.temperature/1000);
  Serial.println();
}
}

//----------------------------------------------------------LED MATRIX
void func_Matrix_LED( void * pvParameters ){ //CAMBIAR VALORES---------------------------------------
  Serial.print("/nMatrix LEDs en el core ");
  Serial.println(xPortGetCoreID());
  while(start == false) delay(1000);
  boolean done = true;
  String stringo = "";
  for(;;){
    if(presencia == true){
    nowScreen = millis();
    if(done) {
      done = false;
    lastScreen = millis();
    
    temperatureMatrix();
    
    stringo = "Humedad:" + String(v.humidity/1000) + "% ";
    StringWarningsMatrix(stringo);

    if(calidadAire != 0){
      switch (calidadAire) {
        case 1:
          exclamationBlinkMatrix(1);
          stringo = "Calidad de aire baja ";
          StringWarningsMatrix(stringo);
          break;
        case 2:
          exclamationBlinkMatrix(2);
          stringo = "Calidad de aire muy baja ";
          StringWarningsMatrix(stringo);
          StringWarningsMatrix("Abrir ventanas ");
          break;
        case 3:
          exclamationBlinkMatrix(3);
          stringo = "Calidad del aire peligrosa ";
          StringWarningsMatrix(stringo);
          while(calidadAire = 3){
            StringWarningsMatrix("Evacuar aula ");
            stopSignMatrix();
          }
          break;
        default:
        Serial.println("Error Air Quality Matrix");
        break;
      }
    }
    
    if(v.CO2eq > 50){
      exclamationBlinkMatrix(3);
      stringo = "Niveles de CO2 altos ";
      StringWarningsMatrix(stringo);
      StringWarningsMatrix("Abrir ventanas ");
    }
    if(val > 500){ //val variable del sonido
      exclamationBlinkMatrix(1);
      stringo = "Mucho ruido ";
      StringWarningsMatrix(stringo);
    }
    if(concentration > 18000){
      stringo = "Concentracion de polvo alta ";
      StringWarningsMatrix(stringo);
    }
    done = true;
    }
    }
    delay(1000);
  } 
}

void stopSignMatrix(){
  matrix.fillScreen(0);  
  //matrix.drawLine(x1, y1, x2, y2, 1); Draws a line between two coordinates
  //Draws Stop sign (/)
  matrix.drawLine(1, 1, 6, 6, 15);
  matrix.drawLine(0, 2, 0, 5, 15);
  matrix.drawLine(7, 2, 7, 5, 15);
  matrix.drawLine(2, 0, 5, 0, 15);
  matrix.drawLine(2, 7, 5, 7, 15);
  matrix.drawPixel(6, 1, 2);
  matrix.drawPixel(1, 6, 2);
  matrix.write();
  delay(1000);
}

void exclamationBlinkMatrix(int times){
  for(int i=0; i < times; i++){
    matrix.fillScreen(0);  
    matrix.drawRect(3,0,2,5,15); //matrix.drawRect(x,y,s1,s2,15); x,y position of corner -- s1, s2 size of side -- intensity 1 to 15
    matrix.drawRect(3,6,2,2,15);
    matrix.write();
    delay(500);

    matrix.fillScreen(0);  
    matrix.write();
    delay(500);
  }
}

void temperatureMatrix(){
  int temp = v.temperature/1000;
  String my_string = String(temp) + "    ";   // This text will be displayed, a letter a time
  //Draw scrolling text
  int spacer = 1;                            // This will scroll the string
  int width = 5 + spacer;                    // The font width is 5 pixels
  for ( int i = 0 ; i < width * my_string.length() + width - 1 - spacer; i++ ) {
    matrix.fillScreen(0);
    int letter = i / width;
    int x = (matrix.width() - 1) - i % width;
    int y = ((matrix.height() - 8) / 2); 

    while ( x + width - spacer >= 0 && letter >= 0 ) {
      if ( letter < my_string.length() ) {
        matrix.drawChar(x, y, my_string[letter], 1, 0, 1);
      }
      letter--;
      x -= width;
    }
    matrix.drawRect(((width+spacer)*(String(temp).length()+1))-i-1,y,3,3,1); //Simbolo de grados no funciona
    matrix.drawChar((3+(width+spacer)*(String(temp).length()+1))-i, y, 'C', 7, 0, 1); //Asi q se dibuja a mano
    matrix.write(); 
    delay(100);
  }
}

void StringWarningsMatrix(String my_string){
  int spacer = 1;         // This will scroll the string
  int width = 5 + spacer; // The font width is 5 pixels
  for ( int i = 0 ; i < width * my_string.length() + width - 1 - spacer; i++ ) {
    matrix.fillScreen(0);
    int letter = i / width;
    int x = (matrix.width() - 1) - i % width;
    int y = ((matrix.height() - 8) / 2); //center the text vertically

    while ( x + width - spacer >= 0 && letter >= 0 ) {
      if ( letter < my_string.length() ) {
        matrix.drawChar(x, y, my_string[letter], 1, 0, 1);
      }
      letter--;
      x -= width;
    }
    matrix.write();
    delay(100);
  }
}
