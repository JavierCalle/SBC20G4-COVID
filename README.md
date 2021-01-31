# SBC20G4-COVID
Sistema de control de calidad de aire para el control de vectores de contagio por COVID19 en las aulas.

Proyecto realizado con una placa ESP32 DevKit V2 en el Arduino IDE y conectado a Thingsboard.

------------------------------------

Debido a los efectos que produce el COVID-19 dañando la presencialidad de las clases como efecto colateral, en la ETSISI se pretende asegurar una asistencia segura sin contagios por COVID-19 para los usuarios de las aulas de prácticas del campus, pudiendo asistir presencialmente y de forma controlada a las prácticas presenciales de las distintas asignaturas. 

Dos de los grandes problemas para frenar la transmisión del virus son el distanciamiento social y la ventilación de entornos cerrados, estos junto a una mala ventilación están demostrados como uno de los principales focos de contagio. 

Con este proyecto se pretende implementar un sistema que detecte presencia y que indique mediante una matrix de LEDs y señales lumínicas si es posible acceder o no a la sala, mediante sensores y medidores de calidad de aire (CO2, temperatura y humedad), saber si debemos ventilar, y saber si está demasiado ruido.

------------------------------------

Este dispositorio dispone de las librerias utilizadas y una imagen de la disposición de los pines utilizados por los sensores en la placa ESP32.

Está actualmente definido con el nombre (SSID) y la password de la red proporcionado por los profesores Universidad para la realización del proyecto de la asignatura (Sistemas Basados en Computador). Además se está enviando datos al Thingsboard con una cuenta de la Universidad.

------------------------------------

Librerias usadas:

Downloaded from Arduino Library Manager: 
ArduinoHttpClient by Arduino V0.3.2 
Adafruit BusIO by Adafruit V1.7.1 
Adafruit GFX Library by Adafruit V1.10.4 
Arduino Json by Benoit Blanchon V6.9.1 
Grove - Air Quality Sensor by Seeed Studio V1.0.1 
PubSubClient by Nick O'Leary V2.7.0 
ThingsBoard by ThingsBoard Team V0.2.0 

Downloaded from GitHub 
Max72xxPanel by markruys https://github.com/markruys/arduino-Max72xxPanel 
SVM30 by paulvha V1.3 https://github.com/paulvha/svm30 
