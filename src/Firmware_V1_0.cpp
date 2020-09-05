/*****************************
 * SELECCIóN DEL DISPOSITIVO *
 *****************************/

// Este código fuente vale para cualquier dispositivo basado en el hardware V1.0 del Control de Tiras de Led.
// Para facilitar la recompilación del dispositivo (des)comentar el define del dispositivo apropiado.
// OJO. MUY IMPORTANTE. SOLO PUEDE ESTAR DEFINIDO UN SOLO DEFINE EN EL GRUPO. EL RESTO DEBE ESTAR COMENTADO.

#define LEDS_DEL_ACUARIO
//#define LEDS_DE_AMBIENTE

#ifdef LEDS_DEL_ACUARIO
#define DEVICE_ID "NODEMCU-20181021"	// Para que sea único, pongo en el nombre la fecha de fabricación.
#define USE_DHT22						// Indica si se utilizará el sensor de temperatura y humedad. 
#define USE_LDR							// Indica si se utilizará el disparo por LDR. 
#define NUM_OF_LEDS	74					// Número de leds de tiene la tira.
#endif

#ifdef LEDS_DE_AMBIENTE
#define DEVICE_ID "NODEMCU-20190622"	// Para que sea único, pongo en el nombre la fecha de fabricación.
#define USE_DHT22						// Indica si se utilizará el sensor de temperatura y humedad. 
#define USE_LDR							// Indica si se utilizará el disparo por LDR. 
#define NUM_OF_LEDS	37					// Número de leds de tiene la tira.
#endif



/************************************
 * FIN DE SELECCIÓN DEL DISPOSITIVO *
 ************************************/

/**********************************
 * CONFIGURACIÓN DE LA DEPURACIÓN *
 **********************************/

// Si se habilita el LEVEL_2, es obligatorio habilitar también el LEVEL_1
// #define _TOTE_DEBUG_LEVEL_1_ON_   // Descomentar para mostrar info de debug de primer nivel por serial
// #define _TOTE_DEBUG_LEVEL_2_ON_   // Descomentar para mostrar info de debug de segundo nivel por serial

// Macros para facilitar la salida de información de nivel 1 por el Serial.
#ifdef _TOTE_DEBUG_LEVEL_1_ON_
#define _TOTE_DEBUG_LEVEL_1_(type, text) Serial.print("("); Serial.print(millis()); Serial.print(" millis) [LEVEL 1]"); Serial.print(" ["); Serial.print(type); Serial.print("] "); Serial.println(text);
#define _TOTE_DEBUG_LEVEL_1_VALUE_(type, text, value) Serial.print("("); Serial.print(millis()); Serial.print(" millis) [LEVEL 1]"); Serial.print(" ["); Serial.print(type); Serial.print("] "); Serial.print(text); Serial.println(value);
#else
#define _TOTE_DEBUG_LEVEL_1_(type, text) void();
#define _TOTE_DEBUG_LEVEL_1_VALUE_(type, text, value) void();
#endif

// Macros para facilitar la salida de información de nivel 1 por el Serial.
#ifdef _TOTE_DEBUG_LEVEL_2_ON_
#define _TOTE_DEBUG_LEVEL_2_(type, text) Serial.print("("); Serial.print(millis()); Serial.print(" millis) [LEVEL 2]"); Serial.print(" ["); Serial.print(type); Serial.print("] "); Serial.println(text);
#define _TOTE_DEBUG_LEVEL_2_VALUE_(type, text, value) Serial.print("("); Serial.print(millis()); Serial.print(" millis) [LEVEL 2]"); Serial.print(" ["); Serial.print(type); Serial.print("] "); Serial.print(text); Serial.println(value);
#else
#define _TOTE_DEBUG_LEVEL_2_(type, text) void();
#define _TOTE_DEBUG_LEVEL_2_VALUE_(type, text, value) void();
#endif


/********************************************
 * FIN DE LA CONFIGURACIÓN DE LA DEPURACIÓN *
 ********************************************/



/************************************************
 * CONFIGURACIÓN DEL PINOUT DEL NODEMCY MINI D1 *
 ************************************************/

/*

	PINOUT DEL NodeMCU Mini D1

	Los includes de ESP8266Wifi.h y similares redefinen las constantes de los pines de Arduino para que coincidan con el pinout
	de las placas NodeMCU, por lo que se pueden usar en el código sin mayor problema.

	Ciertos pines de la placa están conectados a funciones activas del ESP8266, por lo que no se deben usar como pines de E/S, por ejemplo
	TX, RX, etc.

	Esta es la equivalencia entre los pines de la placa NodeMCU (Mimi D1) y los pines del ESP8266.

	Pin		Function						ESP - 8266 Pin
	---		------------------------------  --------------

	TX		TXD								TXD
	RX		RXD								RXD
	A0		Analog input, max 3.3V input	A0
	D0		IO								GPIO16
	D1		IO, SCL							GPIO5
	D2		IO, SDA							GPIO4
	D3		IO, 10k Pull - up				GPIO0
	D4		IO, 10k pull - up, BUILTIN_LED	GPIO2
	D5		IO, SCK							GPIO14
	D6		IO, MISO						GPIO12
	D7		IO, MOSI						GPIO13
	D8		IO, 10k pull - down, SS			GPIO15
	G		Ground							GND
	5V		5V								5V
	3V3		3.3V							3.3V
	RST		Reset							RST



	Placa NodeMCU .

	IMPORTANTE: Evito usar el D3 (GPIO0) porque tiene significado especial para el arranque del dispositivo.
	IMPORTANTE: Evito usar el D4 (GPIO2 Built-in led), porque si lo uso me da problemas la programación del dispositivo y tengo que sacar en nodemcu de la placa.

	D0  (GPIO16) MASTER_SWITCH_PIN. Se utiliza para (des)habilitar el MOSFET que enciende la tira de leds. Usa lógica inversa.
	D1  (GPIO5)  LED_LDR_PIN. Estará conectado a un led que indica el estado de operación del LDR.
	D2  (GPIO4)  AP_CONF_PIN. Se utiliza para entrar en modo de configuración del AP. Este GPIO se usa para I2C, pero en este proyecto no se usa.
	D5  (GPIO14) LED_WIFI_PIN. Si está encendido indica que se está conectado al AP.
	D6  (GPIO12) DHT22_PIN. Pin para leer el sensor de humedad/temp.
	D7  (GPIO13) STRIP_DATA_PIN. Pin para mandar información serie a la tira de leds.

	A0   Entrada analógica (Máx 3.3V) Se usará para leer el nivel de tensión de la LDR.

*/


// Configuración de los pines de NodeMCU
#define MASTER_SWITCH_PIN D0	  // Gestiona el Gate del Mosfet.

#ifdef USE_LDR
#define LED_LDR_PIN       D1      // Pin para el led verde OK.
#endif

#define AP_CONF_PIN       D2      // Para entrar en modo de selección del AP.

#define LED_WIFI_PIN      D5      // Pin para el led que indica conexión con el AP.		

#define STRIP_DATA_PIN    D7	  // Pin para mandar información serie a la tira de leds.

#ifdef USE_DHT22
#define DHT22_PIN         D8      // Pin para leer el sensor de humedad/temperatura.
#endif

#ifdef USE_LDR
#define LDR_PIN           A0	  // Pin para leer valor de la LDR.
#endif


// Damos un margen de 2 segundos para que los leds parpadeen.
#define CHECKLED_WINDOW 2000UL

// Damos un margen de 10 segundos para pulsar el botón que lanza el portal de configuración.
#define CONFIG_WINDOW 10000UL  



/**********************************************************
 * FIN DE LA CONFIGURACIÓN DEL PINOUT DEL NODEMCY MINI D1 *
 **********************************************************/

 // Librerías de Arduino que usará.
#include <WiFiClientSecure.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>            // Servidor DNS que se usa para redirigir todas las request al portal de configuración.
#include <ESP8266WebServer.h>     // Servidor web local que muestra el portal de configuración.
#include <EEPROM.h>
#include <time.h>

// Librerías clonadas desde Git. 
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Librerias_desde_Git\WiFiManager\WiFiManager.cpp"
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Librerias_desde_Git\pubsubclient\src\PubSubClient.cpp"
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Librerias_desde_Git\Time\Time.cpp"
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Librerias_desde_Git\DHTesp\DHTesp.cpp"



// Librerías propias. Se encuentran disponibles en https://github.com/MisLibrerias
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteDebouncedBtn\ToteDebouncedBtn.cpp"
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteBlinkOutputLed\ToteBlinkOutputLed.cpp"
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteAsyncDelay\ToteAsyncDelay.cpp"
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteNeo\ToteNeoV3.0.cpp"
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteAnalogSensor\ToteAnalogSensor.cpp"


// IMPORTANTE. VER INFORMACIÓN SOBRE EL WDT en la clase ToteESPMillisDelay
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteESPMillisDelay\ToteESPMillisDelay.cpp"


/***********************************************
  * PROTOTIPOS DE FUNCIONES                    *
  **********************************************/

void checkLeds(void);
void doWaitForConfig(void);
void doConnectSSID(void);
void doNormalRun(void);
boolean checkWiFi(void);
void showStripData();
void setAnimation(void);
void firstRun(void);
#ifdef USE_LDR
void checkLDRFire(void);
#endif
void eepromLoadData(void);
void eepromResetData(void);
void eepromUpdateData(void);
void showEEPROMData(void);
void mqttReconnect();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void mqttCheckConnectionStatusCallback(void);
#ifdef USE_DHT22
void sendTempHumCallback(void);
#endif
void sendTimerState(boolean theState);
void stripAutomaticOnOffTimerCallback(void);
void wifiCheckSignalStrengthCallback(void);
void refheshNoderedDashboardsCallback(void);


  /************************************
   * CONFIGURACIÓN DE LA TIRA DE LEDS *
   *************************************/

   // Número total de animaciones. Base 0.
#define MAX_ANIMATIONS				6	
#define DEFAULT_ANIMATION			0		// Animación por defecto.

// 'interval' define la velocidad de las animaciones: más 'interval' --> ralentiza animación, menos 'interval' --> acelera animación.	
#define DEFAULT_INTERVAL			100		// Valor por defecto paara 'interval'

// Las intensidad de los colores se definen por el brillo, que puede ir de 0 (apagado) a 100, máxima intensidad del color.
#define DEFAULT_BRIGHTNESS			8		// Brillo por defecto de las animaciones.

// El modo LDR permite que el dispositivo encienda (o apague) la tira de leds si la luz ambiente supera un umbral determinado.
#ifdef USE_LDR
#define LDR_MODE_ON					1		// Modo LDR ON.
#define LDR_MODE_OFF				0		// Mpdp LDR OFF.
#define LDR_FIRE_THRESHOLD			500		// Nivel de disparo del LDR (0-1000) Si la luz medida cae por debajo de este umbral, la tira se enciende.
#define LDR_MIN_FIRE_THRESHOLD		0		// Mínimo admitido para el disparo por LDR.
#define LDR_MAX_FIRE_THRESHOLD		1000	// Máximo admitido para el disparo por LDR.
#endif

 /**********************************************
  * FIN DE LA CONFIGURACIÓN DE LA TIRA DE LEDS *
  **********************************************/


  /********************************
   * OBJETOS Y VARIABLES GLOBALES *
   ********************************/

unsigned int initialMillis;	// Almacena los milis desde el inicio del programa.
boolean isPanicked = false; // Sirve para indicar si el ESP ha perdido la conectividad con la WiFi.
long wifiSignal = 0;		// Almacena la intensidad de la señal Wifi recibida (RSSI) en dBm (Actualizada en loop)

const char* SSID = "TOTEESPCFG" DEVICE_ID;    // El SSID debe ser único, pongo la fecha de fabricación como parte del GUID.
const char* PASS = "DarthVader*1";			// Password para conectar al punto de acceso que emite el dispositivo.


#define MQTT_CHECK_CONNECTION_STATUS_INTERVAL  10000UL  // Si se pierde la conexión con el servidor MQTT se reintentará pasado este tiempo.
#define MAX_MQTT_CHARS 50								// Longitud de los arrays para procesar mensajes MQTT.
const char* mqttClientID = DEVICE_ID;					// El id debe ser único, pongo la fecha de fabricación.
const char* MQTT_SERVER = "192.168.1.200";				// IP del servidor MQTT
char mqttLastStateOpTxt[MAX_MQTT_CHARS];				// Almacena mensaje sobre la última operación con el servidor MQTT.


bool isValidTime = false;	// Indica si se ha recibido hora desde el topic de NodeRed.
time_t theTime;				// Estructura para leer la hora.
char strTheTime[9];			// Array para almacenar la hora leída. HH:MM:SS\0


#ifdef USE_DHT22
#define SEND_TEMP_HUM_INTERVAL     10000UL	// Tiempo (en ms) entre envío de lecturas de temperatura y humedad del broker MQTT.
#endif

#define WIFI_CHECK_SIGNAL_STRENGTH 5000UL	// Tiempo (en ms) para medir la señal de la Wifi.
#define DASHBOARD_REFRESH_INTERVAL 2500UL	// Tiempo (en ms) para refrescar el Dashboard de Node-Red.

// Configuración del temporizador de encendido automático.
#define DEFAULT_TIMER_ENABLED_MODE	0		// Modo por defecto del temporizador de encendido, 1 = encendido, 0 = apagado.
#define DEFAULT_TIMER_H_INIT		22		// Hora por defecto a la que se inicia el temporizador.
#define DEFAULT_TIMER_M_INIT		0		// Minutos (de la hora anterior) a la que se inicia el temporizador.
#define DEFAULT_TIMER_DURATION      120		// Duración por defecto del temporizador.
#define NO_VALID_DAY				0       // Usado por el algoritmo del temporizador para detectar el estado no inicializado. Poner un valor fuera del entorno [1,31]
#define AUTOMATIC_ON_OFF_INTERVAL	2000UL  // polling de 2 segundos.

uint8_t timerDay = NO_VALID_DAY;	// Almacena la hora recibida por el dispositivo desde el servidor MQTT.

// Sirve para indicar el estado en el que se encuentra el dispositivo en 'loop()'
enum LOOP_RUN_MODE { CHECK_LEDS, WAIT_FOR_CONFIG, CONNECT_SSID, NORMAL_RUN };

// Modo actual de funcionamiento del programa en la función 'loop()'
int myLoopRunMode;


// Objeto para realizar 'delays' alimentando al WDT de software del ESP8266 para evitar los resets.
ToteESPMillisDelay myESPDelay = ToteESPMillisDelay(2000);

// Instancio objeto WiFiManager que presentará un punto de acceso para poder configurar la red WiFi (Leer siguiente comentario)
WiFiManager myWiFiManager;

/*	Cuando se inicia el dispositivo, entra en un bucle de espera para dar posibilidad de pulsar D2 (GPIO4). Este tiempo viene dado por el
	define CONFIG_WINDOW. El led parpadeará rápido durante este tiempo. Si se pulsa, emite un AP con el SSID dado por su correspondiente
	definición en el código, así como el PASS.

	ESP8266 emite un AP a cuya red debemos conectarnos y monta un servidor http para facilitar la configuración al AP
	verdadero. la IP de este servidor es 192.168.4.1 y nos podemos conectar con un móvil, por ejemplo. Una vez configurado, el
	dispositivo se reinicia. Debemos esperar sin hacer nada a que pase de nuevo el periodo de configuración.

	Ahora se conecta al AP correcto.

	Cuando el dispositivo pierde la conexión con el AP, por ejemplo, debido a que éste se ha caido o un corte eléctrico, el ESP8266 se reiniciará
	comenzando de nuevo todo el procedimiento. */

// Instancio objeto  auxiliar para PubSubClient
WiFiClient myWiFiClient;

// Instancio objeto para protocolo MQTT.
PubSubClient myMqttClient(myWiFiClient);

// Instancio un objeto ToteDebounceBtn para quitar el rebote al switch AP_CONF_PIN.
ToteDebouncedBtn  myAPConfigBtn = ToteDebouncedBtn(AP_CONF_PIN, NULL);

#ifdef USE_DHT22
// Instancio el objeto DHT22, en 'setup()' indicará en qué pin esta conectado.
DHTesp myDHT;
#endif

// Instancio el objeto de tira de leds
ToteNeo myStrip(NUM_OF_LEDS, STRIP_DATA_PIN, NEO_GRB + NEO_KHZ800, 10, MASTER_SWITCH_PIN, NULL);

#ifdef USE_LDR
// Instancio un objeto BlinkOutputLed que indica si el disparo por LDR está activo
ToteBlinkOutputLed myLDRLed(100UL, LED_LDR_PIN, ToteBlinkOutputLed::NO_BLINK, NULL); // Será obligatorio llamar a su método 'init' en 'setup'.
#endif

// Instancio un objeto BlinkOutputLed que indica el estado de la conexión con el AP.
ToteBlinkOutputLed myWiFiLed(100UL, LED_WIFI_PIN, ToteBlinkOutputLed::FINITE_BLINK, NULL); // Será obligatorio llamar a su método 'init' en 'setup'.

#ifdef USE_LDR
// Instancio un objeto ToteAnalogSensor para leer el valor de luz ambiente de la LDR.
ToteAnalogSensor myLDRSensor(LDR_PIN, ToteAnalogSensor::LESSTHAN, LDR_FIRE_THRESHOLD, LDR_FIRE_THRESHOLD, 50, 1000UL, NULL); // Será obligatorio llamar a su método 'init' en 'setup'.
#endif

#ifdef USE_DHT22																															 
// Instancio un objeto temporizador que indicará cuando enviar las lecturas de temperatura y humedad al Broker MQTT
ToteAsyncDelay myTempHumTimer(SEND_TEMP_HUM_INTERVAL, sendTempHumCallback);
#endif

// Temporizador de indicará cuando hay que testear la conexión con el servidor MQTT.
ToteAsyncDelay myMQTTCheckConnectionStatusTimer(MQTT_CHECK_CONNECTION_STATUS_INTERVAL, mqttCheckConnectionStatusCallback);

// Este temporizador se llama para ver si hay que encender las luces de forma automática.
ToteAsyncDelay myStripAutomaticOnOffTimer(AUTOMATIC_ON_OFF_INTERVAL, stripAutomaticOnOffTimerCallback);

// Este temporizador mide la intensidad de la señal WiFi.
ToteAsyncDelay myWiFiCheckSignalStrength(WIFI_CHECK_SIGNAL_STRENGTH, wifiCheckSignalStrengthCallback);

// Temporizador para actualizar los objetos de los Dashboards.
ToteAsyncDelay myNodeRedDashboardRefreshTimer(DASHBOARD_REFRESH_INTERVAL, refheshNoderedDashboardsCallback);

// Creo una estructura que servirá para almacenar y recuperar datos en la EEPROM
struct {
	uint8_t			EEPROM_MAGIC_NUMBER;			// Truco, amaceno el valor 69 si ya se ha inicializado los valores de la eeprom al menos una vez.
	uint8_t			EEPROM_ANIMATION_ID_UNIT8_T;	// Id de la animación que se está ejecutando.
	uint8_t			EEPROM_BRIGHT1_INT8_T;			// Valor de brillo para el color1 
	uint8_t			EEPROM_BRIGHT2_INT8_T;			// Valor de brillo para el color2
	unsigned long	EEPROM_INTERVAL_ULONG;			// Intervalo (pausa) entre cada tick de la animación. Si es cero, la animación va a máxima velocidad.
	unsigned long	EEPROM_COLOR1_ULONG;			// Color 1 de la animación.
	unsigned long	EEPROM_COLOR2_ULONG;			// Color 2 de la animación.
	float			EEPROM_COLOR1_WAVELENGTH_FLOAT;	// Color en longitud de onda para el roster de selección del color1.
	float			EEPROM_COLOR2_WAVELENGTH_FLOAT; // Color en longitud de onda para el roster de selección del color2.
#ifdef USE_LDR
	uint8_t			EEPROM_LDR_MODE_UINT8_T;		// Indicará si el modo LDR está activado o no.
	uint16_t		EEPROM_LDR_THRESHOLD_UINT16_T;	// Nivel de disparo configurado.
#endif
	uint8_t			EEPROM_TIMER_ENABLED_UINT8_T;	// Indica si el temporizador está activado.
	uint8_t			EEPROM_TIMER_H_INIT_UINT8_T;	// Hora a la que se activa el temporizador.
	uint8_t			EEPROM_TIMER_M_INIT_UINT8_T;	// Minutos (de la hora anterior) a la que se activa el temporizador.
	uint8_t			EEPROM_TIMER_DURATION_UINT16_T;  // Duración del temporizador en minutos (max 1440 o 24 horas)
} myEEPROMData;



/*********************************************************
 * FIN DE LA DECLARACIÓN DE OBJETOS Y VARIABLES GLOBALES *
 *********************************************************/



void setup() {
#if defined(_TOTE_DEBUG_LEVEL_1_ON_) || defined(_TOTE_DEBUG_LEVEL_2_ON_) 
	// Configuramos la conexión serie
	Serial.begin(115200);
#endif

#ifdef USE_DHT22
	// AM2302 (DHT22) está conectado al pin D1 (GPIO5)
	myDHT.setup(D6, DHTesp::DHT22);
#endif

	// Indicamos al objeto EEPROM cuántos bytes vamos a necesitar.
	EEPROM.begin(sizeof(myEEPROMData));

	// Cargamos los valores de la EEPROM en la estructura.
	eepromLoadData();

	// Intento leer el valor mágico 69 de la EEPROM. si no lo leo, entonces la reseteo para almacenar los valores por defecto.
	if (myEEPROMData.EEPROM_MAGIC_NUMBER != 69)
		eepromResetData();

	// Inicialización del objeto MQTT
	myMqttClient.setServer(MQTT_SERVER, 1883);
	myMqttClient.setCallback(mqttCallback);

	// Evitamos que muerda el perro.
	yield();

#ifdef USE_LDR
	// Inicializamos el objeto que gestiona el LED LDR
	myLDRLed.init();
#endif

	// Inicializamos el objeto que gestiona el LED WiFi.
	myWiFiLed.init();

	// Inicializamos el objeto que gestiona el botón para entrar en modo de configuración de AP.
	myAPConfigBtn.init();

#ifdef USE_LDR
	// Inicializamos en sensor LDR.
	myLDRSensor.init();
#endif

	// Inicializo la tira.
	myStrip.init();

	// Vamos a dar un segundo para que se estabilice todo.
	myESPDelay.stop(1000UL);

	// Limpiamos un poco la ventana del monitor serie.
	_TOTE_DEBUG_LEVEL_1_("setup()", "--------------------------------------------------------------------------");
	_TOTE_DEBUG_LEVEL_1_("setup()", "              INICIANDO LA EJECUCION DEL CODIGO");
	_TOTE_DEBUG_LEVEL_1_("setup()", "--------------------------------------------------------------------------");

	// Mostramos en la depuración los datos cargado desde la EEPROM.
	showEEPROMData();

	// Evitamos que muerda el perro.
	yield();

	// Muestro info de la tira de leds
	showStripData();

	// Inicializamos la  tira de leds.
	myStrip.begin();

	// Habilito el semáforo maestro de encendido por software.
	myStrip.masterSwitchON();

	// Mostramos la tira, en este caso se borran todos los leds.
	myStrip.show();

	// Configuración inicial de la animación leyendo valores almacenados en EEPROM.
	firstRun();

	// Activo el parpadeo de los led, para comprobar que no estén fundidos.
#ifdef USE_LDR
	myLDRLed.setBlinkType(ToteBlinkOutputLed::INFINITE_BLINK);
	myLDRLed.ledON();
#endif
	myWiFiLed.setBlinkType(ToteBlinkOutputLed::INFINITE_BLINK);
	myWiFiLed.ledON();

	// Entramos en loop() en modo de chequeo de leds.
	myLoopRunMode = LOOP_RUN_MODE::CHECK_LEDS;

	// Iniciamos el temporizador
	initialMillis = millis();

	_TOTE_DEBUG_LEVEL_1_("setup", "Entrando en loop()");
}

void loop() {
	switch (myLoopRunMode) {
	case CHECK_LEDS:
		checkLeds();
		break;

	case WAIT_FOR_CONFIG:
		doWaitForConfig();
		break;

	case CONNECT_SSID:
		doConnectSSID();
		break;

	case NORMAL_RUN:
		doNormalRun();
		break;
	}

#ifdef USE_LDR
	// Enciendo o apago el led LDR en función de su configuración.
	if (myEEPROMData.EEPROM_LDR_MODE_UINT8_T)
		myLDRLed.ledON();
	else
		myLDRLed.ledOFF();
#endif

	// Actualizo objetos que lo necesitan en cada iteración del loop().
	myWiFiLed.check();

#ifdef USE_LDR
	myLDRLed.check();
	myLDRSensor.check();
#endif

	// Evitamos que muerda el perro.
	yield();

	// Indico a la tira que procese la animación.
	myStrip.update();

#ifdef USE_LDR
	// Compruebo el disparo por LDR.
	checkLDRFire();
#endif
}

void checkLeds(void) {
	// Esta función realiza una simple espera para dar tiempo a que parpadeen varias veces los leds.
	if (initialMillis + CHECKLED_WINDOW > millis())
		return; // Me quedo en checkLeds sin avanzar durante CHECKLED_WINDOW segundos

	// Cuando llegue aquí es porque ya han pasado el tiempo de parpadeo.
	// Apago los leds.
#ifdef USE_LDR
	myLDRLed.ledOFF();
#endif
	myWiFiLed.ledOFF();

	_TOTE_DEBUG_LEVEL_1_("checkLeds", "Comprobacion finalizada. Entrando en ventana de configuracion de AP.");

	// Iniciamos el temporizador para el siguiente estado.
	initialMillis = millis();

	// Pongo el led Wifi a parpadear.
	myWiFiLed.setBlinkType(ToteBlinkOutputLed::INFINITE_BLINK);
	myWiFiLed.ledON();

#ifdef USE_LDR
	// Pongo el led LDR en apagado.
	myLDRLed.setBlinkType(ToteBlinkOutputLed::NO_BLINK);
	myLDRLed.ledOFF();
#endif

	// Pasamos al siguiente estado.
	myLoopRunMode = LOOP_RUN_MODE::WAIT_FOR_CONFIG;
}

void doWaitForConfig(void) {
	// Ventana de lanzamiento del portal de configuración.
	// Compruebo si se ha pulsado el botón o si la WiFi está desconectada.
	if (myAPConfigBtn.check()) {
		_TOTE_DEBUG_LEVEL_1_("doWaitForConfig", "Se ha pulsado el boton para lanzar el portal de configuracion.");

		// Se ha pulsado. Invocamos al portal de configuración
		// Creamos el AP con la siguiente configuración.
		myWiFiManager.startConfigPortal(SSID, PASS);

		// Mientras el ESP está en modo AP, nos conectamos a él con un navegador en la IP 192.168.4.1, 
		// configuramos la WIFI, la guardamos y reiniciamos el dispositivo para que se conecte al AP deseado,
		// ya sin pulsar el botón de configuración

		// Cambiamos el modo de ejecución.
		myLoopRunMode = LOOP_RUN_MODE::NORMAL_RUN;

		// Congelo la animación del led y lo apago ya que se ha terminado el proceso de configuración con el AP.
		myWiFiLed.setBlinkType(ToteBlinkOutputLed::NO_BLINK);
		myWiFiLed.ledOFF();

		// Salimos.
		return;
	}

	// Compruebo si ha terminado el periodo de configuración en cuyo caso cambiamos el modo de ejecución.
	if (initialMillis + CONFIG_WINDOW < millis()) {
		_TOTE_DEBUG_LEVEL_1_("doWaitForConfig", "Se ha acabado el tiempo de espera para lanzar el portal de configuracion");


		// Congelo la animación del led y lo apago ya que se ha terminado el proceso de configuración con el AP.
		myWiFiLed.setBlinkType(ToteBlinkOutputLed::NO_BLINK);
		myWiFiLed.ledOFF();

		// Cambiamos el modo de ejecución.
		myLoopRunMode = LOOP_RUN_MODE::CONNECT_SSID;
	}
}

void doConnectSSID(void) {
	// Compruebo si la WiFi sigue conectada
	if (checkWiFi()) {
		// Cambiamos el modo de ejecución.
		myLoopRunMode = LOOP_RUN_MODE::NORMAL_RUN;

		_TOTE_DEBUG_LEVEL_1_VALUE_("doConnectSSID", "Conectado a la WiFi con la IP: ", WiFi.localIP());
	}
}

void doNormalRun(void) {
	// Esta función contiene los procedimientos que deben llamarse en cada iteración del loop() cuando la Wifi funciona.
	// El el resto del loop() pongo las acciones que debe ejecutarse siempre.

	// Evitamos que muerda el perro.
	yield();

	// Comprobación de que el AP está OK.
	if (checkWiFi()) {
		// AQUÍ HAY QUE PONER LAS LLAMADAS A UPDATE PARA LOS OBJETOS QUE NECESITAN DE CONECTIVIDAD A LA RED
		// EL RESTO PUEDE PONERSE EN EL PROPIO LOOP.

		// Compruebo que el cliente de publicación/subscripción de MQTT está conectado.
		myMQTTCheckConnectionStatusTimer.check();

#ifdef USE_DHT22
		// Compruebo si se ha cumplido el intervalo para enviar la temperatura y la humedad
		myTempHumTimer.check();
#endif
		// Obligatorio para que se procesen los mensajes
		myMqttClient.loop();

		// Compruebo se se ha cumplido el intervalo para comprobar el temporizador de encendido y apagado automático.
		myStripAutomaticOnOffTimer.check();

		// Compruebo si se ha cumplido el intervalo para medir la intensidad de la señal WiFi.
		myWiFiCheckSignalStrength.check();

		// Compruebo si se ha cumplido el intervalo para refrescar el Dashboard de Node-Red.
		myNodeRedDashboardRefreshTimer.check();
	}
}

boolean checkWiFi(void) {
	// Compruebo si la WiFi sigue conectada
	if (!WiFi.isConnected()) {
		// Tenemos un problema. 
		_TOTE_DEBUG_LEVEL_1_("doNormalRun", "WiFi desconectada. Entrando en ventana de autoconfiguracion de AP.");

		// Pongo el led a parpadear para indicar que se ha caido el AP.
		myWiFiLed.setBlinkType(ToteBlinkOutputLed::INFINITE_BLINK);
		myWiFiLed.ledON();

		// Cambiamos al modo de ejecución para dar la posibilidad de configurar un AP diferente.
		initialMillis = millis(); // Para que empiece la cuenta de nuevo.
		myLoopRunMode = LOOP_RUN_MODE::WAIT_FOR_CONFIG;

		// Indico que la WIFI se ha caido o no conecta.
		return false;
	}
	else {
		// Está conectada. Dejo el led Wifi encendido.
		myWiFiLed.setBlinkType(ToteBlinkOutputLed::NO_BLINK);
		myWiFiLed.ledON();

		// Indico que la WIFI está OK.
		return true;
	}
}

void showStripData() {
	// Muestro datos de depuración de la tira de leds.
	_TOTE_DEBUG_LEVEL_2_VALUE_("showStripData", "Valor de Brillo 1: ", myStrip.getBright(ToteNeo::COLOR1));
	_TOTE_DEBUG_LEVEL_2_VALUE_("showStripData", "Valor de Brillo 1 almacenado: ", myStrip.getSavedBright(ToteNeo::COLOR1));
	_TOTE_DEBUG_LEVEL_2_VALUE_("showStripData", "Valor de Brillo 2: ", myStrip.getBright(ToteNeo::COLOR2));
	_TOTE_DEBUG_LEVEL_2_VALUE_("showStripData", "Valor de Brillo 2 almacenado: ", myStrip.getSavedBright(ToteNeo::COLOR2));
}

void setAnimation(void) {
	// Función Auxiliar. Establece la animación indicada por la variable global 'myEEPROMData.EEPROM_ANIMATION_ID_UNIT8_T'.
	// Posteriormente recupera de la EEPROM la última configuración de usuario para la animación.
	// Establezco animación.
	switch (myEEPROMData.EEPROM_ANIMATION_ID_UNIT8_T) {
	case 0:
		myStrip.circularRainbow(100UL);
		break;

	case 1:
		myStrip.rainbowCycle(10, ToteNeo::FORWARD_REVERSE);
		break;

	case 2:
		myStrip.colorWipe(myStrip.createColor(255, 0, 0), 6, 100, ToteNeo::direction::FORWARD);
		break;

	case 3:
		myStrip.theaterChase(myStrip.createColor(255, 0, 0), myStrip.createColor(0, 255, 0), 8, 100, ToteNeo::direction::FORWARD);
		break;

	case 4:
		myStrip.randomCycle(1000UL, ToteNeo::FORWARD);
		break;

	case 5:
		myStrip.randomOnOff(1000UL, ToteNeo::FORWARD);
		break;

	case 6:
		myStrip.randomOnOffFadding(5UL, ToteNeo::FORWARD);
		break;
	}

	_TOTE_DEBUG_LEVEL_1_VALUE_("SetAnimation", "Seleccionando animacion: ", myEEPROMData.EEPROM_ANIMATION_ID_UNIT8_T);

	// Recupero valores de brillo de la EEPROM
	myStrip.setBright(myEEPROMData.EEPROM_BRIGHT1_INT8_T, ToteNeo::COLOR1);
	myStrip.setBright(myEEPROMData.EEPROM_BRIGHT2_INT8_T, ToteNeo::COLOR2);

	// Recupero el valor de 'interval' almacenado en la EEPROM
	myStrip.setInterval(myEEPROMData.EEPROM_INTERVAL_ULONG);

	// Recupero los colores 1 y 2 de la EEPROM
	myStrip.setColor(ToteNeo::COLOR1, myEEPROMData.EEPROM_COLOR1_ULONG);
	myStrip.setColor(ToteNeo::COLOR2, myEEPROMData.EEPROM_COLOR2_ULONG);
}

void firstRun(void) {
	// Función auxiliar. Recupera de la EEPROM los valores de configuración del dispositivo elegidos por el usuario.
	// Posteriormente llama a 'setAnimation' para iniciar la animación.
	// Leo 'myEEPROMData.EEPROM_ANIMATION_ID_UNIT8_T' almacenado en EEPROM
	eepromLoadData();

#ifdef USE_LDR
	// Inicializo el objeto con el valor de umbral de disparo mínimo.
	myLDRSensor.setMinThreshold(myEEPROMData.EEPROM_LDR_THRESHOLD_UINT16_T);
#endif

	// Iniciamos la animación.
	setAnimation();
}

#ifdef USE_LDR
void checkLDRFire(void) {
	// Función auxiliar. Comprueba si se verifica la condición de disparo del LDR, actuando en consecuencia.
	if (myEEPROMData.EEPROM_LDR_MODE_UINT8_T == LDR_MODE_ON) {
		if (myLDRSensor.check() == true) { // Si LDR_MODE_ON y se ha producido disparo...
			// la tira de leds debe permanecer encendida.
			myStrip.masterSwitchON();
		}
		else { // Si LDR_MODE_ON y no se ha producido disparo...
		 // Apago la tira de leds.
			myStrip.masterSwitchOFF();
		}
	}
}
#endif

void eepromLoadData(void) {
	_TOTE_DEBUG_LEVEL_1_("eepromLoadData", "Leyendo datos de la EEPROM");

	EEPROM.get(0, myEEPROMData);
}

void eepromResetData(void) {
	// Función auxiliar. Guarda en EEPROM valores originales.
	// Almaceno animación por defecto en la EEPROM
	myEEPROMData.EEPROM_MAGIC_NUMBER = 69; // Esto indicará que la EEPROM se inicializó al menos una vez.
	myEEPROMData.EEPROM_ANIMATION_ID_UNIT8_T = DEFAULT_ANIMATION;
	myEEPROMData.EEPROM_BRIGHT1_INT8_T = DEFAULT_BRIGHTNESS;
	myEEPROMData.EEPROM_BRIGHT2_INT8_T = DEFAULT_BRIGHTNESS;
	myEEPROMData.EEPROM_INTERVAL_ULONG = DEFAULT_INTERVAL;
	myEEPROMData.EEPROM_COLOR1_ULONG = myStrip.createColor(255, 0, 0);
	myEEPROMData.EEPROM_COLOR2_ULONG = myStrip.createColor(0, 255, 0);
	myEEPROMData.EEPROM_COLOR1_WAVELENGTH_FLOAT = myStrip.getMinWaveLength();
	myEEPROMData.EEPROM_COLOR2_WAVELENGTH_FLOAT = myStrip.getMinWaveLength();
#ifdef USE_LDR
	myEEPROMData.EEPROM_LDR_MODE_UINT8_T = LDR_MODE_OFF;
	myEEPROMData.EEPROM_LDR_THRESHOLD_UINT16_T = LDR_FIRE_THRESHOLD;
#endif
	myEEPROMData.EEPROM_TIMER_ENABLED_UINT8_T = DEFAULT_TIMER_ENABLED_MODE;
	myEEPROMData.EEPROM_TIMER_H_INIT_UINT8_T = DEFAULT_TIMER_H_INIT;
	myEEPROMData.EEPROM_TIMER_M_INIT_UINT8_T = DEFAULT_TIMER_M_INIT;
	myEEPROMData.EEPROM_TIMER_DURATION_UINT16_T = DEFAULT_TIMER_DURATION;

	_TOTE_DEBUG_LEVEL_1_("eepromResetData", "guardando datos de la EEPROM");
	// Guardamos en EEPROM
	EEPROM.put(0, myEEPROMData);
	EEPROM.commit();
}

void eepromUpdateData(void) {
	_TOTE_DEBUG_LEVEL_1_("eepromUpdateData", "Actualizando datos en la EEPROM");

	// Guardamos en EEPROM
	EEPROM.put(0, myEEPROMData);
	EEPROM.commit();
}

void showEEPROMData(void) {
	_TOTE_DEBUG_LEVEL_2_VALUE_("showEEPROMData", "EEPROM_MAGIC_NUMBER = ", myEEPROMData.EEPROM_MAGIC_NUMBER);
	_TOTE_DEBUG_LEVEL_2_VALUE_("showEEPROMData", "EEPROM_ANIMATION_ID_UNIT8_T = ", myEEPROMData.EEPROM_ANIMATION_ID_UNIT8_T);
	_TOTE_DEBUG_LEVEL_2_VALUE_("showEEPROMData", "EEPROM_BRIGHT1_UINT8_T = ", myEEPROMData.EEPROM_BRIGHT1_INT8_T);
	_TOTE_DEBUG_LEVEL_2_VALUE_("showEEPROMData", "EEPROM_BRIGHT2_UINT8_T = ", myEEPROMData.EEPROM_BRIGHT2_INT8_T);
	_TOTE_DEBUG_LEVEL_2_VALUE_("showEEPROMData", "EEPROM_INTERVAL_ULONG = ", myEEPROMData.EEPROM_INTERVAL_ULONG);
	_TOTE_DEBUG_LEVEL_2_VALUE_("showEEPROMData", "EEPROM_COLOR1_ULONG = ", myEEPROMData.EEPROM_COLOR1_ULONG);
	_TOTE_DEBUG_LEVEL_2_VALUE_("showEEPROMData", "EEPROM_COLOR2_ULONG = ", myEEPROMData.EEPROM_COLOR2_ULONG);
	_TOTE_DEBUG_LEVEL_2_VALUE_("showEEPROMData", "EEPROM_COLOR1_WAVELENGTH_FLOAT = ", myEEPROMData.EEPROM_COLOR1_WAVELENGTH_FLOAT);
	_TOTE_DEBUG_LEVEL_2_VALUE_("showEEPROMData", "EEPROM_COLOR2_WAVELENGTH_FLOAT = ", myEEPROMData.EEPROM_COLOR2_WAVELENGTH_FLOAT);
#ifdef USE_LDR
	_TOTE_DEBUG_LEVEL_2_VALUE_("showEEPROMData", "EEPROM_LDR_MODE_UINT8_T = ", myEEPROMData.EEPROM_LDR_MODE_UINT8_T);
	_TOTE_DEBUG_LEVEL_2_VALUE_("showEEPROMData", "EEPROM_LDR_THRESHOLD_UINT16_T = ", myEEPROMData.EEPROM_LDR_THRESHOLD_UINT16_T);
#endif
	_TOTE_DEBUG_LEVEL_2_VALUE_("showEEPROMData", "EPROM_TIMER_ENABLED_UINT8_T = ", myEEPROMData.EEPROM_TIMER_ENABLED_UINT8_T);
	_TOTE_DEBUG_LEVEL_2_VALUE_("showEEPROMData", "EEPROM_TIMER_H_INIT_UINT8_T = ", myEEPROMData.EEPROM_TIMER_H_INIT_UINT8_T);
	_TOTE_DEBUG_LEVEL_2_VALUE_("showEEPROMData", "EEPROM_TIMER_M_INIT_UINT8_T = ", myEEPROMData.EEPROM_TIMER_M_INIT_UINT8_T);
	_TOTE_DEBUG_LEVEL_2_VALUE_("showEEPROMData", "EEPROM_TIMER_DURATION_UINT16_T = ", myEEPROMData.EEPROM_TIMER_DURATION_UINT16_T);
}

void mqttReconnect() {
	_TOTE_DEBUG_LEVEL_1_("mqttReconnect", "Intentando reconectar con servidor MQTT...");

	// Intento la conexión.		
	if (myMqttClient.connect(mqttClientID)) {
		_TOTE_DEBUG_LEVEL_1_("mqttReconnect", "Conectado al servidor MQTT!!!");

		// Una vez conectado nos volvemos a subscribir.
		myMqttClient.subscribe("casa/time_source");
		myMqttClient.subscribe("casa/" DEVICE_ID "/timer_enabled");
		myMqttClient.subscribe("casa/" DEVICE_ID "/timer_initial_hours");
		myMqttClient.subscribe("casa/" DEVICE_ID "/timer_initial_minutes");
		myMqttClient.subscribe("casa/" DEVICE_ID "/timer_duration");
		myMqttClient.subscribe("casa/" DEVICE_ID "/masterSwitch");
		myMqttClient.subscribe("casa/" DEVICE_ID "/animationid");
		myMqttClient.subscribe("casa/" DEVICE_ID "/interval");
		myMqttClient.subscribe("casa/" DEVICE_ID "/color1");
		myMqttClient.subscribe("casa/" DEVICE_ID "/color2");
		myMqttClient.subscribe("casa/" DEVICE_ID "/bright1");
		myMqttClient.subscribe("casa/" DEVICE_ID "/bright2");
#ifdef USE_LDR
		myMqttClient.subscribe("casa/" DEVICE_ID "/ldr");
		myMqttClient.subscribe("casa/" DEVICE_ID "/ldr_fire_threshold");
#endif
		myMqttClient.subscribe("casa/" DEVICE_ID "/eeprom_reset_data");
	}
	else {
		_TOTE_DEBUG_LEVEL_1_("mqttReconnect", "Error al conectar al servidor MQTT");
	}
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
	// Array para procesar el payload.
	static char mqttPayload[MAX_MQTT_CHARS];

	unsigned int i;
	for (i = 0; i < length && i < MAX_MQTT_CHARS - 1; i++) {
		mqttPayload[i] = (char)payload[i];
	}
	mqttPayload[i] = '\0';


	_TOTE_DEBUG_LEVEL_2_VALUE_("mqttCallback", "Ha llegado un mensaje para el topic: ", topic);
	_TOTE_DEBUG_LEVEL_2_VALUE_("mqttCallback", "con el valor: ", mqttPayload);
	_TOTE_DEBUG_LEVEL_2_VALUE_("mqttCallback", "y longitud : ", length);

	/**********************************
	*  PROCESADO DE LOS MENSAJES MQTT *
	***********************************/

	// Evitamos que muerda el perro.
	yield();

	// casa/time_source
	if (strcmp(topic, "casa/time_source") == 0) {
		_TOTE_DEBUG_LEVEL_1_("mqttCallback", "Detectado topic casa/time_source");

		// Convierto topic a UnixEpoch
		theTime = atol(mqttPayload);

		_TOTE_DEBUG_LEVEL_2_VALUE_("mqttCallback", "theTime: ", theTime);

		_TOTE_DEBUG_LEVEL_2_VALUE_("mqttCallback", "day: ", day(theTime));
		_TOTE_DEBUG_LEVEL_2_VALUE_("mqttCallback", "month: ", month(theTime));
		_TOTE_DEBUG_LEVEL_2_VALUE_("mqttCallback", "year: ", year(theTime));
		_TOTE_DEBUG_LEVEL_2_VALUE_("mqttCallback", "hour: ", hour(theTime));
		_TOTE_DEBUG_LEVEL_2_VALUE_("mqttCallback", "minute: ", minute(theTime));
		_TOTE_DEBUG_LEVEL_2_VALUE_("mqttCallback", "Second: ", second(theTime));

		// Indico que la hora ya es válida.
		isValidTime = true;

		// Salimos.
		return;
	}

	// Evitamos que muerda el perro.
	yield();

		// casa/DEVICE_ID/timer_enabled
	if (strcmp(topic, "casa/" DEVICE_ID "/timer_enabled") == 0) {
		_TOTE_DEBUG_LEVEL_1_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/timer_enabled");

		// Actualizo el valor de activación del temporizador.
		myEEPROMData.EEPROM_TIMER_ENABLED_UINT8_T = (uint8_t)atoi(mqttPayload);

		_TOTE_DEBUG_LEVEL_1_VALUE_("mqttCallback", "myEEPROMData.EEPROM_TIMER_ENABLED_UINT8_T ha cambiado a :", myEEPROMData.EEPROM_TIMER_ENABLED_UINT8_T);

		// Actualizo variables globales en la EEPROM.
		eepromUpdateData();

		// Compruebo si se ha deshabilitado el temporizador.
		if (myEEPROMData.EEPROM_TIMER_ENABLED_UINT8_T == 0) {
			_TOTE_DEBUG_LEVEL_1_("mqttCallback", "habilitado masterSwitch.");
			myStrip.masterSwitchON(); // Si apago el temporizador, entonces habilito el master switch on
		}


		// Salimos.
		return;
	}

	// Evitamos que muerda el perro.
	yield();

	// casa/DEVICE_ID/timer_initial_hours
	if (strcmp(topic, "casa/" DEVICE_ID "/timer_initial_hours") == 0) {
		_TOTE_DEBUG_LEVEL_1_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/timer_initial_hours");

		// Actualizo el valor la hora de inicio del temporizador.
		myEEPROMData.EEPROM_TIMER_H_INIT_UINT8_T = (uint8_t)atoi(mqttPayload);

		_TOTE_DEBUG_LEVEL_1_VALUE_("mqttCallback", "myEEPROMData.EEPROM_TIMER_H_INIT_UINT8_T ha cambiado a :", myEEPROMData.EEPROM_TIMER_H_INIT_UINT8_T);

		// Actualizo variables globales en la EEPROM.
		eepromUpdateData();

		// Salimos.
		return;
	}


	// Evitamos que muerda el perro.
	yield();

	// casa/DEVICE_ID/timer_initial_minutes
	if (strcmp(topic, "casa/" DEVICE_ID "/timer_initial_minutes") == 0) {
		_TOTE_DEBUG_LEVEL_1_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/timer_initial_minutes");

		// Actualizo el valor los minutos de inicio del temporizador.
		myEEPROMData.EEPROM_TIMER_M_INIT_UINT8_T = (uint8_t)atoi(mqttPayload);

		_TOTE_DEBUG_LEVEL_1_VALUE_("mqttCallback", "myEEPROMData.EEPROM_TIMER_M_INIT_UINT8_T ha cambiado a :", myEEPROMData.EEPROM_TIMER_M_INIT_UINT8_T);

		// Actualizo variables globales en la EEPROM.
		eepromUpdateData();

		// Salimos.
		return;
	}


	// Evitamos que muerda el perro.
	yield();

	// casa/DEVICE_ID/timer_duration
	if (strcmp(topic, "casa/" DEVICE_ID "/timer_duration") == 0) {
		_TOTE_DEBUG_LEVEL_1_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/timer_duration");

		// Actualizo el valor de la duración de la temporización.
		myEEPROMData.EEPROM_TIMER_DURATION_UINT16_T = (uint16_t)atoi(mqttPayload);

		_TOTE_DEBUG_LEVEL_1_VALUE_("mqttCallback", "myEEPROMData.EEPROM_TIMER_DURATION_UINT16_T ha cambiado a :", myEEPROMData.EEPROM_TIMER_DURATION_UINT16_T);

		// Actualizo variables globales en la EEPROM.
		eepromUpdateData();

		// Salimos.
		return;
	}

	// Evitamos que muerda el perro.
	yield();

	// casa/DEVICE_ID/masterSwitch
	if (strcmp(topic, "casa/" DEVICE_ID "/masterSwitch") == 0) {
		_TOTE_DEBUG_LEVEL_1_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/masterSwitch");

		// Actualizo el masterSwitch
		uint8_t ms = (uint8_t)atoi(mqttPayload);

		if (ms == 1) {
			myStrip.masterSwitchON();
		}
		else {
			myStrip.masterSwitchOFF();
		}

		_TOTE_DEBUG_LEVEL_1_VALUE_("mqttCallback", "masterSwitch ha cambiado a :", ms);

		// Salimos.
		return;
	}

	// Evitamos que muerda el perro.
	yield();

	// casa/DEVICE_ID/animationid
	if (strcmp(topic, "casa/" DEVICE_ID "/animationid") == 0) {
		_TOTE_DEBUG_LEVEL_1_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/animationid");

		// Actualizo número de la animación en la estructura de datos de EEPROM.
		myEEPROMData.EEPROM_ANIMATION_ID_UNIT8_T = (uint8_t)atoi(mqttPayload);

		_TOTE_DEBUG_LEVEL_1_VALUE_("mqttCallback", "myEEPROMData.EEPROM_ANIMATION_ID_UNIT8_T ha cambiado a :", myEEPROMData.EEPROM_ANIMATION_ID_UNIT8_T);

		// Actualizo variables globales en la EEPROM.
		eepromUpdateData();

		// Configuramos la animación
		setAnimation();

		// Salimos.
		return;
	}

	// Evitamos que muerda el perro.
	yield();

	// casa/DEVICE_ID/interval
	if (strcmp(topic, "casa/" DEVICE_ID "/interval") == 0) {
		_TOTE_DEBUG_LEVEL_1_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/interval");

		// Actualizo la velocidad de la animación.
		myEEPROMData.EEPROM_INTERVAL_ULONG = (unsigned long)atol(mqttPayload);

		_TOTE_DEBUG_LEVEL_1_VALUE_("mqttCallback", "myEEPROMData.EEPROM_INTERVAL_ULONG (ms) ha cambiado a :", myEEPROMData.EEPROM_INTERVAL_ULONG);

		// Actualizamos el valor del intervalo.
		myStrip.setInterval(myEEPROMData.EEPROM_INTERVAL_ULONG);

		// Actualizo variables globales en la EEPROM.
		eepromUpdateData();

		// Salimos.
		return;
	}

	// Evitamos que muerda el perro.
	yield();

	// casa/DEVICE_ID/color1
	if (strcmp(topic, "casa/" DEVICE_ID "/color1") == 0) {
		_TOTE_DEBUG_LEVEL_1_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/color1");

		//  Establece el color1.
		myEEPROMData.EEPROM_COLOR1_WAVELENGTH_FLOAT = atof(mqttPayload);

		_TOTE_DEBUG_LEVEL_1_VALUE_("mattCallback", "myEEPROMData.EEPROM_COLOR1_WAVELENGTH_FLOAT ha cambiado a : ", myEEPROMData.EEPROM_COLOR1_WAVELENGTH_FLOAT);

		if (myStrip.getCanModifyColor()) {
			// Actualizamos el valor del intervalo.
			myStrip.selectRosterWaveLengthColor(ToteNeo::COLOR1, myEEPROMData.EEPROM_COLOR1_WAVELENGTH_FLOAT);

			// Actualizo la variable global.
			myEEPROMData.EEPROM_COLOR1_ULONG = myStrip.getColor(ToteNeo::COLOR1);

			// Actualizo variables globales en la EEPROM.
			eepromUpdateData();
		}

		// Salimos.
		return;
	}

	// Evitamos que muerda el perro.
	yield();

	// casa/DEVICE_ID/color2
	if (strcmp(topic, "casa/" DEVICE_ID "/color2") == 0) {
		_TOTE_DEBUG_LEVEL_1_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/color2");

		//  Establece el color2.
		myEEPROMData.EEPROM_COLOR2_WAVELENGTH_FLOAT = atof(mqttPayload);

		_TOTE_DEBUG_LEVEL_1_VALUE_("mqttCallback", "myEEPROMData.EEPROM_COLOR2_WAVELENGTH_FLOAT ha cambiado a : ", myEEPROMData.EEPROM_COLOR2_WAVELENGTH_FLOAT);

		if (myStrip.getCanModifyColor()) {
			// Actualizamos el valor del intervalo.
			myStrip.selectRosterWaveLengthColor(ToteNeo::COLOR2, myEEPROMData.EEPROM_COLOR2_WAVELENGTH_FLOAT);

			// Actualizo la variable global.
			myEEPROMData.EEPROM_COLOR2_ULONG = myStrip.getColor(ToteNeo::COLOR2);

			// Actualizo variables globales en la EEPROM.
			eepromUpdateData();
		}

		// Salimos.
		return;
	}

	// Evitamos que muerda el perro.
	yield();

	// casa/DEVICE_ID/bright1
	if (strcmp(topic, "casa/" DEVICE_ID "/bright1") == 0) {
		_TOTE_DEBUG_LEVEL_1_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/bright1");

		// Establece el brillo para el color1.
		myEEPROMData.EEPROM_BRIGHT1_INT8_T = (int8_t)atoi(mqttPayload);

		_TOTE_DEBUG_LEVEL_1_VALUE_("mqttCallback", "myEEPROMData.EEPROM_BRIGHT1_INT8_T ha cambiado a :", myEEPROMData.EEPROM_BRIGHT1_INT8_T);

		// Actualizamos el brillo en la animación
		myStrip.setBright(myEEPROMData.EEPROM_BRIGHT1_INT8_T, ToteNeo::COLOR1);

		// Actualizo variables globales en la EEPROM.
		eepromUpdateData();

		// Salimos
		return;
	}

	// Evitamos que muerda el perro.
	yield();

	// casa/DEVICE_ID/bright2
	if (strcmp(topic, "casa/" DEVICE_ID "/bright2") == 0) {
		_TOTE_DEBUG_LEVEL_1_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/bright2");

		// Establece el brillo para el color2.
		myEEPROMData.EEPROM_BRIGHT2_INT8_T = (int8_t)atoi(mqttPayload);

		_TOTE_DEBUG_LEVEL_1_VALUE_("mqttCallback", "myEEPROMData.EEPROM_BRIGHT2_INT8_T ha cambiado a :", myEEPROMData.EEPROM_BRIGHT2_INT8_T);

		// Actualizamos el brillo en la animación
		myStrip.setBright(myEEPROMData.EEPROM_BRIGHT2_INT8_T, ToteNeo::COLOR2);

		// Actualizo variables globales en la EEPROM.
		eepromUpdateData();

		// Salimos
		return;
	}


#ifdef USE_LDR
	// Evitamos que muerda el perro.
	yield();

	// casa/DEVICE_ID/ldr
	if (strcmp(topic, "casa/" DEVICE_ID  "/ldr") == 0) {
		_TOTE_DEBUG_LEVEL_1_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/ldr");

		// Activa o desactiva el LDR.
		myEEPROMData.EEPROM_LDR_MODE_UINT8_T = (uint8_t)atoi(mqttPayload);

		_TOTE_DEBUG_LEVEL_1_VALUE_("mqttCallback", "myEEPROMData.EEPROM_LDR_MODE_UINT8_T :", myEEPROMData.EEPROM_LDR_MODE_UINT8_T);

		// Actualizo variables globales en la EEPROM.
		eepromUpdateData();

		// Salimos
		return;
	}

	// Evitamos que muerda el perro.
	yield();

	// casa/DEVICE_ID/ldr_fire_threshold
	if (strcmp(topic, "casa/" DEVICE_ID "/ldr_fire_threshold") == 0) {
		_TOTE_DEBUG_LEVEL_1_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/ldr_fire_threshold");


		// Configura el umbral de disparo del LDR.
		myEEPROMData.EEPROM_LDR_THRESHOLD_UINT16_T = (uint16_t)atol(mqttPayload);

		_TOTE_DEBUG_LEVEL_1_VALUE_("mqttCallback", "myEEPROMData.EEPROM_LDR_THRESHOLD_UINT16_T :", myEEPROMData.EEPROM_LDR_THRESHOLD_UINT16_T);

		// Actualizamos el valor de disparo del objeto
		myLDRSensor.setMinThreshold(myEEPROMData.EEPROM_LDR_THRESHOLD_UINT16_T);

		// Actualizo variables globales en la EEPROM.
		eepromUpdateData();

		// Salimos
		return;
	}
#endif

	// Evitamos que muerda el perro.
	yield();

	// casa/DEVICE_ID/eeprom_reset_data
	if (strcmp(topic, "casa/" DEVICE_ID "/eeprom_reset_data") == 0) {
		_TOTE_DEBUG_LEVEL_1_("mqttCallback", "Detectado topic casa/" DEVICE_ID "/eeprom_reset_data");

		// Resetea la configuración de la EEPROM.

		_TOTE_DEBUG_LEVEL_1_("mqttCallback", "Activada EEPROM_RESET.");

		// Reseteamos la EEPROM.
		eepromResetData();

		// Configuramos la animación
		setAnimation();

		_TOTE_DEBUG_LEVEL_1_VALUE_("mqttCallback", "Valor de variable global 'myEEPROMData.EEPROM_TIMER_ENABLED_UINT8_T: ", myEEPROMData.EEPROM_TIMER_ENABLED_UINT8_T);

		// Salimos
		return;
	}
}

void mqttCheckConnectionStatusCallback(void) {
	// Si el estado es diferente a 0 (conexión correcta) fuerzo la reconexión.
	// Ver 'mqttGetState' para valores de retorno.
	if (!myMqttClient.connected()) {
		_TOTE_DEBUG_LEVEL_1_("mqttCheckConnectionStatusCallback", "Intentando reconectar con servidor MQTT.");

		// Fuerzo la reconexión.
		mqttReconnect();

		// Hasta la próxima.
		return;
	}

	// Consulto el estado de la última operación con el servidor MQTT.
	int status = myMqttClient.state();

	switch (status) {
	case -4:
		_TOTE_DEBUG_LEVEL_2_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECTION_TIMEOUT - the server didn't respond within the keepalive time.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECTION_TIMEOUT");
		break;

	case -3:
		_TOTE_DEBUG_LEVEL_2_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECTION_LOST - the network connection was broken.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECTION_LOST");
		break;

	case -2:
		_TOTE_DEBUG_LEVEL_2_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECT_FAILED - the network connection failed.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECT_FAILED");
		break;

	case -1:
		_TOTE_DEBUG_LEVEL_2_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_DISCONNECTED - the client is disconnected cleanly.");
		strcpy(mqttLastStateOpTxt, "MQTT_DISCONNECTED");
		break;

	case 0:
		_TOTE_DEBUG_LEVEL_2_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECTED - the client is connected.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECTED");
		break;

	case 1:
		_TOTE_DEBUG_LEVEL_2_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECT_BAD_PROTOCOL - the server doesn't support the requested version of MQTT.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECT_BAD_PROTOCOL");
		break;

	case 2:
		_TOTE_DEBUG_LEVEL_2_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECT_BAD_CLIENT_ID - the server rejected the client identifier.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECT_BAD_CLIENT_ID");
		break;

	case 3:
		_TOTE_DEBUG_LEVEL_2_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECT_UNAVAILABLE - the server was unable to accept the connection.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECT_UNAVAILABLE");
		break;

	case 4:
		_TOTE_DEBUG_LEVEL_2_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECT_BAD_CREDENTIALS - the username / password were rejected.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECT_BAD_CREDENTIALS");
		break;

	case 5:
		_TOTE_DEBUG_LEVEL_2_("mqttGetState", "Mensaje de la ultima operacion con el servidor MQTT: MQTT_CONNECT_UNAUTHORIZED - the client was not authorized to connect.");
		strcpy(mqttLastStateOpTxt, "MQTT_CONNECT_UNAUTHORIZED");
		break;
	}
}


#ifdef USE_DHT22
void sendTempHumCallback(void) {
	// Leemos temperatura.
	float temp = myDHT.getTemperature();

	// Leemos humedad.
	float hum = myDHT.getHumidity();

	_TOTE_DEBUG_LEVEL_2_VALUE_("sendTempHumCallback", "La temperatura en el sensor es: ", temp);
	_TOTE_DEBUG_LEVEL_2_VALUE_("sendTempHumCallback", "La humedad en el sensor es: ", hum);

	// Array para construir el mensaje a enviar.
	static char mqttMsg[MAX_MQTT_CHARS];

	// Convierto a string la temperatura: 3 enteros y 2 decimales.
	sprintf(mqttMsg, "%3.2f", temp);

	// Publicamos mensaje en el broker MQTT
	myMqttClient.publish("casa/" DEVICE_ID "/temperatura", mqttMsg);

	// Convierto a string la humedad: 3 enteros y 2 decimales.
	sprintf(mqttMsg, "%3.2f", hum);

	// Publicamos mensaje en el broker MQTT
	myMqttClient.publish("casa/" DEVICE_ID "/humedad", mqttMsg);
}
#endif

void sendTimerState(boolean theState) {
	if (theState) {
		_TOTE_DEBUG_LEVEL_2_("sendTimerState", "Indicando que el temporizador automatico ha ENCENDIDO la tira de leds");
		myMqttClient.publish("casa/" DEVICE_ID "/timer_state", "SI");
	}
	else {
		_TOTE_DEBUG_LEVEL_2_("sendTimerState", "Indicando que el temporizador automatico ha APAGADO la tira de leds");
		myMqttClient.publish("casa/" DEVICE_ID "/timer_state", "NO");
	}
}

void stripAutomaticOnOffTimerCallback(void) {
	// Si se dan las condiciones enciende o apaga por software la tira de leds. Esto lo hace llamando a métodos de
	// la clase TOTENEO.

	_TOTE_DEBUG_LEVEL_2_VALUE_("stripAutomaticOnOffTimerCallback", "myEEPROMData.EEPROM_TIMER_ENABLED_UINT8_T: ", myEEPROMData.EEPROM_TIMER_ENABLED_UINT8_T);

	// Si no está habilitado el temporizador, no se hace nada.
	if (!myEEPROMData.EEPROM_TIMER_ENABLED_UINT8_T) {
		// Actualizo el nodo en el dashboard
		sendTimerState(false);

		return;
	}

	// Si la hora recibida del topic no es correcta, tampoco se hace nada.
	if (!isValidTime) {
		_TOTE_DEBUG_LEVEL_2_VALUE_("stripAutomaticOnOffTimerCallback", "isValidTime: ", isValidTime);
		return;
	}

	// Compruebo si el dia se ha inicializado.
	if (timerDay == NO_VALID_DAY) {
		timerDay = day(theTime);

		_TOTE_DEBUG_LEVEL_2_VALUE_("stripAutomaticOnOffTimerCallback", "Inicializando el valor de dia leido de epoch : ", timerDay);
	}



	uint16_t him = myEEPROMData.EEPROM_TIMER_H_INIT_UINT8_T * 60 + myEEPROMData.EEPROM_TIMER_M_INIT_UINT8_T; // Hora inicial en minutos.
	_TOTE_DEBUG_LEVEL_2_VALUE_("stripAutomaticOnOffTimerCallback", "timer_initial_hoursInMinutes (him): ", him);

	_TOTE_DEBUG_LEVEL_2_VALUE_("stripAutomaticOnOffTimerCallback", "myEEPROMData.EEPROM_TIMER_DURATION_UINT16_T: ", myEEPROMData.EEPROM_TIMER_DURATION_UINT16_T);

	uint16_t hact = hour(theTime) * 60 + minute(theTime); // Hora actual en minutos.
	_TOTE_DEBUG_LEVEL_2_VALUE_("stripAutomaticOnOffTimerCallback", "Hora actual en minutos: ", hact);


	uint16_t hfm = 0;  // = almacenará el cálculo de la hora final.

	// Situación 1: el inicio y apagado están dentro del mismo día. (0 <= him <= hact <= hfm <= 1440)
	if (him + myEEPROMData.EEPROM_TIMER_DURATION_UINT16_T <= 1440) {
		_TOTE_DEBUG_LEVEL_2_("stripAutomaticOnOffTimerCallback", "Detectada situacion 1: inicio y apagado en el mismo dia.");

		// Calculo la hora de apagado.
		hfm = him + myEEPROMData.EEPROM_TIMER_DURATION_UINT16_T;
		_TOTE_DEBUG_LEVEL_2_VALUE_("stripAutomaticOnOffTimerCallback", "Hora final (de apagado) en minutos calculada: ", hfm);

		// Compruebo si la hora actual está dentro de la ventana de disparo.
		if (hact >= him && hact <= hfm) {
			// Enciendo la tira.
			myStrip.masterSwitchON();

			_TOTE_DEBUG_LEVEL_2_("stripAutomaticOnOffTimerCallback", "Encendiendo tira por medio de control masterSwitch");

			// Actualizo el nodo en el dashboard
			sendTimerState(true);
		}
		else {
			// Apago la tira.
			myStrip.masterSwitchOFF();

			_TOTE_DEBUG_LEVEL_2_("stripAutomaticOnOffTimerCallback", "Apagando tira por medio de control masterSwitch");

			// Actualizo el nodo en el dashboard
			sendTimerState(false);
		}

		// Nada más que hacer.
		return;
	}

	// Situación 2: El inicio y apagado estan en diferentes dias. En primer lugar debo cazar el cambio de día.
	// Para ello me ayudo de una variable auxiliar que se llama 'timerDay'.
	// Si 'timerDay' coincide con day(epoch) entonces no se han pasado las 24 horas y estoy en la parte 'A' del algoritmo.
	// En caso contrario estoy en la parte 'B'

	if (timerDay == day(theTime)) { // Estamos en la parte 'A' del algoritmo (hasta 1440)
		_TOTE_DEBUG_LEVEL_2_("stripAutomaticOnOffTimerCallback", "Detectada situacion 2A: inicio y apagado en diferente dia, hora actual <= 1440.");

		// Se debe encender desde 'him' hasta 1440. Como el tiempo en minutos no puede ser mayor de 1440, solo solo compruebo 'him'
		if (hact >= him) {
			// Enciendo la tira.
			myStrip.masterSwitchON();

			_TOTE_DEBUG_LEVEL_2_("stripAutomaticOnOffTimerCallback", "Encendiendo tira por medio de control masterSwitch");

			// Actualizo el nodo en el dashboard
			sendTimerState(true);
		}
		else {
			// Apago la tira.
			myStrip.masterSwitchOFF();

			_TOTE_DEBUG_LEVEL_2_("stripAutomaticOnOffTimerCallback", "Apagando tira por medio de control masterSwitch");

			// Actualizo el nodo en el dashboard
			sendTimerState(false);
		}
	}
	else { // Estamos en la parte 'B'
		_TOTE_DEBUG_LEVEL_2_("stripAutomaticOnOffTimerCallback", "Detectada situacion 2B: inicio y apagado en diferente dia, hora actual > 1440, es decir >= 0 ");

		// En este caso, se debe encender desde el minuto cero hasta 'hfm'
		hfm = myEEPROMData.EEPROM_TIMER_DURATION_UINT16_T - (1440 - him);
		_TOTE_DEBUG_LEVEL_2_VALUE_("stripAutomaticOnOffTimerCallback", "Hora final (de apagado) en minutos calculada: ", hfm);

		if (hact <= hfm) {
			// Enciendo la tira.
			myStrip.masterSwitchON();

			_TOTE_DEBUG_LEVEL_2_("stripAutomaticOnOffTimerCallback", "Encendiendo tira por medio de control masterSwitch");

			// Actualizo el nodo en el dashboard
			sendTimerState(true);
		}
		else { // Ya se ha salido por la derecha de la parte 'B'.
			timerDay = day(theTime); // Actualizo 'timerDay' para indicar que pasamos al día siguiente.

			// Apago la tira.
			myStrip.masterSwitchOFF();

			_TOTE_DEBUG_LEVEL_2_("stripAutomaticOnOffTimerCallback", "Apagando tira por medio de control masterSwitch");

			// Actualizo el nodo en el dashboard
			sendTimerState(false);
		}
	}
}

void wifiCheckSignalStrengthCallback(void) {
	// Leo el valor de la intensidad de la señal WiFi
	wifiSignal = WiFi.RSSI();

	_TOTE_DEBUG_LEVEL_2_VALUE_("wifiCheckSignalStrengthCallback", "La intensidad de la WiFI en dBm es: ", wifiSignal);
}

void refheshNoderedDashboardsCallback(void) {
	// Array para construir el mensaje a enviar.
	static char mqttMsg[MAX_MQTT_CHARS];

	_TOTE_DEBUG_LEVEL_1_("refheshNoderedDashboardsCallback", "Refrescando Dashboard de Node-Red.");


	// Actualizo 'timer_enabled_refresh'
	sprintf(mqttMsg, "%d", myEEPROMData.EEPROM_TIMER_ENABLED_UINT8_T);
	myMqttClient.publish("casa/" DEVICE_ID "/timer_enabled_refresh", mqttMsg);

	// Actualizo 'timer_initial_hours_refresh'
	sprintf(mqttMsg, "%d", myEEPROMData.EEPROM_TIMER_H_INIT_UINT8_T);
	myMqttClient.publish("casa/" DEVICE_ID "/timer_initial_hours_refresh", mqttMsg);

	// Actualizo 'timer_initial_minutes_refresh'
	sprintf(mqttMsg, "%d", myEEPROMData.EEPROM_TIMER_M_INIT_UINT8_T);
	myMqttClient.publish("casa/" DEVICE_ID "/timer_initial_minutes_refresh", mqttMsg);

	// Actualizo 'timer_duration'
	sprintf(mqttMsg, "%d", myEEPROMData.EEPROM_TIMER_DURATION_UINT16_T);
	myMqttClient.publish("casa/" DEVICE_ID "/timer_duration_refresh", mqttMsg);

	// Actualizo 'masterSwitch'
	sprintf(mqttMsg, "%d", myStrip.isMasterSwitchON());
	myMqttClient.publish("casa/" DEVICE_ID "/masterSwitch_refresh", mqttMsg);

	// Actualizo 'animationid'
	sprintf(mqttMsg, "%d", myEEPROMData.EEPROM_ANIMATION_ID_UNIT8_T);
	myMqttClient.publish("casa/" DEVICE_ID "/animationid_refresh", mqttMsg);

	// Actualizo 'bright1'
	sprintf(mqttMsg, "%d", myEEPROMData.EEPROM_BRIGHT1_INT8_T);
	myMqttClient.publish("casa/" DEVICE_ID "/bright1_refresh", mqttMsg);

	// Actualizo 'bright2'
	sprintf(mqttMsg, "%d", myEEPROMData.EEPROM_BRIGHT2_INT8_T);
	myMqttClient.publish("casa/" DEVICE_ID "/bright2_refresh", mqttMsg);

	// Actualizo 'interval'
	sprintf(mqttMsg, "%lu", myEEPROMData.EEPROM_INTERVAL_ULONG);
	myMqttClient.publish("casa/" DEVICE_ID "/interval_refresh", mqttMsg);

	// Actualizo 'color1'
	sprintf(mqttMsg, "%f", myEEPROMData.EEPROM_COLOR1_WAVELENGTH_FLOAT);
	myMqttClient.publish("casa/" DEVICE_ID "/color1_refresh", mqttMsg);

	// Actualizo 'color2'
	sprintf(mqttMsg, "%f", myEEPROMData.EEPROM_COLOR2_WAVELENGTH_FLOAT);
	myMqttClient.publish("casa/" DEVICE_ID "/color2_refresh", mqttMsg);

#ifdef USE_LDR
	// Actualizo 'ldr'
	sprintf(mqttMsg, "%d", myEEPROMData.EEPROM_LDR_MODE_UINT8_T);
	myMqttClient.publish("casa/" DEVICE_ID "/ldr_refresh", mqttMsg);

	// Actualizo 'ldr_fire_threshold'
	sprintf(mqttMsg, "%u", myEEPROMData.EEPROM_LDR_THRESHOLD_UINT16_T);
	myMqttClient.publish("casa/" DEVICE_ID "/ldr_fire_threshold_refresh", mqttMsg);
#endif

	// Actualizo campo "status Date" del Dashboard.
	if (isValidTime) {
		// Formateo la fecha.
		sprintf(mqttMsg, "Fecha: %02d/%02d/%04d  %02d:%02d:%02d", day(theTime), month(theTime), year(theTime), hour(theTime), minute(theTime), second(theTime));

		// Envío mensaje.
		myMqttClient.publish("casa/" DEVICE_ID "/status_date_refresh", mqttMsg);
	}
	else {
		// Envío mensaje.
		myMqttClient.publish("casa/" DEVICE_ID "/status_date_refresh", "La fecha y hora aun no son correctas");
	}

	// Actualizo campo "mqtt ID" del Dashboard.
	sprintf(mqttMsg, "Dispositivo: %s", mqttClientID);
	myMqttClient.publish("casa/" DEVICE_ID "/mqtt_ID_refresh", mqttMsg);

	// Actualizo campo "mqtt Last Op" del Dashboard
	sprintf(mqttMsg, "Last MQTT op: %s", mqttLastStateOpTxt);
	myMqttClient.publish("casa/" DEVICE_ID "/mqtt_Last_Op_refresh", mqttMsg);

	// Actualizo campo "WiFi Signal" del Dashboard
	sprintf(mqttMsg, "WiFi Signal: %ld dBm", wifiSignal);
	myMqttClient.publish("casa/" DEVICE_ID "/wifi_signal_refresh", mqttMsg);
}

