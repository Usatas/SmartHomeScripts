#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define DHTPIN 2     // what digital pin the DHT22 is conected to
#define DHTTYPE DHT22   // there are multiple kinds of DHT sensors
/* Define types of sensors. */
static const uint8_t DHT11{11};  /**< DHT TYPE 11 */
static const uint8_t DHT12{12};  /**< DHY TYPE 12 */
static const uint8_t DHT21{21};  /**< DHT TYPE 21 */
static const uint8_t DHT22{22};  /**< DHT TYPE 22 */
static const uint8_t AM2301{21}; /**< AM2301 */

uint8_t DHTbuf[5];

//WLAN Login
char* ssid = "WLANName";
char* password = "WLANPasswort";

// Add your MQTT Broker IP address:
char* mqtt_server = "Mosquitto IP";
const int mqtt_port = 1883;
const char* mqtt_user = "Mosquitto Username";
const char* mqtt_password = "Mosquitto Passwort";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

float temperature = 0;
float humidity = 0;
float heatIndex =0;

// LED Pin
const int ledPin =  LED_BUILTIN;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2000);
  // Wait for serial to initialize.
  while(!Serial) { }
 // dht.begin();
 
  setup_wifi();
 client.setServer(mqtt_server, mqtt_port);//MQTT Server, - Port
  client.setCallback(callback);

  pinMode(ledPin, OUTPUT);
  Serial.println("Device Started");
  Serial.println("-------------------------------------");
  Serial.println("Running DHT!");
  Serial.println("-------------------------------------");

}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid); 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic Sensoren/Badezimmer/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "Sensoren/Badezimmer/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
  
  if (client.connect("ESP8266Client" , mqtt_user, mqtt_password)) {
      // Subscribe
      client.subscribe("Sensoren/Badezimmer/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

int timeSinceLastRead = 0;

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  // Report every 2 seconds.
  if(timeSinceLastRead > 2000) {
     if (!readDHT()) {
      Serial.println("Failed to read from DHT sensor!");
      timeSinceLastRead = 0;
      return;
    }
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    humidity = readHumidity();
    // Read temperature as Celsius (the default)
    temperature = readTemperature();

    // Check if any reads failed and exit early (to try again).
   

    // Compute heat index in Fahrenheit (the default)
  //  float hif = dht.computeHeatIndex(f, h);
    // Compute heat index in Celsius (isFahreheit = false)
   heatIndex = computeHeatIndex(temperature, humidity);

    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" *C ");
    Serial.print("Heat index: ");
    Serial.print(heatIndex);
    Serial.print(" *C \n ");

    
    // Convert the value to a char array
    char tempString[8];
    dtostrf(temperature, 1, 2, tempString);
    client.publish("Sensoren/Badezimmer/temperature", tempString);

    // Convert the value to a char array
    char humString[8];
    dtostrf(humidity, 1, 2, humString);
    client.publish("Sensoren/Badezimmer/humidity", humString);
    
    // Convert the value to a char array
    char hicString[8];
    dtostrf(heatIndex, 1, 2, hicString);
    client.publish("Sensoren/Badezimmer/heatIndex", hicString);

    timeSinceLastRead = 0;
  }
  delay(100);
  timeSinceLastRead += 100;
}

float readTemperature() {
  float f = NAN;

  if (true) {
    switch (DHTTYPE) {
    case DHT11:
      f = DHTbuf[2];
      if (DHTbuf[3] & 0x80) {
        f = -1 - f;
      }
      f += (DHTbuf[3] & 0x0f) * 0.1;

      break;
    case DHT12:
      f = DHTbuf[2];
      f += (DHTbuf[3] & 0x0f) * 0.1;
      if (DHTbuf[2] & 0x80) {
        f *= -1;
      }
      break;
    case DHT22:
    case DHT21:
      f = ((word)(DHTbuf[2] & 0x7F)) << 8 | DHTbuf[3];
      f *= 0.1;
      if (DHTbuf[2] & 0x80) {
        f *= -1;
      }
      break;
    }
  }
  return f;
}

float readHumidity() {
  float f = NAN;
  if (true) {
    switch (DHTTYPE) {
    case DHT11:
    case DHT12:
      f = DHTbuf[0] + DHTbuf[1] * 0.1;
      break;
    case DHT22:
    case DHT21:
      f = ((word)DHTbuf[0]) << 8 | DHTbuf[1];
      f *= 0.1;
      break;
    }
  }
  return f;
}

 
float computeHeatIndex(float temperature, float percentHumidity) {
  float hi;
  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) +
              (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 + 2.04901523 * temperature + 10.14333127 * percentHumidity +
         -0.22475541 * temperature * percentHumidity +
         -0.00683783 * pow(temperature, 2) +
         -0.05481717 * pow(percentHumidity, 2) +
         0.00122874 * pow(temperature, 2) * percentHumidity +
         0.00085282 * temperature * pow(percentHumidity, 2) +
         -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if ((percentHumidity < 13) && (temperature >= 80.0) &&
        (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) *
            sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if ((percentHumidity > 85.0) && (temperature >= 80.0) &&
             (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  return hi;
}

uint8_t error;

uint8_t readDHT(){
  error=false;
  pinMode(DHTPIN,OUTPUT);
  digitalWrite(DHTPIN, LOW); 
  delay(2);
  digitalWrite(DHTPIN, HIGH); 
  delayMicroseconds(40);
  pinMode(DHTPIN,INPUT_PULLUP);
  waitLow();
  if(error){     //is sensor pulling low
    Serial.println("DHT not present");
    return false;             //no, so return
  }
  uint8_t i,j;
  waitHigh();
  waitLow();
  for(i=0;i<5;i++){
    for(j=0;j<8;j++){
      waitHigh();
      delayMicroseconds(40);
      DHTbuf[i]<<=1;
      if(digitalRead(DHTPIN)==1)
        DHTbuf[i]++;
      waitLow();
    }
  }
  uint8_t sum = DHTbuf[0] + DHTbuf[1] + DHTbuf[2] + DHTbuf[3];
  if (DHTbuf[4] != sum) 
    return false;
  return !error;
}

void waitLow(){
  if(error)
    return;
  uint16_t timeout=1000;
  while(digitalRead(DHTPIN)==1 && --timeout>0);
  if(timeout>0)
    return;
  error=true;
}

void waitHigh(){
  uint16_t timeout=1000;
  if(error)
    return;
  while(digitalRead(DHTPIN)==0 && --timeout>0);
  if(timeout>0)
    return;
  error=true;
}
