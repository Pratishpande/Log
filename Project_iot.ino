
#include <WiFi.h>
#include <PubSubClient.h>

#define RXD2 16
#define TXD2 17
const char* ssid = "Note";
const char* password = "12345678";
const char* mqttServer = "192.168.43.149";
const int mqttPort = 1883;
const char* clientID = "techmotes";
const char* channelName1 = "cdac/agri/moisture";
char* moisture;
WiFiClient MQTTclient;
PubSubClient client(MQTTclient);

long lastReconnectAttempt = 0;

boolean reconnect()
{
   if(client.connect(clientID)) {
      client.subscribe(channelName1);
      Serial.println("Subscribed");
     
   }
  return client.connected();
}


void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);

Serial.println("Attempting to connect");
WiFi.begin(ssid,password);

if(WiFi.waitForConnectResult() != WL_CONNECTED)
{

   Serial.println("couldn't connect to wifi");
}
client.setServer(mqttServer , mqttPort);
Serial.println("Connected to broker");

Serial2.begin(115200, SERIAL_8N1,RXD2,TXD2); 

lastReconnectAttempt = 0;


}

void loop() {
  // put your main code here, to run repeatedly:
if(!client.connected())
{
  long now = millis();
  if(now - lastReconnectAttempt > 5000) {

    lastReconnectAttempt = now;
    if(reconnect()) {
      lastReconnectAttempt = 0; 
    }
  }
  
}
else
{
  client.loop();
  
  char ch[17]="";  
   
    
  while(Serial2.available())
  {
    int i;
    int j=0;
   
 

  for(i=0;i<17;i++)
  {
  
  ch[i]= char(Serial2.read());
  
  }

  ch[13]='\0';
  ch[14]='\r';
  ch[15]='\n';
  
   }
  Serial.println(ch);  

    
   client.publish(channelName1,String(ch).c_str());
   

    Serial.println("Message Published");
    delay(1000);
   
  
}

}
