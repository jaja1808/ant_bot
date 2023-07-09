#include <ESP8266WiFi.h>
#include <WiFiClient.h>

// Wi-Fi credentials
const char* ssid = "iPhone";
const char* password = "mehdi1719";


// Server details
WiFiServer server(80);

void setup() {
  Serial.begin(115200);
 
  // Connect to Wi-Fi network
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
 
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
 
  // Start the server
  server.begin();
 
  Serial.println("Server started");
}

void loop() {
  WiFiClient client = server.available();
 
  if (client) {
    Serial.println("New client connected");
    while (client.connected()) {
      if (client.available()) {
        String request = client.readStringUntil('\r');
        Serial.println(request);
       
        // Process the request here
       
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("");
        client.println("Response from ESP8266");
        client.println("");
        break;
      }
    }
   
    delay(10);
    client.stop();
    Serial.println("Client disconnected");
  }
}
