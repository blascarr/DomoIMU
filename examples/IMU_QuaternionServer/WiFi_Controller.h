void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.println(WiFi.localIP());
}

void initWebServer() {
  // Handle Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/html", index_html);
  });

  /*server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
    });*/



  //server.serveStatic("/", SPIFFS, "/");

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetX", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetY", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetZ", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", "OK");
  });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient * client) {
    if (client->lastId()) {
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();
}


String getGyroReadings() {

  readings["yaw"] = String( (float)( mpu.ypr[0] * 180) / M_PI );
  readings["roll"] = String( (float)( mpu.ypr[1] * 180) / M_PI );
  readings["pitch"] = String( (float)( mpu.ypr[2] * 180) / M_PI );

  readings["qw"] = String( mpu.q.w );
  readings["qx"] = String( mpu.q.x );
  readings["qy"] = String( mpu.q.y );
  readings["qz"] = String( mpu.q.z );
  
  String jsonString = JSON.stringify(readings);
  return jsonString;
}

void serialReadings() {
  Serial.print("ypr\t");
 //Serial.print(mpu.ypr.x * 180 / M_PI);
  Serial.print("\t");
  //Serial.print(mpu.ypr.y * 180 / M_PI);
  Serial.print("\t");
  //Serial.print(mpu.ypr.z * 180 / M_PI);
  Serial.print("\t\t");
  Serial.print(mpu.q.w * 180 / M_PI);
  Serial.print("\t");
  Serial.print(mpu.q.x * 180 / M_PI);
  Serial.print("\t");
  Serial.print(mpu.q.y * 180 / M_PI);
  Serial.print("\t");
  Serial.print(mpu.q.z * 180 / M_PI);
  Serial.println();
}
