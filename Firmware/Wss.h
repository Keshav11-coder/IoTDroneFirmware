class Wss {
  private:
    const char* ssid = "Prekesrad";
    const char* password =  "pR3K3SR@D^";

    AsyncWebServer server(80);
    AsyncWebSocket ws("/ws");

  public:
    void configureWss() {
      WiFi.begin(ssid, password);

      while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi..");
      }

      Serial.println(WiFi.localIP());
    }

    void initiateCallback() {
      server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
        request->send(SPIFFS, "/index.html", String(), false);
      });

      ws.onEvent(onWsEvent);
      server.addHandler(&ws);

      server.begin();
    }

    void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len) {
      if (type == WS_EVT_CONNECT) {

        Serial.println("Websocket client connection received");

      } else if (type == WS_EVT_DISCONNECT) {
        Serial.println("Client disconnected");
        Serial.println("-----------------------");

      } else if (type == WS_EVT_DATA) {
        char* received_data = (char*) data;
        String data_string = (String)received_data;
        //    Serial.println(data_string.substring(0, 12));
        String jds = data_string.substring(0, 12);

        if (data_string == "m_start") {
          mot_start(1100);
        }
        else if ((received_data[0]) == 'j') {
          //      Serial.println(data_str);
          String data_str = jds;
          if (data_str.substring(0, 3) == "jl_") {
            String temp_data_str = data_str;
            temp_data_str.remove(0, 3); // remove 'jl_'

            char arr[temp_data_str.length() + 1];
            temp_data_str.toCharArray(arr, sizeof(arr));

            char* xStr = strtok(arr, " ");
            char* yStr = strtok(NULL, " ");

            int x = atoi(xStr);
            int y = atoi(yStr);

            int tempThrottle = map(y, 1000, 2000, -100, 100);
            ThrottleIncrement = tempThrottle;
            //        Serial.println(mrc[0]);
            //        Serial.println(data_str);

            //        Serial.print(x);
            //        Serial.print(" x--y ");
            //        Serial.println(y);
          }

          else if (data_str.substring(0, 3) == "jr_") {
            String temp_data_str = data_str;
            temp_data_str.remove(0, 3); // remove 'jr_'

            char arr[temp_data_str.length() + 1];
            temp_data_str.toCharArray(arr, sizeof(arr));

            char* xStr = strtok(arr, " ");
            char* yStr = strtok(NULL, " ");

            int x = atoi(xStr);
            int y = atoi(yStr);

            int pitch_in = map(y, 2000, 1000, -7, 7);
            int roll_in = map(x, 2000, 1000, -7, 7);

            rollIncrement = roll_in;
            pitchIncrement = pitch_in;
            desired_angle_y = pitch_in;
            desired_angle_x = roll_in;

            //        Serial.println(x);
            //        ThrottleIncrement = tempThrottle;
            //        Serial.println(mrc[0]);
            //        Serial.println(data_str);

            //        Serial.print(x);
            //        Serial.print(" x--y ");
            //        Serial.println(y);
          }

          else if (data_str.substring(0, 3) == "je_") {
            //      Serial.println(data_string);
            //      ems();
            FailSafe = true;
          }
        }
      }
    }
};
