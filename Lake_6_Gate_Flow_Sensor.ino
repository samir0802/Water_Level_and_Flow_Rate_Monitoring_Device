#include <HardwareSerial.h>
//#include <SoftwareSerial.h>
#include <LittleFS.h> // File system library
#include <WiFi.h>     // Wifi handling library
#include <WiFiClientSecure.h> // To connect Wifi to internet



#define  INT16 1
#define  INT32 2
#define  FLOAT 3

#define ABCD 1
#define DCBA 2 
#define BADC 3 
#define CDAB 4

//---------------------------------------------------- Handeling Server Upload Parameters -----------------------------------------------//
WiFiClientSecure wifiClient;                              // Initializer for HTTPS network for google Sheets.
String Wifi_SSID; 
String Wifi_PASS;

long wifi_timeout_period_seconds = 5;                            // Timer to limit the wifi scan to only preset time
long wifi_disconnect_timer;                    // Timer to tract how much is device disconnected for.
long wifi_check_timer;                        // Timer to check the Wifi connectivity
int WifiDisconnectionCount = 0;

// bool variables to store condition of wifi connection
bool WifiConnected = false;

const char *host = "script.google.com";                 // Host URL
const int httpsPort = 443;                              // Connection Port of Https network
String Sheets_GAS_ID;  
String Default_apscriptId = "AKfycby6HkuBLtw84voSv7LFgJe3spFzeookhQOgtQeSO0Qe4ZVOEQuAgilT51psuF5KhnGg";
bool server_err = true;                                 // Server Status whether 'Okay' or 'Error'
uint server_error_counter;                              // Increment every time data upload to server fails
const uint16_t data_upload_interval_second = 15*60;     // Time in Seconds when data will be uploaded to google sheets. 
const uint8_t no_recv_counter_limit = 4;                // At this threashold value, sensor error will be displayed
const uint8_t server_error_counter_limit = 8;           // Threashold at which the esp will restart

void send_to_modbus();

/*.Modbus settings....................................*/
HardwareSerial modbus(2);

const int modbus_RX = 3;     //3                        // Custom SCL Pin for ADXL sensor
const int modbus_TX = 8;     //2                        // Custom SDA Pin for ADXL sensor

const int TTL_RX = 4;
const int TTL_TX = 5;

HardwareSerial TTL_Serial(3);
String encoded_string;                              //string for parsed value from modbus

const int status_led = 35;        // Built in LED for Heltecwireless stick V3. 

// For Canal Height retreval
#include <Preferences.h> 
Preferences preferences;
float flow_height;
const int numof_Parameter = 9;
String column_names[numof_Parameter] = {"_device_id", "_device_name", "_gate_status1", "_gate_status2", "_gate_status3", 
                                                                      "_gate_status4", "_gate_status5", "_gate_status6", "_water_height"};
String Server_Sensor_values[numof_Parameter];           // Float array to store paramters value

int sensor_height_offset_mm;  // The distance between the sensor head and the base of the canal in 'mm'
const uint8_t modbusCommand[] = {0x01, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85, 0xF6}; // Modbus command for ulltrasonic Sensor
const int buttonPin = 0;

bool status_led_state = false; 
void blink_led(int interval = 100);
long currentTime;
int Device_ID;

/*---------------------------Input pins for Gates----------------------------*/
const int GATE_PINS[6] = {35,36,37,38,39,40};    //39,40,41,42,45,46
bool gateOpen[6]; // array to save the state of each gate
byte gateStateBinary[6]; // arrary to save gate states in binary format "01" for open "10 for close"

void setup() {
  pinMode(status_led, OUTPUT); 
  pinMode(buttonPin, INPUT_PULLUP);
  // Initialize the limit switch pins as inputs with pull-down resistors
  for (int i = 0; i < 6; i++) {
    pinMode(GATE_PINS[i], INPUT);
  }


  analogReadResolution(12);                                       // Set ADC resolution to 12 bits (0-4095)
  Serial.begin(115200);                                           // Serial monitor
  modbus.begin(9600, SERIAL_8N1, modbus_RX, modbus_TX);           // 3rd Hardware UART for Modbus Module
  TTL_Serial.begin(9600,SERIAL_8N1, TTL_RX,TTL_TX);

  long startTime = millis();
  // displayStatusLED(network_connected, gprsConnected);
  Serial.print("[Info] Bootup, Type 'AT' to get config menu");
  printInstruction();
  
  while(millis() - startTime < 20000){
    config_via_serial();
    blink_led(50);
    delay(50);
    Serial.print(". ");
  }
  delay(1000);
    //Data retreval from Flash
  preferences.begin("canal_flow");
  //sensor_height_offset_mm = preferences.getInt("sensor_height", 0);// set the sensor height offset  default value 0
  Device_ID = preferences.getInt("device_id", 101);
  Wifi_SSID = preferences.getString("wifi_ssid","TP-LINK_A5AFEA");
  Wifi_PASS = preferences.getString("wifi_pass","79641494");
  preferences.end();   

   //--------------------------------------Wifi Login Part--------------------------------------------------------//
    WiFi.mode(WIFI_STA);
    Serial.print("\nConnecting to:" + Wifi_SSID);
    Serial.println("\tPass: " + Wifi_PASS);
    long wifi_timeout = millis();
    WiFi.begin(Wifi_SSID.c_str(), Wifi_PASS.c_str());

    while (WiFi.status() != WL_CONNECTED && (millis() - wifi_timeout <= wifi_timeout_period_seconds * 1000)) {
      blink_led();
      Serial.print(".");
      delay(10);
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\nWiFi Connection Failed");
      WifiDisconnectionCount++;
      Serial.println("Wifi Disconnection Count: "); Serial.println(WifiDisconnectionCount);
      if(WifiDisconnectionCount > 5){
        wifi_disconnect_timer = millis();
      }
    } else {
      Serial.println("\nWiFi Connected");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      WifiConnected = true;
       
    }
    Sheets_GAS_ID = Default_apscriptId;
 currentTime = millis();
}

void loop() {
  // put your main code here, to run repeatedly: 
  float sensor_value; 
  String rs485_message = "";
  sensor_value = 0;
  const int sample_count = 10;
  // Try getting sensor data once again  if fails  
      if(millis() - currentTime >= 5000){
        currentTime =millis();
        encoded_string = handle_modbus(modbusCommand);

        if(encoded_string != "") {
          // sensor_value =0.9*sensor_value + 0.1*(parse_sensor_value(encoded_string, "Total Height", "mm", 1, 1));

          sensor_value = parse_sensor_value(encoded_string, "Total Height", "mm", 1, 1);
          //Serial.printf("Sensor value: %f \n", sensor_value);
          sensor_value /= sample_count;
          flow_height = sensor_value;

          send_to_serial(String(flow_height));
          long response_wait_timer = millis();
          String responseData_String = "";
          String appScriptURL = "https://script.google.com/macros/s/AKfycby6HkuBLtw84voSv7LFgJe3spFzeookhQOgtQeSO0Qe4ZVOEQuAgilT51psuF5KhnGg/exec?";

          while(millis() - response_wait_timer < 2000){
            if(TTL_Serial.available()){
              String receivedResponse_for_HTTP_request = Serial.readStringUntil('\n');
              receivedResponse_for_HTTP_request.trim();
              if (receivedResponse_for_HTTP_request.startsWith(appScriptURL)) {
                responseData_String = receivedResponse_for_HTTP_request.substring(appScriptURL.length());
                responseData_String.trim(); 
                Serial.print("Data responded from Android: "); Serial.println(responseData_String);

                if(WiFi.status() == WL_CONNECTED){
                  // collect_server_Sending_data(String(flow_height));
                  connect_google_server(responseData_String);
                }else{
                  Serial.print("[Info] Reconnecting to: " + Wifi_SSID);
                  Serial.println("\tPass: " + Wifi_PASS);
                  WiFi.begin(Wifi_SSID.c_str(), Wifi_PASS.c_str());
                  delay(2000);
                  connect_google_server(responseData_String);
                }
              }else{
                Serial.println(".............Unwanted Response from Android..........");
              }
            }else{
              delay(10);
            }
          }
          
        }else { 
        Serial.println("[Warning] Modbus Reply Error");
            // flow_height = random(100, 1000);

            //   if(millis() - currentTime >= 5000){
            //     // send_to_serial(String(sensor_value));
            //     currentTime =millis();
            //     if(WiFi.status() == WL_CONNECTED){
            //       collect_server_Sending_data(String(flow_height));
            //       connect_google_server();
            //     }else{
            //       Serial.print("[Info] Reconnecting to:" + Wifi_SSID);
            //       Serial.println("\tPass: " + Wifi_PASS);
            //       WiFi.begin(Wifi_SSID.c_str(), Wifi_PASS.c_str());
            //       delay(2000);
            //     }
            //   }
        }


      }
  config_via_serial();
  delay(100);
}

void printInstruction(){
  Serial.print(F("\n-----------------------------------------------------------------------------------------------------\n"));
  Serial.print(F(" Modbus Type: Modbus RTU\n"));
  Serial.print(F(" Baud rate: 9600\n"));
  Serial.print(F(" Bit: 8\n"));
  Serial.print(F(" Parity: None\n"));
  Serial.print(F(" Stop Bit: 1\n"));
  Serial.print(F(" Suppported Sensor: A02/A20 Ultrasonic Distance Sensor\n\n"))     ;
  Serial.println("");
  
  // Serial.print(F("----------- List of Serial Commands ------------\n"));
  Serial.print("\n--------------- To Change Sensor Parameters-----------\n");
  // Serial.print("Type 'sensor_height=" + String(sensor_height_offset_mm) + "' to set the sensor height from weir to sensor head in millimeter\n");
  Serial.print("Type 'device_id=" + String(Device_ID) + "' to Set the Device Id for this device to communicate with Server\n");
  Serial.print("\t Supported ID: 101, 102, 103, ...\n");
  Serial.print("Type 'wifi_ssid=-wifi_id-' to set the wifi ssid.\n");
  Serial.print("Type 'wifi_pass=-wifi-password-' to set wifi password. \n");
}

void config_via_serial(){
   if (Serial.available()) {
   preferences.begin("canal_flow");
  //  String setsensor_height = "sensor_height=";
   String device_id = "device_id=";
   String wifi_id ="wifi_ssid=";
   String wifi_pw ="wifi_pass=";
   String receivedData = Serial.readStringUntil('\n');

    if (receivedData.startsWith("hello") || receivedData.startsWith("HELLO") || receivedData.startsWith("AT") || receivedData.startsWith("at") || receivedData.startsWith("ramlaxman") || receivedData.startsWith("RAMLAXMAN")) {
      printInstruction();
   
    // } else if (receivedData.startsWith(setsensor_height)){
    //   String data = receivedData.substring(setsensor_height.length());
    //   int value = data.toInt();
    //   preferences.putInt("sensor_height", value);
    //   sensor_height_offset_mm = value;
    //   preferences.end();
    //   Serial.println("Saved successfully...\t\t Sensor height: " + String(value));
    
    }
     else if (receivedData.startsWith(device_id)){
      String data = receivedData.substring(device_id.length());
      data.trim(); // removes whites spaces
      int value = data.toInt();
      preferences.putInt("device_id", value);
      Device_ID = value;
      preferences.end();
      Serial.println("\nSaved successfully...\t\t Device ID: " + String(value));
    }
    else if (receivedData.startsWith(wifi_id)){
      String data =  receivedData.substring(wifi_id.length());
      data.trim();
      data.replace(" ", "");  // Remove any spaces
      preferences.putString("wifi_ssid", data);
      preferences.end();
      Serial.println("\nSaved successfully...\t\t Wifi SSID:" + data);
    }
    else if (receivedData.startsWith(wifi_pw)){
      String data =  receivedData.substring(wifi_pw.length());
      data.trim();
      data.replace(" ", "");  // Remove any spaces
      preferences.putString("wifi_pass", data);
      preferences.end();
      Serial.println("\nSaved successfully...\t\t Wifi PASS:" + data);
    }
  }
}

//Limit switch state detect function
void readLimitSwitches() {
  // Read the state of each limit switch
  for (int i = 0; i < 6; i++) {
    gateOpen[i] = digitalRead(GATE_PINS[i]) == LOW;
  }
} 

// encode binary bits to gate states
void encodeGateStates(){
  byte gateState;
  for(int i = 0; i < 6; i++){
    if(gateOpen[i]){
      gateState = B11;
    }
    else{
      gateState = B00;
    }
    gateStateBinary[i] = gateState;
  }
}
float parse_sensor_value(String recv_data, String name, String unit, uint8_t data_type, uint8_t data_format){ 
  
    //Converting into Float 32
    /* Start at i=6 because, first 3Bytes(6Char) data doesnot contain Value
    * [i < received_data.length() - 4] because last 2Bytes (4 Char) Checksum also doesnot contain Usable Data.
    * first convert from String to Hex with Base 16
    * Then Pass the value to make Unsigned Long.
    * Copy and intrepret as float so that needed value is displayed. 
    * The For loop jumps 8 step to count 4 bytes
    */
    float calc_value_float;
    int32_t calc_value_int;
    int data_size_bytes; 

    if(data_type == INT16)
     data_size_bytes =2;
    else if (data_type == INT32)
      data_size_bytes = 4;
    else if (data_type == FLOAT)
      data_size_bytes = 4;

    // example Modbus reply: 01 03 02 00 00 00 00 AA BB
    // removing initial non-important chars, '01 03 02'
    recv_data = recv_data.substring(6);


    // SWAP bytes to Correct the proper formatting of the data from any other Data formats to ABCD
    if (data_format == DCBA && data_type != INT16) { 
         recv_data = recv_data.substring(6, 8) + recv_data.substring(4, 6) + recv_data.substring(2, 4) + recv_data.substring(0, 2) + recv_data.substring(8);
      
      } else if (data_format == BADC ) {
          if(data_type != INT16)
          recv_data = recv_data.substring(2, 4) + recv_data.substring(0, 2) + recv_data.substring(6, 8) + recv_data.substring(4, 6) + recv_data.substring(8);
          else recv_data = recv_data.substring(2, 4) + recv_data.substring(0, 2) + recv_data.substring(4);

      } else if (data_format == CDAB && data_type != INT16) {
          recv_data = recv_data.substring(4, 8) + recv_data.substring(0, 4) + recv_data.substring(8);
      
      } else if (data_format == ABCD){
          // Already in ABCD format, no need to change anything
      } else {
        // Serial.println("[Warning] Error Data Format");
        return 0.0;
      }
    String byteString = recv_data.substring(0, data_size_bytes*2);
    unsigned long hexValue = strtoul(byteString.c_str(), NULL, 16);

    // Copy the rearranged bytes into calc_value_float
    // Serial.print("\n" + name + ": ");
    if(data_type == INT16){
    memcpy(&calc_value_int, &hexValue, sizeof(int32_t));
    // Serial.print(String(calc_value_int) + " "); 
    // Serial.println(unit);
    return float(calc_value_int);
    }

    else if(data_type == INT32){
    memcpy(&calc_value_int, &hexValue, sizeof(int32_t));
    // Serial.print(String(calc_value_int) + " "); 
    // Serial.println(unit);
    return float(calc_value_int);
    }

    else if(data_type == FLOAT){
    memcpy(&calc_value_float, &hexValue, sizeof(float));
    // Serial.print(String(calc_value_float) + " "); 
    // Serial.println(unit);
    return calc_value_float;
    }
}
String handle_modbus(const uint8_t modbusCommand[8]){ 
  
  String received_array;
  unsigned long listen_startTime;        // Start this timer, once the hex command is sent through Modbus
  const int Serial_timeoutMs = 500;      // Modbus Reply Timeout in MS

   for(int i = 0; i<8; i++)
  {
    modbus.write(modbusCommand[i]);
  }

   Serial.println("\n[Info] Hex Sending Successful");
  listen_startTime = millis();

    // wait untill data is received or timeout period
    while (!modbus.available() && (millis() - listen_startTime < Serial_timeoutMs))
    {
      // Serial.print(".");
      delay(50);
      yield(); 
    }

    if (!modbus.available())  //If no data received, exit function
    {
      Serial.println("\n[Info] Timeout Waiting for data...\n");
      received_array = "";
      return received_array;
    }

      while (modbus.available()) {
      char receivedChar = modbus.read();
          Serial.print("0x");
      if (receivedChar < 0x10) {
          Serial.print("0");  // Print leading zero for single-digit hexadecimal values
        received_array += "0";
      }
        
      Serial.print(receivedChar, HEX);
      received_array = received_array + String(receivedChar,HEX);
      Serial.print(" ");
    }
 //----------------------------------------- Modbus Data validation -------------------------------------//
    int reply_len = received_array.substring(4,6).toInt();
    Serial.println("\n\tReply len: " + String((reply_len*2)+10) );
    if(received_array.length() != (reply_len*2)+10) {
      Serial.print("\nInvalid Data received!...");
      return "";
    }  
    Serial.print("\nModbus Receiving Success!, Data: " + received_array);
    return received_array; 
}
void send_to_serial(String message){
  
  // functions to check the gate open/close state
  readLimitSwitches();
  encodeGateStates();
  
  String rs485_message = "#";
  for(int i=0; i<6; i ++){
    if(gateStateBinary[i] == 3){
      rs485_message += "1";
    }
    else{
      rs485_message += "0";
    }
  }

  rs485_message += ","+ message;
  TTL_Serial.println(rs485_message);
  Serial.println(rs485_message);
}
void blink_led(int interval){
  static long blink_led_timer; 
  
  if(millis()-blink_led_timer > interval){
    status_led_state = !status_led_state;
    digitalWrite(status_led, status_led_state);
    blink_led_timer = millis();
  }
}
void collect_server_Sending_data(String water_level){
    // functions to check the gate open/close state
    readLimitSwitches();
    encodeGateStates();
    int Gate_State[6];
    for(int i=0; i<6; i ++){
    if(gateStateBinary[i] == 3){
      Gate_State[i] = 1;
    }
    else{
      Gate_State[i] = 0;
    }
  }
    Server_Sensor_values[0] = Device_ID;
    Server_Sensor_values[1] = "Canal_Flow_Sensor";
    Server_Sensor_values[2] = String(Gate_State[0]);
    Server_Sensor_values[3] = String(Gate_State[1]);
    Server_Sensor_values[4] = String(Gate_State[2]);
    Server_Sensor_values[5] = String(Gate_State[3]);
    Server_Sensor_values[6] = String(Gate_State[4]);
    Server_Sensor_values[7] = String(Gate_State[5]);
    Server_Sensor_values[8] = water_level;
}
void connect_google_server(String value_description){ //Establish a Connection with the Server

  wifiClient.setInsecure(); // Set the HTPPS handshake to insecure, fingerprint exchaneg and SSL certificate exchange are other methods
  Serial.println("\n[Info] Connecting to Server: " + String(host));
  if (!wifiClient.connect(host, httpsPort))
  {
    Serial.println("[Warning] WiFi Connection Failed!!");
    server_error_counter ++; //Flag as server error; 
    server_err = true; 
    return;
  }
  value_description = "";
  // String url = "/macros/s/" + Sheets_GAS_ID + "/exec?_device_id=" + Device_ID + "&";
  String url = "/macros/s/" + Sheets_GAS_ID + "/exec?";
  url.reserve(350);

  // for (int i = 0; i < numof_Parameter; i++)
  // {
  //   value_description += column_names[i] + "=" + Server_Sensor_values[i];
  //   if (i != numof_Parameter - 1)
  //   value_description = value_description + "&";
  // }
  // // value_description += column_names[8] + "=" + Server_Sensor_values[8] + "&";



  url = url + value_description;
 

  Serial.print("[Info] Server Post Requesting URL: ");
  Serial.print(host);
  Serial.println(url);

  wifiClient.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" + //      "User-Agent: BuildFailureDetectorESP8266\r\n" +
               "Connection: close\r\n\r\n");             

  Serial.println("[Info] Server Data Post request sent...");
  String line;
  while (wifiClient.connected())
  {
      line += wifiClient.readStringUntil('\n');
    if (wifiClient.read() == '\r')
    {
      break;
    }
  }

  wifiClient.stop();
  if (line.indexOf("HTTP/1.1 200 OK") || line.indexOf("HTTP/1.1 302 Moved Temporarily") != -1)
    Serial.println("[Info] ESP32 Posting successfull!");
  else
    Serial.println("[Warning] esp32/Data posting failed");
 
  Serial.println("[Info] closing connection\n");
  server_err = false;
  server_error_counter = 0; // Reset the server_error COunter to 0
}