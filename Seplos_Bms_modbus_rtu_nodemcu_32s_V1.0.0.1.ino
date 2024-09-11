#include <SoftwareSerial.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>  
#include <WiFi.h>
#include <EEPROM.h>

// RS485 setup with ESp32
#define RE 32  // Connect RE terminal with 32 of ESP
#define DE 33    // Connect DE terminal with 33 of ESP  
#define PACK_ZERO 0x00

//A:0x00, 0x04, 0x10, 0x00, 0x00, 0x12, 0x75, 0x16 // Total Voltage, Soc, Am, Temp
//B:0x00, 0x04, 0x11, 0x00, 0x00, 0x1a, 0x75, 0x2c // Voltage of cell
//C:0x00, 0x01, 0x12, 0x00, 0x00, 0x90, 0x38, 0xcf // Alarm

//C --> B --> A --> C
const byte ModReadBuffer_Pia[] = {PACK_ZERO, 0x04, 0x11, 0x00, 0x00, 0x1a, 0x75, 0x2c};  
const byte ModReadBuffer_Pib[] = {PACK_ZERO, 0x01, 0x12, 0x00, 0x00, 0x90, 0x38, 0xcf}; 
const byte ModReadBuffer_Pic[] = {PACK_ZERO, 0x04, 0x10, 0x00, 0x00, 0x12, 0x75, 0x16}; 

byte BufferValue_Pia[65]; //byte number 36
byte BufferValue_Pib[65]; //byte number 52
byte BufferValue_Pic[65]; //byte number 18

bool on_data_pia = true;
bool on_data_pib = true;
bool on_data_pic = true;

bool on_check_pia = false;
bool on_check_pib = false;
bool on_check_pic = false;

SoftwareSerial mod(26, 27); // RX=26 , TX =27

const char* mqtt_to_pia_1 = "deizzem/pia_1";
const char* mqtt_to_pia_2 = "deizzem/pia_2";

const char* mqtt_to_pib_1 = "deizzem/pib_1";
const char* mqtt_to_pib_2 = "deizzem/pib_2";
const char* mqtt_to_pib_3 = "deizzem/pib_3";

const char* mqtt_to_pic_1 = "deizzem/pic_1";
const char* mqtt_to_pic_2 = "deizzem/pic_2";
const char* mqtt_to_pic_3 = "deizzem/pic_3";

const char* mqtt_to_tb02_1 = "deizzem/pic_tb02_1";
const char* mqtt_to_tb02_2 = "deizzem/pic_tb02_2";

const char* mqtt_to_tb03_1 = "deizzem/pic_tb03_1";
const char* mqtt_to_tb03_2 = "deizzem/pic_tb03_2";

const char* mqtt_to_tb04_1 = "deizzem/pic_tb04_1";
const char* mqtt_to_tb04_2 = "deizzem/pic_tb04_2";

const char* mqtt_to_tb05_1 = "deizzem/pic_tb05_1";
const char* mqtt_to_tb05_2 = "deizzem/pic_tb05_2";

const char* mqtt_to_tb06 = "deizzem/pic_tb06";

const char* mqtt_to_tb07 = "deizzem/pic_tb07";

const char* mqtt_to_tb08_1 = "deizzem/pic_tb08_1";
const char* mqtt_to_tb08_2 = "deizzem/pic_tb08_2";

const char* mqtt_to_tb09 = "deizzem/pic_tb09";

const char* mqtt_to_tb15_1 = "deizzem/pic_tb15_1";
const char* mqtt_to_tb15_2 = "deizzem/pic_tb15_2";

const char* mqtt_to_tb16 = "deizzem/pic_tb16";

WiFiClient espClient;
PubSubClient client(espClient);

// Parameters to be configured
char mqtt_server[40];
int mqtt_port;
char mqtt_user[20];
char mqtt_password[20];

// EEPROM addresses for saving and retrieving MQTT parameters
#define EEPROM_ADDR_MQTT_SERVER 0
#define EEPROM_ADDR_MQTT_PORT 50
#define EEPROM_ADDR_MQTT_USER 100
#define EEPROM_ADDR_MQTT_PASSWORD 150

const int button_reset = 0;
const int led_reset = 2;
int buttonState;

unsigned long period = 5000; 
unsigned long last_time = 0;

void setup() {

  //reset_wifi();

  Serial.begin(19200);
  mod.begin(19200, SWSERIAL_8N1);// modbus configuration
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);

  pinMode(led_reset, OUTPUT);
  pinMode(button_reset, INPUT);

  // Load the configuration from EEPROM
  loadConfigData();

  WiFiManager wifiManager;

  // Connect to WiFi using WiFiManager
  WiFiManagerParameter custom_mqtt_server("mqtt_server", "MQTT Server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("mqtt_port", "MQTT Port", String(mqtt_port).c_str(), 6);
  WiFiManagerParameter custom_mqtt_user("mqtt_user", "MQTT User", mqtt_user, 20);
  WiFiManagerParameter custom_mqtt_password("mqtt_password", "MQTT Password", mqtt_password, 20);

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_password);
   
  // Set the configuration portal timeout to 5 minutes
  wifiManager.setConfigPortalTimeout(300);
  
  digitalWrite(led_reset, HIGH);
  wifiManager.autoConnect("Deizzem Smart Home");
  digitalWrite(led_reset, LOW);

  // Get the values from the custom parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  mqtt_port = atoi(custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_password, custom_mqtt_password.getValue());

  // Save the configuration to EEPROM
  saveConfigData();

  // Set up MQTT
  client.setServer(mqtt_server, mqtt_port);

  Serial.println("Connected to Wi-Fi");
  Serial.println("MQTT Server: " + String(mqtt_server));
  Serial.println("MQTT Port: " + String(mqtt_port));
  Serial.println("MQTT User: " + String(mqtt_user));
  Serial.println("MQTT Password: " + String(mqtt_password));

}

void loop() {
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  byte val;   

  while (mod.write(ModReadBuffer_Pia, sizeof(ModReadBuffer_Pia)) == 8){
    val = ModbusData_Pia(); 
    Serial.println("#############################################END PIA#############################################");
    val = ModbusData_Pib();
    Serial.println("#############################################END PIB#############################################");
    val = ModbusData_Pic();
    Serial.println("#############################################END PIC#############################################");   
    
    reset_wifi();
  }

}

/////////////////////////////////////////////////////////PIA//////////////////////////////////////////////////////

byte ModbusData_Pia(){
  byte i;
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);

  delay(1000);
  
  if(mod.write(ModReadBuffer_Pia,sizeof(ModReadBuffer_Pia)) == 8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);

    for(i = 0; i < 65; i++){
      BufferValue_Pia[i] = mod.read();   
    }
    
    if (BufferValue_Pia[3] != 255 && BufferValue_Pia[4] != 255 && BufferValue_Pia[5] != 255 && BufferValue_Pia[6] != 255 && BufferValue_Pia[7] != 255){

      Serial.println("Serial Data received On:");
      Serial.print("Modbus Buffer PIA = [");

      for (int n = 0; n < 65; n++) {
        Serial.print(n);
        Serial.print(":");
        Serial.print(BufferValue_Pia[n]);

        if (n < 65 - 1) {
          Serial.print(", ");
        }
      }

      Serial.println("]");

      /*Serial.print("Modbus Hex PIA = [");
      for (int n = 0; n < 65; n++) {
        Serial.print(n);
        Serial.print(":");
        Serial.print(BufferValue_Pia[n], HEX);
        if (n < 65 - 1) {
          Serial.print(", ");
        }
      }

      Serial.println("]");*/
      
      if(on_data_pia == true){
        float packVoltage = convert_bytes_to_data("UINT16", BufferValue_Pia[3], BufferValue_Pia[4]) * 0.01;
        float Current = convert_bytes_to_data("INT16", BufferValue_Pia[5], BufferValue_Pia[6]) * 0.01;
        float RemainingCapacity = convert_bytes_to_data("UINT16", BufferValue_Pia[7], BufferValue_Pia[8]) * 0.01;
        float TotalCapacity = convert_bytes_to_data("UINT16", BufferValue_Pia[9], BufferValue_Pia[10]) * 0.01;
        float TotalDischargeCapacity = convert_bytes_to_data("UINT16", BufferValue_Pia[11], BufferValue_Pia[12]) * 0.1;
        float SOC = convert_bytes_to_data("UINT16", BufferValue_Pia[13], BufferValue_Pia[14]) * 0.1;
        float SOH = convert_bytes_to_data("UINT16", BufferValue_Pia[15], BufferValue_Pia[16]) * 0.1;
        float Cycle = convert_bytes_to_data("UINT16", BufferValue_Pia[17], BufferValue_Pia[18] * 1);
        float AvgCellVoltage = convert_bytes_to_data("UINT16", BufferValue_Pia[19], BufferValue_Pia[20]) * 0.001;
        float AvgCellTemperature = convert_bytes_to_data("UINT16", BufferValue_Pia[21], BufferValue_Pia[22]) * 0.1 - 273.1;
        float MaxCellVoltage = convert_bytes_to_data("UINT16", BufferValue_Pia[23], BufferValue_Pia[24]) * 0.001;
        float MinCellVoltage = convert_bytes_to_data("UINT16", BufferValue_Pia[25], BufferValue_Pia[26]) * 0.001;
        float MaxCellTemperature = convert_bytes_to_data("UINT16", BufferValue_Pia[27], BufferValue_Pia[28]) * 0.1 - 273.1;
        float MinCellTemperature = convert_bytes_to_data("UINT16", BufferValue_Pia[29], BufferValue_Pia[30]) * 0.1 - 273.1;
        float MaxChargeCurrent = convert_bytes_to_data("UINT16", BufferValue_Pia[33], BufferValue_Pia[34]) * 1;
        float MaxDischargeCurrent = convert_bytes_to_data("UINT16", BufferValue_Pia[35], BufferValue_Pia[36]) * 1;
        
        // Create a JSON object
        DynamicJsonDocument doc_pia_1(1024);
        DynamicJsonDocument doc_pia_2(1024);
        DynamicJsonDocument doc(1024);
        
        doc_pia_1["packVoltage"] = packVoltage;
        doc_pia_1["Current"] = Current;
        doc_pia_1["RemainingCapacity"] = RemainingCapacity;
        doc_pia_1["TotalCapacity"] = TotalCapacity;
        doc_pia_1["TotalDischargeCapacity"] = TotalDischargeCapacity;
        doc_pia_1["SOC"] = SOC;
        doc_pia_1["SOH"] = SOH;
        doc_pia_1["Cycle"] = Cycle;
        doc_pia_1["AvgCellVoltage"] = AvgCellVoltage;
        doc_pia_1["AvgCellTemperature"] = AvgCellTemperature;

        doc_pia_2["MaxCellVoltage"] = MaxCellVoltage;
        doc_pia_2["MinCellVoltage"] = MinCellVoltage;
        doc_pia_2["MaxCellTemperature"] = MaxCellTemperature;
        doc_pia_2["MinCellTemperature"] = MinCellTemperature;
        doc_pia_2["MaxChargeCurrent"] = MaxChargeCurrent;
        doc_pia_2["MaxDischargeCurrent"] = MaxDischargeCurrent;

        // Serialize the JSON object to a String
        String jsonString_pia_1;
        String jsonString_pia_2;
        serializeJson(doc_pia_1, jsonString_pia_1);
        serializeJson(doc_pia_2, jsonString_pia_2);
        Serial.println();

        if (client.connected()) {
            Serial.println("MQTT connected");
            client.publish(mqtt_to_pia_1, jsonString_pia_1.c_str());
            client.publish(mqtt_to_pia_2, jsonString_pia_2.c_str());
        } else {
            Serial.println("MQTT not connected");
            reconnect();
        }

        // Print processed data
        //** over 65535
        if(packVoltage < 654 && TotalCapacity < 654 && on_check_pia == true){
          // Print the JSON string to Serial monitor
          Serial.println("JSON String PIA:");
          Serial.println(jsonString_pia_1);
          Serial.println(jsonString_pia_2);

          Serial.println("Pack Voltage (V): " + String(packVoltage));
          Serial.println("Current (A): " + String(Current));
          Serial.println("Remaining Capacity (Ah): " + String(RemainingCapacity));
          Serial.println("Total Capacity (Ah): " + String(TotalCapacity));
          Serial.println("Total Discharge Capacity (Ah): " + String(TotalDischargeCapacity));
          Serial.println("SOC (%): " + String(SOC));
          Serial.println("SOH (%): " + String(SOH));
          Serial.println("Cycle: " + String(Cycle));
          Serial.println("Avg Cell Voltage (V): " + String(AvgCellVoltage));
          Serial.println("Avg Cell Temperature (°C): " + String(AvgCellTemperature));
          Serial.println("Max Cell Voltage (V): " + String(MaxCellVoltage));
          Serial.println("Min Cell Voltage (V): " + String(MinCellVoltage));
          Serial.println("Max Cell Temperature (°C): " + String(MaxCellTemperature));
          Serial.println("Min Cell Temperature (°C): " + String(MinCellTemperature));
          Serial.println("Max Charge Current (A): " + String(MaxChargeCurrent));
          Serial.println("Max Discharge Current (A): " + String(MaxDischargeCurrent));           
        }
      }   
    }
  }

  return BufferValue_Pia[0];

}

/////////////////////////////////////////////////////////PIB/////////////////////////////////////////////////////

byte ModbusData_Pib(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);  

  delay(1000);

  byte i;
  
  if(mod.write(ModReadBuffer_Pib,sizeof(ModReadBuffer_Pib)) == 8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);

    for(i = 0; i < 65; i++){
      BufferValue_Pib[i] = mod.read();
    }

    if (BufferValue_Pib[3] != 255 && BufferValue_Pib[5] != 255 && BufferValue_Pib[7] != 255 && BufferValue_Pib[9] != 255 && BufferValue_Pib[11] != 255){

      Serial.println("Serial Data received On:");
      Serial.print("Modbus Buffer PIB = [");

      for (int n = 0; n < 65; n++) {
        Serial.print(n);
        Serial.print(":");
        Serial.print(BufferValue_Pib[n]);

        if (n < 65 - 1) {
          Serial.print(", ");
        }
      }

      Serial.println("]");

      /*Serial.print("Modbus Hex PIB = [");
      for (int n = 0; n < 65; n++) {
        Serial.print(n);
        Serial.print(":");
        Serial.print(BufferValue_Pib[n], HEX);
        if (n < 65 - 1) {
          Serial.print(", ");
        }
      }

      Serial.println("]");*/

      if(on_data_pib == true){
        float Cell_1_Voltage = convert_bytes_to_data("UINT16", BufferValue_Pib[3], BufferValue_Pib[4]) * 0.001;
        float Cell_2_Voltage = convert_bytes_to_data("UINT16", BufferValue_Pib[5], BufferValue_Pib[6]) * 0.001;
        float Cell_3_Voltage = convert_bytes_to_data("UINT16", BufferValue_Pib[7], BufferValue_Pib[8]) * 0.001;
        float Cell_4_Voltage = convert_bytes_to_data("UINT16", BufferValue_Pib[9], BufferValue_Pib[10]) * 0.001;
        float Cell_5_Voltage = convert_bytes_to_data("UINT16", BufferValue_Pib[11], BufferValue_Pib[12]) * 0.001;
        float Cell_6_Voltage = convert_bytes_to_data("UINT16", BufferValue_Pib[13], BufferValue_Pib[14]) * 0.001;
        float Cell_7_Voltage = convert_bytes_to_data("UINT16", BufferValue_Pib[15], BufferValue_Pib[16]) * 0.001;
        float Cell_8_Voltage = convert_bytes_to_data("UINT16", BufferValue_Pib[17], BufferValue_Pib[18]) * 0.001;
        float Cell_9_Voltage = convert_bytes_to_data("UINT16", BufferValue_Pib[19], BufferValue_Pib[20]) * 0.001;
        float Cell_10_Voltage = convert_bytes_to_data("UINT16", BufferValue_Pib[21], BufferValue_Pib[22]) * 0.001;
        float Cell_11_Voltage = convert_bytes_to_data("UINT16", BufferValue_Pib[23], BufferValue_Pib[24]) * 0.001;
        float Cell_12_Voltage = convert_bytes_to_data("UINT16", BufferValue_Pib[25], BufferValue_Pib[26]) * 0.001;
        float Cell_13_Voltage = convert_bytes_to_data("UINT16", BufferValue_Pib[27], BufferValue_Pib[28]) * 0.001;
        float Cell_14_Voltage = convert_bytes_to_data("UINT16", BufferValue_Pib[29], BufferValue_Pib[30]) * 0.001;
        float Cell_15_Voltage = convert_bytes_to_data("UINT16", BufferValue_Pib[31], BufferValue_Pib[32]) * 0.001;
        float Cell_16_Voltage = convert_bytes_to_data("UINT16", BufferValue_Pib[33], BufferValue_Pib[34]) * 0.001;
        float Cell_Temperature_1 = convert_bytes_to_data("UINT16", BufferValue_Pib[35], BufferValue_Pib[36]) * 0.1 - 273.1;
        float Cell_Temperature_2 = convert_bytes_to_data("UINT16", BufferValue_Pib[37], BufferValue_Pib[38]) * 0.1 - 273.1;
        float Cell_Temperature_3 = convert_bytes_to_data("UINT16", BufferValue_Pib[39], BufferValue_Pib[40]) * 0.1 - 273.1;
        float Cell_Temperature_4 = convert_bytes_to_data("UINT16", BufferValue_Pib[41], BufferValue_Pib[42]) * 0.1 - 273.1;
        float AmbientTemperature = convert_bytes_to_data("UINT16", BufferValue_Pib[51], BufferValue_Pib[52]) * 0.1 - 273.1;
        float PowerTemperature = convert_bytes_to_data("UINT16", BufferValue_Pib[53], BufferValue_Pib[54]) * 0.1 - 273.1;        

        // Create a JSON object
        DynamicJsonDocument doc_pib_1(1024);
        DynamicJsonDocument doc_pib_2(1024);
        DynamicJsonDocument doc_pib_3(1024);
        DynamicJsonDocument doc(1024);
        
        doc_pib_1["Cell1Voltage"] = Cell_1_Voltage;
        doc_pib_1["Cell2Voltage"] = Cell_2_Voltage;
        doc_pib_1["Cell3Voltage"] = Cell_3_Voltage;
        doc_pib_1["Cell4Voltage"] = Cell_4_Voltage;
        doc_pib_1["Cell5Voltage"] = Cell_5_Voltage;
        doc_pib_1["Cell6Voltage"] = Cell_6_Voltage;
        doc_pib_1["Cell7Voltage"] = Cell_7_Voltage;
        doc_pib_1["Cell8Voltage"] = Cell_8_Voltage;

        doc_pib_2["Cell9Voltage"] = Cell_9_Voltage;
        doc_pib_2["Cell10Voltage"] = Cell_10_Voltage;
        doc_pib_2["Cell11Voltage"] = Cell_11_Voltage;
        doc_pib_2["Cell12Voltage"] = Cell_12_Voltage;
        doc_pib_2["Cell13Voltage"] = Cell_13_Voltage;
        doc_pib_2["Cell14Voltage"] = Cell_14_Voltage;
        doc_pib_2["Cell15Voltage"] = Cell_15_Voltage;
        doc_pib_2["Cell16Voltage"] = Cell_16_Voltage;

        doc_pib_3["CellTemperature1"] = Cell_Temperature_1;
        doc_pib_3["CellTemperature2"] = Cell_Temperature_2;
        doc_pib_3["CellTemperature3"] = Cell_Temperature_3;
        doc_pib_3["CellTemperature4"] = Cell_Temperature_4;
        doc_pib_3["AmbientTemperature"] = AmbientTemperature;
        doc_pib_3["PowerTemperature"] = PowerTemperature;

        // Serialize the JSON object to a String
        String jsonString_pib_1;
        String jsonString_pib_2;
        String jsonString_pib_3;

        serializeJson(doc_pib_1, jsonString_pib_1);
        serializeJson(doc_pib_2, jsonString_pib_2);
        serializeJson(doc_pib_3, jsonString_pib_3);

        Serial.println();

        if (client.connected()) {
            Serial.println("MQTT connected");
            client.publish(mqtt_to_pib_1, jsonString_pib_1.c_str());
            client.publish(mqtt_to_pib_2, jsonString_pib_2.c_str());
            client.publish(mqtt_to_pib_3, jsonString_pib_3.c_str());
        } else {
            Serial.println("MQTT not connected");
            reconnect();
        }

        if(Cell_1_Voltage > 0 && Cell_1_Voltage < 4 && on_check_pib == true){
          // Print the JSON string to Serial monitor
          Serial.println("JSON String PIB:");
          Serial.println(jsonString_pib_1);
          Serial.println(jsonString_pib_2);
          Serial.println(jsonString_pib_3);

          Serial.println("Cell1 Voltage (V): " + String(Cell_1_Voltage)); 
          Serial.println("Cell2 Voltage (V): " + String(Cell_2_Voltage)); 
          Serial.println("Cell3 Voltage (V): " + String(Cell_3_Voltage)); 
          Serial.println("Cell4 Voltage (V): " + String(Cell_4_Voltage)); 
          Serial.println("Cell5 Voltage (V): " + String(Cell_5_Voltage)); 
          Serial.println("Cell6 Voltage (V): " + String(Cell_6_Voltage)); 
          Serial.println("Cell7 Voltage (V): " + String(Cell_7_Voltage)); 
          Serial.println("Cell8 Voltage (V): " + String(Cell_8_Voltage)); 
          Serial.println("Cell9 Voltage (V): " + String(Cell_9_Voltage)); 
          Serial.println("Cell10 Voltage (V): " + String(Cell_10_Voltage)); 
          Serial.println("Cell11 Voltage (V): " + String(Cell_11_Voltage)); 
          Serial.println("Cell12 Voltage (V): " + String(Cell_12_Voltage)); 
          Serial.println("Cell13 Voltage (V): " + String(Cell_13_Voltage)); 
          Serial.println("Cell14 Voltage (V): " + String(Cell_14_Voltage)); 
          Serial.println("Cell15 Voltage (V): " + String(Cell_15_Voltage)); 
          Serial.println("Cell16 Voltage (V): " + String(Cell_16_Voltage)); 
          Serial.println("Cell Temperature 1 (°C): " + String(Cell_Temperature_1));
          Serial.println("Cell Temperature 2 (°C): " + String(Cell_Temperature_2));
          Serial.println("Cell Temperature 3 (°C): " + String(Cell_Temperature_3));
          Serial.println("Cell Temperature 4 (°C): " + String(Cell_Temperature_4));
          Serial.println("Ambient Temperature (°C): " + String(AmbientTemperature));
          Serial.println("Power Temperature (°C): " + String(PowerTemperature));           
        }
      }
    }

  }

  return BufferValue_Pib[0];

}

/////////////////////////////////////////////////////////PIC//////////////////////////////////////////////////////

byte ModbusData_Pic(){
  byte i;
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);

  delay(1000);
  
  if(mod.write(ModReadBuffer_Pic,sizeof(ModReadBuffer_Pic)) == 8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);

    for(i = 0; i < 65; i++){
      BufferValue_Pic[i] = mod.read();
    }

    if (BufferValue_Pic[3] != 255 && BufferValue_Pic[4] != 255 && BufferValue_Pic[5] != 255 && BufferValue_Pic[6] != 255 && BufferValue_Pic[7] != 255){

      Serial.println("Serial Data received On:");
      Serial.print("Modbus Buffer PIC = [");

      for (int n = 0; n < 65; n++) {
        Serial.print(n);
        Serial.print(":");
        Serial.print(BufferValue_Pic[n]);

        if (n < 65 - 1) {
          Serial.print(", ");
        }

      }

      Serial.println("]");

      /*Serial.print("Modbus Hex PIC = [");
      for (int n = 0; n < 65; n++) {
        Serial.print(n);
        Serial.print(":");
        Serial.print(BufferValue_Pic[n], HEX);
        if (n < 65 - 1) {
          Serial.print(", ");
        }
      }

      Serial.println("]");*/
      
      int VoltageEventCode = BufferValue_Pic[12];
      int CellsTemperatureEventCode = BufferValue_Pic[13];
      int EnvironmentAndPowerTemperatureEventCode = BufferValue_Pic[14];
      int CurrentEventCode1 = BufferValue_Pic[15];
      int TheResidualCapacityCode = BufferValue_Pic[17];
      int TheFETEventCode = BufferValue_Pic[18];
      int BatteryEqualizationStateCode = BufferValue_Pic[19];
      int SystemStateCode = BufferValue_Pic[11];
      int HardFaultEventCode = BufferValue_Pic[20];
      int CurrentEventCode2 = BufferValue_Pic[16];

      /////////////////////////////////////////////////////////JSON//////////////////////////////////////////////////////

      if(on_data_pic == true){   
        String Cells_Voltage_08_01_Low_Alarm_State = decToHex(BufferValue_Pic[3]);
        String Cells_Voltage_16_09_Low_Alarm_State = decToHex(BufferValue_Pic[4]);
        String Cells_Voltage_08_01_high_Alarm_State = decToHex(BufferValue_Pic[5]);
        String Cells_Voltage_16_09_high_Alarm_State = decToHex(BufferValue_Pic[6]);
        String Cell_08_01_Temperature_Low_Alarm_State = decToHex(BufferValue_Pic[7]);
        String Cell_08_01_Temperature_High_Alarm_State = decToHex(BufferValue_Pic[8]);
        String Cell_08_01_Equalization_Event_Code = decToHex(BufferValue_Pic[9]);
        String Cell_16_09_Equalization_Event_Code = decToHex(BufferValue_Pic[10]);
        String System_State_Code = decToHex(BufferValue_Pic[11]);
        String Voltage_Event_Code = decToHex(BufferValue_Pic[12]);
        String Cells_Temperature_Event_Code = decToHex(BufferValue_Pic[13]);
        String Environment_and_Power_Temperature_Event_Code = decToHex(BufferValue_Pic[14]);
        String Current_Event_Code_1 = decToHex(BufferValue_Pic[15]);
        String Current_Event_Code_2 = decToHex(BufferValue_Pic[16]);
        String The_Residual_Capacity_Code = decToHex(BufferValue_Pic[17]);
        String The_FET_Event_Code = decToHex(BufferValue_Pic[18]);
        String Battery_Equalization_State_Code = decToHex(BufferValue_Pic[19]);
        String Hard_Fault_Event_Code = decToHex(BufferValue_Pic[20]);

        // Create a JSON object
        DynamicJsonDocument doc_pic_1(1024);
        DynamicJsonDocument doc_pic_2(1024);
        DynamicJsonDocument doc_pic_3(1024);

        DynamicJsonDocument doc_pic_tb02_1(1024);
        DynamicJsonDocument doc_pic_tb02_2(1024);

        DynamicJsonDocument doc_pic_tb03_1(1024);
        DynamicJsonDocument doc_pic_tb03_2(1024);

        DynamicJsonDocument doc_pic_tb04_1(1024);
        DynamicJsonDocument doc_pic_tb04_2(1024);

        DynamicJsonDocument doc_pic_tb05_1(1024);
        DynamicJsonDocument doc_pic_tb05_2(1024);

        DynamicJsonDocument doc_pic_tb06(1024);

        DynamicJsonDocument doc_pic_tb07(1024);

        DynamicJsonDocument doc_pic_tb08_1(1024);
        DynamicJsonDocument doc_pic_tb08_2(1024);

        DynamicJsonDocument doc_pic_tb09(1024);

        DynamicJsonDocument doc_pic_tb15_1(1024);
        DynamicJsonDocument doc_pic_tb15_2(1024);

        DynamicJsonDocument doc_pic_tb16(1024);

        DynamicJsonDocument doc(1024);

        doc_pic_1["CellsVoltage08-01LowAlarmState"] = Cells_Voltage_08_01_Low_Alarm_State;
        doc_pic_1["CellsVoltage16-09LowAlarmState"] = Cells_Voltage_16_09_Low_Alarm_State;
        doc_pic_1["CellsVoltage08-01highAlarmState"] = Cells_Voltage_08_01_high_Alarm_State;
        doc_pic_1["CellsVoltage16-09highAlarmState"] = Cells_Voltage_16_09_high_Alarm_State;
        doc_pic_1["Cell08-01TemperatureLowAlarmState"] = Cell_08_01_Temperature_Low_Alarm_State;
        doc_pic_1["Cell08-01TemperatureHighAlarmState"] = Cell_08_01_Temperature_High_Alarm_State;

        doc_pic_2["Cell08-01EqualizationEventCode"] = Cell_08_01_Equalization_Event_Code;
        doc_pic_2["Cell16-09EqualizationEventCode"] = Cell_16_09_Equalization_Event_Code;
        doc_pic_2["SystemStateCode"] = System_State_Code;
        doc_pic_2["VoltageEventCode"] = Voltage_Event_Code;
        doc_pic_2["CellsTemperatureEventCode"] = Cells_Temperature_Event_Code;
        doc_pic_2["EnvironmentAndPowerTemperatureEventCode"] = Environment_and_Power_Temperature_Event_Code;

        doc_pic_3["CurrentEventCode1"] = Current_Event_Code_1;
        doc_pic_3["CurrentEventCode2"] = Current_Event_Code_2;
        doc_pic_3["TheResidualCapacityCode"] = The_Residual_Capacity_Code;
        doc_pic_3["TheFETEventCode"] = The_FET_Event_Code;
        doc_pic_3["BatteryEqualizationStateCode"] = Battery_Equalization_State_Code;
        doc_pic_3["HardFaultEventCode"] = Hard_Fault_Event_Code;  

        // Add states to the JSON object
        doc_pic_tb02_1["CellHighVoltageAlarm"] = checkAndPrintState(VoltageEventCode, 0, "Cell high voltage alarm");
        doc_pic_tb02_1["CellOverVoltageProtect"] = checkAndPrintState(VoltageEventCode, 1, "Cell over voltage protect");
        doc_pic_tb02_1["CellLowVoltageAlarm"] = checkAndPrintState(VoltageEventCode, 2, "Cell low voltage alarm");
        doc_pic_tb02_1["CellUnderVoltageProtection"] = checkAndPrintState(VoltageEventCode, 3, "Cell under voltage protection");
        doc_pic_tb02_1["PackHighVoltageAlarm"] = checkAndPrintState(VoltageEventCode, 4, "Pack high voltage alarm");
        doc_pic_tb02_1["PackOverVoltageProtect"] = checkAndPrintState(VoltageEventCode, 5, "Pack over voltage protect");
        doc_pic_tb02_2["PackLowVoltageAlarm"] = checkAndPrintState(VoltageEventCode, 6, "Pack low voltage alarm");
        doc_pic_tb02_2["PackUnderVoltageProtection"] = checkAndPrintState(VoltageEventCode, 7, "Pack under voltage protection");

        doc_pic_tb03_1["ChargeHighTemperatureAlarm"] = checkAndPrintState(CellsTemperatureEventCode, 0, "Charge high temperature alarm");
        doc_pic_tb03_1["ChargeOverTemperatureProtection"] = checkAndPrintState(CellsTemperatureEventCode, 1, "Charge over temperature protection");
        doc_pic_tb03_1["ChargeLowTemperatureAlarm"] = checkAndPrintState(CellsTemperatureEventCode, 2, "Charge low temperature alarm");
        doc_pic_tb03_1["ChargeUnderTemperatureProtection"] = checkAndPrintState(CellsTemperatureEventCode, 3, "Charge under temperature protection");
        doc_pic_tb03_1["DischargeHighTemperatureAlarm"] = checkAndPrintState(CellsTemperatureEventCode, 4, "Discharge high temperature alarm");
        doc_pic_tb03_1["DischargeOverTemperatureProtection"] = checkAndPrintState(CellsTemperatureEventCode, 5, "Discharge over temperature protection");
        doc_pic_tb03_2["DischargeLowTemperatureAlarm"] = checkAndPrintState(CellsTemperatureEventCode, 6, "Discharge low temperature alarm");
        doc_pic_tb03_2["DischargeUnderTemperatureProtection"] = checkAndPrintState(CellsTemperatureEventCode, 7, "Discharge under temperature protection");

        doc_pic_tb04_1["HighEnvironmentTemperatureAlarm"] = checkAndPrintState(EnvironmentAndPowerTemperatureEventCode, 0, "High environment temperature alarm");
        doc_pic_tb04_1["OverEnvironmentTemperatureProtection"] = checkAndPrintState(EnvironmentAndPowerTemperatureEventCode, 1, "Over environment temperature protection");
        doc_pic_tb04_1["LowEnvironmentTemperatureAlarm"] = checkAndPrintState(EnvironmentAndPowerTemperatureEventCode, 2, "Low environment temperature alarm");
        doc_pic_tb04_1["UnderEnvironmentTemperatureProtection"] = checkAndPrintState(EnvironmentAndPowerTemperatureEventCode, 3, "Under environment temperature protection");
        doc_pic_tb04_1["HighPowerTemperatureAlarm"] = checkAndPrintState(EnvironmentAndPowerTemperatureEventCode, 4, "High Power temperature alarm");
        doc_pic_tb04_2["OverPowerTemperatureProtection"] = checkAndPrintState(EnvironmentAndPowerTemperatureEventCode, 5, "Over Power temperature protection");
        doc_pic_tb04_2["CellTemperatureLowHeating"] = checkAndPrintState(EnvironmentAndPowerTemperatureEventCode, 6, "Cell temperature low heating");

        doc_pic_tb05_1["ChargeCurrentAlarm"] = checkAndPrintState(CurrentEventCode1, 0, "Charge current alarm");
        doc_pic_tb05_1["ChargeOverCurrentProtection"] = checkAndPrintState(CurrentEventCode1, 1, "Charge over current protection");
        doc_pic_tb05_1["ChargeSecondLevelCurrentProtection"] = checkAndPrintState(CurrentEventCode1, 2, "Charge second level current protection");
        doc_pic_tb05_1["DischargeCurrentAlarm"] = checkAndPrintState(CurrentEventCode1, 3, "Discharge current alarm");
        doc_pic_tb05_1["DischargeOverCurrentProtection"] = checkAndPrintState(CurrentEventCode1, 4, "Discharge over current protection");
        doc_pic_tb05_2["DischargeSecondLevelOverCurrentProtection"] = checkAndPrintState(CurrentEventCode1, 5, "Discharge second level over current protection");
        doc_pic_tb05_2["OutputShortCircuitProtection"] = checkAndPrintState(CurrentEventCode1, 6, "Output short circuit protection");

        doc_pic_tb06["SocAlarm"] = checkAndPrintState(TheResidualCapacityCode, 2, "Soc alarm");
        doc_pic_tb06["SocProtection"] = checkAndPrintState(TheResidualCapacityCode, 3, "Soc protection");
        doc_pic_tb06["CellDiffAlarm"] = checkAndPrintState(TheResidualCapacityCode, 4, "Cell Diff alarm");

        doc_pic_tb07["DischargeFETOn"] = checkAndPrintState(TheFETEventCode, 0, "Discharge FET on");
        doc_pic_tb07["ChargeFETOn"] = checkAndPrintState(TheFETEventCode, 1, "Charge FET on");
        doc_pic_tb07["CurrentLimitingFETOn"] = checkAndPrintState(TheFETEventCode, 2, "Current limiting FET on");
        doc_pic_tb07["HeatingOn"] = checkAndPrintState(TheFETEventCode, 3, "Heating on");

        doc_pic_tb08_1["LowSocAlarm"] = checkAndPrintState(BatteryEqualizationStateCode, 0, "low Soc alarm");
        doc_pic_tb08_1["IntermittentCharge"] = checkAndPrintState(BatteryEqualizationStateCode, 1, "Intermittent charge");
        doc_pic_tb08_1["ExternalSwitchControl"] = checkAndPrintState(BatteryEqualizationStateCode, 2, "External switch control");
        doc_pic_tb08_1["StaticStandbyAndSleepMode"] = checkAndPrintState(BatteryEqualizationStateCode, 3, "Static standby and sleep mode");
        doc_pic_tb08_1["HistoryDataRecording"] = checkAndPrintState(BatteryEqualizationStateCode, 4, "History data recording");
        doc_pic_tb08_1["UnderSocProtect"] = checkAndPrintState(BatteryEqualizationStateCode, 5, "Under Soc protect");
        doc_pic_tb08_2["AcktiveLimitedCurrent"] = checkAndPrintState(BatteryEqualizationStateCode, 6, "Acktive-Limited Current");
        doc_pic_tb08_2["PassiveLimitedCurrent"] = checkAndPrintState(BatteryEqualizationStateCode, 7, "Passive-Limited Current");

        doc_pic_tb09["Discharge"] = checkAndPrintState(SystemStateCode, 0, "Discharge");
        doc_pic_tb09["Charge"] = checkAndPrintState(SystemStateCode, 1, "Charge");
        doc_pic_tb09["FloatingCharge"] = checkAndPrintState(SystemStateCode, 2, "Floating charge");
        doc_pic_tb09["FullCharge"] = checkAndPrintState(SystemStateCode, 3, "Full charge");
        doc_pic_tb09["StandbyMode"] = checkAndPrintState(SystemStateCode, 4, "Standby mode");
        doc_pic_tb09["TurnOff"] = checkAndPrintState(SystemStateCode, 5, "Turn off");

        doc_pic_tb15_1["NTCFault"] = checkAndPrintState(HardFaultEventCode, 0, "NTC Fault");
        doc_pic_tb15_1["AFEFault"] = checkAndPrintState(HardFaultEventCode, 1, "AFE Fault");
        doc_pic_tb15_1["ChargeMosfetsFault"] = checkAndPrintState(HardFaultEventCode, 2, "Charge Mosfets Fault");
        doc_pic_tb15_1["DischargeMosfetsFault"] = checkAndPrintState(HardFaultEventCode, 3, "Discharge Mosfets Fault");
        doc_pic_tb15_1["CellFault"] = checkAndPrintState(HardFaultEventCode, 4, "Cell Faul");
        doc_pic_tb15_1["BreakLineFault"] = checkAndPrintState(HardFaultEventCode, 5, "Break Line Fault");
        doc_pic_tb15_2["KeyFault"] = checkAndPrintState(HardFaultEventCode, 6, "Key Fault");
        doc_pic_tb15_2["AerosolAlarm"] = checkAndPrintState(HardFaultEventCode, 7, "Aerosol Alarm");

        doc_pic_tb16["OutputShortLatchUp"] = checkAndPrintState(CurrentEventCode2, 0, "Output short latch up");
        doc_pic_tb16["SecondChargeLatchUp"] = checkAndPrintState(CurrentEventCode2, 2, "Second Charge latch up");
        doc_pic_tb16["SecondDischargeLatchUp"] = checkAndPrintState(CurrentEventCode2, 3, "Second Discharge latch up");

        // Serialize the JSON object to a String
        String jsonString_pic_1;
        String jsonString_pic_2;
        String jsonString_pic_3;

        String jsonString_pic_tb02_1;
        String jsonString_pic_tb02_2;

        String jsonString_pic_tb03_1;
        String jsonString_pic_tb03_2;

        String jsonString_pic_tb04_1;
        String jsonString_pic_tb04_2;

        String jsonString_pic_tb05_1;
        String jsonString_pic_tb05_2;

        String jsonString_pic_tb06;

        String jsonString_pic_tb07;

        String jsonString_pic_tb08_1;
        String jsonString_pic_tb08_2;

        String jsonString_pic_tb09;

        String jsonString_pic_tb15_1;
        String jsonString_pic_tb15_2;

        String jsonString_pic_tb16;

        serializeJson(doc_pic_1, jsonString_pic_1);
        serializeJson(doc_pic_2, jsonString_pic_2);
        serializeJson(doc_pic_3, jsonString_pic_3);

        serializeJson(doc_pic_tb02_1, jsonString_pic_tb02_1);
        serializeJson(doc_pic_tb02_2, jsonString_pic_tb02_2);

        serializeJson(doc_pic_tb03_1, jsonString_pic_tb03_1);
        serializeJson(doc_pic_tb03_2, jsonString_pic_tb03_2);

        serializeJson(doc_pic_tb04_1, jsonString_pic_tb04_1);
        serializeJson(doc_pic_tb04_2, jsonString_pic_tb04_2);

        serializeJson(doc_pic_tb05_1, jsonString_pic_tb05_1);
        serializeJson(doc_pic_tb05_2, jsonString_pic_tb05_2);

        serializeJson(doc_pic_tb06, jsonString_pic_tb06);

        serializeJson(doc_pic_tb07, jsonString_pic_tb07);

        serializeJson(doc_pic_tb08_1, jsonString_pic_tb08_1);
        serializeJson(doc_pic_tb08_2, jsonString_pic_tb08_2);

        serializeJson(doc_pic_tb09, jsonString_pic_tb09);

        serializeJson(doc_pic_tb15_1, jsonString_pic_tb15_1);
        serializeJson(doc_pic_tb15_2, jsonString_pic_tb15_2);

        serializeJson(doc_pic_tb16, jsonString_pic_tb16);

        Serial.println();

        if (client.connected()) {
            Serial.println("MQTT connected");
            client.publish(mqtt_to_pic_1, jsonString_pic_1.c_str());
            client.publish(mqtt_to_pic_2, jsonString_pic_2.c_str());
            client.publish(mqtt_to_pic_3, jsonString_pic_3.c_str());

            client.publish(mqtt_to_tb02_1, jsonString_pic_tb02_1.c_str());
            client.publish(mqtt_to_tb02_2, jsonString_pic_tb02_2.c_str());

            client.publish(mqtt_to_tb03_1, jsonString_pic_tb03_1.c_str());
            client.publish(mqtt_to_tb03_2, jsonString_pic_tb03_2.c_str());

            client.publish(mqtt_to_tb04_1, jsonString_pic_tb04_1.c_str());
            client.publish(mqtt_to_tb04_2, jsonString_pic_tb04_2.c_str());

            client.publish(mqtt_to_tb05_1, jsonString_pic_tb05_1.c_str());
            client.publish(mqtt_to_tb05_2, jsonString_pic_tb05_2.c_str());

            client.publish(mqtt_to_tb06, jsonString_pic_tb06.c_str());

            client.publish(mqtt_to_tb07, jsonString_pic_tb07.c_str());

            client.publish(mqtt_to_tb08_1, jsonString_pic_tb08_1.c_str());
            client.publish(mqtt_to_tb08_2, jsonString_pic_tb08_2.c_str());

            client.publish(mqtt_to_tb09, jsonString_pic_tb09.c_str());

            client.publish(mqtt_to_tb15_1, jsonString_pic_tb15_1.c_str());
            client.publish(mqtt_to_tb15_2, jsonString_pic_tb15_2.c_str());

            client.publish(mqtt_to_tb16, jsonString_pic_tb16.c_str());

        } else {
            Serial.println("MQTT not connected");
            reconnect();
        }

        if(on_check_pic == true){
          // Print the JSON string to Serial monitor
          Serial.println("JSON String PIC:");
          Serial.println(jsonString_pic_1);
          Serial.println(jsonString_pic_2);
          Serial.println(jsonString_pic_3);

          Serial.println(jsonString_pic_tb02_1);
          Serial.println(jsonString_pic_tb02_2);

          Serial.println(jsonString_pic_tb03_1);
          Serial.println(jsonString_pic_tb03_2);

          Serial.println(jsonString_pic_tb04_1);
          Serial.println(jsonString_pic_tb04_2);

          Serial.println(jsonString_pic_tb05_1);
          Serial.println(jsonString_pic_tb05_2);

          Serial.println(jsonString_pic_tb06);

          Serial.println(jsonString_pic_tb07);

          Serial.println(jsonString_pic_tb08_1);
          Serial.println(jsonString_pic_tb08_2);

          Serial.println(jsonString_pic_tb09);

          Serial.println(jsonString_pic_tb15_1);
          Serial.println(jsonString_pic_tb15_2);

          Serial.println(jsonString_pic_tb16);

          /*Battery Cell 8 - Cell 1
            Battery Cell 16 - Cell 9*/

          Serial.println("Cells Voltage 08-01 Low Alarm: " + Cells_Voltage_08_01_Low_Alarm_State);
          Serial.println("Cells Voltage 16-09 Low Alarm State: " + Cells_Voltage_16_09_Low_Alarm_State);
          Serial.println("Cells Voltage 08-01 High Alarm State: " + Cells_Voltage_08_01_high_Alarm_State);
          Serial.println("Cells Voltage 16-09 High Alarm State: " + Cells_Voltage_16_09_high_Alarm_State);
          Serial.println("Cell 08-01 Temperature Low Alarm State: " + Cell_08_01_Temperature_Low_Alarm_State);
          Serial.println("CCell 08-01 Temperature High Alarm State: " + Cell_08_01_Temperature_High_Alarm_State);
          Serial.println("Cell 08-01 Equalization Event Code: " + Cell_08_01_Equalization_Event_Code);
          Serial.println("CCell 16-09 Equalization Event Code: " + Cell_16_09_Equalization_Event_Code);

          Serial.println("System State Code: " + System_State_Code);
          Serial.println("Voltage Event Code: " + Voltage_Event_Code);
          Serial.println("Cells Temperature Event Code: " + Cells_Temperature_Event_Code);
          Serial.println("Environment and Power Temperature Event Code: " + Environment_and_Power_Temperature_Event_Code);
          Serial.println("Current Event Code 1: " + Current_Event_Code_1);
          Serial.println("Current Event Code 2: " + Current_Event_Code_2);
          Serial.println("The Residual Capacity Code: " + The_Residual_Capacity_Code);   
          Serial.println("The FET Event Code: " + The_FET_Event_Code);
          Serial.println("Battery Equalization State Code: " + Battery_Equalization_State_Code);
          Serial.println("Hard Fault Event Code: " + Hard_Fault_Event_Code);

          /////////////////////////////////////////////////////////BIT ALARM//////////////////////////////////////////////////////
          Serial.println();
          Serial.print("Combined States Voltage Event Code(TB02): "); 
          Serial.println();  
          checkAndPrintState(VoltageEventCode, 0, "Cell high voltage alarm");
          checkAndPrintState(VoltageEventCode, 1, "Cell over voltage protection");
          checkAndPrintState(VoltageEventCode, 2, "Cell low voltage alarm");
          checkAndPrintState(VoltageEventCode, 3, "Cell under voltage protection");
          checkAndPrintState(VoltageEventCode, 4, "Pack high voltage alarm");
          checkAndPrintState(VoltageEventCode, 5, "Pack over voltage protect");
          checkAndPrintState(VoltageEventCode, 6, "Pack low voltage alarm");
          checkAndPrintState(VoltageEventCode, 7, "Pack under voltage protection");
          Serial.println();

          Serial.print("Combined States CellsTemperature Event Code(TB03): ");
          Serial.println();  
          checkAndPrintState(CellsTemperatureEventCode, 0, "Charge high temperature alarm");
          checkAndPrintState(CellsTemperatureEventCode, 1, "Charge over temperature protection");
          checkAndPrintState(CellsTemperatureEventCode, 2, "Charge low temperature alarm");
          checkAndPrintState(CellsTemperatureEventCode, 3, "Charge under temperature protection");
          checkAndPrintState(CellsTemperatureEventCode, 4, "Discharge high temperature alarm");
          checkAndPrintState(CellsTemperatureEventCode, 5, "Discharge over temperature protection");
          checkAndPrintState(CellsTemperatureEventCode, 6, "Discharge low temperature alarm");
          checkAndPrintState(CellsTemperatureEventCode, 7, "Discharge under temperature protection");
          Serial.println();

          Serial.print("Combined States Environment And Power Temperature Event Code(TB04): ");
          Serial.println();  
          checkAndPrintState(EnvironmentAndPowerTemperatureEventCode, 0, "High environment temperature alarm");
          checkAndPrintState(EnvironmentAndPowerTemperatureEventCode, 1, "Over environment temperature protection");
          checkAndPrintState(EnvironmentAndPowerTemperatureEventCode, 2, "Low environment temperature alarm");
          checkAndPrintState(EnvironmentAndPowerTemperatureEventCode, 3, "Under environment temperature protection");
          checkAndPrintState(EnvironmentAndPowerTemperatureEventCode, 4, "High Power temperature alarm");
          checkAndPrintState(EnvironmentAndPowerTemperatureEventCode, 5, "Over Power temperature protection");
          checkAndPrintState(EnvironmentAndPowerTemperatureEventCode, 6, "Cell temperature low heating");
          checkAndPrintState(EnvironmentAndPowerTemperatureEventCode, 7, "Reservation");
          Serial.println();
          
          Serial.print("Combined States Current Event Code1(TB05): ");
          Serial.println();  
          checkAndPrintState(CurrentEventCode1, 0, "Charge current alarm");
          checkAndPrintState(CurrentEventCode1, 1, "Charge over current protection");
          checkAndPrintState(CurrentEventCode1, 2, "Charge second level current protection");
          checkAndPrintState(CurrentEventCode1, 3, "Discharge current alarm");
          checkAndPrintState(CurrentEventCode1, 4, "Discharge over current protection");
          checkAndPrintState(CurrentEventCode1, 5, "Discharge second level over current protection");
          checkAndPrintState(CurrentEventCode1, 6, "Output short circuit protection");
          checkAndPrintState(CurrentEventCode1, 7, "Reservation");
          Serial.println();

          Serial.print("Combined States TheResidual Capacity Code(TB06): ");
          Serial.println();  
          checkAndPrintState(TheResidualCapacityCode, 0, "Reservation");
          checkAndPrintState(TheResidualCapacityCode, 1, "Reservation");
          checkAndPrintState(TheResidualCapacityCode, 2, "Soc alarm");
          checkAndPrintState(TheResidualCapacityCode, 3, "Soc protection");
          checkAndPrintState(TheResidualCapacityCode, 4, "Cell Diff alarm");
          checkAndPrintState(TheResidualCapacityCode, 5, "Reservation");
          checkAndPrintState(TheResidualCapacityCode, 6, "Reservation");
          checkAndPrintState(TheResidualCapacityCode, 7, "Reservation");
          Serial.println();

          Serial.print("Combined States The FET Event Code(TB07): ");
          Serial.println();  
          checkAndPrintState(TheFETEventCode, 0, "Discharge FET on");
          checkAndPrintState(TheFETEventCode, 1, "Charge FET on");
          checkAndPrintState(TheFETEventCode, 2, "Current limiting FET on");
          checkAndPrintState(TheFETEventCode, 3, "Heating on");
          checkAndPrintState(TheFETEventCode, 4, "Reservation");
          checkAndPrintState(TheFETEventCode, 5, "Reservation");
          checkAndPrintState(TheFETEventCode, 6, "Reservation");
          checkAndPrintState(TheFETEventCode, 7, "Reservation");
          Serial.println();

          Serial.print("Combined States Battery Equalization State Code(TB08): ");
          Serial.println();  
          checkAndPrintState(BatteryEqualizationStateCode, 0, "low Soc alarm");
          checkAndPrintState(BatteryEqualizationStateCode, 1, "Intermittent charge");
          checkAndPrintState(BatteryEqualizationStateCode, 2, "External switch control");
          checkAndPrintState(BatteryEqualizationStateCode, 3, "Static standby and sleep mode");
          checkAndPrintState(BatteryEqualizationStateCode, 4, "History data recording");
          checkAndPrintState(BatteryEqualizationStateCode, 5, "Under Soc protect");
          checkAndPrintState(BatteryEqualizationStateCode, 6, "Acktive-Limited Current");
          checkAndPrintState(BatteryEqualizationStateCode, 7, "Passive-Limited Current");
          Serial.println();

          Serial.print("Combined States System State Code(TB09): ");
          Serial.println();  
          checkAndPrintState(SystemStateCode, 0, "Discharge");
          checkAndPrintState(SystemStateCode, 1, "Charge");
          checkAndPrintState(SystemStateCode, 2, "Floating charge");
          checkAndPrintState(SystemStateCode, 3, "Full charge");
          checkAndPrintState(SystemStateCode, 4, "Standby mode");
          checkAndPrintState(SystemStateCode, 5, "Turn off");
          checkAndPrintState(SystemStateCode, 6, "Reservation");
          checkAndPrintState(SystemStateCode, 7, "Reservation");
          Serial.println();        

          Serial.print("Combined States Hard Fault Event Code(TB15): ");
          Serial.println();  
          checkAndPrintState(HardFaultEventCode, 0, "NTC Fault");
          checkAndPrintState(HardFaultEventCode, 1, "AFE Fault");
          checkAndPrintState(HardFaultEventCode, 2, "Charge Mosfets Fault");
          checkAndPrintState(HardFaultEventCode, 3, "Discharge Mosfets Fault");
          checkAndPrintState(HardFaultEventCode, 4, "Cell Faul");
          checkAndPrintState(HardFaultEventCode, 5, "Break Line Fault");
          checkAndPrintState(HardFaultEventCode, 6, "Key Fault");
          checkAndPrintState(HardFaultEventCode, 7, "Aerosol Alarm");
          Serial.println();  

          Serial.print("Combined States Current Event Code2(TB16): ");
          Serial.println();  
          checkAndPrintState(CurrentEventCode2, 0, "Output short latch up");
          checkAndPrintState(CurrentEventCode2, 1, "Reservation");
          checkAndPrintState(CurrentEventCode2, 2, "Second Charge latch up");
          checkAndPrintState(CurrentEventCode2, 3, "Second Discharge latch up");
          checkAndPrintState(CurrentEventCode2, 4, "Reservation");
          checkAndPrintState(CurrentEventCode2, 5, "Reservation");
          checkAndPrintState(CurrentEventCode2, 6, "Reservationn");
          checkAndPrintState(CurrentEventCode2, 7, "Reservation");
          Serial.println();
        }

      }   

    }

  }

  return BufferValue_Pic[0];

}

// Function to convert bytes to the appropriate data type based on the type field
float convert_bytes_to_data(String data_type, uint8_t byte1, uint8_t byte2) {
  if (data_type == "UINT16") {
    return (byte1 << 8) | byte2;
  } else if (data_type == "INT16") {
    int16_t value = (byte1 << 8) | byte2;
    // Convert to signed integer
    if (value & 0x8000) {
      value -= 0x10000;
    }
    return value;
  } else {
    return 0.0;
  }
}

String decToHex(int decValue) {
  char hexValue[9]; 
  sprintf(hexValue, "%X", decValue);
  return String(hexValue);
}

/*void checkAndPrintState(int code, int bitIndex, const char *stateName) {
  // Check if the bit at bitIndex is set (1)
  if ((code & (1 << bitIndex)) != 0) {
    Serial.print(stateName);
    Serial.print(", ");
  }
  else{
    Serial.print("None");
    Serial.print(", ");
  }
}*/

String checkAndPrintState(int code, int bitIndex, const char *stateName) {
  String result;
  // Check if the bit at bitIndex is set (1)
  if ((code & (1 << bitIndex)) != 0) {
    result = "1";
  } else {
    result = "0";
  }
  if(on_check_pic == true){
    Serial.println(String(stateName) + ": " + result);
  }
  return result;

}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);

  Serial.print("Message: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
      // Once connected, subscribe to the topic
      //client.subscribe(mqtt_to_pia);
    } else {
      //Serial.println("failed, rc=");
      //Serial.print(client.state());
      //Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      //delay(1000);
      Serial.println();
      reset_wifi();
    }
  }
}

void saveConfigData() {
  EEPROM.begin(512);
  EEPROM.put(EEPROM_ADDR_MQTT_SERVER, mqtt_server);
  EEPROM.put(EEPROM_ADDR_MQTT_PORT, mqtt_port);
  EEPROM.put(EEPROM_ADDR_MQTT_USER, mqtt_user);
  EEPROM.put(EEPROM_ADDR_MQTT_PASSWORD, mqtt_password);
  EEPROM.commit();
  EEPROM.end();
}

void loadConfigData() {
  EEPROM.begin(512);
  EEPROM.get(EEPROM_ADDR_MQTT_SERVER, mqtt_server);
  mqtt_port = EEPROM.get(EEPROM_ADDR_MQTT_PORT, mqtt_port);
  EEPROM.get(EEPROM_ADDR_MQTT_USER, mqtt_user);
  EEPROM.get(EEPROM_ADDR_MQTT_PASSWORD, mqtt_password);
  EEPROM.end();
}

void reset_wifi(){
  WiFiManager wm;

  buttonState = digitalRead(button_reset);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == LOW) {
    if( millis() - last_time > period) {
      // turn LED on:
      digitalWrite(led_reset, HIGH);

      // Connect to WiFi using WiFiManager
      WiFiManagerParameter custom_mqtt_server("mqtt_server", "MQTT Server", mqtt_server, 40);
      WiFiManagerParameter custom_mqtt_port("mqtt_port", "MQTT Port", String(mqtt_port).c_str(), 6);
      WiFiManagerParameter custom_mqtt_user("mqtt_user", "MQTT User", mqtt_user, 20);
      WiFiManagerParameter custom_mqtt_password("mqtt_password", "MQTT Password", mqtt_password, 20);

      wm.addParameter(&custom_mqtt_server);
      wm.addParameter(&custom_mqtt_port);
      wm.addParameter(&custom_mqtt_user);
      wm.addParameter(&custom_mqtt_password);
      
      // Set the configuration portal timeout to 5 minutes
      wm.setConfigPortalTimeout(300);
      
      clearEEPROM();
      wm.resetSettings();
      
      wm.autoConnect("Deizzem Smart Home");

      // Get the values from the custom parameters
      strcpy(mqtt_server, custom_mqtt_server.getValue());
      mqtt_port = atoi(custom_mqtt_port.getValue());
      strcpy(mqtt_user, custom_mqtt_user.getValue());
      strcpy(mqtt_password, custom_mqtt_password.getValue());

      // Save the configuration to EEPROM
      saveConfigData();

      // Set up MQTT
      client.setServer(mqtt_server, mqtt_port);

      Serial.println("Connected to Wi-Fi");
      Serial.println("MQTT Server: " + String(mqtt_server));
      Serial.println("MQTT Port: " + String(mqtt_port));
      Serial.println("MQTT User: " + String(mqtt_user));
      Serial.println("MQTT Password: " + String(mqtt_password));

      last_time = millis();
    }
  } 
  else {
    // turn LED off:
    digitalWrite(led_reset, LOW);
    last_time = millis();
  }    
  
}

void clearEEPROM() {
  // Begin using EEPROM with a specified size
  EEPROM.begin(512); // Adjust the size according to your needs

  // Clear data by writing 0 to each address in the EEPROM
  for (int i = 0; i < EEPROM.length(); ++i) {
    EEPROM.write(i, 0);
  }

  // Commit the changes
  EEPROM.commit();

  // End using EEPROM
  EEPROM.end();
}