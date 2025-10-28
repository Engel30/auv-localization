#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h> //NUOVO mqtt cloud
#include <Wire.h>
#include <JY901.h>
#include <time.h>
#include <math.h>
#include "M5CoreS3.h"
#include "M5GFX.h"
#include "M5_SIM7080G.h"
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// WiFi credentials
const char* ssid = "RP-Net-WD03";//iPhone di Gianluca";
const char* password = "20190901";//gianluca003";

// MQTT local broker configuration
/*const char* mqtt_server = "172.20.10.6"; //router IP address
const int mqtt_port = 1884; */

const char* mqtt_server = "h6a86e8b.ala.eu-central-1.emqxsl.com"; //url del broker online
const int mqtt_port = 8883; //porta 8883 per connessione sicura

const char* mqtt_topic_pub = "/devices";
const char* mqtt_topic_pub_2 = "/terminal"; 
const char* mqtt_topic_sub_commands = "/commands";
const char* mqtt_topic_sub_manual_1 = "/angle";
const char* mqtt_topic_sub_manual_2 = "/frequency";
const char* mqtt_topic_sub_automatic = "/waypoints";

String terminalFlag="";
bool serial_inviato = false;


//SoftwareSerial Serial3 (18, 17);

M5GFX display;
M5Canvas canvas(&display);

//PIN SD per salvare dati
#define SD_SPI_SCK_PIN  36
#define SD_SPI_MISO_PIN 35
#define SD_SPI_MOSI_PIN 37
#define SD_SPI_CS_PIN   4

File file;
String nomeFile = "";
bool sd_ready = false;
unsigned long recordingStartMillis = 0;
int n_rec = 0;

//GNSS
M5_SIM7080G device;


float mytime, q0, q1, q2, q3, ax, ay, az, gx, gy, gz, mx, my, mz;
float lat, lon, altitude;

int satellitesUsed;
String response, utcTime="00:00";


// WiFi and MQTT client
//WiFiClient espClient; //usare questo in caso di porta 1883
WiFiClientSecure espClient; //usare questo in caso di porta 8883
PubSubClient client(espClient);

//ca_cert scaricabile dal broker per la sicurezza (serve solo per la 8883, MQTT over TLS/SSL)
static const char ca_cert[]
PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB
CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97
nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt
43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P
T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4
gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO
BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR
TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw
DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr
hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg
06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF
PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls
YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk
CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=
-----END CERTIFICATE-----
)EOF";


const char* ntpServer = "pool.ntp.org";
unsigned long long epochTime; 
unsigned long long time_rx; 

unsigned long long getTime() {  
  struct tm timeinfo;   
  if (!getLocalTime(&timeinfo)) 
  {     
    //Serial.println("Error: it's impossibile to obtain the time");    
    return 0;   
  } 

  time_t now = time(nullptr);   
  return static_cast<unsigned long long>(now) * 1000 + (millis() % 1000); //time in UNIX format 13 digits
}

//Funzione per leggere i dati dall'IMU
void readIMUData() {
    mytime = millis() / 1000.0;
    ax = JY901.stcAcc.a[0] / 32768.0 * 16;
    ay = JY901.stcAcc.a[1] / 32768.0 * 16;
    az = JY901.stcAcc.a[2] / 32768.0 * 16;
    gx = JY901.stcGyro.w[0] / 32768.0 * 2000;
    gy = JY901.stcGyro.w[1] / 32768.0 * 2000;
    gz = JY901.stcGyro.w[2] / 32768.0 * 2000;
    mx = JY901.stcMag.h[0];
    my = JY901.stcMag.h[1];
    mz = JY901.stcMag.h[2];

    //lat = JY901.stcLonLat.lLat; 
    //lon = JY901.stcLonLat.lLon; 
 
    q0 = JY901.stcQuater.q0;
    q1 = JY901.stcQuater.q1;
    q2 = JY901.stcQuater.q2;
    q3 = JY901.stcQuater.q3;
    float norm_q = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm_q;
    q1 /= norm_q;
    q2 /= norm_q;
    q3 /= norm_q;

    //Serial.println("Preso i dati IMU");
}

//Funzione che trova in automatico il nome da dare al prossimo file da salvare (numerazione automatica)
int findNextFileNumber(String baseName) {
  int fileNumber = 1;
  while (SD.exists(baseName + String(fileNumber) + ".csv")) {
    fileNumber++;
  }
  return fileNumber;
}

//Salva i dati sulla SD e li mostra sullo schermo dell'M5
void displayAndLogData() { 

  /*
  file = SD.open(nomeFile, FILE_APPEND);
  //Serial.println("Aperto file");
  if (n_rec == 0) {
    file.println("Time,ax,ay,az,gx,gy,gz,mx,my,mz,UTC,lon,lat,altitude,SatellitiUsed,q0,q1,q2,q3");
    n_rec ++;
  }

  String riga = String(mytime, 2) + "," + String(ax) + "," + String(ay) + "," + String(az) + "," +
              String(gx) + "," + String(gy) + "," + String(gz) + "," +
              String(mx) + "," + String(my) + "," + String(mz) + "," + 
              utcTime + "," + String(lon, 8) + "," + String(lat, 8) + "," +
              String(altitude, 3) + "," + String(satellitesUsed) + "," +
              String(q0) + "," + String(q1) + "," + String(q2) + "," + String(q3);

  file.println(riga);
  */
  //Serial.println("Stampato primi dati");
  


  //file.println(String(mytime) + "," + String(ax) + "," + String(ay) + "," + String(az) + "," + String(gx) + "," + String(gy) + "," + String(gz) + "," +
  //String(mx) + "," + String(my) + "," + String(mz) + "," + String(UTC) + "," + String(lon, 8) + "," + String(lat, 8) + "," + String(altitude) + "," + String(satellitesUsed) + "," + String(q0) + "," + String(q1) + "," + String(q2) + "," + String(q3));
  //file.close();
  
  //delay(500);


  file = SD.open(nomeFile, FILE_APPEND);
  //Serial.println("Aperto file");

  if (n_rec == 0) {
    // Scrivere l'intestazione solo una volta
    file.print("Time;");
    file.print("ax;");
    file.print("ay;");
    file.print("az;");
    file.print("gx;");
    file.print("gy;");
    file.print("gz;");
    file.print("mx;");
    file.print("my;");
    file.print("mz;");
    file.print("UTC;");
    file.print("lon;");
    file.print("lat;");
    file.print("altitude;");
    file.print("SatellitiUsed;");
    file.print("q0;");
    file.print("q1;");
    file.print("q2;");
    file.println("q3");  //Ultima colonna, quindi uso println per terminare la riga
    n_rec++;
  }

  //Scrivere i dati su una nuova riga sotto l'intestazione
  file.print(mytime, 2);            
  file.print(";"); 
  file.print(ax);                   
  file.print(";");
  file.print(ay);                   
  file.print(";");
  file.print(az);                   
  file.print(";");
  file.print(gx);                   
  file.print(";");
  file.print(gy);                   
  file.print(";");
  file.print(gz);                   
  file.print(";");
  file.print(mx);                   
  file.print(";");
  file.print(my);                   
  file.print(";");
  file.print(mz);                   
  file.print(";");
  file.print(utcTime);              
  file.print(";");
  file.print(lon, 8);               
  file.print(";");
  file.print(lat, 8);               
  file.print(";");
  file.print(altitude, 3);          
  file.print(";");
  file.print(satellitesUsed);       
  file.print(";");
  file.print(q0);                   
  file.print(";");
  file.print(q1);                   
  file.print(";");
  file.print(q2);                   
  file.print(";");
  file.println(q3);
  file.close();

  //Serial.println("File chiuso");
  
  

  //Serial.printf("t=%f | q=[%.2f, %.2f, %.2f, %.2f] | lat/lon: %6f, %6f\n", mytime, q0, q1, q2, q3, lat, lon);

  CoreS3.Display.fillRect(0, 30, CoreS3.Display.width(), 200, TFT_BLACK);
  CoreS3.Display.setCursor(0, 35);
  CoreS3.Display.setTextColor(TFT_WHITE, TFT_BLACK);

  CoreS3.Display.println("Time: " + String(mytime));
  CoreS3.Display.println("Quaternion:");
  CoreS3.Display.printf("q0: %.4f\nq1: %.4f\nq2: %.4f\nq3: %.4f\n", q0, q1, q2, q3);

  CoreS3.Display.println("UTC Time: " + utcTime);
  CoreS3.Display.println("Latitude: " + String(lat, 8));
  CoreS3.Display.println("Longitude: " + String(lon, 8));
  CoreS3.Display.println("Altitude: " + String(altitude, 8) + " m"); //RIMETTI
  CoreS3.Display.println("Satellites Used: " + String(satellitesUsed)); //RIMETTI

  //Serial.println("Stampati dati a schermo");
}

//Serve per convertire la risposta del modulo GPS in un formato leggibile da mettere nelle variabili sopra dichiarate
void parseCGNSINF(String response) {
    response.replace("+CGNSINF: ", "");
    response.trim();
    int fieldIndex = 0;
    String fields[22];
    int lastIndex = 0;
 
    for (int i = 0; i < response.length(); i++) {
        if (response[i] == ',' || i == response.length() - 1) {
            fields[fieldIndex++] = response.substring(lastIndex, (i == response.length() - 1) ? i + 1 : i);
            lastIndex = i + 1;
        }
        if (fieldIndex >= 22) break;
    }
 
    
    
    utcTime = fields[2];
    lat = fields[3].toFloat();
    lon = fields[4].toFloat();
    altitude = fields[5].toFloat();
    satellitesUsed = fields[15].toInt();
    
    //Serial.println("Preso i dati gps");

     /*else {
        CoreS3.Display.fillRect(0, 0, CoreS3.Display.width(), 200, TFT_BLACK);
        CoreS3.Display.setCursor(0, 35);
        CoreS3.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        CoreS3.Display.println("NO FIX YET");
        CoreS3.Display.println("Searching satellites...");
    }*/
}


//Funzione che prende e stampa i dati chiamando le altre funzioni
void collectAndLogData(){

  readIMUData();
  String response = device.send_and_getMsg("AT+CGNSINF\r\n");
  parseCGNSINF(response);
  displayAndLogData();


}

void log(String str) {
    CoreS3.Display.fillRect(0, 30, CoreS3.Display.width(), 200, TFT_BLACK);
    CoreS3.Display.setCursor(0, 35);
    CoreS3.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    //Serial.print(str);
    CoreS3.Display.println(str);
}

bool isFloat(String s) {
  bool decimalPointFound = false;
  int startIndex = 0;
  int digitCount = 0;

  if (s.length() == 0) return false;

  // Gestione segno
  if (s.charAt(0) == '-') {
    startIndex = 1;
    if (s.length() == 1) return false;
  }

  for (int i = startIndex; i < s.length(); i++) {
    char c = s.charAt(i);

    if (c == '.') {
      if (decimalPointFound) return false; // Più di un punto
      decimalPointFound = true;
    } else if (isDigit(c)) {
      digitCount++;
    } else {
      return false; // Carattere non valido
    }
  }

  // Deve esserci almeno una cifra e deve avere almeno una virgola (poichè è float)
  return digitCount > 0 && decimalPointFound;
}




void setup() {

  //Serial.begin(115200);
  //Serial.begin(115200, SERIAL_8N1, 1, 2);
  Serial.end(); // forza detach della UART0
  Serial.begin(115200, SERIAL_8N1, 1, 2);
  //Serial.begin(115200, SERIAL_8N1, 1, 2); FUNZIONANTE
  //Serial2.begin(115200, SERIAL_8N1, 1, 2);

  //NUOVO
  //Serial2.begin(115200, SERIAL_8N1, 1, 2); ATTIVARE PER COM. PESCE

  M5.begin();
  auto cfg = M5.config();
  // if using ext power input(Grove Port or DC input power supply) needs to be set to false.
  cfg.output_power = false;
  CoreS3.begin(cfg);



  setup_wifi(); //Collegamento al Wi-Fi

  
  configTime(0, 0, ntpServer);
  
  espClient.setCACert(ca_cert); //Serve per TLS/SSL per la 8883, per la 1883 commentare questa riga

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback); //callback = funzione da eseguire quando si riceve un messaggio su un topic
  //

  //PARTE NUOVA
  display.begin();
  CoreS3.Display.setTextSize(2);
  if (display.isEPD()) {
        display.setEpdMode(epd_mode_t::epd_fastest);
        display.invertDisplay(true);
        display.clear(TFT_BLACK);
    }
  if (display.width() < display.height()) {
      display.setRotation(display.getRotation() ^ 1);
  }

  //Inizializzazione SD
  SPI.begin(SD_SPI_SCK_PIN, SD_SPI_MISO_PIN, SD_SPI_MOSI_PIN, SD_SPI_CS_PIN);

  sd_ready = SD.begin(SD_SPI_CS_PIN, SPI, 25000000);
  while (!sd_ready) {
    //Serial.println("SD initialization failed!");
  }
  //Serial.println("SD ready");
  int fileID = findNextFileNumber("/registrazione_");
  nomeFile = "/registrazione_" + String(fileID) + ".csv";
  recordingStartMillis = millis();
  //Serial.println("Logging to: " + nomeFile);
  
  

  //Seriale GPS e IMU
  device.Init(&Serial2, 8, 9); //OGGI NON FUNZIONA
  Serial1.begin(115200, SERIAL_8N1, 17, 18); //17,18 per boa   18, 17 per normale

  // NON VUOLE ACCENDERSI STO COSO
   log("Reboot SIM7080G..\n");
  while (device.send_and_getMsg("AT+CREBOOT\r\n").indexOf("OK") == -1) {
       log("..\n");
      // delay(1000);
  }

   delay(10000); 

  // log("\nTurn GNSS power on..\n");
  while (device.send_and_getMsg("AT+CGNSPWR=1\r\n").indexOf("OK") == -1) {
       log("..\n");
      delay(1000);
  }

  // log("\nImpostando gnss work mode..\n");
  while (device.send_and_getMsg("AT+CGNSMOD=1,1,0,0,0\r\n").indexOf("OK") == -1) {
      log("..\n");
      delay(1000);
  }

  log("\nReady to print GNSS info!\n");
  //FINE NUOVA

  
  //Serial2.begin(115200, SERIAL_8N1, 17, 18);


 
  delay(5000);
  
  reconnect();
}

/*
void saveData() {
  myTime = (float)millis() / 1000;
  ax = (float)JY901.stcAcc.a[0] / 32768 * 16;
  ay = (float)JY901.stcAcc.a[1] / 32768 * 16;
  az = (float)JY901.stcAcc.a[2] / 32768 * 16;
  gx = (float)JY901.stcGyro.w[0] / 32768 * 2000;
  gy = (float)JY901.stcGyro.w[1] / 32768 * 2000;
  gz = (float)JY901.stcGyro.w[2] / 32768 * 2000;
  mx = (float)JY901.stcMag.h[0];
  my = (float)JY901.stcMag.h[1];
  mz = (float)JY901.stcMag.h[2];
  lon = JY901.stcLonLat.lLon / 10000000.0;
  lat = JY901.stcLonLat.lLat / 10000000.0;
  SN = JY901.stcSN.sSVNum;
  PDOP = (float)JY901.stcSN.sPDOP / 100;
  HDOP = (float)JY901.stcSN.sHDOP / 100;
  VDOP = (float)JY901.stcSN.sVDOP / 100;
  q0 = JY901.stcQuater.q0;
  q1 = JY901.stcQuater.q1;
  q2 = JY901.stcQuater.q2;
  q3 = JY901.stcQuater.q3;
  norm_q = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
 
  q0 /= norm_q;
  q1 /= norm_q;
  q2 /= norm_q;
  q3 /= norm_q;
 
  Serial.println(String(1) + "," + String(myTime) + "," + String(ax) + "," + String(ay) + "," + String(az) + "," + String(gx) + "," + String(gy) + "," + String(gz) + "," +
    String(mx) + "," + String(my) + "," + String(mz) + ", LON: " + String(lon, 6) + ", LAT: " + String(lat, 6) + "," + String(SN) + "," + String(PDOP) + "," +
    String(HDOP) + "," + String(VDOP) + ",q0: " + String(q0) + ", q1: " + String(q1) + ", q2: " + String(q2) + ", q3: " + String(q3));

}

void fakeSalva(){
  // Accelerazione lineare in m/s^2 (circa -10 a +10)
  ax = random(-1000, 1000) / 100.0;
  ay = random(-1000, 1000) / 100.0;
  az = random(800, 1200) / 100.0;  // gravità verso il basso

  // Velocità angolare in rad/s (circa -5 a +5)
  gx = random(-500, 500) / 100.0;
  gy = random(-500, 500) / 100.0;
  gz = random(-500, 500) / 100.0;

  // Magnetometro in µT (microtesla), range indicativo della Terra
  mx = random(-50, 50) / 1.0;
  my = random(-50, 50) / 1.0;
  mz = random(-50, 50) / 1.0;

  // Coordinate GPS (Roma, ad esempio)
  lon = 12.4964 + random(-100, 100) / 10000.0;
  lat = 41.9028 + random(-100, 100) / 10000.0;

  // Satelliti e precisione GPS
  SN = random(5, 15);             // numero di satelliti visibili
  PDOP = random(100, 300) / 100.0;
  HDOP = random(50, 150) / 100.0;
  VDOP = random(50, 150) / 100.0;

  // Quaternion (valori normalizzati tra -1 e 1)
  q0 = random(-100, 100) / 100.0;
  q1 = random(-100, 100) / 100.0;
  q2 = random(-100, 100) / 100.0;
  q3 = random(-100, 100) / 100.0;

  // Normalizzazione (facoltativa)
  float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

  Serial.println(String(1) + "," + String(myTime) + "," + String(ax) + "," + String(ay) + "," + String(az) + "," + String(gx) + "," + String(gy) + "," + String(gz) + "," +
    String(mx) + "," + String(my) + "," + String(mz) + ", LON: " + String(lon) + ", LAT: " + String(lat) + "," + String(SN) + "," + String(PDOP) + "," +
    String(HDOP) + "," + String(VDOP) + ", q0: " + String(q0) + ", q1: " + String(q1) + ", q2: " + String(q2) + ", q3: " + String(q3));
}
*/

// WiFi Connection
void setup_wifi() {

  WiFi.disconnect();
  delay(10);
  // Serial.println();
  // Serial.print("Connecting to ");
  // Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    // Serial.print(".");
  }

  // Serial.println();
  // Serial.println("WiFi connected");
  // Serial.println("IP address: ");
  // Serial.println(WiFi.localIP());
}

//Funzione per collegari e ricollegarsi in caso di problemi a MQTT
void reconnect() {

  while (!client.connected()) {
    //Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", "m5", "password")) { //Aggiunto "m5", "password" per poter utilizzare il broker online con la versione sicura. Per utilizzare la 1883 basta cancellare "m5","password"
      //Serial.println("connected");
      CoreS3.Display.fillRect(0, 30, CoreS3.Display.width(), 200, TFT_BLACK);
      CoreS3.Display.setCursor(0, 35);
      CoreS3.Display.setTextColor(TFT_WHITE, TFT_BLACK);

      CoreS3.Display.println("Connessione con MQTT correttamente eseguita");

      client.subscribe(mqtt_topic_sub_commands); // Wait to receive data from mqtt topic /commands
      client.subscribe(mqtt_topic_sub_manual_1); // Wait to receive data from mqtt topic /angle
      client.subscribe(mqtt_topic_sub_manual_2); // Wait to receive data from mqtt topic /frequency
      client.subscribe(mqtt_topic_sub_automatic); // Wait to receive data from mqtt topic /waypoints
    } else {
      // Serial.print("failed, rc=");
      // Serial.print(client.state());
      CoreS3.Display.fillRect(0, 30, CoreS3.Display.width(), 200, TFT_BLACK);
      CoreS3.Display.setCursor(0, 35);
      CoreS3.Display.setTextColor(TFT_WHITE, TFT_BLACK);

      CoreS3.Display.println("Connessione con MQTT Fallita");
      delay(1000);
    }
  }
}
//LOOP VECCHIO
/*
void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  //AAAA
  
  while (Serial2.available()) {
    JY901.CopeSerialData(Serial2.read());
  }
  saveData();
  
  //fakeSalva();
  epochTime = getTime();

  /*
  float linear_x = random(-1000, 1000) / 1000.0;
  float linear_y = random(-1000, 1000) / 1000.0;
  float linear_z = random(-1000, 1000) / 1000.0;
  float angular_x = random(-1000, 1000) / 1000.0;
  float angular_y = random(-1000, 1000) / 1000.0;
  float angular_z = random(-1000, 1000) / 1000.0;
  float lon = random(13178345,13178495) / 1000000.0; 
  float lat = random(43657835,43657985) / 1000000.0; 
  float depth = random(-1000, 1000) / 1000.0;
  float q0 = random(-1000, 1000) / 1000.0;
  float q1 = random(-1000, 1000) / 1000.0;
  float q2 = random(-1000, 1000) / 1000.0;
  float q3 = random(-1000, 1000) / 1000.0;
  float norm_q = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
  float temperature = random(100,200)/10;
  float steering_angle = random(-13090, 13090)/1000;
  float frequency = random(0,400)/100;
  float voltage = random(1800, 2200)/100;
  float current = random(1800, 2200)/100;
  float battery_charge = random(0, 1000)/10;

  q0 /=  norm_q;
  q1 /=  norm_q;
  q2 /=  norm_q;
  q3 /=  norm_q;
  */
/*
  float depth = random(-1000, 1000) / 1000.0;
  float temperature = random(100,200)/10;
  float steering_angle = random(-13090, 13090)/1000;
  float frequency = random(0,400)/100;
  float voltage = random(1800, 2200)/100;
  float current = random(1800, 2200)/100;
  float battery_charge = random(0, 1000)/10;



  String position_data = String("{") +
                 "\"time\": " + String(epochTime) + ", " +
                 "\"linear_x\": " + String(ax) + ", " +
                 "\"linear_y\": " + String(ay) + ", " +
                 "\"linear_z\": " + String(az) + ", " +
                 "\"angular_x\": " + String(gx) + ", " +
                 "\"angular_y\": " + String(gy) + ", " +
                 "\"angular_z\": " + String(gz) + ", " +
                 "\"lon\": " + String(lon, 6) + ", " + 
                 "\"lat\": " + String(lat, 6) + ", " +
                 "\"depth\": " + String(depth) + ", " +
                 "\"q0\": " + String(q0) + ", " +
                 "\"q1\": " + String(q1) + ", " +
                 "\"q2\": " + String(q2) + ", " +
                 "\"q3\": " + String(q3) + 
                 "}";

  String devices_data = String("{") +
                  "\"time\": " + String(epochTime) + ", " +
                  "\"temperature\": " + String(temperature) + ", " +
                  "\"steering_angle\": " + String(steering_angle) + ", " +
                  "\"frequency\": " + String(frequency) + ", " +
                  "\"voltage\": " + String(voltage) + ", " +
                  "\"current\": " + String(current) + ", " +
                  "\"battery_charge\": " + String(battery_charge) +
                  "}";
  
  //Serial.println("Data sent from ESP32: ");
  //Serial.println(position_data);
  //Serial.println(devices_data);

  // Publish data to mqtt topic /devices
  client.publish(mqtt_topic_pub, position_data.c_str());
  client.publish(mqtt_topic_pub, devices_data.c_str());
  delay(1000);
}
*/


void loop() {
  //String response = device.send_and_getMsg("AT+CGNSINF\r\n");


  if (!client.connected()) {
    reconnect();
  }
  client.loop();


  while (Serial1.available()) {
     JY901.CopeSerialData(Serial1.read()); //Legge i dati dalla seriale dell'IMU
  }

  collectAndLogData();

  epochTime = getTime();

  float depth = random(-1000, 1000) / 1000.0;
  float temperature = random(100,200)/10;
  float steering_angle = random(-13090, 13090)/1000;
  float frequency = random(0,400)/100;
  float voltage = random(1800, 2200)/100;
  float current = random(1800, 2200)/100;
  float battery_charge = random(0, 1000)/10;

  //Messaggio da dover inviare sul topic
  String position_data = String("{") +
                 "\"time\": " + String(epochTime) + ", " +
                 "\"linear_x\": " + String(ax) + ", " +
                 "\"linear_y\": " + String(ay) + ", " +
                 "\"linear_z\": " + String(az) + ", " +
                 "\"angular_x\": " + String(gx) + ", " +
                 "\"angular_y\": " + String(gy) + ", " +
                 "\"angular_z\": " + String(gz) + ", " +
                 "\"lon\": " + String(lon, 6) + ", " + 
                 "\"lat\": " + String(lat, 6) + ", " +
                 //"\"depth\": " + String(depth) + ", " +
                 "\"q0\": " + String(q0) + ", " +
                 "\"q1\": " + String(q1) + ", " +
                 "\"q2\": " + String(q2) + ", " +
                 "\"q3\": " + String(q3) + 
                 "}";

  //Messaggio da dover inviare sul topic
  String devices_data = String("{") +
                  "\"time\": " + String(epochTime) + ", " +
                  "\"temperature\": " + String(temperature) + ", " +
                  "\"steering_angle\": " + String(steering_angle) + ", " +
                  "\"frequency\": " + String(frequency) + ", " +
                  "\"voltage\": " + String(voltage) + ", " +
                  "\"current\": " + String(current) + ", " +
                  "\"battery_charge\": " + String(battery_charge) +
                  "}";
  
  //Serial.println("Data sent from ESP32: ");
  //Serial.println(position_data);
  //Serial.println(devices_data);

  //Serial.println("Sto INVIANDO");

  while (Serial.available()) {
      String terminale = Serial.readStringUntil('\n');
      terminale.trim();
      
      /*
      if(serial_inviato){
        serial_inviato = false;
      }
      else{

        Serial.println("RICEVUTO DA SERIAL2: " + terminale);
        /*
        if(comando[0] == 'M'){

          String to_send = "{\"text\": \"" + comando + "\"}";
          client.publish(mqtt_topic_pub_2, to_send.c_str());
        }//
        String to_send = "{\"text\": \"" + terminale + "\"}";
        client.publish(mqtt_topic_pub_2, to_send.c_str());
        terminalFlag = terminale;
        

      }*/


      if(terminale.length() > 2 && !(isFloat(terminale))){

        //Serial.println("RICEVUTO DA SERIAL: " + terminale);

        String escapedTerm = escapeJSON(terminale);
        
        String to_send = "{\"text\": \"" + escapedTerm + "\"}";
        client.publish(mqtt_topic_pub_2, to_send.c_str());
      }

      //String ciao = "qualcosa";

      //client.publish(mqtt_topic_pub_2, ciao.c_str());      
      
  }

  // Publish data to mqtt topic /devices
  client.publish(mqtt_topic_pub, position_data.c_str());
  client.publish(mqtt_topic_pub, devices_data.c_str());


  delay(1000);
}


//Funzione che viene eseguito quando riceve un messaggio da un topic a cui è iscritto
void callback(char* topic, byte* payload, unsigned int length) {
  // Serial.print("Received message on topic [");
  // Serial.print(topic);
  // Serial.println("]:");
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (String(message) != "---") {
     //Serial.println("Message from interface: " + message);
     time_rx = getTime();
     //Serial.print("Timestamp: ");
     //Serial.println(time_rx);

    //Actions based on the topic
    if (String(topic) == mqtt_topic_sub_manual_1) 
    {
      //Serial.println("Steering angle assignment...");
    } 
    else if (String(topic) == mqtt_topic_sub_manual_2) 
    {
      //Serial.println("Tail frequency assignment...");
    } 
    else if (String(topic) == mqtt_topic_sub_commands)
    {
      //Serial.println("COMANDOOOOOOOOOO RICEVUTOOOOOOOOOOOO...");
      //Serial2.println(message);
      Serial.println(message);
      serial_inviato = true;

    } else if(String(topic) == mqtt_topic_sub_automatic){
      //Serial.println("WAYPOINTS ARRIVATIIIIII...");
    } else
    {
      //Serial.println("Unrecognized topic...");
    }
  } 
}


String escapeJSON(const String& input) {
  String output = "";
  for (size_t i = 0; i < input.length(); i++) {
    char c = input.charAt(i);
    if (c == '\"') {
      output += "\\\"";
    } else if (c == '\\') {
      output += "\\\\";
    } else {
      output += c;
    }
  }
  return output;
}














