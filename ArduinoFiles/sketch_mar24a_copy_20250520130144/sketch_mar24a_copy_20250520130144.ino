#include <WiFi.h>

const char* ssid = "FFRb";
const char* pwd  = "nana0916";
const uint16_t TCP_PORT = 3333;

WiFiServer  server(TCP_PORT);
WiFiClient  client;   


const int  EMGPin     = 35;
const int  baseline   = 1900;
const int  windowSize = 50;
long  squareBuffer[windowSize];
int   bufIndex = 0;


void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pwd);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.printf("\nWiFi connected, IP=%s\n", WiFi.localIP().toString().c_str());
  server.begin();

  // 缓冲区清零
  for (int i = 0; i < windowSize; i++) squareBuffer[i] = 0;
}

void loop() {

  if (!client || !client.connected()) {
    client = server.available();      
  }

  /* ——— 2. 读取 EMG 并计算 RMS ——— */
  int raw   = analogRead(EMGPin);
  long diff = raw - baseline;
  long sq   = diff * diff;

  squareBuffer[bufIndex] = sq;
  bufIndex = (bufIndex + 1) % windowSize;

  long sum = 0;
  for (int i = 0; i < windowSize; i++) sum += squareBuffer[i];
  float rms = sqrt((float)sum / windowSize);

  // 串口监视器
  Serial.print(raw); Serial.print(','); Serial.println(rms);

  // Wi-Fi 客户端
  if (client && client.connected()) {  
    client.printf("%d,%.1f\n", raw, rms);  
  }

  delay(5);   
}
