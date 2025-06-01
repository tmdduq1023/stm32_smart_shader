#include <WiFiEsp.h>
#include <SoftwareSerial.h>
#include <Stepper.h>
#include <LiquidCrystal_I2C.h>

#define AP_SSID "embA"
#define AP_PASS "embA1234"
#define SERVER_NAME "10.10.141.74"
#define SERVER_PORT 5000
#define LOGID "KSH_ARD"
#define PASSWD "PASSWD"

#define WIFIRX 6
#define WIFITX 7

#define CMD_SIZE 50
#define ARR_CNT 5

SoftwareSerial wifiSerial(WIFIRX, WIFITX);
WiFiEspClient client;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// 28BYJ-48 스텝모터 2048스텝/회전 기준
#define STEPS_PER_REV 2048
Stepper stepper(STEPS_PER_REV, 8, 10, 9, 11);

char sendBuf[CMD_SIZE];
char lcdLine1[17] = "Smart IoT By KSH";
char lcdLine2[17] = "WiFi Connecting!";

void setup() {
  Serial.begin(115200);
  wifiSerial.begin(38400);
  WiFi.init(&wifiSerial);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(lcdLine1);
  lcd.setCursor(0, 1);
  lcd.print(lcdLine2);

  wifi_Init();

  lcd.clear();
  lcd.print("Connected");

  if (server_Connect()) {
    lcd.setCursor(0, 1);
    lcd.print("Server Connected");
  } else {
    lcd.setCursor(0, 1);
    lcd.print("Server Failed   ");
  }

  stepper.setSpeed(10);
}

void loop() {
  if (client.available()) {
    socketEvent();
  }

  if (!client.connected()) {
    lcd.setCursor(0, 1);
    lcd.print("Reconnecting...   ");
    if (server_Connect()) {
      lcd.setCursor(0, 1);
      lcd.print("Server Connected  ");
    } else {
      delay(1000);
    }
  }
}

void socketEvent() {
  char recvBuf[CMD_SIZE] = {0};
  int len = client.readBytesUntil('\n', recvBuf, CMD_SIZE);
  client.flush();

  char *pToken;
  char *pArray[ARR_CNT] = {0};
  int i = 0;

  pToken = strtok(recvBuf, "[@]");
  while (pToken != NULL && i < ARR_CNT) {
    pArray[i++] = pToken;
    pToken = strtok(NULL, "[@]");
  }

  if (pArray[1] && strcmp(pArray[1], "MOTOR") == 0 && pArray[2]) {
    if (strcmp(pArray[2], "GO") == 0) {
      stepper.step(STEPS_PER_REV * 3);
    } else if (strcmp(pArray[2], "BACK") == 0) {
      stepper.step(-STEPS_PER_REV * 3);
    }

    sprintf(sendBuf, "[%s]%s@%s\n", pArray[0], pArray[1], pArray[2]);
    client.write(sendBuf, strlen(sendBuf));
    client.flush();
  }
}

void wifi_Init() {
  while (WiFi.status() == WL_NO_SHIELD) {
    delay(500);
  }
  while (WiFi.begin(AP_SSID, AP_PASS) != WL_CONNECTED) {
    delay(500);
  }
}

bool server_Connect() {
  if (client.connect(SERVER_NAME, SERVER_PORT)) {
    client.print("[" LOGID ":" PASSWD "]");
    return true;
  }
  return false;
}
