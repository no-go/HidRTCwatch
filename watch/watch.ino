#include <sam.h>
#include <power.h>
#include <wdt.h>

// -------------------------------------------Config Start
const int MPU=0x69;  // I2C address of the MPU-6050 (AD0 to 3.3V)

#define SET_INIT_RTC 0

#define OLED_DC      5
#define OLED_CS     A4
#define OLED_RESET   6
#define WAKEUPACC   23000

#define VBATPIN   A7  // A7 = D9 !!

#define VCCMAX 4370
#define VCCMIN 3550

#define BUTTON    A1 //(to -on press)
#define BUTTON2   A3 //(to -on press)
#define BUTTON3   A5 //(to -on press)
#define POTI      A2
#define SPEAKER   10
#define LED_EFFECT  12
#define LED_BLUE    13

#define LED_WHITE  A0
#define VIBRATE    11
#define OFFSEC     20 // 5sec = 20x 1/4 sec

#include <Adafruit_GFX.h>
#include "Adafruit_SSD1331.h"

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"

#include <SPI.h>
#include <Wire.h>
#include "DS3231M.h"

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int last;
unsigned short potival;

int vccVal;
int alarm = -1;
byte fade = 200;
byte tick = 0;
int oldsec = 0;
String keymode = "         ";

//RTCdata data = {40,53,21, 3, 21,03,18}; // (3 == )Mittwoch, 21:53:40 Uhr 21.03.2018 //7=sonntag
#if SET_INIT_RTC > 0
  RTCdata data = {00,50,14, 7, 25,03,18};
#else
  RTCdata data;
#endif

int displayOnSec =  0;
int batLength    = 34;

// Color definitions
#define BLACK           0x0000
#define GREY            0b0001000010000010
#define GREYBLUE        0b0010000100010000
#define LIGHTBLUE       0b0110001000011111
#define CYAN            0x07FF
#define BLUE            0x001F
#define MAGENTA         0xF81F
#define RED             0xF800
#define RED2            0b0101100000000000
#define AMBER           0b1111101111100111
#define YELLOW          0xFFE0  
#define GREENYELLOW     0b0111111111100000
#define GREEN           0x07E0
#define WHITE           0xFFFF
#define BACKGROUND      0x0000
#define BLACK2          0b0000000000000001

#define MOD_OFF    0
#define MOD_ON     1
#define MOD_STROB  2
#define MOD_POLICE 3
#define MOD_HUI    4

int ledMode = 0;

short colors[15] = {
  RED2, BLACK, GREY, GREYBLUE, LIGHTBLUE, CYAN, BLUE, MAGENTA,
  RED, AMBER, YELLOW, GREENYELLOW, GREEN, WHITE, WHITE
};

Adafruit_SSD1331 oled = Adafruit_SSD1331(OLED_CS, OLED_DC, OLED_RESET);
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

void readVcc() {
  float mv = analogRead(VBATPIN);
  mv *= 2;
  mv *= 3.3;
  vccVal = mv;
  if (vccVal > VCCMAX) vccVal = VCCMAX;
  if (vccVal < VCCMIN) vccVal = VCCMIN;
}

short green2red(int val, int maxi) {
  // 16 bit = 5+6+5
  short result = 0x0000;
  int redPart   = 0;
  int greenPart = 0;
  if (val > (maxi/2)) {
    greenPart = 63;
    redPart = 31 - 62 * ((float) val)/((float) maxi); // 31 = 0b11111
  } else {
    redPart = 31;
    greenPart = 127 * ((float) val)/((float) maxi); // 63 = 0b111111
  }
  result += redPart  <<11;
  result += greenPart<<5;
  return result;
}

void batteryBar() {
  batteryFrame();
  readVcc();
  int val = map(vccVal, VCCMIN, VCCMAX, 0, batLength);
  int short col = WHITE;
  oled.fillRect(oled.width()-5, 2, 4, batLength-val, BACKGROUND);
  for (int v=val; v>0; --v) {
    oled.drawLine(
      oled.width()-5, batLength-v+2,
      oled.width()-2, batLength-v+2,
      green2red(v, batLength)
    );
  }
}

inline void batteryFrame() {
  oled.drawPixel(oled.width()-4, 0, WHITE);
  oled.drawPixel(oled.width()-3, 0, WHITE);
  oled.drawRect(oled.width()-6, 1, 6, batLength+2, WHITE);  
}

inline void ticking() {
  tick++;
  if (alarm>0) {
    if (oldsec != data.second) alarm-=4;
    if (alarm<1) alarm=0;
  }
  oldsec = data.second;

  // no sleep if LED ON and Mouse function
  // : set time, alarm and no-beep mode is with sleep
  if (potival > 550 && potival <= 900) {
    WDT->CTRL.reg = 0;
    while(WDT->STATUS.bit.SYNCBUSY);
    WDT->INTENSET.bit.EW   = 1;
    WDT->CONFIG.bit.PER    = 0xB;
    WDT->CONFIG.bit.WINDOW = 0x5; // 256ms
    WDT->CTRL.bit.WEN      = 1;
    while(WDT->STATUS.bit.SYNCBUSY);
  
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
    while(WDT->STATUS.bit.SYNCBUSY);
    
    WDT->CTRL.bit.ENABLE = 1;
    while(WDT->STATUS.bit.SYNCBUSY);
  
    system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
    system_sleep();
  } else {
    WDT->CTRL.reg = 0;
    while(WDT->STATUS.bit.SYNCBUSY);
  }
    
  if (displayOnSec >= 0) displayOnSec++;
  
  DS3231M_get(data);
}

void setup() {
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);

  pinMode(POTI, INPUT);
  pinMode(SPEAKER, OUTPUT);
  pinMode(LED_WHITE, OUTPUT);
  pinMode(LED_EFFECT, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(VIBRATE, OUTPUT);
    
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  ble.begin(false);
  ble.echo(false);
  ble.sendCommandCheckOK(F("AT+BleHIDEn=On"));
  //ble.sendCommandCheckOK(F("AT+BleKeyboardEn=On"));
  ble.verbose(false);
  ble.reset();
  delay(7);
  
#if SET_INIT_RTC > 0
  DS3231M_set(data);
#endif
  oled.begin();
  
  oled.fillScreen(BACKGROUND);
  oled.setTextSize(1);

  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) |
                      GCLK_GENCTRL_GENEN |
                      GCLK_GENCTRL_SRC_OSCULP32K |
                      GCLK_GENCTRL_DIVSEL;
  while(GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT |
                      GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN_GCLK2;
}

void loop() {
  ticking();
  potival = analogRead(POTI);
  if (potival > 900) {
    keymode = "[l] MOUSE  [r]";
    if (digitalRead(BUTTON2) == LOW) {
      ble.sendCommandCheckOK(F("AT+BleHidMouseButton=L,click"));
    }
    if (digitalRead(BUTTON3) == LOW) {
      ble.sendCommandCheckOK(F("AT+BleHidMouseButton=R,click"));
    }
    int x = map(GyX,-40200, 40200, -32, 31);
    int y = map(GyY, 40200,-40200, -32, 31);
    ble.print(F("AT+BleHidMouseMove="));
    ble.println(String(x)+String(',')+String(y));
    
  } else if (potival > 850) {
    keymode = "left,right    ";
    if (digitalRead(BUTTON2) == LOW) {
      ble.sendCommandCheckOK(F("AT+BLEKEYBOARDCODE=00-00-50"));
      ble.sendCommandCheckOK(F("AT+BLEKEYBOARDCODE=00-00"));
    }
    if (digitalRead(BUTTON3) == LOW) {
      ble.sendCommandCheckOK(F("AT+BLEKEYBOARDCODE=00-00-4F"));
      ble.sendCommandCheckOK(F("AT+BLEKEYBOARDCODE=00-00"));
    }
  } else if (potival > 800) {
    keymode = "up,down       ";
    if (digitalRead(BUTTON2) == LOW) {
      ble.sendCommandCheckOK(F("AT+BLEKEYBOARDCODE=00-00-52"));
      ble.sendCommandCheckOK(F("AT+BLEKEYBOARDCODE=00-00"));
    }
    if (digitalRead(BUTTON3) == LOW) {
      ble.sendCommandCheckOK(F("AT+BLEKEYBOARDCODE=00-00-51"));
      ble.sendCommandCheckOK(F("AT+BLEKEYBOARDCODE=00-00"));
    }
  } else if (potival > 750) {
    keymode = "page up/down  ";
    if (digitalRead(BUTTON2) == LOW) {
      ble.sendCommandCheckOK(F("AT+BLEKEYBOARDCODE=00-00-4B"));
      ble.sendCommandCheckOK(F("AT+BLEKEYBOARDCODE=00-00"));
    }
    if (digitalRead(BUTTON3) == LOW) {
      ble.sendCommandCheckOK(F("AT+BLEKEYBOARDCODE=00-00-4E"));
      ble.sendCommandCheckOK(F("AT+BLEKEYBOARDCODE=00-00"));
    }
  } else if (potival > 700) {
    keymode = "DEL, SPACE    ";
    if (digitalRead(BUTTON2) == LOW) {
      ble.sendCommandCheckOK(F("AT+BLEKEYBOARDCODE=00-00-2A"));
      ble.sendCommandCheckOK(F("AT+BLEKEYBOARDCODE=00-00"));
    }
    if (digitalRead(BUTTON3) == LOW) {
      ble.sendCommandCheckOK(F("AT+BLEKEYBOARDCODE=00-00-2C"));
      ble.sendCommandCheckOK(F("AT+BLEKEYBOARDCODE=00-00"));
    }
  } else if (potival > 650) {
    keymode = "Volume - / +  ";
    if (digitalRead(BUTTON2) == LOW) {
      ble.sendCommandCheckOK(F("AT+BLEHIDCONTROLKEY=VOLUME-,400"));
    }
    if (digitalRead(BUTTON3) == LOW) {
      ble.sendCommandCheckOK(F("AT+BLEHIDCONTROLKEY=VOLUME+,400"));
    }
  } else if (potival > 600) {
    keymode = "<pref    next>";
    if (digitalRead(BUTTON2) == LOW) {
      ble.sendCommandCheckOK(F("AT+BLEHIDCONTROLKEY=MEDIAPREVIOUS"));
    }
    if (digitalRead(BUTTON3) == LOW) {
      ble.sendCommandCheckOK(F("AT+BLEHIDCONTROLKEY=MEDIANEXT"));
    }
  } else if (potival > 550) {
    keymode = "stop   [>] [\"]";
    if (digitalRead(BUTTON2) == LOW) {
      ble.sendCommandCheckOK(F("AT+BLEHIDCONTROLKEY=MEDIASTOP"));
    }
    if (digitalRead(BUTTON3) == LOW) {
      ble.sendCommandCheckOK(F("AT+BLEHIDCONTROLKEY=PLAYPAUSE"));
    }
    
  } else if (potival > 500) {
    keymode = "LED Strobo   >";
    if (digitalRead(BUTTON2) == LOW) {
      ledMode = MOD_OFF;
      delay(100);
    }
    if (digitalRead(BUTTON3) == LOW) {
      ledMode = MOD_STROB;
      delay(100);
    }

  } else if (potival > 450) {
    keymode = "LED Bee-Boo  >";
    if (digitalRead(BUTTON2) == LOW) {
      ledMode = MOD_OFF;
      delay(100);
    }
    if (digitalRead(BUTTON3) == LOW) {
      ledMode = MOD_HUI;
      delay(100);
    }

  } else if (potival > 400) {
    keymode = "LED WHITE    >";
    if (digitalRead(BUTTON2) == LOW) {
      ledMode = MOD_OFF;
      delay(100);
    }
    if (digitalRead(BUTTON3) == LOW) {
      ledMode = MOD_ON;
      delay(100);
    }
    
  } else if (potival > 350) {
    keymode = "LED Police   >";
    if (digitalRead(BUTTON2) == LOW) {
      ledMode = MOD_OFF;
      delay(100);
    }
    if (digitalRead(BUTTON3) == LOW) {
      ledMode = MOD_POLICE;
      delay(100);
    }

  } else if (potival > 250) {
    keymode = "-   Alarm    +";
    if (digitalRead(BUTTON2) == LOW) {
      ledMode = MOD_OFF;
      displayOnSec=0;
      alarm = alarm-120;
      if (alarm <= 0) alarm = -1;
      delay(100);
    }
    if (digitalRead(BUTTON3) == LOW) {
      ledMode = MOD_OFF;
      displayOnSec=0;
      alarm = (alarm+120)%1200;
      delay(100);
    }

  } else if (potival > 150) {
    keymode = "-  Minutes   +";
    if (digitalRead(BUTTON2) == LOW) {
      ledMode = MOD_OFF;
      displayOnSec=0;
      data.minute = (data.minute-1)%60;
      data.second = 0;
      delay(100);
      DS3231M_set(data);
    }
    if (digitalRead(BUTTON3) == LOW) {
      ledMode = MOD_OFF;
      displayOnSec=0;
      data.minute = (data.minute+1)%60;
      data.second = 0;
      delay(100);
      DS3231M_set(data);
    }

//  } else if (potival > 50) {

  } else {
    keymode = "-   Hours    +";
    if (digitalRead(BUTTON2) == LOW) {
      ledMode = MOD_OFF;
      displayOnSec=0;
      data.hour = (data.hour-1)%24;
      delay(100);
      DS3231M_set(data);
    }
    if (digitalRead(BUTTON3) == LOW) {
      ledMode = MOD_OFF;
      displayOnSec=0;
      data.hour = (data.hour+1)%24;
      delay(100);
      DS3231M_set(data);
    }

  }
  
  if (displayOnSec >= 0) {
    oled.setTextSize(1);
    oled.setTextColor(WHITE, RED2);
    oled.setCursor(0, 2);
    oled.println(keymode);

    if (alarm>0) {
      oled.setTextColor(GREEN, BACKGROUND);
      oled.print(alarm/4);
      oled.print(" s  ");
    } else {
      oled.setTextColor(BLACK2, BACKGROUND);
      oled.print(".........");
    } 
    oled.setTextColor(YELLOW, BACKGROUND);
    oled.setCursor(5, 20);
    oled.print(" ");
    oled.print(data.day);
    oled.print(".");
    oled.print(data.month);
    oled.print(".20");      
    oled.print(data.year);
    oled.print("  ");

    oled.setTextColor(LIGHTBLUE, BACKGROUND);
    oled.setCursor(5, 28);
    switch(data.dofweek) {
      case 1:
        oled.print(" Montag    ");
        break;
      case 2:
        oled.print(" Dienstag  ");
        break;
      case 3:
        oled.print(" Mittwoch  ");
        break;
      case 4:
        oled.print(" Donnerstag");
        break;
      case 5:
        oled.print(" Freitag   ");
        break;
      case 6:
        oled.print(" Samstag   ");
        break;
      case 7:
        oled.print(" Sonntag   ");
        break;
      default:
        ;
    }
    
    oled.setTextSize(2);
    oled.setTextColor(WHITE, BACKGROUND);
    oled.setCursor(0, 46);
    if (data.hour<10) oled.print("0");
    oled.print(data.hour);
    oled.print(":");
    if (data.minute<10) oled.print("0");
    oled.print(data.minute);
    oled.print(":");
    if (data.second<10) oled.print("0");
    oled.print(data.second);
  }
  
  if ( digitalRead(BUTTON) == LOW || ((last -(int)GyY) > WAKEUPACC) ) {
    displayOnSec=0;
    batteryBar();
    oled.writeCommand(SSD1331_CMD_DISPLAYON);
    delay(200);
    if (digitalRead(BUTTON) == LOW && alarm == 0) alarm = -1;
  }

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers

  last = GyY;
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      
  if (GyY > 17200) GyY = 17200;
  if (GyY < -17200) GyY = -17200;
  if (GyX > 17200) GyX = 17200;
  if (GyX < -17200) GyX = -17200;
  
  if (displayOnSec > OFFSEC) {
    oled.fillScreen(BLACK);
    oled.writeCommand(SSD1331_CMD_DISPLAYOFF);
    displayOnSec = -1;
  }
  
  if (alarm == 0) {
    if (tick%2==0) {
      analogWrite(SPEAKER, 0);
      analogWrite(LED_BLUE, 0);
      digitalWrite(LED_WHITE, LOW);
      digitalWrite(LED_EFFECT, HIGH);
      digitalWrite(VIBRATE, LOW);      
    } else {
      analogWrite(SPEAKER, 50);
      analogWrite(LED_BLUE, 255);
      digitalWrite(LED_WHITE, HIGH);
      digitalWrite(LED_EFFECT, LOW);
      digitalWrite(LED_BLUE, HIGH);
      digitalWrite(VIBRATE, HIGH);      
    }  
  } else {
    if (ledMode == MOD_OFF) {
      analogWrite(SPEAKER, 0);
      analogWrite(LED_BLUE, 0);
      digitalWrite(LED_WHITE, LOW);
      digitalWrite(LED_EFFECT, LOW);
      digitalWrite(VIBRATE, LOW); 
    } else if (ledMode == MOD_ON) {
      analogWrite(SPEAKER, 0);
      analogWrite(LED_BLUE, 0);
      digitalWrite(LED_WHITE, HIGH);
      digitalWrite(LED_EFFECT, LOW);
      digitalWrite(VIBRATE, LOW);      
    } else if (ledMode == MOD_STROB) {
      analogWrite(SPEAKER, 0);
      analogWrite(LED_BLUE, 0);
      digitalWrite(LED_WHITE, tick%2==0);
      digitalWrite(LED_EFFECT, LOW);
      digitalWrite(VIBRATE, LOW);
      delay(20);
    } else if (ledMode == MOD_POLICE) {
      analogWrite(SPEAKER, 0);
      analogWrite(LED_BLUE, tick%2==0? 0 : 255);
      digitalWrite(LED_EFFECT, tick%2==0);
      digitalWrite(VIBRATE, LOW);
      delay(100);
    } else if (ledMode == MOD_HUI) {
      analogWrite(SPEAKER, (GyY/150)+(tick%64));
      analogWrite(LED_BLUE, 0);
      digitalWrite(LED_WHITE, (GyX/150)>70);
      digitalWrite(LED_EFFECT, LOW);
      digitalWrite(VIBRATE, (GyZ/150)>70); 
    }
  }
}


