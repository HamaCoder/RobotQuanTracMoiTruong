/* ================================================
 *     CHƯƠNG TRÌNH ĐIỀU KHIỂN ROBOT QUAN TRẮC
 *                   code by Hama
 *     DỰ ÁN PHỤC VỤ ĐỀ TÀI LUẬN VĂN TỐT NGHIỆP
 *    NGÀNH KỸ THUẬT CƠ ĐIỆN TỬ K43 NĂM 2021-2022
 *    Tên dự án: Robot quan trắc môi trường.
 *    Thời gian thực hiện: 12 tháng.
 *    Từ 1/2021 đến 12/2021.
 *    Giảng viên hướng dẫn: Thạc sĩ Trần Lê Trung Chánh.
 *    Sinh viên thực hiện:
 *      + Nguyễn Trọng Thức
 *      + Nguyễn Nhất Phẩm
 *    Thông tin phần cứng:
 *    Dự án sử dụng vi điều khiển: STM32F103RC và STM32F407VET
 *    Kết hợp ứng dụng android hiển thị dữ liệu quan trắc.
 *    Ngôn ngữ lập trình: C, C++, Java, JavaCript, HTML, CSS.
 *    
 *    Thông tin source code:
 *    Ngôn ngữ: C, C++.
 *    Ứng dụng: Điều khiển hoạt động của Robot.
 *    Điều khiển phần cứng:
 *      LCD 16x2 (or OLED, Oled TFT)
 *      RF24L01
 *      Sensor MQ02, MQ04, MQ09
 *      Bơm DC
 *      Phao điện tử
 *      Động cơ DC 12V 775
 *      GPS
 *    Thông tin vi điều khiển:
 *    RF24L01:
 *        VCC   +3.3V
 *        GND   GND
 *        CE    PB6
 *        CSN   PB7
 *        SCK   PB3   
 *        MOSI  PB5
 *        MISO  PB4
 *        IRQ   PB8
 *    LCD - I2C:
 *        VCC   +5V
 *        GND   GND
 *        SDA   PB9
 *        SCL   PB8
 *    UART:
 *        TX    PA9
 *        RX    PA10
 *    VNH2SP30:
 *        EN_DC1  PE5
 *        EN_DC2  PE6
 *        IN1     PE7
 *        IN2     PE8
 *        IN3     PE9
 *        IN4     PE10
 *        PWM1    PC4
 *        PWM2    PC5
 *        CS1     PC2
 *        CS2     PC3
 *    Ngoại vi:
 *        Phao    PE12
 *        Bơm     PE11
 *        MQ02    PA1
 *        MQ04    PA2
 *        MQ09    PA3
 *        LEDR    PE13
 *        LEDG    PE14
 *        LEDB    PE15
 *    
 * ================================================
 */

#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <LiquidCrystal_I2C.h>
#include <MQ2.h>

#define EN_DC1  PE5
#define EN_DC2  PE6
#define IN1     PE7
#define IN2     PE8
#define IN3     PE9
#define IN4     PE10
#define PWM1    PC4
#define PWM2    PC5
#define CS1     PC2
#define CS2     PC3
#define bom     PE11
#define phao    PE12
#define LEDR    PE13
#define LEDG    PE14
#define LEDB    PE15
#define readMQ2 PA1
#define readMQ4 PA2
#define readMQ9 PA3
#define rf_CE   PB6
#define rf_CS   PB7

MQ2 mq2(readMQ2);
MQ2 mq4(readMQ4);

RF24 radio(rf_CE, rf_CS);
LiquidCrystal_I2C lcd(0x27, 16, 2);
const byte address[][6] = {"00001", "00002"};
unsigned long readTime = 0;
unsigned long oldTime = 0;

struct Data_Package {
  byte getx1;
  byte gety1;
  byte getx2;
  byte gety2;
  byte getTWO;
  byte getDC;
  byte getHopSo;
  byte getSaveSpeed;
  byte getBom;
  byte getLedColor;
  byte getBlinkLed;
  byte getChangeGPS;
};
Data_Package datal;


struct Data_Robot {
  byte  id;
  float setLPG;
  float setCH4;
  float setCO;
  float setH2;
  int   setDistance;
  byte  setPhao;
  byte  setTrangThai;
  byte  setKetNoi;
  byte  setTocDo;
};
Data_Robot robot;


 
float gpslat[24] = {
10.0351414,10.0352652,10.0353910,10.0355146,10.0356135,
10.0356674,10.0356156,10.0355415,10.0354516,10.0354291,
10.0353550,10.0352493,10.0351954,10.0351549,10.0351459,
10.0351481,10.0350784,10.0350266,10.0349748,10.0350176,
10.0350829,10.0351796
};
float gpslng[24] = {
105.7669564,105.7669588,105.7669678,105.7669403,105.7668764,
105.7667736,105.7666618,105.7666230,105.7666412,105.7665294,
105.7664746,105.7664974,105.7665796,105.7666868,105.7668011,
105.7669313,105.7669061,105.7668239,105.7667440,105.7668354,
105.7669267,105.7669382
};


int tx1, tx2, ty1, ty2;
int all       = 0;
int DC        = 0;
int hopSo     = 0;
int saveSpeed = 0;
int maxSpeed  = 0;
int trangThai = 0;
int tocdo1    = 0;
int tocdo2    = 0;
int pump      = 0;
int ledColor  = 0;
int ledBlink  = 0;
int timeLED   = 0;
boolean blinkLED = false;

float LPG_MQ2;
float CO_MQ2;
float Smoke;
float CH4_MQ4;
float LPG_MQ9;
float CO_MQ9;
float CH4_MQ9;

int tocDo;
int readSensor;
int khoangCach;
float sensor_volt;
float RS;
float ratio;
float sensor_MQ9;
float LPG, CH4, CO, H2;
float begin_xmap = 10.03482303;
float begin_ymap = 105.7677079;
float latMap;
float lngMap;
int readlatlng   = 0;
int ketNoi       = 0;

void setup() {
  Serial.begin(115200);
  
  radio.begin();
  radio.setAutoAck(false);
  radio.openReadingPipe(1, address[1]);
  radio.openWritingPipe(address[0]);
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(26);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("   Hello sky!   ");
  delay(200);
  lcd.setCursor(0,1);
  lcd.print("  I am a Robot  ");

  mq2.begin();
  mq4.begin();

  pinMode(EN_DC1, OUTPUT);
  pinMode(EN_DC2, OUTPUT);
  pinMode(IN1,    OUTPUT);
  pinMode(IN2,    OUTPUT);
  pinMode(IN3,    OUTPUT);
  pinMode(IN4,    OUTPUT);
  pinMode(PWM1,   OUTPUT);
  pinMode(PWM2,   OUTPUT);
  pinMode(CS1,    OUTPUT);
  pinMode(CS2,    OUTPUT);
  pinMode(bom,    OUTPUT);
  pinMode(LEDR,   OUTPUT);
  pinMode(LEDG,   OUTPUT);
  pinMode(LEDB,   OUTPUT);

  pinMode(phao,   INPUT_PULLUP);
  pinMode(readMQ2,INPUT);
  pinMode(readMQ4,INPUT);
  pinMode(readMQ9,INPUT);

}

void loop() {

  mq2.read();
  
  delay(10);
  radio.startListening();
  readTime = millis();
  if(readTime - oldTime > 1000){
    resetData();
  }
  if (radio.available()){
    oldTime = millis();
    radio.read(&datal, sizeof(Data_Package));
    tx1 = datal.getx1;
    ty1 = datal.gety1;
    tx2 = datal.getx2;
    ty2 = datal.gety2;
    all = datal.getTWO;
    DC  = datal.getDC;
    hopSo     = datal.getHopSo;
    saveSpeed = datal.getSaveSpeed;
    pump      = datal.getBom;
    ledColor  = datal.getLedColor;
    ledBlink  = datal.getBlinkLed;
    ketNoi    = 1;
  }


/*
 * ===== CÀI ĐẶT TỐC ĐỘ THEO HỘP SỐ =====
 * Cấp số 0: số dừng.
 * Cấp số 1: số lùi.
 * Cấp số 2: số tiến chậm.
 * Cấp số 3: số tiến vừa.
 * Cấp số 4: số tiến nhanh.
 */
  if (hopSo == 0){
    maxSpeed = 0;
  } else if (hopSo == 1){
    maxSpeed = 80;
  } else if (hopSo == 2){
    maxSpeed = 90;
    if (saveSpeed == 1){
      maxSpeed = 80;
    }
  } else if (hopSo == 3){
    maxSpeed = 160;
    if (saveSpeed == 1){
      maxSpeed = 150;
    }
  } else if (hopSo == 4){
    maxSpeed = 255;
    if (saveSpeed == 1){
      maxSpeed = 230;
    }
  }

/*
 * ===== CHƯƠNG TRÌNH CHẠY 2 ĐỘNG CƠ ====
 * Chạy lên
 * Chạy lui
 * Sang trái
 * Sang phải
 * Dừng
 */
  if (all == 1){
    digitalWrite(EN_DC1, HIGH);
    digitalWrite(EN_DC2, HIGH);

    if (tx2 > 140 && hopSo > 1 && saveSpeed == 0){
      // Robot đi lên
      trangThai = 1;
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      tocdo1 = map(tx2, 140, 255, 0, maxSpeed);
      tocdo2 = map(tx2, 140, 255, 0, maxSpeed);
    } else if (tx2 < 110 && hopSo == 1){
      // Robot đi lùi
      trangThai = 4;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      tocdo1 = map(tx2, 110, 0, 0, maxSpeed);
      tocdo2 = map(tx2, 110, 0, 0, maxSpeed);
    } else if (tx2 > 140 && hopSo == 1){
      // Robot đi lùi
      trangThai = 4;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      tocdo1 = map(tx2, 140, 255, 0, maxSpeed);
      tocdo2 = map(tx2, 140, 255, 0, maxSpeed);
    } else if (ty2 < 110 && hopSo == 1){
      // Robot lùi trái
      trangThai = 11;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      tocdo1 = map(ty2, 110, 0, 0, maxSpeed);
      tocdo2 = map(ty2, 110, 0, 0, maxSpeed);
    } else if (ty2 > 140 && hopSo == 1){
      // Robot lùi phải
      trangThai = 12;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      tocdo1 = map(ty2, 140, 255, 0, maxSpeed);
      tocdo2 = map(ty2, 140, 255, 0, maxSpeed);
    } else if (ty2 < 110 && hopSo > 1){
      // Robot rẻ trái
      trangThai = 2;
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      tocdo1 = map(ty2, 110, 0, 0, maxSpeed);
      tocdo2 = 0;
    } else if(ty2 > 140 && hopSo > 1){
      // Robot rẻ phải
      trangThai = 3;
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      tocdo1 = 0;
      tocdo2 = map(ty2, 140, 255, 0, maxSpeed);
    } else {
      // Robot dừng
      if (saveSpeed == 0){
        trangThai = 0;
        tocdo1    = 0;
        tocdo2    = 0;
      } else {
        trangThai = 1;
      }
      
    }
  }

/*
 * ==== CHƯƠNG TRÌNH ĐỘNG CƠ TRÁI ====
 * Chạy lên
 * Chạy lùi
 * Dừng
 */
  else if (DC == 1){
    digitalWrite(EN_DC1, HIGH);
    if (tx2 < 110 && hopSo == 1){
      // Động cơ 1 đi lùi
      trangThai = 5;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      tocdo1 = map(tx2, 110, 0, 0, maxSpeed);
    } else if(tx2 > 140 && hopSo == 1){
      // Động cơ 1 đi lùi
      trangThai = 5;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      tocdo1 = map(tx2, 140, 255, 0, maxSpeed);
    } else if(tx2 > 140 && hopSo > 1){
      // Động cơ 1 đi lên
      trangThai = 6;
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      tocdo1 = map(tx2, 140, 255, 0, maxSpeed);
    } else {
      // Động cơ 1 dừng
      trangThai = 7;
      tocdo1    = 0;
    }
  }

/*
 * ==== CHƯƠNG TRÌNH ĐỘNG CƠ PHẢI ====
 * Chạy lên
 * Chạy lùi
 * Dừng
 */
  else if (DC == 2){
    digitalWrite(EN_DC2, HIGH);
    if (tx2 < 110 && hopSo == 1){
      // Động cơ 2 đi lùi
      trangThai = 8;
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      tocdo2 = map(tx2, 110, 0, 0, maxSpeed);
    } else if (tx2 > 140 && hopSo == 1){
      // Động cơ 2 đi lùi
      trangThai = 8;
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      tocdo2 = map(tx2, 140, 255, 0, maxSpeed);
    } else if (tx2 > 140 && hopSo > 1){
      // Động cơ 2 đi lên
      trangThai = 9;
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      tocdo2 = map(tx2, 140, 255, 0, maxSpeed);
    } else {
      // Động cơ 2 dừng
      trangThai = 10;
      tocdo2    = 0;
    }
  }

/*
 * ==== CHƯƠNG TRÌNH DỪNG ROBOT ====
 * Gửi trạng thái về bộ điều khiển
 * Thiết lập tốc độ pwm
 */
  else {
    // Robot dừng
    digitalWrite(EN_DC1, LOW);
    digitalWrite(EN_DC2, LOW);
    trangThai = 0;
    tocdo1    = 0;
    tocdo2    = 0;
  }

  if (tocdo1 < 70){
    tocdo1 = 0;
  }
  if (tocdo2 < 70){
    tocdo2 = 0;
  }

  analogWrite(PWM1, tocdo1);
  analogWrite(PWM2, tocdo2);

/*
 * ==== CHƯƠNG TRÌNH CHUYỂN ĐỔI VẬN TỐC ROBOT ====
 * Chuyển đổi theo chương trình 2 động cơ hoặc chỉ động cơ trái
 * Chuyển đổi theo chương trình động cơ phải
 */
  if (all == 1 || DC == 1){
    tocDo = map(tocdo1, 0, 255, 0, 20);
  } else {
    tocDo = map(tocdo2, 0, 255, 0, 20);
  }

  
  Serial.print("tocdo1-"+String(tocdo1));
  Serial.print("-tocdo2-"+String(tocdo2));
  Serial.print("-Phao-"+String(digitalRead(phao)));
  Serial.print("-Hopso-"+String(hopSo));
  Serial.print("-All-"+String(all));
  Serial.print("-DC-"+String(DC));

/*  
 * ==== CHƯƠNG TRÌNH ĐIỀU KHIỂN BƠM ====
 * Nếu phao đầy thì không bật bơm
 * Ngược lại thì bật bơm.
 */
  if (pump == 1){
    if (digitalRead(phao) == 0){
      digitalWrite(bom, HIGH);
    }
  } else {
    digitalWrite(bom, LOW);
  }

/*
 * ==== CHƯƠNG TRÌNH ĐIỀU KHIỂN LED ====
 * Kiểm tra yêu cầu có nhấp nháy LED hay không
 * Nếu ledBlink = 0 thì không nhấp nháy
 * Nếu ledColor = 0 => tắt
 *     ledColor = 1 => sáng màu vàng
 *     ledColor = 2 => sáng màu trắng
 * Nếu ledBlink = 1 thì nhấp nháy LED
 * Nếu blinkLED = true thì chạy bộ đếm thời gian timeLED
 * Nếu ledColor = 0 => tắt
 *     ledColor = 1 => nhấp nháy màu vàng
 *     ledColor = 2 => nhấp nháy màu trắng
 * Nếu blinkLED = false thì timeLED = 0
 */
  if (ledBlink == 0){
    blinkLED = false;
    switch(ledColor){
      case 0:
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, HIGH);
        digitalWrite(LEDB, HIGH);
      break;
      case 1:
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, LOW);
        digitalWrite(LEDB, LOW);
      break;
      case 2:
        digitalWrite(LEDR, LOW);
        digitalWrite(LEDG, LOW);
        digitalWrite(LEDB, LOW);
      break;
    }
  } else {
    blinkLED = true;
    switch(ledColor){
      case 0:{
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, HIGH);
        digitalWrite(LEDB, HIGH);
        break;
      }
      case 1:{
        if (timeLED < 30){
          digitalWrite(LEDR, HIGH);
          digitalWrite(LEDG, LOW);
          digitalWrite(LEDB, LOW);
        } else {
          digitalWrite(LEDG, HIGH);
          digitalWrite(LEDB, HIGH);
          if (timeLED > 60){
            timeLED = 0;
          }
        }
        break;
      }
      case 2:{
        if (timeLED < 30){
          digitalWrite(LEDR, LOW);
          digitalWrite(LEDG, LOW);
          digitalWrite(LEDB, LOW);
        } else {
          digitalWrite(LEDR, HIGH);
          digitalWrite(LEDG, HIGH);
          digitalWrite(LEDB, HIGH);
          if (timeLED > 60){
            timeLED = 0;
          }
        }
        break;
      }
    }
  }
  if (blinkLED){
    timeLED++;
  } else {
    timeLED = 0;
  }
  
/*
 * Tắt trạng thái nhận tín hiệu RF
 */
  delay(10);
  radio.stopListening();

/*
 * ========== ĐỌC TÍN HIỆU CẢM BIẾN ==========
 */
  readSensor++;
  if (readSensor == 40){
    readSensor = 0;
    LPG_MQ2 = mq2.readLPG(),2;
    CO_MQ2  = mq2.readCO(), 2;

    int adc = analogRead(readMQ4);
    sensor_volt = (adc * 5.0) / 1023.0;
    RS = ((5.0 * 10.0) / sensor_volt) - 10.0;
    ratio = RS / 11.82;
    double ppm_CH4 = ((log10(ratio) - 0.27) / -0.38) + 2.3;
    double ppm_H2  = ((log10(ratio) - 0.57) / -0.16) + 2.3;
    double ppm_Metan = pow(10, ppm_CH4);
    double ppm_Hidro = pow(10, ppm_H2);
    CH4_MQ4 = ppm_Metan, 2;
    H2      = ppm_H2, 2;

    sensor_MQ9 = analogRead(readMQ4);
    float RS_MQ9 = sensor_MQ9 * 3.3 / 4095.0;
    LPG_MQ9 = 26.572 * exp(1.2894 * RS_MQ9);
    CO_MQ9  = 10.938 * exp(1.7742 * RS_MQ9);
    CH4_MQ9 =  3.027 * exp(1.0698 * RS_MQ9);

    LPG = (LPG_MQ2 + LPG_MQ9) / 2.0;
    CH4 = (CH4_MQ4 + CH4_MQ9) / 2.0;
    CO = (CO_MQ2 + CO_MQ9) / 2.0;

    robot.setLPG = LPG;
    robot.setCH4 = CH4;
    robot.setCO  = CO;
    robot.setH2  = H2;
    
    if (readlatlng >= 24){
      readlatlng = 0;
      khoangCach = 0;
    }
    latMap = gpslat[readlatlng];
    lngMap = gpslng[readlatlng];
    readlatlng++;

    khoangCach = khoangcach(begin_xmap, begin_ymap, latMap, lngMap);

  } else {
    robot.setLPG = LPG;
    robot.setCH4 = CH4;
    robot.setCO  = CO;
    robot.setH2  = H2;
  }

/*
 * ========== GỬI TÍN HIỆU RF ===========
 */
  robot.id           = 2;
  robot.setPhao      = digitalRead(phao);
  robot.setKetNoi    = ketNoi;
  robot.setTocDo     = tocDo;
  robot.setDistance  = khoangCach;
  robot.setTrangThai = trangThai;

  radio.write(&robot, sizeof(Data_Robot));

  
/*
 * ========== GỬI DỮ LIỆU GPS ===========
 */
  delay(10);
  robot.id     = 7;
  robot.setLPG = latMap;
  robot.setCH4 = lngMap;
  radio.write(&robot, sizeof(Data_Robot));


  Serial.print("-Ket noi-"+String(robot.setKetNoi));
  Serial.print("-Trang thai-"+String(trangThai)+"-");
  Serial.println(String(LPG)+"-"+String(CH4)+"-"+String(CO)+"-"+String(H2)+"-"+
  String(latMap,7)+"-"+String(lngMap,7)+"-"+String(khoangCach)+"-"+String(sizeof(robot)));

}

void resetData(){
  datal.getx1 = 127;
  datal.gety1 = 126;
  datal.getx2 = 125;
  datal.gety2 = 124;
  datal.getTWO = 0;
  datal.getDC  = 0;
  datal.getHopSo = 1;
  datal.getSaveSpeed = 0;
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
}
void in_lcd(int cot, int hang, String txt){
  lcd.setCursor(cot, hang);
  lcd.print(txt);
}
int khoangcach(float x1, float y1, float x2, float y2){
  float R = 6371.0;
  float dLat = (x2 - x1)*(3.14 / 180);
  float dLng = (y2 - y1)*(3.14 / 180);
  float la1ToRad = x1 * (3.14 / 180);
  float la2ToRad = x2 * (3.14 / 180);
  float a = sin(dLat / 2)*sin(dLat / 2) + cos(la1ToRad)
           * cos(la2ToRad)*sin(dLng / 2)*sin(dLng / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float d = R * c;
  return d*1000;
}
