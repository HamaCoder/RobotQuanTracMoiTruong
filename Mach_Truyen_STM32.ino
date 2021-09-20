
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>


#define joy1  PC1
#define joy2  PC13
#define jx1   PC0
#define jy1   PB1
#define jx2   PC2
#define jy2   PC3
#define btn1  PD2
#define btn2  PB5
#define btn3  PC11
#define btn4  PC12
#define btn5  PC10
#define btn6  PC8
#define btn7  PC9
#define bt_RX PA10
#define bt_TX PA9
#define rf_CE PA4
#define rf_CS PC4

RF24 radio(rf_CE, rf_CS);
LiquidCrystal_I2C lcd(0x27, 20, 4);
const byte address[][6] = {"00001","00002"};
SoftwareSerial Bluetooth(bt_RX, bt_TX);


struct Data_Package {
  byte setx1;
  byte sety1;
  byte setx2;
  byte sety2;
  byte setTWO;
  byte setDC;
  byte setHopSo;
  byte setSaveSpeed;
  byte setBom;
  byte setLedColor;
  byte setBlinkLed;
  byte setChangeGPS;
};
Data_Package sent;


struct Data_Robot {
  byte  id;
  float getLPG;
  float getCH4;
  float getCO;
  float getH2;
  int   getDistance;
  byte  getPhao;
  byte  getTrangThai;
  byte  getKetNoi;
  byte  getTocDo;
};
Data_Robot robot;


boolean sentRadio = true;
boolean bluetooth = false;
boolean all       = false;
boolean motor1    = false;
boolean motor2    = false;
boolean saveSpeed = false;
boolean button1   = false;
boolean button2   = false;
int     manhinh   = 1;
int     motor     = 0;
int     hopSo     = 0;
int     pump      = 0;
int     trangThai = 0;
int     ketNoi    = 0;
int     turnLed   = 0;
int     blinkLed  = 0;
int     tocDo, khoangCach;
int     tx1, ty1, tx2, ty2;
float   LPG, CH4, H2, CO;
float   latMap, lngMap;
String  ketnoi;

int     delay_bluetooth = 0;


void setup() {
  Serial.begin(9600);
  Bluetooth.begin(9600);
  
  radio.begin();
  radio.setAutoAck(false);
  radio.openWritingPipe(address[1]);
  radio.openReadingPipe(1, address[0]);
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(26);

  lcd.init();
  lcd.backlight();

  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  pinMode(btn3, INPUT_PULLUP);
  pinMode(btn4, INPUT_PULLUP);
  pinMode(btn5, INPUT_PULLUP);
  pinMode(btn6, INPUT_PULLUP);
  pinMode(btn7, INPUT_PULLUP);
  pinMode(joy1, INPUT_PULLUP);
  pinMode(joy2, INPUT_PULLUP);
  pinMode(jx1,  INPUT);
  pinMode(jy1,  INPUT);
  pinMode(jx2,  INPUT);
  pinMode(jy2,  INPUT);
  
}



void loop() {

/*
 * Tắt trạng thái đọc tín hiệu RF.
 */
  delay(10);
  radio.stopListening();

/*
 * Ánh xạ, chuyển đổi tín hiệu analog của joytick.
 */
  tx1 = map(analogRead(jx1), 0, 965, 255, 0);
  ty1 = map(analogRead(jy1), 0, 965, 255, 0);
  tx2 = map(analogRead(jx2), 0, 965, 255, 0);
  ty2 = map(analogRead(jy2), 0, 965, 255, 0);

/*
 * ============== BUTTON 1 ===============
 * Điều khiển bật/tắt kết nối RF.
 */
  if (digitalRead(btn1) == LOW){
    delay(50);
    sentRadio = !sentRadio;
    lcd.clear();
  }

/*
 * ============== BUTTON 2 ===============
 * Điều khiển bật/tắt 2 động cơ.
 */
  if (digitalRead(btn2) == LOW){
    delay(50);
    manhinh = 1;
    all = !all;
    if (all == true){
      chay_hai_dongco();
    } else {
      hopSo = 0;
    }
    lcd.clear();
  }

/*
 * ============== BUTTON 3 ===============
 * Điều khiển bật/tắt động cơ 1/động cơ 2
 */
  if (digitalRead(btn3) == LOW){
    delay(50);
    manhinh = 1;
    motor++;
    if (motor == 1){
      chay_dongco_trai();
    } else if(motor == 2){
      chay_dongco_phai();
    } else {
      motor = 0;
      motor2 = false;
      hopSo = 0;
    }
    lcd.clear();
  }

/*
 * ============= ĐIỀU KHIỂN 2 ĐỘNG CƠ =============
 * Joytick1 : điều khiển cấp hộp số.
 * Joytick2 : điều khiển tốc độ động cơ.
 */
  if (all){ 
    if (tx1 < 50)  hopSo = 1;
    if (ty1 < 50)  hopSo = 2;
    if (tx1 > 200) hopSo = 3;
    if (ty1 > 200) hopSo = 4;
  } else {
    sent.setTWO = 0;
  }

/*
 * ============= ĐIỀU KHIỂN ĐỘNG CƠ 1-2 ==============
 * Joytick1 : điều khiển cấp hộp số.
 * Joytick2 : điều khiển tốc độ động cơ.
 */
  if (motor1){
    in_lcd(0, 1, "Motor1");
    if (tx1 < 50)  hopSo = 1;
    if (ty1 < 50)  hopSo = 2;
    if (tx1 > 200) hopSo = 3;
    if (ty1 > 200) hopSo = 4;
  } else if (motor2 == true){
    in_lcd(0, 1, "Motor2");
    if (tx1 < 50)  hopSo = 1;
    if (ty1 < 50)  hopSo = 2;
    if (tx1 > 200) hopSo = 3;
    if (ty1 > 200) hopSo = 4;
  } else {
    sent.setDC = 0;
  }

/*
 * ========= BUTTON 4 =========
 * Chuyển về màn hình trước
 */
  if (digitalRead(btn4) == LOW){
    delay(50);
    manhinh--;
    if (manhinh < 1){
      manhinh = 5;
    }
    all = false;
    motor1 = false;
    motor2 = false;
    lcd.clear();
  }

/*
 * ========= BUTTON 5 =========
 * Chuyển về màn hình sau
 */
  if (digitalRead(btn5) == LOW){
    delay(50);
    manhinh++;
    if (manhinh > 5){
      manhinh = 1;
    }
    all = false;
    motor1 = false;
    motor2 = false;
    lcd.clear();
  }

/*
 * ========= BUTTON 6 =========
 * Bật đèn
 * Bật đèn vàng hoặc trắng
 */
  if (digitalRead(btn6) == LOW){
    delay(50);
    lcd.clear();
    turnLed++;
    if (turnLed > 2){
      turnLed = 0;
      blinkLed = 0;
    }
    manhinh = 5;
  }

/*
 * ========= BUTTON 7 =========
 * Bật/tắt trạng thái nhấp nháy đèn
 */
  if (digitalRead(btn7) == LOW){
    delay(50);
    lcd.clear();
    blinkLed++;
    if (blinkLed > 1){
      blinkLed = 0;
    }

    if (blinkLed == 1){
      manhinh = 5;
      if (turnLed == 0){
        turnLed = 1;
      }
    }
  }


/*
*========== Màn hình 1 ===========
*Hiển thị trạng thái:
*Bơm, vận tốc, hộp số, khoảng cách, pin.
*/
  if (manhinh == 1){
    in_lcd(14, 1, "-");
    in_lcd(8,  2, "(km/h)");
    in_lcd(14, 2, "Gear:");
    in_lcd(19, 2, String(hopSo));
    in_lcd(0,  3, "Distance:");
    in_lcd(0,  3, "");
    in_lcd(13, 3, "m");
    
    if (all && !motor1 && !motor2){
      in_lcd(0, 1, "2Motor");
    } else if (!motor1 && !motor2 && !all){
      in_lcd(0, 1, "Status");
    }

    if (button2 && (hopSo == 2 || hopSo == 3 || hopSo == 4)){
      in_lcd(0,  2, "Hold: ");
      sent.setSaveSpeed = 1;
    } else {
      in_lcd(0,  2, "Speed:");
      if (tocDo > 9){
        in_lcd(6, 2, String(tocDo));
      } else {
        in_lcd(6, 2, "0"+String(tocDo));
      }
      sent.setSaveSpeed = 0;
      button2 = false;
    }
  
    if (!sentRadio){
      in_lcd(5, 0, "Disconnect");
    }


    if (khoangCach < 10){
      in_lcd(9, 3, "000"+String(khoangCach));
    } else if (khoangCach > 9 && khoangCach < 100){
      in_lcd(9, 3, "00"+String(khoangCach));
    } else if (khoangCach > 99 && khoangCach < 1000){
      in_lcd(9, 3, "0"+String(khoangCach));
    } else {
      in_lcd(9, 3, String(khoangCach));
    }
  }
  
/*
*============= Màn hình 2 =============
*Hiển thị trạng thái kết nối
*Bluetooth, RF, vị trí joytick2
*/
  if (manhinh == 2){
    in_lcd(0, 0, "Connect to:");
    in_lcd(15, 0,"LCD:2");
    if (bluetooth)   in_lcd(0, 1, "Bluetooth: yes");
    else             in_lcd(0, 1, "Bluetooth: no ");

    if (ketNoi == 0) in_lcd(0, 2, "Robot:     no ");
    else             in_lcd(0, 2, "Robot:     yes");

    in_lcd(0,  3, "tx2"+String(tx2));
    in_lcd(11, 3, "ty2"+String(ty2));
    
  }

/*
*============= Màn hình 3 =============
*Hiển thị thông số quan trắc
*LPG, H2, CH4, CO
*/
  if (manhinh == 3){
    in_lcd(0,  0, "Monitoring:");
    in_lcd(15, 0,"LCD:3");
    in_lcd(0,  1, "LPG:");
    in_lcd(4,  1, String(LPG));
    in_lcd(9,  1, "ppm");
    in_lcd(12, 1, "H2:");
    in_lcd(15, 1, String(H2));
    in_lcd(0,  2, "CH4:");
    in_lcd(9,  2, "ppm");
    in_lcd(4,  2, String(CH4));
    in_lcd(12, 2, "CO:");
    in_lcd(15, 2, String(CO));
    in_lcd(0,  3, "Pump:");

    if (button1 && pump == LOW){
      in_lcd(6, 3, "ON  ");
      sent.setBom = 1;
    } else if (pump == HIGH){
      in_lcd(6, 3, "Full");
      sent.setBom = 0;
      button1 = false;
    } else {
      in_lcd(6, 3, "OFF ");
      sent.setBom = 0;
    }
  }


/*
*============= Màn hình 4 =============
*Hiển thị vị trí robot
*Latitude
*Longitude
*/
  if (manhinh == 4){
    in_lcd(0, 0, "Location Robot");
    in_lcd(15, 0,"LCD:4");
    in_lcd(0, 1, "Lat: ");
    in_lcd(5,1, String(latMap,7));
    in_lcd(0, 2, "Lng: ");
    in_lcd(5,2, String(lngMap,7));
  }

/*
*============= Màn hình 5 =============
*  Trạng thái led trên robot
*  Màu led
*  Nhấp nháy led.
*/
  if (manhinh == 5){
    in_lcd(0, 0, "Light Robot");
    in_lcd(15,0, "LCD:5");
    in_lcd(0, 1, "Led turn: ");
    in_lcd(0, 2, "Led color:");
    in_lcd(0, 3, "Led blink:");

    switch(turnLed){
      case 0:
        in_lcd(10, 1, " OFF");
        in_lcd(10, 2, "       ");
      break;
      case 1:
        in_lcd(10, 1, " ON ");
        in_lcd(10, 2, " Yellow");
      break;
      case 2:
        in_lcd(10, 1, " ON ");
        in_lcd(10, 2, " White ");
      break;
    }

    switch(blinkLed){
      case 0:
        in_lcd(10, 3, " OFF");
      break;
      case 1:
        in_lcd(10, 3, " ON");
      break;
    }

  }
  


/*
 * ================= BUTTON JOYTICK 1 =================
 * Điều khiển bật/tắt bơm
 */
  if (digitalRead(joy1) == LOW){
    delay(50);
    manhinh = 3;
    button1 = !button1;
    lcd.clear();
  }
  
/*
 * ================= BUTTON JOYTICK 2 =================
 * Điều khiển giữ (duy trì) tốc độ động cơ.
 */
  if (digitalRead(joy2) == LOW && manhinh == 1){
    delay(50);
    button2 = !button2;
    if (button2 == true && (hopSo == 2 || hopSo == 3 || hopSo == 4)){
      lcd.clear();
    } else {
      lcd.clear();
      button2 = false;
    }
  }

  guiDuLieu();
  
  
  //============================================\\
  
/*
 * Bật trạng thái nhận tín hiệu RF
 */

  int getKetNoi = 0;
  delay(10);
  radio.startListening();
  if(radio.available()){
    radio.read(&robot, sizeof(Data_Robot));
  }

  if(robot.id == 2) {
    LPG = robot.getLPG;
    CH4 = robot.getCH4;
  }
  delay(10);
  if(robot.id == 7) {
    latMap = robot.getLPG;
    lngMap = robot.getCH4;
  }
  
  H2         = robot.getH2;
  CO         = robot.getCO;
  trangThai  = robot.getTrangThai;
  pump       = robot.getPhao;
  tocDo      = robot.getTocDo;
  khoangCach = robot.getDistance;
  getKetNoi  = robot.getKetNoi;
  
  if (manhinh == 1){
    
    trangThaiRobot();

    if (sentRadio){
      if (radio.available() || getKetNoi == 1){
        in_lcd(5, 0, "Connected ");
        ketNoi = 1;
      } else {
        in_lcd(5, 0, "Disconnect");
        ketNoi = 0;
      }
    }
  }
  

/*
 * ====== CHƯƠNG TRÌNH HOẠT ĐỘNG CỦA BLUETOOTH =========
 * Nếu phone kết nối thì sẽ gửi tín hiệu sau mỗi 300 ms.
 * Gửi một đoạn chuỗi gồm dữ liệu cảm biến và location của robot.
 */

  if (Bluetooth.available()){
    ketnoi = Bluetooth.readString();
//    delay(500);
  }
  
  delay_bluetooth++;
  if (delay_bluetooth == 300 || ketnoi == "<PhoneOK>") {
    String chuoi = 
          String(LPG)
          +"!"+ String(CH4)
          +"@"+ String(H2)
          +"#"+ String(CO)
          +"%"+ String(CO*0.8,2)
          +"&"+ String(latMap,7)
          +"-"+ String(lngMap,7);
    Bluetooth.println(chuoi);
    Serial.println(chuoi);
    delay_bluetooth = 0;
  }
  
  if (ketnoi == "<PhoneOK>") {
    bluetooth = true;
  } else {
    bluetooth = false;
  }
 
}


void in_lcd(int cot, int hang, String txt){
  lcd.setCursor(cot, hang);
  lcd.print(txt);
}
void chay_hai_dongco(){
  hopSo  = 0;
  motor  = 0;
  motor1 = false;
  motor2 = false;
  sent.setTWO = 1;
}
void chay_dongco_trai(){
  hopSo  = 0;
  all    = false;
  motor1 = true;
  motor2 = false;
  sent.setDC = 1;
}
void chay_dongco_phai(){
  hopSo  = 0;
  all    = false;
  motor1 = false;
  motor2 = true;
  sent.setDC = 2;
}
void guiDuLieu(){
  sent.setx1    = tx1;
  sent.sety1    = ty1;
  sent.setx2    = tx2;
  sent.sety2    = ty2;
  sent.setHopSo = hopSo;
  sent.setLedColor = turnLed;
  sent.setBlinkLed = blinkLed;
  
  radio.write(&sent, sizeof(Data_Package));
}
void trangThaiRobot(){
  switch(trangThai){
    case 0:
      in_lcd(6,  1, "  none  ");
      in_lcd(15, 1, "     ");
      break;
    case 1:
      in_lcd(6,  1, "Straight");
      in_lcd(15, 1, "     ");
      break;
    case 2:
      in_lcd(6,  1, "Straight");
      in_lcd(15, 1, "Left ");
      break;
    case 3:
      in_lcd(6,  1, "Straight");
      in_lcd(15, 1, "Right");
      break;
    case 4:
      in_lcd(6,  1, "  Back  ");
      in_lcd(15, 1, "     ");
      break;
    case 5:
      in_lcd(6,  1, "   On   ");
      in_lcd(15, 1, " Back");
      break;
    case 6:
      in_lcd(6,  1, "   On   ");
      in_lcd(15, 1, " Up  ");
      break;
    case 7:
      in_lcd(6,  1, "   On   ");
      in_lcd(15, 1, " Stop");
      break;
    case 8:
      in_lcd(6,  1, "   On   ");
      in_lcd(15, 1, " Back");
      break;
    case 9:
      in_lcd(6,  1, "   On   ");
      in_lcd(15, 1, " Up  ");
      break;
    case 10:
      in_lcd(6,  1, "   On   ");
      in_lcd(15, 1, " Stop");
      break;
    case 11:
      in_lcd(6,  1, "  Back  ");
      in_lcd(15, 1, " Left");
      break;
    case 12:
      in_lcd(6,  1, "  Back  ");
      in_lcd(15, 1, "Right");
      break;
  }
}
