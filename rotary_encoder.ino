
#include <avr/wdt.h>
#include <Encoder.h>
#include <LiquidCrystal_I2C.h>

#define BUTTONPIN 7
#define SSRPIN 11

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

uint8_t uarrow[8]  = {B00100, 
                      B01010, 
                      B10001, 
                      B00100, 
                      B00100, 
                      B00100, 
                      B00100, 
                      B00000};
uint8_t darrow[8]  = {B00100, 
                      B00100, 
                      B00100, 
                      B00100, 
                      B00100, 
                      B10001, 
                      B01010, 
                      B00100};
                      
uint8_t rarrow[8]  = {B00000, 
                      B01000, 
                      B00100, 
                      B00010, 
                      B00001, 
                      B10010, 
                      B00100, 
                      B01000};
                      
uint8_t larrow[8]  = {B00100, 
                      B00100, 
                      B00100, 
                      B00100, 
                      B00100, 
                      B10001, 
                      B01010, 
                      B00100};
/*uint8_t ldarrow[8]  = {B0,B0,B1,B10010,B10100,B11000,B11110};
uint8_t ruarrow[8] = {B01111,B00011,B00101,B01001,B10000,0,0};
uint8_t rdarrow[8] = {0,0,B10000,B01001,B00101,B00011,B01111};*/
uint8_t check[8] = {0x0,0x1,0x3,0x16,0x1c,0x8,0x0};
uint8_t degc[8] = {B01000, B10100, B01000, 0, B00011, B00100, B00100, B00011};

Encoder myEnc(2,3);
unsigned long lastTime;

#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))  // set bit
void setup_watchdog() 
{
  cli();
  MCUSR &= ~(1<<WDRF);
    
  WDTCSR |= (1<<WDCE) | (1<< WDE);
  WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);
  sei();
}

int sec_passed = 0;
ISR(WDT_vect) {
  //setup_watchdog();
  
  sec_passed++;
  //lcd.setCursor(0, 1);
  //lcd.print(sec_passed);
}


long oldPosition  = -999;

void setup() {
  turn_off();
  
  //Serial.begin(9600);
  //Serial.println("Basic Encoder Test:");

  
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, uarrow);
  lcd.createChar(1, darrow);
  
  lcd.clear();
  
  pinMode(BUTTONPIN, INPUT);
  pinMode(SSRPIN, OUTPUT);
  digitalWrite(SSRPIN, LOW);
  
  oldPosition = myEnc.read();
  
  
  
  sei();
  //setup_watchdog();
}


enum MENUITEMS {ONTIME, OFFTIME, RUN};
enum MENUSTATES {M_ONTIME, M_OFFTIME, M_MENU};
enum POWERSTATES {P_ON, P_OFF};
uint8_t power_state = P_OFF;
uint8_t state = M_MENU;
uint8_t menu_item = ONTIME;

uint8_t set_ontime = 1;
uint8_t set_offtime = 1;
bool buttonDown = false;

void turn_on() {
  power_state = P_ON;
  digitalWrite(SSRPIN, HIGH);
  lastTime = millis();
}
void turn_off() {
  power_state = P_OFF;
  digitalWrite(SSRPIN, LOW);
  lastTime = millis();
}
void loop() {

  lcd.setCursor(6, 0);
  unsigned long elapsed = (millis() - lastTime)/1000;
  long remaining;
  switch(power_state) {
    case P_ON:
      remaining = set_ontime*60 - elapsed;
      lcd.print("[ON] ");
      if ( remaining <= 0 ) {
        turn_off();
      }
      break;
    case P_OFF:
      remaining = set_offtime*60 - elapsed;
      lcd.print("[OFF]");
      if ( remaining <= 0 ) {
        turn_on();
      }
      break;
  }

  remaining = max(0, remaining);
  lcd.setCursor(8, 1);
  lcd.print(remaining/60);
  lcd.print(":");
  lcd.print(remaining%60);
  lcd.print("  ");
  
  long increment;
  long newPosition = myEnc.read();
  //Serial.println(newPosition);
  increment = -(newPosition - oldPosition)/2;
  if (increment != 0)
    oldPosition = newPosition;

  // process button
  if (digitalRead(BUTTONPIN) == LOW && buttonDown == false) {
    buttonDown = true;
    
    switch (state) {
      case M_MENU:
        switch(menu_item) {
          case ONTIME: state = M_ONTIME; break; 
          case OFFTIME: state = M_OFFTIME; break;
          case RUN: turn_on();//turn on stuff
          lastTime = millis();
          break;
        }
        return;
        break;
        
      case M_ONTIME: // set time
        state = M_MENU;
        return;
        break;
      case M_OFFTIME:
        state = M_MENU;
        return;
        break;
      default :
      break;
    }
  }
  if (digitalRead(BUTTONPIN) == HIGH && buttonDown == true) {
    buttonDown = false; }
  
  
  switch (state) {
    case M_MENU:
      menu_item = (menu_item + increment + 3) % 3;

      lcd.setCursor(0,0); lcd.print("[");
      switch (menu_item) {
        case ONTIME: lcd.print(" T"); lcd.write(0); break;
        case OFFTIME: lcd.print(" T"); lcd.write(1); break;
        case RUN: lcd.print("Run"); break;
      }
      lcd.print("]");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.print(set_ontime);
      lcd.print("/");
      lcd.print(set_offtime);
      lcd.print("  ");
      
      break;
    case M_ONTIME:
      set_ontime = max(1, set_ontime + increment);
      set_ontime = min(99, set_ontime);
      lcd.setCursor(0, 1);
      lcd.print(">");
      lcd.print(set_ontime);
      lcd.print("/");
      lcd.print(set_offtime);
      lcd.print("  ");
      break;
    case M_OFFTIME:
      set_offtime = max(1, set_offtime + increment);
      set_offtime = min(99, set_offtime);
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.print(set_ontime);
      lcd.print("/");
      lcd.print(set_offtime);
      lcd.print("< ");
      break;
    default :
    break;
  }


  
  //delay(200);
}
