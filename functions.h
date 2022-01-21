#include "config.h"

#define arraySize(_array) ( sizeof(_array) / sizeof(*(_array)) ) //размер массива

void showSplashscreen() { //вывод заставки на экран
  digitalWrite(displayCSPin, LOW);
  display.firstPage();
  do {
    display.setColorIndex(1);
    display.setFont(u8g2_font_profont22_tf);
    //display.drawRFrame(u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h, u8g_uint_t r);
    display.drawRFrame(0, 0, 128, 64, 3);
    display.drawStr( 42, 18, "burn");
    display.drawStr( 46, 34, "and");
    display.drawStr( 30, 50, "emboss");
    //drawRBox(u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h, u8g_uint_t r)
    display.drawBox(0, 53, 128, 11);
    display.setFont(u8g2_font_profont11_tf );
    display.setColorIndex(0);
    display.drawStr( 33, 62, "version 1.0");
  } while ( display.nextPage() );
  digitalWrite(displayCSPin, HIGH);
}
 void pinConfig() { //конфигурация выводов
  pinMode(PIN_24V, INPUT);
  pinMode(PIN_PWM, OUTPUT);
  digitalWrite(PIN_PWM, LOW);
  pinMode(PIN_PNEUMO, OUTPUT);
  digitalWrite(PIN_PNEUMO, LOW);
  pinMode(PIN_HEATER, OUTPUT);
  digitalWrite(PIN_HEATER, LOW);
  pinMode(PIN_L_MINUS, INPUT);
  pinMode(PIN_L_PLUS, INPUT);
  pinMode(PIN_L_RESET, INPUT);
  pinMode(PIN_T_MINUS, INPUT);
  pinMode(PIN_T_PLUS, INPUT);
  pinMode(PIN_PATTERN, INPUT);

  pinMode(pin_CS_max, OUTPUT);
  pinMode(pin_CLK_max, OUTPUT); 
  pinMode(pin_SO_max, INPUT);
  digitalWrite(pin_SO_max, HIGH);
  
  pinMode(A6, OUTPUT);
  digitalWrite(pin_CS_max, HIGH);
  buttonLPlus.attach(PIN_L_PLUS); // устанавливаем кнопку
  buttonLPlus.interval(debounceInterval);
  buttonLMinus.attach(PIN_L_MINUS); // устанавливаем кнопку
  buttonLMinus.interval(debounceInterval);
  buttonTMinus.attach(PIN_T_MINUS); // устанавливаем кнопку
  buttonTMinus.interval(debounceInterval);
  buttonTPlus.attach(PIN_T_PLUS); // устанавливаем кнопку
  buttonTPlus.interval(debounceInterval);
  buttonLReset.attach(PIN_L_RESET); // устанавливаем кнопку
  buttonLReset.interval(debounceInterval);
  buttonPattern.attach(PIN_PATTERN); // устанавливаем кнопку
  buttonPattern.interval(debounceInterval);
 }

void displayClear() {
  digitalWrite(displayCSPin, LOW);
  display.firstPage(); 
  do {
    display.setColorIndex(1);
    display.drawBox(0, 0, 128, 64);   
  } while ( display.nextPage() );
  digitalWrite(displayCSPin, HIGH);
}
 void displayShowData(systemData_t& _systemData) { //вывод информации на дисплей
  uint8_t xPosition = 0;
  uint8_t i = 0;
  digitalWrite(displayCSPin, LOW);
  display.firstPage();
  do {
    display.setColorIndex(1);
    display.drawFrame(0, 0, 128, 39);    
  
    if(_systemData.laserBeamDutyPercent == 100)
      xPosition = 5;
    else {
      if(_systemData.laserBeamDutyPercent >= 10)
        xPosition = 11;
      else{
        xPosition = 18;    
      }
    }
    display.setFont(u8g2_font_profont22_tf);      
    display.setCursor(xPosition,24);
    display.print("POWER ");
    display.print(_systemData.laserBeamDutyPercent);
    display.print("%");
    if(_systemData.patternPresetNo == 0)
      display.drawBox(0,30,128,4);
    if(_systemData.patternPresetNo == 1)
      for(i=0; i < 4; i++)
        display.drawBox(i*36,30,20,4);  
    if(_systemData.patternPresetNo == 2)
      for(i=0; i < 11; i++)
        display.drawBox(1+i*12,30,6,4);          
    display.drawFrame(0, 38, 128, 26);
    display.setCursor(80, 50);           
    if((_systemData.heaterState == HEATER_STATE_ERROR_UNKNOWN) or (_systemData.heaterState == HEATER_STATE_ERROR_CONNECT_SENSOR)or (_systemData.heaterState == HEATER_STATE_ERROR_CONNECT_TERMOCOUPLE)) {                    
      display.drawBox(0, 38, 128, 26);
      display.setColorIndex(0);
      display.setCursor(16, 56);//левый нижний угол сообщения об ошибке
      display.setFont(u8g2_font_profont17_tf);  //размер шрифта сообщения об ошибке, цифру можно заменить на 10,11,12,15,17,22,29
      if(_systemData.heaterState == HEATER_STATE_ERROR_UNKNOWN) {
        display.print("ERROR 1 OFF");
      }
      if(_systemData.heaterState == HEATER_STATE_ERROR_CONNECT_SENSOR) {
        display.print("ERROR 2 OFF");
      }
      if(_systemData.heaterState == HEATER_STATE_ERROR_CONNECT_TERMOCOUPLE) {
        display.print("ERROR 3 OFF");
      }      
    } else {
      display.setFont(u8g2_font_profont12_tf); 
      display.setColorIndex(1);
      switch (_systemData.heaterTemperatureState) {
        case HEATER_TEMPERATURE_IN_RANGE: {
          display.drawBox(0, 38, 128, 26);
          display.setColorIndex(0);
          display.print("OK");
          break;
        }
        case HEATER_TEMPERATURE_HIGH: {
          display.print("HI");
          break;
        }
        case HEATER_TEMPERATURE_LOW: {
          display.print("LO");
          break;
        }
      } 
      display.setCursor(3, 50);
      display.print("Temperature ");
      display.setCursor(108, 50);
      switch (_systemData.heaterState) {
        case HEATER_STATE_ON: {
          display.print("ON");
          break;
        }
        case HEATER_STATE_OFF: {
          display.print("OFF");
          break;
        }
        default: {
          display.print("UNK");
          break;
        }
      }
      display.setCursor(30, 61);
      display.print(_systemData.heaterTemperature);
      display.print("C => ");
      display.print(_systemData.heaterTargetTemperature);
      display.print("C");
    }
  } while ( display.nextPage() );
  digitalWrite(displayCSPin, HIGH);
}


 void displayShowDataNoHeater(systemData_t& _systemData) { //вывод информации на дисплей
  uint8_t xPosition = 0;
  uint8_t i = 0;
  digitalWrite(displayCSPin, LOW);
  display.firstPage();
  do {
    display.setColorIndex(1);
    display.drawFrame(0, 0, 128, 64);
    display.setFont(u8g2_font_profont22_tf);
    if(_systemData.laserBeamDutyPercent == 100)
      xPosition = 5;
    else {
      if(_systemData.laserBeamDutyPercent >= 10)
        xPosition = 11;
      else{
        xPosition = 18;    
      }
    }
    display.setCursor(xPosition,32);
    display.print("POWER ");
    display.print(_systemData.laserBeamDutyPercent);
    display.print("%");
    if(_systemData.patternPresetNo == 0)
      display.drawBox(0,46,128,4);
    if(_systemData.patternPresetNo == 1)
      for(i=0; i < 4; i++)
        display.drawBox(i*36,46,20,4);  
    if(_systemData.patternPresetNo == 2)
      for(i=0; i < 11; i++)
        display.drawBox(1+i*12,46,6,4);  
  } while ( display.nextPage() );
  digitalWrite(displayCSPin, HIGH);
}

byte spiread(void) { //чтение с датчика 1 байта
  int i;
  byte d = 0;

  for (i=7; i>=0; i--)
  {
    digitalWrite(pin_CLK_max, LOW);
    _delay_ms(1);
    if (digitalRead(pin_SO_max)) {
      //set the bit to 0 no matter what
      d |= (1 << i);
    }
    digitalWrite(pin_CLK_max, HIGH);
    _delay_ms(1);
  }
  return d;
}

uint16_t readMAX(void) { //чтение с датчика всей посылки
  uint16_t v=0;  
  digitalWrite(pin_CS_max, LOW);
  _delay_ms(1);
  v = spiread();
  v <<= 8;
  v |= spiread();
  digitalWrite(pin_CS_max, HIGH);
  return v;
}
