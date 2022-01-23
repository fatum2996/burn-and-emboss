#include <TimerOne.h> //https://www.pjrc.com/teensy/td_libs_TimerOne.html
#include <Bounce2.h> //библиотека обработки кнопок https://github.com/thomasfredericks/Bounce2
#include <GyverTimer.h> //https://alexgyver.ru/gyvertimer/
#include <SPI.h>
#include <Wire.h>
#include "functions.h"
#include <U8g2lib.h> //https://github.com/olikraus/u8g2
#include <PWM.h> // https://github.com/RCS101/PWM
 
systemData_t systemData;
uint16_t reading ;
uint8_t timerBusy = 0;
GTimer pneumoTimer(MS);
GTimer maxTimer(MS, 500);
GTimer buttonTimer(MS);
GTimer smallButtonTimer(MS);
uint8_t buttonLPlusDown = 0;
uint8_t buttonLMinusDown = 0;
uint8_t buttonTPlusDown = 0;
uint8_t buttonTMinusDown = 0;
uint8_t buttonTMinusDown1 = 0;
uint8_t buttonTPlusDown1 = 0;
uint8_t changeScreenState = 0;
uint32_t timeLaserWork = 0;
uint32_t timeLaserPeriod = 0;
uint32_t timeLaserStop = 0;
uint8_t toWork = 0;
uint8_t errorState = 0;
uint8_t screenState = 0; //по умолчанию

void callback(){ //обаботка пунктира
  if(toWork){ //период включения ШИМ
    toWork=0;
    Timer1.setPeriod(timeLaserWork*1000);  
    pwmWrite(PIN_PWM, char(systemData.laserBeamDutyPercent * 255 / 100));
  } else{ //период выключения ШИМ
    toWork=1;
    Timer1.setPeriod(timeLaserStop*1000);         // инициализировать timer1, и установить период 
    pwmWrite(PIN_PWM, 0);    
  }
}
 
void setup() {  
  InitTimersSafe();
  pinConfig(); //настройка выводов
  display.begin(); //включение экрана
  showSplashscreen();  //заставка экрана
  SetPinFrequency(PIN_PWM, pwmFrequency); //назначаем частоту ШИМ
  //TCCR2B = TCCR2B & B11111000 | B00000100;  //настройка ШИМ, частота 490 Гц
  memset((uint8_t*) &systemData, 0x00, sizeof(systemData));
  systemData.laserBeamDutyPercent = laserBeamDutyDefault;
  systemData.patternPresetNo = 0x00;
  systemData.patternFrequency = pwmPatternPreset[systemData.patternPresetNo].frequency;
  systemData.patternDutyPercent = pwmPatternPreset[systemData.patternPresetNo].dutyPercent;
  systemData.heaterTemperature = 0x00;
  systemData.heaterState = HEATER_STATE_OFF;
  systemData.heaterTemperatureState = HEATER_TEMPERATURE_LOW;
  systemData.heaterTargetTemperature = heaterTargetTemperature;  
  maxTimer.start(); //таймер опроса датчика температуры
  delay(500); //задержка для включения датчика
  interrupts(); //разрешение прерываний
}


void loop() {
  //чтение температуры
  if(maxTimer.isReady()) {
    reading = readMAX();
    if((reading & 0x8000)) //общая ошибка
      errorState = HEATER_STATE_ERROR_UNKNOWN;
    else {
      if(reading & 0x04)  //отсутствие термопары
        errorState = HEATER_STATE_ERROR_CONNECT_TERMOCOUPLE;  
      else {
        if(reading & 0x02)  //неверный ID микросхемы
          errorState = HEATER_STATE_ERROR_CONNECT_SENSOR;  
        else
          systemData.heaterTemperature = reading >> 5;      
      }    
    }       
    if(errorState) {
      systemData.heaterState = errorState;       
      digitalWrite(PIN_HEATER, LOW);
    }
    else {
      if(screenState){
        if (systemData.heaterTemperature <= systemData.heaterTargetTemperature - lowerDelta) { //если температура меньше чем уставка минус допуск
          systemData.heaterTemperatureState = HEATER_TEMPERATURE_LOW;//включить нагреватель
          digitalWrite(PIN_HEATER, HIGH);
          systemData.heaterState = HEATER_STATE_ON;
        } else if (systemData.heaterTemperature >= systemData.heaterTargetTemperature + higherDelta){ //если температура больше чем уставка плюс допуск
          systemData.heaterTemperatureState = HEATER_TEMPERATURE_HIGH;//выключить нагреватель
          digitalWrite(PIN_HEATER, LOW);
          systemData.heaterState = HEATER_STATE_OFF;
        } else {//если температура в диапазоне
          systemData.heaterTemperatureState = HEATER_TEMPERATURE_IN_RANGE;    //выключить нагреватель    
        }
      }
    }
  }

  if(systemData.patternDutyPercent == 100) //если режим не пунктир
    pwmWrite(PIN_PWM, char(systemData.laserBeamDutyPercent * 255 / 100)); //выводим ШИМ без прерываний

  if((digitalRead(PIN_24V)==LOW)&&(timerBusy == 0)) { //если пришел сигнал 24 вольта
    pneumoTimer.setTimeout(pneumaticReactionDelay);   //запускаем таймер задержки
    timerBusy = 1;
  }
  
  if(digitalRead(PIN_24V)){ //если сигнала нет
    digitalWrite(PIN_PNEUMO, LOW);   //отключаем пневмо
    pneumoTimer.stop();   
    timerBusy=0;
  }
  
  if(pneumoTimer.isReady()) { //задержка закончилась
    digitalWrite(PIN_PNEUMO, HIGH);  //включаем пневмо
  }
  
//обработка кнопки L+
  if(buttonLPlus.update()) {
    if(buttonLPlus.read() == 0) {//кнопка нажата
      if(buttonTimer.isEnabled() == 0) {
        buttonTimer.setTimeout(buttonHoldDelay);   //запускаем таймер удержания
        buttonLPlusDown = 1; //фиксируем, что кнопка была нажата
      }
    } else { //кнопка отпущена
      buttonTimer.stop(); //останавливаем таймер нажатия и сбрасываем флаг нажатия 
      smallButtonTimer.stop();
      systemData.laserBeamDutyPercent += laserBeamDutyStep; //увеличиваем бим пауэр
      if(systemData.laserBeamDutyPercent > 100) {
        systemData.laserBeamDutyPercent = 100;
      }
      buttonLPlusDown = 0;
    }
  }  
  if(buttonLPlusDown) { //
    if(buttonTimer.isReady()){
      if( smallButtonTimer.isEnabled() == 0 ) {//запускаем короткий таймер, каждое срабатывание этого таймера при удержании кнопки - приращение значения
        smallButtonTimer.setInterval(smallButtonTimerDelay);      
      }
    }
    if(smallButtonTimer.isReady()){
      systemData.laserBeamDutyPercent += laserBeamDutyStep; //приращение значения при удержании кнопки
      if(systemData.laserBeamDutyPercent > 100) {
        systemData.laserBeamDutyPercent = 100;
      }  
    }
  }
//обработка кнопки L-
  if(buttonLMinus.update()) {
    if(buttonLMinus.read() == 0) {//кнопка нажата
      if(buttonTimer.isEnabled() == 0) {
        buttonTimer.setTimeout(buttonHoldDelay);   
        buttonLMinusDown = 1;
      }
    } else { //кнопка отпущена
      buttonTimer.stop();
      smallButtonTimer.stop();
      systemData.laserBeamDutyPercent -= laserBeamDutyStep; 
      if(systemData.laserBeamDutyPercent < 1) {
        systemData.laserBeamDutyPercent = 1;
      }
      buttonLMinusDown = 0;
    }
  }
  if(buttonLMinusDown) {
    if(buttonTimer.isReady()){
      if( smallButtonTimer.isEnabled() == 0 ) {
        smallButtonTimer.setInterval(smallButtonTimerDelay);      
      }
    }
    if(smallButtonTimer.isReady()){
      systemData.laserBeamDutyPercent -= laserBeamDutyStep; 
      if(systemData.laserBeamDutyPercent < 1) {
        systemData.laserBeamDutyPercent = 1;
      }  
    }
  }    
//обработка кнопки L Reset
  if(buttonLReset.update()) {
    if(buttonLReset.read() == 0) {//кнопка нажата
    } else { //кнопка отпущена
      systemData.laserBeamDutyPercent = laserBeamDutyDefault; 
      systemData.patternPresetNo = 0; //меняем пресет
      systemData.patternFrequency = pwmPatternPreset[systemData.patternPresetNo].frequency;
      systemData.patternDutyPercent = pwmPatternPreset[systemData.patternPresetNo].dutyPercent;
    }   
  }   
//обработка кнопки T-
  if(buttonTMinus.update()) {
    if(buttonTMinus.read() == 0) {//кнопка нажата
        buttonTMinusDown1 = 1;
      if(buttonTimer.isEnabled() == 0) {
        buttonTimer.setTimeout(buttonHoldDelay);   
        buttonTMinusDown = 1;
      }      
    } else { //кнопка отпущена
      buttonTMinusDown1 = 0; 
      buttonTimer.stop();
      smallButtonTimer.stop();
      systemData.heaterTargetTemperature -= heaterTargetTemperatureStep; 
      if(systemData.heaterTargetTemperature < heaterTargetTemperatureMin) {
        systemData.heaterTargetTemperature = heaterTargetTemperatureMin;
      }
      buttonTMinusDown = 0;
    }
  }
 
  if(buttonTMinusDown) {
    if(buttonTimer.isReady()){
      if( smallButtonTimer.isEnabled() == 0 ) {
        smallButtonTimer.setInterval(smallButtonTimerDelay);      
      }
    }
    if(smallButtonTimer.isReady()){
      systemData.heaterTargetTemperature -= heaterTargetTemperatureStep; 
      if(systemData.heaterTargetTemperature < heaterTargetTemperatureMin) {
        systemData.heaterTargetTemperature = heaterTargetTemperatureMin;
      }  
    }
  }   
//обработка кнопки T+
  if(buttonTPlus.update()) {
    if(buttonTPlus.read() == 0) {//кнопка нажата
      buttonTPlusDown1 = 1;
      if(buttonTimer.isEnabled() == 0) {
        buttonTimer.setTimeout(buttonHoldDelay);   
        buttonTPlusDown = 1;
      }
    } else { //кнопка отпущена
      buttonTimer.stop();
      smallButtonTimer.stop();
      buttonTPlusDown1 = 0;
      systemData.heaterTargetTemperature += heaterTargetTemperatureStep; 
      if(systemData.heaterTargetTemperature > heaterTargetTemperatureMax) {
        systemData.heaterTargetTemperature = heaterTargetTemperatureMax;
      }
      buttonTPlusDown = 0;
    }
  }
  if(buttonTPlusDown) {
    if(buttonTimer.isReady()){
      if( smallButtonTimer.isEnabled() == 0 ) {
        smallButtonTimer.setInterval(smallButtonTimerDelay);      
      }
    }
    if(smallButtonTimer.isReady()){
      systemData.heaterTargetTemperature += heaterTargetTemperatureStep; 
      if(systemData.heaterTargetTemperature > heaterTargetTemperatureMax) {
        systemData.heaterTargetTemperature = heaterTargetTemperatureMax;
      }  
    }
  }   

  //обработка нажатия кнопки Pattern
  if(buttonPattern.update()) {
    if(buttonPattern.read() == 0) {//кнопка нажата
    } else { //кнопка отпущена
      systemData.patternPresetNo = (systemData.patternPresetNo + 1) % arraySize(pwmPatternPreset); //меняем пресет
      systemData.patternFrequency = pwmPatternPreset[systemData.patternPresetNo].frequency;
      systemData.patternDutyPercent = pwmPatternPreset[systemData.patternPresetNo].dutyPercent;
      if(systemData.patternDutyPercent != 100) { //если требуется пунктир
        timeLaserPeriod = 1000 / systemData.patternFrequency ; //период работы лазера пунктиром в мс
        timeLaserWork = timeLaserPeriod * systemData.patternDutyPercent / 100;//работа пунктиром в мс
        timeLaserStop = timeLaserPeriod - timeLaserWork; //простой пунктиром в мс
        Timer1.initialize(timeLaserWork*1000); // инициализировать timer1, и установить период в мкс
        Timer1.attachInterrupt(callback);  // прикрепить callback(), как обработчик прерывания по переполнению таймера
      } else {
        Timer1.detachInterrupt();
        Timer1.stop();
      }
    } 
  }    
  //обновить дисплей        

  if(buttonTPlusDown1 && buttonTMinusDown1)
        changeScreenState = 1;
  
  if(changeScreenState) {
    screenState = !screenState;
    changeScreenState = 0;
    buttonTPlusDown1 = 0;
    buttonTMinusDown1 = 0;
  }
  if(screenState) 
    displayShowData(systemData);  
  else {
    displayShowDataNoHeater(systemData);
    digitalWrite(PIN_HEATER, LOW);
  }
}
