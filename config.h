#include <U8g2lib.h>

const uint8_t  displayCSPin = 10; // вывод Arduino к которому подключен вход CS дисплея
const uint8_t  displayDCPin = 8; // вывод Arduino к которому подключен вход DC дисплея

U8G2_SH1106_128X64_NONAME_2_4W_HW_SPI display(U8G2_R0, displayCSPin, displayDCPin);//объект дисплея

#define PIN_24V 2 //вывод на который приходит сигнал 24В
#define PIN_PWM 3 //вывод ШИМ
#define PIN_PNEUMO 5 //вывод пневмо
#define PIN_HEATER 4 //вывод управления нагревателем

#define PIN_L_MINUS A0 //выводы, отвечающие за кнопки
#define PIN_L_PLUS A1
#define PIN_L_RESET A2
#define PIN_T_MINUS A3
#define PIN_T_PLUS A4
#define PIN_PATTERN A5



Bounce buttonLMinus = Bounce(); //объекты кнопок
Bounce buttonLPlus = Bounce();
Bounce buttonLReset = Bounce();
Bounce buttonTMinus = Bounce();
Bounce buttonTPlus = Bounce();
Bounce buttonPattern = Bounce();

const uint16_t buttonHoldDelay = 750; // задержка до начала режима удержания кнопки, мс
const uint16_t smallButtonTimerDelay = 100; //период увеличения значения при удержании кнопки
const uint8_t debounceInterval = 4;// интервал подавления дребезга

uint8_t pin_SO_max=9; //пины датчика температуры 
uint8_t pin_CS_max=6;
uint8_t pin_CLK_max=7;

const uint32_t pneumaticReactionDelay = 1500; // Задержка срабатывания пневмо

typedef struct {
  uint16_t laserBeamDutyPercent;
  uint16_t patternFrequency;
  uint16_t patternDutyPercent;
  uint8_t  patternPresetNo;
  uint16_t heaterTargetTemperature;
  uint16_t heaterTemperature;
  uint8_t  heaterTemperatureState;
  int8_t   heaterState;
} systemData_t; //структура состояния системы

typedef struct {
  uint16_t frequency;
  uint16_t dutyPercent;
} pwmPattern_t;

uint16_t pwmFrequency = 490; // частота ШИМ в Гц

const pwmPattern_t pwmPatternPreset[] = { //Пресеты
  100, 100,  // Pattern #1
  30, 40,  // Pattern #2
  60, 40,  // Pattern #3
};


const uint16_t heaterTargetTemperature = 20; // Уставка температуры по-умолчанию, градусы Цельсия (С)
const uint16_t heaterTargetTemperatureMax = 200; // Максимальная температура 
const uint16_t heaterTargetTemperatureMin = 20; //Минимальная температура
const uint16_t heaterTargetTemperatureStep = 5;//Шаг изменения температуры
const uint16_t lowerDelta = 1;//Нижний "гистерезис", при температуре менее уставка минус это значение включится нагрев
const uint16_t higherDelta = 1;//Верхний "гистерезис", при температуре более уставка минус это значение выключится нагрев
 
const uint16_t laserBeamDutyStep = 1; //шаг изменения бим пауэр
const uint16_t laserBeamDutyDefault = 1; //Значение бим пауэр по умолчанию



//ошибки и маски при чтении с датчика
#define MAX6675_READING_ERROR                                   (-0x01)
#define MAX6675_BITMASK_ID                                      (0x02)
#define MAX6675_BITMASK_TERMOCOUPLE_INPUT                       (0x04)
#define MAX6675_GENERAL_ERROR                                   (-0x01)
#define MAX6675_WRONG_ID                                        (-0x02)
#define MAX6675_TERMOCOUPLE_PROBLEM                             (-0x03)

#define HEATER_STATE_ERROR_UNKNOWN                              (-0x01)
#define HEATER_STATE_ERROR_CONNECT_SENSOR                       (-0x02)
#define HEATER_STATE_ERROR_CONNECT_TERMOCOUPLE                  (-0x03)

//#define HEATER_STATE_HEATS_UP                                   (0x02)
//#define HEATER_STATE_COOLS_DOWN                                 (0x03)
#define HEATER_STATE_ON                                         (0x02)
#define HEATER_STATE_OFF                                        (0x03)
#define HEATER_STATE_IDLE                                        (0x00)
 
#define HEATER_TEMPERATURE_IN_RANGE                             (0x01)
#define HEATER_TEMPERATURE_HIGH                                 (0x02)
#define HEATER_TEMPERATURE_LOW                                  (0x03)
#define HEATER_TEMPERATURE_ON_SENSOR_ERROR                      (0x00)
