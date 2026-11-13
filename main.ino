//библиотеки
#include <LiquidCrystal_I2C.h> //дисплей
#include <microDS18B20.h> //датчик температуры
#include <GyverPID.h> //регулятор
#include <VirtualButton.h>
#include <EEPROM.h>
#include <PWMrelay.h> //низкочастотный ШИМ

//пины
#define START_BTN 8
#define LEFT_BTN 7
#define MENU_BTN 5
#define RIGHT_BTN 6
#define COOLER 10
#define HEATER 11
//#define BUZZER 9

//таймауты кнопок
#define HOLD_TIMEOUT 2000
#define STEP_TIMEOUT 100

//период регулирования, фильтрации
#define DT 1000
#define FILT_INTERVAL 100

//константы для охлаждения
#define HEATER_POWER 500
#define COOLER_POWER 72.23
#define BOX_POWER 1.69
#define C_WATER 4200.0
#define WEIGHT_WATER 0.6

//константы для дисплея
#define ITEMS 3
#define DISPLAY_WIDTH 20
#define EEPROM_CHECK 1023
#define EEPROM_KEY 13

//создание экземпляров
LiquidCrystal_I2C lcd(0x27, 20, 4);
MicroDS18B20<A0> ds;
PWMrelay relay(HEATER);
GyverPID cooler_pid(18, 1, 10, DT);
GyverPID heater_pid(3, 0.08, 22, DT);
VButton start_btn;
VButton left_btn;
VButton menu_btn;
VButton right_btn;

//сохраняемые в EEPROM данные
struct {
  float initial_temp = 50, final_temp = 30;
  int32_t def_total_time = 86400;
} settings, def_settings;

enum {MENU, WARMING, COOLING} modes; //режимы работы

float temp, filt_temp, filt_k = 0.03;
int8_t menu_cursor;
uint32_t total_time = settings.def_total_time;
//bool enable_cooler = true;
uint64_t ds_timer, pid_timer, filt_timer, cool_timer, clock_timer; //error_timer; //таймеры

void setup() {
  //настройка пинов
  pinMode(COOLER, OUTPUT);
  //pinMode(HEATER, OUTPUT);
  //pinMode(BUZZER, OUTPUT);
  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(START_BTN, INPUT_PULLUP);
  pinMode(LEFT_BTN, INPUT_PULLUP);
  pinMode(MENU_BTN, INPUT_PULLUP);
  pinMode(RIGHT_BTN, INPUT_PULLUP);

  relay_off();

  //настройка регуляторов
  cooler_pid.setDirection(REVERSE);
  cooler_pid.setpoint = settings.initial_temp;

  heater_pid.setDirection(NORMAL);
  heater_pid.setpoint = settings.initial_temp;

  //инициализация порта
  Serial.begin(9600);
  Serial.setTimeout(50);
  Serial.flush();
  //Serial.println("input, pid_signal, setpoint");

  //инициализация дисплея
  lcd.init();
  lcd.backlight();

  //проверка наличия данных в EEPROM
  if (EEPROM.read(EEPROM_CHECK) != EEPROM_KEY) {
    EEPROM.write(EEPROM_CHECK, EEPROM_KEY);
    EEPROM.put(0, settings);
  }
  
  //получение данных из EEPROM
  EEPROM.get(0, settings);

  //установка таймаутов кнопок
  start_btn.setHoldTimeout(HOLD_TIMEOUT);
  right_btn.setStepTimeout(STEP_TIMEOUT);
  left_btn.setStepTimeout(STEP_TIMEOUT);

  //if (!ds.online()) Serial.println("error");
}

void loop() {
  switch (modes) {
    //меню
    case 0:
      analogWrite(COOLER, 0);
      relay_off();

      lcd.clear();

      while(1) {
        loop_actions();

        //опрос кнопок
        left_btn.poll(!digitalRead(LEFT_BTN));
        menu_btn.poll(!digitalRead(MENU_BTN));
        right_btn.poll(!digitalRead(RIGHT_BTN));

        menu(); //меню
        if (start_btn.hasClicks(3)) settings = def_settings; //сброс настроек

        //управление курсором
        if (menu_btn.click()) {
          menu_cursor++;
          menu_cursor %= ITEMS;
        }

        total_time = settings.def_total_time;
    
        cooler_pid.setpoint = settings.initial_temp;
        heater_pid.setpoint = settings.initial_temp;

        print_data("Initial T:", settings.initial_temp, 1, 1); //вывод начальной температуры на дисплей
        print_data("Final T:", settings.final_temp, 1, 2); //вывод конечной температуры на дисплей

        //вывод курсора на дисплей
        lcd.setCursor(0, (menu_cursor == 0) ? ITEMS : menu_cursor);
        lcd.print(" ");
        lcd.setCursor(0, menu_cursor + 1);
        lcd.write(126);

        if (start_btn.held()) break;
      }

      EEPROM.put(0, settings); //сохранение данных в EEPROM
      lcd.clear();

      modes = WARMING;      
      break;

    //нагрев
    case 1:
      while(1) {
        loop_actions();

        relay_on();

        if (filt_temp >= settings.initial_temp) {
          modes = COOLING;
          break;
        }
        //выход в меню
        else if (start_btn.held()) {
          modes = MENU;
          break;
        }
      }

      relay_off();

      //cooler_pid.setpoint = settings.final_temp;
      //heater_pid.setpoint = settings.final_temp;

      cool_timer = millis();
      //enable_cooler = true;
      break;
    
    //охлаждение
    case 2:
      loop_actions();

      timer(); //обратный отсчет
      cooling();

      if (total_time <= 0 || start_btn.held()) {
        modes = MENU;
        break;
      }
  }
}

void loop_actions() {
  //опрос реле и кнопки
  relay.tick();
  start_btn.poll(!digitalRead(START_BTN));

  read_temp(); //чтение температуры
  filt_temp = running_average(); //фильтрация температуры

  //указание входных данных для регуляторов
  cooler_pid.input = filt_temp;
  heater_pid.input = filt_temp;

  print_data("CURRENT T", filt_temp, 0, 0); //вывод текущей температуры на дисплей
  print_data("Time: ", time_output(), 1, 3); //вывод времени на дисплей

  if (modes != MENU) {
    print_data("Target T:", cooler_pid.setpoint, 1, 2); //вывод целевой температуры на дисплей

    //парсер
    if (Serial.available() > 1) {
      char key = Serial.read();
      float val = Serial.parseFloat();

      switch (key) {
        case 's':
          cooler_pid.setpoint = val;
          heater_pid.setpoint = val;
          break;
        case 'p': cooler_pid.Kp = val; break;
        case 'i': cooler_pid.Ki = val; break;
        case 'd': cooler_pid.Kd = val; break;
        case 't': cooler_pid.setDt(val); break;
        //case 'k': filt_k = val; break;
        //case 'f': filt_interval = int(val); break;
      }
    }

    //вывод графиков
    /*if (millis() - pid_timer >= DT) {
      pid_timer = millis();

      Serial.print(cooler_pid.input); Serial.print(",");
      Serial.print(cooler_pid.output); Serial.print(",");
      Serial.print(heater_pid.output); Serial.print(",");
      Serial.println(cooler_pid.setpoint);
    }*/
  }
}

//охлаждение
void cooling() {
  uint8_t delta_temp = abs(settings.initial_temp - settings.final_temp);
  uint16_t set_work = C_WATER * WEIGHT_WATER * delta_temp; //разность температур
  float box_cooling_work = BOX_POWER * settings.def_total_time; //на сколько градусов жидкость остынет при пассивном охлаждении
  bool using_cooler = (set_work > box_cooling_work); //использовать кулер или нагреватель
  float power = (using_cooler) ? COOLER_POWER : HEATER_POWER; //мощность кулера/нагревателя
  uint32_t real_work = power * settings.def_total_time; //время охлаждения при постоянно включенном кулере/нагревателе
  float real_time = set_work / power;
  /*float k = 0.05; //процент от заданного времени
  float all_on_time = float(settings.def_total_time * set_work) / float(real_work); //отношение времени работы к отдыху кулера/нагревателя

  float on_time = k * all_on_time * 1000; //время включения
  float off_time = k * (float(settings.def_total_time) * 1000) - on_time; //время выключения*/

  float iteration_time = (real_time + settings.def_total_time / real_time) * 1000;
  float iteration_temp = delta_temp * iteration_time / settings.def_total_time / 1000;
  static float current_setpoint = settings.initial_temp - iteration_temp;

  /*float on_time, off_time;

  if (use_cooler) {
    on_time = k * st_time * 1000;
    off_time = on_time * (freq_k - 1);
  }
  else {
    off_time = k * st_time * 1000;
    on_time = off_time * (freq_k - 1);
  }*/

  if (millis() - pid_timer >= DT) {
    pid_timer = millis();

    Serial.println("iteration_temp " + String(iteration_temp));
    Serial.println("iteration_time " + String(iteration_time));
    Serial.println("current_setpoint " + String(current_setpoint));
    Serial.println("def_total_time " + String(settings.def_total_time));
    //Serial.println("all_on_time " + String(all_on_time));
    Serial.println("set_work " + String(set_work));
    Serial.println("using_cooler " + String(using_cooler));
    Serial.println("box_cooling_work " + String(box_cooling_work));
    Serial.println("real_work " + String(real_work));
    Serial.println("real_time " + String(real_time));
    //Serial.println("on_time " + String(on_time));
    //Serial.println("off_time " + String(off_time));
    //Serial.println("enable_cooler " + String(enable_cooler));
    Serial.println("modes " + String(modes));
    Serial.println("filt_temp " + String(filt_temp));
    Serial.println("cooler_output " + String(cooler_pid.output));
    Serial.println("heater_pid_output " + String(heater_pid.output));
    Serial.println("setpoint " + String(settings.final_temp));
    //Serial.println("cool_timer " + String(cool_timer));
  }

  if (set_work != box_cooling_work) {
    cooler_pid.setpoint = current_setpoint;
    heater_pid.setpoint = current_setpoint;
    
    //get_pin(using_cooler); //включение/выключение
    if (using_cooler) {
      cooler_pid.getResultTimer();
      analogWrite(COOLER, cooler_pid.output);
    }
    else relay_on();

    //таймер времени включения
    if (millis() - cool_timer >= iteration_time && current_setpoint - iteration_temp >= settings.final_temp) {
      cool_timer = millis();
      current_setpoint -= iteration_temp;
    }
  }
}

//включение/выключение кулера/нагревателя
/*void get_pin(bool use_cooler) {
  if (use_cooler) {
    cooler_pid.getResultTimer();
    analogWrite(COOLER, cooler_pid.output);
    //if (!is_on) cooler_pid.output = 0;
  }
  else {
    relay_on();
    //if (!is_on) heater_pid.output = 0;
  }
}*/

//выключить реле
void relay_off() {
  relay.setLevel(LOW);
  relay.setPWM(0);
}

//включить реле
void relay_on() {
  //relay.setLevel(HIGH);
  heater_pid.getResultTimer();
  relay.setPWM(heater_pid.output);
}

//оповещение об ошибке/завершении
/*bool info(String txt) {
  lcd.clear();
  lcd.setCursor((DISPLAY_WIDTH - txt.length()) / 2, 1);
  lcd.print(txt);

  String note = "Press any button to continue";
  lcd.setCursor((DISPLAY_WIDTH - note.length()) / 2, 3);
  lcd.print(note);

  if (right_btn.click() || menu_btn.click() || left_btn.click() || start_btn.click()) {
    return true;
  }

  /*for (uint8_t i; i < 2; i++) {
    lcd.noBacklight();
    analogWrite(BUZZER, 100);
    delay(500);
    lcd.backlight();
    analogWrite(BUZZER, 0);
    delay(500);
  }
  static bool enable = true;

  //if (millis() - pid_timer >= DT) {
    pid_timer = millis();

    Serial.println("delta_time " + String(delta_time));
    Serial.println("error_timer " + String(error_timer));
    Serial.println("enable " + String(enable));
  //}

  if (enable) {
    lcd.noBacklight();
    analogWrite(BUZZER, 100);
  }
  else {
    lcd.backlight();
    analogWrite(BUZZER, 0);
  }

  if (millis() - error_timer >= time + delta_time && enable) {
    error_timer = millis();
    delta_time = 0;
    enable = false;
  }

  /*for (uint8_t i; i < 2; i++) {
    lcd.noBacklight();
    analogWrite(BUZZER, 100);
    
    if (millis() - error_timer >= 500 + delta_time) {
      error_timer = millis();
      delta_time = 0;
      
      lcd.backlight();
      analogWrite(BUZZER, 0);
    }
  }
}*/

//чтение температуры с датчика
void read_temp() {
  if (millis() - ds_timer >= DT) {
    ds_timer = millis();

    if (ds.readTemp()) {
      temp = ds.getTemp();
    }

    ds.requestTemp();
  }
}

//медианный фильтр
/*float median() {
  static float buf[3];
  static uint8_t count;

  buf[count] = temp;
  if (++count > 2) count = 0;

  float middle;

  if (buf[0] <= buf[1] && buf[0] <= buf[2]) {
    middle = (buf[1] <= buf[2]) ? buf[1] : buf[2];
  } else if (buf[1] <= buf[0] && buf[1] <= buf[2]) {
    middle = (buf[0] <= buf[2]) ? buf[0] : buf[2];
  } else if (buf[2] <= buf[0] && buf[2] <= buf[1]) {
    middle = (buf[0] <= buf[1]) ? buf[0] : buf[1];
  }

  return middle;
}*/

//бегущее среднее
float running_average() {
  static float average = temp;

  if (millis() - filt_timer >= FILT_INTERVAL) {
    filt_timer = millis();
    average += filt_k * (temp - average);
  }

  return average;
}

//фильтрация
/*float filtering() {
  float res;

  if (millis() - filt_timer >= FILT_INTERVAL) {
    filt_timer = millis();

    //filt_temp = median();
    res = running_average();
  }

  return res;
}*/

//изменение параметров в меню
void menu() {
  switch (menu_cursor) {
    //начальная температура
    case 0:
      if (right_btn.click() || right_btn.step()) {
        settings.initial_temp += 1 + (9 * !digitalRead(MENU_BTN));
      }
      else if (left_btn.click() || left_btn.step()) {
        settings.initial_temp -= 1 + (9 * !digitalRead(MENU_BTN));
      }
      break;

    //конечная температура
    case 1:
      if (right_btn.click() || right_btn.step()) {
        settings.final_temp += 1 + (9 * !digitalRead(MENU_BTN));
      }
      else if (left_btn.click() || left_btn.step()) {
        settings.final_temp -= 1 + (9 * !digitalRead(MENU_BTN));
      }
      break;
    
    //время
    case 2:
      if (right_btn.click() || right_btn.step()) {
        settings.def_total_time += 60 + (3540 * !digitalRead(MENU_BTN));
      }
      else if (left_btn.click() || left_btn.step()) {
        settings.def_total_time = constrain(settings.def_total_time - 60 - (3540 * !digitalRead(MENU_BTN)), 0, pow(2, 31) - 1);
      }
      break;
  }
}

//обратный остчет
void timer() {
  if (millis() - clock_timer >= 1000) {
    clock_timer = millis();
    total_time--;
  }
}

//вывод времени
String time_output() {
  uint32_t local_total_time = total_time, num;
  uint16_t divider;
  String res, num_str;

  for (uint8_t i = 0; i < 3; i++) {
    divider = 3600 / pow(60, i);
    num = local_total_time / divider;
    local_total_time -= num * divider;

    num_str = String(num);
    if (num < 10) num_str = "0" + num_str;

    res += num_str;
    if (i < 2) res += ":";
  }

  return res;
}

//печатать температуру на дисплей
void print_data(String txt, float data, uint8_t txt_x, uint8_t y) {
  String str_data = String(data);
  uint8_t data_x = DISPLAY_WIDTH - str_data.length() - txt_x - 1;

  lcd.setCursor(txt_x, y);
  lcd.print(txt);

  /*uint8_t space_count = (data == 0) ? 2 : 1;

  lcd.setCursor(data_x - space_count, y);
  lcd.print(String(space_count, " "));*/

  if (data == 0) {
    lcd.setCursor(data_x - 2, y);
    lcd.print("  ");
  } else {
    lcd.setCursor(data_x - 1, y);
    lcd.print(" ");
  }
  
  lcd.setCursor(data_x, y);
  lcd.print(str_data);
  lcd.write(223);
}

//печатать время на дисплей
void print_data(String txt, String data, uint8_t txt_x, uint8_t y) {
  uint8_t data_x = DISPLAY_WIDTH - data.length() - txt_x - 1;

  lcd.setCursor(txt_x, y);
  lcd.print(txt);
  
  lcd.setCursor(data_x - 1, y);
  lcd.print(" ");
  
  lcd.setCursor(data_x, y);
  lcd.print(data);
}