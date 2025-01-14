#include <Arduino.h>
#include <math.h>

#include <Wire.h>                     // Подключаем библиотеку Wire
#include <Adafruit_PWMServoDriver.h>  // Подключаем библиотеку Adafruit_PWMServoDriver

#include <stdio.h>

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
bool input_open = false;

// "Переменные основной программы"
float prevErrR = 0;
float prevErrL = 0;
float kp = 2.5*0.75;
float ki = kp*0.0012;
float kd = 0.2/8;
const short filterD = 3;
//Для машины состояний:
char stateR = 0;  // предыдущий номер состояния правого двигателя
char stateL = 0;  // предыдущий номер состояния левого двигателя

long int posR = 0;      // подсчитанное число меток
long int posTempR = 0;  // подсчитанное число меток на прошлом шаге
long int posL = 0;      // подсчитанное число меток
long int posTempL = 0;  // подсчитанное число меток на прошлом шаге
long int globalpos = 0;
char globalstate = 0;
const unsigned int timer_timeout = 40;              // период работы тайиера контроля скорости
const unsigned int im_timer_timeout = 5000;
unsigned int im_timer = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x60);  // Установка адреса I2C 0x60

double speedLeft = 0.0;
double speedRight = 0.0;
double speedLeft_encient = 0.0;
double speedRight_encient = 0.0;
double x_pos_ = 0.0;
double y_pos_ = 0.0;
double heading_ = 0.0;
double RealspeedRight = 0.0;
double RealspeedLeft = 0.0;
String input_m[2] = {"", ""};
String Odom_m_str = "";
double a = 11.5;  // расстояние между центрами колёс
double r = 3.2;   // радиус колёс
uint32_t tmr;
uint32_t tmr_pub_odom;

#define vmax 1000

#define pin1 32  // Пин для сигнала 1
#define pin2 33  // Пин для сигнала 2
#define pin3 34  // Пин для сигнала 3
#define pin4 35  // Пин для сигнала 4

#define LED_PIN 2
#define MOTOR_LEFT 0x00
#define MOTOR_RIGHT 0x02

// функция пид правый
float computePIDR(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  float err = setpoint - input;
  static float integralR = 0, prevErrR = 0;
  integralR = constrain(integralR + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErrR) / dt;
  prevErrR = err;
  return constrain(err * kp + integralR + D * kd, minOut, maxOut);
}

// функция пид левый
float computePIDL(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  float err = setpoint - input;
  static float integralL = 0, prevErrL = 0;
  integralL = constrain(integralL + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErrL) / dt;
  prevErrL = err;
  return constrain(err * kp + integralL + D * kd, minOut, maxOut);
}

void PWM_control(int CH, float set_speed) {
    speedLeft_encient = speedLeft;
    speedRight_encient = speedRight;
    int16_t PWM = 0;
    set_speed = set_speed;
    if (CH == 0x00) set_speed *= -1;  // Инвертирование левого двигателя
    if (set_speed > 0)                // Вращение вперед
    {
      PWM = set_speed * 4096;
      pwm.setPin(CH + 1, 0, false);  // Переключенеи направления вращения (если вращался в эту сторону)
      pwm.setPin(CH + 0, PWM, false);
    } else  // Вращение назад
    {
      set_speed *= -1;
      PWM = set_speed * 4096;
      pwm.setPin(CH + 0, 0, false);  // Переключенеи направления вращения (если вращался в эту сторону) CH+0 и CH+1 нужны для выбора канала
      pwm.setPin(CH + 1, PWM, false);
    }  
}

void StateControlMachine(char state, char s, long int pos) {
  switch (state) {
    case 0:
      // из состояния 0 можно попасть в состояние:
      // 1 – тогда это движение вперед;
      // или 2 – тогда это движение назад.
      if (s == 1) {
        pos++;
        state = s;
        break;
      }
      if (s == 2) {
        pos--;
        state = s;
        break;
      }
      break;
    case 1:
      // из состояния 1 можно попасть в состояние:
      // 3 – тогда это движение вперед;
      // или 0 – тогда это движение назад.
      if (s == 3) {
        pos++;
        state = s;
        break;
      }
      if (s == 0) {
        pos--;
        state = s;
        break;
      }
      break;
    case 2:
      // из состояния 2 можно попасть в состояние:
      // 0 – тогда это движение вперед;
      // или 3 – тогда это движение назад.
      if (s == 0) {
        pos++;
        state = s;
        break;
      }
      if (s == 3) {
        pos--;
        state = s;
        break;
      }
      break;
    case 3:
      // из состояния 3 можно попасть в состояние:
      // 2 – тогда это движение вперед;
      // или 1 – тогда это движение назад.
      if (s == 2) {
        pos++;
        state = s;
        break;
      }
      if (s == 1) {
        pos--;
        state = s;
        break;
      }
      break;
  }
  globalpos = pos;
  globalstate = state;
}

// Фильтр1
float Filter1(float newVal) {
  static float filValR[filterD] = {0};
  static float filPreR = 0;
  float k;
  int ind;
  float result = 0;
  for (ind =filterD-2;ind>=0;ind--){filValR[ind+1]=filValR[ind];}
  filValR[0] = newVal;
  for (ind =0;ind<filterD-1;ind++){result +=filValR[ind];}
  result = result/filterD;
  
  if (abs(result - filPreR)>0.16){k = 0.7;}
  else{k = 0.5;}
  filPreR += (result - filPreR)*k;
  return filPreR;
  
  return result;
}


// Фильтр2
float Filter2(float newVal) {
  static float filValL[filterD] = {0};
  static float filPreL = 0;
  float k;
  int ind;
  float result = 0;
  for (ind =filterD-2;ind>=0;ind--){filValL[ind+1] = filValL[ind];}
  filValL[0] = newVal;
  for (ind =0;ind<filterD-1;ind++){result +=filValL[ind];}
  result = result/filterD;
  
  if (abs(result - filPreL)>0.16){k = 0.7;}
  else{k = 0.5;}
  filPreL += (result - filPreL)*k;
  return filPreL;
  
  return result;
}

void speed_converter(float xl, float zw) {
  speedLeft = xl + -1 * zw * a / (r * r);
  speedRight = xl - -1 * zw * a / (r * r);
  speedLeft = constrain(speedLeft, -0.75, 0.75);
  speedRight = constrain(speedRight, -0.75, 0.75);
}

void odomPublish (double x_pos_, double y_pos_, float linear_vel_x, float angular_vel_z) {
  Odom_m_str = /*"*;" + */String(x_pos_, 5) + ';' + String(y_pos_, 5) + ';' + String(linear_vel_x, 5) + ';' + String(angular_vel_z, 5) + ';'/* + ";#"*/;
  Serial.println(Odom_m_str);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(10);
  pwm.begin();          // Инициализация
  pwm.setPWMFreq(1000);  // Частота следования импульсов 6000 Гц
  delay(10);            // Пауза

  Serial.begin(115200);
  inputString.reserve(150);

  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);
  pinMode(pin3, INPUT);
  pinMode(pin4, INPUT);

  delay(100);
}

void loop() {
  if (millis() - im_timer > im_timer_timeout){
    speedRight = 0;
    speedLeft = 0;
  }

  for(uint il = 0; il<128; il++)
  {
  StateControlMachine(stateR, (digitalRead(pin3) | digitalRead(pin4) << 1), posR);
  posR = globalpos;
  stateR = globalstate;
  
  StateControlMachine(stateL, (digitalRead(pin2) | digitalRead(pin1) << 1), posL);
  posL = globalpos;
  stateL = globalstate;
  }

  if (millis() - tmr >= timer_timeout) {
    tmr = millis();
    RealspeedRight= Filter1((float)(posR - posTempR)*1 / 2300*(timer_timeout));
    RealspeedLeft = Filter2((float)(posL - posTempL)*1 / 2300*(timer_timeout));

    float vel_dt = timer_timeout;
    float linear_vel_x = (RealspeedRight + RealspeedLeft)/2/0.6*50;
    float angular_vel_z = (RealspeedRight - RealspeedLeft)/a/0.1*9;
    float delta_heading = angular_vel_z * vel_dt; //radians
    float cos_h = cos(heading_);
    float sin_h = sin(heading_);
    float delta_x = (linear_vel_x * cos_h) * vel_dt/1000; //m
    float delta_y = (linear_vel_x * sin_h) * vel_dt/1000; //m

    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    posTempR = posR;
    posTempL = posL;
    
    PWM_control(MOTOR_RIGHT, computePIDR(RealspeedRight, speedRight, kp, ki, kd, timer_timeout, -1, 1));
    PWM_control(MOTOR_LEFT, computePIDL(RealspeedLeft, speedLeft, kp, ki, kd, timer_timeout, -1, 1));

    odomPublish(x_pos_, y_pos_, speedRight, angular_vel_z);
  }

  for(uint il = 0; il<128; il++)
  {
  StateControlMachine(stateR, (digitalRead(pin3) | digitalRead(pin4) << 1), posR);
  posR = globalpos;
  stateR = globalstate;
  
  StateControlMachine(stateL, (digitalRead(pin2) | digitalRead(pin1) << 1), posL);
  posL = globalpos;
  stateL = globalstate;
  }

  //Обработка входного значения
  if (stringComplete) {
    int i = 0;
    for (i = 1; i < inputString.length()-1; i++){
      if (inputString[i] == ';') break;
      input_m[0] += inputString[i];
    }

    for (i = i+1; i < inputString.length()-1; i++){
      if (inputString[i] == ';') break;
      input_m[1] += inputString[i];
    }
    char str1[input_m[0].length()];
    char str2[input_m[1].length()]; 
    for(i=0; i < input_m[0].length(); i++) str1[i] = input_m[0][i];
    for(i=0; i < input_m[1].length(); i++) str2[i] = input_m[1][i];
    //speed_converter(atof(input_m[0]), atof(input_m[1]));
    input_m[0] = "";
    input_m[1] = "";

    speed_converter(atof(str1), atof(str2));

    inputString = "";
    stringComplete = false;
  }
}

//Ответ на входное сообщение
void serialEvent() {
  im_timer = millis();
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if(inChar == '*' or input_open == true){
      // add it to the inputString:
      inputString += inChar;
      input_open = true;
    }
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '#') {
      stringComplete = true;
      input_open = false;
    }
  }
}