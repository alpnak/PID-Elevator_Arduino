#include <LedControl.h> // 8x8 LED Matrix library
#include <LiquidCrystal.h> // 2x16 LCD Display library

#define BUTTON_1 4 // Button is connected to D4 pin
#define BUTTON_2 5 // Button is connected to D5 pin
#define BUTTON_3 6 // Button is connected to D6 pin

#define PIN_TRIG 2 // HC-SR04 trig pin is connected to D2 pin
#define PIN_ECHO 3 // HC-SR04 echo pin is connected to D3 pin

#define PIN_IN1 42 // L298N IN1 pin is connected to D42 pin
#define PIN_IN2 40 // L298N IN2 pin is connected to D40 pin
#define PIN_ENA 7  // L298N ENA pin is connected to D7 pin

LiquidCrystal lcd(52, 50, 48, 46, 53, 51); // 2x16 LCD Display VSS pin is connected to GND pin
                                           // 2x16 LCD Display VDD pin is connected to 5V pin
                                           // 2x16 LCD Display V0 pin is connected to 10K pot mid pin
                                           // 2x16 LCD Display RS pin is connected to D52 pin
                                           // 2x16 LCD Display E pin is connected to D50 pin
                                           // 2x16 LCD Display D4 pin is connected to D48 pin 
                                           // 2x16 LCD Display D5 pin is connected to D46 pin
                                           // 2x16 LCD Display D6 pin is connected to D53 pin
                                           // 2x16 LCD Display D7 pin is connected to D51 pin
                                           // 2x16 LCD Display A pin is connected to 5V pin 
                                           // 2x16 LCD Display K pin is connected to GND pin

LedControl lc = LedControl(11, 12, 13, 1); // 8x8 LED Matrix DIN pin is connected to D11 pin
                                           // 8x8 LED Matrix CLK pin is connected to D12 pin
                                           // 8x8 LED Matrix CS pin is connected to D13 pin

int target_floor = 0;
int motor_signal;

long duration; // HC-SR04 response duration

unsigned long previous_millis = 0; // Zaman ölçümü için değişken

const long millis_interval = 400; // 0.4 saniyelik aralık

float height = 0; // Cabin height

float kP = 0.0; // P-Controller gain
float kD = 0.0; // D-Controller gain
float kI = 0.0; // I-Controller gain

float error_P = 0.0; // Difference in target floor height and current cabin height, for P-controller
float error_I = 0.0; // Difference in target floor height and current cabin height, for I-controller
float error_PI = 0.0; // Difference in target floor height and current cabin height, for PI-controller
float error_PD = 0.0; // Difference in target floor height and current cabin height, for PD-controller
float error_PID = 0.0; // Difference in target floor height and current cabin height, for PID-controller

float integral_I = 0.0; // Integral of error_I, cumulative sum of it, over time, for I-controller
float integral_PI = 0.0; // Integral of error_PI, cumulative sum of it, over tiem, for PI-controller
float integral_PID = 0.0; // Integral of error_PID, cumulative sum of it, over time, for PID-controller

float previous_error_PD = 0.0; // Previous value of error_PD, for PD-controller
float previous_error_PID = 0.0; // Previous value of error_PID, for PID-controller

float derivative_PD = 0.0; // Derivative of error_PD, difference between current and previous value of it, with respect to time, for PD-controller
float derivative_PID = 0.0; // Derivative of error_PID, difference between current and previous value of it, with respect to time, for PID-controller

float motor_voltage;

bool button_pressed = false;

bool is_P_controller = false;
bool is_I_controller = false;
bool is_PD_controller = false;
bool is_PI_controller = false;
bool is_PID_controller = false;

void setup()
{
  pinMode(BUTTON_1, INPUT_PULLUP); // D4 pin is set as input pin that enables the internal pull-up resistor
  pinMode(BUTTON_2, INPUT_PULLUP); // D5 pin is set as input pin that enables the internal pull-up resistor
  pinMode(BUTTON_3, INPUT_PULLUP); // D6 pin is set as input pin that enables the internal pull-up resistor

  pinMode(PIN_TRIG, OUTPUT); // D2 pin is set as output pin
  pinMode(PIN_ECHO, INPUT);  // D3 pin is set as input pin

  pinMode(PIN_IN1, OUTPUT); // D42 pin is set as output pin
  pinMode(PIN_IN2, OUTPUT); // D40 pin is set as output pin
  pinMode(PIN_ENA, OUTPUT); // D7 pin is set as output pin
  
  lcd.begin(16, 2); // Initialize 2x16 LCD Display with 16 characters and 2 lines
  
  lc.shutdown(0, false); // Activate 8x8 LED Matrix
  lc.setIntensity(0, 8); // Set brightness level of 8x8 LED MAtrix to 8
  lc.clearDisplay(0); // Clear 8x8 LED Matris

  Serial.begin(9600); // Start serial communication and set baud rate to 9600
}

void loop()
{
  if (digitalRead(BUTTON_1) == LOW) // If BUTTON_1 is pressed
  {
    target_floor = 1; // target_floor is set to 1, indicating 1. floor
    delay(200); // Wait for debounce of the button
    button_pressed = true;
  }
  else if (digitalRead(BUTTON_2) == LOW) // If BUTTON_2 is pressed
  {
    target_floor = 2; // target_floor is set to 2, indicating 2. floor
    delay(200); // Wait for debounce of the button
    button_pressed = true;
  } 
  else if (digitalRead(BUTTON_3) == LOW) // If BUTTON_3 is pressed
  {
    target_floor = 3; // target_floor is set to 3, indicating 3. floor
    delay(200); // Wait for debounce of the button
    button_pressed = true;
  }

  digitalWrite(PIN_TRIG, LOW); // HC-SR04 trig pin is set to LOW
  delayMicroseconds(2); // Wait for 2 microseconds
  digitalWrite(PIN_TRIG, HIGH); // HC-SR04 trig pin is set to HIGH
  delayMicroseconds(10); // Wait for 10 microseconds
  digitalWrite(PIN_TRIG, LOW); // HC-SR04 trig pin is set to LOW

  duration = pulseIn(PIN_ECHO, HIGH); // Read HC-SR04 response duration from echo pin in microseconds

  height = duration * 0.03432 / 2; // Calculate the height of the cabin in centimeters, (cabin height) =  ((HC-SR04 response duration) * (sound speed in air)) / 2
  
  if (height < 25) show_floor_number(1); // If cabin height is less than 25 cm, show 1 on 8x8 LED Matrix, indicatin 1. floor
  else if (height < 50) show_floor_number(2); // Else if cabin height is less than 50 cm, show 2 on 8x8 LED Matrix, indicating 2. floor
  else show_floor_number(3); // Else, show 3 on 8x8 LED Matrix, indicating 3. floor

  // Read potentiometer values for kP, kD and kI
  kP = analogRead(A0) / 1023.0 * 10.0; // Scale to 0-10
  kD = analogRead(A1) / 1023.0 * 10.0; // Scale to 0-10
  kI = analogRead(A2) / 1023.0 * 10.0; // Scale to 0-10

  // Determine control algorithm
  if (button_pressed == true)
  {
    if (kP == 0 && kD == 0 && kI == 0) ON_OFF_controller((25 * (target_floor - 1)) + 5);
    else if (kP > 0 && kD == 0 && kI == 0)
    {
      P_controller((25 * (target_floor - 1)) + 5);
      is_P_controller = true;
    }
    else if (kP == 0 && kD == 0 && kI > 0)
    {
      I_controller((25 * (target_floor - 1)) + 5);
      is_I_controller = true;
    }
    else if (kP > 0 && kD > 0 && kI == 0)
    {
      PD_controller((25 * (target_floor - 1)) + 5);
      is_PD_controller = true;
    }
    else if (kP > 0 && kD == 0 && kI > 0)
    {
      PI_controller((25 * (target_floor - 1)) + 5);
      is_PI_controller = true;
    }
    else if (kP > 0 && kD > 0 && kI > 0)
    {
      PID_controller((25 * (target_floor - 1)) + 5);
      is_PID_controller = true;
    }
    else
    {
      lcd.setCursor(0, 1);
      lcd.print("ERROR        ");
    }

    button_pressed = false;
  }

//  if (target_floor == 1)
//  {
//        //ON_OFF_CONTROLLER(5);
//        pController(5);
//        //piController(5);
//        //pdController(5);
//        //pidController(5);
//  }
//  else if (target_floor == 2)
//  {
//       //ON_OFF_CONTROLLER(30); 
//       pController(30);
//       //piController(30);
//       //pdController(30);
//       //pidController(30);
//  }
//  else if (target_floor == 3)
//  {
//        //ON_OFF_CONTROLLER(55);
//        pController(55);
//        //piController(55);
//        //pdController(55);
//        //pidController(55);
//  }

  unsigned long current_millis = millis();

  if (current_millis - previous_millis >= millis_interval)
  {
    previous_millis = current_millis; // Time update
    
    // Print cabin height to serial port monitor
    Serial.print("Height: ");
    Serial.print(height);
    Serial.println(" cm");

    // Print floor number to serial port monitor
    Serial.print("Floor: ");
    Serial.println(target_floor);

    // Print gains to serial port monitor
    Serial.print("kp = ");
    Serial.print(kP);
    Serial.print(" | kd = ");
    Serial.print(kD);
    Serial.print(" | ki = ");
    Serial.println(kI);

    // Print motor voltage to serial port monitor
    Serial.print("Motor voltage: ");
    Serial.print((motor_signal * 7) / 255);
    Serial.println(" V");

    // Print motor signal to serial port monitor
    Serial.print("Motor signal: ");
    Serial.println(motor_signal);

    if (is_P_controller == true)
    {
      // Print P-controller parameters to serial port monitor
      Serial.print("P-controller error: ");
      Serial.println(error_P);
      // is_P_controller = false;
    }
    else if (is_I_controller == true)
    {
      // Print I-controller parameters to serial port monitor
      Serial.print("I-controller error: ");
      Serial.println(error_I);
      Serial.print("I-controller integral: ");
      Serial.println(integral_I);
      // is_I_controller = false;
    }
    else if (is_PD_controller == true)
    {
      // Print PD-controller parameters to serial port monitor
      Serial.print("PD-controller error: ");
      Serial.println(error_PD);
      Serial.print("PD-controller derivative: ");
      Serial.println(derivative_PD);
      // is_PD_controller = false;
    }
    else if (is_PI_controller == true)
    {
      // Print PI-controller parameters to serial port monitor
      Serial.print("PI-controller error: ");
      Serial.println(error_PI);
      Serial.print("PI-controller integral: ");
      Serial.println(integral_PI);
      // is_PI_controller = false;
    }
    else if (is_PID_controller == true)
    {
      // Print PID-controller parameters to serial port monitor
      Serial.print("PID-controller error: ");
      Serial.println(error_PID);
      Serial.print("PID-controller integral: ");
      Serial.println(integral_PID);
      Serial.print("PID-controller derivative: ");
      Serial.println(derivative_PID);
      // is_PID_controller = false;
    }
    Serial.print("\n");
  }

  update_LCD(height, kP, kI, kD);
    
  delay(100);
}

void update_LCD(float cabin_height, float P_gain, float I_gain, float D_gain)
{
  // 1. satıra mesafe yaz
  lcd.setCursor(0, 0);
  lcd.print("Height: ");
  lcd.print(int(cabin_height));
  lcd.print("cm  "); // "  " boşluk bırakmak eski yazıyı temizler

  // 2. satıra PID katsayılarını yaz
  lcd.setCursor(0, 1);
  lcd.print("kP=");
  lcd.print(int(P_gain));
  lcd.print(" kI=");
  lcd.print(int(I_gain));
  lcd.print(" kD=");
  lcd.print(int(D_gain));
  lcd.print("   "); // Eski yazıyı temizlemek için boşluk
}

// ON-OFF kontrol fonksiyonu
void ON_OFF_controller(int target_height)
{
  if (height < target_height)
  {
    // Motoru ileri yönde çalıştırma
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, 255); // Motoru hızla çalıştırma
  }
  else if (height > target_height)
  {
    // Motoru geri yönde çalıştırma
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_ENA, 255); // Motoru hızla çalıştırma
  }
  else
  {
    // Hedef mesafeye ulaşıldığında motoru durdurma
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, 0); // Motoru durdurma
  }
}

// P kontrol fonksiyonu
void P_controller(int target_height)
{
  error_P = target_height - height;
  
  // Hata negatifse motor geri döner, pozitifse ileri gider
  if (error_P > 0.4)
  {
    // Motoru ileri yönde çalıştırma
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  }
  else if (error_P < 0)
  {
    // Motoru geri yönde çalıştırma
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
  else
  {
    // Hedef mesafeye ulaşıldı, motoru durdur
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }
  
  // Motor gücünü sınırlama (0 ile 255 arasında)
  motor_signal = constrain(abs(kP * error_P), 0, 255);
  
  // Motor hızını ayarla
  analogWrite(PIN_ENA, motor_signal);
}

void I_controller(int target_height)
{
  error_I = target_height - height;

  integral_I += error_I;

  // Yönü belirle
  if (error_I > 0.4)
  {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  }
  else if (error_I < 0)
  {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
  else
  {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }

  motor_signal = constrain(abs(kI * integral_PI), 0, 255);  // Mutlak değerini al ve sınırlama uygula

  // Motor hızını ayarla
  analogWrite(PIN_ENA, motor_signal);
}

// PI kontrol fonksiyonu
void PI_controller(int target_height)
{
  error_PI = target_height - height;
  
  integral_PI += error_PI;  // Integral, hata üzerinde birikim yapar
  
  // Yönü belirle
  if (error_PI > 0.4)
  {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  }
  else if (error_PI < 0)
  {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
  else
  {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }

  motor_signal = constrain(abs((kP * error_PI) + (kI * integral_PI)), 0, 255);  // Mutlak değerini al ve sınırlama uygula
  
  // Motor hızını ayarla
  analogWrite(PIN_ENA, motor_signal);
  
//  // Seri monitöre değerleri yazdır
//  Serial.print("Hata: ");
//  Serial.print(error2);
//  Serial.print(", Integral: ");
//  Serial.print(integral);
//  Serial.print(", Motor Gücü: ");
//  Serial.println(motorPower);
}

// PD kontrol fonksiyonu
void PD_controller(int target_height)
{
  error_PD = target_height - height;

  derivative_PD = error_PD - previous_error_PD; // Hatanın türevini hesapla

  // Motorun yönünü belirle
  if (error_PD > 0.4)
  {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);  // İleri yönde dön
  }
  else if (error_PD < 0)
  {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);  // Geri yönde dön
  }
  else
  {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);  // Motoru durdur
  }

  motor_signal = constrain(abs((kP * error_PD) + (kD * derivative_PD)), 0, 255);  // Mutlak değer alınır

  // Motor hızını ayarla
  analogWrite(PIN_ENA, motor_signal);

  // Önceki hatayı güncelle
  previous_error_PD = error_PD;

//  // Seri monitöre değerleri yazdır
//  Serial.print("Mesafe: ");
//  Serial.print(height);
//  Serial.print(" cm, Hata: ");
//  Serial.print(error3);
//  Serial.print(", Türev: ");
//  Serial.print(derivative);
//  Serial.print(", Motor Gücü: ");
//  Serial.println(motorPower);
}

// PID kontrol fonksiyonu
void PID_controller(int target_height)
{
  error_PID = target_height - height;

  integral_PID += error_PID;

  derivative_PID = error_PID - previous_error_PID; // Derivative hesapla (hata değişim oranı)

  // Motor yönünü belirle
  if (error_PID > 0.4)
  {
    // Motor ileri yönde dönmeli
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  }
  else if (error_PID < 0)
  {
    // Motor geri yönde dönmeli
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
  else
  {
    // Hedefe ulaşıldığında motoru durdur
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }

  motor_signal = constrain(abs((kP * error_PID) + (kI * integral_PID) + (kD * derivative_PID)), 0, 255);

  // Motor hızını ayarla
  analogWrite(PIN_ENA, motor_signal);

  // Önceki hatayı güncelle (D terimi için)
  previous_error_PID = error_PID;

//  // Seri monitöre değerleri yazdır (debugging için)
//  Serial.print("Mesafe: ");
//  Serial.print(height);
//  Serial.print(" cm, Hata: ");
//  Serial.print(error);
//  Serial.print(", Integral: ");
//  Serial.print(integral2);
//  Serial.print(", Derivative: ");
//  Serial.print(derivative);
//  Serial.print(", Motor Gücü: ");
//  Serial.println(motorPower);
}

void show_floor_number(int floor_number)
{
  if (floor_number == 1)
  {
    lc.clearDisplay(0);
    
    byte floor_1[8] = {
                      B00000000,
                      B00011000,
                      B00111000,
                      B00011000,
                      B00011000,
                      B00011000,
                      B00111110,
                      B00000000
                      };
                      
    for (int i = 0; i < 8; i++)
    {
      lc.setRow(0, i, floor_1[i]);
    }  
  }
  else if (floor_number == 2)
  {
    lc.clearDisplay(0);
    
    byte floor_2[8] = {
                      B00000000,
                      B00111100,
                      B01100110,
                      B00000110,
                      B00001100,
                      B00110000,
                      B01111110,
                      B00000000
                      };
                      
    for (int i = 0; i < 8; i++)
    {
      lc.setRow(0, i, floor_2[i]);
    }  
  }
  else if (floor_number == 3)
  {
    lc.clearDisplay(0);
    
    byte floor_3[8] = {
                      B00000000,
                      B00111100,
                      B01100110,
                      B00001100,
                      B00001100,
                      B01100110,
                      B00111100,
                      B00000000
                      };
                      
    for (int i = 0; i < 8; i++)
    {
      lc.setRow(0, i, floor_3[i]);
    }    
  }
}
