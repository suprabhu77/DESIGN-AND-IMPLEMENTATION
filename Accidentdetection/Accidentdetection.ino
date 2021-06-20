#include <LiquidCrystal.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <SimpleDHT.h>
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);
TinyGPS gps;  //Creates a new instance of the TinyGPS object

int pinDHT11 = 10;
SimpleDHT11 dht11(pinDHT11);
SoftwareSerial SIM900(7, 11);

#define vibration_sen 8 
#define MQPin A0
#define buzzer 9

void setup()
{ 
  Serial.begin(9600);
  SIM900.begin(9600);
  lcd.begin(16, 2);
  pinMode(vibration_sen, INPUT);
   delay(1000);
   pinMode(MQPin, INPUT);
   pinMode(buzzer, OUTPUT);
    lcd.setCursor(0, 0);
    lcd.print("ACCIDENT DETECTI");
    lcd.setCursor(0, 1);
    lcd.print("-ON AND ANALYSIS");
    delay(1000);
    lcd.clear();
    Serial.begin(9600);
 
}

void loop()
{  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  int vibration = digitalRead(vibration_sen);
  int gas_value = digitalRead(MQPin);
  
  if(gas_value==HIGH)
    { 
      digitalWrite(buzzer, HIGH);
//      delay(1000);
//      digitalWrite(buzzer,LOW);
      lcd.setCursor(0, 0);
      lcd.print("ALCOHOL DETECTED");
      lcd.setCursor(0, 1);
      lcd.print("CAR TURNED OFF");
//      delay(1000);
//      lcd.clear();
      delay(100);
    }
    else
    {
     lcd.clear();
     digitalWrite(buzzer, LOW); 
    }

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial.available())
    {
      char c = Serial.read();
      //Serial.print(c);
      if (gps.encode(c)) 
        newData = true;  
    }
  }

if(vibration == HIGH) {
  if (newData)      //If newData is true
  { 
    lcd.clear();
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age); 
//    float lat=(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);  
//    float lon=(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    SIM900.print("AT+CMGF=1\r");
    lcd.setCursor(0,0);
    lcd.print("Lat= ");
    lcd.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    lcd.setCursor(0,1);
    lcd.print("Lon= ");
    lcd.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    delay(1000);
    lcd.clear();
    delay(400);
    SIM900.println("AT + CMGS = \"+917795865462\"");// recipient's mobile number with country code
//    delay(300);
//    double lat = {flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6
    SIM900.print("Latitude = ");
    SIM900.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    SIM900.print(" Longitude = ");
    SIM900.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    delay(200);
    SIM900.print("https://www.google.com/maps/@" + String(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6) + "," + String(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6) + String("15z"));
    delay(200);
    SIM900.println((char)26); // End AT command with a ^Z, ASCII code 26
    delay(200);
    SIM900.println();

    Serial.println(failed);
    
  }
}
 
  Serial.println(failed);
//    if (chars == 0)
//    Serial.println("** No characters received from GPS: check wiring **");
   
  // read without samples.
  byte temperature = 0;
  byte humidity = 0;
  int err = SimpleDHTErrSuccess;
  if ((err = dht11.read(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT11 failed, err="); Serial.print(SimpleDHTErrCode(err));
    Serial.print(","); Serial.println(SimpleDHTErrDuration(err));
    delay(1000);
    return;
  }
  
//  Serial.print("Sample OK: ");
  Serial.print((int)temperature); Serial.print(" *C, "); 
  Serial.print((int)humidity); Serial.println(" H");
  
// DHT11 sampling rate is 1HZ.
//  delay(150);
    
}
