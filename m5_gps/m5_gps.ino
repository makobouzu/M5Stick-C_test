#include <M5StickC.h>
#include <TinyGPS++.h>

HardwareSerial GPSRaw(2);
TinyGPSPlus gps;

void setup() {
  M5.begin();
  M5.Lcd.setRotation(3);
  GPSRaw.begin(9600, SERIAL_8N1, 33, 32);

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE, BLACK);
}

void loop() {
  M5.Lcd.setCursor(0, 0, 2);
  M5.Lcd.println("GPS TEST");
  while(GPSRaw.available()>0) {
    if(gps.encode(GPSRaw.read())) {
      break;
    }
  }
  if(gps.location.isValid()) {
    M5.Lcd.printf("LAT:%.6f\n", gps.location.lat() );
    M5.Lcd.printf("LNG:%.6f\n", gps.location.lng() );
    M5.Lcd.printf("ALT:%.2f\n", gps.altitude.meters() );
  } else {
    M5.Lcd.printf("INVALID\n");
  }
  delay(5000);
}
