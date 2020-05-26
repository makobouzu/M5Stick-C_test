#include <M5StickC.h>

#include "DHT12.h"
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>
DHT12 dht12;
Adafruit_BMP280 bme;

#include <TinyGPS++.h>
HardwareSerial GPSRaw(2);
TinyGPSPlus gps;

#include <driver/i2s.h>
#define PIN_CLK  0
#define PIN_DATA 34
#define READ_LEN (2 * 256)
#define GAIN_FACTOR 3
uint8_t BUFFER[READ_LEN] = {0};
uint16_t oldy[160];
int16_t *adcBuffer = NULL;


int sceen = 0;

void setup() {
  M5.begin();
  Wire.begin(0, 26);
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(0, 0, 2);

  if(!bme.begin(0x76)){
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while(1);
  }

  GPSRaw.begin(9600, SERIAL_8N1, 33, 32);
}

void loop() {
  M5.update();
  if(sceen == 0){
    M5.Lcd.setCursor(0, 0, 2);
    M5.Lcd.println("ENV TEST");
    while(GPSRaw.available()>0){
      if(gps.encode(GPSRaw.read())){
        break;
      }
    }

    float tmp  = dht12.readTemperature();
    float hum  = dht12.readHumidity();
    float pres = bme.readPressure();
    float lat = gps.location.lat();
    float lng = gps.location.lng();
    if(gps.location.isValid()){
      float lat = gps.location.lat();
      float lng = gps.location.lng();
    }
    M5.Lcd.setCursor(0, 20, 2);
    M5.Lcd.printf("Temp: %2.1f Humi: %2.0f%%", tmp, hum);
    M5.Lcd.setCursor(0, 40, 2);
    M5.Lcd.printf("pressure: %2.1f", pres);
    M5.Lcd.setCursor(0, 60, 2);
    if(gps.location.isValid()){
      M5.Lcd.printf("Lat: %.6f\n Lng: %.6f\n", lat, lng);
    }else{
      M5.Lcd.printf("GPS: INVALID\n");
    }

    delay(100);

    if(M5.BtnB.wasPressed()){
      sceen = 1;
      M5.Lcd.setCursor(0, 0, 2);
      M5.Lcd.fillScreen(BLACK);
      i2sInit();
      xTaskCreate(micRecordTask, "micRecordTask", 2048, NULL, 1, NULL);
    }
  }else if(sceen == 1){
    M5.Lcd.setCursor(0, 0, 2);
    M5.Lcd.println("MIC TEST");

    vTaskDelay(1000 / portTICK_RATE_MS);

    if(M5.BtnB.wasPressed()){
      sceen = 0;
      M5.Lcd.fillScreen(BLACK);
      if(!bme.begin(0x76)){
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        while(1);
      }
      GPSRaw.begin(9600, SERIAL_8N1, 33, 32);
      M5.Lcd.setCursor(0, 0, 2);
    }
  }
}

void i2sInit(){
   i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate =  44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
    .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = 128,
   };

   i2s_pin_config_t pin_config;
   pin_config.bck_io_num   = I2S_PIN_NO_CHANGE;
   pin_config.ws_io_num    = PIN_CLK;
   pin_config.data_out_num = I2S_PIN_NO_CHANGE;
   pin_config.data_in_num  = PIN_DATA;
  
   
   i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
   i2s_set_pin(I2S_NUM_0, &pin_config);
   i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

void micRecordTask (void* arg){   
  size_t bytesread;
  while(1){
    i2s_read(I2S_NUM_0,(char*) BUFFER, READ_LEN, &bytesread, (100 / portTICK_RATE_MS));
    adcBuffer = (int16_t *)BUFFER;
    showSignal();
    vTaskDelay(100 / portTICK_RATE_MS);
  }
}

void showSignal(){
  int y;
  for (int n = 0; n < 160; n++){
    y = adcBuffer[n] * GAIN_FACTOR;
    y = map(y, INT16_MIN, INT16_MAX, 10, 70);
    M5.Lcd.drawPixel(n, oldy[n],BLACK);
    M5.Lcd.drawPixel(n,y,WHITE);
    oldy[n] = y;
  }
}
