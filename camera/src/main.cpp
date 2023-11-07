#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>

#include <SPI.h>
#include <SD.h>
#include "FS.h"

#include <U8x8lib.h>

#include "reader.h"
#include "settings.h"

const int buttonPin = D1;
int buttonState = 0;

uint16_t imageCount = 0;
char filename[32];
char logString[100];
char dumpster[10000];
bool isPressed = false;


U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // OLEDs without Reset of the Display

void refreshIndex(void)
{
  while (true)
  {
    sprintf(filename, "/IMG_%04d.bin", imageCount);
    if (SD.exists(filename))
    {
      imageCount++;
    }
    else
    {
      break;
    }
  }
}

void freshPrint(const char *s)
{
  u8x8.clear();
  u8x8.setCursor(0, 0);
  u8x8.println(s);
}


void log(char *string)
{
  Serial.println(string);
  freshPrint(string);
}

void writeFile(fs::FS &fs, const char *path, uint8_t *data, size_t len)
{
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.write(data, len) == len)
  {
    Serial.println("File written");
  }
  else
  {
    Serial.println("Write failed");
  }
  file.close();

  imageCount++;
  sprintf(filename, "/IMG_%04d.bin", imageCount);
}

void setup()
{

  u8x8.begin();
  u8x8.setFlipMode(1);
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
  Wire.begin();

  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(9600);

  // Begin serial with A010's default baudrate.
  CameraSerial.begin(115200, SERIAL_8N1, RX, TX);
  while (!CameraSerial)
  {
    delay(100);
  }
  delay(2000);

  // Then set to a higher baudrate. Otherwise the A010 module will became sluggish.
  CameraSerial.print(ATC_BAUDRATE);
  delay(1000);
  CameraSerial.updateBaudRate(230400);
  delay(1000);

  log("Setting up \n camera...");

  CameraSerial.print(ATC_UNIT);
  delay(1000);
  CameraSerial.print(ATC_FPS);
  delay(1000);
  CameraSerial.print(ATC_BINNMODE);
  delay(1000);
  CameraSerial.print(ATC_DISP);

  log("Initializing \n SD card...");
  pinMode(D2, OUTPUT);
  if (!SD.begin(D2))
  {
    log("initialization \n failed!");
    return;
  }
  log("initialization \n done.");
  refreshIndex();
}

void loop()
{
  buttonState = digitalRead(buttonPin);

  if (buttonState == LOW)
  {
    if (!isPressed)
    {
      
      isPressed = true;
      writeFile(SD, filename, dataBuffer, fileSize);

      u8x8.clear();
      u8x8.printf("%s \nwritten.\n", filename);
      u8x8.printf("File size:\n %6d\n", fileSize);
      

      Serial.printf("File written. \n File size: \n %8d\n", fileSize);
    }
  }
  else
  {
    isPressed = false;
  }

  while (CameraSerial.available())
  {
    rawData.push_back((byte)CameraSerial.read());
  }

  parseFrame();
}
