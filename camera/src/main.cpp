#include <Arduino.h>
// #include <U8G2lib.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <queue>

#include <SPI.h>
#include <SD.h>
#include "FS.h"

#include "reader.h"
#include "settings.h"

const int buttonPin = D1;
int buttonState = 0;

// u8g2_SSD1306_128X64_NONAME_F_HW_I2C //u8g2(//u8g2_R0, U8X8_PIN_NONE, SCL, SDA);

int frameDataLength;
uint16_t imageCount = 0;
char filename[32];
char logString[100];
char dumpster[10000];
bool isPressed = false;

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
  // u8g2.setCursor(0, 0);
  // u8g2.println(s);
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

  // //u8g2.begin();
  // u8x8.setFlipMode(1);
  // u8x8.setFont(u8x8_font_chroma48medium8_r); // choose a suitable font
  Wire.begin();

  // initialize the LED pin as an output:
  // pinMode(LED_BUILTIN, OUTPUT);
  // initialize the pushbutton pin as an input:
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

  CameraSerial.print(ATC_UNIT);
  delay(1000);
  CameraSerial.print(ATC_FPS);
  delay(1000);
  CameraSerial.print(ATC_BINNMODE);
  delay(1000);
  CameraSerial.print(ATC_DISP);

  Serial.print("Initializing SD card...");
  pinMode(D2, OUTPUT);
  if (!SD.begin(D2))
  {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
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
      freshPrint("File written");
      // u8g2.printf("File size: %8d", fileSize);

      Serial.printf("File written. File size: %8d\n", fileSize);
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
