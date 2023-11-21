#include <Arduino.h>
#include <HardwareSerial.h>
#include <deque>
#include <queue>
extern HardwareSerial CameraSerial;
extern uint16_t packetSize;
extern std::deque<byte> rawData;
extern std::queue<byte> extractedFrame;
extern void parseFrame();
extern uint8_t blockingRead(void);
extern uint8_t dataBuffer[11111];
extern uint16_t fileSize;
extern bool isNewFrameReady;