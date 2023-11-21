#include <Arduino.h>
#include <deque>
#include <queue>
#include "reader.h"

std::deque<byte> rawData;
std::queue<byte> extractedFrame;

bool frameStarted;
bool isNewFrameReady = false;
uint16_t packetSize = 0;
byte checksum;

uint16_t fileSize = 0;

byte dataBuffer[11111];

HardwareSerial CameraSerial(0);

void queueMove(uint16_t amount)
{
    for (uint16_t i = 0; i < amount; i++)
    {
        extractedFrame.push(rawData.front());

        rawData.pop_front();
    }
}

void check()
{
    uint32_t sum = 0;
    for (uint16_t i = 0; i < packetSize - 2; i++)
    {
        sum += dataBuffer[i];
    }
    byte csum = (byte) sum & 0xff;

    if (csum == dataBuffer[packetSize + 3])
    {
        Serial.println("Checksum check failed.");
    }
}

void parseFrame()
{

    uint16_t unparsedLength = rawData.size();
    if (!frameStarted)
    {
        if (unparsedLength < 2)
        {
            return;
        }
        if (rawData[0] == 0x00 && rawData[1] == 0xff)
        {
            frameStarted = true;
            queueMove(2);
        }
        else
        {
            rawData.pop_front();

            //Serial.println("Dropping 1 byte.");
            return;
        }
    }

    if (frameStarted && packetSize == 0)
    {
        if (unparsedLength < 2)
        {
            return;
        }
        else
        {
            packetSize = ((uint16_t)rawData[0]) + ((uint16_t)rawData[1] << 8);
            queueMove(2);
        }
    }

    if (frameStarted && packetSize != 0)
    {
        if (unparsedLength > 50000)
        {
            while (!rawData.empty())
                rawData.pop_front();
            Serial.println("Buffer overflow. Cleared\n");
            packetSize = 0;
            frameStarted = false;

            return;
        }

        if (unparsedLength < packetSize + 2)
        {
            return;
        }

        queueMove(packetSize + 2);

        if (extractedFrame.back() != 0xdd)
        {
            Serial.println("Packet broken.");
            //Serial.printf("End of packet: %3d, extracted packet size: %6d, expected packet size: %6d\n", extractedFrame.back(), extractedFrame.size(), packetSize + 6);
            Serial.printf("Size of unparsed data: %4d, front byte: %4d, back byte: %d\n", rawData.size(), rawData.front(), rawData.back());
            Serial.printf("Size of   parsed data: %4d, front byte: %4d, back byte: %d\n", extractedFrame.size(), extractedFrame.front(), extractedFrame.back());
            // Serial.printf("extractedFrame size: %8d\n",extractedFrame.size());
            while (!extractedFrame.empty())
                extractedFrame.pop();

            packetSize = 0;
            frameStarted = false;

            return;
        }

        Serial.printf("Received frame. Copying to buffer. Frame end: %4d\n",extractedFrame.back());

        if (fileSize == 0)
        {
            fileSize = extractedFrame.size();
        }

        int i = 0;
        while (!extractedFrame.empty())
        {
            dataBuffer[i] = extractedFrame.front();
            extractedFrame.pop();
            i++;
        }
        Serial.printf("Copied %8d bytes\n", i);

        check();

        packetSize = 0;
        isNewFrameReady = true;
        frameStarted = false;
    }
}
