#include <TimerOne.h>

#include <SPI.h>

#include <EthernetClient.h>
#include <Dhcp.h>
#include <EthernetServer.h>
#include <util.h>
#include <Dns.h>
#include <EthernetUdp.h>
#include <Ethernet.h>

// Structs

struct pktHeader {
  uint8_t frames;
  uint8_t repeat;
};
#define HDR_SIZE sizeof(pktHeader)

struct frameData {
  uint8_t flags;
  uint8_t pause;
  uint8_t steps;
  uint8_t r;
  uint8_t g;
  uint8_t b;
};
#define FRAME_SIZE sizeof(frameData)

// Constants
const int pinR = 5;
const int pinG = 6;
const int pinB = 3;

const int pinL = 13;

byte mac[] = { 0x90, 0xa2, 0xda, 0x00, 0x8d, 0x7e };
IPAddress ip(192, 168, 1, 37);
unsigned int localPort = 30241;
EthernetUDP Udp;

// Globals
uint8_t oldLevelR = 0;
uint8_t oldLevelG = 0;
uint8_t oldLevelB = 0;

uint16_t curLevelR = 0;
uint16_t curLevelG = 0;
uint16_t curLevelB = 0;

uint16_t newLevelR = 0;
uint16_t newLevelG = 0;
uint16_t newLevelB = 0;

// Frame info;
bool    frameDirty = 0;
uint8_t frameCount = 0;
uint8_t frameWaitC = 0;
uint8_t framePause = 0;
uint8_t frameSteps = 0;
uint8_t frameRepeat = 0;
uint8_t framePosition = 0;

#define MAX_FRAMES 200
frameData frames[MAX_FRAMES];

#define BUFFER_SIZE 256
byte buffer[BUFFER_SIZE];
//byte packetBuffer[UDP_TX_PACKET_MAX_SIZE];

// functions
void setLight(int rVal, int gVal, int bVal) {
  curLevelR = rVal;
  curLevelG = gVal;
  curLevelB = bVal;
}

void updateChannel(uint16_t * curLevel, uint16_t newLevel, uint8_t steps) {
  uint16_t delta, change;
  if (newLevel > *curLevel) {
    delta = newLevel - *curLevel;
    change = (delta/(steps+1));
    *curLevel += change;
  } else {
    delta = *curLevel - newLevel;
    change = (delta/(steps+1));
    *curLevel -= change;
  }
}

void loadFrameData() {
  frameData frame = frames[framePosition];
  frameWaitC = framePause = frame.pause;
  frameSteps = frame.steps;
  if (frame.flags & 0x01) {
    newLevelR = (uint16_t)oldLevelR * (frame.r + 1);
    newLevelG = (uint16_t)oldLevelG * (frame.g + 1);
    newLevelB = (uint16_t)oldLevelB * (frame.b + 1);
  } else {
    newLevelR = frame.r<<8;
    newLevelG = frame.g<<8;
    newLevelB = frame.b<<8;
  }
}
  
void updateLights() {
  if (!frameDirty && frameCount > 0) {
    // Load frame data
    if (frameWaitC > 0) {
      frameWaitC--;
    } else { // frameWaitC == 0
      updateChannel(&curLevelR, newLevelR, frameSteps);
      updateChannel(&curLevelG, newLevelG, frameSteps);
      updateChannel(&curLevelB, newLevelB, frameSteps);
      if (frameSteps > 0) {
        frameSteps--;
        frameWaitC = framePause;
      } else { // frameSteps == 0
        // Done with this frame, advance
        framePosition++;
        if (framePosition >= frameCount) {
          switch (frameRepeat) {
            case 0: // no more repeats, mark dirty to stop updates
              frameDirty = true;
              break;
            case 255: // infinite repeats, do nothing
              break;
            default: // decrease repeats remaining
              frameRepeat--;
              break;
          }
          framePosition = 0;
        }
        loadFrameData();
      }
    }
  }
  analogWrite(pinR, curLevelR>>8);
  analogWrite(pinG, curLevelG>>8);
  analogWrite(pinB, curLevelB>>8);
}

void setup() {
  pinMode(pinL, OUTPUT);
  digitalWrite(pinL, LOW);
  
  digitalWrite(pinR, LOW);
  pinMode(pinR, OUTPUT);
  analogWrite(pinR, 0);

  digitalWrite(pinG, LOW);
  pinMode(pinG, OUTPUT);
  analogWrite(pinG, 0);

  digitalWrite(pinB, LOW);
  pinMode(pinB, OUTPUT);
  analogWrite(pinB, 0);

//  Serial.begin(9600);

  Ethernet.begin(mac, ip);
  Udp.begin(localPort);

  Timer1.initialize(15625); // 1/64 second
  Timer1.attachInterrupt(updateLights, 15625);
}

void loop() {
  int packetLeft = Udp.parsePacket();
  if (packetLeft) {
    Udp.read(buffer, HDR_SIZE);
    if (packetLeft > HDR_SIZE) {
      packetLeft -= HDR_SIZE;
      pktHeader hdr;
      memcpy(&hdr, buffer, HDR_SIZE);
      int expectedPayloadSize = hdr.frames * FRAME_SIZE;
      if (hdr.frames > 0 && hdr.frames <= MAX_FRAMES && packetLeft == expectedPayloadSize) {
        frameDirty = true;
        Udp.read((byte *)frames, expectedPayloadSize);
        oldLevelR = curLevelR>>8;
        oldLevelG = curLevelG>>8;
        oldLevelB = curLevelB>>8;
        packetLeft = 0;
        frameCount = hdr.frames;
        frameRepeat = hdr.repeat;
        framePosition = 0;
        loadFrameData();
        frameDirty = false;
      }
    }
    
    // Consume anything left
    while (packetLeft > 0) {
      if (packetLeft > BUFFER_SIZE) {
        Udp.read(buffer, BUFFER_SIZE);
        packetLeft -= BUFFER_SIZE;
      } else {
        Udp.read(buffer, packetLeft);
        packetLeft = 0;
      }
    }      
  }
}

