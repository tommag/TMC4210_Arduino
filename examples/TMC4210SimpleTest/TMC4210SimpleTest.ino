/*
 * TMC4210 simple test sketch.
 *
 * The position, speed and acceleration as reported by the motion controller is printed on the serial port every 50ms.
 *
 * Tested on ESP32
 *
 * Tom Magnier <tom@tmagnier.fr> 07/2018
 */
#include "TMC4210.h"

#define TMC4210_CS_PIN      16
#define TMC4210_CLK_PIN     17
#define SPI_SCK_PIN         5
#define SPI_MISO_PIN        19
#define SPI_MOSI_PIN        18

// #define TMC4210_CS_PIN      33
// #define TMC4210_CLK_PIN     32
// #define SPI_SCK_PIN         5
// #define SPI_MISO_PIN        17
// #define SPI_MOSI_PIN        16

#define TMC4210_CLK_FREQ    16000000L

TMC4210 tmc;

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting TMC4210 simple test...");

  //Init TMC4210 clock
  pinMode(TMC4210_CLK_PIN, OUTPUT);
  ledcSetup(0, TMC4210_CLK_FREQ, 2); //2 bits
  ledcAttachPin(TMC4210_CLK_PIN, 0);
  ledcWrite(0, 2);

  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  tmc.begin(TMC4210_CLK_FREQ, TMC4210_CS_PIN, 200*16*2, 200*16*8);
  tmc.setTargetPosition(1000);
}

void loop()
{
  Serial.print("Pos\t");
  Serial.print(tmc.getCurrentPosition());
  Serial.print("\tSpeed\t");
  Serial.print(tmc.getCurrentSpeed());
  Serial.print("\tAccel\t");
  Serial.println(tmc.getCurrentAcceleration());

  if (tmc.isTargetReached())
  {
    long nextTarget = tmc.getCurrentPosition() < 0 ? 10000 : -10000;
    tmc.setTargetPosition(nextTarget);
    Serial.print("New target : ");
    Serial.println(nextTarget);
  }

  delay(50);
}
