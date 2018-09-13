/*
 * TMC4210 velocity mode sketch.
 * Initializes the IC then listen on the serial port.
 * Write speed values (microsteps / second) to get the motor turning.
 * *
 * Tested on ESP32
 *
 * Tom Magnier <tom@tmagnier.fr> 07/2018
 */
#include "TMC4210.h"

// #define TMC4210_CS_PIN      16
// #define TMC4210_CLK_PIN     17
#define TMC4210_CS_PIN      33
#define TMC4210_CLK_PIN     32
#define SPI_SCK_PIN         5
#define SPI_MISO_PIN        17
#define SPI_MOSI_PIN        16

#define TMC4210_CLK_FREQ    16000000L

TMC4210 tmc;

//TODO test if max speed has been set lower

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting TMC4210 velocity mode test...");

  //Init TMC4210 clock
  pinMode(TMC4210_CLK_PIN, OUTPUT);
  ledcSetup(0, TMC4210_CLK_FREQ, 2); //2 bits
  ledcAttachPin(TMC4210_CLK_PIN, 0);
  ledcWrite(0, 2);

  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  tmc.begin(TMC4210_CLK_FREQ, TMC4210_CS_PIN, 200*32*4, 200*32*2);
  tmc.setRampMode(TMC4210::VELOCITY_MODE);
  tmc.setMaxSpeed(200*16);
  tmc.setTargetSpeed(0);
}

void loop()
{
  if (Serial.available())
  {
    int newSpeed = Serial.parseInt();
    Serial.print("Setting speed to ");
    Serial.print(newSpeed);
    Serial.println(" usteps / s");
    tmc.setTargetSpeed(newSpeed);
  }

  static unsigned long lastPrint = millis();
  if (millis() - lastPrint > 250)
  {
    lastPrint = millis();
    Serial.print("Pos\t");
    Serial.print(tmc.getCurrentPosition());
    Serial.print("\tSpeed\t");
    Serial.print(tmc.getCurrentSpeed());
    Serial.print("\tAccel\t");
    Serial.println(tmc.getCurrentAcceleration());
  }
}
