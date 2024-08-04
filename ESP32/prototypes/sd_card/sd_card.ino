#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define SPI_CLK  5
#define SPI_MISO 18
#define SPI_MOSI 19
#define CS1  15

SPIClass spi = SPIClass(VSPI);

void setup() {
    Serial.begin(230400);
    spi.begin(SPI_CLK, SPI_MISO, SPI_MOSI, CS1);
    
    if(!SD.begin(CS1, spi, 80000000)){
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

}

void loop() {
  // put your main code here, to run repeatedly:

}
