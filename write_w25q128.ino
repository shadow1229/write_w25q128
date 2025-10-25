/*
  current issue: this program writes data on Flash ROM in Feather M4, not W25Q128JVSSIQ.
  
  W25Q128JVSSIQ Flash Programmer for Adafruit Feather M4

  This sketch demonstrates how to program an external Winbond
  W25Q128JVSSIQ Quad‑SPI flash chip from an Adafruit Feather M4.  The
  program expects the binary data to be streamed over the USB serial
  connection and writes it directly to the flash, erasing only the
  necessary sectors first.  It will also verify the contents of the
  flash after programming and report any errors to the serial console.

  Serial protocol
  ---------------
  The Feather waits for a 32‑bit little‑endian length header sent over
  the serial port.  This header specifies the number of bytes in the
  binary image to be programmed.  Immediately after the length header
  the host should transmit exactly that many bytes of binary data.
  The Feather writes the data to successive pages of the flash as
  bytes arrive.  When all bytes have been received the sketch
  verifies the contents of the flash and prints a success or error
  message.

  To send a binary file from your computer you can use a simple
  Python script like this:

    import serial, struct, sys
    with open('firmware.bin', 'rb') as f:
        data = f.read()
    ser = serial.Serial('/dev/ttyACM0', 115200)
    ser.write(struct.pack('<I', len(data)))
    ser.write(data)

  Replace '/dev/ttyACM0' with the appropriate serial device on your
  system (e.g. COM3 on Windows).  The script writes the length of
  your firmware as a 32‑bit little‑endian integer followed by the
  firmware bytes themselves.

  About the W25Q128JVSSIQ
  -----------------------
  According to the Winbond W25Q128JV datasheet, the Quad Page Program
  instruction (0x32) allows up to 256 bytes (one page) to be
  programmed at previously erased locations and requires the Quad
  Enable (QE) bit in status register 2 to be set and a prior Write
  Enable instruction【599804259612367†L3968-L3985】.  Sector Erase
  (0x20) erases 4‑KiB of flash, setting all bits within the sector to
  1 and also requires the Write Enable latch be set before the
  command【599804259612367†L4132-L4153】.  The Adafruit SPIFlash
  library abstracts these details and will automatically set QE and
  manage Write Enable for you, but it is still necessary to erase
  sectors before programming them.  This sketch therefore erases each
  affected 4‑KiB sector before writing data to it.  Only the required
  sectors (those covering the incoming binary) are erased to save
  time.

  Connections
  -----------
  The Feather M4 Express already includes a QSPI flash chip wired to
  dedicated pins that are not exposed on the breakout headers.  If
  you're using a DIP‑style W25Q128JV module or a replacement chip you
  must ensure it is connected to the board's QSPI pads.  The
  Adafruit_SPIFlash library uses the microcontroller's QSPI peripheral
  via the Adafruit_FlashTransport_QSPI class, so no pin numbers are
  required in this sketch.
*/

//#include <Arduino.h>
//#include <Adafruit_SPIFlash.h>
//#include <Adafruit_FlashTransport.h>
//// Use the built‑in QSPI peripheral on the SAMD51.  The default
//// constructor automatically selects the correct pins for the board.
//Adafruit_FlashTransport_QSPI flashTransport;
//Adafruit_SPIFlash flash(&flashTransport);


#include <SPI.h>
#include <Adafruit_SPIFlash.h>

// for flashTransport definition
#include "flash_config.h"

Adafruit_SPIFlash flash(&flashTransport);



// Constants for flash geometry
const uint32_t SECTOR_SIZE = 4096;  // 4 KiB sectors as per datasheet
const uint32_t PAGE_SIZE   = 256;   // 256 byte pages

// A small scratch buffer used for both programming and verification.
uint8_t pageBuffer[PAGE_SIZE];

void waitForSerial() {
  // Give the USB serial port time to open on a native USB board.
  Serial.begin(115200);
  while (!Serial) {
    // tiny delay to avoid hogging the CPU
    delay(10);
  }
}

// Read exactly len bytes from the Serial port into dst.  Blocks until all
// bytes have been received.  Returns true if successful, false if a
// timeout occurred (Serial.setTimeout controls the timeout period).
bool readExactBytes(uint8_t *dst, size_t len) {
  size_t received = 0;
  while (received < len) {
    int c = Serial.read();
    if (c < 0) {
      // no data available yet, yield and retry
      yield();
      continue;
    }
    dst[received++] = (uint8_t)c;
  }
  return true;
}

void setup() {
  waitForSerial();
  Serial.println("W25Q128JV Flash Programmer");

  // Initialise the flash chip
  if (!flash.begin()) {
    Serial.println("Error: failed to initialize flash chip.");
    while (true) {
      delay(1000);
    }
  }

  // Print JEDEC ID to confirm communication
  uint32_t jedec = flash.getJEDECID();
  Serial.print("Detected JEDEC ID: 0x");
  Serial.println(jedec, HEX);
  Serial.print(F("Flash size: ")); 
  Serial.print(flash.size() / 1024);
  Serial.println(F(" KB"));

  Serial.println();
  Serial.println("Please send a 32‑bit little‑endian address and length header followed by the binary data."); //added address
  Serial.println("For example: struct.pack('<I', addr + len(data)) + data");
  Serial.println("Waiting for length header...");

  // Increase the timeout for reading the header to 90 seconds.  If no
  // header arrives within this time the readBytes() call below will
  // return 0.
  Serial.setTimeout(144000000); //mod - a lot of time
  
  //mod_start - shadow1229 / original version cannot write longer than 4MiB file
  // Read 4 bytes little‑endian start address header
  uint8_t addrBytes[4];
  if (!readExactBytes(addrBytes, sizeof(addrBytes))) {
    Serial.println("Error: timeout while waiting for start address header.");
    while (true) {
      delay(1000);
    }
  }

  uint32_t startaddr = (uint32_t)addrBytes[0] |
                         ((uint32_t)addrBytes[1] << 8) |
                         ((uint32_t)addrBytes[2] << 16) |
                         ((uint32_t)addrBytes[3] << 24);
  //mod_end - shadow1229

  // Read 4 bytes little‑endian length header
  uint8_t lenBytes[4];
  if (!readExactBytes(lenBytes, sizeof(lenBytes))) {
    Serial.println("Error: timeout while waiting for length header.");
    while (true) {
      delay(1000);
    }
  }
  uint32_t dataLength = (uint32_t)lenBytes[0] |
                         ((uint32_t)lenBytes[1] << 8) |
                         ((uint32_t)lenBytes[2] << 16) |
                         ((uint32_t)lenBytes[3] << 24);
  Serial.print("Start Address: ");
  Serial.println(startaddr);
  Serial.print("Incoming binary size: ");
  Serial.print(dataLength);
  Serial.println(" bytes");

  // Calculate number of sectors needed.  We only erase sectors that
  // contain data.  Each sector is 4 KiB.
  uint32_t sectorStart = startaddr / SECTOR_SIZE; //startaddr could be divided by 2**22, so using like this is fine. // modified
  uint32_t sectorCount = (dataLength + (SECTOR_SIZE - 1)) / SECTOR_SIZE;
  Serial.print("Erasing ");
  Serial.print(sectorCount);
  Serial.println(" sector(s)...");
  for (uint32_t sector = sectorStart; sector < (sectorStart+sectorCount); sector++) {
    if (!flash.eraseSector(sector)) {
      Serial.print("Error erasing sector ");
      Serial.println(sector);
      while (true) {
        delay(1000);
      }
    }
    flash.waitUntilReady();
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Sector erase complete.");

  // Program the binary data page by page
  Serial.println("Programming flash...");
  uint32_t bytesRemaining = dataLength;
  uint32_t address        = startaddr; //modified (original: 0)
  while (bytesRemaining > 0) {
    // Determine how many bytes to read this iteration (max PAGE_SIZE)
    uint32_t chunkSize = bytesRemaining;
    if (chunkSize > PAGE_SIZE) {
      chunkSize = PAGE_SIZE;
    }
    // Read chunk from serial
    if (!readExactBytes(pageBuffer, chunkSize)) {
      Serial.println("Error: timeout while reading binary data from serial.");
      while (true) {
        delay(1000);
      }
    }
    // Pad the rest of the page with 0xFF (flash default erased value)
    if (chunkSize < PAGE_SIZE) {
      memset(pageBuffer + chunkSize, 0xFF, PAGE_SIZE - chunkSize);
    }
    // Write page to flash
    if (flash.writeBuffer(address, pageBuffer, PAGE_SIZE) == 0) {
      Serial.print("Error writing page at address 0x");
      Serial.println(address, HEX);
      while (true) {
        delay(1000);
      }
    }
    flash.waitUntilReady();
    address        += PAGE_SIZE;
    bytesRemaining -= chunkSize;
    if ((address / PAGE_SIZE) % 64 == 0) {
      // Print progress every 64 pages (16 KiB)
      Serial.print("Written ");
      Serial.print(address);
      Serial.println(" bytes");
    }
  }
  Serial.println("Programming complete.");

}

void loop() {
  // Nothing to do in loop
  delay(1000);
}