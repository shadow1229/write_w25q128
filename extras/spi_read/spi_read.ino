/*
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

#include <Arduino.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_FlashTransport.h>

// Use the built‑in QSPI peripheral on the SAMD51.  The default
// constructor automatically selects the correct pins for the board.
Adafruit_FlashTransport_QSPI flashTransport;
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
  // Initialize serial port and wait for it to open before continuing.
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  Serial.println("Adafruit SPI Flash Total Erase Example");

  // Initialize flash library and check its chip ID.
  if (!flash.begin()) {
    Serial.println("Error, failed to initialize flash chip!");
    while (1) {
    }
  }
  Serial.print("Flash chip JEDEC ID: 0x");
  Serial.println(flash.getJEDECID(), HEX);
  Serial.print(F("Flash size: ")); // why 2048 KB??????? TODO
  Serial.print(flash.size() / 1024);
  Serial.println(F(" KB"));


  // Wait for user to send OK to continue.
  // Increase timeout to print message less frequently.
  Serial.setTimeout(144000000);
  do {
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                   "!!!!!!!!!!");
    Serial.println("This sketch will READ ALL DATA on the flash chip!");
    Serial.println("Type OK (all caps) and press enter to continue.");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                   "!!!!!!!!!!");
  } while (!Serial.find((char *)"OK"));


  // Verify written data - human written
  uint32_t bytesRemaining_read = 16777216; //128Mbit
  //uint32_t bytesRemaining_read = 1024*1024; //1Kbyte
  uint32_t address_read        = 0;
  uint32_t idx;
  while (bytesRemaining_read > 0) {

    // read page
    Serial.println("#new page");
    flash.readBuffer(address_read, pageBuffer, PAGE_SIZE);
    bytesRemaining_read -= PAGE_SIZE;
    address_read += PAGE_SIZE; //byte address

    for (idx = 0; idx < PAGE_SIZE; idx++) {
      Serial.print(pageBuffer[idx]);
      Serial.print(" ");
      if ((idx) % 16 == 15) {
        Serial.println(" ");
      }
    }
  }


}

void loop() {
  // Nothing to do in loop
  delay(1000);
}