/* 
  RC522 Code Reference: https://github.com/miguelbalboa/RC522
  RC522 Documentation: https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf
*/

#include <Arduino.h>
#include "esp_system.h"
#include "RC522.h"


void writeByte(byte val) {                // should always begin just after falling edge
  int i = 7;
  digitalWrite(RC522_MOSI, (val & 1<<i) != 0);  // write bit i
  
  falling_RC522_SCLK_edge = 0;
  i--;
  while (i >= 0) {
    while (!falling_RC522_SCLK_edge) {}          // write on falling edge
    falling_RC522_SCLK_edge = 0;
    digitalWrite(RC522_MOSI, (val & 1<<i) != 0); // write bit i
    i--;
  }
  falling_RC522_SCLK_edge = 0;                   // wait until next falling edge to unblock
  while (!falling_RC522_SCLK_edge)  {}            
}

byte readByte() {                          // should always begin just after falling edge
  byte val = 0;
  int i = 7;
  rising_RC522_SCLK_edge = 0;

  while (i >= 0) {
    while (!rising_RC522_SCLK_edge) {}
    rising_RC522_SCLK_edge = 0;
    int reading = digitalRead(RC522_MISO);
    val |= reading<<i;
    i--;
  }
  falling_RC522_SCLK_edge = 0;                   // wait until next falling edge to unblock
  while (!falling_RC522_SCLK_edge) {}
  return val;
}

byte transferByte(byte writeVal) {         // should always begin just after falling edge
  byte readVal = 0;
  int read_index = 7;
  int write_index = 7;
  rising_RC522_SCLK_edge = 0;
  falling_RC522_SCLK_edge = 0;

  // write fiRC522_RST bit immediately as we're just after falling edge
  digitalWrite(RC522_MOSI, (writeVal & 1 << write_index) != 0);
  write_index--;

  // read and write seven times (seven rising and falling edges)
  while (write_index >= 0)
  {
    while (!rising_RC522_SCLK_edge) {}
    rising_RC522_SCLK_edge = 0;
    byte reading = digitalRead(RC522_MISO);
    readVal |= reading << read_index;
    read_index--;

    while (!falling_RC522_SCLK_edge) {}
    falling_RC522_SCLK_edge = 0;
    digitalWrite(RC522_MOSI, (writeVal & 1 << write_index) != 0);
    write_index--;
  }

  // read one last time (rising edge)
  while (!rising_RC522_SCLK_edge) {}
  rising_RC522_SCLK_edge = 0;
  byte reading = digitalRead(RC522_MISO);
  readVal |= reading << read_index;
  read_index--;

  // block until next falling edge
  falling_RC522_SCLK_edge = 0; // wait until next falling edge to unblock
  while (!falling_RC522_SCLK_edge) {} 

  return readVal;
}

void RC522_writeReg(byte address, byte val) {
  timerWrite(timer, 0);
  RC522_SCLK_en = 1;
  digitalWrite(RC522_SS, LOW);
  writeByte((address << 1) & B01111110);   // MSB is 0 for writing, LSB is 0 (Documentation 8.1.2.3)
  writeByte(val);
  digitalWrite(RC522_SS, HIGH);
  RC522_SCLK_en = 0;
  delayMicroseconds(100);
}

byte RC522_readReg(byte address) {
  timerWrite(timer, 0);
  RC522_SCLK_en = 1;
  digitalWrite(RC522_SS, LOW);
  writeByte(((address << 1) | B10000000) & B11111110); // MSB is 1 for reading, LSB is 0 (Documentation 8.1.2.3)
  byte val = readByte();
  digitalWrite(RC522_SS, HIGH);
  RC522_SCLK_en = 0;
  delayMicroseconds(100);
  return val;
}

void RC522_writeRegs(byte address, int count, byte *values) {
  timerWrite(timer, 0);
  RC522_SCLK_en = 1;
  digitalWrite(RC522_SS, LOW);
  writeByte((address <<1 ) & B01111110); // MSB is 0 for writing, LSB is 0 (Documentation 8.1.2.3)
  for (int i = 0; i < count; i++) {
    writeByte(values[i]);
  }
  digitalWrite(RC522_SS, HIGH);
  RC522_SCLK_en = 0;
  delayMicroseconds(100);
}

void RC522_readRegs(byte address, int count, byte *values, byte rxAlign) {
  timerWrite(timer, 0);
  RC522_SCLK_en = 1;
  digitalWrite(RC522_SS, LOW);              
  count--;
  writeByte(((address << 1) | B10000000) & B11111110); // Send address, MSB is 1 for reading, LSB is 0 (Documentation 8.1.2.3)
  for (int i = 0; i < count; i++) {
    if (i == 0 && rxAlign) {
      byte mask = (B11111111 << rxAlign) & B11111111;
      byte val = transferByte(((address << 1) | B10000000) & B11111110);
      values[0] = (values[0] & ~mask) | (val & mask); 
    }
    else
      values[i] = transferByte(((address << 1) | B10000000) & B11111110); // Read n-1 bytes while sending same address each time
  }
  values[count] = readByte();          // read nth byte while we sdon't care what to send on last byte
  digitalWrite(RC522_SS, HIGH);
  RC522_SCLK_en = 0;
  delayMicroseconds(100);
}

void RC522_softReset() {
  RC522_writeReg(CommandReg, RC522_SoftReset);
  do 
    delay(10);
  while (RC522_readReg(CommandReg) & (1 << 4)); // Loop until bit 4 is reset
}

void RC522_selfTest() {
  // Documentation 16.1.1

  // 1. Soft Reset
  RC522_softReset();

  // 2. Clear internal buffer: write 25 bytes of 0x0 and implement config command
  byte zeros[25] = {0x0};
  RC522_writeReg(FIFOLevelReg, B10000000);   // Clearing FIFO by setting FlushBuffer bit 7
  RC522_writeRegs(FIFODataReg, 25, zeros);   // Write 25 bytes of 0x0 to FIFO
  RC522_writeReg(CommandReg, RC522_Mem);      // Store 25 bytes from FIFO to internal buffer (Documentation 10.3.1.2)

  // 3. Enable self test
  RC522_writeReg(AutoTestReg, B00001001);

  // 4. Write 0x0 to FIFO buffer
  RC522_writeReg(FIFODataReg, 0);

  // 5. Start self test with CalcCRC Command
  RC522_writeReg(CommandReg, RC522_CalcCRC);
  delay(500);
  // 6. Wait until self test is complete
  while (RC522_readReg(FIFOLevelReg) < 63) {} // Check how many bytes are in FIFO buffer
  RC522_writeReg(CommandReg, RC522_Idle);

  // 7. Read FIFO buffer bytes
  byte result[64];
  for (int i = 0; i <64; i++)
    RC522_readReg(i);
  RC522_readRegs(FIFODataReg, 64, result, 0); 
  for (int i = 0; i < 64; i++)
    Serial.println(result[i]);
}

void RC522_init() {
  // RFID Reader
  pinMode(RC522_RST, OUTPUT);
  pinMode(RC522_IRQ, INPUT);
  pinMode(RC522_MISO, INPUT);
  pinMode(RC522_MOSI, OUTPUT);
  pinMode(RC522_SCLK, OUTPUT);
  pinMode(RC522_SS, OUTPUT);
  // RFID Reader
  digitalWrite(RC522_RST, HIGH);    // Active High enabled, reset on low
  digitalWrite(RC522_MOSI, LOW);
  digitalWrite(RC522_SCLK, LOW);
  digitalWrite(RC522_SS, HIGH);    // Active Low Chip Select
  // RFID reader software SPI timer init
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &intRoutine, true);
  timerAlarmWrite(timer, RC522_SCLK_interrupt_counter, true);
  timerAlarmEnable(timer);

  // reset timer states
  RC522_SCLK_state = 0;
  rising_RC522_SCLK_edge = 0;
  falling_RC522_SCLK_edge = 0;
  RC522_SCLK_en = 0; 
  // hard reset
  digitalWrite(RC522_RST, LOW);
  delay(1);
  digitalWrite(RC522_RST, HIGH);
  delay(50);

  // Reset baud rates
  RC522_writeReg(TxModeReg, 0);
  RC522_writeReg(RxModeReg, 0);

  // Reset ModWidthReg
  RC522_writeReg(ModWidthReg, 0x26); // Default value

  // Create timeout timer for communicating with device
  RC522_writeReg(TModeReg, B10000000); // Set TAuto bit to start timer automatically after all transmiRC522_SSions
  RC522_writeReg(TPrescalerReg, B10101001); // 169 for a timer frequnecy of 40kHz, period=25us (Documentation 8.5)
  RC522_writeReg(TReloadReg_hi, B00000011);
  RC522_writeReg(TReloadReg_lo, B11101000); // 00000011 11101000 => 1000decimal, for a timeout period of 25ms

  // RF Settings
  RC522_writeReg(TxASKReg, B01000000); // Set bit 7 to force a 100% ASK modulation independent of the ModGsPReg register setting
  RC522_writeReg(ModeReg, B00111101); // Setting lower two bits to 01 for 0x6363 preset value for CRC coporceRC522_SSor for CalcCRC command

  // Turn antenna on
  byte val = RC522_readReg(TxControlReg);
  RC522_writeReg(TxControlReg, val | B00000011); // Setting Tx2RFEn and Tx1RFEn
}

bool RC522_isNewCardPresent() {
  byte bufferATQA[2];                    // buffer to store answer to request
  byte bufferSize = sizeof(bufferATQA);

  // Reset baud rates
  RC522_writeReg(TxModeReg, 0x0);
  RC522_writeReg(RxModeReg, 0x0);
  // Reset ModWidthReg
  RC522_writeReg(ModWidthReg, 0x26);

  StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);
  return (result == STATUS_OK || result == STATUS_COLLISION);
}

StatusCode PICC_RequestA(byte *bufferATQA, byte *bufferSize) {
  return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
}

StatusCode PICC_REQA_or_WUPA(byte command, byte *bufferATQA, byte *bufferSize) {
  byte validBits;
  StatusCode status;

  if (bufferATQA == nullptr || *bufferSize < 2) {
    return STATUS_NO_ROOM;
  }
  // All received bits will be cleared after a collision (Documentation 9.3.1.15)
  byte tmpReg = RC522_readReg(CollReg);
  tmpReg = tmpReg & B01111111;
  RC522_writeReg(CollReg, tmpReg);

  // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
  validBits = 7;
  status = RC522_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits);

  if (status != STATUS_OK)
    return status;
  if (*bufferSize != 2 || validBits != 0)  // ATQA must be exactly 16 bits.
    return STATUS_ERROR;
  return STATUS_OK;
}

StatusCode RC522_TransceiveData(	
                          byte *sendData,		
                          // Pointer to the data to transfer to the FIFO.
													byte sendLen,		  // Number of bytes to transfer to the FIFO.
													byte *backData,		// nullptr or pointer to buffer if data should be read back after executing the command.
													byte *backLen,		// In: Max number of bytes to write to *backData. Out: The number of bytes returned.
													byte *validBits,	// In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
													byte rxAlign,		  // In: Defines the bit position in backData[0] for the fiRC522_RST bit received. Default 0.
													bool checkCRC     // In: True => The last two bytes of the response is aRC522_SSumed to be a CRC_A that must be validated.
								 ) {
  byte waitRC522_IRQ = 0x30; // RxRC522_IRQ and IdleRC522_IRQ
  return RC522_CommunicateWithPICC(RC522_Transceive, waitRC522_IRQ, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}

StatusCode RC522_CommunicateWithPICC(	
                            byte command,		 // The command to execute. One of the PCD_Command enums.
														byte waitRC522_IRQ,		 // The bits in the ComRC522_IRQReg register that signals succeRC522_SSful completion of the command.
														byte *sendData,  // Pointer to the data to transfer to the FIFO.
														byte sendLen,	   // Number of bytes to transfer to the FIFO.
														byte *backData,  // nullptr or pointer to buffer if data should be read back after executing the command.
														byte *backLen,	 // In: Max number of bytes to write to *backData. Out: The number of bytes returned.
														byte *validBits, // In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
														byte rxAlign,		 // In: Defines the bit position in backData[0] for the fiRC522_RST bit received. Default 0.
														bool checkCRC		 // In: True => The last two bytes of the response is aRC522_SSumed to be a CRC_A that must be validated.
									 ) {
  // Prepare values for BitFraminReg
  byte txLastBits = validBits? *validBits : 0;
  byte bitFraming = (rxAlign << 4) + txLastBits; // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

  RC522_writeReg(CommandReg, RC522_Idle);           // Stop any current commands
  RC522_writeReg(ComIrqReg, B01111111);            // Clear all seven interrupt request bits
  RC522_writeReg(FIFOLevelReg, B10000000);         // Set FlushBuffer to initialize FIFO
  RC522_writeRegs(FIFODataReg, sendLen, sendData); // Write sendData to FIFO buffer
  RC522_writeReg(BitFramingReg, bitFraming);       // Adjust bit framing
  RC522_writeReg(CommandReg, command);             // Send command
  if (command == RC522_Transceive) {               // Set StartSend bit to start the transmiRC522_SSion of data
    byte val = RC522_readReg(BitFramingReg);
    val = val | B10000000;
    RC522_writeReg(BitFramingReg, val);
  }

  // Wait for command to complete. In init, TAuto flag is set in TModeReg which automatically starts when the PCD stops transmitting.
  long startTime = millis();
  while(1) {
    byte val = RC522_readReg(ComIrqReg);
    if (val & waitRC522_IRQ)       // One of the interrupts that signal succeRC522_SS has been set.
      break;
    if (val & B00000001);    // Timer interrupt - nothing received in 25ms
      return STATUS_TIMEOUT;
    if (millis() - startTime > 100)
      return STATUS_TIMEOUT; // Nothing happened after 100ms, comms might not be working
  }

  // Check for errors
  byte errorRegVal = RC522_readReg(ErrorReg);
  if (errorRegVal & B00010011) // Checking for BufferOvfl, ParityErr, or ProtocolErr
    return STATUS_ERROR;

  byte _validBits = 0;
  
  // Collect return data if needed
  if (backData && backLen) {
    byte n = RC522_readReg(FIFOLevelReg); 
    if (n > *backLen) 
      return STATUS_NO_ROOM;
    *backLen = n;
    RC522_readRegs(FIFODataReg, n, backData, rxAlign);  // Get return data from FIFO buffer
    _validBits = RC522_readReg(ControlReg) & B00000111; // Find how many valid bits in last byte, B000 means all are valid
    if (validBits)                                     // If not all bits are valid, update how many are valid for caller function
      *validBits = _validBits;
  }

  // Check for collision
  if (errorRegVal & B00001000)
    return STATUS_COLLISION;

  // Validate CRC_A if needed
  if (backData && backLen && checkCRC) {
    // In this case a MIFARE ClaRC522_SSic NAK is not OK.
    if (*backLen == 1 && _validBits == 4) 
      return STATUS_MIFARE_NACK;
    // We need at least the CRC_A value and all 8 bits of the last byte must be received.
    if (*backLen < 2 || _validBits != 0) 
      return STATUS_CRC_WRONG;
    // Verify CRC_A - do our own calculation and store the control in controlBuffer.
    byte controlBuffer[2];
    StatusCode status = RC522_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
  }

  return STATUS_OK;
}

bool PICC_getUID(byte *uid_bytes) {
  byte validBits = 0;
  bool uidComplete;
  bool useCascadeTag;
  byte cascadeLevel = 1;
  StatusCode result;
  byte count;
  byte checkBit;
  byte index;
  byte uidIndex;             // The fiRC522_RST index in uid_bytes that is used in the current Cascade Level
  int currentLevelKnownBits; // The number of known UID bits in the current Cascade Level
  byte buffer[9];            // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
  byte bufferUsed;           // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO
  byte rxAlign;              // Used in BitFramingReg. Defines the bit position for the fiRC522_RST bit received
  byte txLastBits;           // Used in BitFramingReg. The number of valid bits in the last transmitted byte
  byte *responseBuffer;
  byte responseLength;

  if (validBits > 80)
    return false;

  // Reset ValuesAfterColl so all received bits will be cleared after a collision
  byte val = RC522_readReg(CollReg);
  val = val & B01111111;  
  RC522_writeReg(CollReg, val);

  // Repeat Cascade Level loop until there is a complete UID
  uidComplete = false;
  while (!uidComplete) {
    // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && 0;	// When we know that the UID has more than 4 bytes
				break;
			
			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && 0;	// When we know that the UID has more than 7 bytes
				break;
			
			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;
			
			default:
				return false;
				break;
		}

    // How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) 
			currentLevelKnownBits = 0;

    // Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag)
			buffer[index++] = PICC_CMD_CT;
    byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
    if (bytesToCopy) {
			byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid_bytes[uidIndex + count];
			}
		}

    // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag)
			currentLevelKnownBits += 8;

    // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		while (1) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = RC522_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) 
					return false;
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}
			
			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			RC522_writeReg(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			
			// Transmit the buffer and receive the response.
			result = RC522_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				byte valueOfCollReg = RC522_readReg(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return false; // Without a valid collision position we cannot continue
				}
				byte collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progreRC522_SS - should not happen 
					return false;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // FiRC522_RST byte is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != STATUS_OK) {
				return false;
			}
			else { // STATUS_OK
        uid_bytes[0] = buffer[2];
        uid_bytes[1] = buffer[3];
        uid_bytes[2] = buffer[4];
        uid_bytes[3] = buffer[5];
        return true;
			}
		} 
  }
  return false;
}

StatusCode RC522_CalculateCRC(byte *data, byte length, byte *result) {
  RC522_writeReg(CommandReg, RC522_Idle);      // Stop current command
  RC522_writeReg(DivIrqReg, B00000100);       // Reset CRCRC522_IRQ interrupt request bit
  RC522_writeReg(FIFOLevelReg, B10000000);    // Set FlushBuffer bit to clear FIFO buffer
  RC522_writeRegs(FIFODataReg, length, data); // Send data to FIFO buffer
  RC522_writeReg(CommandReg, RC522_CalcCRC);   // Begin CRC calculation

  // Wait for CRC calculation to finish
  long startTime = millis();
  while(1) {
    byte val = RC522_readReg(DivIrqReg);
    // If CalcCRC command is active and all data is proceRC522_SSed (Calculation complete) (Documentation 9.3.1.6)
    if (val & B00000100) {
      RC522_writeReg(CommandReg, RC522_Idle);   // Stop calculating CRC for anything new in the FIFO buffer
      result[0] = RC522_readReg(CRCResultReg_lo);
      result[1] = RC522_readReg(CRCResultReg_hi);
      return STATUS_OK;
    }
    if (millis() - startTime > 100)
      return STATUS_TIMEOUT;
  }
}