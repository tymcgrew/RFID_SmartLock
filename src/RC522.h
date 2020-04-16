/* 
  RC522 Code Reference: https://github.com/miguelbalboa/RC522
  RC522 Documentation: https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf
*/

#include <Arduino.h>

#define RC522_RST  23  // Hard Reset (Active High)
#define RC522_IRQ  22  // Interrupt Request
#define RC522_MISO 21  // Master-in, Slave-out
#define RC522_MOSI 19  // Master-out, Slave-in
#define RC522_SCLK 18  // SPI Clock
#define RC522_SS   5   // Chip Select (Active Low)

#define RC522_SCLK_freq 10000  // SPI Clock Freq (Hz)
#define RC522_SCLK_interrupt_counter (1000000/RC522_SCLK_freq)/2

extern volatile byte RC522_SCLK_state;
extern volatile byte rising_RC522_SCLK_edge;
extern volatile byte falling_RC522_SCLK_edge;
extern volatile byte RC522_SCLK_en;
extern hw_timer_t *timer;


/* Registers */
// Page 0: Command and Status
const byte CommandReg = 0x1;
const byte ComlEnReg = 0x2;
const byte DivlEnReg = 0x3;
const byte ComIrqReg = 0x4;
const byte DivIrqReg = 0x5;
const byte ErrorReg = 0x6;
const byte Status1Reg = 0x7;
const byte Status2Reg = 0x8;
const byte FIFODataReg = 0x9;
const byte FIFOLevelReg = 0xA;
const byte WaterLevelReg = 0xB;
const byte ControlReg = 0xC;
const byte BitFramingReg = 0xD;
const byte CollReg = 0xE;
// Page 1: Command
const byte ModeReg = 0x11;
const byte TxModeReg = 0x12;
const byte RxModeReg = 0x13;
const byte TxControlReg = 0x14;
const byte TxASKReg = 0x15;
const byte TxSelReg = 0x16;
const byte RxSelReg = 0x17;
const byte RxThresholdReg = 0x18;
const byte DemodReg = 0x19;
const byte MfTxReg = 0x1C;
const byte MfRxReg = 0x1D;
const byte SerialSpeedReg = 0x1F;
// Page 2: Configuration
const byte CRCResultReg_hi = 0x21;
const byte CRCResultReg_lo = 0x22;
const byte ModWidthReg = 0x24;
const byte RFCfgReg = 0x26;
const byte GsNReg = 0x27;
const byte CWGsPReg = 0x28;
const byte ModGsPReg = 0x29;
const byte TModeReg = 0x2A;
const byte TPrescalerReg = 0x2B;
const byte TReloadReg_hi = 0x2C;
const byte TReloadReg_lo = 0x2D;
const byte TCounterValReg_hi = 0x2E;
const byte TCounterValReg_lo = 0x2F;
// Page 3: Test register:
const byte TestSel1Reg = 0X31;
const byte TestSel2Reg = 0X32;
const byte TestPinEnReg = 0X33;
const byte TestPinValueReg = 0X34;
const byte TestBusReg = 0X35;
const byte AutoTestReg = 0X36;
const byte VersionReg = 0X37;
const byte AnalogTestReg = 0X38;
const byte TestDAC1Reg = 0X39;
const byte TestDAC2Reg = 0X3A;
const byte TestADCReg = 0X3B;


/* Commands */
const byte RC522_Idle = 0x0;
const byte RC522_Mem = 0x1;
const byte RC522_GenerateRandomID = 0x2;
const byte RC522_CalcCRC = 0x3;
const byte RC522_Transmit = 0x4;
const byte RC522_NoCmdChange = 0x7;
const byte RC522_Receive = 0x8;
const byte RC522_Transceive = 0xC;
const byte RC522_MFAuthent = 0xE;
const byte RC522_SoftReset = 0xF;


/* Return Status Codes */
enum StatusCode : byte {
    STATUS_OK				,	// Success
    STATUS_ERROR			,	// Error in communication
    STATUS_COLLISION		,	// Collission detected
    STATUS_TIMEOUT			,	// Timeout in communication
    STATUS_NO_ROOM			,	// A buffer is not big enough
    STATUS_INTERNAL_ERROR	,	// Internal error in the code
    STATUS_INVALID			,	// Invalid argument
    STATUS_CRC_WRONG		,	// The CRC_A does not match
    STATUS_MIFARE_NACK		= 0xff	// A MIFARE PICC responded with NAK.
};


/* Commands sent to the PICC. */
enum PICC_Command : byte {
    // The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
    PICC_CMD_REQA			= 0x26,		// REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
    PICC_CMD_WUPA			= 0x52,		// Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
    PICC_CMD_CT				= 0x88,		// Cascade Tag. Not really a command, but used during anti collision.
    PICC_CMD_SEL_CL1		= 0x93,		// Anti collision/Select, Cascade Level 1
    PICC_CMD_SEL_CL2		= 0x95,		// Anti collision/Select, Cascade Level 2
    PICC_CMD_SEL_CL3		= 0x97,		// Anti collision/Select, Cascade Level 3
    PICC_CMD_HLTA			= 0x50,		// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
    PICC_CMD_RATS           = 0xE0,     // Request command for Answer To Reset.
};

void IRAM_ATTR intRoutine();
void writeByte(byte val);
byte readByte();
byte transferByte(byte writeVal);
void RC522_writeReg(byte address, byte val);
byte RC522_readReg(byte address);
void RC522_writeRegs(byte address, int count, byte *values);
void RC522_readRegs(byte address, int count, byte *values, byte rxAlign);
void RC522_softReset();
void RC522_selfTest();
void RC522_init();
bool RC522_isNewCardPresent();
bool PICC_getUID(byte *uid);
StatusCode RC522_CalculateCRC(byte *data, byte length, byte *result);
StatusCode RC522_TransceiveData(byte *sendData, byte sendLen, byte *backData, byte *backLen, byte *validBits = nullptr, byte rxAlign = 0, bool checkCRC = false);
StatusCode RC522_CommunicateWithPICC(byte command, byte waitIRq, byte *sendData, byte sendLen, byte *backData = nullptr, byte *backLen = nullptr, byte *validBits = nullptr, byte rxAlign = 0, bool checkCRC = false);
StatusCode PICC_RequestA(byte *bufferATQA, byte *bufferSize);
StatusCode PICC_REQA_or_WUPA(byte command, byte *bufferATQA, byte *bufferSize);