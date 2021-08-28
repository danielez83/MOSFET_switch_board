/* Shift Register MOSFET controller with Arduino
The Arduino board is connected to a series of 16-bit shift registers.
The 16 bit shift register is built with 2 SN74HC595D in daisy chain.
We will call a 16 bit shift register a "unit". 
Arduino is capable of driving up to 256 units: the Master unit, where the Arduino
is installed and 255 addidional slave units of 16 bit shift registers.

Shift registers are driven via SPI with the following connections:
  - Master Reclear (Pin 10)         on Arduino D6. Active low, this sets all the bits in the shift 
                                    register to 0 or off if pulled LOW. [Shared to all units]
  - Latch (Pin 12)                  on Arduino D8. When pulled HIGH, it outputs the new shift 
                                    register values. [Shared to all units]
  - Data In (Pin 14)                on Arduino D11 (SPI MOSI). This is the input pin for the new 
                                    serial data.
  - Shift Register Clock  (Pin 11)  on Arduino D13 (SPI SCK). If pulled HIGH, this shifts all the 
                                    values in the shift register forward one.


Set new board state. Send by serial port: "bxxxpyyyyy\n", where
- xxx is the board number (0 - 255, 1 byte)
- yyyyy is the port state (0 - 65535, 2 byte)
eg.
>b000p00436 whill turn to high the 3rd, 5th, 6th, 8th, 9th port of board #0
>b001p00009 whill turn to high the 1st and 4th port of board #1


LOG:
now you can send by serial port the board state command.
nex step: implementing EEPROM to save state                                    
*/

// Includes
#include <SPI.h>

// Constants
const char MClearPin = 6;
const char LatchPin  = 8;

char NumBoards = 2;
char BoardState[2*256];   // Max number of board is 256
char board[4], state[6];

bool NewState; 

void setup() {
  // Set output ports
  pinMode(MClearPin, OUTPUT);
  pinMode(LatchPin, OUTPUT);
  
  // Start SPI in MODE0
  // SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.begin();

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

void loop() {
  // Test
  //BoardState[0] = 0b00000001; // MSB
  //BoardState[1] = 0b00000010; // LS

  while (1) {
    digitalWrite(MClearPin, HIGH);      // Ensure master clear is HIGH
    digitalWrite(LatchPin, LOW);        // Ensure latch is LOW
    // Read command from serial port ----------------------------------------------
    while (Serial.available() > 0) {
      String ReadBuff = Serial.readString();
      switch(ReadBuff[0]){
              case 'b':
                //char board[4];
                memcpy(board, &ReadBuff[1], 3); // Copy part of the string after b
                board[3] = '\0';                // String terminator
                //char state[6];
                memcpy(state, &ReadBuff[5], 5);
                state[5] = '\0';                // String terminator
                NewState = 1;                   // Set Flag
              break;
      }
      // Serial.println(atoi(board)); // Send back values
      // Serial.println(atoi(state)); // Send back values
    }

    // New state for board -------------------
    if (NewState == 1){
      NewState = 0;                           // Clear flag
    digitalWrite(LatchPin, LOW);        // Ensure latch is LOW
      // Edit board state registers
      int StateTmp = atoi(state);
      int BoardTmp = atoi(board);
      BoardState[BoardTmp * 2] = StateTmp << 8;   //MSB
      BoardState[BoardTmp * 2 + 1] = StateTmp;    //LSB
      // Clear Cycle
      for (char j = 0; j < NumBoards; j++){
        SPI.transfer16(0);
      }
      delay(1);
      // Write Cycle
      for (char j = NumBoards; j >= 0; j = j - 2){
        int BoardValue = BoardState[j] << 8 | BoardState[j + 1];
        //Serial.println(printf("%d\n", BoardValue));
        SPI.transfer16(BoardValue);
      }
      digitalWrite(LatchPin, HIGH);        // Change state of the board
      delay(1);
    }
    // End of new state for board -------------------
  } 
}
