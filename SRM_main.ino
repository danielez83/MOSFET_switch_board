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
29/08/2021
short text menu included
board state saved into EEPROM
next step: read EEPROM state after power down then reset board state to previous state
28/08/2021
now you can send by serial port the board state command.
next step: implementing EEPROM to save state                                    

*/

// Includes
#include <SPI.h>
#include <EEPROM.h>

// Constants
const char  MClearPin       = 6;
const char  LatchPin        = 8;

// Functions
void HelpMessage(void);
void ReadEEPROM(int start, int stop);

// EEPROM ADDRESSES
int ADDRNumBoards, ADDRDispWelc, ADDRboardState;

// Default settings
uint8_t NumBoards;
// bool WelcMess  = true;


char BoardState[2*128];   // Max number of board is 128, 256Byte EEPROM used to store boards state
char board[4], state[6];
char StrBuff[16];

int StateTmp, BoardTmp;

bool NewState; 


void setup() {
  // EEPROM mapping  
  ADDRNumBoards   = 0x0;
  ADDRDispWelc    = 0xA;    // Display welcome/help message (h command)
  ADDRboardState  = 0x100;  // 256 decimal, FF+1. This is the address of BoardState[0]
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
  //--------- DEFAULT SETTINGS,  check if this a factory new Arduino 
  if(EEPROM.read(ADDRNumBoards) == 0xFF){
    Serial.println("First-time run:");
    EEPROM.write(ADDRNumBoards, 1); // Set boards number to 1
    Serial.println("boards number set to 1...done");
    Serial.println("Set board state to 0...");
    for(int j = ADDRboardState; j <= ADDRboardState + 0x1FF; j++){
        EEPROM.write(j, 0);
        switch(j){
          case 320:
            Serial.println("25%%...");
            break;
          case 384:
            Serial.println("50%%...");
            break;
          case 448:
            Serial.println("75%%...");
            break;
        }
    }
    Serial.println("100%%...done");   
   }

  // Read and set configuraion from EEPROM
  // Number of boards
  NumBoards = EEPROM.read(ADDRNumBoards);
  // Full boards state
  Serial.print("Restore last state (only EEPROM read)");
  /*for(int j = 0; j < 256; j++){
    BoardState[j] = EEPROM.read(ADDRboardState + j);
  }
  // Clear Cycle, send a sequence of zeros
  for (int j = 0; j <= 255; j++){
    SPI.transfer16(0);
  }
  delay(1);
  // Write Cycle, read BoardState array to set new state
  for (int j = 510; j >= 256; j = j - 2){
    //int BoardValue = BoardState[j] << 8 | BoardState[j + 1];
    int BoardValue = BoardState[j + 1] << 8 | BoardState[j];
    //Serial.println(printf("%d\n", BoardValue));
    SPI.transfer16(BoardValue);
  }*/
  Serial.println("...done!");  
  //ReadEEPROM(ADDRboardState, ADDRboardState+255);
   
  // ------------------ WELCOME MESSAGE
   if(EEPROM.read(ADDRDispWelc) > 0){
     Serial.println("Shift Register MOSFET controller");
     Serial.println("Created by Daniele Zannoni daniele.zannoni@uib.no");
     Serial.println("University of Bergen, Sept.2021");
     HelpMessage();
   }

  

  //  ------------------ MAIN LOOP
  while (1) {
    digitalWrite(MClearPin, HIGH);      // Ensure master clear is HIGH
    digitalWrite(LatchPin, LOW);        // Ensure latch is LOW
    // Read command from serial port ----------------------------------------------
    while (Serial.available() > 0) {
      String ReadBuff = Serial.readString();
      switch(ReadBuff[0]){
              case 'b': // ---------------------------------------- b command, SET NEW BOARD STATE
                //char board[4];
                memcpy(board, &ReadBuff[1], 3); // Copy part of the string after b
                board[3] = '\0';                // String terminator
                //char state[6];
                memcpy(state, &ReadBuff[5], 5);
                state[5] = '\0';                // String terminator
                NewState = 1;                   // Set Flag
                break;

              case 'h': // ---------------------------------------- h command, SHOW HELP
                HelpMessage();
                break;

              case 'r': // ---------------------------------------- b command, READ BOARD STATE
                memcpy(board, &ReadBuff[1], 3); // Copy part of the string after b
                board[3] = '\0';                // String terminator
                BoardTmp = atoi(board);
                if(BoardTmp < NumBoards){
                  StateTmp = BoardState[BoardTmp * 2] << 8 | BoardState[BoardTmp * 2 + 1];
                  Serial.print("Board ");
                  Serial.print(BoardTmp, DEC);
                  Serial.print(" state is: ");
                  Serial.println(StateTmp, DEC);
                }
                else{
                  Serial.print("Error, max boards number is ");
                  Serial.print(NumBoards, DEC);
                  Serial.println(" (zero-indexed)");
                }

                break;

              case 'n': // ---------------------------------------- h command, SET BOARDS NUMBER
                if(ReadBuff[1] == '\0' || ReadBuff[2] == '\0' || ReadBuff[3] == '\0'){
                  Serial.print("You have ");
                  Serial.print(NumBoards, DEC);
                  Serial.print(" boards\n");
                }
                else{
                  memcpy(StrBuff, &ReadBuff[1], 3); // Copy part of the string after n
                    StrBuff[4] = '\0';
                    if(atoi(StrBuff) < 129){
                      NumBoards = atoi(StrBuff);
                      EEPROM.update(ADDRNumBoards, NumBoards);
                      Serial.print("Boards number updated to ");
                      Serial.println(NumBoards, DEC);
                    }
                    else{
                      Serial.println("Max boards number allowed is 128");                      
                    }
                }
                break;
              
              case 'R': // ------------------------------------- R command, MCLR
                Serial.println("Resetting state of all boards...");
                digitalWrite(MClearPin, LOW);      // set MCLR LOW
                for(int j = ADDRboardState; j <= ADDRboardState + 0x1FF; j++){
                  EEPROM.update(j, 0);
                  switch(j){
                    case 320:
                      Serial.println("25%%...");
                      break;
                    case 384:
                      Serial.println("50%%...");
                      break;
                    case 448:
                      Serial.println("75%%...");
                      break;                      
                  }
                }
                Serial.println("100%%...done"); 
                digitalWrite(MClearPin, HIGH);
                digitalWrite(LatchPin, HIGH);       
                delay(1);
                digitalWrite(LatchPin, LOW);
                break;
      }
      // Serial.println(atoi(board)); // Send back values
      // Serial.println(atoi(state)); // Send back values
    }

    // New state for board -------------------
    if (NewState == 1){
      NewState = 0;                           // Clear flag
      digitalWrite(LatchPin, LOW);              // Ensure latch is LOW
      // Edit board state registers
      StateTmp = atoi(state);
      BoardTmp = atoi(board);
      BoardState[BoardTmp * 2] = StateTmp << 8;   //MSB
      BoardState[BoardTmp * 2 + 1] = StateTmp;    //LSB
      // Clear Cycle, send a sequence of zeros
      for (char j = 0; j <= NumBoards; j++){
        SPI.transfer16(0);
      }
      delay(1);
      // Write Cycle, read BoardState array to set new state
      for (char j = NumBoards; j >= 0; j = j - 2){
        //int BoardValue = BoardState[j] << 8 | BoardState[j + 1];
        int BoardValue = BoardState[j + 1] << 8 | BoardState[j];
        //Serial.println(printf("%d\n", BoardValue));
        SPI.transfer16(BoardValue);
      }
      digitalWrite(LatchPin, HIGH);        // Change state of the board
      delay(1);
      // Update state in EEPROM
      for(int j = 0; j < 256; j++){
        EEPROM.update(ADDRboardState + j, BoardState[j]);
      } 
    }
    // End of new state for board -------------------
  } 
}

void HelpMessage(){
  Serial.println("--------- Command menu ---------");
  Serial.println("h             shows this help");
  Serial.println("bxxxpyyyyy    set new board state:");
  Serial.println("              xxx, boards number (e.g. 001)");
  Serial.println("              yyyyy, boards state (e.g. 00256)");
  Serial.println("rxxx          read board xxx state (e.g. r001");
  Serial.println("n             set boards number (e.g. n001)");
  Serial.println("w             show welcome message 0/1 (e.g. w0)");
  Serial.println("R             RESET ALL BOARDS STATE (MCLR low)");
}

void ReadEEPROM(int start, int stop){
  // Read EEPRORM routine
  for(int j = start; j <= stop; j++){
    Serial.println(EEPROM.read(j), DEC);
  }
}

