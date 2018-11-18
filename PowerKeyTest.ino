// Test routines for the PowerKey pad
// Jeff Cooke Nov 2018

/*
 * Todos
 * - While 'Charging' - adjust the PWM 1hz rate of the Charger red LED to indicate state of charge
 * - While 'Park' - if the handbrake is not on, flash the Park button
 * - Only activate Cruise Adjust (white) the Up and Down buttons when in Drive/Reverse and MPH > 0
 * - Monitor ande detect Cruise setting/clearing (up or down held down for 2 full secs
*/

#include <mcp2515.h>
#include <SPI.h>

#define debug 1

// a=target variable, b=bit number to act upon 0-n
#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1ULL<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1ULL<<(b))))        // '!!' to make sure this returns 0 or 1

// Set up CS on chip
MCP2515 mcp2515(10);

// Create statuses
enum PadState {
  Drive = 0,
  Charge = 1,
  Reverse = 3,
  Park = 4,
  FinCharge = 5 
};

// Create Events
enum PadEvent {
  Dbutton = 0,
  Cbutton = 1,
  Upbutton = 3,
  Rbutton = 4,
  Pbutton = 5,
  Dwbutton = 6, 
  Fcharge = 7
};

// Foot Brake States
enum FootBrake {
  BrakeOn,
  BrakeOff,
  BrakeUnk
};

// Handbrake States
enum HandBrake {
  HandOn,
  HandOff
};

// Roadspeed Status
enum RoadSpeed {
  Moving,
  Stopped  
};

#define BUTTON_EVENT_STATES 12

// Define the State Change table
int StateTable [BUTTON_EVENT_STATES][3] = {
{Park, Dbutton,  Drive},
{Park, Rbutton,  Reverse},
{Drive, Pbutton, Park},
{Reverse, Pbutton,  Park},
{Drive, Rbutton,  Reverse},
{Reverse, Dbutton,  Drive},
{Park, Cbutton,  Charge},
{Charge, Cbutton, Park},
{FinCharge, Cbutton, Park},
{Charge, Pbutton, Park},
{FinCharge, Pbutton, Park},
{Charge, Fcharge, FinCharge}
};

enum ButtonColor {
  Black,
  Blue,
  Green,
  Cyan,
  Red,
  Magenta,
  Amber,
  White
};

#define BUTTON_ACTIONS 5

// Define the Actions associated with a new state
int ActionTable [BUTTON_ACTIONS][7] = {
{Drive,Blue,Black,White,White,White,White},
{Reverse,White,Black,White,Blue,White,White},
{Park,White,Black,Black,White,Blue,Black},
{Charge,Black,Red,Black,Black,Blue,Black},
{FinCharge,White,Red,Black,White,Blue,Black}
};

// Define color table
uint8_t Colors[8][3] = 
{
{0,0,0},  // 0 Black - Off
{0,0,1},  // 1 Blue
{0,1,0},  // 2 Green
{0,1,1},  // 3 Cyan ?
{1,0,0},  // 4 Red
{1,0,1},  // 5 Magenta
{1,1,0},  // 6 Amber
{1,1,1}   // 7 White
};



// Set up CAN frames
struct can_frame COpreop;
struct can_frame COstart;
struct can_frame COreadkey;
struct can_frame COledon;
struct can_frame CObckon;

// Set up frame for incoming
struct can_frame canMsg;

// Array of button states
bool ButtonStates [6] = {false,false,false,false,false,false};
char ButtonCode [6] = {'D','C','^','R','P','v'};
PadEvent ButtonEvent [6] = {Dbutton,Cbutton,Upbutton,Rbutton,Pbutton,Dwbutton};

// Global Drive State Status Vars
enum PadState DrvState;       // Current driving mode
enum FootBrake BrkState;      // Current foot brake status
enum HandBrake HndState;      // Current handbrake status
int SpeedMPH;             // Current road speed

void setup() {

  // Set the Current Status variables
  DrvState = Park;
  BrkState = BrakeOn;
  HndState = HandOn;
  SpeedMPH = 0;

  // Populate the pre-op frame
  COstart.can_id = 0x00;
  COstart.can_dlc = 8;
  COstart.data[0] = 0x80;
  COstart.data[1] = 0x15;
  COstart.data[2] = 0x00;
  COstart.data[3] = 0x00;
  COstart.data[4] = 0x00;
  COstart.data[5] = 0x00;
  COstart.data[6] = 0x00;
  COstart.data[7] = 0x00;

  // Populate the startup frame
  COstart.can_id = 0x00;
  COstart.can_dlc = 8;
  COstart.data[0] = 0x01;
  COstart.data[1] = 0x15; // ID for the PowerKey Pro
  COstart.data[2] = 0x00;
  COstart.data[3] = 0x00;
  COstart.data[4] = 0x00;
  COstart.data[5] = 0x00;
  COstart.data[6] = 0x00;
  COstart.data[7] = 0x00;

  // Populate the readkey frame
  COreadkey.can_id = 0x615; // 600h and 15h for the keypad ID
  COreadkey.can_dlc = 8;
  COreadkey.data[0] = 0x40;
  COreadkey.data[1] = 0x00;
  COreadkey.data[2] = 0x20;
  COreadkey.data[3] = 0x01;
  COreadkey.data[4] = 0x00;
  COreadkey.data[5] = 0x00;
  COreadkey.data[6] = 0x00;
  COreadkey.data[7] = 0x00;

  // Populate the button LED frame
  COledon.can_id = 0x215; // Solid
//  COledon.can_id = 0x315; // Blinking
  COledon.can_dlc = 8;
  COledon.data[0] = 0x00;
  COledon.data[1] = 0x00;
  COledon.data[2] = 0x00;
  COledon.data[3] = 0x00;
  COledon.data[4] = 0x00;
  COledon.data[5] = 0x00;
  COledon.data[6] = 0x00;
  COledon.data[7] = 0x00;

  // Backlight brightness
  CObckon.can_id = 0x515;
  CObckon.can_dlc = 8;
  CObckon.data[0] = 0x01;
  CObckon.data[1] = 0x00;
  CObckon.data[2] = 0x00;
  CObckon.data[3] = 0x00;
  CObckon.data[4] = 0x00;
  CObckon.data[5] = 0x00;
  CObckon.data[6] = 0x00;
  CObckon.data[7] = 0x00;
  
  // init the serial port - for status/debug
  while (!Serial);
  Serial.begin(115200);

  // init the SPI communications
  SPI.begin();

  // Startup CAN
  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS);
  mcp2515.setNormalMode();
  Serial.println("CAN Initialized");

  // init the PowerKey
  //CAN0.sendMsgBuf(0x00, 0, 8, COstart); 
//  mcp2515.sendMessage(&COpreop); 
//  delay(500);
  mcp2515.sendMessage(&COstart); 
  delay(500);

  // Set the backlight
  mcp2515.sendMessage(&CObckon); 
  delay(100);

  // Reset the keypad display
  setButtonLEDstate(DrvState);
}

// Set or Clear bit 'b1' in the byte 'by' depending on value 'bval'
byte setclearBit(byte by, uint8_t b1, uint8_t bval)
{
  byte tempby = by;
  if (bval == 1)
  {BIT_SET(tempby,b1);}
  else
  {BIT_CLEAR(tempby,b1);}
  return tempby;
}

void setButtonColor(uint8_t bnum, ButtonColor bcol)
{
   COledon.data[0] = setclearBit(COledon.data[0],bnum,Colors[bcol][0]);
   COledon.data[1] = setclearBit(COledon.data[1],bnum,Colors[bcol][1]);
   COledon.data[2] = setclearBit(COledon.data[2],bnum,Colors[bcol][2]);
   mcp2515.sendMessage(&COledon);
}

void allButtonLEDoff()
{
   COledon.data[0] = 0x00;
   COledon.data[1] = 0x00;
   COledon.data[2] = 0x00;
   mcp2515.sendMessage(&COledon);
}

void setButtonLEDstate(PadState state)
{
  // Find the new LED state and implement
  for (int i = 0; i < BUTTON_ACTIONS; i++)
  {
    if (ActionTable[i][0] == state)
    {
      #ifdef debug
      Serial.print("Button state ");
      Serial.print(state);
      #endif

      // Set/Clear the button LEDs and colors
      for (int j = 0; j < 6;j++)
      {
        #ifdef debug
        Serial.print(" [b: ");
        Serial.print(j);
        Serial.print(" c: ");
        Serial.print((ButtonColor)ActionTable[i][j+1]);
        Serial.print("]");
        #endif
        setButtonColor(j,(ButtonColor)ActionTable[i][j+1]);
        delay(1);   // KeyPad seems to need time to process the CAN frames if they come quickly
      }
      #ifdef debug
      Serial.println();
      #endif
    }
  }
}

void loop() {
  //mcp2515.sendMessage(&COreadkey);
  //Serial.println("readkey send");
  //delay(500);

  // Receive check
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {

    // Output the frame received
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    Serial.print(" ");
    
    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
      Serial.print(canMsg.data[i],HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Output an intepretation of the frame
    // ID: 0x195 DLC: 5 0x20 0x0 0x0 0x0 0x6 
    for (int b=0;b<6;b++)
    {
      bool butntest = BIT_CHECK(canMsg.data[0],b);
      if (butntest && !ButtonStates[b])
      {
         // Info on button pressed
         #ifdef debug
         Serial.print(ButtonCode[b]);
         Serial.println(" was pressed");
         #endif

         // Update the drive state, if necessary
          // Scan the State Table
          for (int s=0;s<BUTTON_EVENT_STATES;s++)
          {
            // Find the row in the state table where the conditions match
            if ((StateTable[s][0] == DrvState) && (StateTable[s][1] == ButtonEvent[b]))
            {
              // Capture the new DrvState
              DrvState = (PadState)StateTable[s][2];

              #ifdef debug
              Serial.print("from State : ");
              Serial.print(StateTable[s][0]);
              Serial.print(" to State : ");
              Serial.print(DrvState);
              Serial.print(" Event : ");
              Serial.println(ButtonEvent[b]);              
              #endif

              // Switch off all the lights
              setButtonLEDstate(DrvState);

              // be sure to exit the loop
              s = BUTTON_EVENT_STATES + 1;
            }
          }
      }
      if (!butntest && ButtonStates[b])
      {
        #ifdef debug
        Serial.print(ButtonCode[b]);
        Serial.println(" was released");
        #endif
      }
      ButtonStates[b] = butntest;
    }
  }
  else
  {
  //  Serial.println(" : no msg");
  }

}
