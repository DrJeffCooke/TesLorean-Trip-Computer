// Test routines for the PowerKey pad
// Jeff Cooke Nov 2018

/*
 * DONE
 * - While 'Charging' - adjust the PWM 1hz rate of the Charger red LED to indicate state of charge
 *    cNNN - set charging level to NNN%
 * - While 'Park' - if the handbrake is not on, flash the Park button
 *    h1/0 handbrake on/off
 * - Switch from daytime (bright) to nighttime(dim) brightness settings
 *    d1/0 daytime/nighttime 
 * - When the Battery is fully charged, set the charge LED to green 
 *    mNNN - set the current speed to NNN 
 * - When in Park, only allow selection of R or D when FootBreak is pressed 
 *    f1/0 footbrake on/off
 * - Only allow R selection from D, or visa versa, when MPH < 2 mph 
 * - Only activate Cruise Adjust (white) the Up and Down buttons when in Drive and MPH > 10         
 * - Monitor and detect Cruise setting/clearing when up or down held down for 1 full secs 
 * 
 * TODOs
 * 
 * CHECKS
*/

#include <mcp2515.h>
#include <SPI.h>

#define debug 1

// a=target variable, b=bit number to act upon 0-n
#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1ULL<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1ULL<<(b))))        // '!!' to make sure this returns 0 or 1

// Unit Setups
#define NO_HANDBRAKE_FLASH_RATE 250     // ms on/off for no handbrake during Park or Charge
#define CHARGING_MIN_FLASH 100          // ms minimum flash time per second
#define ACTIVATED_DAYTIME_BRIGHTNESS 0x2F
#define BACKLIGHT_DAYTIME_BRIGHTNESS 0x01
#define ACTIVATED_NIGHTTIME_BRIGHTNESS 0x0F
#define BACKLIGHT_NIGHTTIME_BRIGHTNESS 0x01
#define BACKLIGHT_COLOR 0x07    // 01 Red, 02 Green, 03 Blue, 04 Yellow, 05 Cyan, 06 Violet, 07 White, 08 Amber, 09 YelGreen
#define CRUISE_TIMEHOLD_ACTIVATE 1000   // ms needed to hold up/down buttons to activate cruise control mode
#define CRUISE_MIN_SPEED 10     // Lowest speed at which cruise mode can be set

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
  BrakeOff
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
{Reverse,White,Black,Black,Blue,White,Black},
{Park,White,Black,Black,White,Blue,Black},
{Charge,Black,Red,Black,Black,Blue,Black},
{FinCharge,Black,Green,Black,Black,Blue,Black}
};

// Define color table
uint8_t Colors[8][3] = 
{
{0,0,0},  // 0 Black - Off
{0,0,1},  // 1 Blue
{0,1,0},  // 2 Green
{0,1,1},  // 3 Cyan
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
struct can_frame CObrigt;
struct can_frame CObcolr;

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
int chargePCT;            // Current charge percent (whole integers 0-100)
int speedMPH;             // Current road speed
long time_handbrake_flash;    // Timer setting when the Park button light was last toggled
bool parkLEDon;              // Flag to indicate if the Park LED is on or off
bool dayTimeLights;               // Flag to indicate if daytime or nighttime brightness settings
bool chargeLEDon;           // Flag to indicate if the charging light is on or off
long time_charger_flash_on;    // Timer since last On status for the charger light
long time_charger_flash_off;    // Timer since last Off status for the charger light
bool cruiseActive;            // Flag to indicate if currently in cruise mode
long time_cruise_activate;    // time since the start of a cruise activate press
int cruiseMPH;                // cruise speed MPH

void setup() {

  // Set the Current Status variables
  DrvState = Park;
  BrkState = BrakeOn;
  HndState = HandOn;
  chargePCT = 0;
  speedMPH = 0;
  time_handbrake_flash = 0;
  time_charger_flash_on = 0;
  time_charger_flash_off = 0;
  dayTimeLights = true;
  parkLEDon = false;
  chargeLEDon = false;
  cruiseActive = false;
  time_cruise_activate = 0;
  cruiseMPH = 0;

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

  // Lighting Settings
  CObrigt.can_id = 0x00;
  CObrigt.can_dlc = 8;
  CObrigt.data[0] = 0x00;
  CObrigt.data[1] = 0x00;
  CObrigt.data[2] = 0x00;
  CObrigt.data[3] = 0x00;
  CObrigt.data[4] = 0x00;
  CObrigt.data[5] = 0x00;
  CObrigt.data[6] = 0x00;
  CObrigt.data[7] = 0x00;

  // Background color
  CObcolr.can_id = 0x615;
  CObcolr.can_dlc = 8;
  CObcolr.data[0] = 0x2F;
  CObcolr.data[1] = 0x03;
  CObcolr.data[2] = 0x20;
  CObcolr.data[3] = 0x03;
  CObcolr.data[4] = BACKLIGHT_COLOR;
  CObcolr.data[5] = 0x00;
  CObcolr.data[6] = 0x00;
  CObcolr.data[7] = 0x00;

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

  // Set the background color
  mcp2515.sendMessage(&CObcolr);
  
  // Set the keypad brightness
  CObrigt.can_id = 0x515;
  CObrigt.data[0] = BACKLIGHT_DAYTIME_BRIGHTNESS;
  mcp2515.sendMessage(&CObrigt);
  delay(1);
  CObrigt.can_id = 0x415;
  CObrigt.data[0] = ACTIVATED_DAYTIME_BRIGHTNESS;
  mcp2515.sendMessage(&CObrigt);
  delay(1);

  // Initialize button states
  for (int b=0;b<6;b++){ButtonStates[b]=false;}

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

   // Extra check to set Park light related flags & timers
   if (bnum==4 and bcol!=Black)
   {
     parkLEDon = true;
     time_handbrake_flash = millis();
   }
   if (bnum==4 and bcol==Black)
   {
     parkLEDon = false;
     time_handbrake_flash = millis();
   }

   // Extra check to set Charge light related flags & timers
   if (bnum==1 and bcol !=Black)
   {
     chargeLEDon = true;
     time_charger_flash_on = millis();
   }
   if (bnum==1 and bcol==Black)
   {
     chargeLEDon = false;
     time_charger_flash_off = millis();
   }
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
        
        // Set the color of the button
        setButtonColor(j,(ButtonColor)ActionTable[i][j+1]);

        // Delay to allow KeyPad to catch up
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

  // Test the Serial input for test commands
  char inChar = ' ';
  int inNum = -1;
  if (Serial.available())
  {
    inChar = Serial.read(); // read the incoming byte:
    delay(1);
    if (Serial.available())
    {
      inNum = Serial.parseInt();

      switch (inChar)
      {
        case 'h':     // 'h' Handbrake
          if (inNum == 0)
          {HndState = HandOff;Serial.println("Handbrake OFF");}
          else
          {HndState = HandOn;Serial.println("Handbrake ON");}
          break;
          
        case 'f':     // 'f' Footbrake
          if (inNum == 0)
          {BrkState = BrakeOff;Serial.println("Footbrake OFF");}
          else
          {BrkState = BrakeOn;Serial.println("Footbrake ON");}
          break;
            
        case 'c':     // 'c' Charge state
          if (inNum < 0){inNum = 0;}
          if (inNum > 100){inNum = 100;}
          chargePCT = inNum;
          if (chargePCT == 100)
          {
            // Set the charge to finished
            DrvState = FinCharge;
            // Reset the keypad display
            setButtonLEDstate(DrvState);
          }
          Serial.print("Charge ");
          Serial.print(chargePCT,DEC);
          Serial.println("%");
          break;

        case 'm':     // 'm' Speed
          if (inNum < 0){inNum = 0;}
          if (inNum > 160){inNum = 160;}    // Whoa slow down there buddy!
          speedMPH = inNum;
          Serial.print("Speed ");
          Serial.print(speedMPH,DEC);
          Serial.println(" mph");
          break;
          
        case 'd':     // 'd' Daytime
          if (inNum == 0)
          {
              dayTimeLights = false;
              Serial.println("Nighttime");
          }
          else
          {
              dayTimeLights = true;
              Serial.println("Daytime");
          }
          // Set the appropriate brightnesses for the keys and background lights
          if (dayTimeLights)
          {
            CObrigt.can_id = 0x515;
            CObrigt.data[0] = BACKLIGHT_DAYTIME_BRIGHTNESS;
            mcp2515.sendMessage(&CObrigt);
            delay(1);
            CObrigt.can_id = 0x415;
            CObrigt.data[0] = ACTIVATED_DAYTIME_BRIGHTNESS;
            mcp2515.sendMessage(&CObrigt);
            delay(1);
          }
          else
          {
            CObrigt.can_id = 0x515;
            CObrigt.data[0] = BACKLIGHT_NIGHTTIME_BRIGHTNESS;
            mcp2515.sendMessage(&CObrigt);
            delay(1);
            CObrigt.can_id = 0x415;
            CObrigt.data[0] = ACTIVATED_NIGHTTIME_BRIGHTNESS;
            mcp2515.sendMessage(&CObrigt);
            delay(1);
          }
          break;
          
        default:
          // default is optional
          break;
      }
    }
  }  // end of Serial input check

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
    if (canMsg.can_id == 0x195)
    {
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

           // If Up or Down pressed, start the timer to detect if cruise mode is on or off
           if (b == 2 || b == 5)
           {
             // capture the time of the press 
             time_cruise_activate = millis();
             if (cruiseActive)
             {
                // Increase or Descrease the cruise speed
                if (b == 2) // UP
                {
                  cruiseMPH = cruiseMPH + 1;
                  setButtonColor(2,Blue);
                  delay(100);
                  setButtonColor(2,Cyan);
                }
                if (b == 5)  // DOWN
                {
                  cruiseMPH = cruiseMPH - 1;
                  setButtonColor(5,Blue);
                  delay(100);
                  setButtonColor(5,Cyan);
                }
                // Checks
                if (cruiseMPH < 0){cruiseMPH = 0;}
                if (cruiseMPH > 160){cruiseMPH = 160;}
                
                #ifdef debug
                Serial.print("New Cruise Speed ");
                Serial.print(cruiseMPH,DEC);
                Serial.println(" MPH");
                #endif
             }
           }
  
           // Update the drive state, if necessary
            // Scan the State Table
            for (int s=0;s<BUTTON_EVENT_STATES;s++)
            {
              // Keep prior DrvState
              PadState PriorState = DrvState;
              
              // Find the row in the state table where the conditions match
              if ((StateTable[s][0] == DrvState) && (StateTable[s][1] == ButtonEvent[b]))
              {
                // Capture the new DrvState
                DrvState = (PadState)StateTable[s][2];

                // Check for full charged, if so put in FinCharge mode
                if (DrvState == Charge && chargePCT == 100){DrvState = FinCharge;}

                // if in Park, check that the Footbrake is applied before allowing D or R
                if (PriorState == Park && (DrvState == Drive || DrvState == Reverse) && BrkState == BrakeOff)
                {
                  // Flash the Drive or Reverse button red and then restore to white
                  if (DrvState == Drive){setButtonColor(0,Red); delay(100); setButtonColor(0,White);}
                  if (DrvState == Reverse){setButtonColor(3,Red); delay(100); setButtonColor(3,White);}
                  
                  // Switch back into Park
                  DrvState = Park;
                }

                // if in Drive or Reverse, don't allow selection of the other unless MPH < 2
                if (PriorState == Drive && DrvState == Reverse && speedMPH > 2)
                {
                  setButtonColor(3,Red);
                  delay(100);
                  setButtonColor(3,White);
                  DrvState = Drive;
                }
                if (PriorState == Reverse && DrvState == Drive && speedMPH > 2)
                {
                  setButtonColor(0,Red);
                  delay(100);
                  setButtonColor(0,White);
                  DrvState = Reverse;
                }

                // Check for Changes that drop out of cruise
                if (cruiseActive && DrvState != Drive)
                {
                  cruiseActive = false;
                }
  
                #ifdef debug
                Serial.print("from State : ");
                Serial.print(StateTable[s][0]);
                Serial.print(" to State : ");
                Serial.print(DrvState);
                Serial.print(" Event : ");
                Serial.println(ButtonEvent[b]);              
                #endif
  
                // Update all the lights
                setButtonLEDstate(DrvState);
  
                // be sure to exit the loop
                s = BUTTON_EVENT_STATES + 1;
              }
            }
        }
        if (!butntest && ButtonStates[b])
        {
          // Test if Up or Down was released
          if (DrvState == Drive && (b == 2 || b == 5))
          {
            // Test if button was held down long enough
            if (time_cruise_activate!= 0 && (time_cruise_activate < millis() - CRUISE_TIMEHOLD_ACTIVATE))
            {
              if (speedMPH >= CRUISE_MIN_SPEED)
              {
                // Toggle Cruise mode and capture the current speed as the setting
                cruiseActive = !cruiseActive;
                cruiseMPH = speedMPH;
  
                // Set button colors to show cruise active
                if (cruiseActive)
                {
                  setButtonColor(2,Cyan);
                  setButtonColor(5,Cyan);
                }
                else
                {
                  setButtonColor(2,White);
                  setButtonColor(5,White);
                }
  
                #ifdef debug
                Serial.print("Cruise ");
                Serial.print(cruiseActive?"ON":"OFF");
                Serial.print(" : Speed ");
                Serial.println(cruiseMPH, DEC);
                #endif
              }
              else    // speed too low for cruise CRUISE_MIN_SPEED
              {
                // Flash red for 1/10 sec
                setButtonColor(2,Red);
                setButtonColor(5,Red);
                delay(100);
                setButtonColor(2,White);
                setButtonColor(5,White);
              }
            }
            // Reset the time cruise activate timer - will be restarted on the next Up/Down button press
            time_cruise_activate = 0;

          }  // end of up/down release
          
          #ifdef debug
          Serial.print(ButtonCode[b]);
          Serial.println(" was released");
          #endif
        }
        ButtonStates[b] = butntest;
      }
    }
  }     // End of CAN receive checks

  // Check HandBrake
  if ((DrvState == Park || DrvState == Charge || DrvState == FinCharge) && HndState == HandOff)
  {
    // Check delay and toggle as necessary
    if (time_handbrake_flash < (millis() - NO_HANDBRAKE_FLASH_RATE))
    {
      if (parkLEDon)
      {setButtonColor(4,Black);}
      else
      {setButtonColor(4,Blue);}
    }
  }
  // If the Handbrake restores while the Park LED is off (due to flashing) turn it back on
  if ((DrvState == Park || DrvState == Charge || DrvState == FinCharge) && HndState == HandOn)
  {
    if (!parkLEDon)
    {
      // Turn the Park LED back on 
      setButtonColor(4,Blue);
    }
  }   // End of Handbrake check

  // Check for charger flash timing
  if (DrvState == Charge && chargeLEDon)
  {
    // Calculate the % of the charger light on-time
    float chargePCTf = chargePCT;
    chargePCTf = (chargePCTf/100 * (1000 - CHARGING_MIN_FLASH)) + CHARGING_MIN_FLASH;
    int charge_light_on_time = (int)chargePCTf;

    // Check delay and toggle as necessary
    if (time_charger_flash_on < (millis() - charge_light_on_time))
    {
      #ifdef debug
      Serial.print("Charge on time : ");
      Serial.println(charge_light_on_time);
      #endif

      // Turn the charge light off
      setButtonColor(1,Black);
    }
  }
  if (DrvState == Charge && !chargeLEDon)
  {
    // Calculate the % of the charger light off-time
    float chargePCTf = chargePCT;
    chargePCTf = 1000 - CHARGING_MIN_FLASH - (chargePCTf/100 * (1000 - CHARGING_MIN_FLASH)) ;
    int charge_light_off_time = int(chargePCTf);

    // Check delay and toggle as necessary
    if (time_charger_flash_off < (millis() - charge_light_off_time))
    {
      #ifdef debug
      Serial.print("Charge off time : ");
      Serial.println(charge_light_off_time);
      #endif

      // Turn the charge light off again
      setButtonColor(1,Red);
    }
  } // end of Charge flashing

}
