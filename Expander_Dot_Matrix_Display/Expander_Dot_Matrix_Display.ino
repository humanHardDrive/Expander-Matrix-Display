#include <avr/pgmspace.h>

#include <SPI.h>
#include <TimerOne.h>
#include <EEPROM.h>

#define DEBUG_COMM
//#define DEBUG_MCP
#define RUN

#define BYTES_PER_COL   1
#define COL_PER_DISP    8
#define TOTAL_DISP      4

#ifdef DEBUG_COMM
#define MESSAGE_BUFFER_SIZE     64
#define ANIMATION_BUFFER_SIZE   4
#else
#define MESSAGE_BUFFER_SIZE     64
#define ANIMATION_BUFFER_SIZE   22
#endif

#define EEPROM_LOAD_ON_START      0x00  //1 byte
#define EEPROM_MESSAGE_LENGTH     0x01  //1 byte
#define EEPROM_ANIMATION_LENGTH   0x02  //1 byte
#define EEPROM_DISPLAY_MODE       0x03  //1 byte
#define EEPROM_SCROLL_RATE        0x04  //1 byte
#define EEPROM_BUFFER_MAP         0x05  //4 bytes
#define EEPROM_MESSAGE_BUFFER     EEPROM_BUFFER_MAP + sizeof(buffer2disp)
#define EEPROM_ANIMATION_BUFFER   EEPROM_MESSAGE_BUFFER + sizeof(frontMessageBuffer)

#define PIN_SS_1    10
#define PIN_SS_2    9
#define PIN_SS_3    8
#define PIN_SS_4    7

#ifdef DEBUG_COMM
#define PIN_SPEED   6
#endif

#define IODIRA      0x00
#define IODIRB      0x01
#define IOCON       0x0A
#define GPIOA       0x12
#define GPIOB       0x13
#define OLATA       0x14
#define OLATB       0x15

#define ANIM_MASK     0x01  //0 sets the display mode to use the message buffer, 1 to use the animation buffer
#define SCROLL_MASK   0x02  //0 sets the dipslay mode to be static, 1 to use the scroll rate to scroll the buffer
#define INVERT_MASK   0x04  //0 keeps the display as shown in data, 1 inverts on the draw
#define BLINK_MASK    0x08  //0 no blinking, 1 causes periodic inversions by toggling the invert flag with the scroll rate
//Useful combinations
//0 Normal display
//2 Scrolling text
//1 Raw mode, complete control of every pixel
//3 Updating animations

const unsigned char bitMask[] PROGMEM = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
const unsigned char invBitMask[] PROGMEM = {0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F};

union disp_data
{
  uint64_t full;
  uint32_t half[2];
  uint16_t quarter[4];
  uint8_t eighth[8];
};

union command_data
{
  uint16_t full;
  uint8_t half[2];
};

//Array for the bitmaps of the alphabet, starting with upper case set,
//then the lower case set, offset of 26. This and the digit array are stored
//in program memory, because otherwise there would be no space left in RAM
const uint64_t letter_table[] PROGMEM =
{
  0x6666667e66663c00,
  0x3e66663e66663e00,
  0x3c66060606663c00,
  0x3e66666666663e00,
  0x7e06063e06067e00,
  0x0606063e06067e00,
  0x3c66760606663c00,
  0x6666667e66666600,
  0x3c18181818183c00,
  0x1c36363030307800,
  0x66361e0e1e366600,
  0x7e06060606060600,
  0xc6c6c6d6feeec600,
  0xc6c6e6f6decec600,
  0x3c66666666663c00,
  0x06063e6666663e00,
  0x603c766666663c00,
  0x66361e3e66663e00,
  0x3c66603c06663c00,
  0x18181818185a7e00,
  0x7c66666666666600,
  0x183c666666666600,
  0xc6eefed6c6c6c600,
  0xc6c66c386cc6c600,
  0x1818183c66666600,
  0x7e060c1830607e00,
  0x7c667c603c000000,
  0x3e66663e06060600,
  0x3c6606663c000000,
  0x7c66667c60606000,
  0x3c067e663c000000,
  0x0c0c3e0c0c6c3800,
  0x3c607c66667c0000,
  0x6666663e06060600,
  0x3c18181800180000,
  0x1c36363030003000,
  0x66361e3666060600,
  0x1818181818181800,
  0xd6d6feeec6000000,
  0x6666667e3e000000,
  0x3c6666663c000000,
  0x06063e66663e0000,
  0xf0b03c36363c0000,
  0x060666663e000000,
  0x3e403c027c000000,
  0x1818187e18180000,
  0x7c66666666000000,
  0x183c666600000000,
  0x7cd6d6d6c6000000,
  0x663c183c66000000,
  0x3c607c6666000000,
  0x3c0c18303c000000
};

//Bit maps for the numbers 0-9
const uint64_t digit_table[] PROGMEM =
{
  0x3c66666e76663c00,
  0x7e1818181c181800,
  0x7e060c3060663c00,
  0x3c66603860663c00,
  0x30307e3234383000,
  0x3c6660603e067e00,
  0x3c66663e06663c00,
  0x1818183030667e00,
  0x3c66663c66663c00,
  0x3c66607c66663c00
};

//2 buffers for the display
//The front buffer is what is displayed, the back buffer is where draw updates happen
//By flipping the buffers, it updates the display
uint64_t frontBuffer[TOTAL_DISP], backBuffer[TOTAL_DISP];
byte buffer2disp[TOTAL_DISP];

char frontMessageBuffer[MESSAGE_BUFFER_SIZE + TOTAL_DISP];
char backMessageBuffer[MESSAGE_BUFFER_SIZE];
char tempBackMessageBuffer[MESSAGE_BUFFER_SIZE];
byte messagePointer = 0, messageSize = 0, tempMessageSize = 0;

disp_data frontAnimationBuffer[ANIMATION_BUFFER_SIZE][TOTAL_DISP];
disp_data backAnimationBuffer[ANIMATION_BUFFER_SIZE][TOTAL_DISP];
disp_data tempAnimationFrame[TOTAL_DISP];
byte animationSize = 0, tempAnimationSize = 0;

bool dispChanged = false;

byte displayMode = 0, tempDisplayMode; //The display mode is a bitwise variable with flags for animation, scrolling, inversion and blinking
byte scrollRate = 0, tempScrollRate;
unsigned long lastScrollTime = 0;

byte tempLoadOnStartup = 0;

//IO Expander functions
/*------------------------------------------------------------------------------------------*/
/*
   expanderWriteReg
   Arguments: chip select to write to, register to write, value to write to register
   Returns: none
   Notes:
*/
void expanderWriteReg(unsigned char chipSelect, unsigned char reg, unsigned char val)
{
  digitalWrite(chipSelect, LOW);

  //Send the device address
  SPI.transfer(0x40 | 0x0E);
  //Send the register address
  SPI.transfer(reg);
  //Send the register value
  SPI.transfer(val);

  digitalWrite(chipSelect, HIGH);
}

/*
   expanderAllWriteReg
   Arguments: register to write, value to write to register selected
   Returns: none
   Notes: This was an idea to speed up the update call. By selecting all of the chips
   and ignoring the MISO lines (no hardware connection), I can make changes to all
   of the expanders at once for calls where I would have to perform the write reg
   single call multiple times. Useful for turning all off or all on the GPIO.
*/
void expanderAllWriteReg(unsigned char reg, unsigned char val)
{
  digitalWrite(PIN_SS_1, LOW);
  digitalWrite(PIN_SS_2, LOW);
  digitalWrite(PIN_SS_3, LOW);
  digitalWrite(PIN_SS_4, LOW);

  SPI.transfer(0x40 | 0x0E);
  SPI.transfer(reg);
  SPI.transfer(val);

  digitalWrite(PIN_SS_1, HIGH);
  digitalWrite(PIN_SS_2, HIGH);
  digitalWrite(PIN_SS_3, HIGH);
  digitalWrite(PIN_SS_4, HIGH);
}

/*
   expanderReadReg
   Arguments: chip select of the expander, the register to read (see map above and MCP23s17 datasheet)
   Returns: value of register
   Notes:
*/
unsigned char expanderReadReg(unsigned char chipSelect, unsigned char reg)
{
  unsigned char retVal;

  digitalWrite(chipSelect, LOW);

  //Send the device address
  SPI.transfer(0x40 | 0x0E | 0x01);
  //Send the register address
  SPI.transfer(reg);
  //Get the data
  retVal = SPI.transfer(0x00);

  digitalWrite(chipSelect, HIGH);

  return retVal;
}

/*
   setupExpander
   Arguments: chip select of the expander to setup
   Returns: none
   Notes: Function is fairly straight forward, just sets the direction
   registers for bank A&B to be outputs, then setting all the outputs low.
*/
void setupExpander(unsigned char chipSelect)
{
#ifdef DEBUG_MCP
  expanderWriteReg(chipSelect, IODIRA, 0xAB);
  expanderWriteReg(chipSelect, IODIRB, 0xCD);

  expanderWriteReg(chipSelect, OLATA, 0xEF);
  expanderWriteReg(chipSelect, OLATB, 0x12);
#else
  expanderWriteReg(chipSelect, IODIRA, 0x00);
  expanderWriteReg(chipSelect, IODIRB, 0x00);

  expanderWriteReg(chipSelect, OLATA, 0x00);
  expanderWriteReg(chipSelect, OLATB, 0x00);  
#endif
}
/*---------------------------------------------------------------------*/


void loadBuffersFromEEPROM()
{
  messageSize = EEPROM.read(EEPROM_MESSAGE_LENGTH);
  animationSize = EEPROM.read(EEPROM_ANIMATION_LENGTH);
  displayMode = EEPROM.read(EEPROM_DISPLAY_MODE);
  scrollRate = EEPROM.read(EEPROM_SCROLL_RATE);

  EEPROM.get(EEPROM_BUFFER_MAP, buffer2disp);
  EEPROM.get(EEPROM_MESSAGE_BUFFER, frontMessageBuffer);
  EEPROM.get(EEPROM_ANIMATION_BUFFER, frontAnimationBuffer);

  drawDisplay();
}


//Utility functions
/*--------------------------------------------------------------------*/
/*
   hextodecimal
   Arguments: character
   Returns: the conversion from hexadecimal (0 - F), case insensitive, to decimal (0 - 15)
   Notes:
*/
byte hextodecimal(char c)
{
  switch (toupper(c))
  {
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      return c - 48;
      break;

    case 'A':
    case 'B':
    case 'C':
    case 'D':
    case 'E':
    case 'F':
      return toupper(c) - 55;
      break;

    default:
      return 0;
      break;
  }
}

/*
   ishexdigit
   Arguments: character to test
   Returns: true if the character is 0-9 or A-F, case insensitive, false otherwise
   Notes:
*/
bool ishexdigit(char c)
{
  switch (toupper(c))
  {
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
    case 'A':
    case 'B':
    case 'C':
    case 'D':
    case 'E':
    case 'F':
      return true;
      break;

    default:
      return false;
      break;
  }
}

/*
   getDotTranslation
   Arguments: character to display
   Returns: a 64-bit value that represents how the character is drawn on one display
   Notes: This funciton pulls the values out of program space and return them. The
   program read dword is the largest return value function, but still only returns
   half of what is needed for 1 display. So I needed to make 2 calls offset to get
   the full 8 bytes. This was what lead to the creation of the disp_data union.
*/
uint64_t getDotTranslation(char c)
{
  disp_data d;
  d.full = 0;

  if (isdigit(c))
  {
    d.half[0] = pgm_read_dword_near((uint32_t*)digit_table + (c - 48) * 2 + 0);
    d.half[1] = pgm_read_dword_near((uint32_t*)digit_table + (c - 48) * 2 + 1);
  }
  else if (isalpha(c) && c != ' ')
  {
    if (isupper(c))
    {
      d.half[0] = pgm_read_dword_near((uint32_t*)letter_table + (c - 65) * 2 + 0);
      d.half[1] = pgm_read_dword_near((uint32_t*)letter_table + (c - 65) * 2 + 1);
    }
    else
    {
      d.half[0] = pgm_read_dword_near((uint32_t*)letter_table + (c - 97 + 26) * 2 + 0);
      d.half[1] = pgm_read_dword_near((uint32_t*)letter_table + (c - 97 + 26) * 2 + 1);
    }
  }

  return d.full;
}
/*--------------------------------------------------------------------------------------*/


//Commands to do draws and updates
/*---------------------------------------------------------------------------------------*/
/*
   flip
   Arguments: none
   Returns: none
   Notes: Copies the back buffer onto the front buffer to be actively shown.
   The front buffer is what is actually shown on the display. By creating two buffers,
   changes can be done onto a buffer without having to worry about changing the display
   before the timer. Once all of the draws are done, the buffers can be swapped and the
   display updated all at once.
*/
void flip()
{
  memcpy(frontBuffer, backBuffer, sizeof(frontBuffer));
}

/*
   drawDisplay
   Arguments: none
   Returns: none
   Notes: drawDisplay takes the current display mode value and uses it to copy things out
   of the appropriate buffer and draw it onto the back buffer for the display. This function
   also manages all of the scrolling and blinking updates. Updates the disp changed flag
   at the end.
*/
void drawDisplay()
{
  if (displayMode & ANIM_MASK)
  {
    //Draw the animation buffer to the back buffer
    backBuffer[0] = frontAnimationBuffer[messagePointer % animationSize][0].full;
    backBuffer[1] = frontAnimationBuffer[messagePointer % animationSize][1].full;
    backBuffer[2] = frontAnimationBuffer[messagePointer % animationSize][2].full;
    backBuffer[3] = frontAnimationBuffer[messagePointer % animationSize][3].full;

    //Update the scroll pointer to update the animation
    if (displayMode & SCROLL_MASK)
    {
      messagePointer++;
      if (messagePointer >= animationSize)
        messagePointer = 0;
    }
  }
  else
  {
    //Draw the text to the back buffer
    //Do the conversion of letter to dot matrix here
    backBuffer[0] = getDotTranslation(frontMessageBuffer[(messagePointer + 0) % (messageSize + TOTAL_DISP)]);
    backBuffer[1] = getDotTranslation(frontMessageBuffer[(messagePointer + 1) % (messageSize + TOTAL_DISP)]);
    backBuffer[2] = getDotTranslation(frontMessageBuffer[(messagePointer + 2) % (messageSize + TOTAL_DISP)]);
    backBuffer[3] = getDotTranslation(frontMessageBuffer[(messagePointer + 3) % (messageSize + TOTAL_DISP)]);

    if (displayMode & SCROLL_MASK)
    {
      //scrolling text, update pointer
      messagePointer++;
      if (messagePointer > messageSize + TOTAL_DISP)
        messagePointer = 1;
    }
  }

  //Toggle the inversion if blinking
  if (displayMode & BLINK_MASK)
  {
    if (displayMode & INVERT_MASK)
      displayMode &= ~INVERT_MASK;
    else
      displayMode |= INVERT_MASK;
  }

  //Only do the invert once
  //Probably inefficient given this is done on every update
  if (displayMode & INVERT_MASK)
  {
    backBuffer[0] = ~backBuffer[0];
    backBuffer[1] = ~backBuffer[1];
    backBuffer[2] = ~backBuffer[2];
    backBuffer[3] = ~backBuffer[3];
  }

  //Let other functions know the display changed
  dispChanged = true;
}

volatile unsigned char currentColumn = 0;
/*
   updateDisplay
   Arguments: none
   Returns: none
   Notes: This function is called on a timer interrupt, independent of the main background loop, thus
   keeping it in constant time. This function needs to be kept to the bare essentials to prevent running
   into the backgound loop.
*/
void updateDisplay()
{
#ifdef DEBUG_SPEED
  digitalWrite(PIN_SPEED, HIGH);
#endif

  expanderAllWriteReg(OLATB, 0x00); //Turn off the drivers first
  expanderAllWriteReg(OLATA, pgm_read_byte_near(invBitMask + currentColumn)); //Then adjust the sync pins
  //Turning off the drivers first prevents ghosting that would happen if this was done reverse
  expanderWriteReg(PIN_SS_1, OLATB, ((char*)&frontBuffer[buffer2disp[0]])[currentColumn]); //Draw the buffer
  expanderWriteReg(PIN_SS_2, OLATB, ((char*)&frontBuffer[buffer2disp[1]])[currentColumn]);
  expanderWriteReg(PIN_SS_3, OLATB, ((char*)&frontBuffer[buffer2disp[2]])[currentColumn]);
  expanderWriteReg(PIN_SS_4, OLATB, ((char*)&frontBuffer[buffer2disp[3]])[currentColumn]);

#ifdef DEBUG_SPEED
  digitalWrite(PIN_SPEED, LOW);
#endif

  //Increment the column
  currentColumn++;
  //This is a clever trick to get around having to use the modulous function.
  //This only works if the max number of columns is a power of 2
  //Wraps the current column from 0 - 7.
  currentColumn &= 0x07;
}
/*--------------------------------------------------------------------------------*/


//Commands to deal with the command line and parsing commands
/*----------------------------------------------------------------------------*/

/*
   parseCommand
   Arguments: pointer to command_data object, character to parse, current index
   Returns: -1 (failed), 0 (in progress), 1 (no more characters needed)
   Notes: All this command does is take 2 characters and unify them so that comparisons
   can be done on them as a 2-byte integer. The 2 character code is an idea that Elmo
   uses for communication between controllers and their drives. I liked it so I used it
   here.
*/
char parseCommand(command_data* currentCommand, char c, byte* index)
{
#ifdef DEBUG_COMM
  Serial.print("PC"); Serial.print(" CC "); Serial.print(currentCommand->full, HEX); Serial.print(" C "); Serial.print(c, HEX); Serial.print(" I "); Serial.println(*index);
#endif

  switch (*index)
  {
    case 0:
      currentCommand->half[1] = toupper(c);
      (*index)++;
      break;

    case 1:
      currentCommand->half[0] = toupper(c);
      (*index)++;
      return 1;
      break;

    default:
      return -1;
      break;
  }

  return 0;
}

/*
   parseArguments
   Arguments: pointer to command_data object, character to parse, current argument, index variable for internal machine, variable to pass on to other parsers
   Returns: -1 (failed to parse), 0 (parse in progress, not a failed state), 1 (end of command, no more arguments needed)
   Notes: This function runs the initial sanity checks on a command by checking the number of and type of arguments supplied.
   This was the function that caused the biggest headache when first writing it. The state that it's in now is so much cleaner
   than it originally was. Probably should have kept the old version for posterity.
*/
char parseArguments(command_data* currentCommand, char c, byte argumentCount, byte *index, byte *keepMe)
{
#ifdef DEBUG_COMM
  Serial.print("PA"); Serial.print(" CC "); Serial.print(currentCommand->full, HEX); Serial.print(" C "); Serial.print(c, HEX); Serial.print(" AC "); Serial.print(argumentCount); Serial.print(" I "); Serial.print(*index); Serial.print(" KM "); Serial.println(*keepMe);
#endif

  switch (currentCommand->full)
  {
    case 0x4146: //AF, animation frame
      switch (argumentCount)
      {
        case 0: //animation frame to edit
          if (*index == 0)
          {
            //keepMe is used later to replace the specific frame in the buffer
            //Reset it on the first index
            *keepMe = 0;
            (*index)++;
          }

          //The decision to use hex instead of decimal made trying to enter the command line to edit these buffers A LOT easier
          *keepMe *= 16;
          *keepMe += hextodecimal(c);
          break;

        case 1: //disp 0 value
        case 2: //disp 1 value
        case 3: //disp 2 value
        case 4: //disp 3 value
          if (*index == 0)
          {
            tempAnimationFrame[argumentCount - 1].full = 0;
            (*index)++;
          }

          //Especially this part
          tempAnimationFrame[argumentCount - 1].full *= 16;
          tempAnimationFrame[argumentCount - 1].full += hextodecimal(c);
          break;

        default:
          return -1;
          break;
      }
      break;

    case 0x4153: //AS, animation size
      switch (argumentCount)
      {
        case 0:
          if (ishexdigit(c)) //Only accept hex digits for the rate
          {
            if (*index == 0)
            {
              //Reset the value only on the first iteration of this function
              tempAnimationSize = 0;
              *keepMe = 2; //Set val
              (*index)++;
            }

            tempAnimationSize *= 16;
            tempAnimationSize += hextodecimal(c);
          }
          else if (c == '?') //Not a hex digit, is this a query?
          {
            *keepMe = 1; //Query
            return 1;
          }
          else //Nope. Fail out
          {
            return -1;
          }
          break;

        default:
          return -1;
          break;
      }
      break;

    case 0x4442: //DB, draw buffer
      switch (argumentCount)
      {
        case 0:
          switch (toupper(c))
          {
            case 'M':
              *keepMe = 1; //Message
              return 1;
              break;

            case 'A':
              *keepMe = 2; //Animation
              return 1;
              break;

            default:
              return -1; //Something else, fail out
              break;
          }
          break;

        default:
          return -1;
          break;
      }
      break;

    case 0x444D: //DM, display mode
      switch (argumentCount)
      {
        case 0:
          if (ishexdigit(c))
          {
            if (*index == 0)
            {
              tempDisplayMode = 0;
              *keepMe = 2; //Set value
              (*index)++;
            }

            tempDisplayMode *= 16;
            tempDisplayMode += hextodecimal(c);
          }
          else if (c == '?')
          {
            *keepMe = 1; //Query
            return 1;
          }
          else
          {
            return -1;
          }
          break;

        default:
          return -1;
          break;
      }
      break;

    case 0x454C: //EL, EEPROM look
      //This command isn't documented. For obvious reasons
      return -1;
      break;

    case 0x4C44: //LD, load from EEPROM
      return -1;
      break;

    case 0x4C53: //LS, load from EEPROM on startup
      switch (argumentCount)
      {
        case 0:
          tempLoadOnStartup = hextodecimal(c) > 0 ? 1 : 0;
          *keepMe = 1;
          return 1;
          break;

        default:
          return -1;
          break;
      }
      break;

    case 0x4D42: //MB, message buffer
      switch (argumentCount)
      {
        case 0:
          if (*index == 0)
          {
            *keepMe = 1;
            tempMessageSize = 0;
            memset(tempBackMessageBuffer, 0, sizeof(tempBackMessageBuffer));
          }

          if (tempMessageSize < MESSAGE_BUFFER_SIZE)
          {
            tempBackMessageBuffer[*index] = c;
            tempMessageSize++;
            (*index)++;
          }
          else //Keep the length of the buffer within bounds
          {
            return -1;
          }
          break;

        default:
          return -1;
          break;
      }
      break;

    case 0x5244: //RD, reverse display
      return -1; //There shouldn't be any arguments supplied to this command
      break;

    case 0x5352: //SR, scroll rate
      switch (argumentCount)
      {
        case 0:
          if (ishexdigit(c))
          {
            if (*index == 0)
            {
              tempScrollRate = 0;
              *keepMe = 2;
              (*index)++;
            }

            tempScrollRate *= 16;
            tempScrollRate += hextodecimal(c);
          }
          else if (c == '?')
          {
            *keepMe = 1;
            return 1;
          }
          else
          {
            return -1;
          }
          break;

        default:
          return -1;
          break;
      }
      break;

    case 0x5356: //SV, save to EEPROM
      switch (argumentCount)
      {
        case 0:
          switch (toupper(c))
          {
            case 'M':
              *keepMe = 1;
              break;

            case 'A':
              *keepMe = 2;
              break;

            default:
              return -1;
              break;
          }
          break;

        default:
          return -1;
          break;
      }
      break;

    default:
      return -1;
      break;
  }

  return 0;
}

/*
   handleCommand
   Arguments: pointer to a command_data object, byte for the type of command
   Returns: a bool indicating whether or not the command was successful
   Notes: This handles the actual execution of the command. Before this, all arguments
   are written into temporary variables that are only copied here, if the command is valid.
   Commands:  AF, animation frame. Set 1 frame of the animation buffer. AF <frame number (HEX)> <display 0 value (HEX)> <display 1 value (HEX)> <display 2 value (HEX)> <display 3 value (HEX)>
              AS, animation size. Set the length of the animation, or query the current length. AS <length (HEX)> OR AS ?
              DB, draw buffer. Move a back buffered message or animation to the front. DB <A (animation) OR M (message)>
              DM, display mode. Set the display mode variable, or query the current mode. DM <mode (HEX)>. See variable definition for mode description
              LD, load. Loads both the message and animation buffers out of EEPROM
              LS, load on startup. Sets flag in EEPROM to load on startup of display. LS <1 (yes) OR 0 (no)>
              MB, message buffer. Write a message to the back message buffer. DB <message>. Note: spaces in the message must be escaped, or they're counted as an argument delimiter, invalidating the message
              RD, reverse display. Reverse the buffer to display mapping. RD
              SR, scroll rate. Set the scroll rate, or query the current value. SR <rate (10's of milliseconds)(HEX)> OR SR ?
              SV, save. Save message or animation buffer to EEPROM. SV <A (animation) OR M (message)>
*/
bool handleCommand(command_data* currentCommand, byte var)
{
#ifdef DEBUG_COMM
  Serial.print("HC"); Serial.print(" CC "); Serial.print(currentCommand->full, HEX); Serial.print(" V "); Serial.println(var);
#endif

  switch (currentCommand->full)
  {
    case 0x4146: //AF, animation frame
      backAnimationBuffer[var][0].full = tempAnimationFrame[0].full;
      backAnimationBuffer[var][1].full = tempAnimationFrame[1].full;
      backAnimationBuffer[var][2].full = tempAnimationFrame[2].full;
      backAnimationBuffer[var][3].full = tempAnimationFrame[3].full;
      break;

    case 0x4153: //AS, animation size
      switch (var)
      {
        case 1:
          Serial.println(animationSize);
          break;

        case 2:
          animationSize = tempAnimationSize;
          break;

        default:
          return false;
          break;
      }
      break;

    case 0x4442: //DB, draw buffer
      //A valid DB command copies the back message buffer to the front, and issues a redraw of the display
      switch (var)
      {
        case 1:
          memcpy(frontMessageBuffer, backMessageBuffer, sizeof(backMessageBuffer));
          messageSize = tempMessageSize;
          messagePointer = 0;
          lastScrollTime = millis();

          drawDisplay();
          break;

        case 2:
          memcpy(frontAnimationBuffer, backAnimationBuffer, sizeof(backAnimationBuffer));
          messagePointer = 0;
          lastScrollTime = millis();

          drawDisplay();
          break;

        default:
          return false;
          break;
      }
      break;

    case 0x444D: //DM, display mode
      //A valid DM command causes an immidiate redraw of the display
      switch (var)
      {
        case 1:
          Serial.println(displayMode);
          break;

        case 2:
          displayMode = tempDisplayMode;

          messagePointer = 0;
          lastScrollTime = millis();

          drawDisplay();
          break;

        default:
          return false;
          break;
      }
      break;

    case 0x454C:
      for (short i = 0; i < EEPROM.length(); i++)
      {
        Serial.print(EEPROM.read(i), HEX); Serial.print(F(" "));

        if ((i & 0x0F) == 0x0F)
          Serial.println();
      }
      break;

    case 0x4C44: //LD, load from EEPROM
      loadBuffersFromEEPROM();
      break;

    case 0x4C53: //LS, load from EEPROM on startup
      if (var != 0)
        EEPROM.update(EEPROM_LOAD_ON_START, tempLoadOnStartup);
      else
        return false;
      break;

    case 0x4D42: //MB, message buffer
      if (var != 0)
        memcpy(backMessageBuffer, tempBackMessageBuffer, MESSAGE_BUFFER_SIZE);
      else
        return false;
      break;

    case 0x5244: //RD, reverse display
      for (byte i = 0; i < TOTAL_DISP / 2; i++)
      {
        byte temp;
        temp = buffer2disp[i];
        buffer2disp[i] = buffer2disp[TOTAL_DISP - i - 1];
        buffer2disp[TOTAL_DISP - i - 1] = temp;
      }
      break;

    case 0x5352: //SR, scroll rate
      switch (var)
      {
        case 1:
          Serial.println(scrollRate);
          break;

        case 2:
          scrollRate = tempScrollRate;
          break;

        default:
          return false;
          break;
      }
      break;

    case 0x5356: //SV, save to EEPROM
      switch (var)
      {
        case 1: //message
          EEPROM.update(EEPROM_MESSAGE_LENGTH, messageSize);
          EEPROM.update(EEPROM_DISPLAY_MODE, displayMode);
          EEPROM.update(EEPROM_SCROLL_RATE, scrollRate);
          EEPROM.put(EEPROM_BUFFER_MAP, buffer2disp);
          EEPROM.put(EEPROM_MESSAGE_BUFFER, frontMessageBuffer);
          break;

        case 2: //animation
          EEPROM.update(EEPROM_ANIMATION_LENGTH, animationSize);
          EEPROM.update(EEPROM_DISPLAY_MODE, displayMode);
          EEPROM.update(EEPROM_SCROLL_RATE, scrollRate);
          EEPROM.put(EEPROM_BUFFER_MAP, buffer2disp);
          EEPROM.put(EEPROM_MESSAGE_BUFFER, frontMessageBuffer);
          break;

        default:
          return false;
          break;
      }
      break;

    default:
      return false;
      break;
  }

  return true;
}

/*
   ParseCommandLine
   Arguments: The current character from the command line
   Returns: none
   Notes: The parse command line function is the highest level of parser, calling the above functions
   and maintaining all the status variables statically. The command line parser needs to be able to
   handle commands fed one at a time, not just delivered in bulk as done through the Arduino terminal.
   The function will print an OK or NOK depending on the command delivered. For a full list of commands
   see the handlecommand function.
*/
void parseCommandLine(char c)
{
  //Static variables stick around between calls.
  //I like statics in some places compared to globals because it allows for cleaner control
  //This function handles all of the command line stuff, so it makes sense for all the necessary
  //variable to stay here instead of every function having global access
  static byte argumentCount = 0, keepMe = 0, index = 0;
  static bool escape = false, validCommand = true;
  static command_data currentCommand;
  /*
     The state machine to parse these commands was originally awful and almost impossible to read.
     Even to the guy who wrote it.
     This machine is much cleaner but there are still some remanants of things hacked together. The
     index and keepMe variables were how the lower level parsers kept track of their own states. This
     made it difficult to reset those machines when new commands needed to be loaded, or thrown out. Index
     serves as tracker for the current argument, reset each time, where as keepMe is kept around for the entire
     command.
     There may be commands added in the future that may require reworking those variables.
  */

#ifdef DEBUG_COMM
  Serial.print("PCL"); Serial.print(" C "); Serial.print(c, HEX), Serial.print(" AC "); Serial.print(argumentCount); Serial.print(" I "); Serial.print(index); Serial.print(" KM "); Serial.print(keepMe); Serial.print(" ES "); Serial.print(escape, HEX); Serial.print(" VC "); Serial.println(validCommand, HEX);
#endif

  //Some systems use just a newline, some use a newline and carrige return. Support both!
  if (c == '\n' || c == '\r')
  {
    //Ignore empty commands
    if (currentCommand.full != 0)
    {
      //The command isn't full evaluated until this point
      if (validCommand && handleCommand(&currentCommand, keepMe))
      {
        Serial.println(F("OK"));
      }
      else
      {
        Serial.println(F("NOK"));
      }
    }

    //Reset everything
    argumentCount = 0;
    keepMe = 0;
    index = 0;

    currentCommand.full = 0;

    escape = false;
    validCommand = true;
  }
  else if (validCommand) //Only continue with valid commands. Invalid commands have to wait until a newline to be reset
  {
    if (c == '\\' && !escape) //\ marks the start of an escape sequence. Any character that follows it is part of the sequence
    {
      escape = true; //Set the flag
    }
    else if (c == ' ' && !escape) //A space is used to delimit the arguments, unless it's part of an escape sequence. Necessary for the MB command which can handle spaces
    {
      argumentCount++; //Increment the arguments
      index = 0; //Reset the index for this argument
    }
    else
    {
      if (argumentCount == 0) //The zero-th argument is the command
      {
        if (parseCommand(&currentCommand, c, &index) < 0) //Check for the correct number of characters
          validCommand = false;
      }
      else if (currentCommand.full != 0) //Only parse arguments for non-zero commands
      {
        //Some sanity checks for the command can be done here
        //See function for more detail
        if (parseArguments(&currentCommand, c, argumentCount - 1, &index, &keepMe) < 0)
          validCommand = false;
      }
      else //This point should never be hit, but switch to a non-valid command anyway
      {
        validCommand = false;
      }

      //Any character resets the escape flag
      escape = false;
    }
  }
}
/*-----------------------------------------------------------------------------*/

void setup()
{
  //Chip select pins for the 4 IO expanders
  pinMode(PIN_SS_1, OUTPUT);
  pinMode(PIN_SS_2, OUTPUT);
  pinMode(PIN_SS_3, OUTPUT);
  pinMode(PIN_SS_4, OUTPUT);
#ifdef DEBUG_SPEED
  pinMode(PIN_SPEED, OUTPUT);
#endif

  //Default the pins to high
  digitalWrite(PIN_SS_1, HIGH);
  digitalWrite(PIN_SS_2, HIGH);
  digitalWrite(PIN_SS_3, HIGH);
  digitalWrite(PIN_SS_4, HIGH);
#ifdef DEBUG_SPEED
  digitalWrite(PIN_SPEED, LOW);
#endif

  Serial.begin(115200);

  //Run the SPI clock at the highest rate possible
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  //The MCP23s17 is a nice chip as it runs with the default SPI configuration
  SPI.begin();

#ifdef RUN
  //Setup the timer for the display update
  Timer1.initialize(1500); //Set to every 1.5mS. Anything more than 2ms is noticeable.
  //A smaller time is doable, but you'll run into the main loop at some point
  Timer1.attachInterrupt(updateDisplay);
#endif

  //Setup the IO expanders
  //Yes, this could be done with a global write to the expanders
  //But do it one at a time anyway
  setupExpander(PIN_SS_1);
  setupExpander(PIN_SS_2);
  setupExpander(PIN_SS_3);
  setupExpander(PIN_SS_4);

  //Zero out the front and back buffers
  memset(frontBuffer, 0, sizeof(frontBuffer));
  memset(backBuffer, 0, sizeof(backBuffer));

  memset(frontMessageBuffer, 0, sizeof(frontMessageBuffer));
  memset(backMessageBuffer, 0, sizeof(backMessageBuffer));
  memset(tempBackMessageBuffer, 0, sizeof(tempBackMessageBuffer));

  memset(frontAnimationBuffer, 0, sizeof(frontAnimationBuffer));
  memset(backAnimationBuffer, 0, sizeof(backAnimationBuffer));
  memset(tempAnimationFrame, 0, sizeof(tempAnimationFrame));

  lastScrollTime = millis();

  //Setup the buffer to display mappings
  buffer2disp[0] = 0;
  buffer2disp[1] = 1;
  buffer2disp[2] = 2;
  buffer2disp[3] = 3;

  //Clear out the displays
  //Turn on all the sink pins, turn off the source pins
#ifndef DEBUG_MCP
  expanderAllWriteReg(OLATA, 0xFF);
  expanderAllWriteReg(OLATB, 0x00);
#endif

  //Check if we should load from EEPROM
  if (EEPROM.read(EEPROM_LOAD_ON_START) == 1)
  {
    loadBuffersFromEEPROM();
  }

#ifdef DEBUG_MCP
  Serial.print("OLATA: "); Serial.println(expanderReadReg(PIN_SS_1, OLATA));
  Serial.print("OLATB: "); Serial.println(expanderReadReg(PIN_SS_1, OLATB));
  Serial.print("IODIRA: "); Serial.println(expanderReadReg(PIN_SS_1, IODIRA));
  Serial.print("IODIRB: "); Serial.println(expanderReadReg(PIN_SS_1, IODIRB));   
#endif
}

void loop()
{
#ifdef RUN
  //Check for new data on ther serial line
  if (Serial.available())
    parseCommandLine(Serial.read());

  //Update the display if it needs to scroll text, animate, or blink
  if ((displayMode & SCROLL_MASK || displayMode & BLINK_MASK) &&
      scrollRate != 0 && (millis() - lastScrollTime) > 10 * (unsigned short)(scrollRate))
  {
    drawDisplay();
    lastScrollTime = millis();
  }

  //Draw the changes to the front buffer, but only if necessary
  if (dispChanged)
  {
    flip();
    dispChanged = false;
  }
#endif
}
