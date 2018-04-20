/**/
/*============================================================================*/
/**
**      @file  progMemFunctions.c
**
**      @brief Functions to program or to read flash on PIC24F and
**      dsPIC33F devices.
**
**      @version Alfa Color Tester
**/
/*============================================================================*/
/**/
#include "GenericTypeDefs.h"
#include "Compiler.h"

#include "progMemFunctions.h"

/** T E X T  ************************************************************************** */
void WriteFlashWord(long Addr,long Val)

/******************************************************************************
 *
 *      Oggetto        : Use word writes to write code chunks less than a full
 *                                         64 byte block size.
 *
 *      Parametri      : Addr(address), Val(valore)
 *
 *      Ritorno        : void
 *
 ******************************************************************************/
{
//int progData1L = 0x1111;
//int progData2L = 0x3333;
//char progData1H = 0x11;
//char progData2H = 0x33;

unsigned int progData1L, progData2L;
unsigned int progData1H, progData2H;
//unsigned char progData1H, progData2H;

//DWORD_VAL Address = {Addr};
//DWORD_VAL Value = {Val};

//unsigned int offset;

progData1L = (unsigned int)(Val & 0x0000FFFF);
progData1H = (unsigned int)((Val & 0x00FF0000)>>16);
//progData1H = (unsigned char)((Val & 0x00FF0000)>>16);

progData2L = (unsigned int)(Val & 0x00000000);
progData2H = (unsigned int)((Val & 0x00000000)>>16);
//progData2H = (unsigned char)((Val & 0x00000000)>>16);

  NVMCON = 0x4001;                //Double-word program or executive memory
  TBLPAG = 0xFA;                 //Point TBLPAG to the write latches 
//  NVMADRU = Address.word.HW;  
//  NVMADR  = Address.word.LW;
    
  //Write the low word to the latch
//  __builtin_tblwtl(Address.word.LW, Value.word.LW );

  //Write the high word to the latch (8 bits of data + 8 bits of
  //"phantom data")
//  __builtin_tblwth(Address.word.LW, Value.word.HW );

__builtin_tblwtl(0,progData1L); // Load write latches
__builtin_tblwth(0,progData1H);
__builtin_tblwtl(0x2,progData2L);
__builtin_tblwth(0x2,progData2H);  
  
  //Disable interrupts for next few instructions for unlock sequence
  asm("DISI #5");
  __builtin_write_NVM();
  
  while(NVMCONbits.WR == 1){}

  //Good practice to clear WREN bit anytime we are not expecting to
  //do erase/write operations, further reducing probability of
  //accidental activation.
  NVMCONbits.WREN = 0;
  
}/*end TimerMg*/


/*********************************************************************
 * Function:        DWORD ReadProgramMemory(DWORD address)
 *
 * PreCondition:    None
 *
 * Input:           Program memory address to read from.  Should be
 *                            an even number.
 *
 * Output:          Program word at the specified address.  For the
 *                            PIC24, dsPIC, etc. which have a 24 bit program
 *                            word size, the upper byte is 0x00.
 *
 * Side Effects:    None
 *
 * Overview:        Modifies and restores TBLPAG.  Make sure that if
 *                            using interrupts and the PSV feature of the CPU
 *                            in an ISR that the TBLPAG register is preloaded
 *                            with the correct value (rather than assuming
 *                            TBLPAG is always pointing to the .const section.
 *
 * Note:            None
 ********************************************************************/
DWORD ReadProgramMemory(DWORD address)
{
    DWORD_VAL dwvResult;
    WORD wTBLPAGSave;

    wTBLPAGSave = TBLPAG;
    TBLPAG = ((DWORD_VAL*)&address)->w[1];

    dwvResult.w[1] = __builtin_tblrdh((WORD)address);
    dwvResult.w[0] = __builtin_tblrdl((WORD)address);
    TBLPAG = wTBLPAGSave;

    return dwvResult.Val;
}


/*******************************************************************************
 * Function: void WriteFlashSubBlock(DWORD StartAddress, unsigned
 * short Size, unsigned short* DataBuffer)
 *
 * PreCondition:    None
 *
 * Input:           Program memory address to read from.  Should be
 *                            an even number.
 *
 * Output:          Program word at the specified address.  For the
 *                            PIC24, dsPIC, etc. which have a 24 bit program
 *                            word size, the upper byte is 0x00.
 *
 * Side Effects:    None
 *
 * Overview:        Modifies and restores TBLPAG.  Make sure that if
 *                            using interrupts and the PSV feature of the CPU
 *                            in an ISR that the TBLPAG register is preloaded
 *                            with the correct value (rather than assuming
 *                            TBLPAG is always pointing to the .const section.
 *
 * Note:            None
 ******************************************************************************/
//Use word writes to write code chunks less than a full 64 byte block size.
void WriteFlashSubBlock(DWORD StartAddress, unsigned short Size,
                         unsigned short * DataBuffer)
{
//int progData1L = 0x1111;
//int progData2L = 0x2222;
//char progData1H = 0x11;
//char progData2H = 0x22;
  int progData1L;
  char progData1H;

  unsigned short DataIndex = 0;

  DWORD_VAL Address;

  NVMCON = 0x4001;                //Double-word program or executive memory
  TBLPAG = 0xFA;                 //Point TBLPAG to the write latches 
  
  while(DataIndex < Size)                 //While data is still in the buffer.
  {
    Address = (DWORD_VAL)(StartAddress + DataIndex);

    NVMADRU = Address.word.HW;  
    NVMADR  = Address.word.LW;
  
    progData1L = (int)(DataBuffer[DataIndex] & 0x0000FFFF);
    progData1H = (char)(DataBuffer[DataIndex + 1]);
    //progData1H = (char)((DataBuffer[DataIndex + 1] & 0x00FF0000)>>16);

      //Write the low word to the latch
    __builtin_tblwtl(0,progData1L); // Load write latches
      //Write the high word to the latch (8 bits of data + 8 bits of "phantom data")
    __builtin_tblwth(0,progData1H);
    //__builtin_tblwtl(1,progData2L);
    //__builtin_tblwth(1,progData2H);  

    DataIndex = DataIndex + 2;

    //Disable interrupts for next few instructions for unlock sequence
    asm("DISI #16");
    __builtin_write_NVM();

    while(NVMCONbits.WR == 1){}
      
      //Write the low word to the latch
//      __builtin_tblwtl(Address.word.LW, DataBuffer[DataIndex]);

      //Write the high word to the latch (8 bits of data + 8 bits of
      //"phantom data")
//      __builtin_tblwth(Address.word.LW, DataBuffer[DataIndex + 1]);
//      DataIndex = DataIndex + 2;

      //Disable interrupts for next few instructions for unlock
      //sequence
//      asm("DISI #16");
//      __builtin_write_NVM();

//      while(NVMCONbits.WR == 1){}

  }

  //Good practice to clear WREN bit anytime we are not expecting to
  //do erase/write operations, further reducing probability of
  //accidental activation.
  NVMCONbits.WREN = 0;
}

/******************************************************************************
 *
 *      Oggetto        : Use word writes to write code chunks less than a full
 *                                         64 byte block size.
 *
 *      Parametri      : Addr(address), Val(valore)
 *
 *      Ritorno        : void
 *
 ******************************************************************************/
void EraseFlashPage(unsigned char PageToErase)
{
  DWORD_VAL MemAddressToErase = {0x00000000};
  MemAddressToErase = (DWORD_VAL)(((DWORD)PageToErase) << 10);

  NVMADRU = MemAddressToErase.word.HW;
  NVMADR  = MemAddressToErase.word.LW;  
  NVMCON  = 0x4003;  //Erase page on next WR
  
  //Disable interrupts for next few instructions for unlock sequence
  asm("DISI #16");
  //asm("DISI #5");
  __builtin_write_NVM();

  while(NVMCONbits.WR == 1)
    ;

  NVMCONbits.WREN = 0;
  //EECON1bits.WREN = 0; //Good practice now to clear the WREN bit,
  //as further protection against any future accidental activation
  //of self write/erase operations.
}
