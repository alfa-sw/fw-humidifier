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
void WriteFlashDoubleWord(long Addr,long Val)

/******************************************************************************
 *
 *      Oggetto        : Use double word writes to write code chunks less than a full
 *                       64 byte block size.
 *
 *      Parametri      : Addr(address), Val(valore)
 *
 *      Ritorno        : void
 *
 ******************************************************************************/
{
  unsigned int progData1L, progData2L;
  unsigned char progData1H, progData2H;
//  unsigned long Val1;
  DWORD_VAL Address = {Addr};
/*
  // Reverse Write
  progData1L = (unsigned int)(Val & 0x0000FFFF);
  progData1H = (unsigned char)((Val & 0x00FF0000)>>16);
  progData2L = (unsigned int)((Val1 & 0x00FF0000)>>16);
  progData2L += (unsigned int)(Val1 & 0x0000FF00);
  progData2H = (unsigned char)(Val1 & 0x000000FF);
*/

  // Forward Write
  progData1L = (unsigned int)((Val & 0x00FF0000)>>16);
  progData1L += (unsigned int)(Val & 0x0000FF00);
  progData1H = (unsigned char)(Val & 0x000000FF);

  progData2L = (unsigned int)((Val & 0x00FF0000)>>16);
  progData2L += (unsigned int)(Val & 0x0000FF00);
  progData2H = (unsigned char)(Val & 0x000000FF);
  
  TBLPAG = 0xFA;                //Point TBLPAG to the write latches   
  NVMCON = 0x4001;               //Double-word program or executive memory
  NVMADRU = Address.word.HW;  
  NVMADR  = Address.word.LW;
  
  __builtin_tblwtl(0,progData1L); // Load write latches
  __builtin_tblwth(0,progData1H);
  __builtin_tblwtl(2,progData2L);
  __builtin_tblwth(2,progData2H);  

  //Disable interrupts for next few instructions for unlock sequence
  __builtin_disi(5);  
  __builtin_write_NVM();

  while(NVMCONbits.WR == 1){}
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
  unsigned int progData1L, progData2L;
  unsigned char progData1H, progData2H;
  unsigned short DataIndex = 0;
//  unsigned int Addr;
  DWORD_VAL Address;

  TBLPAG = 0xFA;    // Point TBLPAG to the write latches   
  NVMCON = 0x4001;  // Double-word program or executive memory

  // While data is still in the buffer.
  // All the packet (493) are 56 bytes, with exception of the last one who is 24 bytes
  while(DataIndex < Size)                 
  {
    Address = (DWORD_VAL)(StartAddress + DataIndex);
/*
    Addr = (unsigned int)(StartAddress + DataIndex);
    if (Addr == 0x2200)
        DataBuffer[DataIndex] = 0x1234;
*/
    // 1 Word
    progData1L = (unsigned int)(DataBuffer[DataIndex]);
    progData1H = (unsigned char)(DataBuffer[DataIndex + 1]);
    // 2 Word
    progData2L = (unsigned int)(DataBuffer[DataIndex + 2]);
    progData2H = (unsigned char)(DataBuffer[DataIndex + 3]);

    NVMADRU = Address.word.HW;  
    NVMADR  = Address.word.LW;
    
    // Load write latches
    __builtin_tblwtl(0,progData1L); 
    __builtin_tblwth(0,progData1H);
    __builtin_tblwtl(2,progData2L);
    __builtin_tblwth(2,progData2H);  

    // 1 Double Word = 4 unsigned short
    DataIndex = DataIndex + 4;

    // Disable interrupts for next few instructions for unlock sequence
    __builtin_disi(5);  
    __builtin_write_NVM();

    while(NVMCONbits.WR == 1){}
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

  NVMCON  = 0x4003;  //Erase page on next WR
  NVMADRU = MemAddressToErase.word.HW;
  NVMADR  = MemAddressToErase.word.LW;  
  
  //Disable interrupts for next few instructions for unlock sequence
  //asm("DISI #16");
  asm("DISI #5");
  __builtin_write_NVM();

  while(NVMCONbits.WR == 1)
    ;

  NVMCONbits.WREN = 0;
  //EECON1bits.WREN = 0; //Good practice now to clear the WREN bit,
  //as further protection against any future accidental activation
  //of self write/erase operations.
}
