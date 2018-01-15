/**
  I2C1 Generated Driver API Header File

  @Company
    Microchip Technology Inc.

  @File Name
    i2c1.h

  @Summary
    This is the generated header file for the I2C1 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This header file provides APIs for driver for I2C1.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - pic24-dspic-pic32mm : v1.45
        Device            :  PIC24FJ64GA704
        Driver Version    :  1.0
    The generated drivers are tested against the following:
        Compiler          :  XC16 1.32
        MPLAB 	          :  MPLAB X 3.61
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#ifndef _I2C1_H
#define _I2C1_H

/**
  Section: Included Files
*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <xc.h>

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

/**
 Section: Data Type Definitions
*/

/**
  I2C Driver Message Status Type Enumeration

  @Summary
    Defines the different message status when processing
    TRBs.

  @Description
    This type enumeration specifies the different types of status
    that an i2c request will have. For every submission of an i2c
    transaction, the status of that transaction is available.
    Based on the status, new transactions can be requested to the
    driver or a recovery can be performed to resend the transaction.

 */

typedef enum
{
    I2C1_MESSAGE_FAIL,
    I2C1_MESSAGE_PENDING,
    I2C1_MESSAGE_COMPLETE,
    I2C1_STUCK_START,
    I2C1_MESSAGE_ADDRESS_NO_ACK,
    I2C1_DATA_NO_ACK,
    I2C1_LOST_STATE
} I2C1_MESSAGE_STATUS;

typedef enum
{
    I2C_IDLE,
    I2C_WRITE_SINGLE_SHOT_DATA_ACQUISITION,
    I2C_WAIT_READ_RESULTS,
    I2C_READ_RESULTS,
    I2C_CALCULATE_HUMIDITY_TEMPERATURE,
    I2C_WRITE_STATUS_COMMAND,
    I2C_WAIT_READ_STATUS,
    I2C_READ_STATUS,
    I2C_HARD_RESET,
    I2C_WRITE_HEATER_ON,
    I2C_WRITE_HEATER_OFF,
    I2C_WAIT_HEATER_ON,        
    I2C_WRITE_HEATER_STATUS_COMMAND,
    I2C_READ_ON_STATUS,
    I2C_WAIT_READ_STATUS_OFF,
    I2C_WRITE_HEATER_OFF_STATUS_COMMAND,
    I2C_WAIT_READ_OFF_STATUS,
    I2C_READ_OFF_STATUS,
    I2C_WAIT_READ_ON_STATUS,
} I2C1_ACQUISITION_STATUS;


/**
  I2C Driver Transaction Request Block (TRB) type definition.

  @Summary
    This defines the Transaction Request Block (TRB) used by the
    i2c master in sending/receiving data to the i2c bus.

  @Description
    This data type is the i2c Transaction Request Block (TRB) that
    the needs to be built and sent to the driver to handle each i2c requests.
    Using the TRB, simple to complex i2c transactions can be constructed
    and sent to the i2c bus. This data type is only used by the master mode.

 */
typedef struct
{
    uint16_t  address;          // Bits <10:1> are the 10 bit address.
                                // Bits <7:1> are the 7 bit address
                                // Bit 0 is R/W (1 for read)
    uint8_t   length;           // the # of bytes in the buffer
    uint8_t   *pbuffer;         // a pointer to a buffer of length bytes
} I2C1_TRANSACTION_REQUEST_BLOCK;

typedef union
{
    struct
    {
            uint8_t full:1;
            uint8_t empty:1;
            uint8_t reserved:6;
    }s;
    uint8_t status;
} I2C_TR_QUEUE_STATUS;

/**
  I2C Driver Queue Entry Type

  @Summary
    Defines the object used for an entry in the i2c queue items.

  @Description
    This defines the object in the i2c queue. Each entry is a composed
    of a list of TRBs, the number of the TRBs and the status of the
    currently processed TRB.
 */
typedef struct
{
    uint8_t                         count;          // a count of trb's in the trb list
    I2C1_TRANSACTION_REQUEST_BLOCK  *ptrb_list;     // pointer to the trb list
    I2C1_MESSAGE_STATUS             *pTrFlag;       // set with the error of the last trb sent.
                                                    // if all trb's are sent successfully,
                                                    // then this is I2C1_MESSAGE_COMPLETE
} I2C_TR_QUEUE_ENTRY;

/**
  I2C Master Driver Object Type

  @Summary
    Defines the object that manages the i2c master.

  @Description
    This defines the object that manages the sending and receiving of
    i2c master transactions.
  */

typedef struct
{
    /* Read/Write Queue */
    I2C_TR_QUEUE_ENTRY          *pTrTail;       // tail of the queue
    I2C_TR_QUEUE_ENTRY          *pTrHead;       // head of the queue
    I2C_TR_QUEUE_STATUS         trStatus;       // status of the last transaction
    uint8_t                     i2cDoneFlag;    // flag to indicate the current
                                                // transaction is done
    uint8_t                     i2cErrors;      // keeps track of errors


} I2C_OBJECT ;

/**
  I2C Master Driver State Enumeration

  @Summary
    Defines the different states of the i2c master.

  @Description
    This defines the different states that the i2c master
    used to process transactions on the i2c bus.
*/

typedef enum
{
    S_MASTER_IDLE,
    S_MASTER_RESTART,
    S_MASTER_SEND_ADDR,
    S_MASTER_SEND_DATA,
    S_MASTER_SEND_STOP,
    S_MASTER_ACK_ADDR,
    S_MASTER_RCV_DATA,
    S_MASTER_RCV_STOP,
    S_MASTER_ACK_RCV_DATA,
    S_MASTER_NOACK_STOP,
    S_MASTER_SEND_ADDR_10BIT_LSB,
    S_MASTER_10BIT_RESTART,
    
} I2C_MASTER_STATES;
       
extern void I2C1_Initialize(void);
extern void I2C1_MasterWrite(
                                uint8_t *pdata,
                                uint8_t length,
                                uint16_t address,
                                I2C1_MESSAGE_STATUS *pstatus);

extern void I2C1_MasterRead(
                                uint8_t *pdata,
                                uint8_t length,
                                uint16_t address,
                                I2C1_MESSAGE_STATUS *pstatus);
                                

extern void I2C1_MasterTRBInsert(
                                uint8_t count,
                                I2C1_TRANSACTION_REQUEST_BLOCK *ptrb_list,
                                I2C1_MESSAGE_STATUS *pflag);
                                

extern void I2C1_MasterReadTRBBuild(
                                I2C1_TRANSACTION_REQUEST_BLOCK *ptrb,
                                uint8_t *pdata,
                                uint8_t length,
                                uint16_t address);                               
                                
extern void I2C1_MasterWriteTRBBuild(
                                I2C1_TRANSACTION_REQUEST_BLOCK *ptrb,
                                uint8_t *pdata,
                                uint8_t length,
                                uint16_t address);                           
                                
extern bool I2C1_MasterQueueIsEmpty(void);                              

extern bool I2C1_MasterQueueIsFull(void);             

extern unsigned int ResetProcedure (unsigned char type);
extern void I2C_Manager (void);
extern uint8_t Write_I2C_Command (uint8_t writeCommand[2]);
extern uint8_t Read_I2C_Command (uint8_t *res, uint8_t bytes_n);

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif //_I2C1_H
    
/**
 End of File
*/
