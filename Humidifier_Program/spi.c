/*
*//*=====================================================================*//**
**
**      File Name  : spi.c
**
**      Description   : SPI Interface Management
**
**      Project       : Alfa HUTBRD
**
*//*=====================================================================*//**
*/
#include "p24FJ64GA704.h"
#include "spi.h"
#include "ram.h"
#include "gestio.h"
#include "define.h"
#include "timerMg.h"
#include <xc.h>
#include <stdlib.h>

/*
*//*=====================================================================*//**
**      @brief Initialization SPi1
**
**      @param void
**
**      @retval void
**
*//*=====================================================================*//**
*/
void SPI1_Initialize(void)
{
    // AUDEN disabled; FRMEN disabled; AUDMOD I2S; FRMSYPW One clock wide; AUDMONO stereo; FRMCNT 0; MSSEN disabled; FRMPOL disabled; IGNROV disabled; SPISGNEXT not sign-extended; FRMSYNC disabled; URDTEN disabled; IGNTUR disabled; 
    SPI1CON1H = 0x0000;
    // WLENGTH 0; 
    SPI1CON2L = 0x0000;
    // SPIROV disabled; FRMERR disabled; 
    SPI1STATL = 0x0000;
    // SPI1BRGL 1 = 1MHZ; 
    SPI1BRGL = 0x0001;
    // SPITBFEN disabled; SPITUREN disabled; FRMERREN disabled; SRMTEN disabled; SPIRBEN disabled; BUSYEN disabled; SPITBEN disabled; SPIROVEN disabled; SPIRBFEN disabled; 
    SPI1IMSKL = 0x0000;
    // RXMSK 0; TXWIEN disabled; TXMSK 0; RXWIEN disabled; 
    SPI1IMSKH = 0x0000;
    // SPI1URDTL 0; 
    SPI1URDTL = 0x0000;
    // SPI1URDTH 0; 
    SPI1URDTH = 0x0000;
    // SPIEN enabled; DISSDO disabled; MCLKEN FOSC/2; CKP Idle:Low, Active:High; SSEN disabled; MSTEN Master; MODE16 disabled; SMP Middle; DISSCK disabled; SPIFE Frame Sync pulse precedes; CKE Idle to Active; MODE32 enabled; SPISIDL disabled; ENHBUF enabled; DISSDI disabled; 
    // 32 bit
    //SPI1CON1L = 0x8821;
    // SPIEN enabled; DISSDO disabled; MCLKEN FOSC/2; CKP Idle:Low, Active:High; SSEN disabled; MSTEN Master; MODE16 disabled; SMP Middle; DISSCK disabled; SPIFE Frame Sync pulse precedes; CKE Idle to Active; MODE32 disabled; SPISIDL disabled; ENHBUF enabled; DISSDI disabled; 
    // 8 bit - CKP = 0 (Idle state for clock is a low level; active state is a high level) - CKE = 0 (Transmit happens on transitions from idle clock to active clock state)
    SPI1CON1L = 0x8021;

    // Al momento NON usiamo gli Interrupt
    // Enable alt 3 SPI1 Interrupts
    //    SPITXI: SPI1TX - SPI1 Transfer Done
    //    Priority: 1
//    IPC2bits.SPI1TXIP = 1;
    //    SPII: SPI1 - SPI1 General
    //    Priority: 1
//    IPC2bits.SPI1IP = 1;
    //    SPIRXI: SPI1RX - SPI1 Receive Done
    //    Priority: 1
//    IPC14bits.SPI1RXIP = 1;

    // TC72 Enabled
    SPI_SS = 1;
}

/*
*//*=====================================================================*//**
**      @brief Write_SPI_Command: 2 bytes sent
**
**      @param void
**
**      @retval void
**
*//*=====================================================================*//**
*/
/*
void Write_SPI_Command(uint16_t *pTransmitData)
{
    while( SPI1STATLbits.SPITBF == TRUE)
    {

    }
    SPI1BUFL = *(pTransmitData);
    SPI1BUFH = *(pTransmitData+1);
}
*/
void Write_SPI_Command(uint8_t *pTransmitData)
{
    // Send 1 Byte
    while(SPI1STATLbits.SPITBF == TRUE) {}
    SPI1BUFL = *(pTransmitData);
    // Send 2 Byte
    while(SPI1STATLbits.SPITBF == TRUE) {}
    SPI1BUFL = *(pTransmitData+1);
}  

/*
*//*=====================================================================*//**
**      @brief Read_SPI_Command
**
**      @param void
**
**      @retval void
**
*//*=====================================================================*//**
*/
/*
void Read_SPI_Command(uint16_t *pTransmitData, uint16_t *pReceiveData)
{
    while( SPI1STATLbits.SPITBF == TRUE)
    {

    }
    SPI1BUFL = *(pTransmitData);
    SPI1BUFH = *(pTransmitData+1);

    while ( SPI1STATLbits.SPIRBE == FALSE)
    {
    
    }
    *(pReceiveData) = SPI1BUFL;
    *(pReceiveData+1) = SPI1BUFH;    
}
*/
void Read_SPI_Command(uint8_t *pTransmitData, uint8_t *pReceiveData)
{
    // Send 1 Byte
    while( SPI1STATLbits.SPITBF == TRUE) {}
    SPI1BUFL = *(pTransmitData);
    // Read 1 Byte
    while ( SPI1STATLbits.SPIRBE == TRUE) {}
    *(pReceiveData) = SPI1BUFL;
    // Read 2 Byte
    while ( SPI1STATLbits.SPIRBE == TRUE) {}
    *(pReceiveData+1) = SPI1BUFL;
    // Read 3 Byte
    while ( SPI1STATLbits.SPIRBE == TRUE) {}
    *(pReceiveData+2) = SPI1BUFL;
}
/*
*//*=====================================================================*//**
**      @brief Updates Temperature TC72 Acquisition
**
**      @param void
**
**      @retval void
**
*//*=====================================================================*//**
*/
void SPI_Manager (void)
{
    static uint8_t Status_SPI = SPI_IDLE;
    static uint8_t Read_res[3];
    //static uint16_t Read_res[2];
    uint16_t TC72_Raw_Temperature;     
    //uint16_t writeBuffer[2];
    uint8_t writeBuffer[2];
    uint8_t Read_results[2];
    
    switch (Status_SPI)
    {
        case SPI_IDLE:
            Read_res[0] = 0;
            Read_res[1] = 0;
            Read_res[2] = 0;
            
            TemperatureResetProcedure(OFF);
            if (Start_New_Temp_Measurement == TRUE)
                Status_SPI = SPI_WRITE_MULTIPLE_BYTE_TRANSFER;
        break;      
// -----------------------------------------------------------------------------
        // Write Multiple Byte Transfer Command to TC72
        case SPI_WRITE_MULTIPLE_BYTE_TRANSFER:
            // Build the write buffer first: 
            //writeBuffer[0] = 0x8015; // MSB = 1,OS = 1 - SHDN = 1           
            //writeBuffer[1] = 0x0000;             
            writeBuffer[0] = 0x80; // MSB = 1           
            writeBuffer[1] = 0x15; // OS = 1 - SHDN = 1             
            Write_SPI_Command (writeBuffer);
            StartTimer(T_SPI_MEASUREMENT);           
            Status_SPI = SPI_WAIT_READ_RESULTS;                        
        break;

        // WAIT 200 msec before to Read results (at least 150msec)
        case SPI_WAIT_READ_RESULTS:
            if (StatusTimer(T_SPI_MEASUREMENT) == T_ELAPSED)
            {
                StopTimer(T_SPI_MEASUREMENT);
                Status_SPI = SPI_READ_RESULTS;                
            }
        break;

        // Read Temperature 
        case SPI_READ_RESULTS:
            // Build the write buffer first: 
//            writeBuffer[0] = 0x0200; // MSB = 0,ADDRESS = 0x02        
//            writeBuffer[1] = 0x0000;             
            writeBuffer[0] = 0x02; // MSB = 0,ADDRESS = 0x02        
            Read_SPI_Command (writeBuffer, Read_res);            
            // Read Control Register if it is correct: After completion of the temperature conversion, the One-Shot bit (OS) is reset to ?0?
//            if (Read_res[2] == 0x05) // OS = 0 - SHDN = 1 
//            {
                Sensor_Temp_Measurement_Error = FALSE;
                Status_SPI = SPI_CALCULATE_TEMPERATURE;  
/*            }
            else
            {
                Sensor_Temp_Measurement_Error = TRUE;
                // Set HARD RESET
                Status_SPI = SPI_HARD_RESET;                                        
            }                
*/        break;

        // Calculate Absolute values from Raw data
        case SPI_CALCULATE_TEMPERATURE:
            Read_results[0] = Read_res[0];
            Read_results[1] = Read_res[1]>>6;
            // Temperature (°C) x 10            
            TC72_Raw_Temperature = Read_results[0] * 10 + (Read_results[1] * 25) / 10; 
            TC72_Temperature = TC72_Raw_Temperature; 
            Start_New_Temp_Measurement = FALSE;            
            // At the end of Measuremnent Sensor TC72 goes automatically into SHUTDOWN mode 
            Status_SPI = SPI_IDLE;
        break;
        
        case SPI_HARD_RESET:
            // Wait Reset End            
            if (TemperatureResetProcedure(ON) == TRUE)
            { 
                Start_New_Temp_Measurement = FALSE;  
                Status_SPI = SPI_IDLE;
            }
        break;

        default:
        break;    
    }        
}     

/*
*//*=====================================================================*//**
**      @brief SPI Hard reset Procedure 
**
**      @param unsigned chat 'type': ON  --> Hard Reset Activation
**                                   OFF --> StopHard Reset   
**
**      @retval unsigned int status: TRUE: Hard Reset Done or Stopped
**                                   WAIT: Hard Reset in Execution         
**
*//*=====================================================================*//**
*/
unsigned int TemperatureResetProcedure (unsigned char type)
{
    static unsigned int reset_procedure = RESET_ON;
    
    if (type == OFF)
    {
        StopTimer(T_SPI_HARD_RESET);
        // TC72 Enabled
        SPI_SS = 1;
        reset_procedure = RESET_ON;
        return TRUE;
    }
    else 
    {
        switch (reset_procedure)
        {            
            case RESET_ON:
                StopTimer(T_SPI_HARD_RESET);
                StartTimer(T_SPI_HARD_RESET);
                // TC72 Disabled
                SPI_SS = 0;
                reset_procedure = RESET_WAIT;
                return WAIT;
            break;
            case RESET_WAIT:
                if (StatusTimer(T_SPI_HARD_RESET) == T_ELAPSED)
                {            
                    // TC72 Enabled
                    SPI_SS = 1;
                    reset_procedure = RESET_ON;
                    return TRUE;
                }                            
                else
                    return WAIT;
            break;
            default:
                return WAIT;
            break;
        }                            
    }    
}
