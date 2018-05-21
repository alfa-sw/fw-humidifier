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
    //SPI1BRGL = 0x0001;
    // SPI1BRGL = 125KHZ; 
    SPI1BRGL = 0x008;
    // SPITBFEN disabled; SPITUREN disabled; FRMERREN disabled; SRMTEN disabled; SPIRBEN disabled; BUSYEN disabled; SPITBEN disabled; SPIROVEN disabled; SPIRBFEN disabled; 
    SPI1IMSKL = 0x0000;
    // RXMSK 0; TXWIEN disabled; TXMSK 0; RXWIEN disabled; 
    SPI1IMSKH = 0x0000;
    // SPI1URDTL 0; 
    SPI1URDTL = 0x0000;
    // SPI1URDTH 0; 
    SPI1URDTH = 0x0000;
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
}

void Write_SPI_Command(uint8_t *pTransmitData)
{
    // Send first Byte
    while(SPI1STATLbits.SPITBF == TRUE) {}
    SPI1BUFL = *(pTransmitData);
    // Send second Byte
    while(SPI1STATLbits.SPITBF == TRUE) {}
    SPI1BUFL = *(pTransmitData+1);
}  

void Read_SPI_Command(uint8_t *pTransmitData, uint8_t *pReceiveData)
{
    // Send first Byte
    while( SPI1STATLbits.SPITBF == TRUE) {}
      SPI1BUFL = *(pTransmitData);
    // Read first Byte
    while ( SPI1STATLbits.SPIRBE == TRUE) {}
    *(pReceiveData) = SPI1BUFL;
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
    uint8_t Read_res[1];
    uint16_t TC72_Raw_Temperature;     
    uint8_t writeBuffer[2];
    static uint8_t Read_results[3];
    
    switch (Status_SPI)
    {
        case SPI_IDLE:
            if (Dos_Temperature_Enable == TRUE) {
                // Wait RESET ends
                if (TemperatureResetProcedure(ON) == TRUE)  {                   
                    // Continuous Mode selected 
                    writeBuffer[0] = 0x80; // MSB = 1           
                    writeBuffer[1] = 0x04; // OS = 0 - SHDN = 0             
                    // Write Multiple Byte Temperature Command to TC72
                    Write_SPI_Command (writeBuffer);
                    Status_SPI = SPI_WRITE_MULTIPLE_BYTE_TRANSFER;                    
                }
            }    
        break;      
// -----------------------------------------------------------------------------
        case SPI_WRITE_MULTIPLE_BYTE_TRANSFER:
            if (Dos_Temperature_Enable == TRUE) { 
                if (Start_New_Temp_Measurement == TRUE) { 
                    StartTimer(T_SPI_MEASUREMENT);           
                    SPI_SS = 0;
                    Status_SPI = SPI_WAIT_READ_RESULTS;                
                }
            }
            else 
                Status_SPI = SPI_IDLE;
        break;

        // WAIT 200 msec before to Read results (at least 150msec)
        case SPI_WAIT_READ_RESULTS:
            if (StatusTimer(T_SPI_MEASUREMENT) == T_ELAPSED)
            {
                StopTimer(T_SPI_MEASUREMENT);
                SPI_SS = 1;
                Read_results[0] = 0x00;
                Read_results[1] = 0x00;
                Read_results[2] = 0x00 ;       
                Status_SPI = SPI_READ_RESULTS;                          
            }
        break;

        // Read Temperature 
        case SPI_READ_RESULTS:
            // Write Read Temperature Command to TC72
            writeBuffer[0] = 0x02; // MSB = 0, ADDRESS = 0x02        
            Read_SPI_Command (writeBuffer, Read_res);
            // Clock output for other 3 bytes
            writeBuffer[0] = 0x00;        
            Read_SPI_Command (writeBuffer, Read_res);
            Read_SPI_Command (writeBuffer, Read_res);
            Read_SPI_Command (writeBuffer, Read_res);            
            Read_results[0] = Read_res[0];
            Read_SPI_Command (writeBuffer, Read_res);            
            Read_results[1] = Read_res[0]>>6;
            Read_SPI_Command (writeBuffer, Read_res);            
            Read_results[2] = Read_res[0];
            if (Read_results[2] != 0x04) { 
                Sensor_Temp_Measurement_Error = TRUE;
                Status_SPI = SPI_HARD_RESET;          
            }
            else 
            {
                Sensor_Temp_Measurement_Error = FALSE;
                Status_SPI = SPI_CALCULATE_TEMPERATURE;          
            }
        break;

        // Calculate Absolute values from Raw data
        case SPI_CALCULATE_TEMPERATURE:
            // Temperature (°C) x 10            
            TC72_Raw_Temperature = Read_results[0] * 10 + (Read_results[1] * 25) / 10; 
            TC72_Temperature = TC72_Raw_Temperature; 
            Start_New_Temp_Measurement = FALSE;            
            Status_SPI = SPI_WRITE_MULTIPLE_BYTE_TRANSFER;            
        break;
        
        case SPI_HARD_RESET:
            Start_New_Temp_Measurement = FALSE;  
            Status_SPI = SPI_IDLE;
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
        reset_procedure = RESET_ON;
        return TRUE;
    }
    else 
    {
        switch (reset_procedure)
        {            
            case RESET_ON:
SPI1CON1L = 0x4000;                
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
SPI1CON1L = 0x8021;                    
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
