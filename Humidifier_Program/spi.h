#ifndef SPI_H
#define	SPI_H

typedef enum
{
    SPI_IDLE,
    SPI_WRITE_MULTIPLE_BYTE_TRANSFER,
    SPI_WAIT_READ_RESULTS,
    SPI_READ_RESULTS,
    SPI_CALCULATE_TEMPERATURE,
    SPI_HARD_RESET    
} SPI1_ACQUISITION_STATUS;

extern void SPI1_Initialize(void);
extern void SPI_Manager (void);
extern unsigned int TemperatureResetProcedure (unsigned char type);
extern void Write_SPI_Command( uint8_t *pTransmitData);

#endif	/* SPI_H */

