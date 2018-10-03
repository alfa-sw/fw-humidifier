#define TRUE 1
#define FALSE 0

#define DISABLE   0
#define ENABLE    1

// Watchdog control
#define ENABLE_WDT()                            \
  do {                                          \
    _SWDTEN = 1;                                \
  } while (0)

#define DISABLE_WDT()                           \
  do {                                          \
    _SWDTEN = 0;                                \
  } while (0)

// Default values for Humidifier 2.0
// -----------------------------------------------------------------------------
#define OFF 0
#define ON 1
#define DONE 2

#define HUMIDIFIER_DISABLE	0
#define HUMIDIFIER_ENABLE	1

#define HUMIDIFIER_TYPE_0	0 // SENSIRION SHT31
#define HUMIDIFIER_TYPE_1	1 // NO SENSOR - Process Humidifier 1.0
#define HUMIDIFIER_TYPE_2	2 // NO SENSOR - Process THOR

#define HUMIDIFIER_PWM      100 // WATER RESISTANCE for THOR process

#define HUMIDIFIER_PERIOD   1200 // 20 min
#define HUMIDIFIER_DURATION 2    // 2 sec

#define HUMIDIFIER_MULTIPLIER 100
#define MAX_HUMIDIFIER_MULTIPLIER 1000

#define AUTOCAP_OPEN_DURATION	10 // 10"
#define MAX_HUMIDIFIER_DURATION_AUTOCAP_OPEN 120 // 120"
#define AUTOCAP_OPEN_PERIOD		30 // 30"

#define TEMP_DISABLE		0
#define TEMP_ENABLE			1

#define TEMPERATURE_TYPE_0  0 // SENSIRION SHT31
#define TEMPERATURE_TYPE_1  1 // MICROCHIP TC72

#define TEMP_PERIOD			300 // 5 min
#define MIN_TEMP_PERIOD		10

#define TEMP_T_LOW			10
#define TEMP_T_HIGH			20
#define HEATER_TEMP         10
#define HEATER_HYSTERESIS   1 

#define NEBULIZER			1
#define POMPA    			2
#define TURN_LED            4
#define RISCALDATORE        8
        
#define OUTPUT_OFF			0
#define OUTPUT_ON			1

#define AUTOCAP_CLOSED		0
#define AUTOCAP_OPEN		1
#define AUTOCAP_ERROR		2

#define MEASUREMENT_OK      0
#define MEASUREMENT_ERROR   1

#define READ_OK      0
#define READ_ERROR   1

#define HUMIDIFIER_MAX_ERROR           5
#define DOSING_TEMPERATURE_MAX_ERROR   5

#define HUMIDIFIER_MAX_ERROR_DISABLE          20
#define DOSING_TEMPERATURE_MAX_ERROR_DISABLE  20
// -----------------------------------------------------------------------------

# define NEBULIZER_OFF()	\
do {                        \
	NEB = OFF;              \
} while (0)

# define NEBULIZER_ON()     \
do {                        \
	NEB = ON;               \
} while (0)

# define AIR_PUMP_OFF()     \
do {                        \
	PUMP = OFF;             \
} while (0)

# define AIR_PUMP_ON()      \
do {                        \
	PUMP = ON;              \
} while (0)

# define RISCALDATORE_OFF() \
do {                        \
	RISCALD = OFF;          \
} while (0)

# define RISCALDATORE_ON()  \
do {                        \
	RISCALD = ON;           \
} while (0)

# define isColorCmdStop()  		  (HumidifierAct.command.stop)
# define isColorCmdStopProcess()  (HumidifierAct.command.stop_process)
# define isColorCmdSetupParam()   (HumidifierAct.command.setup_param)
# define isColorCmdSetupOutput()  (HumidifierAct.command.setup_output)

// -----------------------------------------------------------------------------
// I2C1
#ifndef I2C1_CONFIG_TR_QUEUE_LENGTH
        #define I2C1_CONFIG_TR_QUEUE_LENGTH 1
#endif

#define I2C1_TRANSMIT_REG                       I2C1TRN                 // Defines the transmit register used to send data.
#define I2C1_RECEIVE_REG                        I2C1RCV                 // Defines the receive register used to receive data.

// The following control bits are used in the I2C state machine to manage
// the I2C module and determine next states.
#define I2C1_WRITE_COLLISION_STATUS_BIT         I2C1STATbits.IWCOL      // Defines the write collision status bit.
#define I2C1_ACKNOWLEDGE_STATUS_BIT             I2C1STATbits.ACKSTAT    // I2C ACK status bit.

#define I2C1_START_CONDITION_ENABLE_BIT         I2C1CONLbits.SEN         // I2C START control bit.
#define I2C1_REPEAT_START_CONDITION_ENABLE_BIT  I2C1CONLbits.RSEN        // I2C Repeated START control bit.
#define I2C1_RECEIVE_ENABLE_BIT                 I2C1CONLbits.RCEN        // I2C Receive enable control bit.
#define I2C1_STOP_CONDITION_ENABLE_BIT          I2C1CONLbits.PEN         // I2C STOP control bit.
#define I2C1_ACKNOWLEDGE_ENABLE_BIT             I2C1CONLbits.ACKEN       // I2C ACK start control bit.
#define I2C1_ACKNOWLEDGE_DATA_BIT               I2C1CONLbits.ACKDT       // I2C ACK data control bit.

#define RESET_ON    0
#define RESET_WAIT  1
        
#define WAIT 0   

// Max Write Command Retry Number with Sensor
#define SLAVE_I2C_GENERIC_RETRY_MAX 5
