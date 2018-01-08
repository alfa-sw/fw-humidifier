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
// ------------------------------------------------------
#define OFF 0
#define ON 1

#define HUMIDIFIER_DISABLE	0
#define HUMIDIFIER_ENABLE	1

#define HUMIDIFIER_TYPE_0	0 // SENSIRION SHT30

#define HUMIDIFIER_PERIOD   300 // 5 min

#define HUMIDIFIER_MULTIPLIER 100
#define MAX_HUMIDIFIER_MULTIPLIER 1000

#define AUTOCAP_OPEN_DURATION	10 // 10"
#define MAX_HUMIDIFIER_DURATION_AUTOCAP_OPEN 120 // 120"
#define AUTOCAP_OPEN_PERIOD		30 // 30"

#define TEMP_DISABLE		0
#define TEMP_ENABLE			1

#define TEMPERATURE_TYPE_0  0 // SENSIRION SHT30

#define TEMP_PERIOD			300 // 5 min
#define MIN_TEMP_PERIOD		10

#define TEMP_T_LOW			10

#define TEMP_T_HIGH			20

#define NEBULIZER			0
#define POMPA    			1

#define OUTPUT_OFF			0
#define OUTPUT_ON			1

#define AUTOCAP_CLOSED		0
#define AUTOCAP_OPEN		1
#define AUTOCAP_ERROR		2
// ------------------------------------------------------

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

# define isColorCmdStop()  		  (HumidifierAct.command.stop)
# define isColorCmdStopProcess()  (HumidifierAct.command.stop_process)
# define isColorCmdSetupParam()   (HumidifierAct.command.setup_param)
# define isColorCmdSetupOutput()  (HumidifierAct.command.setup_output)

