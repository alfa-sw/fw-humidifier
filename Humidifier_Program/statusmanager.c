/*
*//*=====================================================================*//**
**
**      Nome del file  : statusmanager.c
**
**      Descrizione    : Gestore dei Processi
**
**      Progetto       : Alfa HUTBRD
**
*//*=====================================================================*//**
*/

#include "p24FJ64GA704.h"
#include "statusmanager.h"
#include "timerMg.h"
#include "serialcom.h"
#include "ram.h"
#include "gestio.h"
#include "define.h"
#include <xc.h>
/*
*//*=====================================================================*//**
**      @brief Initialization humidifier status
**
**      @param void
**
                                                                              * 
**      @retval void
**
*//*=====================================================================*//**
*/
void initStatusManager(void)
{
	Status.level = HUMIDIFIER_INIT_ST;
}


/*
*//*=====================================================================*//**
**      @brief Initialization humidifier parameters
**
**      @param void
**
**      @retval void
**
*//*=====================================================================*//**
*/
void initParam(void)
{
    // Humidifier process Enable / Disable
	HumidifierAct.Humidifier_Enable = HUMIDIFIER_DISABLE;
    // Humidifier Type
	HumidifierAct.Humdifier_Type = HUMIDIFIER_TYPE_0;
	// Starting Humidifier Period
    HumidifierAct.Humidifier_Period = HUMIDIFIER_PERIOD;
	// Humidifier Multiplier
    HumidifierAct.Humidifier_Multiplier = HUMIDIFIER_MULTIPLIER;
	// Humidifier Nebulizer and Pump Duration with AUTOCAP OPEN
    HumidifierAct.AutocapOpen_Duration = AUTOCAP_OPEN_DURATION;
	// Humidifier Nebulizer and Pump Period with AUTOCAP OPEN
    HumidifierAct.AutocapOpen_Period = AUTOCAP_OPEN_PERIOD;
    // Temperature controlled Dosing process Enable / Disable
	HumidifierAct.Temp_Enable = TEMP_DISABLE;
    // Temperature Type
	HumidifierAct.Temperature_Type = TEMPERATURE_TYPE_0;
	// Temperature controlled Dosing process Period 
    HumidifierAct.Temp_Period = TEMP_PERIOD;
	// LOW Temperature threshold value 
    HumidifierAct.Temp_T_LOW = TEMP_T_LOW;
	// HIGH Temperature threshold value 
    HumidifierAct.Temp_T_HIGH = TEMP_T_HIGH;
	// Heater Activation 
    HumidifierAct.Heater = HEATER_TEMP;
	// Heater Hysteresis 
    HumidifierAct.Heater_Hysteresis = HEATER_HYSTERESIS;

	HumidifierAct.Nebulizer_state = OFF;
	HumidifierAct.Pump_state = OFF;
    Start_New_Measurement = 0;
    Sensor_Measurement_Error = FALSE;
}

/*
*//*=====================================================================*//**
**      @brief Stop Humidifier Process: Nebulizer, Pump and Sensor Acquisition
**
**      @param void
**
**      @retval void
**
*//*=====================================================================*//**
*/
void StopHumidifier(void)
{
	NEBULIZER_OFF();
	AIR_PUMP_OFF();
//	StopSensor();
}

/*
*//*=====================================================================*//**
**      @brief Analyze Humidifier parameter received
**
**      @param void
**
**      @retval void
**
*//*=====================================================================*//**
*/
int AnalyzeParam(void)
{
    // Humidifier process Enable / Disable
	if ( (HumidifierAct.Humidifier_Enable != HUMIDIFIER_DISABLE) && (HumidifierAct.Humidifier_Enable != HUMIDIFIER_ENABLE) )
		return FALSE;
    // Humidifier Type
	else if ( (HumidifierAct.Humdifier_Type != HUMIDIFIER_TYPE_0) && (HumidifierAct.Humidifier_Enable == HUMIDIFIER_ENABLE) )
		return FALSE;
	// Humidifier Multiplier
    else if ( (HumidifierAct.Humidifier_Multiplier > MAX_HUMIDIFIER_MULTIPLIER) && (HumidifierAct.Humidifier_Enable == HUMIDIFIER_ENABLE) )
		return FALSE;
    else if ( (HumidifierAct.AutocapOpen_Duration > MAX_HUMIDIFIER_DURATION_AUTOCAP_OPEN) && (HumidifierAct.Humidifier_Enable == HUMIDIFIER_ENABLE) )
		return FALSE;
	// Period has to be >= Duration
	else if ( (HumidifierAct.AutocapOpen_Duration > HumidifierAct.AutocapOpen_Period) && (HumidifierAct.AutocapOpen_Period != 0) && 
              (HumidifierAct.Humidifier_Enable == HUMIDIFIER_ENABLE) )
		return FALSE;
	// Temperature controlled Dosing process Enable / Disable
	else if ( (HumidifierAct.Temp_Enable != TEMP_DISABLE) && (HumidifierAct.Temp_Enable != TEMP_ENABLE) )
		return FALSE;
    // Temperature Type
	else if ( (HumidifierAct.Temperature_Type != TEMPERATURE_TYPE_0) && (HumidifierAct.Temp_Enable == TEMP_ENABLE) )
		return FALSE;
	// Temperature controlled Dosing process Period 
    else if ( (HumidifierAct.Temp_Period < MIN_TEMP_PERIOD) && (HumidifierAct.Temp_Enable == TEMP_ENABLE) )
		return FALSE;
	else
	{
		// 1sec = 500
		Durata[T_HUM_CAP_OPEN_ON] = HumidifierAct.AutocapOpen_Duration * 500;	
		// Initial Duration = Period / 10
        Durata[T_HUM_CAP_CLOSED_ON] = (HumidifierAct.Humidifier_Period / 10) * 500;	
        Process_Period = (HumidifierAct.Humidifier_Period);
		return TRUE;
	}		
}

/*
*//*=====================================================================*//**
**      @brief Analyze Setup Humidifier Outputs
**
**      @param void
**
**      @retval void
**
*//*=====================================================================*//**
*/
int AnalyzeSetupOutputs(void)
{
    // Type of Peripheral: 0 = Nebulizer - 1 = Pump - 2 = LED
	if ( (PeripheralAct.Peripheral_Types.bytePeripheral != NEBULIZER) && (PeripheralAct.Peripheral_Types.bytePeripheral != POMPA) &&
         (PeripheralAct.Peripheral_Types.bytePeripheral != POMPA_AND_NEBULIZER) && (PeripheralAct.Peripheral_Types.bytePeripheral != LED_ON) )
		return FALSE;
	// Peripheral Action (ON / OFF)
	else if ( (PeripheralAct.Action != OUTPUT_OFF) && (PeripheralAct.Action != OUTPUT_ON) )
		return FALSE;
	else
		return TRUE;	
}
	
/*
*//*=====================================================================*//**
**      @brief Updates humidifier status
**
**      @param void
**
**      @retval void
**
*//*=====================================================================*//**
*/
void humidifierStatusManager(void)
{
	static long count_dosing_period = 0;
	static long count_humidifier_period = 0;
	static long count_humidifier_period_closed = 0;
	unsigned long Dos_Temperature;
	unsigned long Temperature, RH;
	static unsigned char Humidifier_Enable, Dos_Temperature_Enable;
	static unsigned long Process_Pump_Duration, Process_Neb_Duration;
	
	switch (Status.level)
	{
		// HUMIDIFIER_INIT
		// ------------------------------------------------------------------------------------------------------------
		case HUMIDIFIER_INIT_ST:
			Status.level = HUMIDIFIER_READY_ST;
			StopTimer(T_HUM_CAP_OPEN_ON);
			StopTimer(T_HUM_CAP_OPEN_PERIOD);
			StopTimer(T_HUM_CAP_CLOSED_ON);
			StopTimer(T_HUM_CAP_CLOSED_PERIOD);
			StopTimer(T_DOS_PERIOD);
			Humidifier_Enable = FALSE;
			Dos_Temperature_Enable = FALSE;
 
		break;
		// HUMIDIFIER_READY
		// ------------------------------------------------------------------------------------------------------------
		case HUMIDIFIER_READY_ST:
			StopHumidifier();
			HumidifierAct.Nebulizer_state = OFF;
			HumidifierAct.Pump_state = OFF;
			if (HumidifierAct.Humidifier_Enable == HUMIDIFIER_ENABLE)
			{
				Humidifier_Enable = TRUE;
				Status.step = STEP_0;
				count_humidifier_period = 0;
				count_humidifier_period_closed = 0;
				Status.level = HUMIDIFIER_RUN_ST;
			}
			if (HumidifierAct.Temp_Enable == TEMP_ENABLE)
			{
				Dos_Temperature_Enable = TRUE;
				count_dosing_period = 0;
				StartTimer(T_DOS_PERIOD);
				Status.level = HUMIDIFIER_RUN_ST;			
			}					
			// Check for NEW ommmands receivd
			// ------------------------------------------------------
			if(isColorCmdStopProcess() )
			{
				StopHumidifier();
				Status.level = HUMIDIFIER_READY_ST;
    			HumidifierAct.command.cmd = CMD_IDLE;			
			}
			else if (isColorCmdSetupParam() ) 
			{
				if (AnalyzeParam() == TRUE)
				{
                    NextStatus.level = HUMIDIFIER_READY_ST;
                    Status.level = HUMIDIFIER_PAR_RX;
				}
                else
					Status.level = HUMIDIFIER_BAD_PAR_ERROR;					

    			HumidifierAct.command.cmd = CMD_IDLE;			
			}	
			else if (isColorCmdSetupOutput() )
			{
				if (AnalyzeSetupOutputs() == FALSE)
					Status.level = HUMIDIFIER_BAD_PAR_ERROR;
				else 
                {
                    Status.level = HUMIDIFIER_PAR_RX;
                    StopHumidifier();
                    NextStatus.level = HUMIDIFIER_NEBULIZER_PUMP_ON_ST;
                    if (PeripheralAct.Peripheral_Types.bytePeripheral == NEBULIZER) 
                    {
                        if (PeripheralAct.Action == OUTPUT_ON)
                            HumidifierAct.Nebulizer_state = ON;
                        else 
                            HumidifierAct.Nebulizer_state = OFF;
                    }		
    				else if (PeripheralAct.Peripheral_Types.bytePeripheral == POMPA) 
        			{
    					if (PeripheralAct.Action == OUTPUT_ON)
        					HumidifierAct.Pump_state = ON;
    					else 
        					HumidifierAct.Pump_state = OFF;
        			}
    				else if (PeripheralAct.Peripheral_Types.bytePeripheral == POMPA_AND_NEBULIZER) 
        			{
    					if (PeripheralAct.Action == OUTPUT_ON)
                        {    
            				HumidifierAct.Pump_state = ON;
                            HumidifierAct.Nebulizer_state = ON;
                    	}
                        else 
                        {
                            HumidifierAct.Pump_state = OFF;
                            HumidifierAct.Nebulizer_state = OFF;
                        }
                    }                
        			HumidifierAct.command.cmd = CMD_IDLE;	
                }    
			}
			// ------------------------------------------------------
		break;		
		// HUMIDIFIER_RUN		
		// ------------------------------------------------------------------------------------------------------------
		case HUMIDIFIER_RUN_ST:
			
			// Humidifier process
			if (Humidifier_Enable == TRUE)
			{	
                // Check Water Level
				if (getWaterLevel() == OFF)
				{
					StopHumidifier();
					StopTimer(T_HUM_CAP_OPEN_ON);
					StopTimer(T_HUM_CAP_OPEN_PERIOD);
					StopTimer(T_HUM_CAP_CLOSED_ON);
					StopTimer(T_HUM_CAP_CLOSED_PERIOD);
					StopTimer(T_DOS_PERIOD);
					if (HumidifierAct.Temp_Enable == TEMP_ENABLE)
						StartTimer(T_DOS_PERIOD);
					HumidifierAct.Nebulizer_state = OFF;
					HumidifierAct.Pump_state = OFF;
					count_dosing_period = 0;
					count_humidifier_period = 0;
					count_humidifier_period_closed = 0;
					Status.level = HUMIDIFIER_TOO_LOW_WATER_LEVEL;
				}
				else
				{	
					// Multiplier = 1000 -> Punp and Nebulizer always ON
					if (HumidifierAct.Humidifier_Multiplier == 1000)
					{
						HumidifierAct.Nebulizer_state = ON;
						NEBULIZER_ON();
						HumidifierAct.Pump_state = ON;
						AIR_PUMP_ON();
					}
					// Multiplier = 0 --> Punp and Nebulizer always OFF
					else if (HumidifierAct.Humidifier_Multiplier == 0)
					{
						HumidifierAct.Nebulizer_state = OFF;
						NEBULIZER_OFF();
						HumidifierAct.Pump_state = OFF;
						AIR_PUMP_OFF();
					}
					else
					{	
						//  Manage Humidity Process
						switch (HumidifierAct.Autocap_Status)
						{
							// Autocap Closed
							case AUTOCAP_CLOSED:
								if ( (Status.step == STEP_0) || (Status.step == STEP_1))
								{	
									StopTimer(T_HUM_CAP_CLOSED_ON);
									StopTimer(T_HUM_CAP_CLOSED_PERIOD);
									StartTimer(T_HUM_CAP_CLOSED_ON);
									StartTimer(T_HUM_CAP_CLOSED_PERIOD);
									count_humidifier_period_closed = 0;
									HumidifierAct.Nebulizer_state = ON;
									NEBULIZER_ON();
									HumidifierAct.Pump_state = ON;
									AIR_PUMP_ON();
									Status.step = STEP_2;
								}
								else if (Status.step == STEP_2)
								{
									// Check Duration
									if (StatusTimer(T_HUM_CAP_CLOSED_ON) == T_ELAPSED)
									{
										StopTimer(T_HUM_CAP_CLOSED_ON);
										HumidifierAct.Nebulizer_state = OFF;
										NEBULIZER_OFF();
										HumidifierAct.Pump_state = OFF;
										AIR_PUMP_OFF();	
									}
									// Check Period
									if (StatusTimer(T_HUM_CAP_CLOSED_PERIOD) == T_ELAPSED) 
									{
										count_humidifier_period_closed++;
										StopTimer(T_HUM_CAP_CLOSED_PERIOD);				
										StartTimer(T_HUM_CAP_CLOSED_PERIOD);	
                                        pippo = count_humidifier_period_closed;
                                        pippo1 = Process_Period;
										if (count_humidifier_period_closed == Process_Period) 
										{
                                            //if (TRUE)
                                            if (AcquireHumidityTemperature(HumidifierAct.Humdifier_Type, &Temperature, &RH) == TRUE)
											{
												HumidifierAct.Temperature = Temperature;
												HumidifierAct.Dosing_Temperature = Temperature;
												HumidifierAct.RH = RH;
												HumidifierProcessCalculation(HumidifierAct.Humidifier_Multiplier,HumidifierAct.RH, HumidifierAct.Temperature, 
													&Process_Period, &Process_Pump_Duration, &Process_Neb_Duration);

                                                pippo = Process_Pump_Duration;
                                                pippo1 = Process_Period;
                                                // 1sec = 500
												Durata[T_HUM_CAP_CLOSED_ON] = Process_Pump_Duration * 500;	
												StartTimer(T_HUM_CAP_CLOSED_ON);
												count_humidifier_period_closed = 0;
												HumidifierAct.Nebulizer_state = ON;
												NEBULIZER_ON();
												HumidifierAct.Pump_state = ON;
												AIR_PUMP_ON();
											}
											else
											{	
												StopTimer(T_HUM_CAP_CLOSED_PERIOD);
												Status.level = HUMIDIFIER_RH_ERROR;
											}
										}
									}										
								}
							break;
							// Autocap Open
							case AUTOCAP_OPEN:
								// Period = 0 -> Punp and Nebulizer always ON
								if (HumidifierAct.AutocapOpen_Period == 0)
								{
									HumidifierAct.Nebulizer_state = ON;
									NEBULIZER_ON();
									HumidifierAct.Pump_state = ON;
									AIR_PUMP_ON();
								}
								// Duration = 0 AND Period != 0 -> Punp and Nebulizer always OFF
								else if (HumidifierAct.AutocapOpen_Duration == 0)
								{
									HumidifierAct.Nebulizer_state = OFF;
									NEBULIZER_OFF();
									HumidifierAct.Pump_state = OFF;
									AIR_PUMP_OFF();
								}
								// Duration > 0 AND Period > 0
								else
								{
									// Initialization
									if ( (Status.step == STEP_0) || (Status.step == STEP_2) )
									{	
										StopTimer(T_HUM_CAP_OPEN_ON);
										StopTimer(T_HUM_CAP_OPEN_PERIOD);
										StartTimer(T_HUM_CAP_OPEN_ON);
										StartTimer(T_HUM_CAP_OPEN_PERIOD);
										count_humidifier_period = 0;
										HumidifierAct.Nebulizer_state = ON;
										NEBULIZER_ON();
										HumidifierAct.Pump_state = ON;
										AIR_PUMP_ON();
										Status.step = STEP_1;
									}
									else if (Status.step == STEP_1)
									{
										// Check Duration
										if (StatusTimer(T_HUM_CAP_OPEN_ON) == T_ELAPSED)
										{
											StopTimer(T_HUM_CAP_OPEN_ON);
											HumidifierAct.Nebulizer_state = OFF;
											NEBULIZER_OFF();
											HumidifierAct.Pump_state = OFF;
											AIR_PUMP_OFF();	
										}
										// Check Period
										if (StatusTimer(T_HUM_CAP_OPEN_PERIOD) == T_ELAPSED) 
										{
											count_humidifier_period++;
											StopTimer(T_HUM_CAP_OPEN_PERIOD);				
											StartTimer(T_HUM_CAP_OPEN_PERIOD);	
											if (count_humidifier_period == HumidifierAct.AutocapOpen_Period) 
											{
												StartTimer(T_HUM_CAP_OPEN_ON);
												count_humidifier_period = 0;
												HumidifierAct.Nebulizer_state = ON;
												NEBULIZER_ON();
												HumidifierAct.Pump_state = ON;
												AIR_PUMP_ON();
											}
										}										
									}							
								}
							break;

							// Autocap Error				
							case AUTOCAP_ERROR:
								StopHumidifier();
								Humidifier_Enable = FALSE;
								if (Dos_Temperature_Enable == TRUE)
								{	
									count_dosing_period = 0;
									StopTimer(T_DOS_PERIOD);
									StartTimer(T_DOS_PERIOD);
								}								
							break;
					
							default:
                            break;
						}
					}	
				}	
			}
			// Dosing Temperature process
			else if (Dos_Temperature_Enable == TRUE)
			{
				if (StatusTimer(T_DOS_PERIOD) == T_ELAPSED) 
				{
					count_dosing_period++;
					StopTimer(T_DOS_PERIOD);			
					StartTimer(T_DOS_PERIOD);							
					if (count_dosing_period == HumidifierAct.Temp_Period) 
					{
						count_dosing_period = 0;
						if (AcquireTemperature(HumidifierAct.Temperature_Type, &Dos_Temperature) == TRUE)
							HumidifierAct.Dosing_Temperature = Dos_Temperature;
						else
						{	
							StopTimer(T_DOS_PERIOD);
							Status.level = HUMIDIFIER_TEMPERATURE_ERROR;
						}
					}
				}					
			}	
			// Check for NEW ommmands receivd
			// ------------------------------------------------------
			if(isColorCmdStopProcess() )
			{
				StopHumidifier();
				Status.level = HUMIDIFIER_READY_ST;
                HumidifierAct.command.cmd = CMD_IDLE;						
			}
			else if (isColorCmdSetupParam() ) 
			{
				if (AnalyzeParam() == TRUE)
                {
                    NextStatus.level = HUMIDIFIER_READY_ST;				
                    Status.level = HUMIDIFIER_PAR_RX;
                }    
				else
					Status.level = HUMIDIFIER_BAD_PAR_ERROR;					
			
            	HumidifierAct.command.cmd = CMD_IDLE;						
            }	
			else if (isColorCmdSetupOutput() )
			{
				if (AnalyzeSetupOutputs() == FALSE)
					Status.level = HUMIDIFIER_BAD_PAR_ERROR;
				else 
                {
                    Status.level = HUMIDIFIER_PAR_RX;
                    StopHumidifier();
                    NextStatus.level = HUMIDIFIER_NEBULIZER_PUMP_ON_ST;
                    if (PeripheralAct.Peripheral_Types.bytePeripheral == NEBULIZER) 
                    {
                        if (PeripheralAct.Action == OUTPUT_ON)
                            HumidifierAct.Nebulizer_state = ON;
                        else 
                            HumidifierAct.Nebulizer_state = OFF;
                    }		
    				else if (PeripheralAct.Peripheral_Types.bytePeripheral == POMPA) 
        			{
    					if (PeripheralAct.Action == OUTPUT_ON)
        					HumidifierAct.Pump_state = ON;
    					else 
        					HumidifierAct.Pump_state = OFF;
        			}
    				else if (PeripheralAct.Peripheral_Types.bytePeripheral == POMPA_AND_NEBULIZER) 
        			{
    					if (PeripheralAct.Action == OUTPUT_ON)
                        {    
            				HumidifierAct.Pump_state = ON;
                            HumidifierAct.Nebulizer_state = ON;
                    	}
                        else 
                        {
                            HumidifierAct.Pump_state = OFF;
                            HumidifierAct.Nebulizer_state = OFF;
                        }
                    }                
        			HumidifierAct.command.cmd = CMD_IDLE;	
                }    
			}
			// ------------------------------------------------------
				
		break;		
		// HUMIDIFIER_NEBULIZER_PUMP ON	
		// ------------------------------------------------------------------------------------------------------------
		case HUMIDIFIER_NEBULIZER_PUMP_ON_ST:
			if (HumidifierAct.Pump_state == ON)
				AIR_PUMP_ON();
			else 
				AIR_PUMP_OFF();
			
			if (HumidifierAct.Nebulizer_state == ON)
				NEBULIZER_ON();
			else 
				NEBULIZER_OFF();
			
			// Check for NEW ommmands receivd
			// ------------------------------------------------------
			if(isColorCmdStopProcess() )
			{
				StopHumidifier();
				Status.level = HUMIDIFIER_READY_ST;
			}
			else if (isColorCmdSetupOutput() )
			{
				if (AnalyzeSetupOutputs() == FALSE)
					Status.level = HUMIDIFIER_BAD_PAR_ERROR;
				else if (PeripheralAct.Peripheral_Types.bytePeripheral == NEBULIZER) 
				{
					if (PeripheralAct.Action == OUTPUT_ON)
						HumidifierAct.Nebulizer_state = ON;
					else
						HumidifierAct.Nebulizer_state = OFF;
				}		
				else if (PeripheralAct.Peripheral_Types.bytePeripheral == POMPA) 
				{
					if (PeripheralAct.Action == OUTPUT_ON)
						HumidifierAct.Pump_state = ON;
					else 
						HumidifierAct.Pump_state = OFF;
				}		
				else if (PeripheralAct.Peripheral_Types.bytePeripheral == POMPA_AND_NEBULIZER) 
				{
					if (PeripheralAct.Action == OUTPUT_ON) 
                    {
						HumidifierAct.Pump_state = ON;
    					HumidifierAct.Nebulizer_state = ON;
                    }
                    else 
                    {
						HumidifierAct.Pump_state = OFF;
                        HumidifierAct.Nebulizer_state = OFF;
                    }
                    Status.level = HUMIDIFIER_PAR_RX;
                    NextStatus.level = HUMIDIFIER_NEBULIZER_PUMP_ON_ST;                    
                }		
			}
			HumidifierAct.command.cmd = CMD_IDLE;						
			// ------------------------------------------------------			
		break;		
		// HUMIDIFIER_PARAMETER CORRECTLY RECEIVED		
		// ------------------------------------------------------------------------------------------------------------
		case HUMIDIFIER_PAR_RX:
			if (isColorCmdStop())
                Status.level = NextStatus.level;
            else if (isColorCmdStopProcess())
				Status.level = HUMIDIFIER_READY_ST;                
		break;	
		// HUMIDIFIER_TOO LOW WATER LEVEL
		// ------------------------------------------------------------------------------------------------------------
		case HUMIDIFIER_TOO_LOW_WATER_LEVEL:
			if (getWaterLevel() == ON) {
				// Gestione Lampeggio LED
				StopTimer(T_DOS_PERIOD);
				count_dosing_period = 0;
				Status.level = HUMIDIFIER_READY_ST;
			}
			// Manage Dosing Temperature process if activated
			if (HumidifierAct.Temp_Enable == TEMP_ENABLE) 
			{
				if (StatusTimer(T_DOS_PERIOD) == T_ELAPSED) 
				{
					count_dosing_period++;
					StopTimer(T_DOS_PERIOD);			
					StartTimer(T_DOS_PERIOD);							
					if (count_dosing_period == HumidifierAct.Temp_Period) 
					{
						count_dosing_period = 0;
						if (AcquireTemperature(HumidifierAct.Temperature_Type, &Dos_Temperature) == TRUE)
							HumidifierAct.Dosing_Temperature = Dos_Temperature;
						else
						{	
							StopTimer(T_DOS_PERIOD);
							Status.level = HUMIDIFIER_TEMPERATURE_ERROR;
						}
					}
				}					
			}
			// Check for NEW ommmands receivd
			// ------------------------------------------------------
			if(isColorCmdStopProcess() )
			{
				StopTimer(T_DOS_PERIOD);
				count_dosing_period = 0;
				StopHumidifier();
				Status.level = HUMIDIFIER_READY_ST;
			}
			else if (isColorCmdSetupParam() ) 
			{
				if (AnalyzeParam() == TRUE)
                {
                    NextStatus.level = HUMIDIFIER_READY_ST;				
                    Status.level = HUMIDIFIER_PAR_RX;
                }    
				else
					Status.level = HUMIDIFIER_BAD_PAR_ERROR;					
			
            	HumidifierAct.command.cmd = CMD_IDLE;						
            }	            
			else if (isColorCmdSetupOutput() )
			{
				StopTimer(T_DOS_PERIOD);
				count_dosing_period = 0;
				if (AnalyzeSetupOutputs() == FALSE)
					Status.level = HUMIDIFIER_BAD_PAR_ERROR;
				else 
                {
					StopHumidifier();
                    Status.level = HUMIDIFIER_PAR_RX;
                    NextStatus.level = HUMIDIFIER_NEBULIZER_PUMP_ON_ST;
                    if (PeripheralAct.Peripheral_Types.bytePeripheral == NEBULIZER) 
        			{
            			if (PeripheralAct.Action == OUTPUT_ON)
                			HumidifierAct.Nebulizer_state = ON;
                    	else 
                        	HumidifierAct.Nebulizer_state = OFF;
                    }		
                    else if (PeripheralAct.Peripheral_Types.bytePeripheral == POMPA) 
                    {
                        if (PeripheralAct.Action == OUTPUT_ON)
                            HumidifierAct.Pump_state = ON;
                        else 
                            HumidifierAct.Pump_state = OFF;
                    }
                    else if (PeripheralAct.Peripheral_Types.bytePeripheral == POMPA_AND_NEBULIZER) 
                    {
                        if (PeripheralAct.Action == OUTPUT_ON)
                        {    
                            HumidifierAct.Pump_state = ON;
                            HumidifierAct.Nebulizer_state = ON;
                        }
                        else 
                        {
                            HumidifierAct.Pump_state = OFF;
                            HumidifierAct.Nebulizer_state = OFF;
                        }
                    }
                }    
			}
			HumidifierAct.command.cmd = CMD_IDLE;						
			// ------------------------------------------------------			
		break;	
		// HUMIDIFIER_ERROR
		// ------------------------------------------------------------------------------------------------------------
		case HUMIDIFIER_BAD_PAR_ERROR:		
		case HUMIDIFIER_RH_ERROR:
		case HUMIDIFIER_TEMPERATURE_ERROR:
			// Error condition detected, stop Nebulizer,Pump and Sensor Acquisition
			StopHumidifier();	
			if(isColorCmdStopProcess() )
				Status.level = HUMIDIFIER_READY_ST;
		break;	
		
		default:
			Status.level = HUMIDIFIER_INIT_ST;
		
	}
}

/*
*//*=====================================================================*//**
**      @brief Temperature Measurement
**
**      @param unsigned char Temp_Type --> Type of Sensor: 0 = Sensirion SHT31
**			   unsigned long *Temp	   --> Temperature Measurement
**
**      @retval bool --> TRUE  = good measurement
**					 --> FALSE = bad measurement
**
*//*=====================================================================*//**
*/
int AcquireTemperature(unsigned char Temp_Type, unsigned long *Temp)
{
	switch (Temp_Type)
	{
		// SHT31
		case 0:
/*            
			if (Start_New_Measurement == 0) 
                Start_New_Measurement = 1;
            if (Sensor_Measurement_Error == FALSE)
            {
                *Temp = SHT31_Temperature;
    			return TRUE;
            }
            else
    			return FALSE;
*/
    	return TRUE;

        break;
		
		default:
			return FALSE;
	}	
}

/*
*//*=====================================================================*//**
**      @brief Humidity Temperature Measurement
**
**      @param unsigned char Temp_Type --> Type of Sensor: 0 = Sensirion SHT31
**			   unsigned long *Temp	   --> Temperature Measurement
**
**      @retval bool --> TRUE  = good measurement
**					 --> FALSE = bad measurement
**
*//*=====================================================================*//**
*/
int AcquireHumidityTemperature(unsigned char Temp_Type, unsigned long *Temp, unsigned long *Humidity)
{
	switch (Temp_Type)
	{
		// SHT31
		case 0:
			if (Start_New_Measurement == 0) 
                Start_New_Measurement = 1;
            if (Sensor_Measurement_Error == FALSE)
            {
                *Temp = SHT31_Temperature;
                *Humidity = SHT31_Humidity;
    			return TRUE;
            }
            else
    			return FALSE;
		break;
		
		default:
			return FALSE;
	}	
}

/*
*//*=====================================================================*//**
**      @brief Humidity Temperature Measurement
**
**      @param unsigned long Multiplier  --> 0 - 1000
**			   unsigned long RH --> RH Humiity 			   
**			   unsigned long Temperature --> Temperature x 10 
**
**			   unsigned long Period --> Period of Humidifier Process
**			   unsigned long Pump_Duration --> Pump activation Duration of Humidifier Process
**			   unsigned long Neb_Duration --> Nebulizer activation Duration of Humidifier Process				
**
**      @retval void
**					 
**
*//*=====================================================================*//**
*/
void HumidifierProcessCalculation(unsigned long Multiplier, unsigned long RH, unsigned long Temperature, 
														unsigned long *Period, unsigned long *Pump_Duration, unsigned long *Neb_Duration)
{
	float KT1, KH1, KT2, KH2, KBoostT, KBoostH, KBoostTH, KRT, KRH, Temp, RH_Humidity, KS;
	
	Temp = (float)Temperature/10;
	RH_Humidity = (float)RH/1000;
	KS = (float)Multiplier/100;
	
	// KT1 = variabile di temperatura per definizione del "Period"
	// Calcolo KT1
	if (Temp < 10)
		KT1 = 0.75;
	else if (Temp > 30)
		KT1 = 0.25;
	else
		KT1 = 0.5 + (20 - Temp) * 0.025;

	// KH1 = variabile di umidità per definizione del "Period"
	// Calcolo KH1
	if (RH_Humidity < 0.4)
		KH1 = 0.25;
	else if (RH_Humidity > 0.8)
		KH1 = 0.75;
	else
		KH1 = 0.5 -(0.6 - RH_Humidity) * 5 / 4;

	// "Period = tempo all'interno del quale si calcola il tempo di funzionamento della pompa
	// e del Nebulizzatore dell'acuqa della bottiglia
	// Calcolo "Period"
	*Period = 30 * (KT1 + KH1);

	// KT2 = variabile di temperatura per definizione di "Pump_Duration"
	// Calcolo KT2
	if (Temp < 10)
		KT2 = 0.5;
	else if (Temp > 30)
		KT2 = 1.5;
	else
		KT2 = 0.5 + (Temp - 10) / 20;

	// KH2 = variabile di umidità per definizione del "Pump_Duration"
	// Calcolo KH2
	if (RH_Humidity < 0.4)
		KH2 = 0.6;
	else if (RH_Humidity > 0.8)
		KH2 = 0.2;
	else
		KH2 = 1 - RH_Humidity;

	// KBoostT = variabile di temperatura per aumentare il "Period" in un determinato range di T
	// Calcolo KBoostT
	if (Temp < 30)
		KBoostT = 0.25;
	else if (Temp > 40)
		KBoostT = 0.5;
	else
		KBoostT = 0.25 + (Temp - 30) / 40;

	// KBoostH = variabile di umidità per aumentare il "Period" in un determinato range di H
	// Calcolo KBoostH 
	if (RH_Humidity < 0.2)
		KBoostH = 0.75;
	else if (RH_Humidity > 0.4)
		KBoostH = 0.25;
	else
		KBoostH = 0.25 + (0.4 - RH_Humidity) * 2.5;

	// Calcolo KBoostTH
	KBoostTH = KBoostT + KBoostH;

	// KRT = variabile di temperatura per aumentare "Neb_Duration" in un determinato range di T
	// Calcolo KRT 
	if (Temp < 30)
		KRT = 0.0;
	else if (Temp > 40)
		KRT = 0.45;
	else
		KRT = (Temp - 30) * 4.5 / 100;

	// KRH = variabile di umidità per aumentare il "Neb_Duration" in un determinato range di H
	// Calcolo KRH 
	if (RH_Humidity < 0.2)
		KRH = 0.45;
	else if (RH_Humidity > 0.5)
		KRH = 0.0;
	else
		KRH = (0.5 - RH_Humidity) * 1.5;

	// "Pump_Duration" = tempo durante il quale la pompa rimane accesa
	// Calcolo "Pump_Duration"
	*Pump_Duration = *Period * KT2 * KH2 * KBoostTH * KS;
	if (*Pump_Duration > *Period)
		*Pump_Duration = *Period;

	// "Neb_Duration" = tempo durante il quale il Nebulizzatore dell'acqua della bottiglia rimane acceso
	// Calcolo "Neb_Duration"
	*Neb_Duration = *Pump_Duration * (0.1 + KRT + KRH);
}