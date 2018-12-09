//*********************************************************************************************************************
//INCLUDES
//*********************************************************************************************************************

#include "chip.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include <cr_section_macros.h>


//*********************************************************************************************************************
//DEFINES
//*********************************************************************************************************************

#define PORT(x) 	((uint8_t) x)
#define PIN(x)		((uint8_t) x)

#define OUTPUT		((uint8_t) 1)
#define INPUT		((uint8_t) 0)

#define ON		((uint8_t) 1)
#define OFF		((uint8_t) 0)

#define SENS_MOV	PORT(0),PIN(4)
#define PULS_ONOFF	PORT(0),PIN(5)
#define LED_STICK 	PORT(0),PIN(22)

//I2C
#define PUERTO_0  	 		0
#define I2C1				1
#define PIN_SDA1 	   	   19
#define PIN_SCL1  	   	   20
#define NEITHER				2
#define I2C_FUNC     		3
#define SLAVE_ADDRESS		0x58 	// 1010 bits x default de la memorio 1000 los primeros 3 son el direccionamiento fisico
#define W_ADDRESS			0x0000 	// posicion donde voy a escribir
#define DATO				0xAA

#define TICKRATE_SEG1 (1)		//1 tick = 1 segundo

//UART3

#define TXD3	PORT(0),PIN(0)
#define RXD3	PORT(0),PIN(1)
#define COD_PRE		0x01
#define COD_TEMP	0x02
#define COD_MOV		0x03



//ADC

#define SENS_PRE	0 //canales del adc
#define SENS_TEMP	1
#define ADC_CH0		PORT(0),PIN(23)
#define	ADC_CH1		PORT(0),PIN(24)


#define LIMITE_TEMP	1861	//1.5V son 1861 cuentas
#define LIMITE_PRE	2481	//2V son 2481 cuentas

//*********************************************************************************************************************
//DECLARACIONES
//*********************************************************************************************************************

SemaphoreHandle_t Semaforo_1Seg;
SemaphoreHandle_t Semaforo_1SegB;
SemaphoreHandle_t Semaforo_10SegA;
SemaphoreHandle_t Semaforo_10SegB;
SemaphoreHandle_t Semaforo_30Min;


QueueHandle_t ColaADCTemp;
QueueHandle_t ColaADCPre;
QueueHandle_t Cola10segA;
QueueHandle_t Cola10segB;
QueueHandle_t ColaEEprom;

static ADC_CLOCK_SETUP_T ADCSetup;

void Envio_Datos 		(LPC_USART_T *UART, const char *Data)
{
}
void Envio_Emergencia 	(LPC_USART_T *UART, const char Codigo)
{
}

//*********************************************************************************************************************

void I2C1_IRQHandler(void)
{
	Chip_I2C_MasterStateHandler (I2C1);
}


//*********************************************************************************************************************

void ADC_IRQHandler(void)
{
	BaseType_t testigo = pdFALSE;
	uint16_t dataADC;

	if(Chip_ADC_ReadValue(LPC_ADC, SENS_TEMP, &dataADC))//si da 1 significa que la conversión estaba realizada
	{
		xQueueSendToBackFromISR(ColaADCTemp, &dataADC, &testigo);
		portYIELD_FROM_ISR(testigo);
	}
	if(Chip_ADC_ReadValue(LPC_ADC, SENS_PRE, &dataADC))//si da 1 significa que la conversión estaba realizada
	{
		xQueueSendToBackFromISR(ColaADCPre, &dataADC, &testigo);
		portYIELD_FROM_ISR(testigo);
	}
}
//*********************************************************************************************************************

/* TIMER0_IRQHandler:
 * Controlador del TIMER0
*/
void TIMER0_IRQHandler(void)
{
	BaseType_t Testigo=pdFALSE,TestigoB=pdFALSE;

	if (Chip_TIMER_MatchPending(LPC_TIMER0, 0))
	{
		Chip_TIMER_ClearMatch(LPC_TIMER0, 0);				//Resetea match

		xSemaphoreGiveFromISR(Semaforo_1Seg, &Testigo);		//Devuelve si una de las tareas bloqueadas tiene mayor prioridad que la actual
		xSemaphoreGiveFromISR(Semaforo_1SegB, &TestigoB);		//Devuelve si una de las tareas bloqueadas tiene mayor prioridad que la actual

		portYIELD_FROM_ISR(Testigo);						//Si testigo es TRUE -> ejecuta el scheduler
		portYIELD_FROM_ISR(TestigoB);						//Si testigo es TRUE -> ejecuta el scheduler

	}
}


//*********************************************************************************************************************

static void ADC_Config(void *pvParameters)
{
	Chip_IOCON_PinMux(LPC_IOCON, ADC_CH0, IOCON_MODE_INACT, IOCON_FUNC3);
	Chip_IOCON_PinMux(LPC_IOCON, ADC_CH1, IOCON_MODE_INACT, IOCON_FUNC3);

	Chip_ADC_Init(LPC_ADC, &ADCSetup);

	Chip_ADC_EnableChannel(LPC_ADC, SENS_TEMP, ENABLE);
	Chip_ADC_EnableChannel(LPC_ADC, SENS_PRE, ENABLE);

	Chip_ADC_SetSampleRate(LPC_ADC, &ADCSetup, 50000);

	Chip_ADC_Int_SetChannelCmd(LPC_ADC, SENS_TEMP, ENABLE);
	Chip_ADC_Int_SetChannelCmd(LPC_ADC, SENS_PRE, ENABLE);

	Chip_ADC_SetBurstCmd(LPC_ADC, DISABLE);

	NVIC_ClearPendingIRQ(ADC_IRQn);
	NVIC_EnableIRQ(ADC_IRQn);

	Chip_ADC_SetStartMode (LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);

	vTaskDelete(NULL);
}


//*********************************************************************************************************************

static void I2C_Config(void *pvParameters)
{
	Chip_IOCON_PinMux (LPC_IOCON, PUERTO_0, PIN_SDA1 , NEITHER , I2C_FUNC );
	Chip_IOCON_PinMux (LPC_IOCON, PUERTO_0, PIN_SCL1 , NEITHER , I2C_FUNC );
	Chip_IOCON_EnableOD (LPC_IOCON, PUERTO_0, PIN_SDA1);
	Chip_IOCON_EnableOD (LPC_IOCON, PUERTO_0, PIN_SCL1);

	Chip_I2C_Init (I2C1);
	Chip_I2C_SetClockRate (I2C1, 100000);

	Chip_I2C_SetMasterEventHandler (I2C1, Chip_I2C_EventHandler);
	NVIC_EnableIRQ(I2C1_IRQn);

	vTaskDelete(NULL);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

static void xTaskUART3Config(void *pvParameters)
{
	//Inicializacion de los pines de la UART1
	Chip_GPIO_SetDir (LPC_GPIO, RXD3, INPUT);
	Chip_IOCON_PinMux (LPC_IOCON, RXD3, IOCON_MODE_INACT, IOCON_FUNC1);
	Chip_GPIO_SetDir (LPC_GPIO, TXD3, OUTPUT);
	Chip_IOCON_PinMux (LPC_IOCON, TXD3, IOCON_MODE_INACT, IOCON_FUNC1);

	/* Setup UART for 9600 */
	Chip_UART_Init(LPC_UART3);
	Chip_UART_SetBaud(LPC_UART3, 9600);
	Chip_UART_ConfigData(LPC_UART3, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(LPC_UART3, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(LPC_UART3);

	/* Reset and enable FIFOs, FIFO trigger level 3 (14 chars) */
	Chip_UART_SetupFIFOS(LPC_UART3, (UART_FCR_FIFO_EN | UART_FCR_RX_RS |
							UART_FCR_TX_RS | UART_FCR_TRG_LEV3));

	//Habilito interrupcion UART3
	NVIC_EnableIRQ(UART3_IRQn);

	vTaskDelete(NULL);	//Borra la tarea
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* vTaskInicTimer:
 * Tarea que se encarga de inicializar el TIMER0 y luego se autoelimina
*/
static void vTaskInicTimer(void *pvParameters)
{
	while (1)
	{

		/* Enable timer 1 clock */
		Chip_TIMER_Init(LPC_TIMER0);	//Enciende el modulo

		Chip_TIMER_Reset(LPC_TIMER0);									//Borra la cuenta

		Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_TIMER0, SYSCTL_CLKDIV_8); // para contar un segundo pongo el moduolo 15MHz

		//MATCH 0: RESETEA LA CUENTA
		Chip_TIMER_MatchEnableInt(LPC_TIMER0, 0);						//Habilita interrupcion del match 0 timer 0
		Chip_TIMER_SetMatch(LPC_TIMER0, 0, ((SystemCoreClock / 8) / TICKRATE_SEG1));	//Le asigna un valor al match - seteo la frec a la que quiero que el timer me interrumpa (Ej 5min)
		Chip_TIMER_ResetOnMatchEnable(LPC_TIMER0, 0);					//Cada vez que llega al match resetea la cuenta

		Chip_TIMER_Enable(LPC_TIMER0);									//Comienza a contar
		/* Enable timer interrupt */ 		//El NVIC asigna prioridades de las interrupciones (prioridad de 0 a inf)
		NVIC_ClearPendingIRQ(TIMER0_IRQn);
		NVIC_EnableIRQ(TIMER0_IRQn);		//Enciende la interrupcion que acabamos de configurar

		vTaskDelete(NULL);	//Borra la tarea, no necesitaria el while(1)
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void taskAnalisis(void *pvParameters)
{
	uint16_t datoTemp, datoPre;
	uint8_t PrevioPuls, cont_10seg=0,cont_5seg=0;

	while(1)
	{
		xQueueReceive(ColaADCTemp, &datoTemp, portMAX_DELAY);
		xQueueReceive(ColaADCPre, &datoPre, portMAX_DELAY);

		//limites
		if(datoTemp > LIMITE_TEMP)
		{
			Envio_Emergencia(LPC_UART3,COD_TEMP);//aviso de emergencia temperatura
		}

		if(datoPre > LIMITE_PRE)
		{
			Envio_Emergencia(LPC_UART3,COD_PRE);//aviso de emergencia presion
		}

		//Cigueña
		if(Chip_GPIO_GetPinState (LPC_GPIO, PULS_ONOFF)==ON && PrevioPuls == OFF)		//si vario la cigueña
		{
			cont_5seg=0;
			PrevioPuls = ON;
		}
		else if(Chip_GPIO_GetPinState(LPC_GPIO, PULS_ONOFF)==OFF && PrevioPuls == ON)	//si vario la cigueña
		{
			cont_5seg=0;
			PrevioPuls = OFF;
		}
		else
		{
			cont_5seg++;
			if(cont_5seg == 5)
			{
				Envio_Emergencia (LPC_UART3,COD_MOV);//aviso de emergencia presion
			}
		}

		cont_10seg++;
		if(cont_10seg == 10)
		{
			cont_10seg = 0;

			xQueueSendToBack(Cola10segB, &datoPre, portMAX_DELAY);
			xQueueSendToBack(Cola10segB, &datoTemp, portMAX_DELAY);
			xSemaphoreGive(Semaforo_10SegB);
		}

		//conversion
		Chip_ADC_SetStartMode (LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
		xSemaphoreTake(Semaforo_1SegB, portMAX_DELAY);//espero que pase un segundo del timer 0

	}
	vTaskDelete(NULL);	//Borra la tarea
}
//*********************************************************************************************************************
static void vTaskEEprom(void *pvParameters)
{
	uint8_t i=0;
	unsigned char Datos_Tx [6] = { (W_ADDRESS & 0xFF00) >> 8 , W_ADDRESS & 0x00FF};

	while(1)
	{
		xSemaphoreTake(Semaforo_30Min, portMAX_DELAY);
		for(i=2; i<6; i++)
		{
			xQueueReceive(ColaEEprom, &Datos_Tx[i], portMAX_DELAY);//recibo el dato y lo hago pongo despues de la direccion
		}
		Chip_I2C_MasterSend (I2C1, SLAVE_ADDRESS, Datos_Tx,6); //Selecciono el lugar y escribo el dato.
	}
	vTaskDelete(NULL);	//Borra la tarea
}


//*********************************************************************************************************************

static void vTaskPulsadores(void *pvParameters)
{
	uint8_t PrevioPuls = ON;
	uint8_t hora, min ,Send, seg;
	uint16_t dia;
	unsigned char Datos_Tx [] = { (W_ADDRESS & 0xFF00) >> 8 , W_ADDRESS & 0x00FF} ,Datos_Rx[4];

    Chip_I2C_MasterSend (I2C1, SLAVE_ADDRESS, Datos_Tx, 2); // Nos posicionamos en la Posicion a leer.
    Chip_I2C_MasterRead (I2C1, SLAVE_ADDRESS, Datos_Rx, 4); // Se leen 04 bytes a partir de la Posición.

    dia = (Datos_Rx[0] << 8) + (Datos_Rx[1]); // leo por primera vez
    hora = Datos_Rx[2];
    min = Datos_Rx[3];

	while(1)
	{
		//Pulsador On-Off
		if(Chip_GPIO_GetPinState(LPC_GPIO, PULS_ONOFF)==ON && PrevioPuls == OFF)
		{
			//EQUIPO Encendido
			Chip_TIMER_Reset(LPC_TIMER0);		//Borra la cuenta
			NVIC_EnableIRQ(TIMER0_IRQn);		//Enciende la interrupcion para que vuelva a contar cada 1 seg
			PrevioPuls = ON; //actualizo el estado del pulsador
		}
		else if(Chip_GPIO_GetPinState(LPC_GPIO, PULS_ONOFF)==OFF && PrevioPuls == ON)
		{
			//EQUIPO Apagado

			NVIC_ClearPendingIRQ(TIMER0_IRQn);
			NVIC_ClearPendingIRQ(ADC_IRQn);

			NVIC_DisableIRQ(TIMER0_IRQn);	//Deshabilito la interrupcion para que deje de contar
			NVIC_DisableIRQ(ADC_IRQn);		//Deshabilito la interrupcion para que deje de convertir
			PrevioPuls = OFF; //actualizo el estado del pulsador

			//guardo los datos
			Send = (dia & 0xFF00) >> 8;//parte alta
			xQueueSendToBack(ColaEEprom, &Send, portMAX_DELAY);
			Send = (dia & 0x00FF);//parte baja
			xQueueSendToBack(ColaEEprom, &Send, portMAX_DELAY);
			xQueueSendToBack(ColaEEprom, &hora, portMAX_DELAY);
			xQueueSendToBack(ColaEEprom, &min, portMAX_DELAY);
			xSemaphoreGive(Semaforo_30Min);//una vez que cargue todo en la cola procedo a guardarlo en la eeprom
		}

		xSemaphoreTake(Semaforo_1Seg, portMAX_DELAY);//espero que pase un segundo del timer 0

		//reloj
		seg ++;
		if(seg == 60)
		{
			seg=0;
			min++;
			if(min == 60)
			{
				min=0;
				hora++;
				if(hora == 24)
				{
					hora=0;
					dia++;
				}
			}
		}

		if(min == 0 || min == 30)
		{

			Send = (dia & 0xFF00) >> 8;//parte alta
			xQueueSendToBack(ColaEEprom, &Send, portMAX_DELAY);
			Send = (dia & 0x00FF);//parte baja
			xQueueSendToBack(ColaEEprom, &Send, portMAX_DELAY);
			xQueueSendToBack(ColaEEprom, &hora, portMAX_DELAY);
			xQueueSendToBack(ColaEEprom, &min, portMAX_DELAY);
			xSemaphoreGive(Semaforo_30Min);//una vez que cargue todo en la cola procedo a guardarlo en la eeprom
		}

		if((seg%10) == 0 )
		{

			Send = (dia & 0xFF00) >> 8;//parte alta
			xQueueSendToBack(Cola10segA, &Send, portMAX_DELAY);
			Send = (dia & 0x00FF);//parte baja
			xQueueSendToBack(Cola10segA, &Send, portMAX_DELAY);
			xQueueSendToBack(Cola10segA, &hora, portMAX_DELAY);
			xQueueSendToBack(Cola10segA, &min, portMAX_DELAY);
			xSemaphoreGive(Semaforo_10SegA);//una vez que cargue todo en la cola procedo a guardarlo en la eeprom
		}

	}
	vTaskDelete(NULL);	//Borra la tarea

}

//*********************************************************************************************************************
//envia cada 10 segundos la informacion
static void vTask10seg(void *pvParameters)
{
	uint8_t datos[6], i;

	while(1)
	{
		xSemaphoreTake(Semaforo_10SegB, portMAX_DELAY);
		xSemaphoreTake(Semaforo_10SegA, portMAX_DELAY);
		for(i=0;i<4; i++)
		{
			xQueueReceive(Cola10segA, &datos[i], portMAX_DELAY);
		}
		for(i=4;i<6; i++)
		{
			xQueueReceive(Cola10segB, &datos[i], portMAX_DELAY);
		}
		for(i=0; i<6 ; i++)
		{
			Envio_Datos(LPC_UART3,(char *) &datos[i]);
		}

	}
	vTaskDelete(NULL);	//Borra la tarea

}

//*********************************************************************************************************************

void uC_StartUp (void)
{
	Chip_GPIO_Init (LPC_GPIO);
	Chip_GPIO_SetDir (LPC_GPIO, LED_STICK, OUTPUT);
	Chip_IOCON_PinMux (LPC_IOCON, LED_STICK, IOCON_MODE_INACT, IOCON_FUNC0);
	Chip_GPIO_SetDir (LPC_GPIO, SENS_MOV, INPUT);
	Chip_IOCON_PinMux (LPC_IOCON, SENS_MOV, IOCON_MODE_INACT, IOCON_FUNC0);
	Chip_GPIO_SetDir (LPC_GPIO, PULS_ONOFF, INPUT);
	Chip_IOCON_PinMux (LPC_IOCON, PULS_ONOFF, IOCON_MODE_PULLDOWN, IOCON_FUNC0);

}

int main(void)
{
	uC_StartUp ();

	SystemCoreClockUpdate();

	vSemaphoreCreateBinary(Semaforo_1Seg);
	vSemaphoreCreateBinary(Semaforo_1SegB);
	vSemaphoreCreateBinary(Semaforo_10SegA);
	vSemaphoreCreateBinary(Semaforo_10SegB);
	vSemaphoreCreateBinary(Semaforo_30Min);


	ColaADCTemp = xQueueCreate (1, sizeof(uint16_t));
	ColaADCPre = xQueueCreate (1, sizeof(uint16_t));
	Cola10segA = xQueueCreate (4, sizeof(uint8_t));
	Cola10segB = xQueueCreate (2, sizeof(uint8_t));
	ColaEEprom = xQueueCreate (4, sizeof(uint8_t));


	xSemaphoreTake(Semaforo_1Seg, portMAX_DELAY);
	xSemaphoreTake(Semaforo_1SegB, portMAX_DELAY);
	xSemaphoreTake(Semaforo_10SegA, portMAX_DELAY);
	xSemaphoreTake(Semaforo_10SegB, portMAX_DELAY);
	xSemaphoreTake(Semaforo_30Min, portMAX_DELAY);


	xTaskCreate(taskAnalisis, (char *) "taskAnalisis",
					configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
					(xTaskHandle *) NULL);

	xTaskCreate(ADC_Config, (char *) "ADC_Config",
					configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 3UL),
					(xTaskHandle *) NULL);

	xTaskCreate(I2C_Config, (char *) "I2C_Config",
					configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 3UL),
					(xTaskHandle *) NULL);

	xTaskCreate(xTaskUART3Config, (char *) "xTaskUART3Config",
					configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 3UL),
					(xTaskHandle *) NULL);

	xTaskCreate(vTaskInicTimer, (char *) "vTaskInicTimer",
					configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 3UL),
					(xTaskHandle *) NULL);

	xTaskCreate(vTaskPulsadores, (char *) "vTaskPulsadores",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL),
				(xTaskHandle *) NULL);

	xTaskCreate(vTaskEEprom, (char *) "vTaskEEprom",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL),
				(xTaskHandle *) NULL);
	xTaskCreate(vTask10seg, (char *) "vTask10seg",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL),
				(xTaskHandle *) NULL);


	/* Start the scheduler */
	vTaskStartScheduler();

	/* Nunca debería arribar aquí */
    return 0;
}
