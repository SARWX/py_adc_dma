
/* Includes ------------------------------------------------------------------*/
// #include "main.h" 
// Основано на другом проекте от Гарановича Д.И.
// Используется только аппаратный USB, АЦП, должен определяться в системе, как устройство
// с последовательным интерфейсом
// Это программа должна читать данные с АЦП (на данный момент, с одного, в перспективе - с двух)
// считанные данные должны передаваться в Serial port 
// требуется достичь максимальной скорости оцифровки для АЦП

#include "MDR32F9Qx_config.h"
#include "MDR32F9Qx_usb_handlers.h"
#include "MDR32F9Qx_rst_clk.h"
#include "MDR32F9Qx_ssp.h"
#include "MDR32F9Qx_port.h"
#include "MDR32F9Qx_adc.h"
#include "MDR32F9Qx_dma.h"

#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
//#include <time.h>

/* Private define ------------------------------------------------------------*/
#define BUFFER_LENGTH 128
#define timeout 10
#define FREQ 32000000
//#define DEBUG
#define BufferSize 8

#define LE_XX_Pin PORT_Pin_2
#define LE_XX_Port MDR_PORTF

#define LE_SN_Pin PORT_Pin_0
#define LE_SN_Port MDR_PORTC

#define LED_Pin PORT_Pin_2
#define LED_Port MDR_PORTC


/* Private variables ---------------------------------------------------------*/
static USB_Clock_TypeDef USB_Clock_InitStruct;
static USB_DeviceBUSParam_TypeDef USB_DeviceBUSParam;
static MDR_SSP_TypeDef SSP_InitStruct;
SSP_InitTypeDef sSSP;
PORT_InitTypeDef PORT_InitStructure;

static uint8_t Buffer[BUFFER_LENGTH];
static uint8_t RecBuf[BUFFER_LENGTH];
static uint8_t DoubleBuf[BUFFER_LENGTH * 2];

char *start;
char *end;
char tokens[5][BUFFER_LENGTH * 2]; //usb parsing tokens pointers array
char tempString[100];			   //debug

//uint32_t RecLen = 0;//usb data len
// structures for ADC
ADC_InitTypeDef sADC;
ADCx_InitTypeDef sADCx;
// structures for DMA
DMA_ChannelInitTypeDef sDMA;
DMA_CtrlDataInitTypeDef sDMA_PriCtrlData;				// Primary control data
DMA_CtrlDataInitTypeDef sDMA_AltCtrlData;				// Alternative control data


// DMA_CtrlDataTypeDef sDMA_ctrl_table;




#ifdef USB_CDC_LINE_CODING_SUPPORTED
static USB_CDC_LineCoding_TypeDef LineCoding;
#endif /* USB_CDC_LINE_CODING_SUPPORTED */

/* Private function prototypes -----------------------------------------------*/
static void Setup_CPU_Clock(void); // настройка тактирования ядра
static void Setup_USB(void);
static void VCom_Configuration(void); // конфигурация виртуального COM-порта
static void USB_PrintDebug(char *format, ...); // вывод отладочных сообщений
static void SetupDMA();
static void SetupADC(); // настройка АЦП
void ADC_IRQHandler(void); // обработчик прерываний АЦП

/* Private functions ---------------------------------------------------------*/
// задержка на count тактов 
void delayTick(uint32_t count) 
{
	while (count--)
	{
		__NOP();
	}
}

// задержка в миллисекундах, относительно длительности одного такта
// требует доработки, не используется
// void delay(int milliseconds)
// {
//     long pause;
// //    clock_t now,then;

//     pause = milliseconds*(FREQ/1000);
// //    now = then = clock();
//     while(pause>0 )
//         pause--;
// }



// вывод символов в USB 
void USB_Print(char *format, ...)
{
	va_list argptr;
	va_start(argptr, format);

	vsprintf(tempString, format, argptr);
	va_end(argptr);
	USB_CDC_SendData((uint8_t *)tempString, strlen(tempString));
	
	//delayTick(timeout);
}

void USB_PrintDebug(char *format, ...)
{
#ifdef DEBUG
	va_list argptr;
	va_start(argptr, format);

	vsprintf(tempString, format, argptr);
	va_end(argptr);
	//CDC_Transmit_FS((uint8_t *)tempString,strlen(tempString) );
	USB_CDC_SendData((uint8_t *)tempString, strlen(tempString));
	delayTick(timeout);
#endif
}

// global arrays for ADC buffers
uint16_t ADC1_array[50];
uint16_t ADC2_array[50];
// common ADC counter
uint8_t ADC_USER_COUNTER = 0;
// ADC transmit request
int ADC_TX_REQUEST = 0;

int main(void)
{
	// delayTick(0xFFFF);
	VCom_Configuration();

	/* CDC layer initialization */
	SetupADC();
	USB_CDC_Init(Buffer, 1, SET);
	Setup_CPU_Clock();
	Setup_USB();
  
	// Инициализация пина для светодиода
	RST_CLK_PCLKcmd (RST_CLK_PCLK_PORTC, ENABLE);
	PORT_InitStructure.PORT_Pin = (LED_Pin);
	PORT_InitStructure.PORT_OE = PORT_OE_OUT;
	PORT_InitStructure.PORT_SPEED = PORT_SPEED_MAXFAST;
	PORT_InitStructure.PORT_MODE = PORT_MODE_DIGITAL;
	PORT_Init(LED_Port, &PORT_InitStructure);

	/* Main loop */
	while (1){
		if(ADC_TX_REQUEST) {
			// Send data by USB
			//	USB_Print("%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n", ADC2_array[0], ADC2_array[1], ADC2_array[2],ADC2_array[3], ADC2_array[4], ADC2_array[5], ADC2_array[6], ADC2_array[7], ADC2_array[8], ADC2_array[9]
			//	, ADC2_array[10], ADC2_array[11], ADC2_array[12],ADC2_array[13], ADC2_array[14], ADC2_array[15], ADC2_array[16], ADC2_array[17], ADC2_array[18], ADC2_array[19]);
			// ACD1
			USB_Print("%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n", ADC1_array[0], ADC1_array[1], ADC1_array[2],ADC1_array[3], ADC1_array[4], ADC1_array[5], ADC1_array[6], ADC1_array[7], ADC1_array[8], ADC1_array[9]
			, ADC1_array[10], ADC1_array[11], ADC1_array[12],ADC1_array[13], ADC1_array[14], ADC1_array[15], ADC1_array[16], ADC1_array[17], ADC1_array[18], ADC1_array[19]);
			ADC_TX_REQUEST = 0;
		}
	}
}
void SetupADC()
{
	/* Enable peripheral clocks */
    RST_CLK_PCLKcmd((RST_CLK_PCLK_RST_CLK | RST_CLK_PCLK_ADC), ENABLE);
    RST_CLK_PCLKcmd((RST_CLK_PCLK_PORTC | RST_CLK_PCLK_PORTD), ENABLE);

	/* Init NVIC */
    SCB->AIRCR = 0x05FA0000 | ((uint32_t)0x500);
    SCB->VTOR = 0x08000000;
    /* Disable all interrupt */
    NVIC->ICPR[0] = 0xFFFFFFFF;
    NVIC->ICER[0] = 0xFFFFFFFF;

	NVIC->ISER[0] = (1<<ADC_IRQn);

	/* Reset PORTD settings */
    PORT_DeInit(MDR_PORTD);

	/* Configure ADC pins: ADC1 and ADC2 */
    /* Configure PORTD pins 0 */
    PORT_InitStructure.PORT_Pin   = PORT_Pin_0 | PORT_Pin_1;
    PORT_InitStructure.PORT_OE    = PORT_OE_IN;
    PORT_InitStructure.PORT_MODE  = PORT_MODE_ANALOG;
    PORT_Init(MDR_PORTD, &PORT_InitStructure);

	/* ADC Configuration */
    /* Reset all ADC settings */
    ADC_DeInit();
    ADC_StructInit(&sADC);
    ADC_Init (&sADC);

    ADCx_StructInit (&sADCx);
    sADCx.ADC_ClockSource      = ADC_CLOCK_SOURCE_CPU;
    sADCx.ADC_SamplingMode     = ADC_SAMPLING_MODE_CYCLIC_CONV;
    sADCx.ADC_ChannelSwitching = ADC_CH_SWITCHING_Disable;
    sADCx.ADC_ChannelNumber    = ADC_CH_ADC0;
    sADCx.ADC_Channels         = ADC_CH_ADC0_MSK;
    sADCx.ADC_VRefSource       = ADC_VREF_SOURCE_INTERNAL;
    sADCx.ADC_IntVRefSource    = ADC_INT_VREF_SOURCE_INEXACT;
    sADCx.ADC_Prescaler        = ADC_CLK_div_32;
    sADCx.ADC_DelayGo          = 0xF;
    ADC1_Init (&sADCx);
		// Copy configuration of ADC1 to ADC2, change the channel
    sADCx.ADC_ChannelNumber    = ADC_CH_ADC1;
    sADCx.ADC_Channels         = ADC_CH_ADC1_MSK;
    ADC2_Init (&sADCx);

    /* Enable ADC1 EOCIF and AWOIFEN interupts */
    ADC1_ITConfig((ADCx_IT_END_OF_CONVERSION), ENABLE);

		// lower IRQ priority
	//	NVIC_SetPriority(ADC_IRQn, 0xA);

    /* ADC enable */
    ADC1_Cmd (ENABLE);
		ADC2_Cmd (ENABLE);
}

// обработчик прерываний
void ADC_IRQHandler(void)
{
	// Write mesurenment to the buffer
	// ALL ODD ADC MESURENMENTS write to ADC1_array
	if (ADC_USER_COUNTER % 2) {						
		ADC1_array[ADC_USER_COUNTER++ / 2] = MDR_ADC->ADC1_RESULT & 0x0FFF;
	}
	// ALL EVEN ADC MESURENMENTS write to ADC2_array
	else {
		ADC2_array[ADC_USER_COUNTER++ / 2] = MDR_ADC->ADC2_RESULT & 0x0FFF;
	}
	
	// Check if buffer not full
	if (ADC_USER_COUNTER > 39) {   // 39
		// Reset counter
		ADC_USER_COUNTER = 0;
		// Set Transmit request
		ADC_TX_REQUEST = 1;
	}
}

/**
	* @brief	Frequencies setup
	* @param	None
	* @retval None
	*/
void Setup_CPU_Clock(void)
{
	/* Enable HSE */
	RST_CLK_HSEconfig(RST_CLK_HSE_ON);
	if (RST_CLK_HSEstatus() != SUCCESS)
	{
	/* Trap */ // если не установилась частота, то выпадает в бесконечный цикл
			   // нужно исправить, чтобы было понятно без подключения отладки
		while (1)
		{
		}
	}

	/* CPU_C1_SEL = HSE/2 */
	//RST_CLK_CPU_PLLconfig(RST_CLK_CPU_PLLsrcHSEdiv2, RST_CLK_CPU_PLLmul10);
	/* CPU_C1_SEL = HSI */
	//RST_CLK_CPU_PLLconfig(RST_CLK_CPU_PLLsrcHSIdiv1, RST_CLK_CPU_PLLmul4); //8*4 32 MHz
	RST_CLK_CPU_PLLconfig(RST_CLK_CPU_PLLsrcHSEdiv1, RST_CLK_CPU_PLLmul8); //  MHz
	
	RST_CLK_CPU_PLLcmd(ENABLE);
	if (RST_CLK_CPU_PLLstatus() != SUCCESS)
	{
		/* Trap */ // та же ситуация, что и в предыдущем случае
		while (1)
		{
		}
	}

	/* CPU_C3_SEL = CPU_C2_SEL */
	RST_CLK_CPUclkPrescaler(RST_CLK_CPUclkDIV1);
	/* CPU_C2_SEL = PLL */
	RST_CLK_CPU_PLLuse(ENABLE);
	/* HCLK_SEL = CPU_C3_SEL */
	RST_CLK_CPUclkSelection(RST_CLK_CPUclkCPU_C3);
}

/**
	* @brief	USB Device layer setup and powering on
	* @param	None
	* @retval None
	*/
void Setup_USB(void)
{
	/* Enables the CPU_CLK clock on USB */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_USB, ENABLE);

	/* Device layer initialization */
	//USB_Clock_InitStruct.USB_USBC1_Source = USB_C1HSIdiv1; //HSE not working :( using HSI 8Mhz
	USB_Clock_InitStruct.USB_USBC1_Source = USB_C1HSEdiv2; //HSE 
	USB_Clock_InitStruct.USB_PLLUSBMUL = USB_PLLUSBMUL6;   //was 12

	USB_DeviceBUSParam.MODE = USB_SC_SCFSP_Full;
	USB_DeviceBUSParam.SPEED = USB_SC_SCFSR_12Mb;
	USB_DeviceBUSParam.PULL = USB_HSCR_DP_PULLUP_Set;

	USB_DeviceInit(&USB_Clock_InitStruct, &USB_DeviceBUSParam);

	/* Enable all USB interrupts */
	USB_SetSIM(USB_SIS_Msk);
	// Try to lower USB interrupt priority
	// NVIC_SetPriority(USB_IRQn, 0xA);
	
	USB_DevicePowerOn();

	/* Enable interrupt on USB */
#ifdef USB_INT_HANDLE_REQUIRED
	NVIC_EnableIRQ(USB_IRQn);
#endif /* USB_INT_HANDLE_REQUIRED */

	USB_DEVICE_HANDLE_RESET;
}

/**
	* @brief	Example-relating data initialization
	* @param	None
	* @retval None
	*/
static void VCom_Configuration(void)
{
	#ifdef USB_CDC_LINE_CODING_SUPPORTED
		//LineCoding.dwDTERate = 9600;
		LineCoding.dwDTERate = 115200;
		LineCoding.bCharFormat = 0;
		LineCoding.bParityType = 0;
		LineCoding.bDataBits = 8;
	#endif /* USB_CDC_LINE_CODING_SUPPORTED */
}

/**
	* @brief	USB_CDC_HANDLE_DATA_RECEIVE implementation - data echoing
	* @param	Buffer: Pointer to the user's buffer with received data
	* @param	Length: Length of data
	* @retval @ref USB_Result.
	*/
USB_Result USB_CDC_RecieveData(uint8_t *Buffer, uint32_t Length)
{
	memcpy(RecBuf, Buffer, BUFFER_LENGTH);
	RecBuf[Length] = 0; //wtf last byte on odd wrong.
	return USB_SUCCESS;
}

#ifdef USB_CDC_LINE_CODING_SUPPORTED

/**
	* @brief	USB_CDC_HANDLE_GET_LINE_CODING implementation example
	* @param	wINDEX: Request value 2nd word (wIndex)
	* @param	DATA: Pointer to the USB_CDC Line Coding Structure
	* @retval @ref USB_Result.
	*/
USB_Result USB_CDC_GetLineCoding(uint16_t wINDEX, USB_CDC_LineCoding_TypeDef *DATA)
{
	assert_param(DATA);
	if (wINDEX != 0)
	{
		/* Invalid interface */
		return USB_ERR_INV_REQ;
	}

	/* Just store received settings */
	*DATA = LineCoding;
	return USB_SUCCESS;
}

/**
	* @brief	USB_CDC_HANDLE_SET_LINE_CODING implementation example
	* @param	wINDEX: Request value 2nd word (wIndex)
	* @param	DATA: Pointer to the USB_CDC Line Coding Structure
	* @retval @ref USB_Result.
	*/
USB_Result USB_CDC_SetLineCoding(uint16_t wINDEX, const USB_CDC_LineCoding_TypeDef *DATA)
{
	assert_param(DATA);
	if (wINDEX != 0)
	{
		/* Invalid interface */
		return USB_ERR_INV_REQ;
	}

	/* Just send back settings stored earlier */
	LineCoding = *DATA;
	return USB_SUCCESS;
}

#endif /* USB_CDC_LINE_CODING_SUPPORTED */




void SetupDMA() {
	  DMA_DeInit();															// reset all DMA settings
    DMA_StructInit(&sDMA);										// init sDMA structure of the type DMA_ChannelInitTypeDef
	
	
	
	
	
    DMA_CtrlDataInit(&sDMA_ctrl, &sDMA_ctrl_table);	//initialize
	
	

    ADCx_StructInit (&sADCx);
    sADCx.ADC_ClockSource      = ADC_CLOCK_SOURCE_CPU;
	
	
	
	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_DMA, ENABLE);		// enable clocking of the dma controller
	
	// initialize DMA CtrlData structure
	sDMA_PriCtrlData.DMA_SourceBaseAddr = (uint32_t) &MDR_ADC->ADC1_RESULT;		// From where we will get the data
	sDMA_PriCtrlData.DMA_DestBaseAddr = (uint32_t) ADC1_array;								// Our buffer for adc1
	sDMA_PriCtrlData.DMA_SourceIncSize = DMA_SourceIncNo;											// NO increment for ADC
	sDMA_PriCtrlData.DMA_DestIncSize = DMA_DestIncHalfword;										// increment = 16 bit for our uint_16t buffer
	
	
	
	
	
	
	// initialize DMA
	DMA_Init(DMA_Channel_ADC1, &sDMA);				// init 8 channel via sDMA structure (8 channel - DMA_Channel_ADC1)
	// enable DMA
	DMA_Cmd(DMA_Channel_ADC1, ENABLE);
}



