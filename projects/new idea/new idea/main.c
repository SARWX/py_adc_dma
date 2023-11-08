
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
ADC_InitTypeDef sADC;
ADCx_InitTypeDef sADCx;

#ifdef USB_CDC_LINE_CODING_SUPPORTED
static USB_CDC_LineCoding_TypeDef LineCoding;
#endif /* USB_CDC_LINE_CODING_SUPPORTED */

/* Private function prototypes -----------------------------------------------*/
static void Setup_CPU_Clock(void); // настройка тактирования ядра
static void Setup_USB(void);
static void VCom_Configuration(void); // конфигурация виртуального COM-порта
static void USB_PrintDebug(char *format, ...); // вывод отладочных сообщений
static void SetupADC(); // настройка АЦП 1
static void SetupADC2(); // настройка АЦП 2
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
	delayTick(timeout);
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

// global variable for ADC interrupt
int ADC_INTER_CNT = 0;
// global array for ADC interrupt
int ADC_tmp[24];

int main(void)
{
	VCom_Configuration();

	/* CDC layer initialization */
	SetupADC();
	SetupADC2();
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
	while (1) // закомментированно мигание светодиодом
	{
		// PORT_ResetBits(LED_Port, LED_Pin); // сброс состояния бита в логический ноль
		// delayTick(24000000); // задержка в тактах
		// PORT_SetBits(LED_Port, LED_Pin); // установка бита в логическую 1
		// delayTick(24000000);
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

	/* Configure ADC pins: ADC1 */
    /* Configure PORTD pins 1 */
    PORT_InitStructure.PORT_Pin   = PORT_Pin_1;
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
    sADCx.ADC_Channels         = 0;
    sADCx.ADC_VRefSource       = ADC_VREF_SOURCE_INTERNAL;
    sADCx.ADC_IntVRefSource    = ADC_INT_VREF_SOURCE_INEXACT;
    sADCx.ADC_Prescaler        = ADC_CLK_div_32;
    sADCx.ADC_DelayGo          = 0xF;
    ADC1_Init (&sADCx);

    /* Enable ADC1 EOCIF and AWOIFEN interupts */
    ADC1_ITConfig((ADCx_IT_END_OF_CONVERSION), ENABLE);

    /* ADC1 enable */
    ADC1_Cmd (ENABLE);
}


void SetupADC2()
{
    ADC_StructInit(&sADC);
    ADC_Init (&sADC);

    ADCx_StructInit (&sADCx);
    sADCx.ADC_ClockSource      = ADC_CLOCK_SOURCE_CPU;
    sADCx.ADC_SamplingMode     = ADC_SAMPLING_MODE_CYCLIC_CONV;
    sADCx.ADC_ChannelSwitching = ADC_CH_SWITCHING_Disable;
    sADCx.ADC_ChannelNumber    = ADC_CH_ADC1;
    sADCx.ADC_Channels         = ADC_CH_ADC1_MSK;
    sADCx.ADC_VRefSource       = ADC_VREF_SOURCE_INTERNAL;
    sADCx.ADC_IntVRefSource    = ADC_INT_VREF_SOURCE_INEXACT;
    sADCx.ADC_Prescaler        = ADC_CLK_div_32;
    sADCx.ADC_DelayGo          = 0xF;
    ADC2_Init (&sADCx);

    /* Enable ADC2 EOCIF and AWOIFEN interupts */
    ADC2_ITConfig((ADCx_IT_END_OF_CONVERSION), ENABLE);

    /* ADC2 enable */
    ADC2_Cmd (ENABLE);
}







// обработчик прерываний
void ADC_IRQHandler(void)
{
	ADC_tmp[ADC_INTER_CNT] = MDR_ADC->ADC2_RESULT & 0x0FFF;
	if (++ADC_INTER_CNT > 23) {
		ADC_INTER_CNT = 0;
    USB_Print("%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n", ADC_tmp[0], ADC_tmp[1], ADC_tmp[2], ADC_tmp[3], ADC_tmp[4], ADC_tmp[5], ADC_tmp[6], ADC_tmp[7], ADC_tmp[8], ADC_tmp[9], ADC_tmp[10], ADC_tmp[11], ADC_tmp[12], ADC_tmp[13], ADC_tmp[14], ADC_tmp[15], ADC_tmp[16], ADC_tmp[17], ADC_tmp[18], ADC_tmp[19], ADC_tmp[20], ADC_tmp[21], ADC_tmp[22], ADC_tmp[23]);
//	ADC_tmp[24], ADC_tmp[25], ADC_tmp[26], ADC_tmp[27], ADC_tmp[28], ADC_tmp[29], ADC_tmp[30], ADC_tmp[31], ADC_tmp[32], ADC_tmp[33], ADC_tmp[34], ADC_tmp[35], ADC_tmp[36], ADC_tmp[37], ADC_tmp[38], ADC_tmp[39], ADC_tmp[40], ADC_tmp[41], ADC_tmp[42], ADC_tmp[43], ADC_tmp[44], ADC_tmp[45], ADC_tmp[46], ADC_tmp[47]);
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
		LineCoding.dwDTERate = 2000000;
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
