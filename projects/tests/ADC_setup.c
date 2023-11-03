typedef struct
{
    ADCx_Clock_Source      ADC_ClockSource;      /*!< Specifies the ADCx clock source.
                                                      This parameter can be a value of @ref ADCx_Clock_Source */

// typedef enum
// {
//     ADC_CLOCK_SOURCE_CPU = (((uint32_t)0x0) << ADC1_CFG_REG_CLKS_Pos), /*!< Selects CPU_CLK as ADC clock. */                             ЭТО тактирование от того же источника, что и ядро
//     ADC_CLOCK_SOURCE_ADC = (((uint32_t)0x1) << ADC1_CFG_REG_CLKS_Pos)  /*!< Selects ADC_CLK as ADC clock. */
// } ADCx_Clock_Source;

    ADCx_Sampling_Mode     ADC_SamplingMode;     /*!< Specifies the ADCx sampling mode.
                                                      This parameter can be a value of @ref ADCx_Sampling_Mode */
// typedef enum
// {
//     ADC_SAMPLING_MODE_SINGLE_CONV = (((uint32_t)0x0) << ADC1_CFG_REG_SAMPLE_Pos), /*!< Selects ADC single mode operation. */
//     ADC_SAMPLING_MODE_CYCLIC_CONV = (((uint32_t)0x1) << ADC1_CFG_REG_SAMPLE_Pos)  /*!< Selects ADC cyclic mode operation. */             ЭТО
// } ADCx_Sampling_Mode;

    ADCx_Channel_Switching ADC_ChannelSwitching; /*!< Enables or disables the ADCx channel switching.
                                                      This parameter can be a value of @ref ADCx_Channel_Switching */
// typedef enum
// {
//     ADC_CH_SWITCHING_Disable = (((uint32_t)0x0) << ADC1_CFG_REG_CHCH_Pos), /*!< Disables Channel Swithing. */                            ЭТО
//     ADC_CH_SWITCHING_Enable  = (((uint32_t)0x1) << ADC1_CFG_REG_CHCH_Pos)  /*!< Enables Channel Swithing. */
// } ADCx_Channel_Switching;

    ADCx_Channel_Number    ADC_ChannelNumber;    /*!< Specifies the ADCx channel number.
                                                      This parameter can be a value of @ref ADCx_Channel_Number */
// typedef enum
// {
//     ADC_CH_ADC0        = ((uint32_t)0x00), /*!< Specifies the ADC channel 0.  */                                                         ЭТО и вот тут странно, у нас всего 2 ADC
//     ADC_CH_ADC1        = ((uint32_t)0x01), /*!< Specifies the ADC channel 1.  */
//     ADC_CH_ADC2        = ((uint32_t)0x02), /*!< Specifies the ADC channel 2.  */
//     ADC_CH_ADC3        = ((uint32_t)0x03), /*!< Specifies the ADC channel 3.  */
//     ADC_CH_ADC4        = ((uint32_t)0x04), /*!< Specifies the ADC channel 4.  */
//     ADC_CH_ADC5        = ((uint32_t)0x05), /*!< Specifies the ADC channel 5.  */
//     ADC_CH_ADC6        = ((uint32_t)0x06), /*!< Specifies the ADC channel 6.  */
//     ADC_CH_ADC7        = ((uint32_t)0x07), /*!< Specifies the ADC channel 7.  */
//     ADC_CH_ADC8        = ((uint32_t)0x08), /*!< Specifies the ADC channel 8.  */
//     ADC_CH_ADC9        = ((uint32_t)0x09), /*!< Specifies the ADC channel 9.  */
//     ADC_CH_ADC10       = ((uint32_t)0x0A), /*!< Specifies the ADC channel 10. */
//     ADC_CH_ADC11       = ((uint32_t)0x0B), /*!< Specifies the ADC channel 11. */
//     ADC_CH_ADC12       = ((uint32_t)0x0C), /*!< Specifies the ADC channel 12. */
//     ADC_CH_ADC13       = ((uint32_t)0x0D), /*!< Specifies the ADC channel 13. */
//     ADC_CH_ADC14       = ((uint32_t)0x0E), /*!< Specifies the ADC channel 14. */
//     ADC_CH_ADC15       = ((uint32_t)0x0F), /*!< Specifies the ADC channel 15. */
//     ADC_CH_INT_VREF    = ((uint32_t)0x1E), /*!< Specifies the ADC channel 30 (Internal VRef). */
//     ADC_CH_TEMP_SENSOR = ((uint32_t)0x1F)  /*!< Specifies the ADC channel 31 (Temperature Sensor). */
// } ADCx_Channel_Number;

    uint32_t               ADC_Channels;         /*!< Specifies the ADCx channels mask.
                                                      This parameter can be a value of @ref ADCx_Channels */

    ADCx_Level_Control     ADC_LevelControl;     /*!< Enables or disables the ADCx level control.
                                                      This parameter can be a value of @ref ADCx_Level_Control */
// typedef enum
// {
//     ADC_LEVEL_CONTROL_Disable = (((uint32_t)0x0) << ADC1_CFG_REG_RNGC_Pos), /*!< Disables Level Control. */
//     ADC_LEVEL_CONTROL_Enable  = (((uint32_t)0x1) << ADC1_CFG_REG_RNGC_Pos)  /*!< Enables Level Control. */
// } ADCx_Level_Control;

    uint16_t               ADC_LowLevel;         /*!< Specifies the ADCx value low level.
                                                      This parameter can be a number between 0x0000 and 0x0FFF. */

    uint16_t               ADC_HighLevel;        /*!< Specifies the ADCx value high level.
                                                      This parameter can be a number between 0x0000 and 0x0FFF. */

    ADCx_VRef_Source       ADC_VRefSource;       /*!< Specifies the ADCx voltage reference source (internal or external).
                                                      This parameter can be a value of @ref ADCx_VRef_Source */
// typedef enum
// {
//     ADC_VREF_SOURCE_INTERNAL = (((uint32_t)0x0) << ADC1_CFG_M_REF_Pos), /*!< Selects Internal Voltage Reference. */                  ЭТО, а почему? Возможно лучше  внешнее питание, хотя наврядли
//     ADC_VREF_SOURCE_EXTERNAL = (((uint32_t)0x1) << ADC1_CFG_M_REF_Pos)  /*!< Selects External Voltage Reference. */
// } ADCx_VRef_Source;  

    ADCx_Int_VRef_Source   ADC_IntVRefSource;    /*!< Specifies the ADCx internal voltage reference source (inexact or exact).
                                                      This parameter can be a value of @ref ADCx_Int_VRef_Source */
// typedef enum
// {
//     ADC_INT_VREF_SOURCE_INEXACT = ((uint32_t)0x0), /*!< Selects inexact Internal Voltage Reference. */                               ЭТО а опять таки вопрос почему, Temperature Sensor в МК вроде есть, почему его не взять 
//     ADC_INT_VREF_SOURCE_EXACT   = ((uint32_t)0x1)  /*!< Selects exact Internal Voltage Reference (from Temperature Sensor). */
// } ADCx_Int_VRef_Source;

    ADCx_Prescaler         ADC_Prescaler;        /*!< Specifies the ADCx Prescaler configuration.
                                                      This parameter can be a value of @ref ADCx_Prescaler */
// typedef enum
// {
//     ADC_CLK_div_None = (((uint32_t)0x0) << ADC1_CFG_REG_DIVCLK_Pos), /*!< The input ADC clock devides by 1. */
//     ADC_CLK_div_2    = (((uint32_t)0x1) << ADC1_CFG_REG_DIVCLK_Pos), /*!< The input ADC clock devides by 2. */
//     ADC_CLK_div_4    = (((uint32_t)0x2) << ADC1_CFG_REG_DIVCLK_Pos), /*!< The input ADC clock devides by 4. */
//     ADC_CLK_div_8    = (((uint32_t)0x3) << ADC1_CFG_REG_DIVCLK_Pos), /*!< The input ADC clock devides by 8. */
//     ADC_CLK_div_16   = (((uint32_t)0x4) << ADC1_CFG_REG_DIVCLK_Pos), /*!< The input ADC clock devides by 16. */
//     ADC_CLK_div_32   = (((uint32_t)0x5) << ADC1_CFG_REG_DIVCLK_Pos), /*!< The input ADC clock devides by 32. */
//     ADC_CLK_div_64   = (((uint32_t)0x6) << ADC1_CFG_REG_DIVCLK_Pos), /*!< The input ADC clock devides by 64. */
//     ADC_CLK_div_128  = (((uint32_t)0x7) << ADC1_CFG_REG_DIVCLK_Pos), /*!< The input ADC clock devides by 128. */
//     ADC_CLK_div_256  = (((uint32_t)0x8) << ADC1_CFG_REG_DIVCLK_Pos), /*!< The input ADC clock devides by 256. */
//     ADC_CLK_div_512  = (((uint32_t)0x9) << ADC1_CFG_REG_DIVCLK_Pos), /*!< The input ADC clock devides by 512. */
//     ADC_CLK_div_1024 = (((uint32_t)0xA) << ADC1_CFG_REG_DIVCLK_Pos), /*!< The input ADC clock devides by 1024.*/
//     ADC_CLK_div_2048 = (((uint32_t)0xB) << ADC1_CFG_REG_DIVCLK_Pos)  /*!< The input ADC clock devides by 2048. */
// } ADCx_Prescaler;

    uint32_t               ADC_DelayGo;          /*!< Specifies the ADCx start conversion delay at sequential conversion mode.
                                                      This parameter can be a number between 0 and 7. */
} ADCx_InitTypeDef;