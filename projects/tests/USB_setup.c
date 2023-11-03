typedef struct {
    uint32_t PULL;  /*!< This member configures the D+ и D- line pulling
                         This member can be combination of the following values:
                         USB_HSCR_DM_PULLDOWN_Set: D- line pull down
                         USB_HSCR_DM_PULLUP_Set:   D- line pull up
                         USB_HSCR_DP_PULLDOWN_Set: D+ line pull down
                         USB_HSCR_DP_PULLUP_Set:   D+ line pull up */
    uint32_t SPEED; /*!< This member configures the USB speed
                         This member can be one of the following values:
                         USB_SC_SCFSR_12Mb:  12 Mbit/sec
                         USB_SC_SCFSR_1_5Mb: 1.5 Mbit/sec */
    uint32_t MODE;  /*!< This member configures the USB polarity
                         This member can be one of the following values:
                         USB_SC_SCFSP_Full: FULL_SPEED
                         USB_SC_SCFSP_Low:  LOW_SPEED */
} USB_DeviceBUSParam_TypeDef;