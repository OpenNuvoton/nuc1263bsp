/******************************************************************************//**
 * @file     Serial_Stripe.c
 * @version  V1.00
 * @brief    Serial Stripe lighting effects sample file
 *
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <stdlib.h>
#include <stdio.h>
#include "NuMicro.h"
#include "LED_Control.h"
#include "Device.h"
#include "HDIV.h"
#include "Global_Variable.h"
#include "Flash.h"

#define printf(...)

volatile uint8_t s_u8StopFlashLED = 1;

uint8_t g_u8OneShot_Flag = 0;

void * const Mode_Function[15] = {(void *)FUNC_Off, (void *)FUNC_Static, (void *)FUNC_Breathing, (void *)FUNC_Strobe, (void *)FUNC_Cycling,
                                  (void *)FUNC_Random, (void *)FUNC_Music, (void *)FUNC_Wave, (void *)FUNC_Spring, (void *)FUNC_Off,
                                  (void *)FUNC_Off, (void *)FUNC_Off, (void *)FUNC_Off, (void *)FUNC_Water, (void *)FUNC_Rainbow};

/* Initial Serial LED Data Array */
#define cStrip1_LED 300
__attribute__((aligned (4))) uint8_t Strip1LEDData[cStrip1_LED*3];

/* Initial Receive LED Data Array */
__attribute__((aligned (4))) uint8_t au8RcvBuffer[cStrip1_LED*3];

/* Initial Strip1 Setting */
__attribute__((aligned (4))) volatile LED_Setting_T Strip1_LEDSetting = {0, 0, cStrip1_LED, 1, 255, 0, 0, 0xFF, 0, Dir_Forward, Type_GRB,
                                                                         1, 1, 0, FUNC_Static, Strip1LEDData, 0, 0, 0, Music_POP, 0, cStrip1_LED*3};

volatile LED_Setting_T *PDMA_Mapping[1] = {&Strip1_LEDSetting};

uint8_t LLSI_FlashLEDRoutine(void);

#define BreathingArraySize 150
const uint8_t BreathingBright[BreathingArraySize]	= {  0,   0,   0,   0,   1,   1,   2,   3,   4,   5,
                                                         7,   8,  10,  11,  13,  15,  17,  19,  21,  23,
                                                        25,  28,  30,  32,  35,  37,  40,  42,  45,  47,
                                                        50,  52,  55,  57,  60,  62,  65,  67,  70,  72,
                                                        75,  77,  79,  81,  83,  85,  87,  89,  90,  92,
                                                        93,  95,  96,  97,  98,  99,  99, 100, 100, 100,
                                                       100, 100, 100, 100, 100,  99,  99,  98,  97,  96,
                                                        95,  93,  92,  90,  89,  87,  85,  83,  81,  79,
                                                        77,  75,  72,  70,  67,  65,  62,  60,  57,  55,
                                                        52,  50,  47,  45,  42,  40,  37,  34,  32,  30,
                                                        28,  25,  23,  21,  19,  17,  15,  13,  11,  10,
                                                         8,   7,   5,   4,   3,   2,   1,   1,   0,   0,
                                                         0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                                                         0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                                                         0,   0,   0,   0,   0,   0,   0,   0,   0,   0};

/* Number of LED for each mode */
#define cMeteor_LED 4

#define RainbowSize	8
const uint8_t RainbowColor[RainbowSize][3]    // (R, G, B)
              = {{255,   0,   0},		// Red
                 {255,  85,   0},		// Orange
                 {255, 255,   0},		// Yellow
                 {  0, 255,   0},		// Green
                 {  0, 127, 255},		// Cyan
                 {  0,   0, 255},		// Blue
                 {127,   0, 255},		// Indigo
                 {255,   0, 255}};      // Purple

void LLSI_Initial(void)
{
    /* Unlock write-protected registers */
    SYS_UnlockReg();

    /* Enable module clock */
    CLK_EnableModuleClock(PDMA_MODULE);
    CLK_EnableModuleClock(LLSI0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set LLSI multi-function pin */
    LLSI0_MFP_Setting();    // Strip1

    /* Set LLSI configuration */
    //LLSI_Open(LLSI0, LLSI_MODE_SW, LLSI_FORMAT_GRB, HCLK_CLK, 1250, 400, 850, 50000, cStrip1_LED, LLSI_IDLE_LOW);
    LLSI_Open(LLSI0, LLSI_MODE_SW, LLSI_FORMAT_RGB, HCLK_CLK, 1200, 300, 900, 50000, cStrip1_LED, LLSI_IDLE_LOW);

    /* Enable reset command function */
    LLSI_ENABLE_RESET_COMMAND(LLSI0);

    /* Enable Reset command interrupt */
    LLSI_EnableInt(LLSI0, LLSI_RSTC_INT_MASK);

    /* Enable NVIC for LLSI */
    NVIC_EnableIRQ(LLSI0_IRQn);

    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);
    /* Open Channel 0 */
    PDMA_Open(0x1);

    /* PDMA Setting for LLSI */
    /* Transfer type is single transfer */
    PDMA_SetBurstType(0, PDMA_REQ_SINGLE, 0);
    /* Set source address, destination address */
    PDMA_SetTransferAddr(0, (uint32_t)Strip1LEDData, PDMA_SAR_INC, (uint32_t)&LLSI0->DATA, PDMA_DAR_FIX);

    /* Lock protected registers */
    SYS_LockReg();
}

void LLSI0_IRQHandler(void)
{
    if(!PDMA_Mapping[0]->fPDMA_Done)
    {
        PDMA_Mapping[0]->fPDMA_Done = 1;
        LLSI_SET_SW_MODE(LLSI0);
    }

    /* Clear interrupt flag */
    LLSI_ClearIntFlag(LLSI0, LLSI_RSTC_INT_MASK);
}

void Set_Single(uint8_t *LED_DATA, uint32_t TotalLED, uint8_t Data_R, uint8_t Data_G, uint8_t Data_B)
{
    uint32_t i;

    for(i = 0; i < TotalLED; i++)
    {
        *(LED_DATA + i*3 + 0) = Data_R;
        *(LED_DATA + i*3 + 1) = Data_G;
        *(LED_DATA + i*3 + 2) = Data_B;
    }
}

void Set_Array(uint8_t *LED_DATA, uint32_t TotalLED, uint8_t *DisplayData, uint8_t MaxBright)
{
    uint32_t i;
    uint8_t TempR, TempG, TempB;

    for(i = 0; i < TotalLED; i++)
    {
        TempR = HDIV_Div(*(DisplayData + i*3 + 0) * MaxBright, 0xFF);
        TempG = HDIV_Div(*(DisplayData + i*3 + 1) * MaxBright, 0xFF);
        TempB = HDIV_Div(*(DisplayData + i*3 + 2) * MaxBright, 0xFF);
        *(LED_DATA + i*3 + 0) = TempR;
        *(LED_DATA + i*3 + 1) = TempG;
        *(LED_DATA + i*3 + 2) = TempB;
    }
}

void Set_InverseArray(uint8_t *LED_DATA, uint32_t TotalLED, uint8_t *DisplayData, uint8_t MaxBright)
{
    uint32_t i, j;
    uint8_t TempR, TempG, TempB;

    for(i = 0, j = TotalLED-1; i < TotalLED; i++, j--)
    {
        TempR = HDIV_Div(*(DisplayData + j*3 + 0) * MaxBright, 0xFF);
        TempG = HDIV_Div(*(DisplayData + j*3 + 1) * MaxBright, 0xFF);
        TempB = HDIV_Div(*(DisplayData + j*3 + 2) * MaxBright, 0xFF);
        *(LED_DATA + i*3 + 0) = TempR;
        *(LED_DATA + i*3 + 1) = TempG;
        *(LED_DATA + i*3 + 2) = TempB;
    }
}

void Set_LED_Data(volatile struct LED_Setting_Tag* LED_Setting)
{
    uint32_t u32Count;

    /* Calculate transfer count */
    u32Count = LED_Setting->Array_Size / 4;
    if(LED_Setting->Array_Size % 4)
        u32Count++;

    /* Set transfer count */
    PDMA_SetTransferCnt(LED_Setting->LLSI_Num, PDMA_WIDTH_32, u32Count);
    /* Set request source */
    PDMA_SetTransferMode(LED_Setting->LLSI_Num, 36 + LED_Setting->LLSI_Num, FALSE, 0);    // PDMA_LLSI0 = 36
    /* Change LLSI mode */
    LLSI_SET_PDMA_MODE((LLSI_T *)(APB1_BASE + 0x100000*(LED_Setting->LLSI_Num%2) + 0x54000 + 0x200*(LED_Setting->LLSI_Num/2)));

    /* Clear done flag */
    LED_Setting->fPDMA_Done = 0;
}

void Clear_LED_Data(volatile struct LED_Setting_Tag* LED_Setting)
{
    uint32_t i;

    for(i = 0; i < LED_Setting->Array_Size; i++)
    {
        LED_Setting->LED_Data[i] = 0x0;
    }

    /* Set data */
    Set_LED_Data(LED_Setting);
}

void TIMER0_Initial(void)
{
    /* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per 1 ms */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER0);

    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);
}

void TMR0_IRQHandler(void)
{
    /* Clear interrupt flag */
    TIMER_ClearIntFlag(TIMER0);

    /* Add LED mode counter */
    Strip1_LEDSetting.TimeCounter++;
}

void LLSI_WriteData(uint16_t u16ByteSel, uint8_t u8Data)
{
    if (u16ByteSel < (sizeof(Strip1LEDData)/sizeof(uint8_t)))
    {
        /* debug log */
        printf("[LLSI_WriteData][%d]ByteSel: %d(0x%x), data: %d(0x%x)\n", Strip1_LEDSetting.LEDNum, u16ByteSel, u16ByteSel, u8Data, u8Data);
        *(au8RcvBuffer + u16ByteSel) = u8Data;
    }
    else
    {
        printf("[ERR][LLSI_WriteData][%d]ByteSel: %d(0x%x) Over buffer size ~~\n", Strip1_LEDSetting.LEDNum, u16ByteSel, u16ByteSel);
    }
}

void LLSI_WriteBlockData(uint16_t u16ByteLen, uint8_t *pu8Data)
{
    uint32_t u32Idx, u32Idx1;
    /* debug log */
    printf("[LLSI_WriteBlockData]%d\n", u16ByteLen);
    
    u16ByteLen <<= 1;
    /* LED even and odd number both use the same data. */
    for (u32Idx = 0; u32Idx < u16ByteLen; u32Idx+=3)
    {
        if (u32Idx < 3)
        {
            u32Idx1 = u32Idx;
            *(au8RcvBuffer + u32Idx1) = *(pu8Data + u32Idx);
            *(au8RcvBuffer + u32Idx1 + 1) = *(pu8Data + u32Idx + 1);
            *(au8RcvBuffer + u32Idx1 + 2) = *(pu8Data + u32Idx + 2);
            *(au8RcvBuffer + u32Idx1 + 3) = *(pu8Data + u32Idx);
            *(au8RcvBuffer + u32Idx1 + 3 + 1) = *(pu8Data + u32Idx + 1);
            *(au8RcvBuffer + u32Idx1 + 3 + 2) = *(pu8Data + u32Idx + 2);
            //printf("%d, %d\n", u32Idx, u32Idx1);
        }
        else
        {
            u32Idx1 = u32Idx * 2;
            *(au8RcvBuffer + u32Idx1) = *(pu8Data + u32Idx);
            *(au8RcvBuffer + u32Idx1 + 1) = *(pu8Data + u32Idx + 1);
            *(au8RcvBuffer + u32Idx1 + 2) = *(pu8Data + u32Idx + 2);
            *(au8RcvBuffer + u32Idx1 + 3) = *(pu8Data + u32Idx);
            *(au8RcvBuffer + u32Idx1 + 3 + 1) = *(pu8Data + u32Idx + 1);
            *(au8RcvBuffer + u32Idx1 + 3 + 2) = *(pu8Data + u32Idx + 2);
            //printf("%d, %d\n", u32Idx, u32Idx1);        
        }
    }
    /* debug log */
    for (u32Idx = 0; u32Idx < u16ByteLen; u32Idx++)
    {
        printf("[%d]%d ", u32Idx, *(au8RcvBuffer + u32Idx));
    }
    printf("\n");
}

void LLSI_StartFlashLED(uint8_t u8LLSIEnable)
{
    if (u8LLSIEnable)
    {
        printf("[LLSI_StartFlashLED] start\n");
        s_u8StopFlashLED = 0;
        
        /* Set Timer configuration */
        TIMER0_Initial();
        TIMER_Start(TIMER0);
#if 0
        /* Clear Color Data */
        Clear_LED_Data(&Strip1_LEDSetting);
#endif
        /* Use TIMER0 counter as seed of random function */
        srand(TIMER0->CNT);
        
        LLSI_FlashLEDRoutine();

    }
    else
    {
        printf("[LLSI_StartFlashLED] stop\n");

        /* Stop Flash LED. */
        s_u8StopFlashLED = 1;

    }
}

/* return s_u8StopFlashLED value. */
uint8_t LLSI_FlashLEDRoutine(void)
{
#if 0
    uint32_t i;
#endif
    if (s_u8StopFlashLED == 1)
    {
        /* Below code does not be executed to write LLSI data. */
        return s_u8StopFlashLED;
    }

    printf("[LLSI_FlashLEDRoutine][%d] start\n", Strip1_LEDSetting.LEDNum);
    
    /* Serial LED Control */
    if(Strip1_LEDSetting.fPDMA_Done)
    {
        printf("[LLSI_FlashLEDRoutine] PDMA_Done\n");
        if(Strip1_LEDSetting.AP_Sync == 1)
        {
            printf("[LLSI_FlashLEDRoutine] AP_Sync\n");

            /* Mapping Color to LED Format */
            Set_Array(Strip1_LEDSetting.LED_Data, Strip1_LEDSetting.LEDNum, (uint8_t *)(au8RcvBuffer), 0x1F);

            /* Set LED data */
            Set_LED_Data(&Strip1_LEDSetting);

            if(g_u8Strip1_Flash_OneShot == 1)
            {
                /* Clear done flag */
                Strip1_LEDSetting.fPDMA_Done = 0;

                /* One shot operation has done. */
                s_u8StopFlashLED = 1;

                printf("[LLSI_FlashLEDRoutine] stop(L:%d)\n", __LINE__);
                return s_u8StopFlashLED;
            }
        }
        else
        {
            printf("\n\t[LLSI_FlashLEDRoutine] g_u8Strip1_Flash_OneShot:%d, g_u8OneShot_Flag:%d\n", g_u8Strip1_Flash_OneShot, g_u8OneShot_Flag);
            //if(g_u8Strip1_Flash_OneShot && g_u8OneShot_Flag == 1)
            if(g_u8Strip1_Flash_OneShot)
            {
                /* Clear done flag */
                Strip1_LEDSetting.fPDMA_Done = 0;

                /* One shot operation has done. */
                s_u8StopFlashLED = 1;

            #if 0
                /* set LED strip to stop flashing */
                CLK_SysTickDelay(100000);

                for(i = 0; i < Strip1_LEDSetting.LEDNum * 3; i++)
                {
                    *(Strip1_LEDSetting.LED_Data + i) = 0x0;
                }

                /* Strip 1 LED data */
                Strip1_LEDSetting.Mode_FUNC(&Strip1_LEDSetting);

                /* Set LED data */
                Set_LED_Data(&Strip1_LEDSetting);
            #endif

                printf("[LLSI_FlashLEDRoutine] stop(L:%d)\n", __LINE__);
                return s_u8StopFlashLED;
            }
        }
    }

    return s_u8StopFlashLED;
}

/*------Lighting Mode-----------------------------------------*/
void FUNC_Off(volatile struct LED_Setting_Tag* LED_Setting)
{
    printf("\n\tFUNC_Off\n");
    /* Mapping Color to LED Format */
    Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, 0, 0, 0);
}

void FUNC_Static(volatile struct LED_Setting_Tag* LED_Setting)
{
    uint32_t TempR, TempG, TempB;
    printf("\n\tFUNC_Static\n");
    /* Calculate Color */
    TempR = HDIV_Div((LED_Setting->Color_R * LED_Setting->Brightness), 0xFF);
    TempG = HDIV_Div((LED_Setting->Color_G * LED_Setting->Brightness), 0xFF);
    TempB = HDIV_Div((LED_Setting->Color_B * LED_Setting->Brightness), 0xFF);

    /* Mapping Color to LED Format */
    Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, TempR, TempG, TempB);

    g_u8OneShot_Flag = 1;
}

void FUNC_Breathing(volatile struct LED_Setting_Tag* LED_Setting)
{
    uint32_t Temp, TempR, TempG, TempB;
    printf("\n\tFUNC_Breathing\n");
    /* Calculate Color */
    Temp = HDIV_Mod(HDIV_Div(LED_Setting->TimeCounter, 8 + HDIV_Div(LED_Setting->Speed, 10)), BreathingArraySize);
    TempR = HDIV_Div((LED_Setting->Color_R * LED_Setting->Brightness * *(BreathingBright + Temp)), 0xFF*100);
    TempG = HDIV_Div((LED_Setting->Color_G * LED_Setting->Brightness * *(BreathingBright + Temp)), 0xFF*100);
    TempB = HDIV_Div((LED_Setting->Color_B * LED_Setting->Brightness * *(BreathingBright + Temp)), 0xFF*100);

    /* Mapping Color to LED Format */
    Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, TempR, TempG, TempB);

    /* Reset CountingTime */
    if(LED_Setting->TimeCounter >= ((8 + HDIV_Div(LED_Setting->Speed, 10)) * BreathingArraySize))
    {
        LED_Setting->TimeCounter -= (8 + HDIV_Div(LED_Setting->Speed, 10)) * BreathingArraySize;

        g_u8OneShot_Flag = 1;
    }
}

void FUNC_Strobe(volatile struct LED_Setting_Tag* LED_Setting)
{
    uint32_t TempR, TempG, TempB;
    printf("\n\tFUNC_Strobe\n");
    /* Reset CountingTime */
    while(LED_Setting->TimeCounter >= (((5 * LED_Setting->Speed) + 275) * 2))
    {
        LED_Setting->TimeCounter -= (((5 * LED_Setting->Speed) + 275) * 2);

        g_u8OneShot_Flag = 1;
    }

    /* Extinguish */
    if(LED_Setting->TimeCounter < ((5 * LED_Setting->Speed) + 275))
    {
        /* Mapping Color to LED Format */
        Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, 0, 0, 0);
    }
    /* Lighten */
    else
    {
        /* Calculate Color */
        TempR = HDIV_Div((LED_Setting->Color_R * LED_Setting->Brightness), 0xFF);
        TempG = HDIV_Div((LED_Setting->Color_G * LED_Setting->Brightness), 0xFF);
        TempB = HDIV_Div((LED_Setting->Color_B * LED_Setting->Brightness), 0xFF);

        /* Mapping Color to LED Format */
        Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, TempR, TempG, TempB);
    }
}

void FUNC_Cycling(volatile struct LED_Setting_Tag* LED_Setting)
{
    static uint8_t u8ColorIndex = eColorRed;
    printf("\n\tFUNC_Cycling\n");
    /* Reset CountingTime */
    if (LED_Setting->TimeCounter >= (500 + LED_Setting->Speed*10))
    {
        LED_Setting->TimeCounter -= (500 + LED_Setting->Speed*10);

        /* Switch to next color state. */
        if(u8ColorIndex == eColorRed)
        {
            u8ColorIndex = eColorGreen;
        }
        else if(u8ColorIndex == eColorGreen)
        {
            u8ColorIndex = eColorBlue;

            g_u8OneShot_Flag = 1;
        }
        else if(u8ColorIndex == eColorBlue)
        {
            u8ColorIndex = eColorRed;
        }
    }

    /* Mapping Color to LED Format */
    Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, HDIV_Div((RainbowColor[u8ColorIndex][0] * LED_Setting->Brightness), 0xFF), \
                                                           HDIV_Div((RainbowColor[u8ColorIndex][1] * LED_Setting->Brightness), 0xFF), \
                                                           HDIV_Div((RainbowColor[u8ColorIndex][2] * LED_Setting->Brightness), 0xFF));
}

void FUNC_Random(volatile struct LED_Setting_Tag* LED_Setting)
{
    static uint8_t u8RandColorIndex = eColorRed;
    uint8_t temp;
    printf("\n\tFUNC_Random\n");
    /* Reset CountingTime */
    if (LED_Setting->TimeCounter >= (500 + LED_Setting->Speed*10))
    {
        LED_Setting->TimeCounter -= (500 + LED_Setting->Speed*10);

        /* Calculate next random color state. */
        /* If next state is same as current state, random again. */
        do
        {
            temp = ((RainbowSize * (rand() % 1024)) / 1024);
        }while(temp == u8RandColorIndex);
        /* Set next color state index. */
        u8RandColorIndex = temp;

        g_u8OneShot_Flag = 1;
    }

    /* Mapping Color to LED Format */
    Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, HDIV_Div((RainbowColor[u8RandColorIndex][0] * LED_Setting->Brightness), 0xFF), \
                                                           HDIV_Div((RainbowColor[u8RandColorIndex][1] * LED_Setting->Brightness), 0xFF), \
                                                           HDIV_Div((RainbowColor[u8RandColorIndex][2] * LED_Setting->Brightness), 0xFF));
}

void FUNC_Music(volatile struct LED_Setting_Tag* LED_Setting)
{
    uint32_t i;
    uint32_t j;
    uint8_t POP_Color[3] = {0};
    uint32_t Unit_Volume = HDIV_Div((LED_Setting->Main_Volume * LED_Setting->LEDNum), 100);
    uint8_t JAZZ_Display[LED_Setting->LEDNum][3];

    printf("\n\tFUNC_Music\n");
    /* Init Array */
    for(j = 0; j < LED_Setting->LEDNum; j++)
    {
        for(i = 0; i < 3; i++)
            JAZZ_Display[j][i] = 0;
    }

    /* POP */
    if(LED_Setting->Music_Action == Music_POP)
    {
        printf("\n\tFUNC_Music: POP\n");
        for(i = 0; i < 3; i++)
        {
            POP_Color[i] = HDIV_Div(RainbowColor[eColorBlue][i] * (100 - LED_Setting->Main_Volume), 100)\
                           + HDIV_Div(RainbowColor[eColorRed][i] * LED_Setting->Main_Volume, 100);

            if(i == 2)
                g_u8OneShot_Flag = 1;
        }

        /* Mapping Color to LED Format */
        Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, HDIV_Div((POP_Color[0] * LED_Setting->Brightness), 0xFF), \
                                                               HDIV_Div((POP_Color[1] * LED_Setting->Brightness), 0xFF), \
                                                               HDIV_Div((POP_Color[2] * LED_Setting->Brightness), 0xFF));
    }
    /* JAZZ */
    else if(LED_Setting->Music_Action == Music_JAZZ)
    {
        printf("\n\tFUNC_Music: JAZZ\n");
        for(j = 0; j < Unit_Volume; j++)
        {
            for(i = 0; i < 3; i++)
            {
                JAZZ_Display[j][i] = *(&LED_Setting->Color_R + i);
            }
        }

        /* Mapping Color to LED Format */
        /* Direction forward or backward */
        if(LED_Setting->Direction == Dir_Forward)
            Set_Array(LED_Setting->LED_Data, LED_Setting->LEDNum, (uint8_t *)JAZZ_Display, LED_Setting->Brightness);
        else if(LED_Setting->Direction == Dir_Backward)
            Set_InverseArray(LED_Setting->LED_Data, LED_Setting->LEDNum, (uint8_t *)JAZZ_Display, LED_Setting->Brightness);
    }
    /* Mixed */
    else if (LED_Setting->Music_Action == Music_Mixed)
    {
        printf("\n\tFUNC_Music: Mixed\n");
        for(i = 0; i < 3; i++)
        {
            POP_Color[i] = HDIV_Div(RainbowColor[eColorBlue][i] * (100 - LED_Setting->Main_Volume), 100)\
                           + HDIV_Div(RainbowColor[eColorRed][i] * LED_Setting->Main_Volume, 100);
        }

        for(j = 0; j < Unit_Volume; j++)
        {
            for(i = 0; i < 3; i++)
            {
                JAZZ_Display[j][i] = POP_Color[i];
            }
        }

        /* Mapping Color to LED Format */
        /* Direction forward or backward */
        if(LED_Setting->Direction == Dir_Forward)
            Set_Array(LED_Setting->LED_Data, LED_Setting->LEDNum, (uint8_t *)JAZZ_Display, LED_Setting->Brightness);
        else if(LED_Setting->Direction == Dir_Backward)
            Set_InverseArray(LED_Setting->LED_Data, LED_Setting->LEDNum, (uint8_t *)JAZZ_Display, LED_Setting->Brightness);
    }
}

void FUNC_Wave(volatile struct LED_Setting_Tag* LED_Setting)
{
    uint32_t i;
    uint32_t DeltaT = 800 + LED_Setting->Speed*4;
    uint32_t CurrentTime;
    uint8_t Color1[3];
    uint8_t Color2[3];
    uint8_t DisplayColor[3];
    uint32_t Temp;
    printf("\n\tFUNC_Wave\n");
    if(LED_Setting->Direction == Dir_Forward)
    {
        /* Take Color1 and Color2 from Rainbow array by TimeCounter */
        Temp = HDIV_Mod(HDIV_Div(LED_Setting->TimeCounter, DeltaT), RainbowSize);
        for(i = 0; i < 3; i++)
        {
            /* Set Color1 */
            Color1[i] = RainbowColor[Temp][i];
            /* Set Color2 */
            Color2[i] = RainbowColor[HDIV_Mod((Temp+1), RainbowSize)][i];
        }
    }
    else if(LED_Setting->Direction == Dir_Backward)
    {
        /* Take Color1 and Color2 from Rainbow array by TimeCounter */
        Temp = HDIV_Mod(HDIV_Div(LED_Setting->TimeCounter, DeltaT), RainbowSize);
        for(i = 0; i < 3; i++)
        {
            /* Set Color1 */
            Color1[i] = RainbowColor[RainbowSize - 1 - Temp][i];
            /* Set Color2 */
            Color2[i] = RainbowColor[HDIV_Mod((RainbowSize + RainbowSize - 2 - Temp), RainbowSize)][i];
        }
    }

    /* Reset CountingTime */
    if(LED_Setting->TimeCounter >= (DeltaT * RainbowSize))
    {
        LED_Setting->TimeCounter -= (DeltaT * RainbowSize);

        g_u8OneShot_Flag = 1;
    }

    /* Judge current time and set mixed Displaycolor by color1 and color2 */
    CurrentTime = HDIV_Mod(LED_Setting->TimeCounter, DeltaT);
    for(i = 0; i < 3; i++)
        DisplayColor[i] = HDIV_Div(Color1[i] * (DeltaT - CurrentTime), DeltaT) + HDIV_Div(Color2[i] * CurrentTime, DeltaT);

    /* Mapping Color to LED Format */
    Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, DisplayColor[0], DisplayColor[1], DisplayColor[2]);
}

void FUNC_Spring(volatile struct LED_Setting_Tag* LED_Setting)
{
    uint8_t DisplayColor[LED_Setting->LEDNum+2*cMeteor_LED][3];
    uint32_t i;
    uint32_t j;
    uint8_t Color[cMeteor_LED][3];
    uint32_t Duration;
    uint32_t Loop, DeltaT, DelayT;
    uint32_t TempColor;
    printf("\n\tFUNC_Spring\n");
    /* Init Array */
    for(j = 0; j < LED_Setting->LEDNum+2*cMeteor_LED; j++)
    {
        for(i = 0; i < 3; i++)
            DisplayColor[j][i] = 0;
    }

    /* DeltaT is unit time */
    DeltaT = 5 + HDIV_Div(LED_Setting->Speed, 20);
    /* Duration is one time pass through */
    Duration = DeltaT * (LED_Setting->LEDNum + cMeteor_LED * 2);
    /* DelayT is delay on two ends */
    DelayT = HDIV_Div(LED_Setting->LEDNum * DeltaT, 2);
    /* Reset CountingTime */
    if(LED_Setting->TimeCounter >= (2 * (Duration + DelayT)))
    {
        LED_Setting->TimeCounter -= 2 * (Duration + DelayT);
    }

    /* Forward */
    if(LED_Setting->TimeCounter < Duration)
    {
        /* Loop to indicate which LED for first Color */
        Loop = HDIV_Div(LED_Setting->TimeCounter, DeltaT);

        for(i = 0; i < 3; i++)
        {
            TempColor = *(&LED_Setting->Color_R + i);

            /* 1st 100%*/
            Color[0][i] = TempColor;
            /* 2nd 75% */
            Color[1][i] = (TempColor * 3) >> 2;
            /* 3rd 50% */
            Color[2][i] = TempColor >> 1;
            /* 4th 25% */
            Color[3][i] = TempColor >> 2;
        }

        for(j = Loop; j < (Loop + cMeteor_LED); j++)
        {
            for(i = 0; i < 3; i++)
                DisplayColor[j][i] = Color[j - Loop][i];
        }
    }
    /* All Extinguish */
    else if(LED_Setting->TimeCounter < (Duration + DelayT) )
    {
        for(j = 0; j < LED_Setting->LEDNum+2*cMeteor_LED; j++)
        {
            for(i = 0; i < 3; i++)
                DisplayColor[j][i] = 0;
        }
    }
    else if(LED_Setting->TimeCounter < (2*Duration + DelayT))
    {
        /* Loop to indicate which LED for first Color */
        Loop = HDIV_Div(LED_Setting->TimeCounter - (Duration + DelayT), DeltaT);

        for(i = 0; i < 3; i++)
        {
            TempColor = *(&LED_Setting->Color_R + i);

            /* 1st 25%*/
            Color[0][i] = TempColor >> 2;
            /* 2nd 50% */
            Color[1][i] = TempColor >> 1;
            /* 3rd 75% */
            Color[2][i] = (TempColor * 3) >> 2;
            /* 4th 100% */
            Color[3][i] = TempColor;
        }

        for(j = (LED_Setting->LEDNum + cMeteor_LED - Loop); j < (LED_Setting->LEDNum + cMeteor_LED - Loop + cMeteor_LED); j++)
        {
            for(i = 0; i < 3; i++)
                DisplayColor[j][i] = Color[j - (LED_Setting->LEDNum + cMeteor_LED - Loop)][i];
        }
    }
    /* All Extinguish */
    else
    {
        for(j = 0; j < LED_Setting->LEDNum+2*cMeteor_LED; j++)
        {
            for(i = 0; i < 3; i++)
                DisplayColor[j][i] = 0;
        }

        g_u8OneShot_Flag = 1;
    }

    /* Mapping Color to LED Format */
    /* Direction forward or backward */
    if(LED_Setting->Direction == Dir_Forward)
        Set_Array(LED_Setting->LED_Data, LED_Setting->LEDNum, (uint8_t *)(DisplayColor+cMeteor_LED), LED_Setting->Brightness);
    else if(LED_Setting->Direction == Dir_Backward)
        Set_InverseArray(LED_Setting->LED_Data, LED_Setting->LEDNum, (uint8_t *)(DisplayColor+cMeteor_LED), LED_Setting->Brightness);
}

void FUNC_Water(volatile struct LED_Setting_Tag* LED_Setting)
{
    const uint8_t Brightness[4] = {100, 15, 5, 15};
    const uint8_t MeteorNum = 4;
    static uint8_t u8ColorIndex = eColorRed;
    uint32_t i, j, Cursor;
    uint8_t DisplayColor[LED_Setting->LEDNum][3], Color[3];
    uint32_t MotionPeriod, TimeInPhase, PhasePeriod;
    printf("\n\tFUNC_Water\n");
    PhasePeriod = 1000 + LED_Setting->Speed * 4;
    MotionPeriod = HDIV_Div(PhasePeriod, 8);
    TimeInPhase = HDIV_Mod(LED_Setting->TimeCounter, PhasePeriod);

    /* Reset CountingTime */
    if (LED_Setting->TimeCounter >= PhasePeriod)
    {
        LED_Setting->TimeCounter -= PhasePeriod;

        /* If color index is overflow, reset color index. */
        u8ColorIndex = (u8ColorIndex >= (RainbowSize - 1)) ? eColorRed : (u8ColorIndex + 1);

        if(u8ColorIndex == (RainbowSize - 1))
            g_u8OneShot_Flag = 1;
    }


    /* Set Mixed Color = Color1 + Color2 */
    for(i = 0; i < 3; i++)
    {
        Color[i] = HDIV_Div(RainbowColor[u8ColorIndex][i] * (PhasePeriod - TimeInPhase), PhasePeriod)\
                + HDIV_Div(RainbowColor[HDIV_Mod((u8ColorIndex + 1), RainbowSize)][i] * TimeInPhase, PhasePeriod);
    }

    /* Calculate the cursor for moving the Brightness */
    Cursor = HDIV_Mod(HDIV_Div(LED_Setting->TimeCounter, MotionPeriod), MeteorNum);

    /* Sets the Light Bar array and modified the brightness for meteor effect. */
    for(j = 0; j < LED_Setting->LEDNum; j++)
    {
        for(i = 0; i< 3; i++)
        {
            DisplayColor[j][i] = HDIV_Div(Color[i] * Brightness[HDIV_Mod((Cursor + j), MeteorNum)], 100);
        }
    }

    /* Mapping Color to LED Format */
    /* Direction forward or backward */
    if(LED_Setting->Direction == Dir_Forward)
        Set_InverseArray(LED_Setting->LED_Data, LED_Setting->LEDNum, (uint8_t *)DisplayColor, LED_Setting->Brightness);
    else if(LED_Setting->Direction == Dir_Backward)
        Set_Array(LED_Setting->LED_Data, LED_Setting->LEDNum, (uint8_t *)DisplayColor, LED_Setting->Brightness);
}

void FUNC_Rainbow(volatile struct LED_Setting_Tag* LED_Setting)
{
    uint8_t DisplayColor[LED_Setting->LEDNum][3];
    uint32_t i, j;
    uint32_t deltaT, TotalDuration, CurrentTime, NextTime;
    uint8_t ColorCursor1, ColorCursor2;
    printf("\n\tFUNC_Rainbow\n");
    /* Init Array */
    for(j = 0; j < LED_Setting->LEDNum; j++)
    {
        for(i = 0; i < 3; i++)
            DisplayColor[j][i] = 0;
    }

    /* Duration for one time effect */
    TotalDuration = HDIV_Div(60 * 1000, HDIV_Div(255 - LED_Setting->Speed, 3) + 5);
    /* Next LED time */
    NextTime = HDIV_Div(TotalDuration, 60);
    /* Current Time */
    CurrentTime = HDIV_Mod(LED_Setting->TimeCounter, TotalDuration);
    /* Unit time for each color */
    deltaT = HDIV_Div(TotalDuration, RainbowSize);

    /* Reset CountingTime */
    if(LED_Setting->TimeCounter >= TotalDuration)
    {
        LED_Setting->TimeCounter -= TotalDuration;

        g_u8OneShot_Flag = 1;
    }

    /* Mix Color */
    for(j = 0; j < LED_Setting->LEDNum; j++, CurrentTime += NextTime)
    {
        if(CurrentTime >= TotalDuration)
            CurrentTime -= TotalDuration;

        if(CurrentTime >= (deltaT * RainbowSize))
        {
            /* Color Cursor */
            ColorCursor1 = 0;
            ColorCursor2 = 1;

            for(i = 0; i < 3; i++)
            {
                DisplayColor[LED_Setting->LEDNum-j-1][i] = RainbowColor[ColorCursor1][i];
            }
        }
        else
        {
            /* Color Cursor */
            ColorCursor1 = HDIV_Mod(HDIV_Div(CurrentTime, deltaT), RainbowSize);
            ColorCursor2 = HDIV_Mod((ColorCursor1 + 1), RainbowSize);

            for(i = 0; i < 3; i++)
            {
                DisplayColor[LED_Setting->LEDNum-j-1][i] = HDIV_Div(((RainbowColor[ColorCursor1][i] * ((ColorCursor1 + 1) * deltaT - CurrentTime)) + \
                                                           (RainbowColor[ColorCursor2][i] * (CurrentTime - ColorCursor1 * deltaT))), deltaT);
            }
        }
    }

    /* Mapping Color to LED Format */
    /* Direction forward or backward */
    if(LED_Setting->Direction == Dir_Forward)
        Set_Array(LED_Setting->LED_Data, LED_Setting->LEDNum, (uint8_t *)DisplayColor, LED_Setting->Brightness);
    else if(LED_Setting->Direction == Dir_Backward)
        Set_InverseArray(LED_Setting->LED_Data, LED_Setting->LEDNum, (uint8_t *)DisplayColor, LED_Setting->Brightness);
}
