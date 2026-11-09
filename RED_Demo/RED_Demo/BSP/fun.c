/**
 * @file fun.c
 * @brief ������ơ����������١�OLED ��ʾ�� USART3 ״̬�ϱ���
 * @note ���������鰴 A��B��C��D ���У���Ӧ TIM8��TIM3��TIM4��TIM2��
 *       TIM5 ÿ 10ms ��������ʾ�����봮�����������ѭ����ִ�С�
 */
#include "headfile.h"

char str[32];                  /* OLED ���и�ʽ���������� */
char uart_rx[32];              /* ���ڽ��ջ����������ݲ���֤���ַ�����������β�� */
/* �ж�����ѭ�����������ݣ�10ms �ڴ��������������ת�٣���λ RPM�� */
volatile int32_t Encoder_Count[4] = {0};
volatile int32_t Speed[4] = {0};
static uint32_t Encoder_Previous[4] = {0};
/* ������ÿȦ���ı�Ƶ������384 Ϊ���ò������������ʱӦ������ٱȡ� */
static const uint32_t Encoder_CountsPerRev[4] = {384, 384, 384, 384};
volatile uint8_t uart_flag;    /* �����¼���־���� 1 ��ʾ���������¼��� */
volatile uint16_t uart_rx_size; /* ���һ�ν����¼��������Ч�ֽ����� */
static char uart_tx[384];      /* �жϷ����ڼ���뱣����Ч�Ҳ��ø�д�� */
volatile uint32_t Vision_UART_ErrorCount = 0;
volatile uint32_t Vision_UART_LastError = 0;
volatile uint32_t Vision_UART_ArmErrorCount = 0;

/* Check and expose RX arming failures instead of silently ignoring HAL status. */
static void Vision_UART_Arm(void)
{
    if (HAL_UARTEx_ReceiveToIdle_IT(&huart3, (uint8_t *)uart_rx, sizeof(uart_rx)) != HAL_OK)
        Vision_UART_ArmErrorCount++;
}

volatile uint32_t UART_TxCount = 0; /* �ɹ��ύ�� HAL �ķ��ʹ��������ǵ���ȷ�ϴ����� */
volatile uint32_t UART_TxErrorCount = 0; /* HAL �ܾ�����������ۼƴ����� */

/** @brief ����ָ������ķ����ƽ��
 *  @param mode �����ţ�1=A��2=B��3=C��4=D������ֵ��ִ�в�����
 *  @note ʵ��ת��ȡ���ڵ�����ߣ��ٶ��� TIM1 �� CCR �������ơ� */
void Motor_Statu(uint8_t mode)
{
	if(mode == 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	}
	else if(mode == 2)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	}
	else if(mode == 3)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	}	
	else if(mode == 4)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
	}
}

/** @brief ��������ת�û� LED��
 *  @param time ���η�ת�ļ������λ ms��������������ԼΪ 2*time�� */
void LED_Blink(uint32_t time)
{
    static uint32_t blink_time = 0;

    if ((uint32_t)(uwTick - blink_time) >= time)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        blink_time = uwTick;
    }
}

/* Call immediately after GPIO init; no UART, timer or target is needed. */
void OLED_PowerOn(void)
{
    OLED_Init();
    if (!OLED_Ready) return;
    OLED_Clear();
    OLED_ShowString(0, 0, "SYSTEM STARTING", OLED_6X8);
    OLED_ShowString(0, 8, "V:OFF R:0 DX:--", OLED_6X8);
    OLED_ShowString(0, 16, "M:0/- STOP PWM:0%", OLED_6X8);
    OLED_ShowString(0, 24, "WAIT MAIXCAM", OLED_6X8);
    OLED_ShowString(0, 32, "A:0 RPM", OLED_6X8);
    OLED_ShowString(0, 40, "B:0 RPM", OLED_6X8);
    OLED_ShowString(0, 48, "C:0 RPM", OLED_6X8);
    OLED_ShowString(0, 56, "D:0 RPM", OLED_6X8);
    OLED_Update();
    if (OLED_I2C_LastError != 0) OLED_Ready = 0;
}

/* 128x64 OLED: eight 6x8 text rows, refreshed every 100 ms. */
void OLED_Display(void)
{
    static uint32_t last_display = 0;
    static uint8_t first_display = 1;
    uint32_t now = HAL_GetTick();
    uint8_t i, phase, motor, pwm;
    uint32_t remaining_tenths;
    const char *direction;
    int32_t rpm[4];
    uint32_t primask;
    if (!first_display && (uint32_t)(now - last_display) < 100U) return;
    first_display = 0;
    last_display = now;
    if (!OLED_Ready)
    {
        static uint32_t last_retry = 0;
        if ((uint32_t)(now - last_retry) < 1000U) return;
        last_retry = now;
        OLED_Init();
        if (!OLED_Ready) return;
    }
    /* ���ٽ���������·ת�٣��ָ�ԭ���ж�״̬�����������;���¡� */
    primask = __get_PRIMASK();
    __disable_irq();
    for (i = 0; i < 4; i++) rpm[i] = Speed[i];
    __set_PRIMASK(primask);
    phase = Vision_MotorPhase;
    motor = Vision_MotorRegion;
    pwm = (phase == 1 || phase == 3) ? VISION_MOTOR_PWM : 0;
    direction = phase == 1 ? "CW" : phase == 2 ? "PAUSE" :
                phase == 3 ? "CCW" : phase == 4 ? "COOL" : "STOP";
    remaining_tenths = (Vision_Motor_RemainingMs() + 99U) / 100U;
    OLED_Clear();
    if (Vision_TreeDX == 9999)
        sprintf(str, "V:%s R:%u DX:--", Vision_Online ? "ON" : "OFF", Vision_Region);
    else
        sprintf(str, "V:%s R:%u DX:%ld", Vision_Online ? "ON" : "OFF",
                Vision_Region, (long)Vision_TreeDX);
    OLED_ShowString(0, 0, str, OLED_6X8);
    sprintf(str, "M:%u/%c %s PWM:%u%%", motor, motor ? 'A' + motor - 1 : '-',
            direction, pwm);
    OLED_ShowString(0, 8, str, OLED_6X8);
    sprintf(str, "LEFT:%lu.%lus %s", (unsigned long)(remaining_tenths / 10U),
            (unsigned long)(remaining_tenths % 10U),
            phase ? "BUSY" : (Vision_Motor_IsArmed() ? "READY" : "WAIT"));
    OLED_ShowString(0, 16, str, OLED_6X8);
    sprintf(str, "RX:%lu", (unsigned long)Vision_FrameCount);
    OLED_ShowString(0, 24, str, OLED_6X8);
    for (i = 0; i < 4; i++)
    {
        sprintf(str, "%c:%ld RPM", 'A' + i, (long)rpm[i]);
        OLED_ShowString(0, 32 + i * 8, str, OLED_6X8);
    }
    OLED_I2C_LastError = 0;
    OLED_Update();
    if (OLED_I2C_LastError != 0) OLED_Ready = 0;
}


/** @brief USART3 ÿ 500ms �ϱ���· RPM �� CAN ״̬��
 *  @note PB10=TX��PB11=RX��115200bit/s��8N1����ѭ�����ã��ж���ɷ��͡�
 *        UART æʱ�������η��ͣ����ȴ������������ڷ��͵Ļ������� */
void UART_SendStatus(void)
{
    static uint32_t last_uart = 0;
    uint32_t now = HAL_GetTick();
    uint32_t primask;
    int32_t rpm[4];
    uint8_t i;
    int length;
    if ((uint32_t)(now - last_uart) < 500U ||
        huart3.gState != HAL_UART_STATE_READY) return;
    /* ���ٽ���������·ת�٣��ָ�ԭ���ж�״̬�����������;���¡� */
    primask = __get_PRIMASK();
    __disable_irq();
    for (i = 0; i < 4; i++) rpm[i] = Speed[i];
    __set_PRIMASK(primask);
    /* ʹ�þ�̬����������֤ HAL �жϷ��ͽ���ǰ����һֱ��Ч�� */
    if ((uint32_t)(now - last_uart) >= 500U && huart3.gState == HAL_UART_STATE_READY)
    {
        last_uart = now;
        length = snprintf(uart_tx, sizeof(uart_tx),
            "A:%ld RPM B:%ld RPM C:%ld RPM D:%ld RPM | WORK:%u TASK:%lu RESULT:%u | CAN2 WORK %s TX:%lu RX:%lu ERR:%lu S:%u | OLED:%u OE:%lu NACK:%lu\r\n",
            (long)rpm[0], (long)rpm[1], (long)rpm[2], (long)rpm[3],
            Vision_WorkState, (unsigned long)Vision_WorkTask, Vision_WorkResult,
            CAN_Test_State == 1 ? "OK" : (CAN_Test_State == 0 ? "WAIT" : "FAIL"),
            (unsigned long)CAN_Test_TxCount, (unsigned long)CAN_Test_RxCount,
            (unsigned long)CAN_Test_ErrorCount, CAN_Test_State,
            OLED_Ready, (unsigned long)OLED_I2C_LastError,
            (unsigned long)OLED_I2C_NackCount);
        if (length < 0 || length >= (int)sizeof(uart_tx))
        {
            UART_TxErrorCount++;
            return;
        }
        if (HAL_UART_Transmit_IT(&huart3, (uint8_t *)uart_tx, (uint16_t)length) == HAL_OK)
            UART_TxCount++;
        else UART_TxErrorCount++;
    }
}

/** @brief ��ʼ�� OLED�����ڽ��ա�PWM����·�������� 10ms ������
 *  @note ������ CubeMX ���ɵĸ� MX_xxx_Init() ������ɺ�ִ�С� */
void Init(void)
{
  Vision_Motor_Init();

    /* USART3 interrupt reception, no DMA. */
	Vision_UART_Arm();

	//PWM���������
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);	
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

	// ��������Ӳ���Զ����������������������жϡ�
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);	
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);	
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
	


	// ��¼��ʼ������������±�־������ TIM5 ���ڲ�����
	Encoder_Previous[0] = __HAL_TIM_GET_COUNTER(&htim8);
  Encoder_Previous[1] = __HAL_TIM_GET_COUNTER(&htim3);
  Encoder_Previous[2] = __HAL_TIM_GET_COUNTER(&htim4);
  Encoder_Previous[3] = __HAL_TIM_GET_COUNTER(&htim2);
  __HAL_TIM_SET_COUNTER(&htim5, 0);
  __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim5);
	
	
	/* ARR=99; start with every motor PWM disabled. */
	TIM1->ARR = 99;

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;	
}

/** @brief TIM5 �����жϣ�ͨ�����ڲ�����ֵ������·ת�١�
 *  @param htim ���������¼��Ķ�ʱ�������
 *  @note �����������������У������ȡ������������嶪ʧ��
 *        ÿ�� 10ms ��������ڵļ����仯����С�ڼ�����Χ��һ�롣 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    TIM_HandleTypeDef *encoders[4] = {&htim8, &htim3, &htim4, &htim2};
    uint8_t i;
    uint32_t current, difference;
    int64_t delta;
    if (htim->Instance != TIM5) return;
    for (i = 0; i < 4; i++)
    {
        current = __HAL_TIM_GET_COUNTER(encoders[i]);
        difference = current - Encoder_Previous[i];
        Encoder_Previous[i] = current;
        if (i == 3) /* TIM2 ʹ�� 32 λ������Χ�� */
            delta = (difference & 0x80000000U) ?
                    (int64_t)difference - 4294967296LL : difference;
        else /* TIM3��TIM4��TIM8 ʹ�� 16 λ������Χ�� */
        {
            difference &= 0xFFFFU;
            delta = (int64_t)difference;
            if (difference & 0x8000U) delta -= 65536LL;
        }
        Encoder_Count[i] = (int32_t)delta;
        /* RPM = ������ֵ / ÿȦ���� * (1000/10) * 60�����ű�ʾ�������� */
        Speed[i] = (int32_t)(delta * 6000 / Encoder_CountsPerRev[i]);
    }
}


/* Feed all received bytes before rearming the shared RX buffer. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance==USART3)
	{
		uart_flag=1;
    uart_rx_size = Size;
    Vision_Motor_Receive((const uint8_t *)uart_rx, Size);
    /* USART3 interrupt reception, no DMA. */
	Vision_UART_Arm();
	}
}

/* Recover framing/overrun errors and rearm reception. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;
    Vision_UART_LastError = HAL_UART_GetError(huart);
    Vision_UART_ErrorCount++;
    Vision_Motor_RxError();
    (void)HAL_UART_AbortReceive(huart);
    Vision_UART_Arm();
}

void Vision_UART_Task(void)
{
    if (huart3.RxState == HAL_UART_STATE_READY)
        Vision_UART_Arm();
    Vision_Motor_Task();
}
