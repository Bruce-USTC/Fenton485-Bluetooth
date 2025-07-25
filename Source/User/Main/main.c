// STM32��׼�����ͷ�ļ�
#include "stm32f10x.h"
#include <string.h>

// RS485 ����������Ŷ���
#define DIR_GPIO_PORT   GPIOA
#define DIR_PIN         GPIO_Pin_1

// ����������ȷ����ʹ��ǰ�ɼ�
static __inline void DIR_Set(uint8_t a);

// ���λ�������С
#define RX_BUFFER_SIZE 256

// �������ջ��λ�����
uint8_t bt_rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t bt_rx_read_ptr = 0;
volatile uint16_t bt_rx_write_ptr = 0;

// RS485 ���ջ��λ�����
uint8_t rs485_rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rs485_rx_read_ptr = 0;
volatile uint16_t rs485_rx_write_ptr = 0;

// SysTick ����������
static volatile uint32_t msTicks = 0;

// ======================= ȫ����������ʼ =======================
// ���巢�͵�RS485������֡��֡ͷ
// ע�⣺���֡ͷ����STM32���������ӵ�RS485���ݰ�ǰ�ģ�
// ����ģ�鷢��ԭʼ����ʱ����Ҫ������֡ͷ��
#define RS485_FRAME_HEADER 0xAA
// ======================= ȫ������������ =======================

// �������� (ȫ����ǰ���Ա������֪�����ǵĴ���)
int16_t get_data_from_buffer(uint8_t *buffer, volatile uint16_t *read_ptr, volatile uint16_t *write_ptr, uint16_t size);
int8_t put_data_to_buffer(uint8_t data, uint8_t *buffer, volatile uint16_t *read_ptr, volatile uint16_t *write_ptr, uint16_t size);
void Init_Usart(void);
void Usart_Configuration(USART_TypeDef* USARTx, uint32_t BaudRate);
void Init_NVIC(void);
void Delay_Ms(uint32_t time);
void RS485_Send_Byte(uint8_t Data);
void Bluetooth_Send_Byte(uint8_t Data);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void Hardware_Init(void);
void Process_Uart_Data(void);
void SysTick_Handler(void);
uint16_t CaculateCRC(uint8_t *Data, uint16_t len);


// ======================= ����ʵ�� =======================

static __inline void DIR_Set(uint8_t a) {
    if (a) {
        GPIO_SetBits(DIR_GPIO_PORT, DIR_PIN);
    } else {
        GPIO_ResetBits(DIR_GPIO_PORT, DIR_PIN);
    }
}

int16_t get_data_from_buffer(uint8_t *buffer, volatile uint16_t *read_ptr, volatile uint16_t *write_ptr, uint16_t size) {
    int16_t data = -1;
    __disable_irq();
    if (*read_ptr != *write_ptr) {
        data = buffer[*read_ptr];
        *read_ptr = (*read_ptr + 1) % size;
    }
    __enable_irq();
    return data;
}

int8_t put_data_to_buffer(uint8_t data, uint8_t *buffer, volatile uint16_t *read_ptr, volatile uint16_t *write_ptr, uint16_t size) {
    uint16_t next_write_ptr = (*write_ptr + 1) % size;
    __disable_irq();
    if (next_write_ptr == *read_ptr) {
        __enable_irq();
        return -1;
    } else {
        buffer[*write_ptr] = data;
        *write_ptr = next_write_ptr;
        __enable_irq();
        return 0;
    }
}

// CRC ���㺯��ʵ�� (����ԭʼ���������������ߵ��ֽ�)
uint16_t CaculateCRC(uint8_t *Data, uint16_t len)
{
    uint16_t n, i;
    uint16_t crc = 0xffff;
    for (n = 0; n < len; n++)
    {
        crc ^= Data[n];
        for (i = 0; i < 8; i++)
        {
            if ((crc & 1) == 1)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc; // ���ؼ���õ���CRC��
}


void SysTick_Handler(void) {
    msTicks++;
}

void Delay_Ms(uint32_t time) {
    uint32_t current_ticks = msTicks;
    while ((msTicks - current_ticks) < time);
}


void RS485_Send_Byte(uint8_t Data)
{
    DIR_Set(1);
    while(!USART_GetFlagStatus(USART2, USART_FLAG_TXE));
    USART_SendData(USART2, Data);
    while(!USART_GetFlagStatus(USART2, USART_FLAG_TC));
    DIR_Set(0);
}

void Bluetooth_Send_Byte(uint8_t Data)
{
    while(!USART_GetFlagStatus(USART3, USART_FLAG_TXE));
    USART_SendData(USART3, Data);
}


void Init_Usart(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = DIR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DIR_GPIO_PORT, &GPIO_InitStructure);
    DIR_Set(0);
}


void Usart_Configuration(USART_TypeDef* USARTx, uint32_t BaudRate)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate              = BaudRate;
    USART_InitStructure.USART_WordLength            = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits              = USART_StopBits_1;
    USART_InitStructure.USART_Parity                = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USARTx, &USART_InitStructure);
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);

    USART_Cmd(USARTx, ENABLE);
}

void USART3_IRQHandler(void)
{
    uint8_t ReceiveData;
    if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
    {
        USART_ReceiveData(USART3);
    }

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        ReceiveData = USART_ReceiveData(USART3);
        put_data_to_buffer(ReceiveData, bt_rx_buffer, &bt_rx_read_ptr, &bt_rx_write_ptr, RX_BUFFER_SIZE);
    }
    USART_ClearITPendingBit(USART3, USART_IT_FE);
    USART_ClearITPendingBit(USART3, USART_IT_NE);
    USART_ClearITPendingBit(USART3, USART_IT_PE);
}

void USART2_IRQHandler(void)
{
    uint8_t ReceiveData;
    if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
    {
        USART_ReceiveData(USART2);
    }

    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        ReceiveData = USART_ReceiveData(USART2);
        put_data_to_buffer(ReceiveData, rs485_rx_buffer, &rs485_rx_read_ptr, &rs485_rx_write_ptr, RX_BUFFER_SIZE);
    }
    USART_ClearITPendingBit(USART2, USART_IT_FE);
    USART_ClearITPendingBit(USART2, USART_IT_NE);
    USART_ClearITPendingBit(USART2, USART_IT_PE);
}


void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART1);
    }
    USART_ClearITPendingBit(USART1, USART_IT_ORE);
    USART_ClearITPendingBit(USART1, USART_IT_NE);
    USART_ClearITPendingBit(USART1, USART_IT_FE);
    USART_ClearITPendingBit(USART1, USART_IT_PE);
}


void Hardware_Init(void)
{
    SystemInit();
    if (SysTick_Config(SystemFrequency / 1000)) {
        while (1);
    }

    Init_NVIC();
    Init_Usart();

    Usart_Configuration(USART2, 9600);
    Usart_Configuration(USART3, 9600);
}

// Process_Uart_Data ���� (STM32 ������֡ͷ)
void Process_Uart_Data(void)
{
    static uint8_t bt_packet_buffer[6]; // �����ݴ��������յ���6�������ֽ�
    static uint8_t bt_packet_idx = 0;   // �������ݰ���ǰ����

    int16_t bt_data;
    int16_t rs485_data;
    uint16_t crc_value;
    uint8_t i; // ����ѭ������ i

    // ����������������
    bt_data = get_data_from_buffer(bt_rx_buffer, &bt_rx_read_ptr, &bt_rx_write_ptr, RX_BUFFER_SIZE);

    if (bt_data != -1) // ���������ݿ���
    {
        // STM32 �������ն˲��ٽ���֡ͬ����
        // ����������ģ��ᷢ��׼ȷ�� 6 ���ֽ���Ϊ���ݰ���
        // ��ˣ��������Ͷ���Ҫȷ����
        // 1. ÿ��ֻ���� 6 ���ֽڵ���Ч���ݡ�
        // 2. ���ݰ�֮�����㹻�ļ�����Ա�������ճ�����λ��
        // ������������˶��� 6 ���ֽڵ����ݣ�������ֽڽ���������

        if (bt_packet_idx < 6) {
            // ��δ������6���ֽڣ�����ǰ�ֽڴ��뻺����
            bt_packet_buffer[bt_packet_idx++] = (uint8_t)bt_data;
        } else {
            // �Ѿ����յ�6���ֽڣ��������ݽ�������
            // (���磬����������� "01 06 00 00 00 00 89 CA"����ô '89' �� 'CA' ��������)
        }

        // �����յ��㹻��6���ֽ�ʱ����������
        if (bt_packet_idx == 6) {
            // ����ǰ6���ֽڵ�CRCУ����
            crc_value = CaculateCRC(bt_packet_buffer, 6);

            // --- �ؼ��޸ģ��ڷ��͸� RS485 ֮ǰ���ȷ���֡ͷ ---
            RS485_Send_Byte(RS485_FRAME_HEADER); // �ȷ���֡ͷ (0xAA)

            // ��6�������ֽڷ��͵�RS485
            for (i = 0; i < 6; i++) {
                RS485_Send_Byte(bt_packet_buffer[i]);
            }
            // ���� Modbus CRC16 ��׼���ȷ��͵��ֽڣ��ٷ��͸��ֽ�
            RS485_Send_Byte((uint8_t)(crc_value & 0x00FF));       // ���� CRC ���ֽ� (���� 0x48)
            RS485_Send_Byte((uint8_t)((crc_value >> 8) & 0x00FF)); // ���� CRC ���ֽ� (���� 0x0A)

            // ������ɺ�����������׼��������һ��6�ֽڵİ�
            bt_packet_idx = 0;
            // ���������һ��С����ʱ����ѡ�����Ƽ�������RS485���ն��㹻�Ĵ���ʱ�䣬
            // ���������������Ͷ����ʱ������������ȶ��ԡ�
            // Delay_Ms(10); // ���磬��ʱ10���룬����ʵ���������
        }
    }

    // ����RS485�������� (�ⲿ���߼����ֲ���)
    rs485_data = get_data_from_buffer(rs485_rx_buffer, &rs485_rx_read_ptr, &rs485_rx_write_ptr, RX_BUFFER_SIZE);
    if (rs485_data != -1)
    {
        Bluetooth_Send_Byte((uint8_t)rs485_data);
    }
}


int main(void)
{
    Hardware_Init();

    while(1)
    {
        Process_Uart_Data();
    }
}

void Init_NVIC(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    #ifdef  VECT_TAB_RAM
      NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
    #else
      NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
    #endif

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
