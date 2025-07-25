/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** ���ڵײ㺯������
** �汾��Rev1.0
** ���ڣ�2018-6-20
** ���ܣ��ṩUSART2(RS485)��USART3(����)�ĵײ�����
** printf�ض���USART3
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
//ͷ�ļ�����
#include "usually.h" // �������õĺ궨������ͣ����ܰ��� u8 ��
#include "usart.h"   // ����USART��صĺꡢ������������������������

#include "stm32f10x.h" // STM32��׼�����ͷ�ļ�
#include <stdio.h>     // ���� fputc ����
#include <string.h>    // ���� memset


// --- printf �����ض��� USART3 (����) ---
// �ò��ִ���֧��printf������������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting) // ���ð�����ģʽ

// ��׼����Ҫ��֧�ֺ���
struct __FILE
{
    int handle;
};
FILE __stdout;

// ����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
    x = x; // ������������棬ʵ���޲���
}

// �ض��� fputc ������ʹ��ͨ�� USART3 �����ַ�
int fputc(int Data, FILE *f)
{
    // �ȴ� USART3 �ķ������ݼĴ���Ϊ�գ�ȷ����һ�������ѷ������
    while(!USART_GetFlagStatus(USART3, USART_FLAG_TXE));
    // ͨ�� USART3 ���͵�ǰ�ַ�
    USART_SendData(USART3, Data);
    return Data;
}
#endif // #if 1


// --- �������ջ���������ر������� ---
// ��Щ������ usart.h ������Ϊ extern
volatile char bluetoothRxBuffer[BT_RX_BUFFER_SIZE];
volatile uint8_t bluetoothRxIndex = 0;
volatile uint8_t bluetoothDataReady = 0; // 0: δ����, 1: �Ѿ���


// --- RS485 ���ջ���������ر������� ---
// ��Щ������ usart.h ������Ϊ extern
//uint8_t Rs485Buffer[100] = {0};
//uint8_t Rs485Txflag = 0;
//uint8_t index_i = 0; // RS485����������

uint8_t Rs485Buffer[100] ;
// uint8_t Rs485Txflag = 0; // �����־���Ա����������ÿ���֡�жϴ���
uint8_t index_i ; // RS485����������

// ����һ����־λ����ʾRS485�������һ֡����
volatile uint8_t Rs485_Frame_Ready ;
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** ��������: RS485_Send_Byte
** ��������: ͨ�� USART2 ����һ���ֽ� (RS485)
** ����������Data Ҫ���͵�����
** ��    ��: Dream
** ��    ��: 2018��6��20��
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void RS485_Send_Byte(uint16_t Data)
{
    // �ȴ� USART2 �ķ������ݼĴ���Ϊ��
    while(!USART_GetFlagStatus(USART2, USART_FLAG_TXE));
    // ����һ���ֽ�
    USART_SendData(USART2, Data);
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** ��������: Init_Usart
** ��������: �������ų�ʼ�� (USART2����RS485��USART3��������)
** ��������: ��
** ��    ��: Dream
** ��    ��: 2011��6��20��
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Init_Usart(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // --- ���� (USART3) GPIO ���� ---
    // ʹ�� GPIOC, USART3, AFIO ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    // ������ӳ�䣺USART3ʹ��PC10 (TX) �� PC11 (RX)
    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

    // TX3: PC10 ����Ϊ���ù����������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // RX3: PC11 ����Ϊ��������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);


    // --- RS485 (USART2) GPIO ���� ---
    // ʹ�� GPIOA, USART2, AFIO ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // TX2: PA2 ����Ϊ���ù����������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // RX2: PA3 ����Ϊ��������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // RS485 �շ��л����� (DIR): PA1 ����Ϊͨ���������
    GPIO_InitStructure.GPIO_Pin = DIR_PIN; // ʹ�ú궨���е�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DIR_GPIO_PORT, &GPIO_InitStructure); // ʹ�ú궨���еĶ˿�
    DIR(0); // ��ʼ����Ϊ����ģʽ (�͵�ƽ)
}


/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** ��������: Usart_Configuration
** ��������: ����ͨ�����ú���
** ��������: USARTx ���ں�, BaudRate ������
** ��    ��: Dream
** ��    ��: 2011��6��20��
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Usart_Configuration(USART_TypeDef* USARTx, uint32_t BaudRate)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate          = BaudRate;
    USART_InitStructure.USART_WordLength        = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits          = USART_StopBits_1;
    USART_InitStructure.USART_Parity            = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode              = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USARTx, &USART_InitStructure); // ��ʼ����������
    
    // !!! �޸����ʹ�ܽ����жϺͿ���֡�ж� !!!
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);   // ʹ�ܴ��ڽ����ж�
    USART_ITConfig(USARTx, USART_IT_IDLE, ENABLE);   // ʹ�ܴ��ڿ���֡�ж� (�ؼ�)
    
    __nop(); __nop(); // ������ʱ��ȷ��������Ч
    USART_Cmd(USARTx, ENABLE); // ʹ�ܴ�������
}


///*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//** ��������: USART3_IRQHandler
//** ��������: USART3 �����жϴ����� (�������ݽ���)
//** ��������: ��
//** ��    ��: Dream
//** ��    ��: 2011��6��20��
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
//void USART3_IRQHandler(void)
//{
//    uint8_t ReceiveData;
//    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
//    {
//        // LED3_TOGGLE; // ���յ���������ʱ��תLED3��Ϊ����ָʾ (���Ա���)
//        ReceiveData = USART_ReceiveData(USART3);

//        if (bluetoothRxIndex < (BT_RX_BUFFER_SIZE - 1))
//        {
//            bluetoothRxBuffer[bluetoothRxIndex++] = (char)ReceiveData;
//        }

//        // ������յ����з�('\n')�򻺳���������������Ϊһ��ָ��������
//        // ע�⣺���������������ַ��͵�ָ��ĩβ���� '\r\n'
//        // ���ߣ������ϣ��ʵʱת��������ȥ�����жϣ�ÿ���յ�һ���ֽھ����ñ�־��������ѭ���д���ʱ��Ҫ��С��
//        if (ReceiveData == '\n' || bluetoothRxIndex == (BT_RX_BUFFER_SIZE - 1))
//        {
//            bluetoothRxBuffer[bluetoothRxIndex] = '\0'; // ����ַ���������
//            bluetoothDataReady = 1; // �������ݾ�����־��֪ͨ��ѭ������
//            bluetoothRxIndex = 0;   // ����������׼��������һ��ָ��
//        }
//        // else if (ReceiveData == '\r') { // �����Ҫ�����Ե�������س���
//        //     // ���Իس���������������Ҳ���� bluetoothDataReady = 1;
//        // }

//        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
//    }
//}
///*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//** ��������: USART2_IRQHandler
//** ��������: USART2 �����жϴ����� (RS485���ݽ���)
//** ��������: ��
//** ��    ��: Dream
//** ��    ��: 2011��6��20��
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
//void USART2_IRQHandler(void) // ���void�Թ淶����ǩ��
//{
//    uint8_t ReceiveData;

//    // ������֡�жϱ�־λ
//    if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
//    {
//        // ���������� IDLE ��־: �ȶ� SR���ٶ� DR
//        USART_GetFlagStatus(USART2, USART_FLAG_IDLE); // ��� IDLE ��־λ����SR��
//        ReceiveData = USART_ReceiveData(USART2);      // ��� IDLE ��־λ����DR�����ֵ�ǿ���ǰ���һ���ֽڣ�

//        // ��������������ݣ�index_i > 0�������ʾһ֡���ݽ������
//        if (index_i > 0) {
//            Rs485_Frame_Ready = 1; // ����֡������־
//        }
//        // ����Ҫ��ջ�����������������������ѭ����������
//    }

//    // ���������ݼĴ����ǿ��жϱ�־λ
//    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
//    {
//        LED2_TOGGLE; // ���յ�RS485����ʱ��תLED2��Ϊ����ָʾ
//        ReceiveData = USART_ReceiveData(USART2); // ��ȡ���յ�������

//        // �����յ������ݴ���RS485������
//        if (index_i < sizeof(Rs485Buffer)) // ȷ�������������
//        {
//            Rs485Buffer[index_i++] = ReceiveData;
//        }
//        else {
//            // ����������������Զ����������ݣ������������� Rs485_Frame_Ready = 1;
//            // ����򵥴���Ϊ������������Ϊ��ǰ֡�����������ñ�־��
//            Rs485_Frame_Ready = 1;
//        }
//        // ��������жϱ�־λ
//        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//    }
//}
///**
//  * @brief  This function handles USART1 global interrupt request.
//  * @param  None
//  * @retval None
//  */
//void USART1_IRQHandler(void)
//{
//  /* ����жϱ�־λ */
//  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//  {
//    /* ���ջ������ǿ��жϴ��� */
//    uint8_t data = USART_ReceiveData(USART1); // ��ȡ���յ�������
//    
//    /* ������յ������� */
//    // ...
//    
//    /* ����жϱ�־ */
//    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
//  }
//  
//  /* �������ܵ��жϴ��� */
//  // ...
//}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
End:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
