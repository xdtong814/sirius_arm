#include "bsp_rc.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	SET_BIT(huart1.Instance->CR3,USART_CR3_DMAR);
	
	//ʹ�ܿ����ж�
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	
	__HAL_DMA_DISABLE(&hdma_usart1_rx);
	//DMA_SxCR��һ���ٲ��������������ȼ���
  //���俪ʼ��ʱ��ʹ����������DMA_SxCR_ENλ��1��������ʱ������
	while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
	{
		//�ϵ����ڱ�������
		__HAL_DMA_DISABLE(&hdma_usart1_rx);
	}
	
	hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //memory buffer 1
    //�ڴ滺����1
  hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //�ڴ滺����2
  hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //���ݳ���
  hdma_usart1_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //ʹ��˫������
  SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //ʹ��DMA
  __HAL_DMA_ENABLE(&hdma_usart1_rx);
	
}
