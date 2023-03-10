#include "bsp_rc.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	SET_BIT(huart1.Instance->CR3,USART_CR3_DMAR);
	
	//使能空闲中断
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	
	__HAL_DMA_DISABLE(&hdma_usart1_rx);
	//DMA_SxCR是一个仲裁器，用来看优先级的
  //传输开始的时候，使能数据流（DMA_SxCR_EN位置1）结束的时候清零
	while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
	{
		//断掉串口保护数据
		__HAL_DMA_DISABLE(&hdma_usart1_rx);
	}
	
	hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //memory buffer 1
    //内存缓冲区1
  hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
  hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
  hdma_usart1_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
  SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
  __HAL_DMA_ENABLE(&hdma_usart1_rx);
	
}
