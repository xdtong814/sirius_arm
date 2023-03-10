#include "bsp_can.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/*
void CAN_SetMsg_Leg1_Shoulder(void)
{
	BLDC_tx_message.StdId = mot_leg1_shoulder_id;
  BLDC_tx_message.IDE = CAN_ID_STD; //format
  BLDC_tx_message.RTR = CAN_RTR_DATA; //type
  BLDC_tx_message.DLC = 0x08;
}
*/
void can_filter_init()
{
  CAN_FilterTypeDef can_filter_st;
	  
	can_filter_st.FilterBank = 0;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = 0x0000 ;
  can_filter_st.FilterIdLow =  0x0000;
  can_filter_st.FilterMaskIdHigh = 0x00000;	
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	  
    //CAN_FilterTypeDef can_filter_st_2;
	can_filter_st.FilterBank = 14;
    //can_filter_st.FilterActivation = ENABLE;
    //can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
    //can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    //can_filter_st.FilterIdHigh = (((uint32_t)can_rec_id<<3)&0xFFFF0000)>>16;
    //can_filter_st.FilterIdLow = (((uint32_t)can_rec_id<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF;
    //can_filter_st.FilterMaskIdHigh =  0xFFFF;
    //can_filter_st.FilterMaskIdLow = 0xFF80;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

HAL_StatusTypeDef CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox)
{
  uint32_t transmitmailbox;
  HAL_CAN_StateTypeDef state = hcan->State;
  uint32_t tsr = READ_REG(hcan->Instance->TSR);

  /* Check the parameters */
  assert_param(IS_CAN_IDTYPE(pHeader->IDE));
  assert_param(IS_CAN_RTR(pHeader->RTR));
  assert_param(IS_CAN_DLC(pHeader->DLC));
  if (pHeader->IDE == CAN_ID_STD)
  {
    assert_param(IS_CAN_STDID(pHeader->StdId));
  }
  else
  {
    assert_param(IS_CAN_EXTID(pHeader->ExtId));
  }
  assert_param(IS_FUNCTIONAL_STATE(pHeader->TransmitGlobalTime));

  if ((state == HAL_CAN_STATE_READY) ||
      (state == HAL_CAN_STATE_LISTENING))
  {
    /* Check that all the Tx mailboxes are not full */
    if (((tsr & CAN_TSR_TME0) != 0U) ||
        ((tsr & CAN_TSR_TME1) != 0U) ||
        ((tsr & CAN_TSR_TME2) != 0U))
    {
      /* Select an empty transmit mailbox */
      transmitmailbox = (tsr & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;

      /* Check transmit mailbox value */
      if (transmitmailbox > 2U)
      {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_INTERNAL;

        return HAL_ERROR;
      }

      /* Store the Tx mailbox */
      *pTxMailbox = (uint32_t)1 << transmitmailbox;

      /* Set up the Id */
			 hcan->Instance->sTxMailBox[transmitmailbox].TIR &= CAN_TI0R_TXRQ;
      if (pHeader->IDE == CAN_ID_STD)
      {
        hcan->Instance->sTxMailBox[transmitmailbox].TIR = ((pHeader->StdId << CAN_TI0R_STID_Pos) |
                                                           pHeader->RTR);
      }
      else
      {
        hcan->Instance->sTxMailBox[transmitmailbox].TIR = ((pHeader->ExtId << CAN_TI0R_EXID_Pos) |
                                                           pHeader->IDE |
                                                           pHeader->RTR);
      }

      /* Set up the DLC */
      hcan->Instance->sTxMailBox[transmitmailbox].TDTR &= (uint32_t)0xFFFFFFF0;
			hcan->Instance->sTxMailBox[transmitmailbox].TDTR |= (pHeader->DLC & (uint8_t)0x0000000F);

      /* Set up the Transmit Global Time mode */
      if (pHeader->TransmitGlobalTime == ENABLE)
      {
        SET_BIT(hcan->Instance->sTxMailBox[transmitmailbox].TDTR, CAN_TDT0R_TGT);
      }

      /* Set up the data field */
      WRITE_REG(hcan->Instance->sTxMailBox[transmitmailbox].TDHR,
                ((uint32_t)aData[7] << CAN_TDH0R_DATA7_Pos) |
                ((uint32_t)aData[6] << CAN_TDH0R_DATA6_Pos) |
                ((uint32_t)aData[5] << CAN_TDH0R_DATA5_Pos) |
                ((uint32_t)aData[4] << CAN_TDH0R_DATA4_Pos));
      WRITE_REG(hcan->Instance->sTxMailBox[transmitmailbox].TDLR,
                ((uint32_t)aData[3] << CAN_TDL0R_DATA3_Pos) |
                ((uint32_t)aData[2] << CAN_TDL0R_DATA2_Pos) |
                ((uint32_t)aData[1] << CAN_TDL0R_DATA1_Pos) |
                ((uint32_t)aData[0] << CAN_TDL0R_DATA0_Pos));

      /* Request transmission */
      SET_BIT(hcan->Instance->sTxMailBox[transmitmailbox].TIR, CAN_TI0R_TXRQ);

      /* Return function status */
      return HAL_OK;
    }
    else
    {
      /* Update error code */
      hcan->ErrorCode |= HAL_CAN_ERROR_PARAM;

      return HAL_ERROR;
    }
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

    return HAL_ERROR;
  }
}

HAL_StatusTypeDef CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
  HAL_CAN_StateTypeDef state = hcan->State;

  assert_param(IS_CAN_RX_FIFO(RxFifo));

  if ((state == HAL_CAN_STATE_READY) ||
      (state == HAL_CAN_STATE_LISTENING))
  {
    /* Check the Rx FIFO */
    if (RxFifo == CAN_RX_FIFO0) /* Rx element is assigned to Rx FIFO 0 */
    {
      /* Check that the Rx FIFO 0 is not empty */
      if ((hcan->Instance->RF0R & CAN_RF0R_FMP0) == 0U)
      {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_PARAM;

        return HAL_ERROR;
      }
    }
    else /* Rx element is assigned to Rx FIFO 1 */
    {
      /* Check that the Rx FIFO 1 is not empty */
      if ((hcan->Instance->RF1R & CAN_RF1R_FMP1) == 0U)
      {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_PARAM;

        return HAL_ERROR;
      }
    }

    /* Get the header */
    pHeader->IDE = ((uint8_t)0x04 & hcan->Instance->sFIFOMailBox[RxFifo].RIR) >> 2;
    if (pHeader->IDE == CAN_ID_STD)
    {
      pHeader->StdId = (uint32_t)0x000007FF & (hcan->Instance->sFIFOMailBox[RxFifo].RIR >> 21);
    }
    else
    {
      pHeader->ExtId = (uint32_t)0x1FFFFFFF & (hcan->Instance->sFIFOMailBox[RxFifo].RIR >> 3);
    }
    pHeader->RTR = ((uint8_t)0x02 & hcan->Instance->sFIFOMailBox[RxFifo].RIR) >> 1;
    pHeader->DLC = (uint8_t)0x0F & hcan->Instance->sFIFOMailBox[RxFifo].RDTR;
    //pHeader->FilterMatchIndex = (CAN_RDT0R_FMI & hcan->Instance->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_FMI_Pos;
    //pHeader->Timestamp = (CAN_RDT0R_TIME & hcan->Instance->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_TIME_Pos;

    /* Get the data */
    aData[0] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[RxFifo].RDLR >> CAN_RDL0R_DATA0_Pos);
    aData[1] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[RxFifo].RDLR >> CAN_RDL0R_DATA1_Pos);
    aData[2] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[RxFifo].RDLR >> CAN_RDL0R_DATA2_Pos);
    aData[3] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[RxFifo].RDLR >> CAN_RDL0R_DATA3_Pos);
    aData[4] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[RxFifo].RDHR >> CAN_RDH0R_DATA4_Pos);
    aData[5] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[RxFifo].RDHR >> CAN_RDH0R_DATA5_Pos);
    aData[6] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[RxFifo].RDHR >> CAN_RDH0R_DATA6_Pos);
    aData[7] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[RxFifo].RDHR >> CAN_RDH0R_DATA7_Pos);

    /* Release the FIFO */
    if (RxFifo == CAN_RX_FIFO0) /* Rx element is assigned to Rx FIFO 0 */
    {
      /* Release RX FIFO 0 */
      SET_BIT(hcan->Instance->RF0R, CAN_RF0R_RFOM0);
    }
    else /* Rx element is assigned to Rx FIFO 1 */
    {
      /* Release RX FIFO 1 */
      SET_BIT(hcan->Instance->RF1R, CAN_RF1R_RFOM1);
    }

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

    return HAL_ERROR;
  }
}
