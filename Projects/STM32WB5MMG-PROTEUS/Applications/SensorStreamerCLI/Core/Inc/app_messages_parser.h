/**
 ******************************************************************************
 * @file    app_messages_parser.h
 * @author  SRA - GPM
 * 
 * 
 *
 * @brief
 *
 * 
 *
 *********************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file in
 * the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *********************************************************************************
 */
#ifndef APP_REPORT_PARSER_H_
#define APP_REPORT_PARSER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "services/syserror.h"

#define APP_MESSAGE_ID_UTIL                             0x0B  ///< Message ID used for the messages class of Utility task.
#define APP_MESSAGE_ID_CTRL                             0x12  ///< Message ID used for the messages class of AppController task.
#define APP_REPORT_ID_FORCE_STEP                        0xFE  ///< Special ID used by the INIT task to force the execution of ManagedTaskEx step.


typedef union _APPReport{
  uint8_t msgId;

  //--------------------------------------------------------------------------------
  //  internalReport 11 (MCU) - Util task command
  //--------------------------------------------------------------------------------

  struct utilMessage_t
  {
    uint8_t   msgId;                                /* Message ID = 0x0B (11) */
    uint8_t   sparam;                               /* optional small parameter */
    uint16_t  cmd_id;                               /* command ID */
    uint32_t  param;                                /* optional parameter. */
  } utilMessage;


  //--------------------------------------------------------------------------------
  //  CtrlMessage 0x12
  //--------------------------------------------------------------------------------

  struct CtrlMessage_t
  {
    uint8_t  msgId;                                 /* Message ID = 0x12 (18) */
    uint8_t  sparam;                                /* small parameter */
    uint16_t cmd_id;                                /* AppController task command ID */
    uint32_t param;                                 /* command parameter */
    uint8_t data[64];                               /* CLI data buff. Used only with the CMD_ID CTRL_CMD_NEW_CHAR*/
  } ctrlMessage;


  //--------------------------------------------------------------------------------
  //  internalReport (MCU)
  //--------------------------------------------------------------------------------

  struct internalReportFE_t
  {
    uint8_t  reportId;                               /* Report ID = 0xFE */
    uint8_t  data;                                   /* reserved. It can be ignored */
  } internalReportFE;



}APPReport;


/**
 * Get the size of the report with a specified ID
 *
 * @param message_id [IN] specifies a the id of the message
 * @return the size of the message with the specified ID or 0 if the message ID is not valid.
 */
uint16_t AppMsgGetSize(uint8_t message_id);


#ifdef __cplusplus
}
#endif


#endif /* APP_REPORT_PARSER_H_ */
