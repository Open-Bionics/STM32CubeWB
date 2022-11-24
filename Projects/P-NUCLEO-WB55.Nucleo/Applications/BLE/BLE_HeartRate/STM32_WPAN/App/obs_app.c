/**
  ******************************************************************************
  * @file    obs_app.c
  * @author  Olly McBride
  * @brief   Open Bionics development service
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_common.h"
#include "ble.h"
#include "obs_app.h"

typedef struct
{
  uint16_t  OBSvcHdle;       /**< Service handle */
  uint16_t  OBCharHdle;     /**< Characteristic handle */
}OBS_Context_t;

typedef struct
{
  uint8_t     *pPayload;
  uint8_t     Length;
}OBS_Data_t;

// SERVICE
const uint8_t OBS_SVC_UUID[16] = { 0x0B, 0x0B, 0x03, 0x00,
                                   0xFE, 0xED, 0xDE, 0xAD,
                                   0xBE, 0xE5, 0x0B, 0xE9,
                                   0xB1, 0x09, 0x1C, 0x50 };

// CHARACTERISTIC
const uint8_t OBC_SVC_UUID[16] = { 0x0B, 0x0B, 0x03, 0x01,
                                   0xFE, 0xED, 0xDE, 0xAD,
                                   0xBE, 0xE5, 0x0B, 0xE9,
                                   0xB1, 0x09, 0x1C, 0x50 };

/**
 * START of Section BLE_DRIVER_CONTEXT
 */

PLACE_IN_SECTION("BLE_DRIVER_CONTEXT") static OBS_Context_t OBS_Context;

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void OBS_Init(void)
{
  tBleStatus hciCmdResult;

  memset ( &OBS_Context, 0, sizeof(OBS_Context_t) );

  /**
   *  Register the event handler to the BLE controller
   *
   *  There is no need of an interrupt handler for this service
   */

  /**
   *  Add OB Test/Development Service
   */
  const uint8_t Max_Attribute_Records = 2;  // Char + Descriptor?
  hciCmdResult = aci_gatt_add_service(UUID_TYPE_128,
                                      (Service_UUID_t *) &OBS_SVC_UUID,
                                      PRIMARY_SERVICE,
                                      Max_Attribute_Records,
                                      &(OBS_Context.OBSvcHdle));

  if (hciCmdResult == BLE_STATUS_SUCCESS)
  {
    BLE_DBG_OBS_MSG ("OB Service (OBS) is added Successfully %04X\n", 
                 OBS_Context.OBSvcHdle);
  }
  else
  {
    BLE_DBG_OBS_MSG ("FAILED to add OBS Service (OBS), Error: %02X !!\n", 
                 hciCmdResult);
  }

  /**
   *  Add OBC Test String Characteristic
   */
  const char* obc_test_str = "Hello World!";
  hciCmdResult = aci_gatt_add_char(OBS_Context.OBSvcHdle,
                                   UUID_TYPE_128,
                                   (Char_UUID_t *) &OBC_SVC_UUID ,
                                   strlen(obc_test_str),
                                   CHAR_PROP_READ,
                                   ATTR_PERMISSION_NONE,
                                   GATT_DONT_NOTIFY_EVENTS, /* gattEvtMask */
                                   10, /* encryKeySize */
                                   CHAR_VALUE_LEN_VARIABLE, /* isVariable */
                                   &(OBS_Context.OBCharHdle));

  if (hciCmdResult == BLE_STATUS_SUCCESS)
  {
    BLE_DBG_OBS_MSG ("OB Characteristic Added Successfully  %04X \n", 
                 OBS_Context.OBCharHdle);
  }
  else
  {
    BLE_DBG_OBS_MSG ("FAILED to add OB Characteristic, Error: %02X !!\n", 
                hciCmdResult);
  }
      
  return;
}

/**
 * @brief  Characteristic update
 * @param  UUID: UUID of the characteristic
 * @retval None
 */
tBleStatus OBS_UpdateChar(OBS_Data_t *pPData)
{
  tBleStatus return_value;

  // switch(UUID)
  // {
  //   case MANUFACTURER_NAME_UUID:
      return_value = aci_gatt_update_char_value(OBS_Context.OBSvcHdle,
                                                OBS_Context.OBCharHdle,
                                                0,
                                                pPData->Length,
                                                (uint8_t *)pPData->pPayload);
  //     break;

  //   default:
  //     return_value = 0;
  //     break;
  // }

  return return_value;
}

#define OBC_VAL "Goodbye Moon!"

/* Functions Definition ------------------------------------------------------*/
void OBSAPP_Init(void)
{
  OBS_Data_t obs_data;

  /**
   * Update OBC Information
   *
   * @param UUID
   * @param pPData
   * @return
   */
  obs_data.pPayload = (uint8_t*)OBC_VAL;
  obs_data.Length = sizeof(OBC_VAL);
  OBS_UpdateChar(&obs_data);

}

