/**
 ******************************************************************************
 * @file    usbd_midi.c
 * @author  Illia Pikin, MCD Application Team
 * @brief   This file provides the MIDI core functions.
 *
 * @verbatim
 *
 *          ===================================================================
 *                                MIDI Class  Description
 *          ===================================================================
 *           This module manages the MIDI class V1.0 following the "Universal Serial Bus
 *           Device Class Definition for MIDI Devices. Release 1.0 Nov 1, 1999".
 *
 * @note     In HS mode and when the DMA is used, all variables and data structures
 *           dealing with the DMA during the transaction process should be 32-bit aligned.
 *
 *
 *  @endverbatim
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2025 Illia Pikin</center></h2>
 * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Licensed under BSD 2-Clause License
 *
 * Copyright (c) 2025, Illia Pikin a.k.a Hypnotriod
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "usbd_midi.h"
#include "usbd_ctlreq.h"
#include "usbd_desc.h"
#include "usbd_webusb.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
 * @{
 */

/** @defgroup USBD_MIDI
 * @brief usbd core module
 * @{
 */

/** @defgroup USBD_MIDI_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_MIDI_Private_Defines
 * @{
 */

/**
 * @}
 */

/** @defgroup USBD_MIDI_Private_Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_MIDI_Private_FunctionPrototypes
 * @{
 */

static uint8_t USBD_MIDI_Init(USBD_HandleTypeDef* pdev, uint8_t cfgidx);
static uint8_t USBD_MIDI_DeInit(USBD_HandleTypeDef* pdev, uint8_t cfgidx);
static uint8_t USBD_MIDI_Setup(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);
static uint8_t* USBD_MIDI_GetCfgDesc(uint16_t* length);
static uint8_t* USBD_MIDI_GetDeviceQualifierDesc(uint16_t* length);
static uint8_t USBD_MIDI_DataIn(USBD_HandleTypeDef* pdev, uint8_t epnum);
static uint8_t USBD_MIDI_DataOut(USBD_HandleTypeDef* pdev, uint8_t epnum);
/**
 * @}
 */

/** @defgroup USBD_MIDI_Private_Variables
 * @{
 */

static uint8_t usb_rx_buffer[MIDI_EPOUT_SIZE] = {0};

/* What arrives on the vendor interface's OUT endpoint. Handed to
   USBD_BMCV_VendorDataOut, which Core overrides. */
static uint8_t vendor_rx_buffer[BMCV_VENDOR_EP_SIZE] = {0};

/* USB MIDI class type definition */
USBD_ClassTypeDef USBD_MIDI = {
    USBD_MIDI_Init,
    USBD_MIDI_DeInit,
    USBD_MIDI_Setup,
    NULL,              /*EP0_TxSent*/
    NULL,              /*EP0_RxReady*/
    USBD_MIDI_DataIn,  /*DataIn*/
    USBD_MIDI_DataOut, /*DataOut*/
    NULL,              /*SOF */
    NULL,
    NULL,
    USBD_MIDI_GetCfgDesc,
    USBD_MIDI_GetCfgDesc,
    USBD_MIDI_GetCfgDesc,
    USBD_MIDI_GetDeviceQualifierDesc,
};

/* USB MIDI device Configuration Descriptor */
/* Unsized on purpose, with the length asserted below: a fixed size would let an
   initialiser that is a byte short be zero-padded to fit, which is a descriptor
   the host reads to the end and finds garbage in. Written out, the compiler
   counts it and the assert checks the count. */
__ALIGN_BEGIN static uint8_t USBD_MIDI_CfgDesc[] __ALIGN_END = {
    0x09,                                                                       /* bLength: Configuration Descriptor size */
    USB_DESC_TYPE_CONFIGURATION,                                                /* bDescriptorType: Configuration */
    LOBYTE(USB_BMCV_CONFIG_DESC_SIZE), HIBYTE(USB_BMCV_CONFIG_DESC_SIZE), 0x02, /*bNumInterfaces: MIDI, and the vendor interface*/
    0x01,                                                                       /*bConfigurationValue: ID of this configuration. */
    0x00, /*iConfiguration: Index of string descriptor describing the configuration (Unused.)*/
    0x80, /*bmAttributes: Bus Powered device, not Self Powered, no Remote wakeup capability. */
    0xFA, /*MaxPower 500 mA: this current is used for detecting Vbus*/

    /************** MIDI Adapter Standard MS Interface Descriptor ****************/
    0x09,                    /*bLength: Interface Descriptor size*/
    USB_DESC_TYPE_INTERFACE, /*bDescriptorType: Interface descriptor type*/
    0x00,                    /*bInterfaceNumber: Index of this interface.*/
    0x00,                    /*bAlternateSetting: Alternate setting*/
    0x02,                    /*bNumEndpoints*/
    0x01,                    /*bInterfaceClass: AUDIO*/
    0x03,                    /*bInterfaceSubClass : MIDISTREAMING*/
    0x00,                    /*nInterfaceProtocol : Unused*/
    0x00,                    /*iInterface: Unused*/

    /******************** MIDI Adapter Class-specific MS Interface Descriptor ********************/
    /* USB_MIDI_CLASS_DESC_SHIFT */
    0x07,                            /*bLength: Descriptor size*/
    0x24,                            /*bDescriptorType: CS_INTERFACE descriptor*/
    0x01,                            /*bDescriptorSubtype: MS_HEADER subtype*/
    0x00, 0x01,                      /*BcdADC: Revision of this class specification*/
    USB_MIDI_REPORT_DESC_SIZE, 0x00, /*wTotalLength: Total size of class-specific descriptors*/

#if MIDI_IN_PORTS_NUM >= 1
    /******************** MIDI Adapter MIDI IN Jack Descriptor (External) ********************/
    0x06,        /*bLength: Size of this descriptor, in bytes*/
    0x24,        /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x02,        /*bDescriptorSubtype: MIDI_IN_JACK subtype*/
    0x02,        /*bJackType: EXTERNAL.*/
    MIDI_JACK_1, /*bJackID: ID of this Jack.*/
    0x00,        /*iJack: Unused.*/

    /******************** MIDI Adapter MIDI OUT Jack Descriptor (Embedded) ********************/
    0x09,        /*bLength: Size of this descriptor, in bytes*/
    0x24,        /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x03,        /*bDescriptorSubtype: MIDI_OUT_JACK subtype*/
    0x01,        /*bJackType: EMBEDDED*/
    MIDI_JACK_2, /*bJackID: ID of this Jack.*/
    0x01,        /*bNrInputPins: Number of Input Pins of this Jack.*/
    MIDI_JACK_1, /*BaSourceID(1): ID of the Entity to which this Pin is connected.*/
    0x01,        /*BaSourcePin(1): Output Pin number of the Entity to which this Input Pin is connected.*/
    0x00,        /*iJack: Unused.*/
#endif

#if MIDI_IN_PORTS_NUM >= 2
    /******************** MIDI Adapter MIDI IN Jack Descriptor (External) ********************/
    0x06,        /*bLength: Size of this descriptor, in bytes*/
    0x24,        /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x02,        /*bDescriptorSubtype: MIDI_IN_JACK subtype*/
    0x02,        /*bJackType: EXTERNAL.*/
    MIDI_JACK_3, /*bJackID: ID of this Jack.*/
    0x00,        /*iJack: Unused.*/

    /******************** MIDI Adapter MIDI OUT Jack Descriptor (Embedded) ********************/
    0x09,        /*bLength: Size of this descriptor, in bytes*/
    0x24,        /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x03,        /*bDescriptorSubtype: MIDI_OUT_JACK subtype*/
    0x01,        /*bJackType: EMBEDDED*/
    MIDI_JACK_4, /*bJackID: ID of this Jack.*/
    0x01,        /*bNrInputPins: Number of Input Pins of this Jack.*/
    MIDI_JACK_3, /*BaSourceID(1): ID of the Entity to which this Pin is connected.*/
    0x01,        /*BaSourcePin(1): Output Pin number of the Entity to which this Input Pin is connected.*/
    0x00,        /*iJack: Unused.*/
#endif

#if MIDI_IN_PORTS_NUM >= 3
    /******************** MIDI Adapter MIDI IN Jack Descriptor (External) ********************/
    0x06,        /*bLength: Size of this descriptor, in bytes*/
    0x24,        /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x02,        /*bDescriptorSubtype: MIDI_IN_JACK subtype*/
    0x02,        /*bJackType: EXTERNAL.*/
    MIDI_JACK_5, /*bJackID: ID of this Jack.*/
    0x00,        /*iJack: Unused.*/

    /******************** MIDI Adapter MIDI OUT Jack Descriptor (Embedded) ********************/
    0x09,        /*bLength: Size of this descriptor, in bytes*/
    0x24,        /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x03,        /*bDescriptorSubtype: MIDI_OUT_JACK subtype*/
    0x01,        /*bJackType: EMBEDDED*/
    MIDI_JACK_6, /*bJackID: ID of this Jack.*/
    0x01,        /*bNrInputPins: Number of Input Pins of this Jack.*/
    MIDI_JACK_5, /*BaSourceID(1): ID of the Entity to which this Pin is connected.*/
    0x01,        /*BaSourcePin(1): Output Pin number of the Entity to which this Input Pin is connected.*/
    0x00,        /*iJack: Unused.*/
#endif

#if MIDI_IN_PORTS_NUM >= 4
    /******************** MIDI Adapter MIDI IN Jack Descriptor (External) ********************/
    0x06,        /*bLength: Size of this descriptor, in bytes*/
    0x24,        /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x02,        /*bDescriptorSubtype: MIDI_IN_JACK subtype*/
    0x02,        /*bJackType: EXTERNAL.*/
    MIDI_JACK_7, /*bJackID: ID of this Jack.*/
    0x00,        /*iJack: Unused.*/

    /******************** MIDI Adapter MIDI OUT Jack Descriptor (Embedded) ********************/
    0x09,        /*bLength: Size of this descriptor, in bytes*/
    0x24,        /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x03,        /*bDescriptorSubtype: MIDI_OUT_JACK subtype*/
    0x01,        /*bJackType: EMBEDDED*/
    MIDI_JACK_8, /*bJackID: ID of this Jack.*/
    0x01,        /*bNrInputPins: Number of Input Pins of this Jack.*/
    MIDI_JACK_7, /*BaSourceID(1): ID of the Entity to which this Pin is connected.*/
    0x01,        /*BaSourcePin(1): Output Pin number of the Entity to which this Input Pin is connected.*/
    0x00,        /*iJack: Unused.*/
#endif

#if MIDI_IN_PORTS_NUM >= 5
    /******************** MIDI Adapter MIDI IN Jack Descriptor (External) ********************/
    0x06,        /*bLength: Size of this descriptor, in bytes*/
    0x24,        /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x02,        /*bDescriptorSubtype: MIDI_IN_JACK subtype*/
    0x02,        /*bJackType: EXTERNAL.*/
    MIDI_JACK_9, /*bJackID: ID of this Jack.*/
    0x00,        /*iJack: Unused.*/

    /******************** MIDI Adapter MIDI OUT Jack Descriptor (Embedded) ********************/
    0x09,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x03,         /*bDescriptorSubtype: MIDI_OUT_JACK subtype*/
    0x01,         /*bJackType: EMBEDDED*/
    MIDI_JACK_10, /*bJackID: ID of this Jack.*/
    0x01,         /*bNrInputPins: Number of Input Pins of this Jack.*/
    MIDI_JACK_9,  /*BaSourceID(1): ID of the Entity to which this Pin is connected.*/
    0x01,         /*BaSourcePin(1): Output Pin number of the Entity to which this Input Pin is connected.*/
    0x00,         /*iJack: Unused.*/
#endif

#if MIDI_IN_PORTS_NUM >= 6
    /******************** MIDI Adapter MIDI IN Jack Descriptor (External) ********************/
    0x06,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x02,         /*bDescriptorSubtype: MIDI_IN_JACK subtype*/
    0x02,         /*bJackType: EXTERNAL.*/
    MIDI_JACK_11, /*bJackID: ID of this Jack.*/
    0x00,         /*iJack: Unused.*/

    /******************** MIDI Adapter MIDI OUT Jack Descriptor (Embedded) ********************/
    0x09,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x03,         /*bDescriptorSubtype: MIDI_OUT_JACK subtype*/
    0x01,         /*bJackType: EMBEDDED*/
    MIDI_JACK_12, /*bJackID: ID of this Jack.*/
    0x01,         /*bNrInputPins: Number of Input Pins of this Jack.*/
    MIDI_JACK_11, /*BaSourceID(1): ID of the Entity to which this Pin is connected.*/
    0x01,         /*BaSourcePin(1): Output Pin number of the Entity to which this Input Pin is connected.*/
    0x00,         /*iJack: Unused.*/
#endif

#if MIDI_IN_PORTS_NUM >= 7
    /******************** MIDI Adapter MIDI IN Jack Descriptor (External) ********************/
    0x06,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x02,         /*bDescriptorSubtype: MIDI_IN_JACK subtype*/
    0x02,         /*bJackType: EXTERNAL.*/
    MIDI_JACK_13, /*bJackID: ID of this Jack.*/
    0x00,         /*iJack: Unused.*/

    /******************** MIDI Adapter MIDI OUT Jack Descriptor (Embedded) ********************/
    0x09,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x03,         /*bDescriptorSubtype: MIDI_OUT_JACK subtype*/
    0x01,         /*bJackType: EMBEDDED*/
    MIDI_JACK_14, /*bJackID: ID of this Jack.*/
    0x01,         /*bNrInputPins: Number of Input Pins of this Jack.*/
    MIDI_JACK_13, /*BaSourceID(1): ID of the Entity to which this Pin is connected.*/
    0x01,         /*BaSourcePin(1): Output Pin number of the Entity to which this Input Pin is connected.*/
    0x00,         /*iJack: Unused.*/
#endif

#if MIDI_IN_PORTS_NUM >= 8
    /******************** MIDI Adapter MIDI IN Jack Descriptor (External) ********************/
    0x06,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x02,         /*bDescriptorSubtype: MIDI_IN_JACK subtype*/
    0x02,         /*bJackType: EXTERNAL.*/
    MIDI_JACK_15, /*bJackID: ID of this Jack.*/
    0x00,         /*iJack: Unused.*/

    /******************** MIDI Adapter MIDI OUT Jack Descriptor (Embedded) ********************/
    0x09,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x03,         /*bDescriptorSubtype: MIDI_OUT_JACK subtype*/
    0x01,         /*bJackType: EMBEDDED*/
    MIDI_JACK_16, /*bJackID: ID of this Jack.*/
    0x01,         /*bNrInputPins: Number of Input Pins of this Jack.*/
    MIDI_JACK_15, /*BaSourceID(1): ID of the Entity to which this Pin is connected.*/
    0x01,         /*BaSourcePin(1): Output Pin number of the Entity to which this Input Pin is connected.*/
    0x00,         /*iJack: Unused.*/
#endif

#if MIDI_OUT_PORTS_NUM >= 1
    /******************** MIDI Adapter MIDI IN Jack Descriptor (Embedded) ********************/
    0x06,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x02,         /*bDescriptorSubtype: MIDI_IN_JACK subtype*/
    0x01,         /*bJackType: EMBEDDED*/
    MIDI_JACK_17, /*bJackID: ID of this Jack.*/
    0x00,         /*iJack: Unused.*/

    /******************** MIDI Adapter MIDI OUT Jack Descriptor (External) ********************/
    0x09,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x03,         /*bDescriptorSubtype: MIDI_OUT_JACK subtype*/
    0x02,         /*bJackType: EXTERNAL.*/
    MIDI_JACK_18, /*bJackID: ID of this Jack.*/
    0x01,         /*bNrInputPins: Number of Input Pins of this Jack.*/
    MIDI_JACK_17, /*BaSourceID(1): ID of the Entity to which this Pin is connected.*/
    0x01,         /*BaSourcePin(1): Output Pin number of the Entity to which this Input Pin is connected.*/
    0x00,         /*iJack: Unused.*/
#endif

#if MIDI_OUT_PORTS_NUM >= 2
    /******************** MIDI Adapter MIDI IN Jack Descriptor (Embedded) ********************/
    0x06,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x02,         /*bDescriptorSubtype: MIDI_IN_JACK subtype*/
    0x01,         /*bJackType: EMBEDDED*/
    MIDI_JACK_19, /*bJackID: ID of this Jack.*/
    0x00,         /*iJack: Unused.*/

    /******************** MIDI Adapter MIDI OUT Jack Descriptor (External) ********************/
    0x09,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x03,         /*bDescriptorSubtype: MIDI_OUT_JACK subtype*/
    0x02,         /*bJackType: EXTERNAL.*/
    MIDI_JACK_20, /*bJackID: ID of this Jack.*/
    0x01,         /*bNrInputPins: Number of Input Pins of this Jack.*/
    MIDI_JACK_19, /*BaSourceID(1): ID of the Entity to which this Pin is connected.*/
    0x01,         /*BaSourcePin(1): Output Pin number of the Entity to which this Input Pin is connected.*/
    0x00,         /*iJack: Unused.*/
#endif

#if MIDI_OUT_PORTS_NUM >= 3
    /******************** MIDI Adapter MIDI IN Jack Descriptor (Embedded) ********************/
    0x06,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x02,         /*bDescriptorSubtype: MIDI_IN_JACK subtype*/
    0x01,         /*bJackType: EMBEDDED*/
    MIDI_JACK_21, /*bJackID: ID of this Jack.*/
    0x00,         /*iJack: Unused.*/

    /******************** MIDI Adapter MIDI OUT Jack Descriptor (External) ********************/
    0x09,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x03,         /*bDescriptorSubtype: MIDI_OUT_JACK subtype*/
    0x02,         /*bJackType: EXTERNAL.*/
    MIDI_JACK_22, /*bJackID: ID of this Jack.*/
    0x01,         /*bNrInputPins: Number of Input Pins of this Jack.*/
    MIDI_JACK_21, /*BaSourceID(1): ID of the Entity to which this Pin is connected.*/
    0x01,         /*BaSourcePin(1): Output Pin number of the Entity to which this Input Pin is connected.*/
    0x00,         /*iJack: Unused.*/
#endif

#if MIDI_OUT_PORTS_NUM >= 4
    /******************** MIDI Adapter MIDI IN Jack Descriptor (Embedded) ********************/
    0x06,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x02,         /*bDescriptorSubtype: MIDI_IN_JACK subtype*/
    0x01,         /*bJackType: EMBEDDED*/
    MIDI_JACK_23, /*bJackID: ID of this Jack.*/
    0x00,         /*iJack: Unused.*/

    /******************** MIDI Adapter MIDI OUT Jack Descriptor (External) ********************/
    0x09,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x03,         /*bDescriptorSubtype: MIDI_OUT_JACK subtype*/
    0x02,         /*bJackType: EXTERNAL.*/
    MIDI_JACK_24, /*bJackID: ID of this Jack.*/
    0x01,         /*bNrInputPins: Number of Input Pins of this Jack.*/
    MIDI_JACK_23, /*BaSourceID(1): ID of the Entity to which this Pin is connected.*/
    0x01,         /*BaSourcePin(1): Output Pin number of the Entity to which this Input Pin is connected.*/
    0x00,         /*iJack: Unused.*/
#endif

#if MIDI_OUT_PORTS_NUM >= 5
    /******************** MIDI Adapter MIDI IN Jack Descriptor (Embedded) ********************/
    0x06,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x02,         /*bDescriptorSubtype: MIDI_IN_JACK subtype*/
    0x01,         /*bJackType: EMBEDDED*/
    MIDI_JACK_25, /*bJackID: ID of this Jack.*/
    0x00,         /*iJack: Unused.*/

    /******************** MIDI Adapter MIDI OUT Jack Descriptor (External) ********************/
    0x09,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x03,         /*bDescriptorSubtype: MIDI_OUT_JACK subtype*/
    0x02,         /*bJackType: EXTERNAL.*/
    MIDI_JACK_26, /*bJackID: ID of this Jack.*/
    0x01,         /*bNrInputPins: Number of Input Pins of this Jack.*/
    MIDI_JACK_25, /*BaSourceID(1): ID of the Entity to which this Pin is connected.*/
    0x01,         /*BaSourcePin(1): Output Pin number of the Entity to which this Input Pin is connected.*/
    0x00,         /*iJack: Unused.*/
#endif

#if MIDI_OUT_PORTS_NUM >= 6
    /******************** MIDI Adapter MIDI IN Jack Descriptor (Embedded) ********************/
    0x06,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x02,         /*bDescriptorSubtype: MIDI_IN_JACK subtype*/
    0x01,         /*bJackType: EMBEDDED*/
    MIDI_JACK_27, /*bJackID: ID of this Jack.*/
    0x00,         /*iJack: Unused.*/

    /******************** MIDI Adapter MIDI OUT Jack Descriptor (External) ********************/
    0x09,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x03,         /*bDescriptorSubtype: MIDI_OUT_JACK subtype*/
    0x02,         /*bJackType: EXTERNAL.*/
    MIDI_JACK_28, /*bJackID: ID of this Jack.*/
    0x01,         /*bNrInputPins: Number of Input Pins of this Jack.*/
    MIDI_JACK_27, /*BaSourceID(1): ID of the Entity to which this Pin is connected.*/
    0x01,         /*BaSourcePin(1): Output Pin number of the Entity to which this Input Pin is connected.*/
    0x00,         /*iJack: Unused.*/
#endif

#if MIDI_OUT_PORTS_NUM >= 7
    /******************** MIDI Adapter MIDI IN Jack Descriptor (Embedded) ********************/
    0x06,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x02,         /*bDescriptorSubtype: MIDI_IN_JACK subtype*/
    0x01,         /*bJackType: EMBEDDED*/
    MIDI_JACK_29, /*bJackID: ID of this Jack.*/
    0x00,         /*iJack: Unused.*/

    /******************** MIDI Adapter MIDI OUT Jack Descriptor (External) ********************/
    0x09,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x03,         /*bDescriptorSubtype: MIDI_OUT_JACK subtype*/
    0x02,         /*bJackType: EXTERNAL.*/
    MIDI_JACK_30, /*bJackID: ID of this Jack.*/
    0x01,         /*bNrInputPins: Number of Input Pins of this Jack.*/
    MIDI_JACK_29, /*BaSourceID(1): ID of the Entity to which this Pin is connected.*/
    0x01,         /*BaSourcePin(1): Output Pin number of the Entity to which this Input Pin is connected.*/
    0x00,         /*iJack: Unused.*/
#endif

#if MIDI_OUT_PORTS_NUM >= 8
    /******************** MIDI Adapter MIDI IN Jack Descriptor (Embedded) ********************/
    0x06,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x02,         /*bDescriptorSubtype: MIDI_IN_JACK subtype*/
    0x01,         /*bJackType: EMBEDDED*/
    MIDI_JACK_31, /*bJackID: ID of this Jack.*/
    0x00,         /*iJack: Unused.*/

    /******************** MIDI Adapter MIDI OUT Jack Descriptor (External) ********************/
    0x09,         /*bLength: Size of this descriptor, in bytes*/
    0x24,         /*bDescriptorType: CS_INTERFACE descriptor.*/
    0x03,         /*bDescriptorSubtype: MIDI_OUT_JACK subtype*/
    0x02,         /*bJackType: EXTERNAL.*/
    MIDI_JACK_32, /*bJackID: ID of this Jack.*/
    0x01,         /*bNrInputPins: Number of Input Pins of this Jack.*/
    MIDI_JACK_31, /*BaSourceID(1): ID of the Entity to which this Pin is connected.*/
    0x01,         /*BaSourcePin(1): Output Pin number of the Entity to which this Input Pin is connected.*/
    0x00,         /*iJack: Unused.*/
#endif

    /******************** MIDI Adapter Standard Bulk OUT Endpoint Descriptor ********************/
    0x09,                   /*bLength: Size of this descriptor, in bytes*/
    USB_DESC_TYPE_ENDPOINT, /*bDescriptorType: ENDPOINT descriptor.*/
    MIDI_EPOUT_ADDR,        /*bEndpointAddress: OUT Endpoint 1.*/
    0x02,                   /*bmAttributes: Bulk, not shared.*/
    MIDI_EPOUT_SIZE, 0x00,  /*wMaxPacketSize*/
    0x00,                   /*bInterval: Ignored for Bulk. Set to zero.*/
    0x00,                   /*bRefresh: Unused.*/
    0x00,                   /*bSynchAddress: Unused.*/

    /******************** MIDI Adapter Class-specific Bulk OUT Endpoint Descriptor ********************/
    (4 + MIDI_OUT_PORTS_NUM), /*bLength: Size of this descriptor, in bytes*/
    0x25,                     /*bDescriptorType: CS_ENDPOINT descriptor*/
    0x01,                     /*bDescriptorSubtype: MS_GENERAL subtype.*/
    MIDI_OUT_PORTS_NUM,       /*bNumEmbMIDIJack: Number of embedded MIDI IN Jacks.*/
#if MIDI_OUT_PORTS_NUM >= 1
    MIDI_JACK_17, /*BaAssocJackID(1): ID of the Embedded MIDI IN Jack.*/
#endif
#if MIDI_OUT_PORTS_NUM >= 2
    MIDI_JACK_19, /*BaAssocJackID(2): ID of the Embedded MIDI IN Jack.*/
#endif
#if MIDI_OUT_PORTS_NUM >= 3
    MIDI_JACK_21, /*BaAssocJackID(3): ID of the Embedded MIDI IN Jack.*/
#endif
#if MIDI_OUT_PORTS_NUM >= 4
    MIDI_JACK_23, /*BaAssocJackID(4): ID of the Embedded MIDI IN Jack.*/
#endif
#if MIDI_OUT_PORTS_NUM >= 5
    MIDI_JACK_25, /*BaAssocJackID(5): ID of the Embedded MIDI IN Jack.*/
#endif
#if MIDI_OUT_PORTS_NUM >= 6
    MIDI_JACK_27, /*BaAssocJackID(6): ID of the Embedded MIDI IN Jack.*/
#endif
#if MIDI_OUT_PORTS_NUM >= 7
    MIDI_JACK_29, /*BaAssocJackID(7): ID of the Embedded MIDI IN Jack.*/
#endif
#if MIDI_OUT_PORTS_NUM >= 8
    MIDI_JACK_31, /*BaAssocJackID(8): ID of the Embedded MIDI IN Jack.*/
#endif

    /******************** MIDI Adapter Standard Bulk IN Endpoint Descriptor ********************/
    0x09,                   /*bLength: Size of this descriptor, in bytes*/
    USB_DESC_TYPE_ENDPOINT, /*bDescriptorType: ENDPOINT descriptor.*/
    MIDI_EPIN_ADDR,         /*bEndpointAddress: IN Endpoint 1.*/
    0x02,                   /*bmAttributes: Bulk, not shared.*/
    MIDI_EPIN_SIZE, 0x00,   /*wMaxPacketSize*/
    0x00,                   /*bInterval: Ignored for Bulk. Set to zero.*/
    0x00,                   /*bRefresh: Unused.*/
    0x00,                   /*bSynchAddress: Unused.*/

    /******************** MIDI Adapter Class-specific Bulk IN Endpoint Descriptor ********************/
    (4 + MIDI_IN_PORTS_NUM), /*bLength: Size of this descriptor, in bytes*/
    0x25,                    /*bDescriptorType: CS_ENDPOINT descriptor*/
    0x01,                    /*bDescriptorSubtype: MS_GENERAL subtype.*/
    MIDI_IN_PORTS_NUM,       /*bNumEmbMIDIJack: Number of embedded MIDI OUT Jacks.*/
#if MIDI_IN_PORTS_NUM >= 1
    MIDI_JACK_2, /*BaAssocJackID(1): ID of the Embedded MIDI OUT Jack.*/
#endif
#if MIDI_IN_PORTS_NUM >= 2
    MIDI_JACK_4, /*BaAssocJackID(2): ID of the Embedded MIDI OUT Jack.*/
#endif
#if MIDI_IN_PORTS_NUM >= 3
    MIDI_JACK_6, /*BaAssocJackID(3): ID of the Embedded MIDI OUT Jack.*/
#endif
#if MIDI_IN_PORTS_NUM >= 4
    MIDI_JACK_8, /*BaAssocJackID(4): ID of the Embedded MIDI OUT Jack.*/
#endif
#if MIDI_IN_PORTS_NUM >= 5
    MIDI_JACK_10, /*BaAssocJackID(5): ID of the Embedded MIDI OUT Jack.*/
#endif
#if MIDI_IN_PORTS_NUM >= 6
    MIDI_JACK_12, /*BaAssocJackID(6): ID of the Embedded MIDI OUT Jack.*/
#endif
#if MIDI_IN_PORTS_NUM >= 7
    MIDI_JACK_14, /*BaAssocJackID(7): ID of the Embedded MIDI OUT Jack.*/
#endif
#if MIDI_IN_PORTS_NUM >= 8
    MIDI_JACK_16, /*BaAssocJackID(8): ID of the Embedded MIDI OUT Jack.*/
#endif

    /******************** BMCV vendor interface, for a browser ********************/
    /* Claimed through WebUSB and bound to WinUSB automatically by the Microsoft
       OS 2.0 descriptors in usbd_webusb.c. A host that knows nothing about it
       sees a MIDI device exactly as before. */
    0x09,                    /*bLength: Interface Descriptor size*/
    USB_DESC_TYPE_INTERFACE, /*bDescriptorType: Interface descriptor type*/
    BMCV_WEBUSB_INTERFACE,   /*bInterfaceNumber*/
    0x00,                    /*bAlternateSetting*/
    0x02,                    /*bNumEndpoints*/
    0xFF,                    /*bInterfaceClass: Vendor specific*/
    0x00,                    /*bInterfaceSubClass*/
    0x00,                    /*bInterfaceProtocol*/
    0x00,                    /*iInterface: Unused*/

    /* Bulk IN: snapshots out of the module. */
    0x07,                      /*bLength*/
    USB_DESC_TYPE_ENDPOINT,    /*bDescriptorType*/
    BMCV_VENDOR_EPIN_ADDR,     /*bEndpointAddress*/
    0x02,                      /*bmAttributes: Bulk*/
    BMCV_VENDOR_EP_SIZE, 0x00, /*wMaxPacketSize*/
    0x00,                      /*bInterval: ignored for bulk*/

    /* Bulk OUT: requests and the input mailbox, into the module. */
    0x07,                      /*bLength*/
    USB_DESC_TYPE_ENDPOINT,    /*bDescriptorType*/
    BMCV_VENDOR_EPOUT_ADDR,    /*bEndpointAddress*/
    0x02,                      /*bmAttributes: Bulk*/
    BMCV_VENDOR_EP_SIZE, 0x00, /*wMaxPacketSize*/
    0x00,                      /*bInterval: ignored for bulk*/
};

/* The one thing a compiler can check here: that what was written out is the
   length the descriptor claims. Everything else about it is the host's
   opinion. */
_Static_assert(sizeof(USBD_MIDI_CfgDesc) == USB_BMCV_CONFIG_DESC_SIZE, "the configuration descriptor is not the length it advertises");

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_MIDI_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END = {
    USB_LEN_DEV_QUALIFIER_DESC, USB_DESC_TYPE_DEVICE_QUALIFIER, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x01, 0x00,
};

/**
 * @}
 */

/** @defgroup USBD_MIDI_Private_Functions
 * @{
 */

/**
 * @brief  USBD_MIDI_Init
 *         Initialize the MIDI interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_MIDI_Init(USBD_HandleTypeDef* pdev, uint8_t cfgidx)
{
  (void) cfgidx;
  uint8_t ret = 0;

  USBD_LL_OpenEP(pdev, MIDI_EPIN_ADDR, USBD_EP_TYPE_BULK, MIDI_EPIN_SIZE);

  USBD_LL_OpenEP(pdev, MIDI_EPOUT_ADDR, USBD_EP_TYPE_BULK, MIDI_EPOUT_SIZE);

  USBD_LL_PrepareReceive(pdev, MIDI_EPOUT_ADDR, usb_rx_buffer, MIDI_EPOUT_SIZE);

  /* The vendor interface's pair. Opened here rather than in a class of its own
     because this device registers one class with the stack, and splitting it
     into a composite framework would be a great deal of machinery for a second
     interface that shares nothing with the first. */
  USBD_LL_OpenEP(pdev, BMCV_VENDOR_EPIN_ADDR, USBD_EP_TYPE_BULK, BMCV_VENDOR_EP_SIZE);
  USBD_LL_OpenEP(pdev, BMCV_VENDOR_EPOUT_ADDR, USBD_EP_TYPE_BULK, BMCV_VENDOR_EP_SIZE);
  USBD_LL_PrepareReceive(pdev, BMCV_VENDOR_EPOUT_ADDR, vendor_rx_buffer, BMCV_VENDOR_EP_SIZE);

  /* The endpoints are new; whatever Core was holding for the last host is not.
     This runs on every SET_CONFIGURATION, which is every enumeration - and the
     module keeps running across those, so nothing else would clear it. */
  USBD_BMCV_VendorReset();

  pdev->pClassData = USBD_malloc(sizeof(USBD_MIDI_HandleTypeDef));

  if (pdev->pClassData == NULL)
  {
    ret = 1;
  }
  else
  {
    ((USBD_MIDI_HandleTypeDef*) pdev->pClassData)->state = MIDI_IDLE;
  }
  return ret;
}

/**
 * @brief  USBD_MIDI_Init
 *         DeInitialize the MIDI layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_MIDI_DeInit(USBD_HandleTypeDef* pdev, uint8_t cfgidx)
{
  (void) cfgidx;

  /* Close every endpoint Init opened. This closed MIDI_EPIN_SIZE - a packet
     size where an endpoint address belongs, so 0x40, which is no endpoint on
     this device - and left the other three open, which is how it survived
     unnoticed: nothing here is called on the path a cable being pulled takes. */
  USBD_LL_CloseEP(pdev, MIDI_EPIN_ADDR);
  USBD_LL_CloseEP(pdev, MIDI_EPOUT_ADDR);
  USBD_LL_CloseEP(pdev, BMCV_VENDOR_EPIN_ADDR);
  USBD_LL_CloseEP(pdev, BMCV_VENDOR_EPOUT_ADDR);

  /* And whatever Core was holding for the host that is going away. Init calls
     this too; a configuration taken down and put back up must not leave a
     snapshot owed to nobody. */
  USBD_BMCV_VendorReset();

  /* FRee allocated memory */
  if (pdev->pClassData != NULL)
  {
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return USBD_OK;
}

/**
 * @brief  USBD_MIDI_Setup
 *         Handle the MIDI specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t USBD_MIDI_Setup(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
  uint16_t len                   = 0;
  uint8_t* pbuf                  = NULL;
  USBD_MIDI_HandleTypeDef* hmidi = pdev->pClassData;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_VENDOR:
    /* Windows asking where to find the descriptor set that names WinUSB as this
       interface's driver. It only knows to ask because the BOS descriptor
       published the request code - see usbd_webusb.c. */
    if (req->bRequest == BMCV_MSOS20_VENDOR_CODE && req->wIndex == BMCV_MSOS20_DESCRIPTOR_INDEX)
    {
      uint16_t set_len = 0;
      uint8_t* set     = (uint8_t*) bmcv_msos20_descriptor(&set_len);
      USBD_CtlSendData(pdev, set, MIN(set_len, req->wLength));
      break;
    }
    if (req->bRequest == BMCV_REQ_VERSION)
    {
      uint16_t len = 0;
      uint8_t* v   = (uint8_t*) USBD_BMCV_Version(&len);
      USBD_CtlSendData(pdev, v, MIN(len, req->wLength));
      break;
    }
    USBD_CtlError(pdev, req);
    return USBD_FAIL;

  case USB_REQ_TYPE_CLASS:
    switch (req->bRequest)
    {
    case MIDI_REQ_SET_PROTOCOL:
      hmidi->Protocol = (uint8_t) (req->wValue);
      break;

    case MIDI_REQ_GET_PROTOCOL:
      USBD_CtlSendData(pdev, (uint8_t*) &hmidi->Protocol, 1);
      break;

    case MIDI_REQ_SET_IDLE:
      hmidi->IdleState = (uint8_t) (req->wValue >> 8);
      break;

    case MIDI_REQ_GET_IDLE:
      USBD_CtlSendData(pdev, (uint8_t*) &hmidi->IdleState, 1);
      break;

    default:
      USBD_CtlError(pdev, req);
      return USBD_FAIL;
    }
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR:
      if (req->wValue >> 8 == MIDI_DESCRIPTOR_TYPE)
      {
        pbuf = USBD_MIDI_CfgDesc + USB_MIDI_CLASS_DESC_SHIFT;
        len  = MIN(USB_MIDI_DESC_SIZE, req->wLength);
      }

      USBD_CtlSendData(pdev, pbuf, len);
      break;

    case USB_REQ_GET_INTERFACE:
      USBD_CtlSendData(pdev, (uint8_t*) &hmidi->AltSetting, 1);
      break;

    case USB_REQ_SET_INTERFACE:
      hmidi->AltSetting = (uint8_t) (req->wValue);
      break;

    /* A host clearing a halt on one of the vendor endpoints. The stack has
       already reset the endpoint and answered the request - see USBD_StdEPReq,
       which calls us afterwards - so there is nothing to send here.

       What it does is give Core the one signal it otherwise has none of: a new
       host is starting. A browser closing the device leaves whatever snapshot
       was in flight half-collected, and clearing the halt resets the endpoint
       out from under it - so the transfer never completes, its DataIn never
       arrives, and the flag saying "a send is outstanding" would stay set for
       the rest of the module's power-on life. Every session after the first
       would find a module that answers nothing. */
    case USB_REQ_CLEAR_FEATURE:
      if ((req->bmRequest & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_ENDPOINT && req->wValue == USB_FEATURE_EP_HALT
          && (LOBYTE(req->wIndex) & 0x7FU) == (BMCV_VENDOR_EPIN_ADDR & 0x7FU))
      {
        USBD_BMCV_VendorReset();
      }
      break;
    }
  }
  return USBD_OK;
}

/**
 * @brief  USBD_MIDI_GetDeviceState
 *         Get MIDI State
 * @param  pdev: device instance
 * @retval usb device state  (USBD_STATE_DEFAULT, USBD_STATE_ADDRESSED, USBD_STATE_CONFIGURED, USBD_STATE_SUSPENDED)
 */
uint8_t USBD_MIDI_GetDeviceState(USBD_HandleTypeDef* pdev) { return pdev->dev_state; }

/**
 * @brief  USBD_MIDI_GetState
 *         Get MIDI State
 * @param  pdev: device instance
 * @retval usb state  (MIDI_IDLE, MIDI_BUSY)
 */
uint8_t USBD_MIDI_GetState(USBD_HandleTypeDef* pdev) { return ((USBD_MIDI_HandleTypeDef*) pdev->pClassData)->state; }

/**
 * @brief  USBD_MIDI_SendReport
 *         Send MIDI Report
 * @param  pdev: device instance
 * @param  report: pointer to report
 * @param  len: size of report
 * @retval status
 */
uint8_t USBD_MIDI_SendReport(USBD_HandleTypeDef* pdev, uint8_t* report, uint16_t len)
{
  USBD_MIDI_HandleTypeDef* hmidi = pdev->pClassData;

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (hmidi->state == MIDI_IDLE)
    {
      hmidi->state = MIDI_BUSY;
      USBD_LL_Transmit(pdev, MIDI_EPIN_ADDR, report, len);
    }
  }
  return USBD_OK;
}

/**
 * @brief  USBD_MIDI_GetCfgDesc
 *         return configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t* USBD_MIDI_GetCfgDesc(uint16_t* length)
{
  *length = sizeof(USBD_MIDI_CfgDesc);
  return USBD_MIDI_CfgDesc;
}

/**
 * @brief  DeviceQualifierDescriptor
 *         return Device Qualifier descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
uint8_t* USBD_MIDI_DeviceQualifierDescriptor(uint16_t* length)
{
  *length = sizeof(USBD_MIDI_DeviceQualifierDesc);
  return USBD_MIDI_DeviceQualifierDesc;
}

/**
 * @brief  USBD_MIDI_DataIn
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_MIDI_DataIn(USBD_HandleTypeDef* pdev, uint8_t epnum)
{
  if (epnum == (BMCV_VENDOR_EPIN_ADDR & 0x0F))
  {
    USBD_BMCV_VendorDataIn();
    return USBD_OK;
  }

  (void) epnum;

  /* Ensure that the FIFO is empty before a new transfer, this condition could
  be caused by  a new transfer before the end of the previous transfer */
  ((USBD_MIDI_HandleTypeDef*) pdev->pClassData)->state = MIDI_IDLE;
  return USBD_OK;
}

/**
 * @brief  USBD_MIDI_DataOut
 *         handle data OUT Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_MIDI_DataOut(USBD_HandleTypeDef* pdev, uint8_t epnum)
{
  if (epnum == (BMCV_VENDOR_EPOUT_ADDR & 0x0F))
  {
    /* The real length this time, not the buffer size: a vendor message is
       whatever the host sent, and the MIDI path above gets away with passing
       its buffer size only because it zeroes what it does not overwrite. */
    USBD_BMCV_VendorDataOut(vendor_rx_buffer, (uint16_t) USBD_LL_GetRxDataSize(pdev, epnum));
    USBD_LL_PrepareReceive(pdev, BMCV_VENDOR_EPOUT_ADDR, vendor_rx_buffer, BMCV_VENDOR_EP_SIZE);
    return USBD_OK;
  }

  if (epnum != (MIDI_EPOUT_ADDR & 0x0F))
    return USBD_FAIL;

  USBD_MIDI_DataInHandler(usb_rx_buffer, MIDI_EPOUT_SIZE);

  memset(usb_rx_buffer, 0, MIDI_EPOUT_SIZE);

  USBD_LL_PrepareReceive(pdev, MIDI_EPOUT_ADDR, usb_rx_buffer, MIDI_EPOUT_SIZE);

  return USBD_OK;
}

/**
 * @brief  USBD_MIDI_DataInHandler
 * @param  usb_rx_buffer: midi messages buffer
 * @param  usb_rx_buffer_length: midi messages buffer length
 */
/**
 * @brief  One transfer received on the vendor interface. Overridden in Core.
 */
/**
 * @brief  The firmware version, as a NUL-terminated string. Overridden in Core,
 *         which is where the build's version number is.
 */
__weak const char* USBD_BMCV_Version(uint16_t* length)
{
  *length = 1;
  return "";
}

__weak void USBD_BMCV_VendorDataIn(void) {}

/**
 * @brief  A host is starting on the vendor interface. Overridden in Core.
 */
__weak void USBD_BMCV_VendorReset(void) {}

/**
 * @brief  One transfer received on the vendor interface. Overridden in Core.
 */
__weak void USBD_BMCV_VendorDataOut(uint8_t* data, uint16_t len)
{
  UNUSED(data);
  UNUSED(len);
}

/**
 * @brief  Send on the vendor IN endpoint. The stack splits anything longer than
 *         the endpoint's 64 bytes into packets itself, so a whole snapshot goes
 *         out in one call.
 */
uint8_t USBD_BMCV_VendorSend(USBD_HandleTypeDef* pdev, uint8_t* data, uint16_t len)
{
  if (pdev->dev_state != USBD_STATE_CONFIGURED)
    return USBD_FAIL;

  return USBD_LL_Transmit(pdev, BMCV_VENDOR_EPIN_ADDR, data, len);
}

__weak extern void USBD_MIDI_DataInHandler(uint8_t* usb_rx_buffer, uint8_t usb_rx_buffer_length)
{
  (void) usb_rx_buffer;
  (void) usb_rx_buffer_length;
  // For user implementation.
}

/**
 * @brief  DeviceQualifierDescriptor
 *         return Device Qualifier descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t* USBD_MIDI_GetDeviceQualifierDesc(uint16_t* length)
{
  *length = sizeof(USBD_MIDI_DeviceQualifierDesc);
  return USBD_MIDI_DeviceQualifierDesc;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
