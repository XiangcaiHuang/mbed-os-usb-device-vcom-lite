/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __USB_DEVICE_CH9_H__
#define __USB_DEVICE_CH9_H__

/*******************************************************************************
* Definitions
******************************************************************************/
/*!
 * @addtogroup usb_device_ch9
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Defines USB device status size when the host request to get device status */
#define USB_DEVICE_STATUS_SIZE (0x02U)

/*! @brief Defines USB device interface status size when the host request to get interface status */
#define USB_INTERFACE_STATUS_SIZE (0x02U)

/*! @brief Defines USB device endpoint status size when the host request to get endpoint status */
#define USB_ENDPOINT_STATUS_SIZE (0x02U)

/*! @brief Defines USB device configuration size when the host request to get current configuration */
#define USB_CONFIGURE_SIZE (0X01U)

/*! @brief Defines USB device interface alternate setting size when the host request to get interface alternate setting
 */
#define USB_INTERFACE_SIZE (0X01U)

/*! @brief Defines USB device status mask */
#define USB_GET_STATUS_DEVICE_MASK (0x03U)

/*! @brief Defines USB device interface status mask */
#define USB_GET_STATUS_INTERFACE_MASK (0x03U)

/*! @brief Defines USB device endpoint status mask */
#define USB_GET_STATUS_ENDPOINT_MASK (0x03U)

/*! @brief Control read and write sequence */
typedef enum _usb_device_control_read_write_sequence
{
    kUSB_DeviceControlPipeSetupStage = 0U, /*!< Setup stage */
    kUSB_DeviceControlPipeDataStage,       /*!< Data stage */
    kUSB_DeviceControlPipeStatusStage,     /*!< status stage */
} usb_device_control_read_write_sequence_t;

/*******************************************************************************
* API
******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Initialize the control pipes.
 *
 * The function is used to initialize the control pipes. This function should be called when event
 * kUSB_DeviceEventBusReset is received.
 *
 * @param handle      The device handle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
extern usb_status_t USB_DeviceControlPipeInit(usb_device_handle handle);

/*******************************************************************************
* Prototypes
******************************************************************************/
/*!
 * @brief Get the setup packet buffer.
 *
 * The function is used to get the setup packet buffer to save the setup packet data.
 *
 * @param handle              The device handle.
 * @param setupBuffer        It is an OUT parameter, return the setup buffer address to the caller.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
extern usb_status_t USB_DeviceGetSetupBuffer(usb_device_handle handle, usb_setup_struct_t **setupBuffer);
/*!
 * @brief Handle the class request.
 *
 * The function is used to handle the class request.
 *
 * @param handle              The device handle.
 * @param setup               The setup packet buffer address.
 * @param length              It is an OUT parameter, return the data length need to be sent to host.
 * @param buffer              It is an OUT parameter, return the data buffer address.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
extern usb_status_t USB_DeviceProcessClassRequest(usb_device_handle handle,
                                                  usb_setup_struct_t *setup,
                                                  uint32_t *length,
                                                  uint8_t **buffer);

/*!
 * @brief Get the buffer to save the class specific data sent from host.
 *
 * The function is used to get the buffer to save the class specific data sent from host.
 * The function will be called when the device receives a setup pakcet, and the host needs to send data to the device in
 * the data stage.
 *
 * @param handle              The device handle.
 * @param setup               The setup packet buffer address.
 * @param length              Pass the length the host needs to sent.
 * @param buffer              It is an OUT parameter, return the data buffer address to save the host's data.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
extern usb_status_t USB_DeviceGetClassReceiveBuffer(usb_device_handle handle,
                                                    usb_setup_struct_t *setup,
                                                    uint32_t *length,
                                                    uint8_t **buffer);

/* standard request */
/*!
 * @brief Get the descritpor.
 *
 * The function is used to get the descritpor, including the device descritpor, configuration descriptor, and string
 * descriptor, etc.
 *
 * @param handle              The device handle.
 * @param setup               The setup packet buffer address.
 * @param length              It is an OUT parameter, return the data length need to be sent to host.
 * @param buffer              It is an OUT parameter, return the data buffer address.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
extern usb_status_t USB_DeviceGetDescriptor(usb_device_handle handle,
                                            usb_setup_struct_t *setup,
                                            uint32_t *length,
                                            uint8_t **buffer);

/*!
 * @brief Set the device configuration.
 *
 * The function is used to set the device configuration.
 *
 * @param handle              The device handle.
 * @param configure           The configuration value.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
extern usb_status_t USB_DeviceSetConfigure(usb_device_handle handle, uint8_t configure);

/*!
 * @brief Get the device configuration.
 *
 * The function is used to get the device configuration.
 *
 * @param handle              The device handle.
 * @param configure           It is an OUT parameter, save the current configuration value.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
extern usb_status_t USB_DeviceGetConfigure(usb_device_handle handle, uint8_t *configure);

/*!
 * @brief Set an interface alternate setting.
 *
 * The function is used to set an interface alternate setting.
 *
 * @param handle              The device handle.
 * @param interface           The interface index.
 * @param alternateSetting   The new alternate setting value.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
extern usb_status_t USB_DeviceSetInterface(usb_device_handle handle, uint8_t interface, uint8_t alternateSetting);

/*!
 * @brief Get an interface alternate setting.
 *
 * The function is used to get an interface alternate setting.
 *
 * @param handle              The device handle.
 * @param interface           The interface index.
 * @param alternateSetting   It is an OUT parameter, save the new alternate setting value of the interface.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
extern usb_status_t USB_DeviceGetInterface(usb_device_handle handle, uint8_t interface, uint8_t *alternateSetting);

/*!
 * @brief Configure a specified endpoint status.
 *
 * The function is used to configure a specified endpoint status, idle or halt.
 *
 * @param handle              The device handle.
 * @param endpointAddress    The endpoint address, the BIT7 is the direction, 0 - USB_OUT, 1 - USB_IN.
 * @param status              The new status of the endpoint, 0 - idle, 1 - halt.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
extern usb_status_t USB_DeviceConfigureEndpointStatus(usb_device_handle handle,
                                                      uint8_t endpointAddress,
                                                      uint8_t status);

/*!
 * @brief Configure the device remote wakeup feature.
 *
 * The function is used to configure the device remote wakeup feature, enable or disbale the remote wakeup feature.
 *
 * @param handle              The device handle.
 * @param enable              The new feature value of the device remote wakeup, 0 - disable, 1 - enable.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
extern usb_status_t USB_DeviceConfigureRemoteWakeup(usb_device_handle handle, uint8_t enable);

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __USB_DEVICE_CH9_H__ */
