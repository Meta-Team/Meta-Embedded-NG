# bsp usb

目前仅支持MC02板STM32H723 HS USB虚拟串口收发。

注意，为了增加发送完成和接收完成回调，对usbd_cdc_if.c usbd_cdc_if.h usbd_conf.c做了修改。

注意，MX_USB_DEVICE_Init();默认在freertos.c StartDefaultTask中调用，如果不启动FreeRTOS调试，需要手动在初始化过程中调用。

## 使用说明
MX_USB_DEVICE_Init();
USBInit(USB_Init_Config_s usb_conf); // 可以绑定回调函数，也可以置为NULL
// USBInit 返回指向接收缓冲区的指针
// 回调函数被调用时传入了实际接收到的字节数 *Len
USBTransmit(uint8_t *buffer, uint16_t len); // 发送

