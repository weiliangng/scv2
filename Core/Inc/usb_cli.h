#ifndef USB_CLI_H
#define USB_CLI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void UsbCli_Init(void);
void UsbCli_OnRx(const uint8_t *buf, uint32_t len);
void UsbCli_Task(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* USB_CLI_H */

