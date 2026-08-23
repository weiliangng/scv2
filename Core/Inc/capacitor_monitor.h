#ifndef CAPACITOR_MONITOR_H
#define CAPACITOR_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  float vcap_max_v;
  int32_t last_energy_gain_mj;
  int32_t last_vcap_gain_mv;
  uint8_t bad_window_count;
  uint8_t derate_count;
  bool unhealthy_latched;
} capacitor_monitor_status_t;

void CapacitorMonitor_Init(void);
void CapacitorMonitor_AccumulateFromIsr(float v_cap, float i_out);
void CapacitorMonitor_Update1kHz(uint32_t now_ms, float v_cap);
float CapacitorMonitor_GetVcapMaxV(void);
void CapacitorMonitor_ReadStatus(capacitor_monitor_status_t *status);

#endif /* CAPACITOR_MONITOR_H */
