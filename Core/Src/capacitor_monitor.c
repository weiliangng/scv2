#include "capacitor_monitor.h"

#include "main.h"

#include "app_constants.h"
#include "shared_state.h"

static float s_isr_block_energy_mj;
static uint32_t s_isr_block_samples;
static volatile float s_pending_energy_mj;

static float s_lifetime_remainder_mj;
static bool s_anchor_initialized;
static bool s_anchor_armed;
static bool s_window_initialized;
static uint32_t s_window_start_ms;
static int32_t s_window_start_energy_mj;
static int64_t s_window_start_vcap_sum_mv;
static uint32_t s_window_start_vcap_samples;
static int64_t s_window_end_vcap_sum_mv;
static uint32_t s_window_end_vcap_samples;
static capacitor_monitor_status_t s_status;

_Static_assert(SCAP_CAP_HEALTH_VOLTAGE_AVG_MS < (SCAP_CAP_HEALTH_WINDOW_MS / 2u),
               "Health voltage averaging intervals must not overlap");

static float take_pending_energy_mj(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const float pending_mj = s_pending_energy_mj;
  s_pending_energy_mj = 0.0f;
  if (primask == 0u)
  {
    __enable_irq();
  }
  return pending_mj;
}

static void reset_energy_pipeline(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_isr_block_energy_mj = 0.0f;
  s_isr_block_samples = 0u;
  s_pending_energy_mj = 0.0f;
  if (primask == 0u)
  {
    __enable_irq();
  }
}

static int32_t vcap_to_mv(float v_cap)
{
  return (int32_t)((v_cap * 1000.0f) + ((v_cap >= 0.0f) ? 0.5f : -0.5f));
}

static void begin_health_window(uint32_t now_ms, float v_cap)
{
  s_window_initialized = true;
  s_window_start_ms = now_ms;
  s_window_start_energy_mj = g_cap_energy_mj;
  s_window_start_vcap_sum_mv = vcap_to_mv(v_cap);
  s_window_start_vcap_samples = 1u;
  s_window_end_vcap_sum_mv = 0;
  s_window_end_vcap_samples = 0u;
}

static void reset_health_window(uint32_t now_ms, float v_cap)
{
  begin_health_window(now_ms, v_cap);
  s_status.bad_window_count = 0u;
}

void CapacitorMonitor_Init(void)
{
  reset_energy_pipeline();
  g_cap_energy_mj = 0;
  s_lifetime_remainder_mj = 0.0f;
  s_anchor_initialized = false;
  s_anchor_armed = false;
  s_window_initialized = false;
  s_window_start_ms = 0u;
  s_window_start_energy_mj = 0;
  s_window_start_vcap_sum_mv = 0;
  s_window_start_vcap_samples = 0u;
  s_window_end_vcap_sum_mv = 0;
  s_window_end_vcap_samples = 0u;
  s_status.vcap_max_v = SCAP_VCAP_MAX_V;
  s_status.last_energy_gain_mj = 0;
  s_status.last_vcap_gain_mv = 0;
  s_status.bad_window_count = 0u;
  s_status.derate_count = 0u;
  s_status.unhealthy_latched = false;
}

void CapacitorMonitor_AccumulateFromIsr(float v_cap, float i_out)
{
  s_isr_block_energy_mj += v_cap * i_out * SCAP_ADC_ISR_INTERVAL_S * 1000.0f;

  s_isr_block_samples++;
  if (s_isr_block_samples >= SCAP_ENERGY_ISR_BLOCK_SAMPLES)
  {
    s_pending_energy_mj += s_isr_block_energy_mj;
    s_isr_block_energy_mj = 0.0f;
    s_isr_block_samples = 0u;
  }
}

void CapacitorMonitor_Update1kHz(uint32_t now_ms, float v_cap)
{
  bool anchored = false;
  if (!s_anchor_initialized)
  {
    s_anchor_armed = v_cap < SCAP_ENERGY_ANCHOR_V;
    s_anchor_initialized = true;
  }
  else if (!s_anchor_armed && (v_cap <= SCAP_ENERGY_ANCHOR_REARM_V))
  {
    s_anchor_armed = true;
  }

  if (s_anchor_armed && (v_cap >= SCAP_ENERGY_ANCHOR_V))
  {
    reset_energy_pipeline();
    g_cap_energy_mj = (int32_t)(SCAP_ENERGY_ANCHOR_VALUE_MJ + 0.5f);
    s_lifetime_remainder_mj = 0.0f;
    s_anchor_armed = false;
    anchored = true;
    reset_health_window(now_ms, v_cap);
  }

  if (!anchored)
  {
    s_lifetime_remainder_mj += take_pending_energy_mj();
    const int32_t whole_mj = (int32_t)s_lifetime_remainder_mj;
    g_cap_energy_mj += whole_mj;
    s_lifetime_remainder_mj -= (float)whole_mj;
  }

  if (!s_window_initialized)
  {
    reset_health_window(now_ms, v_cap);
    return;
  }
  if (anchored)
  {
    return;
  }

  const uint32_t window_elapsed_ms = (uint32_t)(now_ms - s_window_start_ms);
  if (window_elapsed_ms < SCAP_CAP_HEALTH_WINDOW_MS)
  {
    const int32_t vcap_mv = vcap_to_mv(v_cap);
    if (window_elapsed_ms < SCAP_CAP_HEALTH_VOLTAGE_AVG_MS)
    {
      s_window_start_vcap_sum_mv += vcap_mv;
      s_window_start_vcap_samples++;
    }
    else if (window_elapsed_ms >=
             (SCAP_CAP_HEALTH_WINDOW_MS - SCAP_CAP_HEALTH_VOLTAGE_AVG_MS))
    {
      s_window_end_vcap_sum_mv += vcap_mv;
      s_window_end_vcap_samples++;
    }
    return;
  }

  if ((s_window_start_vcap_samples == 0u) || (s_window_end_vcap_samples == 0u))
  {
    s_status.bad_window_count = 0u;
    begin_health_window(now_ms, v_cap);
    return;
  }

  const int32_t start_vcap_avg_mv =
      (int32_t)(s_window_start_vcap_sum_mv / (int64_t)s_window_start_vcap_samples);
  const int32_t end_vcap_avg_mv =
      (int32_t)(s_window_end_vcap_sum_mv / (int64_t)s_window_end_vcap_samples);
  const int64_t endpoint_gain_mv = (int64_t)end_vcap_avg_mv - start_vcap_avg_mv;
  const int64_t endpoint_separation_ms =
      SCAP_CAP_HEALTH_WINDOW_MS - SCAP_CAP_HEALTH_VOLTAGE_AVG_MS;
  int64_t normalized_gain_numerator = endpoint_gain_mv * SCAP_CAP_HEALTH_WINDOW_MS;
  if (normalized_gain_numerator >= 0)
  {
    normalized_gain_numerator += endpoint_separation_ms / 2;
  }
  else
  {
    normalized_gain_numerator -= endpoint_separation_ms / 2;
  }

  const int32_t energy_gain_mj = g_cap_energy_mj - s_window_start_energy_mj;
  const int32_t vcap_gain_mv =
      (int32_t)(normalized_gain_numerator / endpoint_separation_ms);
  s_status.last_energy_gain_mj = energy_gain_mj;
  s_status.last_vcap_gain_mv = vcap_gain_mv;

  if ((energy_gain_mj >= SCAP_CAP_HEALTH_MIN_ENERGY_GAIN_MJ) &&
      (vcap_gain_mv < SCAP_CAP_HEALTH_MAX_VOLTAGE_GAIN_MV))
  {
    if (s_status.bad_window_count < SCAP_CAP_HEALTH_BAD_WINDOWS)
    {
      s_status.bad_window_count++;
    }
  }
  else
  {
    s_status.bad_window_count = 0u;
  }

  if (s_status.bad_window_count >= SCAP_CAP_HEALTH_BAD_WINDOWS)
  {
    s_status.unhealthy_latched = true;
    if (s_status.vcap_max_v > SCAP_VCAP_DERATE_MIN_V)
    {
      float next_max_v = s_status.vcap_max_v - SCAP_VCAP_DERATE_STEP_V;
      if (next_max_v <= (SCAP_VCAP_DERATE_MIN_V + 0.0001f))
      {
        next_max_v = SCAP_VCAP_DERATE_MIN_V;
      }
      if (next_max_v < s_status.vcap_max_v)
      {
        s_status.vcap_max_v = next_max_v;
        s_status.derate_count++;
      }
    }
    s_status.bad_window_count = 0u;
  }

  begin_health_window(now_ms, v_cap);
}

float CapacitorMonitor_GetVcapMaxV(void)
{
  return s_status.vcap_max_v;
}

void CapacitorMonitor_ReadStatus(capacitor_monitor_status_t *status)
{
  if (status == NULL)
  {
    return;
  }
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *status = s_status;
  if (primask == 0u)
  {
    __enable_irq();
  }
}
