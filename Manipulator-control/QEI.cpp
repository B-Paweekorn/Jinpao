#include "esp32-hal-gpio.h"
#include "driver/pulse_cnt.h"
#include "QEI.h"

QEI ::QEI(int encA, int encB, int limit, uint32_t filt)
  : encA_pin(encA), encB_pin(encB), pcnt_limit(limit), filter_ns(filt) {}

void QEI ::begin() {

  pcnt_unit_config_t unit_config;
  unit_config.high_limit = pcnt_limit;
  unit_config.low_limit = -pcnt_limit;
  unit_config.intr_priority = 0;
  pcnt_glitch_filter_config_t filter_config = {
    .max_glitch_ns = filter_ns,
  };

  pcnt_chan_config_t chan_a_config;
  chan_a_config.edge_gpio_num = encA_pin;
  chan_a_config.level_gpio_num = encB_pin;
  chan_a_config.flags.io_loop_back = false;
  pcnt_channel_handle_t pcnt_chan_a = NULL;

  pcnt_chan_config_t chan_b_config;
  chan_b_config.edge_gpio_num = encB_pin;
  chan_b_config.level_gpio_num = encA_pin;
  chan_b_config.flags.io_loop_back = false;
  pcnt_channel_handle_t pcnt_chan_b = NULL;

  pcnt_new_unit(&unit_config, &pcnt_unit);

  pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);

  pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a);
  pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b);

  pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  pcnt_unit_enable(pcnt_unit);
  pcnt_unit_clear_count(pcnt_unit);
  pcnt_unit_start(pcnt_unit);
}

int QEI ::get_diff_count() {
  pcnt_unit_get_count(pcnt_unit, &pulse_count);

  int delta_pulse_count = pulse_count - prev_pulse_count;
  prev_pulse_count = pulse_count;

  if (delta_pulse_count < -pcnt_limit / 2) {
    delta_pulse_count += pcnt_limit;
  } else if (delta_pulse_count > pcnt_limit / 2) {
    delta_pulse_count -= pcnt_limit;
  }

  return delta_pulse_count;
}

void QEI ::reset() {
  pcnt_unit_clear_count(pcnt_unit);
}