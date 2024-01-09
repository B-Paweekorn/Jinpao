#ifndef _QEI_H__
#define _QEI_H__

#include <driver/pulse_cnt.h>

class QEI {
private:

  pcnt_unit_handle_t pcnt_unit;

  int encA_pin;
  int encB_pin;
  int pcnt_limit = 32767;
  uint32_t filter_ns = 0;

  int pulse_count = 0;
  int prev_pulse_count = 0;

public:

  QEI(int encA, int encB, int limit = 32767, uint32_t filt = 0);

  void begin();

  int get_diff_count();

  void reset();
};

#endif