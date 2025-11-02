#ifndef __IO_GPIOD_PI__
#define __IO_GPIOD_PI__

#include "iobase.h"
#include <gpiod.h>

class IOGPIODPi : public IOBase
{
 public:
  IOGPIODPi(int tms, int tck, int tdi, int tdo);
  virtual ~IOGPIODPi();

 protected:
  void tx(bool tms, bool tdi);
  bool txrx(bool tms, bool tdi);

  void txrx_block(const unsigned char *tdi, unsigned char *tdo, int length, bool last) override;
  void tx_tms(unsigned char *pat, int length, int force) override;

  int TMSPin;
  int TCKPin;
  int TDIPin;
  int TDOPin;

 private:
  struct gpiod_chip *chip = nullptr;
  struct gpiod_line_request *req = nullptr;

  inline void set_line(int offset, bool value) {
    gpiod_line_request_set_value(req, (unsigned int)offset,
      value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
  }

  inline bool get_line(int offset) {
    return gpiod_line_request_get_value(req, (unsigned int)offset) == GPIOD_LINE_VALUE_ACTIVE;
  }
};

#endif
