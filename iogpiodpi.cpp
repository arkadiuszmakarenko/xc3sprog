#include "iogpiodpi.h"
#include <iostream>
#include <cstdio>

IOGPIODPi::IOGPIODPi(int tms, int tck, int tdi, int tdo)
  : TMSPin(tms), TCKPin(tck), TDIPin(tdi), TDOPin(tdo)
{
  // libgpiod v2: open chip via path
  chip = gpiod_chip_open("/dev/gpiochip0");
  if (!chip) {
    perror("gpiod_chip_open(/dev/gpiochip0) failed");
    throw std::runtime_error("Failed to open gpiochip0");
  }

  // Create line settings for output-lines (TMS, TCK, TDI)
  gpiod_line_settings *out_settings = gpiod_line_settings_new();
  if (!out_settings) {
    gpiod_chip_close(chip);
    throw std::runtime_error("gpiod_line_settings_new (out) failed");
  }
  gpiod_line_settings_set_direction(out_settings, GPIOD_LINE_DIRECTION_OUTPUT);
  gpiod_line_settings_set_output_value(out_settings, GPIOD_LINE_VALUE_INACTIVE);

  // Create line settings for input-lines (TDO)
  gpiod_line_settings *in_settings = gpiod_line_settings_new();
  if (!in_settings) {
    gpiod_line_settings_free(out_settings);
    gpiod_chip_close(chip);
    throw std::runtime_error("gpiod_line_settings_new (in) failed");
  }
  gpiod_line_settings_set_direction(in_settings, GPIOD_LINE_DIRECTION_INPUT);

  // Line config and offsets
  gpiod_line_config *lcfg = gpiod_line_config_new();
  if (!lcfg) {
    gpiod_line_settings_free(out_settings);
    gpiod_line_settings_free(in_settings);
    gpiod_chip_close(chip);
    throw std::runtime_error("gpiod_line_config_new failed");
  }

  // Add settings per offset
  unsigned int out_offsets[3] = { (unsigned int)TMSPin, (unsigned int)TCKPin, (unsigned int)TDIPin };
  for (size_t i = 0; i < 3; ++i) {
    if (gpiod_line_config_add_line_settings(lcfg, &out_offsets[i], 1, out_settings) < 0) {
      gpiod_line_config_free(lcfg);
      gpiod_line_settings_free(out_settings);
      gpiod_line_settings_free(in_settings);
      gpiod_chip_close(chip);
      throw std::runtime_error("gpiod_line_config_add_line_settings (out) failed");
    }
  }

  unsigned int in_offset = (unsigned int)TDOPin;
  if (gpiod_line_config_add_line_settings(lcfg, &in_offset, 1, in_settings) < 0) {
    gpiod_line_config_free(lcfg);
    gpiod_line_settings_free(out_settings);
    gpiod_line_settings_free(in_settings);
    gpiod_chip_close(chip);
    throw std::runtime_error("gpiod_line_config_add_line_settings (in) failed");
  }

  // Request config
  gpiod_request_config *rcfg = gpiod_request_config_new();
  if (!rcfg) {
    gpiod_line_config_free(lcfg);
    gpiod_line_settings_free(out_settings);
    gpiod_line_settings_free(in_settings);
    gpiod_chip_close(chip);
    throw std::runtime_error("gpiod_request_config_new failed");
  }
  gpiod_request_config_set_consumer(rcfg, "xc3sprog");

  // Create 1 line request for all lines
  req = gpiod_chip_request_lines(chip, rcfg, lcfg);
  if (!req) {
    perror("gpiod_chip_request_lines failed");
    gpiod_request_config_free(rcfg);
    gpiod_line_config_free(lcfg);
    gpiod_line_settings_free(out_settings);
    gpiod_line_settings_free(in_settings);
    gpiod_chip_close(chip);
    throw std::runtime_error("Failed to request lines");
  }

  // Clean helpers (keeps request)
  gpiod_request_config_free(rcfg);
  gpiod_line_config_free(lcfg);
  gpiod_line_settings_free(out_settings);
  gpiod_line_settings_free(in_settings);

  // Init
  set_line(TMSPin, false);
  set_line(TCKPin, false);
  set_line(TDIPin, false);
}

IOGPIODPi::~IOGPIODPi()
{
  if (req) {
    gpiod_line_request_release(req);
    req = nullptr;
  }
  if (chip) {
    gpiod_chip_close(chip);
    chip = nullptr;
  }
}

void IOGPIODPi::txrx_block(const unsigned char *tdi, unsigned char *tdo, int length, bool last)
{
  int i = 0;
  int j = 0;
  unsigned char tdo_byte = 0;
  unsigned char tdi_byte = 0;
  if (tdi)
    tdi_byte = tdi[j];

  while (i < length - 1) {
    tdo_byte = tdo_byte + (txrx(false, tdi ? ((tdi_byte & 1) == 1) : false) << (i % 8));
    if (tdi)
      tdi_byte = tdi_byte >> 1;
    i++;
    if ((i % 8) == 0) { // next byte
      if (tdo)
        tdo[j] = tdo_byte;
      tdo_byte = 0;
      j++;
      if (tdi)
        tdi_byte = tdi[j];
    }
  }
  tdo_byte = tdo_byte + (txrx(last, tdi ? ((tdi_byte & 1) == 1) : false) << (i % 8));
  if (tdo)
    tdo[j] = tdo_byte;

  set_line(TCKPin, false);
}

void IOGPIODPi::tx_tms(unsigned char *pat, int length, int)
{
  int i;
  unsigned char tms = 0;
  for (i = 0; i < length; i++) {
    if ((i & 0x7) == 0)
      tms = pat[i >> 3];
    tx((tms & 0x01), true);
    tms = tms >> 1;
  }
  set_line(TCKPin, false);
}

void IOGPIODPi::tx(bool tms, bool tdi)
{

  set_line(TCKPin, false);

  set_line(TDIPin, tdi);
  set_line(TMSPin, tms);

  set_line(TCKPin, true);
}

bool IOGPIODPi::txrx(bool tms, bool tdi)
{
  tx(tms, tdi);
  return get_line(TDOPin);
}
