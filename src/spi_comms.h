#ifndef COMMS_H
#define COMMS_H

#include "MODDMA.h"
#include "mbed.h"
#include "spi_data.h"

typedef linuxCncData_t rxData_t;
typedef pruData_t txData_t;

class SpiComms {
  MODDMA dma;

  MODDMA_Config* rx_dma1;
  MODDMA_Config* rx_dma2;
  MODDMA_Config* tx_dma1;
  MODDMA_Config* tx_dma2;
  MODDMA_Config* rx_memcpy_dma1;
  MODDMA_Config* rx_memcpy_dma2;

  rxData_t temp_rx_buffer1{};
  rxData_t temp_rx_buffer2{};
  uint8_t reject_count = 0;
  bool data_ready = false;
  bool spi_error = false;

 public:
  SpiComms();

  rxData_t volatile* rx_data;
  txData_t volatile* tx_data;

  void tx1_callback();
  void tx2_callback();
  void rx1_callback();
  void rx2_callback();
  void err_callback();

  void init();
  void start();
  bool get_status() const;
  void set_status(bool);
  bool get_error() const;
  void set_error(bool);
};

#endif
