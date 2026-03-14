#include "quadrature_encoder.h"

#include "LPC17xx.h"
#include "spi_comms.h"

// PCONP bit for QEI
constexpr uint32_t PCONP_QEI = 1 << 18;

// PCLKSEL1 bits for QEI (bits 0-1): 00=CCLK/4, 01=CCLK, 10=CCLK/2, 11=CCLK/8
constexpr uint32_t PCLKSEL1_QEI_MASK = 0x3;
constexpr uint32_t PCLKSEL1_QEI_CCLK = 0x1;

// PINSEL3 configuration for QEI pins
// P1.20 = MCI0 (PhA): bits 8-9 = 01
// P1.23 = MCI1 (PhB): bits 14-15 = 01
constexpr uint32_t PINSEL3_MCI0_MASK = 0x3 << 8;
constexpr uint32_t PINSEL3_MCI0_QEI = 0x1 << 8;
constexpr uint32_t PINSEL3_MCI1_MASK = 0x3 << 14;
constexpr uint32_t PINSEL3_MCI1_QEI = 0x1 << 14;

constexpr uint32_t QEICON_RESP = 1 << 0;  // reset position counter
constexpr uint32_t QEICON_RESV = 1 << 2;  // reset velocity counter

QuadratureEncoder::QuadratureEncoder(const uint8_t position_var_number, const uint8_t velocity_var_number,
                                     const bool count_both_phases, const uint32_t velocity_period_us)
    : position_var_number_(position_var_number), velocity_var_number_(velocity_var_number) {
  LPC_SC->PCONP |= PCONP_QEI;

  // Set QEI peripheral clock to CCLK (100MHz)
  LPC_SC->PCLKSEL1 = (LPC_SC->PCLKSEL1 & ~PCLKSEL1_QEI_MASK) | PCLKSEL1_QEI_CCLK;

  LPC_PINCON->PINSEL3 = (LPC_PINCON->PINSEL3 & ~PINSEL3_MCI0_MASK) | PINSEL3_MCI0_QEI;
  LPC_PINCON->PINSEL3 = (LPC_PINCON->PINSEL3 & ~PINSEL3_MCI1_MASK) | PINSEL3_MCI1_QEI;

  LPC_QEI->QEICONF = count_both_phases ? (1 << 2) : 0;
  LPC_QEI->QEIMAXPOS = 0xFFFFFFFF;
  LPC_QEI->QEICON = QEICON_RESP;
  LPC_QEI->FILTER = 100;

  if (velocity_period_us > 0) {
    LPC_QEI->QEILOAD = SystemCoreClock / 1000000 * velocity_period_us;
    LPC_QEI->QEICON = QEICON_RESV;  // reset velocity counter to start fresh
  }
}

void QuadratureEncoder::OnRx() {
  auto* const vars = SpiComms::Instance()->get_pru_state()->input_vars;
  vars[position_var_number_] = static_cast<int32_t>(LPC_QEI->QEIPOS);
  vars[velocity_var_number_] = static_cast<int32_t>(LPC_QEI->QEICAP);
}
