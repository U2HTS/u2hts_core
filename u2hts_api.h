/*
  Copyright (C) CNflysky.
  U2HTS stands for "USB to HID TouchScreen".
  This file is licensed under GPL V3.
  All rights reserved.
*/

#ifndef _U2HTS_API_H_
#define _U2HTS_API_H_

void u2hts_i2c_init(uint32_t speed_hz);
void u2hts_i2c_set_speed(uint32_t speed_hz);
bool u2hts_i2c_write(uint8_t slave_addr, void* buf, size_t len, bool stop);
bool u2hts_i2c_read(uint8_t slave_addr, void* buf, size_t len);
void u2hts_spi_init(u2hts_spi_config *config);
bool u2hts_spi_transfer(void* buf, size_t len);

// true = out false = in ; true for pull up, false for pull down
void u2hts_tpint_set_mode(bool mode, bool pull);
void u2hts_tpint_set(bool value);
bool u2hts_tpint_get();
void u2hts_ts_irq_set(bool enable);
void u2hts_ts_irq_init(U2HTS_IRQ_TYPES irq_type);
bool u2hts_i2c_detect_slave(uint8_t addr);
void u2hts_tprst_set(bool value);
void u2hts_delay_ms(uint32_t ms);
void u2hts_delay_us(uint32_t us);
void u2hts_usb_report(uint8_t report_id, const u2hts_hid_report* report);
bool u2hts_usb_init();
uint16_t u2hts_get_timestamp();
void u2hts_led_set(bool on);
void u2hts_write_config(uint16_t cfg);
uint16_t u2hts_read_config();
bool u2hts_usrkey_get();
// true = okay false = busy
bool u2hts_get_usb_status();

#endif