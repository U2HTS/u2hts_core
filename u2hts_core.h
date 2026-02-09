/*
  Copyright (C) CNflysky.
  U2HTS stands for "USB to HID TouchScreen".
  This file is licensed under GPL V3.
  All rights reserved.
 */

#ifndef _U2HTS_CORE_H_
#define _U2HTS_CORE_H_

#include <stdbool.h>
#include <stdint.h>  // uint
#include <stdio.h>   // printf
#include <stdlib.h>  // atoi etc
#include <string.h>  // memcpy

#define U2HTS_LOG_LEVEL_ERROR 0
#define U2HTS_LOG_LEVEL_WARN 1
#define U2HTS_LOG_LEVEL_INFO 2
#define U2HTS_LOG_LEVEL_DEBUG 3

#define U2HTS_MAX_TPS 10
#define U2HTS_TPS_RELEASE_TIMEOUT 10 * 1000  // 10 ms
#define U2HTS_DEFAULT_TP_WIDTH 0x30
#define U2HTS_DEFAULT_TP_HEIGHT 0x30
#define U2HTS_DEFAULT_TP_PRESSURE 0x30
#define U2HTS_LOGICAL_MAX 4096

#define U2HTS_HID_REPORT_TP_ID 1
#define U2HTS_HID_REPORT_TP_MAX_COUNT_ID 2
#define U2HTS_HID_REPORT_TP_MS_THQA_CERT_ID 3

#define U2HTS_CONFIG_ROTATION_0 0
#define U2HTS_CONFIG_ROTATION_90 1
#define U2HTS_CONFIG_ROTATION_180 2
#define U2HTS_CONFIG_ROTATION_270 3

#define U2HTS_CUSTOM_CONFIG_STR_MAX_TOTAL_LENGTH 512
#define U2HTS_CUSTOM_CONFIG_STR_MAX_KEY_LENGTH 128

#define U2HTS_UNUSED(x) (void)(x)

#define U2HTS_SWAP16(x) __builtin_bswap16(x)
#define U2HTS_SWAP32(x) __builtin_bswap32(x)

#if U2HTS_LOG_LEVEL >= U2HTS_LOG_LEVEL_ERROR
#define U2HTS_LOG_ERROR(...) \
  do {                       \
    printf("ERROR: ");       \
    printf(__VA_ARGS__);     \
    printf("\n");            \
  } while (0)
#else
#define U2HTS_LOG_ERROR(...) U2HTS_UNUSED(0)
#endif

#if U2HTS_LOG_LEVEL >= U2HTS_LOG_LEVEL_WARN
#define U2HTS_LOG_WARN(...) \
  do {                      \
    printf("WARN: ");       \
    printf(__VA_ARGS__);    \
    printf("\n");           \
  } while (0)
#else
#define U2HTS_LOG_WARN(...) U2HTS_UNUSED(0)
#endif

#if U2HTS_LOG_LEVEL >= U2HTS_LOG_LEVEL_INFO
#define U2HTS_LOG_INFO(...) \
  do {                      \
    printf("INFO: ");       \
    printf(__VA_ARGS__);    \
    printf("\n");           \
  } while (0)
#else
#define U2HTS_LOG_INFO(...) U2HTS_UNUSED(0)
#endif

#if U2HTS_LOG_LEVEL >= U2HTS_LOG_LEVEL_DEBUG
#define U2HTS_LOG_DEBUG(...) \
  do {                       \
    printf("DEBUG: ");       \
    printf(__VA_ARGS__);     \
    printf("\n");            \
  } while (0)
#else
#define U2HTS_LOG_DEBUG(...) U2HTS_UNUSED(0)
#endif

#define U2HTS_MAP_VALUE(value, src, dest) (((value) * (dest)) / (src))

#define U2HTS_SET_BIT(val, bit, set) \
  ((set) ? ((val) |= (1U << (bit))) : ((val) &= ~(1U << (bit))))

#define U2HTS_CHECK_BIT(val, bit) ((val >> (bit)) & 1)

#define U2HTS_TOUCH_CONTROLLER(controller)                                   \
  __attribute__((                                                            \
      __used__,                                                              \
      __section__(                                                           \
          ".u2hts_touch_controllers"))) static const u2hts_touch_controller* \
      u2hts_touch_controller_##controller = &controller

#define U2HTS_SET_TP_COUNT_SAFE(TP_COUNT)                             \
  do {                                                                \
    report->tp_count = TP_COUNT;                                      \
    if (!report->tp_count) return false;                              \
    report->tp_count = (report->tp_count < cfg->coord_config.max_tps) \
                           ? report->tp_count                         \
                           : cfg->coord_config.max_tps;               \
  } while (0)

typedef enum {
  UE_OK,       // No error
  UE_NSLAVE,   // no slave on i2c bus
  UE_NCOMPAT,  // no driver matched
  UE_NCONF,    // required parameters not configured
  UE_FSETUP,   // failed to setup controller
  UE_INCFG,    // read config invalid
} U2HTS_ERROR_CODES;

typedef enum {
  IRQ_TYPE_EDGE_FALLING = 1,
  IRQ_TYPE_EDGE_RISING,
  IRQ_TYPE_LEVEL_LOW,
  IRQ_TYPE_LEVEL_HIGH
} U2HTS_IRQ_TYPES;

typedef enum {
  UB_I2C,
  UB_SPI,
} U2HTS_BUS_TYPES;

/*
  Some controller constantly reporting finger coordinates, even if fingers are
  not moving.
  Others reports after a finger moving event triggered, usually
  use a bit/byte to indicate the finger still presents or not.
*/

typedef enum {
  UTC_REPORT_MODE_CONTINOUS,
  UTC_REPORT_MODE_EVENT
} U2HTS_TOUCH_CONTROLLER_REPORT_MODE;

typedef struct __packed {
  bool contact : 1;
  uint8_t id : 7;
  uint16_t x;
  uint16_t y;
  uint8_t width;
  uint8_t height;
  uint8_t pressure;
} u2hts_tp;

typedef struct __packed {
  u2hts_tp tp[U2HTS_MAX_TPS];
  uint16_t scan_time;
  uint8_t tp_count;
} u2hts_hid_report;

typedef struct {
  uint16_t x_max;
  uint16_t y_max;
  uint8_t max_tps;
} u2hts_touch_controller_config;

typedef struct __packed {
  uint8_t addr;
  uint32_t speed_hz;
} u2hts_i2c_config;

typedef struct __packed {
  uint32_t speed_hz;
  uint8_t cpol;
  uint8_t cpha;
} u2hts_spi_config;

typedef struct __packed {
  bool x_y_swap;
  bool x_invert;
  bool y_invert;
  uint16_t x_max;
  uint16_t y_max;
  uint8_t max_tps;
} u2hts_coord_config;

typedef struct {
  const char* controller;
  U2HTS_BUS_TYPES bus_type;
  u2hts_i2c_config i2c_config;
  u2hts_spi_config spi_config;
  u2hts_coord_config coord_config;
  U2HTS_IRQ_TYPES irq_type;
  uint32_t report_delay;
  bool polling_mode;
  const char* custom_controller_config;
} u2hts_config;

typedef struct {
  bool (*setup)(U2HTS_BUS_TYPES bus_type, const char* custom_controller_config);
  void (*get_config)(u2hts_touch_controller_config* cfg);
  bool (*fetch)(const u2hts_config* cfg, u2hts_hid_report* report);
} u2hts_touch_controller_operations;

typedef struct {
  const char* name;
  U2HTS_TOUCH_CONTROLLER_REPORT_MODE report_mode;
  U2HTS_IRQ_TYPES irq_type;
  u2hts_i2c_config i2c_config;
  uint8_t alt_i2c_addr;  // some controller can have configurable slave address
  u2hts_spi_config spi_config;
  u2hts_touch_controller_operations* operations;
} u2hts_touch_controller;

#include "u2hts_api.h"

U2HTS_ERROR_CODES u2hts_init(u2hts_config* cfg);
void u2hts_task();
uint8_t u2hts_get_max_tps();

void u2hts_i2c_mem_write(uint8_t slave_addr, uint32_t mem_addr,
                         size_t mem_addr_size, void* data, size_t data_len);
void u2hts_i2c_mem_read(uint8_t slave_addr, uint32_t mem_addr,
                        size_t mem_addr_size, void* data, size_t data_len);

void u2hts_ts_irq_status_set(bool status);
void u2hts_apply_config(u2hts_config* cfg, uint8_t config_index);
void u2hts_transform_touch_data(const u2hts_config* cfg, u2hts_tp* tp);
size_t u2hts_get_custom_config(const char* config_name, uint8_t* buf,
                               size_t bufsiz);
inline static int32_t u2hts_get_custom_config_i32(const char* config_name) {
  uint8_t buf[U2HTS_CUSTOM_CONFIG_STR_MAX_KEY_LENGTH] = {0};
  if (u2hts_get_custom_config(config_name, buf, sizeof(buf)))
    return atoi((const char*)buf);
  else
    return -1;
}

#ifdef U2HTS_ENABLE_LED
typedef struct {
  bool state;
  uint32_t delay_ms;
} u2hts_led_pattern;

void u2hts_led_show_error_code(U2HTS_ERROR_CODES code);
#endif

#ifdef U2HTS_ENABLE_PERSISTENT_CONFIG
#define U2HTS_CONFIG_MAGIC 0xBA

typedef union {
  struct {
    uint8_t magic;
    bool x_y_swap : 1;
    bool x_invert : 1;
    bool y_invert : 1;
  };
  uint16_t value;
} u2hts_config_mask;

inline static void u2hts_save_config(u2hts_config* cfg) {
  u2hts_config_mask mask = {
      .magic = U2HTS_CONFIG_MAGIC,
      .x_y_swap = cfg->coord_config.x_y_swap,
      .x_invert = cfg->coord_config.x_invert,
      .y_invert = cfg->coord_config.y_invert,
  };
  U2HTS_LOG_DEBUG("%s: mask.value = 0x%x", __func__, mask.value);
  u2hts_write_config(mask.value);
}

inline static void u2hts_load_config(u2hts_config* cfg) {
  u2hts_config_mask mask = {0};
  mask.value = u2hts_read_config();
  U2HTS_LOG_DEBUG("%s: mask.value = 0x%x", __func__, mask.value);
  cfg->coord_config.x_y_swap = mask.x_y_swap;
  cfg->coord_config.x_invert = mask.x_invert;
  cfg->coord_config.y_invert = mask.y_invert;
}

inline static bool u2hts_config_exists() {
  u2hts_config_mask mask = {0};
  mask.value = u2hts_read_config();
  U2HTS_LOG_DEBUG("%s: mask.value = 0x%x", __func__, mask.value);
  return (mask.magic == U2HTS_CONFIG_MAGIC);
}
#endif

#endif