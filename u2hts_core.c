/*
  Copyright (C) U2HTS Developers. All rights reserved.
  U2HTS stands for "USB to HID TouchScreen".
  This file is licensed under GPL V3.
*/
#include "u2hts_core.h"

#include "u2hts_hid_report_descriptor.h"

#ifndef U2HTS_GIT_SHA
#define U2HTS_GIT_SHA "UNKNOWN"
#endif

#define LOG_TAG "core"

// u2hts_touch_controllers section border
extern u2hts_touch_controller *__start_u2hts_touch_controllers,
    *__stop_u2hts_touch_controllers;

static u2hts_touch_controller* touch_controller = NULL;

const static u2hts_config* config = NULL;

static u2hts_coord_config coord_config = {0};

static uint16_t u2hts_last_report_ts = 0;
__aligned(4) static u2hts_hid_report u2hts_report, u2hts_previous_report;
static uint16_t u2hts_tp_ids_mask = 0;
// union u2hts_status_mask {
//   struct {
//     uint8_t interrupt_status : 1;
//     uint8_t config_mode : 1;
//     uint8_t tps_remain : 1;
//   };
//   uint8_t mask;
// };
static volatile uint8_t u2hts_status_mask = 0x00;

// Rotation configs: default, 90°, 180°, 270°
// x_y_swap, x_invert 90
// x_invert, y_invert 180
// x_y_swap, y_invert 270
static __unused const uint16_t u2hts_configs[] = {0x0, 0x320, 0x620, 0x520};

#ifdef U2HTS_ENABLE_FREERTOS
static TaskHandle_t u2hts_touch_task_handle = NULL;
static StackType_t
    u2hts_touch_task_stack[U2HTS_TOUCH_TASK_STACK_SIZE] = {0},
    u2hts_tps_release_task_stack[U2HTS_TPS_RELEASE_TASK_STACK_SIZE] = {0};
SemaphoreHandle_t u2hts_log_print_mutex = NULL;
static StaticSemaphore_t u2hts_log_print_mutex_buf = {0};
#ifdef U2HTS_ENABLE_KEY
static StackType_t u2hts_key_task_stack[U2HTS_KEY_TASK_STACK_SIZE] = {0};
static StaticTask_t u2hts_key_tcb = {0};
#endif

static StaticTask_t u2hts_touch_tcb = {0}, u2hts_tps_release_tcb = {0};
__weak_symbol void u2hts_delay_ms(uint32_t ms) {
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    u2hts_sleep_ms(ms);
  else
    vTaskDelay(pdMS_TO_TICKS(ms));
}

__weak_symbol uint16_t u2hts_get_timestamp() {
  return xTaskGetTickCount() * portTICK_PERIOD_MS;
}
#endif

#define U2HTS_SET_INT_FLAG(x) U2HTS_SET_BIT(u2hts_status_mask, 0, x)
#define U2HTS_SET_CONFIG_MODE_FLAG(x) U2HTS_SET_BIT(u2hts_status_mask, 1, x)
#define U2HTS_SET_TPS_REMAIN_FLAG(x) U2HTS_SET_BIT(u2hts_status_mask, 2, x)
#define U2HTS_GET_INT_FLAG() U2HTS_CHECK_BIT(u2hts_status_mask, 0)
#define U2HTS_GET_CONFIG_MODE_FLAG() U2HTS_CHECK_BIT(u2hts_status_mask, 1)
#define U2HTS_GET_TPS_REMAIN_FLAG() U2HTS_CHECK_BIT(u2hts_status_mask, 2)

__weak_symbol void u2hts_led_set(bool on) { U2HTS_UNUSED(on); }
__weak_symbol bool u2hts_usrkey_get() { return false; }
__weak_symbol void u2hts_write_config(uint16_t cfg) { U2HTS_UNUSED(cfg); }
__weak_symbol uint16_t u2hts_read_config() { return 0; }

static const char* u2hts_log_str[] = {"E", "W", "I", "D"};

#ifdef U2HTS_ENABLE_LOG_COLOR
static const char* u2hts_log_color[] = {"\x1b[31m", "\x1b[33m", "\x1b[32m",
                                        "\x1b[0m"};
#else
static const char* u2hts_log_color[] = {"", "", "", ""};
#endif

void _u2hts_log(uint8_t level, const char* tag, const char* format, ...) {
  va_list ap;
  va_start(ap, format);

#ifdef U2HTS_ENABLE_FREERTOS
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    xSemaphoreTake(u2hts_log_print_mutex, portMAX_DELAY);
#endif

  printf("%s[%s][%s] ", u2hts_log_color[level], u2hts_log_str[level], tag);
  vprintf(format, ap);
  printf("%s\r\n", u2hts_log_color[U2HTS_LOG_LEVEL_DEBUG]);

#ifdef U2HTS_ENABLE_FREERTOS
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    xSemaphoreGive(u2hts_log_print_mutex);
#endif

  va_end(ap);
}

static void u2hts_led_blink(uint16_t times) {
  for (uint16_t i = 0; i < times; i++) {
    u2hts_led_set(true);
    u2hts_delay_ms(200);
    u2hts_led_set(false);
    u2hts_delay_ms(200);
  }
}

void u2hts_led_show_error_code(U2HTS_ERROR_CODES code) {
  u2hts_led_blink(code);
  u2hts_delay_ms(1000);
}

bool u2hts_i2c_mem_write(uint8_t slave_addr, uint32_t mem_addr,
                         size_t mem_addr_size, void* data, size_t data_len) {
  uint8_t tx_buf[mem_addr_size + data_len];
  uint32_t mem_addr_be = 0x00;
  switch (mem_addr_size) {
    case sizeof(uint16_t):
      mem_addr_be = U2HTS_SWAP16(mem_addr);
      break;
    case sizeof(uint32_t):
      mem_addr_be = U2HTS_SWAP32(mem_addr);
      break;
    default:
      mem_addr_be = mem_addr;
      break;
  }
  memcpy(tx_buf, &mem_addr_be, mem_addr_size);
  memcpy(tx_buf + mem_addr_size, data, data_len);
  bool ret = u2hts_i2c_write(slave_addr, tx_buf, sizeof(tx_buf), true);
  if (!ret)
    ULOGE(LOG_TAG, "%s error, reg = 0x%x, ret = %d", __func__, mem_addr, ret);
  return ret;
}

bool u2hts_i2c_mem_read(uint8_t slave_addr, uint32_t mem_addr,
                        size_t mem_addr_size, void* data, size_t data_len) {
  uint32_t mem_addr_be = 0x00;
  switch (mem_addr_size) {
    case sizeof(uint16_t):
      mem_addr_be = U2HTS_SWAP16(mem_addr);
      break;
    case sizeof(uint32_t):
      mem_addr_be = U2HTS_SWAP32(mem_addr);
      break;
    default:
      mem_addr_be = mem_addr;
      break;
  }
  bool ret = u2hts_i2c_write(slave_addr, &mem_addr_be, mem_addr_size, false);
  if (!ret) {
    ULOGE(LOG_TAG, "%s write error, addr = 0x%x, ret = %d", __func__, mem_addr,
          ret);
    return ret;
  }

  ret = u2hts_i2c_read(slave_addr, data, data_len);
  if (!ret)
    ULOGE(LOG_TAG, "%s error, addr = 0x%x, ret = %d", __func__, mem_addr, ret);
  return ret;
}

void u2hts_int_handler() {
  u2hts_int_set(false);
  U2HTS_SET_INT_FLAG(1);
#ifdef U2HTS_ENABLE_FREERTOS
  BaseType_t hptw = pdFALSE;
  vTaskNotifyGiveFromISR(u2hts_touch_task_handle, &hptw);
  portYIELD_FROM_ISR(hptw);
#endif
}

static inline void u2hts_save_config() {
  u2hts_config_mask mask = {
      .magic = U2HTS_CONFIG_MAGIC,
      .x_y_swap = coord_config.x_y_swap,
      .x_invert = coord_config.x_invert,
      .y_invert = coord_config.y_invert,
  };
  ULOGD(LOG_TAG, "%s: mask.value = 0x%x", __func__, mask.value);
  u2hts_write_config(mask.value);
}

static inline void u2hts_load_config() {
  u2hts_config_mask mask = {0};
  mask.value = u2hts_read_config();
  ULOGD(LOG_TAG, "%s: mask.value = 0x%x", __func__, mask.value);
  coord_config.x_y_swap = mask.x_y_swap;
  coord_config.x_invert = mask.x_invert;
  coord_config.y_invert = mask.y_invert;
}

static inline bool u2hts_config_exists() {
  u2hts_config_mask mask = {0};
  mask.value = u2hts_read_config();
  ULOGD(LOG_TAG, "%s: mask.value = 0x%x", __func__, mask.value);
  return (mask.magic == U2HTS_CONFIG_MAGIC);
}

void u2hts_apply_config(uint8_t config_index) {
  union {
    struct {
      uint8_t magic;
      uint8_t x_y_swap : 1;
      uint8_t x_invert : 1;
      uint8_t y_invert : 1;
    };
    uint16_t mask;
  } u2hts_config_mask;
  u2hts_config_mask.mask = u2hts_configs[config_index];
  coord_config.x_y_swap = u2hts_config_mask.x_y_swap;
  coord_config.x_invert = u2hts_config_mask.x_invert;
  coord_config.y_invert = u2hts_config_mask.y_invert;
}

void u2hts_set_tp_count(uint8_t tp_count) {
  u2hts_report.tp_count =
      tp_count > coord_config.max_tps ? coord_config.max_tps : tp_count;
}

void u2hts_set_tp(uint8_t tp_index, bool contact, uint8_t id, uint16_t x,
                  uint16_t y, uint8_t width, uint8_t height, uint8_t pressure) {
  ULOGD(LOG_TAG,
        "raw coordinates: index = %d, contact = %d, id = %d, x = %d, y = %d, "
        "width = %d, height = "
        "%d, "
        "pressure = %d",
        tp_index, contact, id, x, y, width, height, pressure);
  if (tp_index >= U2HTS_MAX_TPS) return;
  x += coord_config.x_offset;
  y += coord_config.y_offset;
  u2hts_report.tp[tp_index].contact = contact;
  u2hts_report.tp[tp_index].id = id;
  u2hts_report.tp[tp_index].x =
      (x > coord_config.x_max)
          ? U2HTS_LOGICAL_MAX
          : U2HTS_SCALE_VALUE(x, coord_config.x_max, U2HTS_LOGICAL_MAX);
  u2hts_report.tp[tp_index].y =
      (y > coord_config.y_max)
          ? U2HTS_LOGICAL_MAX
          : U2HTS_SCALE_VALUE(y, coord_config.y_max, U2HTS_LOGICAL_MAX);
  if (coord_config.x_y_swap) {
    u2hts_report.tp[tp_index].x ^= u2hts_report.tp[tp_index].y;
    u2hts_report.tp[tp_index].y ^= u2hts_report.tp[tp_index].x;
    u2hts_report.tp[tp_index].x ^= u2hts_report.tp[tp_index].y;
  }
  if (coord_config.x_invert)
    u2hts_report.tp[tp_index].x =
        U2HTS_LOGICAL_MAX - u2hts_report.tp[tp_index].x;
  if (coord_config.y_invert)
    u2hts_report.tp[tp_index].y =
        U2HTS_LOGICAL_MAX - u2hts_report.tp[tp_index].y;
#ifdef U2HTS_ENABLE_COMPACT_REPORT
  U2HTS_UNUSED(width);
  U2HTS_UNUSED(height);
  U2HTS_UNUSED(pressure);
#else
  u2hts_report.tp[tp_index].width = width ? width : U2HTS_DEFAULT_TP_WIDTH;
  u2hts_report.tp[tp_index].height = height ? height : U2HTS_DEFAULT_TP_HEIGHT;
  u2hts_report.tp[tp_index].pressure =
      pressure ? pressure : U2HTS_DEFAULT_TP_PRESSURE;
#endif
}

static inline bool u2hts_get_key_timeout(uint32_t ms) {
  if (u2hts_usrkey_get()) {
    u2hts_delay_ms(ms);
    return u2hts_usrkey_get();
  } else
    return false;
}

static inline void u2hts_handle_config() {
  ULOGI(LOG_TAG, "Enter config mode");
  u2hts_led_set(true);
  u2hts_delay_ms(500);

  uint32_t u2hts_config_timeout = 0;
  uint8_t config_index = 0;
  do {
    u2hts_led_set(true);
    if (u2hts_get_key_timeout(20)) {
      u2hts_config_timeout = 0;
      config_index =
          (config_index < sizeof(u2hts_configs) / sizeof(uint16_t) - 1)
              ? config_index + 1
              : 0;
      ULOGI(LOG_TAG, "switching config %d", config_index);
      u2hts_led_blink(config_index + 1);
    } else {
      u2hts_delay_ms(1);
      u2hts_config_timeout++;
    }
  } while (u2hts_config_timeout < U2HTS_CONFIG_TIMEOUT);
  ULOGI(LOG_TAG, "Exit config mode");
  u2hts_apply_config(config_index);
  ULOGD(LOG_TAG, "Applyed config : x_y_swap = %d, x_invert = %d, y_invert = %d",
        coord_config.x_y_swap, coord_config.x_invert, coord_config.y_invert);
  ULOGI(LOG_TAG, "Saving config");
  u2hts_save_config();
  U2HTS_SET_CONFIG_MODE_FLAG(0);
#ifdef U2HTS_ENABLE_FREERTOS
  vTaskResume(u2hts_touch_task_handle);
#endif
}

static inline void u2hts_list_touch_controller() {
#if U2HTS_LOG_LEVEL >= U2HTS_LOG_LEVEL_INFO
  printf("%s[I][%s] Supported controllers:",
         u2hts_log_color[U2HTS_LOG_LEVEL_INFO], LOG_TAG);
  for (u2hts_touch_controller** tc = &__start_u2hts_touch_controllers;
       tc < &__stop_u2hts_touch_controllers; tc++)
    printf(" %s", (*tc)->name);
  printf("%s\r\n", u2hts_log_color[U2HTS_LOG_LEVEL_DEBUG]);
#endif
}

static inline u2hts_touch_controller* u2hts_get_touch_controller_by_name(
    const char* name) {
  for (u2hts_touch_controller** tc = &__start_u2hts_touch_controllers;
       tc < &__stop_u2hts_touch_controllers; tc++)
    if (!strcmp((*tc)->name, name)) return *tc;
  return NULL;
}

static u2hts_touch_controller* u2hts_get_touch_controller_by_i2c_addr(
    const uint8_t addr) {
  for (u2hts_touch_controller** tc = &__start_u2hts_touch_controllers;
       tc < &__stop_u2hts_touch_controllers; tc++) {
    if ((*tc)->i2c_config.primary_addr == addr) return *tc;
    if ((*tc)->i2c_config.alt_addrs)
      for (const uint8_t* alt = (*tc)->i2c_config.alt_addrs; *alt; alt++)
        if (*alt == addr) return *tc;
  }
  return NULL;
}

size_t u2hts_get_custom_config(const char* config_name, uint8_t* buf,
                               size_t bufsiz) {
  if (!config->custom_controller_config) return 0;

  const char* src = config->custom_controller_config;

  while (*src) {
    while (*src == ' ') src++;
    if (!*src) break;

    const char *token = src, *t = token;
    while (*src && *src != ' ') src++;
    const char *token_end = src, *k = config_name;

    while (*k && t < token_end && *k == *t) {
      k++;
      t++;
    }

    if (!*k && t < token_end && *t == '=') {
      t++;

      size_t len = (size_t)(token_end - t);
      if (len >= bufsiz) len = bufsiz - 1;
      memcpy(buf, t, len);
      buf[len] = '\0';
      return len;
    }
  }
  return 0;
}

static inline U2HTS_ERROR_CODES u2hts_i2c_detect_touch_controller(
    u2hts_touch_controller** tc) {
  uint8_t slave_addr = 0x00;
  ULOGI(LOG_TAG, "Scanning i2c slaves...");
  for (uint8_t i = 0x01; i < 0x7F; i++)
    if (u2hts_i2c_detect_slave(i)) {
      slave_addr = i;
      break;
    } else
      ULOGD(LOG_TAG, "No slave on 0x%x", i);

  if (!slave_addr) {
    ULOGE(LOG_TAG, "No slave on i2c bus");
    return UE_NSLAVE;
  }

  *tc = u2hts_get_touch_controller_by_i2c_addr(slave_addr);
  if (!*tc) {
    ULOGE(LOG_TAG, "Slave 0x%x on I2C has no known controller match",
          slave_addr);
    return UE_NCOMPAT;
  }

  ULOGI(LOG_TAG, "Found controller %s @ addr 0x%x", (*tc)->name, slave_addr);
  return UE_OK;
}

uint8_t u2hts_get_max_tps() { return coord_config.max_tps; }

static inline void u2hts_append_released_tps() {
  if (u2hts_previous_report.tp_count == u2hts_report.tp_count) return;

  uint16_t new_ids_mask = 0;
  for (uint8_t i = 0; i < u2hts_report.tp_count; i++)
    U2HTS_SET_BIT(new_ids_mask, u2hts_report.tp[i].id, 1);

  uint16_t released_ids_mask = u2hts_tp_ids_mask & ~new_ids_mask;

  for (uint8_t i = 0; i < u2hts_previous_report.tp_count; i++) {
    if (U2HTS_CHECK_BIT(released_ids_mask, u2hts_previous_report.tp[i].id)) {
      u2hts_previous_report.tp[i].contact = false;
      u2hts_report.tp[u2hts_report.tp_count++] = u2hts_previous_report.tp[i];
    }
  }
  u2hts_tp_ids_mask = new_ids_mask;
}

static inline void u2hts_handle_touch() {
  ULOGD(LOG_TAG, "Enter %s", __func__);
  ULOGD(LOG_TAG, "u2hts_status_mask = %x", u2hts_status_mask);
  U2HTS_SET_INT_FLAG(config->polling_mode);
  memset(&u2hts_report, 0x00, sizeof(u2hts_hid_report));
  // for (uint8_t i = 0; i < U2HTS_MAX_TPS; i++) u2hts_report.tp[i].id = 0x7F;
  if (!touch_controller->operations->fetch()) {
    ULOGD(LOG_TAG,
          "Failed to fetch touch data, tp_count = %d, previous_tp_count = %d",
          u2hts_report.tp_count, u2hts_previous_report.tp_count);
    return;
  }

  ULOGD(LOG_TAG, "tp_count = %d", u2hts_report.tp_count);
  u2hts_report.scan_time = u2hts_get_timestamp();
  u2hts_report.report_id = U2HTS_HID_REPORT_TP_ID;

  if (!touch_controller->report_mode) u2hts_append_released_tps();

  for (uint8_t i = 0; i < u2hts_report.tp_count; i++) {
#ifdef U2HTS_ENABLE_COMPACT_REPORT
    ULOGD(LOG_TAG,
          "report.tp[%d].contact = %d, report.tp[i].x = %d, "
          "report.tp[i].y = %d, report.tp[i].id = %d",
          i, u2hts_report.tp[i].contact, u2hts_report.tp[i].x,
          u2hts_report.tp[i].y, u2hts_report.tp[i].id);
#else
    ULOGD(LOG_TAG,
          "report.tp[%d].contact = %d, report.tp[i].x = %d, "
          "report.tp[i].y = %d, report.tp[i].height = %d, "
          "report.tp[i].width = %d, report.tp[i].pressure = %d, "
          "report.tp[i].id "
          "= %d",
          i, u2hts_report.tp[i].contact, u2hts_report.tp[i].x,
          u2hts_report.tp[i].y, u2hts_report.tp[i].height,
          u2hts_report.tp[i].width, u2hts_report.tp[i].pressure,
          u2hts_report.tp[i].id);
#endif
  }

  ULOGD(LOG_TAG, "report.scan_time = %d, report.tp_count = %d",
        u2hts_report.scan_time, u2hts_report.tp_count);
  u2hts_delay_ms(config->report_delay);
  u2hts_usb_hid_report(&u2hts_report);
  if (!touch_controller->report_mode) {
    u2hts_previous_report = u2hts_report;
    U2HTS_SET_TPS_REMAIN_FLAG((u2hts_previous_report.tp_count > 0));
    u2hts_last_report_ts = u2hts_get_timestamp();
  }
}

static inline void u2hts_release_tps() {
  for (uint8_t i = 0; i < u2hts_previous_report.tp_count; i++)
    u2hts_previous_report.tp[i].contact = false;
  u2hts_previous_report.scan_time = u2hts_get_timestamp();
  u2hts_usb_hid_report(&u2hts_previous_report);
  u2hts_tp_ids_mask = 0;
  U2HTS_SET_TPS_REMAIN_FLAG(0);
  u2hts_last_report_ts = u2hts_get_timestamp();
}

#ifdef U2HTS_ENABLE_FREERTOS

static void u2hts_touch_task(void* pvParameters) {
  while (1) {
    if (config->polling_mode && u2hts_get_usb_status()) {
      u2hts_handle_touch();
      u2hts_delay_ms(1);  // yield for other tasks
    } else {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
      if (U2HTS_GET_INT_FLAG() && u2hts_get_usb_status()) u2hts_handle_touch();
      u2hts_int_set(!config->polling_mode && u2hts_get_usb_status());
    }
    u2hts_led_set(!u2hts_get_usb_status());
  }
}

static void u2hts_tps_release_task(void* pvParameters) {
  while (1) {
    u2hts_delay_ms(10);
    if (u2hts_get_usb_status() && U2HTS_GET_TPS_REMAIN_FLAG() &&
        (uint16_t)(u2hts_get_timestamp() - u2hts_last_report_ts) >
            U2HTS_TPS_RELEASE_TIMEOUT) {
      ULOGD(LOG_TAG, "releasing remain tps");
      u2hts_release_tps();
    }
  }
}
#ifdef U2HTS_ENABLE_KEY
static void u2hts_key_task(void* pvParameters) {
  while (1) {
    u2hts_delay_ms(10);
    if (u2hts_get_key_timeout(1000)) {
      vTaskSuspend(u2hts_touch_task_handle);
      u2hts_handle_config();
    }
  }
}
#endif
#endif

U2HTS_ERROR_CODES u2hts_init(const u2hts_config* cfg) {
  ULOGD(LOG_TAG, "Enter %s", __func__);
  U2HTS_ERROR_CODES u2hts_error = UE_OK;
  config = cfg;
  coord_config = config->coord_config;
  ULOGI(LOG_TAG, "U2HTS " U2HTS_GIT_SHA " built @ %s %s with feature%s",
        __DATE__, __TIME__,
        ""
#ifdef U2HTS_ENABLE_LED
        " U2HTS_ENABLE_LED"
#endif
#ifdef U2HTS_ENABLE_KEY
        " U2HTS_ENABLE_KEY"
#endif
#ifdef U2HTS_ENABLE_PERSISTENT_CONFIG
        " U2HTS_ENABLE_PERSISTENT_CONFIG"
#endif
#ifdef U2HTS_ENABLE_FREERTOS
        " U2HTS_ENABLE_FREERTOS"
#endif
#ifdef U2HTS_ENABLE_LOG_COLOR
        " U2HTS_ENABLE_LOG_COLOR"
#endif
  );
  u2hts_list_touch_controller();

  ULOGD(LOG_TAG, "This is a DEBUG build and expect low report rate!");

  if (u2hts_config_exists())
    u2hts_load_config();
  else
    u2hts_save_config();

  if (config->bus_type != UB_I2C && !strcmp(config->controller, "auto")) {
    ULOGE(LOG_TAG, "Bus type %d does not support controller detection",
          config->bus_type);
    return UE_NCONF;
  }

  if (config->bus_type == UB_I2C)
    u2hts_i2c_init(100 * 1000);  // 100 KHz for controller detection

  if (config->bus_type == UB_I2C && !strcmp(config->controller, "auto")) {
    if (config->override_i2c_config) {
      ULOGI(LOG_TAG, "Specified controller address: 0x%x",
            config->i2c_config.primary_addr);
      touch_controller = u2hts_get_touch_controller_by_i2c_addr(
          config->i2c_config.primary_addr);
      if (!touch_controller) {
        ULOGE(LOG_TAG, "Specified address 0x%x has no known controller match",
              config->i2c_config.primary_addr);
        return UE_NCOMPAT;
      }
      ULOGI(LOG_TAG, "Found controller %s @ addr 0x%x", touch_controller->name,
            config->i2c_config.primary_addr);
    } else
      u2hts_error = u2hts_i2c_detect_touch_controller(&touch_controller);
  } else {
    ULOGI(LOG_TAG, "Controller: %s", cfg->controller);
    touch_controller = u2hts_get_touch_controller_by_name(cfg->controller);
    if (!touch_controller) u2hts_error = UE_NCOMPAT;
  }

  if (u2hts_error) {
    ULOGE(LOG_TAG, "Failed to get touch controller");
    return u2hts_error;
  }

  if (config->bus_type == UB_SPI && !touch_controller->spi_config.speed_hz) {
    ULOGE(LOG_TAG, "Controller %s does not support SPI",
          touch_controller->name);
    return UE_NCOMPAT;
  }

  if (config->bus_type == UB_I2C && !touch_controller->i2c_config.speed_hz) {
    ULOGE(LOG_TAG, "Controller %s does not support I2C",
          touch_controller->name);
    return UE_NCOMPAT;
  }

  u2hts_i2c_config i2c_config = (config->override_i2c_config)
                                    ? config->i2c_config
                                    : touch_controller->i2c_config;
  u2hts_spi_config spi_config = (config->override_spi_config)
                                    ? config->spi_config
                                    : touch_controller->spi_config;
  U2HTS_INT_FLAGS int_flag =
      (config->int_flag) ? config->int_flag : touch_controller->int_flag;

  switch (config->bus_type) {
    case UB_I2C:
      u2hts_i2c_set_speed(i2c_config.speed_hz);
      break;
    case UB_SPI:
      u2hts_spi_init(&spi_config);
      break;
  }

  // setup controller
  if (!touch_controller->operations->setup((u2hts_bus_config){
          config->bus_type, .i2c_addr = i2c_config.primary_addr})) {
    ULOGE(LOG_TAG, "Failed to setup controller: %s", touch_controller->name);
    return UE_FSETUP;
  }

  u2hts_touch_controller_coord_config tc_coord_cfg = {0};

  if (!coord_config.x_max || !coord_config.y_max || !coord_config.max_tps) {
    if (touch_controller->operations->get_config) {
      touch_controller->operations->get_config(&tc_coord_cfg);
      ULOGI(LOG_TAG,
            "Controller built-in config: max_tps = %d, x_max = %d, y_max = "
            "%d",
            tc_coord_cfg.max_tps, tc_coord_cfg.x_max, tc_coord_cfg.y_max);
      if (!tc_coord_cfg.x_max || !tc_coord_cfg.y_max || !tc_coord_cfg.max_tps) {
        ULOGE(LOG_TAG, "Config values out of range");
        return UE_INCFG;
      }
      if (!coord_config.x_max) coord_config.x_max = tc_coord_cfg.x_max;
      if (!coord_config.y_max) coord_config.y_max = tc_coord_cfg.y_max;
      if (!coord_config.max_tps) coord_config.max_tps = tc_coord_cfg.max_tps;
    } else {
      ULOGE(LOG_TAG,
            "Controller does not support read configuration, but x/y coords or "
            "max_tps are not configured");
      return UE_NCONF;
    }
  }

  if (coord_config.x_max > U2HTS_LOGICAL_MAX ||
      coord_config.y_max > U2HTS_LOGICAL_MAX ||
      coord_config.max_tps > U2HTS_MAX_TPS) {
    ULOGE(LOG_TAG, "Config values out of range");
    return UE_INCFG;
  }

  ULOGI(LOG_TAG,
        "U2HTS config: x_max = %d, y_max = %d, max_tps = %d, x_offset = %d, "
        "y_offset = %d, x_y_swap = %d, "
        "x_invert = %d, y_invert = %d, polling_mode = %d",
        coord_config.x_max, coord_config.y_max, coord_config.max_tps,
        coord_config.x_offset, coord_config.y_offset, coord_config.x_y_swap,
        coord_config.x_invert, coord_config.y_invert, config->polling_mode);
  u2hts_usb_init();
  if (!config->polling_mode) u2hts_int_init(int_flag);

#ifdef U2HTS_ENABLE_FREERTOS
  u2hts_log_print_mutex =
      xSemaphoreCreateMutexStatic(&u2hts_log_print_mutex_buf);

  u2hts_touch_task_handle = xTaskCreateStatic(
      u2hts_touch_task, "u2hts_touch_task", U2HTS_TOUCH_TASK_STACK_SIZE, NULL,
      U2HTS_TOUCH_TASK_PRIORITY, u2hts_touch_task_stack, &u2hts_touch_tcb);
  xTaskCreateStatic(u2hts_tps_release_task, "u2hts_tps_release_task",
                    U2HTS_TPS_RELEASE_TASK_STACK_SIZE, NULL,
                    U2HTS_TPS_RELEASE_TASK_PRIORITY,
                    u2hts_tps_release_task_stack, &u2hts_tps_release_tcb);
#ifdef U2HTS_ENABLE_KEY
  xTaskCreateStatic(u2hts_key_task, "u2hts_key_task", U2HTS_KEY_TASK_STACK_SIZE,
                    NULL, U2HTS_KEY_TASK_PRIORITY, u2hts_key_task_stack,
                    &u2hts_key_tcb);
#endif
#endif
  ULOGD(LOG_TAG, "Exit %s, u2hts_error = %d", __func__, u2hts_error);
  return u2hts_error;
}

void u2hts_task() {
  if (U2HTS_GET_CONFIG_MODE_FLAG())
    u2hts_handle_config();
  else {
    if (u2hts_usrkey_get())
      U2HTS_SET_CONFIG_MODE_FLAG(u2hts_get_key_timeout(1000));
    else {
      if (!touch_controller->report_mode && U2HTS_GET_TPS_REMAIN_FLAG()) {
        if ((uint16_t)(u2hts_get_timestamp() - u2hts_last_report_ts) >
                U2HTS_TPS_RELEASE_TIMEOUT &&
            u2hts_get_usb_status()) {
          ULOGD(LOG_TAG, "releasing remain tps");
          u2hts_release_tps();
        }
      }

      u2hts_int_set(!config->polling_mode && u2hts_get_usb_status());

      u2hts_led_set(!u2hts_get_usb_status());

      if ((config->polling_mode ? true : U2HTS_GET_INT_FLAG()) &&
          u2hts_get_usb_status())
        u2hts_handle_touch();
    }
  }
}