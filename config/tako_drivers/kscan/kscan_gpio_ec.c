/*
 * Copyright (c) 2023 Sviatoslav Bulbakha
 *
 * SPDX-License-Identifier: MIT
 */

#include "debounce.h"
#include "kscan_gpio.h"

#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/logging/log.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define WAIT_DISCHARGE()
#define WAIT_CHARGE()

#define DT_DRV_COMPAT zmk_kscan_gpio_ec

#define INST_ROWS_LEN(n) DT_INST_PROP_LEN(n, row_gpios)
#define INST_MUX_SELS_LEN(n) DT_INST_PROP_LEN(n, mux_sel_gpios)
#define INST_COL_CHANNELS_LEN(n) DT_INST_PROP_LEN(n, col_channels)

#define KSCAN_GPIO_ROW_CFG_INIT(idx, inst_idx)                                 \
  KSCAN_GPIO_GET_BY_IDX(DT_DRV_INST(inst_idx), row_gpios, idx)
#define KSCAN_GPIO_MUX_SEL_CFG_INIT(idx, inst_idx)                             \
  KSCAN_GPIO_GET_BY_IDX(DT_DRV_INST(inst_idx), mux_sel_gpios, idx)

#define INST_MATRIX_LEN(n) (INST_ROWS_LEN(n) * INST_COL_CHANNELS_LEN(n))

// clang-format off
#if IS_ENABLED(CONFIG_SHIELD_TAKO_LEFT)

/* LEFT HAND */
const uint16_t actuation_threshold[] = {
    650, 650, 650, 650, 650,
    650, 650, 650, 650, 650,
    650, 650, 650, 650, 650,
    650, 650, 650, 650, 650,
};

const uint16_t release_threshold[] = {
    600, 600, 600, 600, 600,
    600, 600, 600, 600, 600,
    600, 600, 600, 600, 600,
    600, 600, 600, 600, 600,
};
#else

/* RIGHT HAND */
const uint16_t actuation_threshold[] = {
    650, 650, 650, 650, 650,
    650, 650, 650, 650, 650,
    650, 650, 650, 650, 650,
    650, 650, 650, 650, 650,
};

const uint16_t release_threshold[] = {
    600, 600, 600, 600, 600,
    600, 600, 600, 600, 600,
    600, 600, 600, 600, 600,
    600, 600, 600, 600, 600,
};
#endif
// clang-format on

struct kscan_ec_data {
  const struct device *dev;
  struct adc_sequence adc_seq;
  int16_t adc_raw;

  struct k_work work;
  struct k_timer work_timer;
  kscan_callback_t callback;
  bool *matrix_state;
};

struct kscan_ec_config {
  struct kscan_gpio_list direct;
  struct kscan_gpio_list mux_sels;
  struct kscan_gpio power;
  struct kscan_gpio mux_en;
  struct kscan_gpio discharge;
  struct adc_dt_spec adc_channel;

  size_t rows;
  size_t cols;

  int32_t poll_period_ms;
  int32_t idle_poll_period_ms;
  int32_t sleep_poll_period_ms;

  int32_t col_channels[];
};

/**
 * Get the index of a matrix state array from a row and column.
 */
static int state_index_rc(const struct kscan_ec_config *config, const int row,
                          const int col) {
  __ASSERT(row < config->rows, "Invalid row %i", row);
  __ASSERT(col < config->cols, "Invalid col %i", row);

  return (row * config->cols) + col;
}

static int kscan_ec_configure(const struct device *dev,
                              const kscan_callback_t callback) {
  LOG_DBG("KSCAN EC configure");

  struct kscan_ec_data *data = dev->data;

  if (!callback) {
    return -EINVAL;
  }

  data->callback = callback;

  return 0;
}

static int kscan_ec_enable(const struct device *dev) {
  LOG_DBG("KSCAN EC enable");

  struct kscan_ec_data *data = dev->data;
  const struct kscan_ec_config *config = dev->config;

  k_timer_start(&data->work_timer, K_MSEC(config->poll_period_ms),
                K_MSEC(config->poll_period_ms));

  return 0;
}

static int kscan_ec_disable(const struct device *dev) {
  LOG_DBG("KSCAN EC disable");

  struct kscan_ec_data *data = dev->data;
  k_timer_stop(&data->work_timer);

  return 0;
}

static void kscan_ec_timer_handler(struct k_timer *timer) {
  struct kscan_ec_data *data =
      CONTAINER_OF(timer, struct kscan_ec_data, work_timer);
  k_work_submit(&data->work);
}

static void kscan_ec_work_handler(struct k_work *work) {
  struct kscan_ec_data *data = CONTAINER_OF(work, struct kscan_ec_data, work);
  const struct kscan_ec_config *config = data->dev->config;
  struct adc_sequence *adc_seq = &data->adc_seq;

  int rc;

  int16_t matrix_read[config->rows * config->cols];

  /* Power on */
  gpio_pin_set_dt(&config->power.spec, 1);

  /* Wait for everything to power on. */
  k_sleep(K_MSEC(2));

  for (int col = 0; col < config->cols; col++) {
    uint8_t ch = config->col_channels[col];

    // Select mux
    gpio_pin_set_dt(&config->mux_en.spec, 1);
    gpio_pin_set_dt(&config->mux_sels.gpios[0].spec, ch & 1);
    gpio_pin_set_dt(&config->mux_sels.gpios[1].spec, ch & 2);
    gpio_pin_set_dt(&config->mux_sels.gpios[2].spec, ch & 4);
    gpio_pin_set_dt(&config->mux_en.spec, 0);

    for (int row = 0; row < config->rows; row++) {
      const int index = state_index_rc(config, row, col);

      gpio_pin_set_dt(&config->direct.gpios[row].spec, 0);

      // --- LOCK ---
      const unsigned int lock = irq_lock();
      gpio_pin_configure_dt(&config->discharge.spec, GPIO_INPUT);
      gpio_pin_set_dt(&config->direct.gpios[row].spec, 1);

      WAIT_CHARGE();

      rc = adc_read(config->adc_channel.dev, adc_seq);
      adc_seq->calibrate = false;

      if (rc == 0) {
        matrix_read[index] = data->adc_raw;
      } else {
        LOG_ERR("Failed to read ADC: %d", rc);
        matrix_read[index] = -1;
      }
      irq_unlock(lock);
      // -- END LOCK --

      gpio_pin_set_dt(&config->discharge.spec, 0);
      gpio_pin_configure_dt(&config->discharge.spec, GPIO_OUTPUT);
      WAIT_DISCHARGE();
    }
  }

  /* Power off */
  gpio_pin_set_dt(&config->power.spec, 0);
  gpio_pin_set_dt(&config->mux_en.spec, 0);

  for (int i = 0; i < config->direct.len; i++) {
    gpio_pin_set_dt(&config->direct.gpios[i].spec, 0);
  }

  for (int i = 0; i < config->mux_sels.len; i++) {
    gpio_pin_set_dt(&config->mux_sels.gpios[i].spec, 0);
  }

  /* Print matrix reads */
  static int cnt = 0;
  if (cnt++ >= (300 / config->poll_period_ms)) {
    cnt = 0;

    for (int r = 0; r < config->rows; r++) {
      for (int c = 0; c < config->cols; c++) {
        const int index = state_index_rc(config, r, c);
        printk("%6d", matrix_read[index]);
        if (c < config->cols - 1) {
          printk(",");
        }
      }
      printk("\n");
    }
    printk("\n\n");
  }

  /* Handle matrix reads */
  for (int r = 0; r < config->rows; r++) {
    for (int c = 0; c < config->cols; c++) {
      const int index = state_index_rc(config, r, c);
      const bool pressed = data->matrix_state[index];

      if (!pressed && matrix_read[index] > actuation_threshold[index]) {
        data->matrix_state[index] = true;
        data->callback(data->dev, r, c, true);
      } else if (pressed && matrix_read[index] < release_threshold[index]) {
        data->matrix_state[index] = false;
        data->callback(data->dev, r, c, false);
      }
    }
  }
}

static int kscan_ec_init(const struct device *dev) {
  LOG_DBG("KSCAN EC init");

  struct kscan_ec_data *data = dev->data;
  const struct kscan_ec_config *config = dev->config;

  int rc = 0;

  LOG_WRN("EC Channel: %d", config->adc_channel.channel_cfg.channel_id);
  LOG_WRN("EC Channel 2: %d", config->adc_channel.channel_id);

  gpio_pin_configure_dt(&config->power.spec, GPIO_OUTPUT_INACTIVE);

  data->dev = dev;

  data->adc_seq = (struct adc_sequence){
      .buffer = &data->adc_raw,
      .buffer_size = sizeof(data->adc_raw),
  };

  rc = adc_channel_setup_dt(&config->adc_channel);
  if (rc < 0) {
    LOG_ERR("ADC channel setup error %d", rc);
  }

  rc = adc_sequence_init_dt(&config->adc_channel, &data->adc_seq);
  if (rc < 0) {
    LOG_ERR("ADC sequence init error %d", rc);
  }

  gpio_pin_configure_dt(&config->discharge.spec, GPIO_OUTPUT_INACTIVE);

  // Init rows
  for (int i = 0; i < config->direct.len; i++) {
    gpio_pin_configure_dt(&config->direct.gpios[i].spec, GPIO_OUTPUT_INACTIVE);
  }

  // Init mux sel
  for (int i = 0; i < config->mux_sels.len; i++) {
    gpio_pin_configure_dt(&config->mux_sels.gpios[i].spec,
                          GPIO_OUTPUT_INACTIVE);
  }

  // Enable mux
  gpio_pin_configure_dt(&config->mux_en.spec, GPIO_OUTPUT_INACTIVE);

  k_timer_init(&data->work_timer, kscan_ec_timer_handler, NULL);
  k_work_init(&data->work, kscan_ec_work_handler);

  return 0;
}

static int kscan_ec_activity_event_handler(const struct device *dev,
                                           const zmk_event_t *eh) {
  struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);

  if (ev == NULL) {
    return -ENOTSUP;
  }

  struct kscan_ec_data *data = dev->data;
  const struct kscan_ec_config *config = dev->config;

  uint16_t poll_period;

  switch (ev->state) {
  case ZMK_ACTIVITY_ACTIVE:
    poll_period = config->poll_period_ms;
    break;
  case ZMK_ACTIVITY_IDLE:
    poll_period = config->idle_poll_period_ms;
    break;
  case ZMK_ACTIVITY_SLEEP:
    poll_period = config->sleep_poll_period_ms;
    break;
  default:
    LOG_WRN("Unsupported activity state: %d", ev->state);
    return -EINVAL;
  }

  LOG_DBG("Setting poll period to %dms", poll_period);
  k_timer_start(&data->work_timer, K_MSEC(poll_period), K_MSEC(poll_period));

  return 0;
}

static const struct kscan_driver_api kscan_ec_api = {
    .config = kscan_ec_configure,
    .enable_callback = kscan_ec_enable,
    .disable_callback = kscan_ec_disable};

#define KSCAN_EC_INIT(n)                                                       \
  static struct kscan_gpio kscan_ec_row_gpios_##n[] = {                        \
      LISTIFY(INST_ROWS_LEN(n), KSCAN_GPIO_ROW_CFG_INIT, (, ), n)};            \
  static struct kscan_gpio kscan_ec_mux_sel_gpios_##n[] = {                    \
      LISTIFY(INST_MUX_SELS_LEN(n), KSCAN_GPIO_MUX_SEL_CFG_INIT, (, ), n)};    \
                                                                               \
  static bool kscan_ec_matrix_state_##n[INST_MATRIX_LEN(n)];                   \
                                                                               \
  static struct kscan_ec_data kscan_ec_data_##n = {                            \
      .matrix_state = kscan_ec_matrix_state_##n,                               \
  };                                                                           \
                                                                               \
  static struct kscan_ec_config kscan_ec_config_##n = {                        \
      .direct = KSCAN_GPIO_LIST(kscan_ec_row_gpios_##n),                       \
      .mux_sels = KSCAN_GPIO_LIST(kscan_ec_mux_sel_gpios_##n),                 \
      .power = KSCAN_GPIO_GET_BY_IDX(DT_DRV_INST(n), power_gpios, 0),          \
      .mux_en = KSCAN_GPIO_GET_BY_IDX(DT_DRV_INST(n), mux_en_gpios, 0),        \
      .discharge = KSCAN_GPIO_GET_BY_IDX(DT_DRV_INST(n), discharge_gpios, 0),  \
      .poll_period_ms = DT_INST_PROP(n, poll_period_ms),                       \
      .idle_poll_period_ms = DT_INST_PROP(n, idle_poll_period_ms),             \
      .sleep_poll_period_ms = DT_INST_PROP(n, sleep_poll_period_ms),           \
      .col_channels = DT_INST_PROP(n, col_channels),                           \
      .rows = INST_ROWS_LEN(n),                                                \
      .cols = INST_COL_CHANNELS_LEN(n),                                        \
      .adc_channel = ADC_DT_SPEC_INST_GET(n),                                  \
  };                                                                           \
  static int kscan_ec_activity_event_handler_wrapper##n(                       \
      const zmk_event_t *eh) {                                                 \
    const struct device *dev = DEVICE_DT_INST_GET(n);                          \
    return kscan_ec_activity_event_handler(dev, eh);                           \
  }                                                                            \
  ZMK_LISTENER(kscan_ec##n, kscan_ec_activity_event_handler_wrapper##n);       \
  ZMK_SUBSCRIPTION(kscan_ec##n, zmk_activity_state_changed);                   \
                                                                               \
  DEVICE_DT_INST_DEFINE(n, &kscan_ec_init, NULL, &kscan_ec_data_##n,           \
                        &kscan_ec_config_##n, POST_KERNEL,                     \
                        CONFIG_KSCAN_INIT_PRIORITY, &kscan_ec_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_EC_INIT);
