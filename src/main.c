/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include "includes.h"
#include "commands.h"
#include "drivers/PWM/pwm_local.h"
#include "drivers/SPIM/spim_local.h"
#include "drivers/GPIO_INPUT/gpio_input_local.h"

/*** include sensors */
// #include "sensors/IMU_CEVA_FSM300/ceva_fsm300.h"
#include "sensors/IMU_CEVA_FSM300/ceva_fsmSH2.h"

#define SLEEP_TIME_MS 1

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 6

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
static struct k_work_delayable uart_work;

struct uart_data_t
{
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#if CONFIG_BT_NUS_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
static const struct device *const async_adapter;
#endif

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type)
	{
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) ||
			(!evt->data.tx.buf))
		{
			return;
		}

		if (aborted_buf)
		{
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
							   data);
			aborted_buf = NULL;
			aborted_len = 0;
		}
		else
		{
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,
							   data);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf)
		{
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS))
		{
			LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		LOG_DBG("UART_RX_RDY");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
		buf->len += evt->data.rx.len;

		if (disable_req)
		{
			return;
		}

		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
			(evt->data.rx.buf[buf->len - 1] == '\r'))
		{
			disable_req = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		LOG_DBG("UART_RX_DISABLED");
		disable_req = false;

		buf = k_malloc(sizeof(*buf));
		if (buf)
		{
			buf->len = 0;
		}
		else
		{
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
					   UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		if (buf)
		{
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		}
		else
		{
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("UART_RX_BUF_RELEASED");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
						   data);

		if (buf->len > 0)
		{
			k_fifo_put(&fifo_uart_rx_data, buf);
		}
		else
		{
			k_free(buf);
		}

		break;

	case UART_TX_ABORTED:
		LOG_DBG("UART_TX_ABORTED");
		if (!aborted_buf)
		{
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
						   data);

		uart_tx(uart, &buf->data[aborted_len],
				buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf)
	{
		buf->len = 0;
	}
	else
	{
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
		(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}

static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	if (!device_is_ready(uart))
	{
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK))
	{
		err = usb_enable(NULL);
		if (err && (err != -EALREADY))
		{
			LOG_ERR("Failed to enable USB");
			return err;
		}
	}

	rx = k_malloc(sizeof(*rx));
	if (rx)
	{
		rx->len = 0;
	}
	else
	{
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);

	if (IS_ENABLED(CONFIG_BT_NUS_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart))
	{
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err)
	{
		k_free(rx);
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL))
	{
		LOG_INF("Wait for DTR");
		while (true)
		{
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr)
			{
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err)
		{
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err)
		{
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}

	tx = k_malloc(sizeof(*tx));

	if (tx)
	{
		pos = snprintf(tx->data, sizeof(tx->data),
					   "Starting Bluetooth LE Drone service\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data)))
		{
			k_free(rx);
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	}
	else
	{
		k_free(rx);
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err)
	{
		k_free(rx);
		k_free(tx);
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	err = uart_rx_enable(uart, rx->data, sizeof(rx->data), 50);
	if (err)
	{
		LOG_ERR("Cannot enable uart reception (err: %d)", err);
		/* Free the rx buffer only because the tx buffer will be handled in the callback */
		k_free(rx);
	}

	return err;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err)
	{
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (auth_conn)
	{
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn)
	{
		bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
							 enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err)
	{
		LOG_INF("Security changed: %s level %u", addr, level);
	}
	else
	{
		LOG_WRN("Security failed: %s level %u err %d", addr,
				level, err);
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
	LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
						  uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};
	bleISRComplete = false;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	/* free the previous feed and len and reset to clean state */
	if (ble_received_data)
	{
		k_free(ble_received_data);
		ble_received_data = NULL;
		ble_received_data_len = 0;
		commandReceived = false;
	}
	ble_received_data = k_calloc(len, sizeof(uint8_t));
	if (!ble_received_data)
	{
		printk("Not able to allocate BLE receive data buffer");
		return;
	}
	memcpy(ble_received_data, data, len);
	ble_received_data_len = len;
	commandReceived = true;

	LOG_INF("Received data from: %s", addr);

	for (uint16_t pos = 0; pos != len;)
	{
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx)
		{
			LOG_WRN("Not able to allocate UART send data buffer");
			return;
		}

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

		if ((len - pos) > tx_data_size)
		{
			tx->len = tx_data_size;
		}
		else
		{
			tx->len = (len - pos);
		}

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r'))
		{
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err)
		{
			k_fifo_put(&fifo_uart_tx_data, tx);
		}
	}
	bleISRComplete = true;
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true)
	{
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept)
	{
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	}
	else
	{
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (auth_conn)
	{
		if (buttons & KEY_PASSKEY_ACCEPT)
		{
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT)
		{
			num_comp_reply(false);
		}
	}
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

static void configure_gpio(void)
{
	int err;

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	err = dk_buttons_init(button_changed);
	if (err)
	{
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

	err = dk_leds_init();
	if (err)
	{
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}

void main(void)
{
	LOG_INF("nRF Connect SDK Fundamentals");
	int blink_status = 0;
	uint32_t period = MAX_PERIOD;
	int err = 0;
	int ret;

	configure_gpio();
	err = gpio_pin_configure_dt(&fsmbootn, GPIO_OUTPUT);
	// err = gpio_pin_configure_dt(&fsmwaken, GPIO_OUTPUT);
	printk("Set up button at %s pin %d\n", fsmbootn.port->name, fsmbootn.pin);
	if (err)
	{
		LOG_ERR("Cannot configure fsmbootn gpio");
		return err;
	}
	pwm_check_ready();

	// calibratePWM(MAX_PERIOD);
	setDuty(pwm_led0, period, period / 2U);
	setDuty(pwm_led1, period, period / 2U);
	setDuty(pwm_led2, period, period / 10U);
	setDuty(pwm_led3, period, period / 10U);

	spi_init();
	k_msleep(500);

	/* configure input GPIO to sense IMU interrupt*/
	/**********************************************/

	if (!gpio_is_ready_dt(&button))
	{
		printk("Error: button device %s is not ready\n",
			   button.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0)
	{
		printk("Error %d: failed to configure %s pin %d\n",
			   ret, button.port->name, button.pin);
		return 0;
	}

	/*
	ret = gpio_pin_interrupt_configure_dt(&button,
										  GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0)
	{
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			   ret, button.port->name, button.pin);
		return 0;
	}
	*/
	disableInts();
	gpio_init_callback(&button_cb_data, fsm_interrupt_triggered, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	if (led.port && !device_is_ready(led.port))
	{
		printk("Error %d: LED device %s is not ready; ignoring it\n",
			   ret, led.port->name);
		led.port = NULL;
	}
	if (led.port)
	{
		ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
		if (ret != 0)
		{
			printk("Error %d: failed to configure LED device %s pin %d\n",
				   ret, led.port->name, led.pin);
			led.port = NULL;
		}
		else
		{
			printk("Set up LED at %s pin %d\n", led.port->name, led.pin);
		}
	}
	/************************************************/

	err = uart_init();
	if (err)
	{
		error();
	}

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED))
	{
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err)
		{
			printk("Failed to register authorization callbacks.\n");
			return;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err)
		{
			printk("Failed to register authorization info callbacks.\n");
			return;
		}
	}

	err = bt_enable(NULL);
	if (err)
	{
		error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS))
	{
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err)
	{
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
						  ARRAY_SIZE(sd));
	if (err)
	{
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	// Initialize the demo app (creates FreeRTOS tasks for app)
	// demo_init();
	ceva_fsmSH2_Open();
	//process_command_thread();
	// FSM_init();

	for (;;)
	{
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
		// demo_service();
		/* Interfacting with sensor using SPI */
		// err = gpio_pin_toggle_dt(&spi4_cs);
		// if (err < 0)
		//{
		// return;
		//}
		// k_msleep(SLEEP_TIME_MS);
		// spi_write_test_msg();
		// err = gpio_pin_toggle_dt(&spi4_cs);
		/* end sending info using SPI */

		k_sleep(K_SECONDS(4U));
	}
}

void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);

	for (;;)
	{
		/* Wait indefinitely for data to be sent over bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
											 K_FOREVER);

		if (bt_nus_send(NULL, buf->data, buf->len))
		{
			printk("Failed to send data over BLE connection \n");
			LOG_WRN("Failed to send data over BLE connection");
		}

		k_free(buf);
		k_yield();
	}
}

K_THREAD_DEFINE(THREAD0_ID, STACKSIZE, thread0, NULL, NULL,
				NULL, PRIORITY, 0, 0);

K_THREAD_DEFINE(process_command_thread_id, STACKSIZE, process_command_thread, NULL, NULL,
				NULL, 7, 0, 0);

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
				NULL, PRIORITY, 0, 0);

K_THREAD_DEFINE(FSM_thread_id, STACKSIZE, FSM_thread, NULL, NULL,
				NULL, PRIORITY, 0, 0);
