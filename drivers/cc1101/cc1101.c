/*
 * SPDX-FileCopyrightText: Copyright 2023 Andreas Sandberg <andreas@sandberg.uk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_cc1101

#include <radio/cc1101.h>

#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cc1101, CONFIG_RFCH_RADIO_LOG_LEVEL);

#define CC1101_GDO_INV 0x40

/* #define CC1101_GDO_END_OF_PACKET 0x01 */
#define CC1101_GDO_END_OF_PACKET (CC1101_GDO_INV | 0x06)
#define CC1101_GDO_RX_FIFO_THRESHOLD 0x01
#define CC1101_GDO_CLEAR_CHANNEL 0x09
#define CC1101_GDO_HIGH_IMPEDANCE 0x2E

struct cc1101_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec gpio_gdo0;
	struct gpio_dt_spec gpio_gdo2;

	const uint8_t *rf_cfg;
	const size_t rf_cfg_len;

	const uint8_t *patable;
	const size_t patable_len;

	const uint8_t pkt_len;
};

struct cc1101_data {
	const struct device *dev;

	struct k_mutex mutex;
	enum cc1101_state state;

	struct gpio_callback gdo0_irq_callback;
	struct k_work gdo0_irq_work;

	uint8_t *pkt_buf;
	uint8_t *pkt_buf_cur;
	uint8_t pkt_len;

	cc1101_recv_callback_t cb_recv;
	void *cb_recv_user;

	struct k_sem tx_sem;
	int tx_repeat;
};

static int cc1101_transceive(const struct device *dev,
			     uint8_t reg,
			     const uint8_t *data_tx,
			     uint8_t *status,
			     uint8_t *data_rx,
			     size_t len)
{
	const struct cc1101_config *config = dev->config;
	uint8_t _status;
	const struct spi_buf tx_bufs[] = {
		{ .buf = &reg, .len = 1 },
		{ .buf = (uint8_t *)data_tx, .len = len },
	};
	const struct spi_buf rx_bufs[] = {
		{ .buf = &_status, .len = 1 },
		{ .buf = data_rx, .len = len },
	};
	const struct spi_buf_set tx_buf_set = {
		.buffers = tx_bufs,
		.count = data_tx ? 2 : 2,
	};
	const struct spi_buf_set rx_buf_set = {
		.buffers = rx_bufs,
		.count = data_rx ? 2 : 2,
	};
	int err;

	/* LOG_DBG("transceive: reg: 0x%" PRIx8, reg); */
	err = spi_transceive_dt(&config->bus, &tx_buf_set, &rx_buf_set);
	if (err < 0) {
		LOG_DBG("transceive failed: reg: 0x%" PRIx8 " err: %d",
			reg, err);
		goto spi_out;
	}

	/* LOG_DBG("status: 0x%" PRIx8, _status); */
	if (_status & 0x80) {
		LOG_ERR("CC1101 didn't wake up.");
		err = -ENODEV;
		goto spi_out;
	}

	if (status) {
		*status = _status;
	}

spi_out:
	gpio_pin_set_dt(&config->bus.config.cs.gpio, 0);

	return err;
}

int cc1101_read_reg(const struct device *dev, uint8_t reg)
{
	int err;
	uint8_t value;

	err = cc1101_transceive(dev, CC1101_REG_READ | reg,
				NULL, NULL, &value, sizeof(value));
	if (err < 0) {
		return err;
	}

	return value;
}

int cc1101_write_reg(const struct device *dev, uint8_t reg, uint8_t value)
{
	int err;

	err = cc1101_transceive(dev, CC1101_REG_WRITE | reg,
				&value, NULL, NULL, sizeof(value));
	if (err < 0) {
		return err;
	}

	return 0;
}

int cc1101_cmd(const struct device *dev, uint8_t cmd)
{
	const struct cc1101_config *config = dev->config;
	uint8_t _status;
	const struct spi_buf tx_bufs[] = {
		{ .buf = &cmd, .len = 1 },
	};
	const struct spi_buf rx_bufs[] = {
		{ .buf = &_status, .len = 1 },
	};
	const struct spi_buf_set tx_buf_set = {
		.buffers = tx_bufs,
		.count = 1,
	};
	const struct spi_buf_set rx_buf_set = {
		.buffers = rx_bufs,
		.count = 1,
	};
	int err;

	cmd |= CC1101_REG_READ;
	LOG_DBG("transceive: cmd: 0x%" PRIx8, cmd);
	err = spi_transceive_dt(&config->bus, &tx_buf_set, &rx_buf_set);
	if (err < 0) {
		goto spi_out;
	}

	LOG_DBG("status: 0x%" PRIx8, _status);
	if (_status & 0x80) {
		LOG_ERR("CC1101 didn't wake up.");
		err = -ENODEV;
		goto spi_out;
	}

	return _status;

spi_out:
	return err;
}

static int _cc1101_set_state(const struct device *dev, enum cc1101_state state)
{
	struct cc1101_data *data = dev->data;
	int ret;

	switch (state) {
	case CC1101_STATE_IDLE:
		ret = cc1101_cmd(dev, CC1101_CMD_SIDLE);
		break;

	case CC1101_STATE_RX:
		data->pkt_buf_cur = data->pkt_buf;
		ret = cc1101_write_reg(dev, CC1101_REG_IOCFG0,
				       CC1101_GDO_RX_FIFO_THRESHOLD);
		if (ret < 0) {
			LOG_ERR("Failed to configure GDO0: %d", ret);
			break;
		}
		ret = cc1101_cmd(dev, CC1101_CMD_SRX);
		break;

	case CC1101_STATE_TX:
		ret = cc1101_cmd(dev, CC1101_CMD_STX);
		break;

	case CC1101_STATE_FSTXON:
		ret = cc1101_cmd(dev, CC1101_CMD_SFSTXON);
		break;

	default:
		return -EINVAL;
	}

	if (ret >= 0) {
		data->state = state;
	}

	return ret;
}

static int cc1101_write(const struct device *dev, uint8_t reg,
			const uint8_t *data, size_t len, uint8_t *status)
{
	uint8_t cmd = CC1101_REG_WRITE | (len > 1 ? CC1101_REG_BURST : 0) | reg;
	return cc1101_transceive(dev, cmd, data, status, NULL, len);
}

static int cc1101_read(const struct device *dev, uint8_t reg,
		       uint8_t *data, size_t len, uint8_t *status)
{
	uint8_t cmd = CC1101_REG_READ | (len > 1 ? CC1101_REG_BURST : 0) | reg;
	return cc1101_transceive(dev, cmd, NULL, status, data, len);
}

static void _cc1101_restart_rx(const struct device *dev)
{
	struct cc1101_data *data = dev->data;
	data->pkt_buf_cur = data->pkt_buf;

	/* Flush the RX FIFO and re-enter RX */
	if (cc1101_cmd(dev, CC1101_CMD_SIDLE) < 0 ||
	    cc1101_cmd(dev, CC1101_CMD_SFRX) < 0 ||
	    cc1101_cmd(dev, CC1101_CMD_SRX) < 0) {
		LOG_ERR("Failed to flush RX FIFO and return to RX.");
	}
}

static int _cc1101_rx_packet_done(const struct device *dev, uint8_t status,
				  int fifo_level)
{
	struct cc1101_data *data = dev->data;
	data->pkt_buf_cur = data->pkt_buf;

	if (CC1101_STATUS_STATE(status) == CC1101_STATUS_STATE_IDLE) {
		/* This is the expected FIFO level after processing
		 * the current packet */
		if (fifo_level > 0) {
			LOG_WRN("RX FIFO contains spurious data, flushing.");
			cc1101_cmd(dev, CC1101_CMD_SFRX);
		}
		if (data->state == CC1101_STATE_RX) {
			if (cc1101_cmd(dev, CC1101_CMD_SRX) < 0) {
				LOG_ERR("Failed to enter RX.");
			}
		} else {
			data->state = CC1101_STATE_IDLE;
		}
		/* FIFO has been drained so no need to retry. Any
		 * packet that has arrived since we sent
		 * CC1101_CMD_SRX will raise a new interrupt.
		 */

		return 0;
	}

	/* There is a possibility that we have received the beginning
	 * of a new packet while reading the current one. For that
	 * reason, we need to retry since this won't generate an
	 * interrupt.
	 */
	return 1;
}


static int cc1101_gdo0_irq_work_rx_one(const struct device *dev)
{
	struct cc1101_data *data = dev->data;
	const int pkt_len = data->pkt_len + 2;
	uint8_t *pkt_buf_cur = data->pkt_buf_cur;
	const size_t bytes_received = pkt_buf_cur - data->pkt_buf;
	const size_t bytes_remaining = pkt_len - bytes_received;
	uint8_t read_len;
	uint8_t rxbytes;
	uint8_t status;
	int err;

	/* NOTE: This function is called with data->mutex locked */

	err = cc1101_read(dev, CC1101_REG_RXBYTES,
			  &rxbytes, sizeof(rxbytes), &status);
	if (err < 0) {
		LOG_ERR("Failed to read available bytes: %d", err);
		return 0;
	}

	LOG_DBG("RXBYTES: %" PRIu8 " Status: 0x%" PRIx8, rxbytes, status);
	switch (CC1101_STATUS_STATE(status)) {
	case CC1101_STATUS_STATE_IDLE:
		LOG_WRN("Radio in single-shot mode");
		/* Radio is in single-shot mode and finished receiving */
		if (rxbytes < bytes_remaining) {
			return _cc1101_rx_packet_done(dev, status, rxbytes);
		}
		break;

	case CC1101_STATUS_STATE_RX:
		/* Radio is either in continuous RX mode or a packet
		 * was discarded and it re-entered RX. */
		break;

	case CC1101_STATUS_STATE_RXFIFO_OVERFLOW:
		LOG_WRN("RX FIFO overflow");
		/* TODO: We should probably process the available
		 * packets in this case */
		LOG_WRN("RX FIFO overflow");
		_cc1101_restart_rx(dev);
		return 0;

	default:
		LOG_WRN("Unexpected radio state: %d",
			CC1101_STATUS_STATE(status));
		_cc1101_restart_rx(dev);
		return 0;
	}

	/* TODO: Support variable length packets */
	if (rxbytes >= 1 && rxbytes < bytes_remaining) {
		/* Read RXBYTES - 1 bytes from the FIFO to avoid
		 * triggering erratum */
		read_len = rxbytes - 1;
	} else if (rxbytes >= bytes_remaining) {
		read_len = bytes_remaining;
	} else {
		LOG_DBG("FIFO empty. Waiting for next interrupt.");
		return 0;
	}

	if (read_len > 0) {
		LOG_DBG("Reading %" PRIu8 " bytes from RX FIFO", read_len);
		err = cc1101_read(dev, CC1101_REG_FIFO, data->pkt_buf_cur, read_len,
				  NULL);
		if (err < 0) {
			LOG_ERR("Failed to read from RX FIFO");
			goto err_out;
		}
	}

	if (read_len < bytes_remaining) {
		LOG_DBG("Packet not ready. Retrying later.");
		data->pkt_buf_cur += read_len;
		#if 0
		/* TODO: This is a bit of a hack. We know that the
		 * packet isn't ready and there is 1 byte plus change
		 * in the 64B FIFO. At reasonable transmission rates,
		 * we (sub 500kbps) should be OK using a 1ms delay
		 * here */
		k_sleep(K_MSEC(1));
		#else
		k_yield();
		#endif
		return 1;
	}

	LOG_DBG("Packet ready. Status: 0x%" PRIx8, status);
	data->pkt_buf_cur = data->pkt_buf;

	LOG_HEXDUMP_DBG(data->pkt_buf, pkt_len, "Packet:");
	if (data->cb_recv) {
		data->cb_recv(dev, data->pkt_buf, pkt_len, data->cb_recv_user);
	}

	return _cc1101_rx_packet_done(dev, status, rxbytes - read_len);

err_out:
	_cc1101_restart_rx(dev);
	return 0;
}

static void cc1101_gdo0_irq_work_tx(const struct device *dev)
{
	struct cc1101_data *data = dev->data;
	int ret;
	uint8_t status;

	/* NOTE: This function is called with data->mutex locked */

	LOG_DBG("Packet sent");

	LOG_DBG("Re-transmissions left: %d", data->tx_repeat);
	if (data->tx_repeat > 0) {
		ret = cc1101_write(dev, CC1101_REG_FIFO,
				   data->pkt_buf, data->pkt_len, &status);
		if (ret < 0) {
			LOG_ERR("Failed to write packet to TX fifo: %d", ret);
			goto err_out;
		}
	} else {
		ret = cc1101_cmd(dev, CC1101_CMD_SNOP);
		if (ret < 0) {
			LOG_ERR("Failed to get radio status: %d", status);
			goto err_out;
		}
		status = (uint8_t)ret;
	}

	LOG_DBG("Status: 0x%" PRIx8, status);
	switch (CC1101_STATUS_STATE(status)) {
	case CC1101_STATUS_STATE_IDLE:
		data->state = CC1101_STATE_IDLE;
		if (data->tx_repeat > 0) {
			_cc1101_set_state(dev, CC1101_STATE_TX);
		}
		break;

	case CC1101_STATUS_STATE_FSTXON:
		data->state = CC1101_STATE_FSTXON;
		if (data->tx_repeat > 0) {
			_cc1101_set_state(dev, CC1101_STATE_TX);
		} else {
			_cc1101_set_state(dev, CC1101_STATE_IDLE);
		}
		break;

	case CC1101_STATUS_STATE_TX:
		if (data->tx_repeat <= 0) {
			_cc1101_set_state(dev, CC1101_STATE_IDLE);
		}
		break;

	case CC1101_STATUS_STATE_CALIBRATE:
	case CC1101_STATUS_STATE_SETTLING:
		LOG_DBG("Radio calibration in progress, retrying...");
		return;

	case CC1101_STATUS_STATE_TXFIFO_UNDERFLOW:
	default:
		/* We currently only support sending 1 packet at a
		 * time, so none of these should happen. */
		LOG_WRN("Unexpected radio state: %d",
			CC1101_STATUS_STATE(status));
		goto err_out;
	}

	if (data->tx_repeat > 0) {
		data->tx_repeat--;
	} else {
		k_sem_give(&data->tx_sem);
	}
	return;

err_out:
	_cc1101_set_state(dev, CC1101_STATE_IDLE);
	k_sem_give(&data->tx_sem);
}

static void cc1101_gdo0_irq_work_handler(struct k_work *work)
{
	struct cc1101_data *data = CONTAINER_OF(work, struct cc1101_data,
						gdo0_irq_work);
	const struct device *dev = data->dev;

	LOG_DBG("CC1101 GDO0 interrupt - Packet processed");
	k_mutex_lock(&data->mutex, K_FOREVER);

	LOG_DBG("Got lock");
	if (data->state == CC1101_STATE_RX) {
		while (cc1101_gdo0_irq_work_rx_one(dev))
			;
	} else if (data->state == CC1101_STATE_TX) {
		cc1101_gdo0_irq_work_tx(dev);
	} else {
		LOG_WRN("Spurious interrupt? Unexpected state: %d",
			data->state);
	}

	k_mutex_unlock(&data->mutex);
	return;
}

static void cc1101_gdo0_irq_callback(const struct device *dev,
				     struct gpio_callback *cb, uint32_t pins)
{
	struct cc1101_data *data = CONTAINER_OF(cb, struct cc1101_data,
						gdo0_irq_callback);
	const struct cc1101_config *config = data->dev->config;

	LOG_DBG("GDOx  IRQ early: 0x%04" PRIx32, pins);
	if (pins & BIT(config->gpio_gdo0.pin)) {
		LOG_DBG("GDO0 IRQ");
		k_work_submit(&data->gdo0_irq_work);
	}
}

int cc1101_send(const struct device *dev, const uint8_t *payload, uint8_t size,
		uint8_t repeat)
{
	struct cc1101_data *data = dev->data;
	int ret;

	if (size != data->pkt_len) {
		return -EINVAL;
	}

	k_mutex_lock(&data->mutex, K_FOREVER);

	LOG_DBG("Transmitting %" PRIu8 " bytes %" PRIu8 " times",
		size, repeat + 1);

	ret = _cc1101_set_state(dev, CC1101_STATE_IDLE);
	if (ret < 0) {
		LOG_ERR("Failed to enter IDLE state: %d", ret);
		goto err_out;
	}

	if (cc1101_cmd(dev, CC1101_CMD_SFTX) < 0) {
		LOG_WRN("Failed to flush TX FIFO");
	}

	ret = cc1101_write(dev, CC1101_REG_FIFO, payload, size, NULL);
	if (ret < 0) {
		LOG_ERR("Failed to write packet to TX fifo: %d", ret);
		goto err_out;
	}

	ret = cc1101_write_reg(dev, CC1101_REG_IOCFG0,
			       CC1101_GDO_END_OF_PACKET);
	if (ret < 0) {
		LOG_ERR("Failed to configure GDO0: %d", ret);
		return ret;
	}

	data->tx_repeat = repeat;
	if (repeat) {
		memcpy(data->pkt_buf, payload, size);
		/* TODO: HACK: We don't support partial filling of the
		 * TX buf, so make a copy of the packet here */
		ret = cc1101_write(dev, CC1101_REG_FIFO, payload, size, NULL);
		if (ret < 0) {
			LOG_ERR("Failed to write packet to TX fifo: %d", ret);
		}
	}

	ret = _cc1101_set_state(dev, CC1101_STATE_TX);
	if (ret < 0) {
		LOG_ERR("Failed to enter TX state: %d", ret);
		goto err_out;
	}

	k_mutex_unlock(&data->mutex);
	k_sem_take(&data->tx_sem, K_FOREVER);
	LOG_DBG("TX done");

	return 0;

err_out:
	LOG_DBG("TX failed: %d", ret);
	_cc1101_set_state(dev, CC1101_STATE_IDLE);
	cc1101_cmd(dev, CC1101_CMD_SFTX);

	k_mutex_unlock(&data->mutex);

	return ret;
}

int cc1101_set_state(const struct device *dev, enum cc1101_state state)
{
	struct cc1101_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->mutex, K_FOREVER);

	switch (state) {
	case CC1101_STATE_IDLE:
	case CC1101_STATE_RX:
		ret = _cc1101_set_state(dev, state);
		break;

	default:
		/* TX-related states should be entered by calling
		 * cc1101_send(). */
		ret = -EINVAL;
		break;
	}

	k_mutex_unlock(&data->mutex);
	return ret;
}

int cc1101_set_recv_callback(const struct device *dev,
			     cc1101_recv_callback_t cb, void *user)
{
	struct cc1101_data *data = dev->data;

	data->cb_recv = cb;
	data->cb_recv_user = user;

	return 0;
}


static int cc1101_set_default_modem_config(const struct device *dev)
{
	const struct cc1101_config *config = dev->config;
	struct cc1101_data *data = dev->data;
	int err;

	if (config->rf_cfg_len > CC1101_REG_FIFOTHR) {
		LOG_HEXDUMP_DBG(config->rf_cfg, config->rf_cfg_len,
				"Configuration:");
		err = cc1101_write(dev,
				   CC1101_REG_FIFOTHR,
				   config->rf_cfg + CC1101_REG_FIFOTHR,
				   config->rf_cfg_len - CC1101_REG_FIFOTHR,
				   NULL);
		if (err < 0) {
			LOG_ERR("Failed to initialize CC1101: %d", err);
			return err;
		}
	}

	err = cc1101_write_reg(dev, CC1101_REG_PKTLEN, config->pkt_len);
	if (err < 0) {
		LOG_ERR("Failed to configure radio: %d", err);
		return err;
	}

	data->pkt_len = config->pkt_len;

	return 0;
}

int cc1101_set_modem_config(const struct device *dev,
			    const struct cc1101_modem_config *mc)
{
	struct cc1101_data *data = dev->data;
	int err;

	if (!mc) {
		return cc1101_set_default_modem_config(dev);
	}

	if (mc->pktlen > CONFIG_RFCH_CC1101_MAX_PKT_SIZE) {
		LOG_ERR("Requested packet exceeds configured maximum.");
		return -EINVAL;
	}

	err = cc1101_write(dev, CC1101_REG_SYNC1,
			   (const uint8_t *)mc, sizeof(*mc), NULL);
	if (err < 0) {
		LOG_ERR("Failed to configure radio: %d", err);
		return err;
	}

	data->pkt_len = mc->pktlen;

	return 0;
}

static int cc1101_reset(const struct device *dev)
{
	static const uint8_t iocfg[] = {
		CC1101_GDO_HIGH_IMPEDANCE, /* GDO2 */
		CC1101_GDO_HIGH_IMPEDANCE, /* GDO1 */
		CC1101_GDO_END_OF_PACKET, /* GDO0 */
	};
	const struct cc1101_config *config = dev->config;
	int err;

	err = cc1101_cmd(dev, CC1101_CMD_SRES);
	if (err < 0) {
		LOG_ERR("Failed to reset chip: %d", err);
		return err;
	}

	err = cc1101_write(dev, CC1101_REG_IOCFG2, iocfg, sizeof(iocfg), NULL);
	if (err < 0) {
		LOG_ERR("Failed to configure GDO pins: %d", err);
		return err;
	}

	if (config->patable_len > 0) {
		LOG_HEXDUMP_DBG(config->patable, config->patable_len,
				"PATABLE:");
		err = cc1101_write(dev,
				   CC1101_REG_PATABLE,
				   config->patable,
				   config->patable_len,
				   NULL);
		if (err < 0) {
			LOG_ERR("Failed to set PATABLE: %d", err);
			return err;
		}
	}

	return cc1101_set_modem_config(dev, NULL);
}

static int cc1101_init(const struct device *dev)
{
	const struct cc1101_config *config = dev->config;
	struct cc1101_data *data = dev->data;
	int partnum, rev, err;

	LOG_DBG("");

	data->dev = dev;
	k_mutex_init(&data->mutex);
	k_sem_init(&data->tx_sem, 0, 1);

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI bus %s not ready", config->bus.bus->name);
		return -EIO;
	}

	if (gpio_pin_configure_dt(&config->gpio_gdo0, GPIO_INPUT) ||
	    gpio_pin_configure_dt(&config->gpio_gdo2, GPIO_INPUT)) {
		LOG_ERR("GPIO configuration failed.");
		return -EIO;
	}
	k_work_init(&data->gdo0_irq_work, cc1101_gdo0_irq_work_handler);

	gpio_init_callback(&data->gdo0_irq_callback,
			   cc1101_gdo0_irq_callback, BIT(config->gpio_gdo0.pin));
	err = gpio_add_callback(config->gpio_gdo0.port,
				&data->gdo0_irq_callback);
	if (err < 0) {
		LOG_ERR("Could not set GPIO callback for GDO0: %d", err);
		return -EIO;
	}

	err = cc1101_reset(dev);
	if (err < 0) {
		LOG_ERR("Failed to reset chip: %d", err);
		return -EIO;
	}

	err = gpio_pin_interrupt_configure_dt(&config->gpio_gdo0,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (err < 0) {
		LOG_ERR("Could not configure interrupt for GDO0: %d", err);
		return -EIO;
	}

	partnum = cc1101_read_reg(dev, CC1101_REG_PARTNUM);
	rev = cc1101_read_reg(dev, CC1101_REG_VERSION);
	if (partnum < 0 || rev < 0) {
		return -EIO;
	}
	LOG_INF("CC1101 part %d rev %d", partnum, rev);

	return 0;
}

#define CC1101_INIT(i)							\
	static uint8_t rf_cfg_##i[] = DT_INST_PROP(i, rf_cfg);		\
	static uint8_t patable_##i[] = DT_INST_PROP_OR(i, patable, {});	\
	static uint8_t pkt_buf_##i[CONFIG_RFCH_CC1101_MAX_PKT_SIZE + 2]; \
									\
	BUILD_ASSERT(							\
		DT_INST_PROP(i, pktlen) <= CONFIG_RFCH_CC1101_MAX_PKT_SIZE); \
									\
	static const struct cc1101_config cc1101_config_##i = {		\
		.bus = SPI_DT_SPEC_INST_GET(i,				\
			SPI_OP_MODE_MASTER | SPI_WORD_SET(8),		\
			250),						\
		.gpio_gdo0 = GPIO_DT_SPEC_INST_GET(i, gdo0_gpios),	\
		.gpio_gdo2 = GPIO_DT_SPEC_INST_GET(i, gdo2_gpios),	\
		.rf_cfg = rf_cfg_##i,					\
		.rf_cfg_len = sizeof(rf_cfg_##i),			\
		.patable = patable_##i,					\
		.patable_len = sizeof(patable_##i),			\
		.pkt_len = DT_INST_PROP(i, pktlen),			\
	};								\
									\
	static struct cc1101_data cc1101_data_##i = {			\
		.pkt_buf = pkt_buf_##i,					\
		.pkt_buf_cur = pkt_buf_##i,				\
		.state = CC1101_STATE_IDLE,				\
	};								\
									\
	DEVICE_DT_INST_DEFINE(i, cc1101_init, NULL,			\
			      &cc1101_data_##i,				\
			      &cc1101_config_##i, POST_KERNEL,		\
			      CONFIG_RFCH_RADIO_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(CC1101_INIT)
