// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright (C) 2021-2022 Samuel Holland <samuel@sholland.org>

#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input/matrix_keypad.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

static bool disable_input;
module_param(disable_input, bool, S_IRUGO);
MODULE_PARM_DESC(disable_input, "Disable the keyboard part of the driver");

#define DRV_NAME			"pinephone-keyboard"

#define PPKB_CRC8_POLYNOMIAL		0x07

#define PPKB_DEVICE_ID_HI		0x00
#define PPKB_DEVICE_ID_HI_VALUE			0x4b
#define PPKB_DEVICE_ID_LO		0x01
#define PPKB_DEVICE_ID_LO_VALUE			0x42
#define PPKB_FW_REVISION		0x02
#define PPKB_FW_FEATURES		0x03
#define PPKB_MATRIX_SIZE		0x06
#define PPKB_SCAN_CRC			0x07
#define PPKB_SCAN_DATA			0x08
#define PPKB_SYS_CONFIG			0x20
#define PPKB_SYS_CONFIG_DISABLE_SCAN		BIT(0)
#define PPKB_SYS_SMBUS_COMMAND		0x21
#define PPKB_SYS_SMBUS_DATA		0x22
#define PPKB_SYS_COMMAND		0x23
#define PPKB_SYS_COMMAND_SMBUS_READ		0x91
#define PPKB_SYS_COMMAND_SMBUS_WRITE		0xa1

#define PPKB_DEFAULT_KEYMAP_ROWS	6
#define PPKB_DEFAULT_KEYMAP_COLS	12

static const uint32_t ppkb_default_fn_keymap[] = {
	KEY(0,  0, KEY_FN_ESC),
	KEY(0,  1, KEY_F1),
	KEY(0,  2, KEY_F2),
	KEY(0,  3, KEY_F3),
	KEY(0,  4, KEY_F4),
	KEY(0,  5, KEY_F5),
	KEY(0,  6, KEY_F6),
	KEY(0,  7, KEY_F7),
	KEY(0,  8, KEY_F8),
	KEY(0,  9, KEY_F9),
	KEY(0, 10, KEY_F10),
	KEY(0, 11, KEY_DELETE),

	KEY(2,  0, KEY_SYSRQ),
	KEY(2, 10, KEY_INSERT),

	KEY(3,  0, KEY_LEFTSHIFT),
	KEY(3,  8, KEY_HOME),
	KEY(3,  9, KEY_UP),
	KEY(3, 10, KEY_END),

	KEY(4, 1, KEY_LEFTCTRL),
	KEY(4, 6, KEY_LEFT),
	KEY(4, 8, KEY_RIGHT),
	KEY(4, 9, KEY_DOWN),

	KEY(5, 2, KEY_FN),
	KEY(5, 3, KEY_LEFTALT),
	KEY(5, 5, KEY_RIGHTALT),
};

static const struct matrix_keymap_data ppkb_default_fn_keymap_data = {
	.keymap		= ppkb_default_fn_keymap,
	.keymap_size	= ARRAY_SIZE(ppkb_default_fn_keymap),
};

static const uint32_t ppkb_default_keymap[] = {
	KEY(0,  0, KEY_ESC),
	KEY(0,  1, KEY_1),
	KEY(0,  2, KEY_2),
	KEY(0,  3, KEY_3),
	KEY(0,  4, KEY_4),
	KEY(0,  5, KEY_5),
	KEY(0,  6, KEY_6),
	KEY(0,  7, KEY_7),
	KEY(0,  8, KEY_8),
	KEY(0,  9, KEY_9),
	KEY(0, 10, KEY_0),
	KEY(0, 11, KEY_BACKSPACE),

	KEY(1,  0, KEY_TAB),
	KEY(1,  1, KEY_Q),
	KEY(1,  2, KEY_W),
	KEY(1,  3, KEY_E),
	KEY(1,  4, KEY_R),
	KEY(1,  5, KEY_T),
	KEY(1,  6, KEY_Y),
	KEY(1,  7, KEY_U),
	KEY(1,  8, KEY_I),
	KEY(1,  9, KEY_O),
	KEY(1, 10, KEY_P),
	KEY(1, 11, KEY_ENTER),

	KEY(2,  0, KEY_LEFTMETA),
	KEY(2,  1, KEY_A),
	KEY(2,  2, KEY_S),
	KEY(2,  3, KEY_D),
	KEY(2,  4, KEY_F),
	KEY(2,  5, KEY_G),
	KEY(2,  6, KEY_H),
	KEY(2,  7, KEY_J),
	KEY(2,  8, KEY_K),
	KEY(2,  9, KEY_L),
	KEY(2, 10, KEY_SEMICOLON),

	KEY(3,  0, KEY_LEFTSHIFT),
	KEY(3,  1, KEY_Z),
	KEY(3,  2, KEY_X),
	KEY(3,  3, KEY_C),
	KEY(3,  4, KEY_V),
	KEY(3,  5, KEY_B),
	KEY(3,  6, KEY_N),
	KEY(3,  7, KEY_M),
	KEY(3,  8, KEY_COMMA),
	KEY(3,  9, KEY_DOT),
	KEY(3, 10, KEY_SLASH),

	KEY(4,  1, KEY_LEFTCTRL),
	KEY(4,  4, KEY_SPACE),
	KEY(4,  6, KEY_APOSTROPHE),
	KEY(4,  8, KEY_RIGHTBRACE),
	KEY(4,  9, KEY_LEFTBRACE),

	KEY(5,  2, KEY_FN),
	KEY(5,  3, KEY_LEFTALT),
	KEY(5,  5, KEY_RIGHTALT),
};

static const struct matrix_keymap_data ppkb_default_keymap_data = {
	.keymap		= ppkb_default_keymap,
	.keymap_size	= ARRAY_SIZE(ppkb_default_keymap),
};

struct pinephone_keyboard {
	struct i2c_adapter adapter;
	struct regulator *vbat_supply;
	struct input_dev *input;
	unsigned short *fn_keymap;
	u8 crc_table[CRC8_TABLE_SIZE];
	u8 row_shift;
	u8 rows;
	u8 cols;
	u8 fn_state;
	u8 buf_swap;
	u8 buf[];
};

static int ppkb_adap_smbus_xfer(struct i2c_adapter *adap, u16 addr,
				unsigned short flags, char read_write,
				u8 command, int size,
				union i2c_smbus_data *data)
{
	struct i2c_client *client = adap->algo_data;
	u8 buf[3];
	int ret;

	buf[0] = command;
	buf[1] = data->byte;
	buf[2] = read_write == I2C_SMBUS_READ ? PPKB_SYS_COMMAND_SMBUS_READ
					      : PPKB_SYS_COMMAND_SMBUS_WRITE;

	ret = i2c_smbus_write_i2c_block_data(client, PPKB_SYS_SMBUS_COMMAND,
					     sizeof(buf), buf);
	if (ret)
		return ret;

	/* Read back the command status until it passes or fails. */
	do {
		usleep_range(300, 500);
		ret = i2c_smbus_read_byte_data(client, PPKB_SYS_COMMAND);
	} while (ret == buf[2]);
	if (ret < 0)
		return ret;
	/* Commands return 0x00 on success and 0xff on failure. */
	if (ret)
		return -EIO;

	if (read_write == I2C_SMBUS_READ) {
		ret = i2c_smbus_read_byte_data(client, PPKB_SYS_SMBUS_DATA);
		if (ret < 0)
			return ret;

		data->byte = ret;
	}

	return 0;
}

static u32 ppkg_adap_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_BYTE_DATA;
}

static const struct i2c_algorithm ppkb_adap_algo = {
	.smbus_xfer		= ppkb_adap_smbus_xfer,
	.functionality		= ppkg_adap_functionality,
};

static int ppkb_set_scan(struct i2c_client *client, bool enable)
{
	struct device *dev = &client->dev;
	int ret, val;

	ret = i2c_smbus_read_byte_data(client, PPKB_SYS_CONFIG);
	if (ret < 0) {
		dev_err(dev, "Failed to read config: %d\n", ret);
		return ret;
	}

	if (enable)
		val = ret & ~PPKB_SYS_CONFIG_DISABLE_SCAN;
	else
		val = ret | PPKB_SYS_CONFIG_DISABLE_SCAN;
	ret = i2c_smbus_write_byte_data(client, PPKB_SYS_CONFIG, val);
	if (ret) {
		dev_err(dev, "Failed to write config: %d\n", ret);
		return ret;
	}

	return 0;
}

static void ppkb_update(struct i2c_client *client)
{
	struct pinephone_keyboard *ppkb = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	size_t buf_len = ppkb->cols + 1;
	u8 *old_buf = ppkb->buf;
	u8 *new_buf = ppkb->buf;
	unsigned short *keymap;
	int col, crc, ret, row;

	if (ppkb->buf_swap)
		old_buf += buf_len;
	else
		new_buf += buf_len;

	ret = i2c_smbus_read_i2c_block_data(client, PPKB_SCAN_CRC,
					    buf_len, new_buf);
	if (ret != buf_len) {
		dev_err(dev, "Failed to read scan data: %d\n", ret);
		return;
	}

	crc = crc8(ppkb->crc_table, new_buf + 1, ppkb->cols, CRC8_INIT_VALUE);
	if (crc != new_buf[0]) {
		dev_err(dev, "Bad scan data (%02x != %02x)\n", crc, new_buf[0]);
		return;
	}

	ppkb->buf_swap = !ppkb->buf_swap;

	keymap = ppkb->fn_state ? ppkb->fn_keymap : ppkb->input->keycode;
	for (col = 0; col < ppkb->cols; ++col) {
		u8 old = *(++old_buf);
		u8 new = *(++new_buf);
		u8 changed = old ^ new;

		for (row = 0; row < ppkb->rows; ++row) {
			int code = MATRIX_SCAN_CODE(row, col, ppkb->row_shift);
			int value = new & BIT(row);

			if (!(changed & BIT(row)))
				continue;

			dev_dbg(dev, "row %u col %u %sed\n",
				row, col, value ? "press" : "releas");
			if (keymap[code] == KEY_FN) {
				dev_dbg(dev, "FN is now %sed\n",
					value ? "press" : "releas");
				keymap = value ? ppkb->fn_keymap
					       : ppkb->input->keycode;
				ppkb->fn_state = value;
			}
			input_event(ppkb->input, EV_MSC, MSC_SCAN, code);
			input_report_key(ppkb->input, keymap[code], value);
		}
	}
	input_sync(ppkb->input);
}

static int ppkb_open(struct input_dev *input)
{
	struct i2c_client *client = input_get_drvdata(input);
	int ret;

	ret = ppkb_set_scan(client, true);
	if (ret)
		return ret;

	ppkb_update(client);

	return 0;
}

static void ppkb_close(struct input_dev *input)
{
	struct i2c_client *client = input_get_drvdata(input);

	ppkb_set_scan(client, false);
}

static irqreturn_t ppkb_irq_thread(int irq, void *data)
{
	struct i2c_client *client = data;

	ppkb_update(client);

	return IRQ_HANDLED;
}

static int ppkb_probe(struct i2c_client *client)
{
	const struct matrix_keymap_data *fn_keymap_data = &ppkb_default_fn_keymap_data;
	const struct matrix_keymap_data *keymap_data = &ppkb_default_keymap_data;
	struct device *dev = &client->dev;
	unsigned int phys_rows, phys_cols;
	unsigned int map_rows, map_cols;
	struct pinephone_keyboard *ppkb;
	u8 info[PPKB_MATRIX_SIZE + 1];
	struct device_node *i2c_bus;
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, 0, sizeof(info), info);
	if (ret != sizeof(info))
		return dev_err_probe(dev, ret, "Failed to read device ID\n");

	if (info[PPKB_DEVICE_ID_HI] != PPKB_DEVICE_ID_HI_VALUE ||
	    info[PPKB_DEVICE_ID_LO] != PPKB_DEVICE_ID_LO_VALUE)
		return dev_err_probe(dev, -ENODEV, "Unexpected device ID\n");

	dev_info(dev, "Found keyboard firmware version %d.%d features %#x\n",
		 info[PPKB_FW_REVISION] >> 4,
		 info[PPKB_FW_REVISION] & 0xf,
		 info[PPKB_FW_FEATURES]);

	/* Disable scan by default to save power. */
	ret = ppkb_set_scan(client, false);
	if (ret)
		return ret;

	/* Allow the devicetree to override the default keymaps. */
	if (of_property_read_bool(dev->of_node, "linux,fn-keymap") ||
	    of_property_read_bool(dev->of_node, "linux,keymap")) {
		ret = matrix_keypad_parse_properties(dev, &map_rows, &map_cols);
		if (ret)
			return ret;

		fn_keymap_data = keymap_data = NULL;
	} else {
		map_rows = PPKB_DEFAULT_KEYMAP_ROWS;
		map_cols = PPKB_DEFAULT_KEYMAP_COLS;
	}

	phys_rows = info[PPKB_MATRIX_SIZE] & 0xf;
	phys_cols = info[PPKB_MATRIX_SIZE] >> 4;
	if (map_rows != phys_rows || map_cols != phys_cols)
		return dev_err_probe(dev, -EINVAL,
				     "Keyboard size is %ux%u, but keymap is %ux%u\n",
				     phys_rows, phys_cols, map_rows, map_cols);

	/* Allocate two buffers, and include space for reading the CRC. */
	ppkb = devm_kzalloc(dev, struct_size(ppkb, buf, 2 * (phys_cols + 1)), GFP_KERNEL);
	if (!ppkb)
		return -ENOMEM;

	i2c_set_clientdata(client, ppkb);

	ppkb->vbat_supply = devm_regulator_get(dev, "vbat");
	if (IS_ERR(ppkb->vbat_supply))
		return dev_err_probe(dev, PTR_ERR(ppkb->vbat_supply),
				     "Failed to get vbat_supply\n");

	i2c_bus = of_get_child_by_name(dev->of_node, "i2c-bus");
	if (i2c_bus) {
		ppkb->adapter.owner = THIS_MODULE;
		ppkb->adapter.algo = &ppkb_adap_algo;
		ppkb->adapter.algo_data = client;
		ppkb->adapter.dev.parent = dev;
		ppkb->adapter.dev.of_node = i2c_bus;
		strscpy(ppkb->adapter.name, DRV_NAME, sizeof(ppkb->adapter.name));

		ret = devm_i2c_add_adapter(dev, &ppkb->adapter);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to add I2C adapter\n");
	}

	crc8_populate_msb(ppkb->crc_table, PPKB_CRC8_POLYNOMIAL);
	ppkb->row_shift = get_count_order(map_cols);
	ppkb->rows = map_rows;
	ppkb->cols = map_cols;

	if (disable_input)
		goto enable_regulator;

	ppkb->input = devm_input_allocate_device(dev);
	if (!ppkb->input)
		return -ENOMEM;

	input_set_drvdata(ppkb->input, client);

	ppkb->input->name = "PinePhone Keyboard";
	ppkb->input->phys = DRV_NAME "/input0";
	ppkb->input->id.bustype = BUS_I2C;
	ppkb->input->open = ppkb_open;
	ppkb->input->close = ppkb_close;

	__set_bit(EV_MSC, ppkb->input->evbit);
	__set_bit(EV_REP, ppkb->input->evbit);

	ret = matrix_keypad_build_keymap(fn_keymap_data, "linux,fn-keymap",
					 map_rows, map_cols, NULL, ppkb->input);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to build FN keymap\n");

	ppkb->fn_keymap = ppkb->input->keycode;

	ret = matrix_keypad_build_keymap(keymap_data, "linux,keymap",
					 map_rows, map_cols, NULL, ppkb->input);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to build keymap\n");

	ret = input_register_device(ppkb->input);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register input\n");

	ret = devm_request_threaded_irq(dev, client->irq, NULL, ppkb_irq_thread,
					IRQF_ONESHOT, client->name, client);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request IRQ\n");

enable_regulator:
	ret = regulator_enable(ppkb->vbat_supply);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable keyboard vbat supply\n");

	return 0;
}

static int ppkb_remove(struct i2c_client *client)
{
	struct pinephone_keyboard *ppkb = i2c_get_clientdata(client);

	regulator_disable(ppkb->vbat_supply);

	return 0;
}

static const struct of_device_id ppkb_of_match[] = {
	{ .compatible = "pine64,pinephone-keyboard" },
	{ }
};
MODULE_DEVICE_TABLE(of, ppkb_of_match);

static struct i2c_driver ppkb_driver = {
	.probe_new	= ppkb_probe,
	.remove		= ppkb_remove,
	.driver		= {
		.name		= DRV_NAME,
		.of_match_table = ppkb_of_match,
	},
};
module_i2c_driver(ppkb_driver);

MODULE_AUTHOR("Samuel Holland <samuel@sholland.org>");
MODULE_DESCRIPTION("Pine64 PinePhone keyboard driver");
MODULE_LICENSE("GPL");
