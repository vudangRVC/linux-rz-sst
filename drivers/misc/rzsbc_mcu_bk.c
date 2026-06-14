// SPDX-License-Identifier: GPL-2.0
/*
 *
 * RZ SBC Board Touchscreen MCU Driver.
 *
 * Copyright (C) 2024 Renesas Electronics Corporation
 * Copyright (c) 2016 ASUSTek Computer Inc.
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/backlight.h>
#include "rzsbc_mcu.h"
#include <linux/fb.h>

#define BL_DEBUG 0
static struct rzsbc_mcu_data *g_mcu_data;
static int connected;
static int lcd_bright_level;
static struct backlight_device *bl;

#define MAX_BRIGHENESS		(255)

static int is_hex(char num)
{
	//0-9, a-f, A-F
	if ((num > 47 && num < 58) || (num > 64 && num < 71) || (num > 96 && num < 103))
		return 1;
	return 0;
}

static int string_to_byte(const char *source, unsigned char *destination, int size)
{
    int i = 0, counter = 0;
    int hi, lo;

    if (size % 2 == 1)
        return -EINVAL;

    for (i = 0; i < size; i += 2) {
        if (!is_hex(source[i]) || !is_hex(source[i + 1]))
            return -EINVAL;

        hi = hex_to_bin(source[i]);
        lo = hex_to_bin(source[i + 1]);
        if (hi < 0 || lo < 0)
            return -EINVAL;

        destination[counter++] = (hi << 4) | lo;
    }

    return 0;
}
static int send_cmds(struct i2c_client *client, const char *buf)
{
    int ret, conv_ret, size = strlen(buf), retry = 5;
    int len = size / 2;
    u8 byte_cmd[MAX_I2C_LEN];

    if ((size % 2) != 0) {
        LOG_ERR("size should be even\n");
        return -EINVAL;
    }

    if (len <= 0 || len > MAX_I2C_LEN) {
        LOG_ERR("invalid command length %d\n", len);
        return -EINVAL;
    }

    conv_ret = string_to_byte(buf, byte_cmd, size);
    if (conv_ret < 0) {
        LOG_ERR("string_to_byte failed, ret=%d\n", conv_ret);
        return conv_ret;
    }

    LOG_INFO("%s\n", buf);

    while (retry-- > 0) {
        ret = i2c_master_send(client, byte_cmd, len);
        if (ret == len)
            goto ok;

        LOG_ERR("send command failed, ret = %d, retry again!\n", ret);
        msleep(20);
    }

    LOG_ERR("send command failed\n");
    return ret < 0 ? ret : -ECOMM;

ok:
    msleep(20);
    return 0;
}

static int recv_cmds(struct i2c_client *client, char *buf, int size)
{
    int ret;

    ret = i2c_master_recv(client, buf, size);
    if (ret != size) {
        LOG_ERR("receive commands failed, ret=%d expected=%d\n", ret, size);
        return ret < 0 ? ret : -ECOMM;
    }

    msleep(20);
    return 0;
}

static int init_cmd_check(struct rzsbc_mcu_data *mcu_data)
{
    int ret, retry = 5;
    char recv_buf[1] = {0};

    while (retry-- > 0) {
        ret = send_cmds(mcu_data->client, "80");
        if (ret < 0) {
            msleep(50);
            continue;
        }

        ret = recv_cmds(mcu_data->client, recv_buf, 1);
        if (ret < 0) {
            msleep(50);
            continue;
        }

        LOG_INFO("recv_cmds: 0x%X\n", recv_buf[0]);
        if (recv_buf[0] == 0xDE || recv_buf[0] == 0xC3)
            return 0;

        LOG_ERR("read wrong info: 0x%X\n", recv_buf[0]);
        ret = -EINVAL;
        msleep(50);
    }

    return ret;
}

int rzsbc_mcu_screen_power_off(void)
{
    int ret;

    if (!connected)
        return -ENODEV;

    LOG_INFO("\n");

    ret = send_cmds(g_mcu_data->client, "8500");
    if (ret < 0)
        return ret;

    usleep_range(10000, 20000);
    return 0;
}
EXPORT_SYMBOL_GPL(rzsbc_mcu_screen_power_off);

static int rzsbc_mcu_wait_ready(struct i2c_client *client, int timeout_ms)
{
    int elapsed = 0;
    int step = 50;
    int ret;
    char recv_buf[1];

    while (elapsed < timeout_ms) {
        recv_buf[0] = 0;

        ret = send_cmds(client, "80");
        if (ret == 0) {
            ret = recv_cmds(client, recv_buf, 1);
            if (ret == 0 &&
                (recv_buf[0] == 0xDE || recv_buf[0] == 0xC3)) {
                LOG_INFO("MCU ready after %d ms (0x%X)\n",
                         elapsed, recv_buf[0]);
                return 0;
            }
        }

        msleep(step);
        elapsed += step;
    }

    LOG_ERR("MCU not ready after %d ms\n", timeout_ms);
    return -ETIMEDOUT;
}

int rzsbc_mcu_screen_power_up(void)
{
    int ret, retry;

    if (!connected)
        return -ENODEV;

    LOG_INFO("\n");

    /* Step 1: send 8501 with retry */
    retry = 5;
    do {
        ret = send_cmds(g_mcu_data->client, "8501");
        if (!ret)
            break;
        msleep(100);
    } while (--retry);

    if (ret < 0)
        return ret;

    /* Wait for MCU to settle after power-on */
    ret = rzsbc_mcu_wait_ready(g_mcu_data->client, 3000);
    if (ret < 0)
        return ret;

    /* Step 2: send 8104 with retry */
    retry = 5;
    do {
        ret = send_cmds(g_mcu_data->client, "8104");
        if (!ret)
            break;
        msleep(150);
    } while (--retry);

    if (ret < 0)
        return ret;

    /* Wait for MCU to settle again before brightness writes */
    ret = rzsbc_mcu_wait_ready(g_mcu_data->client, 5000);
    if (ret < 0)
        return ret;

    return 0;
}
EXPORT_SYMBOL_GPL(rzsbc_mcu_screen_power_up);

int rzsbc_mcu_set_bright(int bright)
{
    unsigned char cmd[2];
    int ret, retry = 10;

    if (!connected)
        return -ENODEV;

    if (bright > 0xff || bright < 0)
        return -EINVAL;

    cmd[0] = 0x86;
    cmd[1] = bright;

    msleep(100);

    do {
        ret = i2c_master_send(g_mcu_data->client, cmd, 2);
        if (ret == 2)
            goto ok;

        LOG_ERR("send command failed, ret = %d\n", ret);
        msleep(100);
    } while (--retry);

    return ret < 0 ? ret : -ECOMM;

ok:
    lcd_bright_level = bright;
    return 0;
}
EXPORT_SYMBOL_GPL(rzsbc_mcu_set_bright);

int rzsbc_mcu_get_brightness(void)
{
	return lcd_bright_level;
}
EXPORT_SYMBOL_GPL(rzsbc_mcu_get_brightness);

static int rzsbc_mcu_bl_get_brightness(struct backlight_device *bd)
{
	return lcd_bright_level;
}

int rzsbc_mcu_bl_update_status(struct backlight_device *bd)
{
	int brightness = bd->props.brightness;

	if (brightness > MAX_BRIGHENESS)
		brightness = MAX_BRIGHENESS;

	if (brightness <= 0)
		brightness = 1;

	if (bd->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bd->props.state & BL_CORE_SUSPENDED)
		brightness = 0;

	LOG_INFO("%s  brightness=%d power=%d state =%d bd->props.brightness=%d\n",
					__func__, brightness, bd->props.power,
					bd->props.state,
					bd->props.brightness);
	return rzsbc_mcu_set_bright(brightness);
}

static const struct backlight_ops rzsbc_mcu_bl_ops = {
	.get_brightness	= rzsbc_mcu_bl_get_brightness, //actual_brightness_show
	.update_status	= rzsbc_mcu_bl_update_status,
	.options	= BL_CORE_SUSPENDRESUME,
};

struct backlight_device *rzsbc_mcu_get_backlightdev(void)
{
	if (!connected) {
		LOG_INFO("not ready\n");
		return NULL;
	}
	return bl;
}
EXPORT_SYMBOL_GPL(rzsbc_mcu_get_backlightdev);

static ssize_t rzsbc_mcu_bl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (BL_DEBUG)
		LOG_INFO("get bright = 0x%x\n", lcd_bright_level);

	return sprintf(buf, "%d\n", lcd_bright_level);
}

static ssize_t rzsbc_mcu_bl_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	unsigned long value;

	ret = kstrtoul(buf, 0, &value);

	if ((value > MAX_BRIGHENESS) || (ret < 0))
		LOG_ERR("Invalid value for backlight setting, value = %lu\n", value);
	else
		rzsbc_mcu_set_bright(value);

	return strnlen(buf, count);
}
static DEVICE_ATTR_RW(rzsbc_mcu_bl);

int rzsbc_mcu_is_connected(void)
{
	return connected;
}
EXPORT_SYMBOL_GPL(rzsbc_mcu_is_connected);

static int rzsbc_mcu_probe(struct i2c_client *client)
{
	struct rzsbc_mcu_data *mcu_data;
	int ret;
	struct backlight_properties props;

	LOG_INFO("address = 0x%x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		LOG_ERR("I2C check functionality failed\n");
		return -ENODEV;
	}

	mcu_data = kzalloc(sizeof(*mcu_data), GFP_KERNEL);
	if (!mcu_data)
		return -ENOMEM;

	mcu_data->client = client;
	i2c_set_clientdata(client, mcu_data);
	g_mcu_data = mcu_data;
	connected = 0;
	bl = NULL;
	lcd_bright_level = 0;

	ret = init_cmd_check(mcu_data);
	if (ret < 0) {
		LOG_ERR("init_cmd_check failed, %d\n", ret);
		goto error_free;
	}

	connected = 1;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = MAX_BRIGHENESS;

	bl = backlight_device_register("panel_backlight", NULL, NULL,
					&rzsbc_mcu_bl_ops, &props);
	if (IS_ERR(bl)) {
		ret = PTR_ERR(bl);
		bl = NULL;
		goto error_connected;
	}

	ret = device_create_file(&client->dev, &dev_attr_rzsbc_mcu_bl);
	if (ret) {
		dev_err(&client->dev, "Failed to create rzsbc_mcu_bl sysfs files %d\n", ret);
		goto error_bl;
	}

	ret = rzsbc_mcu_screen_power_off();
	if (ret < 0)
		goto error_sysfs;

	return 0;

error_sysfs:
	device_remove_file(&client->dev, &dev_attr_rzsbc_mcu_bl);
error_bl:
	backlight_device_unregister(bl);
	bl = NULL;
error_connected:
	connected = 0;
error_free:
	g_mcu_data = NULL;
	kfree(mcu_data);
	return ret;
}

static void rzsbc_mcu_remove(struct i2c_client *client)
{
	struct rzsbc_mcu_data *mcu_data = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_rzsbc_mcu_bl);

	if (bl) {
		backlight_device_unregister(bl);
		bl = NULL;
	}

	connected = 0;
	g_mcu_data = NULL;
	kfree(mcu_data);
}

static const struct i2c_device_id rzsbc_mcu_id[] = {
	{"rzsbc_mcu", 0},
	{/* sentinel */},
};

static struct i2c_driver rzsbc_mcu_driver = {
	.driver = {
		.name = "rzsbc_mcu",
	},
	.probe = rzsbc_mcu_probe,
	.remove = rzsbc_mcu_remove,
	.id_table = rzsbc_mcu_id,
};
module_i2c_driver(rzsbc_mcu_driver);

MODULE_DESCRIPTION("Tinker Board TouchScreen MCU driver");
MODULE_LICENSE("GPL");
