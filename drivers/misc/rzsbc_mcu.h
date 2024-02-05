/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _RZSBC_MCU_H_
#define _RZSBC_MCU_H_

#define LOG_INFO(fmt, arg...) pr_info("rzsbc-mcu: %s: "fmt, __func__, ##arg)
#define LOG_ERR(fmt, arg...) pr_err("rzsbc-mcu: %s: "fmt, __func__, ##arg)

#define MAX_I2C_LEN 255

struct rzsbc_mcu_data {
	struct device *dev;
	struct i2c_client *client;
};

struct backlight_device *rzsbc_mcu_get_backlightdev(void);

int rzsbc_mcu_screen_power_off(void);
int rzsbc_mcu_screen_power_up(void);
int rzsbc_mcu_set_bright(int bright);
int rzsbc_mcu_get_brightness(void);
int rzsbc_mcu_bl_update_status(struct backlight_device *bd);
int rzsbc_mcu_is_connected(void);

#endif
