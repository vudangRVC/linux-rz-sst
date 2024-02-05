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

#endif
