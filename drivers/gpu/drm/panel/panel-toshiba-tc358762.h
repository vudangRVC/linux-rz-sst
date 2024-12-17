/* SPDX-License-Identifier: GPL-2.0 */
#ifndef PANEL_TOSHIBA_TC358762_H
#define PANEL_TOSHIBA_TC358762_H

#include <linux/backlight.h>

extern struct backlight_device *rzsbc_mcu_get_backlightdev(void);
extern int rzsbc_mcu_set_bright(int bright);
extern int rzsbc_mcu_screen_power_up(void);
extern int rzsbc_mcu_screen_power_off(void);
extern void rzsbc_ft5406_start_polling(void);
extern int rzsbc_mcu_is_connected(void);

int tc358762_dsi_probe(struct mipi_dsi_device *dsi);
void tc358762_dsi_remove(struct mipi_dsi_device *dsi);
void tc358762_dsi_shutdown(struct mipi_dsi_device *dsi);

#endif
