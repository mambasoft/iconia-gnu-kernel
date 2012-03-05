/* arch/arm/mach-tegra/board-picasso.c
 *
 * Copyright (c) 2011, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>

#include <mach/gpio.h>

#include <media/ov5650.h>
#include <media/ssl3250a.h>
#include <media/yuv_sensor.h>
#include <media/yuv5_sensor.h>

#include <generated/mach-types.h>

#include "gpio-names.h"
#include "board.h"
#include "board-picasso.h"

#define AL3000A_IRQ_GPIO	TEGRA_GPIO_PZ2
#define AKM8975_IRQ_GPIO	TEGRA_GPIO_PN5
#define CAMERA_POWER_GPIO	TEGRA_GPIO_PV4
#define CAMERA_CSI_MUX_SEL_GPIO	TEGRA_GPIO_PBB4
#define CAMERA_FLASH_ACT_GPIO	TEGRA_GPIO_PD2
#define CAMERA_FLASH_STRB_GPIO	TEGRA_GPIO_PA0

#ifdef CONFIG_VIDEO_OV5650
#define OV5650_PWDN_GPIO      TEGRA_GPIO_PL0
#define OV5650_RST_GPIO       TEGRA_GPIO_PL6
#endif
#ifdef CONFIG_VIDEO_YUV
#define YUV_SENSOR_OE_GPIO    TEGRA_GPIO_PL2
#define YUV_SENSOR_RST_GPIO   TEGRA_GPIO_PL4
#endif
#ifdef CONFIG_VIDEO_YUV5
#define YUV5_SENSOR_PWDN_GPIO TEGRA_GPIO_PL0
#define YUV5_SENSOR_RST_GPIO  TEGRA_GPIO_PL6
#endif

#define CAMERA_FLASH_OP_MODE		0 /*0=I2C mode, 1=GPIO mode*/
#define CAMERA_FLASH_MAX_LED_AMP	7
#define CAMERA_FLASH_MAX_TORCH_AMP	11
#define CAMERA_FLASH_MAX_FLASH_AMP	31

#if defined(CONFIG_MACH_PICASSO) || defined(CONFIG_MACH_ACER_MAYA)  \
	|| defined(CONFIG_MACH_ACER_VANGOGH)
struct camera_gpios {
	const char *name;
	int gpio;
};

#define CAMERA_GPIO(_name, _gpio)  \
	{                          \
		.name = _name,     \
		.gpio = _gpio,     \
	}
#endif

#ifdef CONFIG_VIDEO_OV5650
static struct camera_gpios ov5650_gpio_keys[] = {
	[0] = CAMERA_GPIO("cam_power_en", CAMERA_POWER_GPIO),
	[1] = CAMERA_GPIO("ov5650_pwdn", OV5650_PWDN_GPIO),
	[2] = CAMERA_GPIO("ov5650_rst", OV5650_RST_GPIO),
};

static int ov5650_power_on(void)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(ov5650_gpio_keys); i++) {
		tegra_gpio_enable(ov5650_gpio_keys[i].gpio);
		ret = gpio_request(ov5650_gpio_keys[i].gpio,
				ov5650_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail;
		}
	}

	gpio_direction_output(OV5650_PWDN_GPIO, 1);
	gpio_direction_output(OV5650_RST_GPIO, 1);
	msleep(1);
	gpio_direction_output(CAMERA_POWER_GPIO, 1);
	msleep(5);
	gpio_direction_output(OV5650_PWDN_GPIO, 0);
	msleep(20);
	gpio_direction_output(OV5650_RST_GPIO, 0);
	msleep(1);
	gpio_direction_output(OV5650_RST_GPIO, 1);
	msleep(20);

	return 0;

fail:
	while (i--)
		gpio_free(ov5650_gpio_keys[i].gpio);
	return ret;
}

static int ov5650_power_off(void)
{
	int i;

	gpio_direction_output(OV5650_PWDN_GPIO, 1);
	gpio_direction_output(CAMERA_POWER_GPIO, 0);
	gpio_direction_output(OV5650_PWDN_GPIO, 0);
	gpio_direction_output(OV5650_RST_GPIO, 0);

	i = ARRAY_SIZE(ov5650_gpio_keys);
	while (i--)
		gpio_free(ov5650_gpio_keys[i].gpio);

	return 0;
}

struct ov5650_platform_data ov5650_data = {
	.power_on = ov5650_power_on,
	.power_off = ov5650_power_off,
};
#endif /* CONFIG_VIDEO_OV5650 */

#ifdef CONFIG_VIDEO_YUV
static struct camera_gpios yuv_sensor_gpio_keys[] = {
	[0] = CAMERA_GPIO("cam_power_en", CAMERA_POWER_GPIO),
	[1] = CAMERA_GPIO("yuv_sensor_oe", YUV_SENSOR_OE_GPIO),
	[2] = CAMERA_GPIO("yuv_sensor_rst", YUV_SENSOR_RST_GPIO),
};

static int yuv_sensor_power_on(void)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(yuv_sensor_gpio_keys); i++) {
		tegra_gpio_enable(yuv_sensor_gpio_keys[i].gpio);
		ret = gpio_request(yuv_sensor_gpio_keys[i].gpio,
				yuv_sensor_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail;
		}
	}


	gpio_direction_output(YUV_SENSOR_OE_GPIO, 0);
	gpio_direction_output(YUV_SENSOR_RST_GPIO, 1);
	msleep(1);
	gpio_direction_output(CAMERA_POWER_GPIO, 1);
	msleep(1);
	gpio_direction_output(YUV_SENSOR_RST_GPIO, 0);
	msleep(1);
	gpio_direction_output(YUV_SENSOR_RST_GPIO, 1);

	return 0;

fail:
	while (i--)
		gpio_free(yuv_sensor_gpio_keys[i].gpio);
	return ret;
}

static int yuv_sensor_power_off(void)
{
	int i;

	gpio_direction_output(YUV_SENSOR_OE_GPIO, 1);
	gpio_direction_output(CAMERA_POWER_GPIO, 0);
	gpio_direction_output(YUV_SENSOR_OE_GPIO, 0);
	gpio_direction_output(YUV_SENSOR_RST_GPIO, 0);

	i = ARRAY_SIZE(yuv_sensor_gpio_keys);
	while (i--)
		gpio_free(yuv_sensor_gpio_keys[i].gpio);

	return 0;
}

struct yuv_sensor_platform_data yuv_sensor_data = {
	.power_on = yuv_sensor_power_on,
	.power_off = yuv_sensor_power_off,
};
#endif /* CONFIG_VIDEO_YUV */

#ifdef CONFIG_VIDEO_YUV5
static struct camera_gpios yuv5_sensor_gpio_keys[] = {
	[0] = CAMERA_GPIO("cam_power_en", CAMERA_POWER_GPIO),
	[1] = CAMERA_GPIO("yuv5_sensor_pwdn", YUV5_SENSOR_PWDN_GPIO),
	[2] = CAMERA_GPIO("yuv5_sensor_rst", YUV5_SENSOR_RST_GPIO),
};

static int yuv5_sensor_power_on(void)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(yuv5_sensor_gpio_keys); i++) {
		tegra_gpio_enable(yuv5_sensor_gpio_keys[i].gpio);
		ret = gpio_request(yuv5_sensor_gpio_keys[i].gpio,
				yuv5_sensor_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail;
		}
	}

	gpio_direction_output(YUV5_SENSOR_PWDN_GPIO, 0);
	gpio_direction_output(YUV5_SENSOR_RST_GPIO, 0);
	msleep(1);
	gpio_direction_output(CAMERA_POWER_GPIO, 1);
	msleep(1);
	gpio_direction_output(YUV5_SENSOR_RST_GPIO, 1);
	msleep(1);

	return 0;

fail:
	while (i--)
	gpio_free(yuv5_sensor_gpio_keys[i].gpio);
	return ret;
}

static int yuv5_sensor_power_off(void)
{
	int i;

	gpio_direction_output(YUV5_SENSOR_RST_GPIO, 0);
	msleep(1);
	gpio_direction_output(YUV5_SENSOR_PWDN_GPIO, 1);
	gpio_direction_output(CAMERA_POWER_GPIO, 0);
	gpio_direction_output(YUV5_SENSOR_PWDN_GPIO, 0);

	i = ARRAY_SIZE(yuv5_sensor_gpio_keys);
	while (i--)
		gpio_free(yuv5_sensor_gpio_keys[i].gpio);

	return 0;
};

struct yuv5_sensor_platform_data yuv5_sensor_data = {
	.power_on = yuv5_sensor_power_on,
	.power_off = yuv5_sensor_power_off,
};
#endif /* CONFIG_VIDEO_YUV5 */

static int picasso_camera_init(void)
{
	int i, ret;

#ifdef CONFIG_VIDEO_OV5650
	// initialize OV5650
	for (i = 0; i < ARRAY_SIZE(ov5650_gpio_keys); i++) {
		tegra_gpio_enable(ov5650_gpio_keys[i].gpio);
		ret = gpio_request(ov5650_gpio_keys[i].gpio,
				ov5650_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail1;
		}
	}

	gpio_direction_output(CAMERA_POWER_GPIO, 0);
	gpio_direction_output(OV5650_PWDN_GPIO, 0);
	gpio_direction_output(OV5650_RST_GPIO, 0);

	for (i = 0; i < ARRAY_SIZE(ov5650_gpio_keys); i++) {
		gpio_free(ov5650_gpio_keys[i].gpio);
	}
#endif

#ifdef CONFIG_VIDEO_YUV
	// initialize MT9D115
	for (i = 0; i < ARRAY_SIZE(yuv_sensor_gpio_keys); i++) {
		tegra_gpio_enable(yuv_sensor_gpio_keys[i].gpio);
		ret = gpio_request(yuv_sensor_gpio_keys[i].gpio,
				yuv_sensor_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail2;
		}
	}

	gpio_direction_output(CAMERA_POWER_GPIO, 0);
	gpio_direction_output(YUV_SENSOR_OE_GPIO, 0);
	gpio_direction_output(YUV_SENSOR_RST_GPIO, 0);

	for (i = 0; i < ARRAY_SIZE(yuv_sensor_gpio_keys); i++) {
		gpio_free(yuv_sensor_gpio_keys[i].gpio);
	}
#endif

#ifdef CONFIG_VIDEO_YUV5
	// initialize MT9P111
	for (i = 0; i < ARRAY_SIZE(yuv5_sensor_gpio_keys); i++) {
		tegra_gpio_enable(yuv5_sensor_gpio_keys[i].gpio);
		ret = gpio_request(yuv5_sensor_gpio_keys[i].gpio,
				yuv5_sensor_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail3;
		}
	}

	gpio_direction_output(CAMERA_POWER_GPIO, 0);
	gpio_direction_output(YUV5_SENSOR_PWDN_GPIO, 0);
	gpio_direction_output(YUV5_SENSOR_RST_GPIO, 0);

	for (i = 0; i < ARRAY_SIZE(yuv5_sensor_gpio_keys); i++) {
		gpio_free(yuv5_sensor_gpio_keys[i].gpio);
	}
#endif
	return 0;

#ifdef CONFIG_VIDEO_OV5650
fail1:
	while (i--)
		gpio_free(ov5650_gpio_keys[i].gpio);
	return ret;
#endif

#ifdef CONFIG_VIDEO_YUV
fail2:
        while (i--)
                gpio_free(yuv_sensor_gpio_keys[i].gpio);
	return ret;
#endif

#ifdef CONFIG_VIDEO_YUV5
fail3:
	while (i--)
		gpio_free(yuv5_sensor_gpio_keys[i].gpio);
	return ret;
#endif
}

extern void tegra_throttling_enable(bool enable);

#ifdef CONFIG_TORCH_SSL3250A
static int picasso_ssl3250a_init(void)
{
	gpio_request(CAMERA_FLASH_ACT_GPIO, "torch_gpio_act");
	gpio_direction_output(CAMERA_FLASH_ACT_GPIO, 0);
	tegra_gpio_enable(CAMERA_FLASH_ACT_GPIO);
	gpio_request(CAMERA_FLASH_STRB_GPIO, "torch_gpio_strb");
	gpio_direction_output(CAMERA_FLASH_STRB_GPIO, 0);
	tegra_gpio_enable(CAMERA_FLASH_STRB_GPIO);
	gpio_export(CAMERA_FLASH_STRB_GPIO, false);
	return 0;
}

static void picasso_ssl3250a_exit(void)
{
	gpio_set_value(CAMERA_FLASH_STRB_GPIO, 0);
	gpio_free(CAMERA_FLASH_STRB_GPIO);
	tegra_gpio_disable(CAMERA_FLASH_STRB_GPIO);
	gpio_set_value(CAMERA_FLASH_ACT_GPIO, 0);
	gpio_free(CAMERA_FLASH_ACT_GPIO);
	tegra_gpio_disable(CAMERA_FLASH_ACT_GPIO);
}

static int picasso_ssl3250a_gpio_strb(int val)
{
	int prev_val;
	prev_val = gpio_get_value(CAMERA_FLASH_STRB_GPIO);
	gpio_set_value(CAMERA_FLASH_STRB_GPIO, val);
	return prev_val;
};

static int picasso_ssl3250a_gpio_act(int val)
{
	int prev_val;
	prev_val = gpio_get_value(CAMERA_FLASH_ACT_GPIO);
	gpio_set_value(CAMERA_FLASH_ACT_GPIO, val);
	return prev_val;
};

static struct ssl3250a_platform_data picasso_ssl3250a_data = {
	.config		= CAMERA_FLASH_OP_MODE,
	.max_amp_indic	= CAMERA_FLASH_MAX_LED_AMP,
	.max_amp_torch	= CAMERA_FLASH_MAX_TORCH_AMP,
	.max_amp_flash	= CAMERA_FLASH_MAX_FLASH_AMP,
	.init		= picasso_ssl3250a_init,
	.exit		= picasso_ssl3250a_exit,
	.gpio_act	= picasso_ssl3250a_gpio_act,
	.gpio_en1	= NULL,
	.gpio_en2	= NULL,
	.gpio_strb	= picasso_ssl3250a_gpio_strb,
};
#endif /* CONFIG_TORCH_SSL3250A */

static void picasso_al3000a_init(void)
{
	tegra_gpio_enable(AL3000A_IRQ_GPIO);
	gpio_request(AL3000A_IRQ_GPIO, "al3000a_ls");
	gpio_direction_input(AL3000A_IRQ_GPIO);
}

static const struct i2c_board_info picasso_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO("al3000a_ls", 0x1C),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PZ2),
	},
};

#ifdef CONFIG_TORCH_SSL3250A
static const struct i2c_board_info picasso_i2c3_board_info_ssl3250a[] = {
	{
		I2C_BOARD_INFO("ssl3250a", 0x30),
		.platform_data = &picasso_ssl3250a_data,
	},
};
#endif /* CONFIG_TORCH_SSL3250A */

static struct i2c_board_info picasso_i2c3_board_info[] = {
#ifdef CONFIG_VIDEO_OV5650
	{
		I2C_BOARD_INFO("ov5650", 0x36),
		.platform_data = &ov5650_data,
	},
#endif
#ifdef CONFIG_VIDEO_YUV
	{
		I2C_BOARD_INFO("mt9d115", 0x3C),
		.platform_data = &yuv_sensor_data,
	},
#endif
#ifdef CONFIG_VIDEO_YUV5
	{
		I2C_BOARD_INFO("mt9p111", 0x3D),
		.platform_data = &yuv5_sensor_data,
	},
#endif
#ifdef CONFIG_VIDEO_AD5820
	{
		I2C_BOARD_INFO("ad5820", 0x0C),
	},
#endif
#ifdef CONFIG_VIDEO_LTC3216
	{
		I2C_BOARD_INFO("ltc3216", 0x33),
	},
#endif
};

void __init picasso_sensors_init(void) {

	picasso_al3000a_init();
	picasso_camera_init();

	i2c_register_board_info(0, picasso_i2c0_board_info,
		ARRAY_SIZE(picasso_i2c0_board_info));

#ifdef CONFIG_TORCH_SSL3250A
	i2c_register_board_info(3, picasso_i2c3_board_info_ssl3250a,
		ARRAY_SIZE(picasso_i2c3_board_info_ssl3250a));
#endif

	i2c_register_board_info(3, picasso_i2c3_board_info,
		ARRAY_SIZE(picasso_i2c3_board_info));

}
