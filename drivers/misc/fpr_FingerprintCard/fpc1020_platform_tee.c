// SPDX-License-Identifier: GPL-2.0-only
/*
 * FPC1020 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, controlling GPIOs such as sensor reset
 * line, sensor IRQ line.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node.
 *
 * This driver will NOT send any commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 * Copyright (C) 2021 XiaoMi, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_wakeup.h>
#include <linux/pinctrl/qcom-pinctrl.h>

#define FPC_GPIO_NO_DEFAULT -1
#define FPC_GPIO_NO_DEFINED -2
#define FPC_GPIO_REQUEST_FAIL -3

#define FPC_TTW_HOLD_TIME		2000
#define RESET_LOW_SLEEP_MIN_US		5000
#define RESET_LOW_SLEEP_MAX_US		(RESET_LOW_SLEEP_MIN_US + 100)
#define RESET_HIGH_SLEEP1_MIN_US	100
#define RESET_HIGH_SLEEP1_MAX_US	(RESET_HIGH_SLEEP1_MIN_US + 100)
#define RESET_HIGH_SLEEP2_MIN_US	5000
#define RESET_HIGH_SLEEP2_MAX_US	(RESET_HIGH_SLEEP2_MIN_US + 100)
#define PWR_ON_SLEEP_MIN_US		100
#define PWR_ON_SLEEP_MAX_US		(PWR_ON_SLEEP_MIN_US + 900)
#define NUM_PARAMS_REG_ENABLE_SET	2

#define RELEASE_WAKELOCK_W_V		"release_wakelock_with_verification"
#define RELEASE_WAKELOCK		"release_wakelock"
#define START_IRQS_RECEIVED_CNT		"start_irqs_received_counter"

static const char * const pctl_names[] = {
	"fpc1020_reset_reset",
	"fpc1020_reset_active",
};

struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
	int gpio;
};

static struct vreg_config vreg_conf[] = {
	{ "vdd_ana", 1800000UL, 1800000UL, 6000, FPC_GPIO_NO_DEFAULT },
/*
	{ "vcc_spi", 1800000UL, 1800000UL, 10, },
	{ "vdd_io", 1800000UL, 1800000UL, 6000, },
*/
};

struct fpc1020_data {
	struct device *dev;
	struct pinctrl *fingerprint_pinctrl;
	struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];
	// struct regulator *vreg[ARRAY_SIZE(vreg_conf)];
	struct wakeup_source *ttw_wl;
	struct mutex lock; /* To set/get exported values in sysfs */
	int irq_gpio;
	int rst_gpio;
	int nbr_irqs_received;
	int nbr_irqs_received_counter_start;
	bool prepared;
	atomic_t wakeup_enabled; /* Used both in ISR and non-ISR */
	bool irq_requested;
	bool gpios_requested;
	int irqf;
};

/*
 * request/release GPIO for voltage control.
 *
 */
static int reset_gpio_res(struct fpc1020_data *fpc1020);
static int request_vreg_gpio(struct fpc1020_data *fpc1020, bool enable);
static int irq_setup(struct fpc1020_data *fpc1020, bool enable);
static int vreg_setup(struct fpc1020_data *fpc1020, const char *name,
		      bool enable);
static irqreturn_t fpc1020_irq_handler(int irq, void *handle);
static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
				      const char *label, int *gpio);

static int reset_gpio_res(struct fpc1020_data *fpc1020)
{
	int rc = 0;
	struct device *dev;

	if (!fpc1020) {
		rc = -ENOMEM;
		pr_err
		    ("fpc %s: failed to allocate memory for struct fpc1020_data!\n",
		     __func__);
		return rc;
	}

	dev = fpc1020->dev;
	fpc1020->irq_gpio = FPC_GPIO_NO_DEFAULT;

	return rc;
}

static int request_vreg_gpio(struct fpc1020_data *fpc1020, bool enable)
{
	int rc = 0;
	struct device *dev = fpc1020->dev;

	dev_info(dev, "fpc %s --->: enter!\n", __func__);

	mutex_lock(&fpc1020->lock);

	if (enable && !fpc1020->gpios_requested) {
		rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_irq",
						&fpc1020->irq_gpio);
		if (rc) {
			pr_err("fpc irq gpio request failed!\n");
			goto release_irq_gpio;
		} else {
			dev_info(dev, "fpc irq gpio applied at  %d\n",
				 fpc1020->irq_gpio);
		}

		dev_info(dev, "fpc irq gpio requested successfully!\n");

		if (fpc1020->irq_requested) {
			devm_free_irq(dev, gpio_to_irq(fpc1020->irq_gpio), fpc1020);
			fpc1020->irq_requested = false;
			dev_info(dev,
				 "fpc irq has been requested already, free firstly!\n");
		}

		rc = devm_request_threaded_irq(dev,
					       gpio_to_irq(fpc1020->irq_gpio),
					       NULL, fpc1020_irq_handler,
					       fpc1020->irqf, dev_name(dev),
					       fpc1020);
		if (rc) {
			pr_err("fpc could not request irq %d\n",
			       gpio_to_irq(fpc1020->irq_gpio));

			goto release_irq_gpio;
		}

		fpc1020->irq_requested = true;
		fpc1020->gpios_requested = true;
		dev_info(dev, "fpc requested irq %d\n",
			 gpio_to_irq(fpc1020->irq_gpio));
	} else if (!enable && fpc1020->gpios_requested) {
		if (fpc1020->irq_requested) {
			devm_free_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
				      fpc1020);
			dev_info(dev, "fpc irq free successfully!\n");
			fpc1020->irq_requested = false;
		}

release_irq_gpio:
		if (gpio_is_valid(fpc1020->irq_gpio)) {
			devm_gpio_free(dev, fpc1020->irq_gpio);
			fpc1020->irq_gpio = FPC_GPIO_NO_DEFAULT;
			dev_info(dev, "fpc irq gpio released successfully!\n");
		}

		fpc1020->gpios_requested = false;
	} else {
		dev_info(dev, "%s: enable: %d, gpios_requested: %d ???\n",
			 __func__, enable, fpc1020->gpios_requested);
	}

	mutex_unlock(&fpc1020->lock);
	dev_info(dev, "fpc %s <---: exit!\n", __func__);
	return rc;
}

static int irq_setup(struct fpc1020_data *fpc1020, bool enable)
{
	int rc = 0;
	struct device *dev = fpc1020->dev;

	if (gpio_is_valid(fpc1020->irq_gpio)) {
		dev_info(dev, "fpc %s irq_gpio is validate!\n", __func__);
		if (enable) {
			enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));
			dev_info(dev, "fpc irq_gpio is enable.\n");
		} else {
			disable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));
			dev_info(dev, "fpc irq_gpio is disable.\n");
		}
	} else {
		dev_info(dev, "fpc %s irq_gpio is invalidate!\n", __func__);
		return -EINVAL;
	}

	dev_info(dev, "fpc %s <---: exit!\n", __func__);
	return rc;
}

static int vreg_setup(struct fpc1020_data *fpc1020, const char *name,
	bool enable)
{
	size_t i;
	int rc = 0;
	int gpio = 0;
	struct device *dev = fpc1020->dev;

	for (i = 0; i < ARRAY_SIZE(vreg_conf); i++) {
		const char *n = vreg_conf[i].name;

		if (!memcmp(n, name, strlen(n)))
			goto found;
	}

	dev_err(dev, "Regulator %s not found\n", name);

	return -EINVAL;

found:
/*
	vreg = fpc1020->vreg[i];
	if (enable) {
		if (!vreg) {
			vreg = devm_regulator_get(dev, name);
			if (IS_ERR_OR_NULL(vreg)) {
				dev_info(dev,
					"No regulator %s, maybe fixed regulator\n",
					name);
				return 0;
			}
		}

		if (regulator_count_voltages(vreg) > 0) {
			rc = regulator_set_voltage(vreg, vreg_conf[i].vmin,
					vreg_conf[i].vmax);
			if (rc)
				dev_err(dev,
					"Unable to set voltage on %s, %d\n",
					name, rc);
		}

		rc = regulator_set_load(vreg, vreg_conf[i].ua_load);
		if (rc < 0)
			dev_err(dev, "Unable to set current on %s, %d\n",
					name, rc);

		rc = regulator_enable(vreg);
		if (rc) {
			dev_err(dev, "error enabling %s: %d\n", name, rc);
			vreg = NULL;
		}
		fpc1020->vreg[i] = vreg;
	} else {
		if (vreg) {
			if (regulator_is_enabled(vreg)) {
				regulator_disable(vreg);
				dev_dbg(dev, "disabled %s\n", name);
			}
			fpc1020->vreg[i] = NULL;
		}
		rc = 0;
	}
*/
	gpio = vreg_conf[i].gpio;

	if(!gpio_is_valid(gpio)) {
		dev_err(dev, "fpc %s: unable to get gpio %d!\n",
			__func__, gpio);
		return 0;
	}

	rc = gpio_direction_output(gpio, enable ? 1 : 0);
	if (rc) {
		dev_err(dev, "fpc %s: failed to set gpio %d to %d!\n", __func__, gpio, enable ? 1 : 0);
		return rc;
	}

	return rc;
}

/*
 * sysfs node for release GPIO.
 *
 */
static ssize_t request_vreg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!memcmp(buf, "enable", strlen("enable")))
		rc = request_vreg_gpio(fpc1020, true);
	else if (!memcmp(buf, "disable", strlen("disable")))
		rc = request_vreg_gpio(fpc1020, false);
	else
		return -EINVAL;

	return rc ? rc : count;
}
static DEVICE_ATTR_WO(request_vreg);

/*
 * sysfs node for controlling clocks.
 *
 * This is disabled in platform variant of this driver but kept for
 * backwards compatibility. Only prints a debug print that it is
 * disabled.
 */
static ssize_t clk_enable_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	dev_dbg(dev,
		"clk_enable sysfs node not enabled in platform driver\n");

	return count;
}
static DEVICE_ATTR_WO(clk_enable);

/*
 * Will try to select the set of pins (GPIOS) defined in a pin control node of
 * the device tree named @p name.
 *
 * The node can contain several eg. GPIOs that is controlled when selecting it.
 * The node may activate or deactivate the pins it contains, the action is
 * defined in the device tree node itself and not here. The states used
 * internally is fetched at probe time.
 *
 * @see pctl_names
 * @see fpc1020_probe
 */
static int select_pin_ctl(struct fpc1020_data *fpc1020, const char *name)
{
	size_t i;
	int rc;
	struct device *dev = fpc1020->dev;

	for (i = 0; i < ARRAY_SIZE(pctl_names); i++) {
		const char *n = pctl_names[i];

		if (!memcmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(fpc1020->fingerprint_pinctrl,
					fpc1020->pinctrl_state[i]);
			if (rc)
				dev_err(dev, "cannot select '%s'\n", name);
			else
				dev_dbg(dev, "Selected '%s'\n", name);

			return rc;
		}
	}

	dev_err(dev, "%s:'%s' not found\n", __func__, name);
	return -EINVAL;
}

static ssize_t pinctl_set_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int rc;

	mutex_lock(&fpc1020->lock);
	rc = select_pin_ctl(fpc1020, buf);
	mutex_unlock(&fpc1020->lock);

	return rc ? rc : count;
}
static DEVICE_ATTR_WO(pinctl_set);

static ssize_t regulator_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	char op;
	char name[16];
	int rc;
	bool enable;

	if (sscanf(buf, "%15[^,],%c", name, &op) != NUM_PARAMS_REG_ENABLE_SET)
		return -EINVAL;
	if (op == 'e')
		enable = true;
	else if (op == 'd')
		enable = false;
	else
		return -EINVAL;

	mutex_lock(&fpc1020->lock);
	rc = vreg_setup(fpc1020, name, enable);
	mutex_unlock(&fpc1020->lock);

	return rc ? rc : count;
}
static DEVICE_ATTR_WO(regulator_enable);

static void hw_reset(struct fpc1020_data *fpc1020)
{
	(void)gpio_get_value(fpc1020->irq_gpio);

	select_pin_ctl(fpc1020, "fpc1020_reset_active");

	usleep_range(RESET_HIGH_SLEEP1_MIN_US, RESET_HIGH_SLEEP1_MAX_US);

	select_pin_ctl(fpc1020, "fpc1020_reset_reset");

	usleep_range(RESET_LOW_SLEEP_MIN_US, RESET_LOW_SLEEP_MAX_US);

	select_pin_ctl(fpc1020, "fpc1020_reset_active");

	usleep_range(RESET_HIGH_SLEEP2_MIN_US, RESET_HIGH_SLEEP2_MAX_US);

	(void)gpio_get_value(fpc1020->irq_gpio);
}

static ssize_t hw_reset_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!memcmp(buf, "reset", strlen("reset"))) {
		mutex_lock(&fpc1020->lock);
		hw_reset(fpc1020);
		mutex_unlock(&fpc1020->lock);
		return count;
	}

	return -EINVAL;
}
static DEVICE_ATTR_WO(hw_reset);

/*
 * Will setup GPIOs, and regulators to correctly initialize the touch sensor to
 * be ready for work.
 *
 * In the correct order according to the sensor spec this function will
 * enable/disable regulators, and reset line, all to set the sensor in a
 * correct power on or off state "electrical" wise.
 *
 * @see  device_prepare_set
 * @note This function will not send any commands to the sensor it will only
 *       control it "electrically".
 */
static void device_prepare(struct fpc1020_data *fpc1020, bool enable)
{
	mutex_lock(&fpc1020->lock);
	if (enable && !fpc1020->prepared) {
		// fpc1020->prepared = true;
		irq_setup(fpc1020, true);
		select_pin_ctl(fpc1020, "fpc1020_reset_reset");

/*
		vreg_setup(fpc1020, "vcc_spi", true);
		vreg_setup(fpc1020, "vdd_io", true);
		vreg_setup(fpc1020, "vdd_ana", true);
*/

		usleep_range(PWR_ON_SLEEP_MIN_US, PWR_ON_SLEEP_MAX_US);

		/* As we can't control chip select here the other part of the
		 * sensor driver eg. the TEE driver needs to do a _SOFT_ reset
		 * on the sensor after power up to be sure that the sensor is
		 * in a good state after power up. Okeyed by ASIC.
		 */
		select_pin_ctl(fpc1020, "fpc1020_reset_active");
		hw_reset(fpc1020);

		fpc1020->prepared = true;
	} else if (!enable && fpc1020->prepared) {
		irq_setup(fpc1020, false);
		select_pin_ctl(fpc1020, "fpc1020_reset_reset");

		usleep_range(PWR_ON_SLEEP_MIN_US, PWR_ON_SLEEP_MAX_US);

/*
		vreg_setup(fpc1020, "vdd_ana", false);
		vreg_setup(fpc1020, "vdd_io", false);
		vreg_setup(fpc1020, "vcc_spi", false);
*/

		fpc1020->prepared = false;
	}
	mutex_unlock(&fpc1020->lock);
}

/*
 * sysfs node to enable/disable (power up/power down) the touch sensor
 *
 * @see device_prepare
 */
static ssize_t device_prepare_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc = 0;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!memcmp(buf, "enable", strlen("enable")))
		device_prepare(fpc1020, true);
	else if (!memcmp(buf, "disable", strlen("disable")))
		device_prepare(fpc1020, false);
	else
		rc = -EINVAL;

	return rc ? rc : count;
}
static DEVICE_ATTR_WO(device_prepare);

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t wakeup_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	ssize_t ret = count;

	mutex_lock(&fpc1020->lock);
/*
	if (!memcmp(buf, "enable", strlen("enable")))
		atomic_set(&fpc1020->wakeup_enabled, 1);
	else if (!memcmp(buf, "disable", strlen("disable")))
		atomic_set(&fpc1020->wakeup_enabled, 0);
	else
		ret = -EINVAL;
*/
	mutex_unlock(&fpc1020->lock);

	return ret;
}
static DEVICE_ATTR_WO(wakeup_enable);


/*
 * sysfs node for controlling the wakelock.
 */
static ssize_t handle_wakelock_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	ssize_t ret = count;

	mutex_lock(&fpc1020->lock);
	if (!memcmp(buf, RELEASE_WAKELOCK_W_V,
		min(count, strlen(RELEASE_WAKELOCK_W_V)))) {
		if (fpc1020->nbr_irqs_received_counter_start ==
				fpc1020->nbr_irqs_received) {
			__pm_relax(fpc1020->ttw_wl);
		} else {
			dev_dbg(dev, "Ignore releasing of wakelock %d != %d",
			fpc1020->nbr_irqs_received_counter_start,
			fpc1020->nbr_irqs_received);
		}
	} else if (!memcmp(buf, RELEASE_WAKELOCK, min(count,
				strlen(RELEASE_WAKELOCK)))) {
		__pm_relax(fpc1020->ttw_wl);
	} else if (!memcmp(buf, START_IRQS_RECEIVED_CNT,
			min(count, strlen(START_IRQS_RECEIVED_CNT)))) {
		fpc1020->nbr_irqs_received_counter_start =
		fpc1020->nbr_irqs_received;
	} else
		ret = -EINVAL;
	mutex_unlock(&fpc1020->lock);

	return ret;
}
static DEVICE_ATTR_WO(handle_wakelock);

/*
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int irq = gpio_get_value(fpc1020->irq_gpio);

	return scnprintf(buf, PAGE_SIZE, "%i\n", irq);
}
static DEVICE_ATTR_RO(irq);

static ssize_t irq_enable_set(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	int rc = 0;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	mutex_lock(&fpc1020->lock);
	if (!memcmp(buf, "1", strlen("1"))) {
		enable_irq(gpio_to_irq(fpc1020->irq_gpio));
	} else if (!memcmp(buf, "0", strlen("0"))) {
		disable_irq(gpio_to_irq(fpc1020->irq_gpio));
	}
	mutex_unlock(&fpc1020->lock);

	return rc ? rc : count;
}

static DEVICE_ATTR(irq_enable, S_IWUSR | S_IRUSR | S_IRGRP | S_IWGRP, NULL,
		   irq_enable_set);

static struct attribute *attributes[] = {
	&dev_attr_request_vreg.attr,
	&dev_attr_pinctl_set.attr,
	&dev_attr_device_prepare.attr,
	&dev_attr_regulator_enable.attr,
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_handle_wakelock.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq_enable.attr,
	&dev_attr_irq.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;

	pr_info("fpc1020 irq handler: %s\n", __func__);
	mutex_lock(&fpc1020->lock);
	if (atomic_read(&fpc1020->wakeup_enabled)) {
		fpc1020->nbr_irqs_received++;
		__pm_wakeup_event(fpc1020->ttw_wl,
					msecs_to_jiffies(FPC_TTW_HOLD_TIME));
	}
	mutex_unlock(&fpc1020->lock);

	sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);

	return IRQ_HANDLED;
}

static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
	const char *label, int *gpio)
{
	struct device *dev = fpc1020->dev;
	struct device_node *np = dev->of_node;
	int rc;

	rc = of_get_named_gpio(np, label, 0);

	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		return FPC_GPIO_NO_DEFINED;
	}
	*gpio = rc;

	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return FPC_GPIO_REQUEST_FAIL;
	}
	dev_dbg(dev, "%s %d\n", label, *gpio);

	return 0;
}

static int fpc1020_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fpc1020_data *fpc1020;
	int rc;
	size_t i;

	fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020), GFP_KERNEL);
	if (!fpc1020)
		return -ENOMEM;

	fpc1020->dev = dev;
	platform_set_drvdata(pdev, fpc1020);

/*
	rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_irq",
			&fpc1020->irq_gpio);
	if (rc)
		return -EINVAL;
	rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_rst",
			&fpc1020->rst_gpio);
	if (rc)
		return -EINVAL;
*/

	fpc1020->fingerprint_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(fpc1020->fingerprint_pinctrl)) {
		rc = PTR_ERR(fpc1020->fingerprint_pinctrl);
		if (rc == -EPROBE_DEFER) {
			dev_info(dev, "pinctrl not ready\n");
			return rc;
		}
		dev_err(dev, "Cannot get pinctrl\n", rc);
		return rc;
		// fpc1020->fingerprint_pinctrl = NULL;
		//return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(pctl_names); i++) {
	// for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state =
			pinctrl_lookup_state(fpc1020->fingerprint_pinctrl, n);
		if (IS_ERR(state)) {
			dev_err(dev, "cannot find '%s'\n", n);
			return PTR_ERR(state);
		}
		dev_dbg(dev, "found pin control %s\n", n);
		fpc1020->pinctrl_state[i] = state;
	}

/*
	select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	select_pin_ctl(fpc1020, "fpc1020_irq_active");

	atomic_set(&fpc1020->wakeup_enabled, 0);
*/

	atomic_set(&fpc1020->wakeup_enabled, 1);

	fpc1020->irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
	fpc1020->irq_requested = false;
	fpc1020->gpios_requested = false;
	device_init_wakeup(dev, 1);
/*
	if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
		irqf = IRQF_NO_SUSPEND;
		device_init_wakeup(dev, 1);
	}
*/

	mutex_init(&fpc1020->lock);
/*
	rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
			NULL, fpc1020_irq_handler,
			irqf | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			dev_name(dev), fpc1020);
	if (rc) {
		dev_err(dev, "could not request irq %d\n",
				gpio_to_irq(fpc1020->irq_gpio));
		return rc;
	}

	dev_dbg(dev, "requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));

	enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));
*/

	fpc1020->ttw_wl = wakeup_source_register(dev, "fpc_ttw_wl");
	if (!fpc1020->ttw_wl)
		return -ENOMEM;

	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		return rc;
	}

	rc = reset_gpio_res(fpc1020);
	if (rc) {
		dev_err(dev, "fpc %s: could not initialize GPIOs No.!\n",
			__func__);
		return rc;
	}

	if (of_property_read_bool(dev->of_node, "fpc,enable-on-boot")) {
		dev_dbg(dev, "Enabling hardware\n");
		device_prepare(fpc1020, true);
	}

	// hw_reset(fpc1020);

	if (msm_gpio_mpm_wake_set(17, true))
		dev_info(dev, "%s: GPIO 17 wake set failed!\n", __func__);
	else
		dev_info(dev, "%s: GPIO 17 wake set success!\n", __func__);

	return 0;
}

static int fpc1020_remove(struct platform_device *pdev)
{
	struct fpc1020_data *fpc1020 = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &attribute_group);
	mutex_destroy(&fpc1020->lock);
	wakeup_source_unregister(fpc1020->ttw_wl);
	(void)reset_gpio_res(fpc1020);
/*
	vreg_setup(fpc1020, "vdd_ana", false);
	vreg_setup(fpc1020, "vdd_io", false);
	vreg_setup(fpc1020, "vcc_spi", false);
*/

	return 0;
}

static struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{}
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct platform_driver fpc1020_driver = {
	.driver = {
		.name	= "fpc1020",
		.of_match_table = fpc1020_of_match,
	},
	.probe	= fpc1020_probe,
	.remove	= fpc1020_remove,
};

module_platform_driver(fpc1020_driver);

MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");
MODULE_LICENSE("GPL v2");
