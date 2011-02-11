/*
 * Copyright 2011, Matt Isaacs <izzy@buglabs.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/if.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/leds.h>
#include <net/cfg80211.h>
#include "dev.h"

void lbs_led_assoc(struct lbs_private *priv, bool associated)
{
	if (unlikely(!priv->assoc_led))
		return;
	if (associated)
		led_trigger_event(priv->assoc_led, LED_FULL);
	else
		led_trigger_event(priv->assoc_led, LED_OFF);
}

void lbs_led_radio(struct lbs_private *priv)
{
	if (unlikely(!priv->radio_led))
		return;
	if (priv->radio_on)
		led_trigger_event(priv->radio_led, LED_FULL);
	else
		led_trigger_event(priv->radio_led, LED_OFF);
}

void lbs_led_init(struct lbs_private *priv)
{
	priv->assoc_led = kzalloc(sizeof(struct led_trigger), GFP_KERNEL);
	if (priv->assoc_led) {
		snprintf(priv->assoc_led_name, sizeof(priv->assoc_led_name),
			"%sassoc", wiphy_name(priv->wdev->wiphy));
		priv->assoc_led->name = priv->assoc_led_name;
		if (led_trigger_register(priv->assoc_led)) {
			kfree(priv->assoc_led);
			priv->assoc_led = NULL;
		}
			
	}
	priv->radio_led = kzalloc(sizeof(struct led_trigger), GFP_KERNEL);
	if (priv->radio_led) {
		snprintf(priv->radio_led_name, sizeof(priv->radio_led_name),
			"%sradio", wiphy_name(priv->wdev->wiphy));
		priv->radio_led->name = priv->radio_led_name;
		if (led_trigger_register(priv->radio_led)) {
			kfree(priv->radio_led);
			priv->radio_led = NULL;
		}
			
	}
}

void lbs_led_exit(struct lbs_private *priv)
{
	if (priv->assoc_led) {
		led_trigger_unregister(priv->assoc_led);
		kfree(priv->assoc_led);
	}
	if (priv->radio_led) {
		led_trigger_unregister(priv->radio_led);
		kfree(priv->radio_led);
	}

}


