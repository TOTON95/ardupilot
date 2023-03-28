# Control of Spektrum power pin
PJ12 SPEKTRUM_PWR OUTPUT LOW GPIO(67)
define HAL_GPIO_SPEKTRUM_PWR 67

# Spektrum Power is Active High
define HAL_SPEKTRUM_PWR_ENABLED 0

# Spektrum RC Input pin, used as GPIO for bind for Satellite Receivers
PB0 SPEKTRUM_RC INPUT PULLUP GPIO(68)
define HAL_GPIO_SPEKTRUM_RC 68