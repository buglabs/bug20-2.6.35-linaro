#ifndef BMI_SLOT_H
#define BMI_SLOT_H

//void bmi_slot_resrc_init(void);

void bmi_slot_power_on  (int num);
void bmi_slot_power_off (int num);

void bmi_slot_gpio_direction_out (int num, unsigned gpio, int value);
void bmi_slot_gpio_direction_in (int num, unsigned gpio);
int bmi_slot_gpio_get_value (int num, unsigned gpio);
void bmi_slot_gpio_set_value (int num, unsigned gpio, int data);
int bmi_slot_gpio_get_all (int num);

void bmi_slot_uart_enable  (int num);
void bmi_slot_uart_disable (int num);

void bmi_slot_spi_enable (int num);
void bmi_slot_spi_disable (int num);

void bmi_slot_audio_enable  (int num);
void bmi_slot_audio_disable (int num);

void bmi_slot_battery_enable (int num);
void bmi_slot_battery_disable (int num);

int bmi_slot_module_present (int num);
//int bmi_slot_status_irq_state (int num);


#endif
