#include <linux/device.h>
#include <linux/bmi.h>


#define bmi_config_attr(field, format_string)				\
static ssize_t								\
field##_show(struct device *dev, struct device_attribute *attr, char *buf)				\
{									\
	struct bmi_device *pdev;						\
									\
	pdev = to_bmi_device (dev);					\
	return sprintf (buf, format_string, pdev->field);		\
}

static ssize_t description_show(struct device *dev, struct device_attribute *attr, char *buf)
{									
	struct bmi_device *pdev;					
									
	pdev = to_bmi_device (dev);					
	return sprintf (buf, "%s\n", pdev->ident->description);		
}

bmi_config_attr(vendor, "0x%04x\n");
bmi_config_attr(product, "0x%04x\n");
bmi_config_attr(revision, "0x%04x\n");

static ssize_t suspend_show(struct device *dev, 
				    struct device_attribute *attr, char *buf)
{
	int len = 0;
	int status = 0;
	struct bmi_device *pdev;
	
	pdev = to_bmi_device (dev);
	status = pdev->dev.power.status;

	if (status == DPM_ON)
		len += sprintf(buf+len, "0");
	else
		len += sprintf(buf+len, "1");

	len += sprintf(len+buf, "\n");
	return len;
}

static ssize_t suspend_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t len)
{
	struct bmi_device *pdev;

	pdev = to_bmi_device (dev);

	if (strchr(buf, '1') != NULL) {
		pdev->dev.bus->pm->suspend(&pdev->dev);
	}
	else if (strchr(buf, '0') != NULL) {
		pdev->dev.bus->pm->resume(&pdev->dev);
	}

	return len;
}

struct device_attribute bmi_dev_attrs[] = {
	__ATTR_RO(vendor),
	__ATTR_RO(product),
	__ATTR_RO(revision),
	__ATTR_RO(description),
	__ATTR(suspend, 0664, suspend_show, suspend_store),
	__ATTR_NULL,
};
