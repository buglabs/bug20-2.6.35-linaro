#ifndef VS6624_ACCESS_H
#define VS6624_ACCESS_H

#include <linux/i2c.h>

void ov2640_patch (struct i2c_adapter *adap);

int vs6624_ReadByte(struct i2c_adapter *adap, unsigned short offset, unsigned char *data);
int vs6624_WriteByte(struct i2c_adapter *adap, unsigned short offset, unsigned char data);
int  vs6624_WriteSequence(struct i2c_adapter *adap, const unsigned short array[][2], unsigned short len);
void vs6624_dump_regs(struct i2c_adapter *adap);

#endif

