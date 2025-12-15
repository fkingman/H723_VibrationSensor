#ifndef DELAY_H_
#define DELAY_H_

#include "main.h"

/* 初始化函数，务必在 main() 开头调用 */
void Delay_Init(void);

/* 延时函数 */
void delay_us(uint32_t us);

#endif /* DELAY_H_ */