#include "Ds18b20.h"

/****************************************
函数名称：void gpio_init(void)
函数参数：无参
函数返回值：无
函数功能：IO口初始化

****************************************/
void Tempetature_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(Temp_DS_GPIO_Port, Temp_DS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : TD_Pin */
    GPIO_InitStruct.Pin = Temp_DS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(Temp_DS_GPIO_Port, &GPIO_InitStruct);
}

/****************************************
函数名称：void mode_output(void)
函数参数：无参
函数返回值：无
函数功能：输出模式

****************************************/
void Mode_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin : TD_Pin */
    GPIO_InitStruct.Pin = Temp_DS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(Temp_DS_GPIO_Port, &GPIO_InitStruct);
}

/****************************************
函数名称：void mode_input(void)
函数参数：无参
函数返回值：无
函数功能：输入模式

****************************************/
void Mode_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin : TD_Pin */
    GPIO_InitStruct.Pin = Temp_DS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(Temp_DS_GPIO_Port, &GPIO_InitStruct);
}

/****************************************
函数名称：void Ds18b20_Rst(void)
函数参数：无参
函数返回值：无
函数功能：复位

****************************************/
void Ds18b20_Rst(void)
{
    int i=0;
    Mode_Output();                //输出模式
    HAL_GPIO_WritePin(Temp_DS_GPIO_Port, Temp_DS_Pin, GPIO_PIN_RESET);              //拉低DQ引脚
    delay_us(600);         //延时480-960us
    HAL_GPIO_WritePin(Temp_DS_GPIO_Port, Temp_DS_Pin, GPIO_PIN_SET);              //拉高DQ引脚
    Mode_Input();
    while(HAL_GPIO_ReadPin(Temp_DS_GPIO_Port, Temp_DS_Pin))       //等待器件应答（器件拉低），约15-60us后
    {
        HAL_Delay(1);
        if(HAL_GPIO_ReadPin(Temp_DS_GPIO_Port, Temp_DS_Pin))
        {
            i=0;
            break;
        }
        else if(i>10)
        {
            i=0;
            break;
        }
        else
        {
            i++;
        }
    }
    while(!HAL_GPIO_ReadPin(Temp_DS_GPIO_Port, Temp_DS_Pin))        //应答脉冲出现后，等待器件拉高，约60-240us后
    {
        HAL_Delay(1);
        if(HAL_GPIO_ReadPin(Temp_DS_GPIO_Port, Temp_DS_Pin))
        {
            i=0;
            break;
        }
        else if(i>10)
        {
            i=0;
            break;
        }
        else
        {
            i++;
        }
    }
//  while(HAL_GPIO_ReadPin(TD_GPIO_Port, TD_Pin));         //等待器件应答（器件拉低），约15-60us后
//  while(!HAL_GPIO_ReadPin(TD_GPIO_Port, TD_Pin));        //应答脉冲出现后，等待器件拉高，约60-240us后
}

/****************************************
函数名称：void ds18b20_write_zero(void)
函数参数：无参
函数返回值：无
函数功能：写0位

****************************************/
void ds18b20_write_zero(void)
{
    Mode_Output();                    //输出模式
    HAL_GPIO_WritePin(Temp_DS_GPIO_Port, Temp_DS_Pin, GPIO_PIN_RESET);   //拉低引脚
    delay_us(80);              //延时60-120us
    HAL_GPIO_WritePin(Temp_DS_GPIO_Port, Temp_DS_Pin, GPIO_PIN_SET);     //拉高引脚
    delay_us(2);
}

/****************************************
函数名称：void ds18b20_write_one(void)
函数参数：无参
函数返回值：无
函数功能：写1位

****************************************/
void ds18b20_write_one(void)
{
    Mode_Output();                           //输出模式
    HAL_GPIO_WritePin(Temp_DS_GPIO_Port, Temp_DS_Pin, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(Temp_DS_GPIO_Port, Temp_DS_Pin, GPIO_PIN_SET);
    delay_us(80);
}

/****************************************
函数名称：void ds18b20_read_bit(void)
函数参数：无参
函数返回值：无
函数功能：读1位数据

****************************************/
unsigned char ds18b20_read_bit(void)
{
    unsigned char data;
    Mode_Output();
    HAL_GPIO_WritePin(Temp_DS_GPIO_Port, Temp_DS_Pin, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(Temp_DS_GPIO_Port, Temp_DS_Pin, GPIO_PIN_SET);
    Mode_Input();
    delay_us(5);
    if(HAL_GPIO_ReadPin(Temp_DS_GPIO_Port, Temp_DS_Pin))
        data=1;
    else
        data=0;
    delay_us(500);
    return data;
}

/****************************************
函数名称：void Ds18b20_Write_Byte(void)
函数参数：无参
函数返回值：无
函数功能：写一个字节

****************************************/
void Ds18b20_Write_Byte(unsigned char data)
{
    unsigned char i,testb;
    Mode_Output();
    for(i=0; i<8; i++)
    {
        testb=data&0x01;                 //从低位开始写
        data>>=1;
        if(testb)
            ds18b20_write_one();
        else
            ds18b20_write_zero();
    }
}

/****************************************
函数名称：unsigned char Ds18b20_Read_Byte(void)
函数参数：无参
函数返回值：无
函数功能：读一个字节

****************************************/
unsigned char Ds18b20_Read_Byte(void)
{
    unsigned char i,j,data=0;
    for(i=0; i<8; i++)
    {
        j=ds18b20_read_bit();
        data=(j<<7)|(data>>1);      //从低位开始读
    }
    return data;
}

/****************************************
函数名称：void Ds18b20_Start(void)
函数参数：无参
函数返回值：无
函数功能：开始转换

****************************************/
void Ds18b20_Start(void)
{
    Ds18b20_Rst();
    Ds18b20_Write_Byte(0xcc);   // 跳过ROM
    Ds18b20_Write_Byte(0x44);   //温度转换
}

/****************************************
函数名称：void ds18b20_init(void)
函数参数：无参
函数返回值：无
函数功能：初始化

****************************************/
void Ds18b20_Init(void)
{   
	  //需先拉高EN_DZ引脚提供DZ参考电压 用于温度测量使能
    Tempetature_GPIO_Init();     //此引脚需根据相应的单片机进行配置
    Ds18b20_Rst();
}

/****************************************
函数名称：short get_tempetature(void)
函数参数：无参
函数返回值：无
函数功能：获取温度

****************************************/
short Get_Tempetature(void)
{
    unsigned char TL,TH;
    short tem;
    Ds18b20_Start();           //开始转换
    HAL_Delay(700);             //等待转换完成
    Ds18b20_Init();
    Ds18b20_Write_Byte(0xcc);   //跳过ROM
    Ds18b20_Write_Byte(0xbe);   //读取暂存寄存器
    TL=Ds18b20_Read_Byte();     //低八位
    TH=Ds18b20_Read_Byte();     //高八位，注意前五位为符号位只有全为0的时候温度是正的
    tem=TH;                      //获得高八位
    tem<<=8;
    tem+=TL;                     //获得底八位
    return tem;
}


/****************************************
函数名称：void tempetatureDis(void)
函数参数：无参
函数返回值：无
函数功能：温度串口显示

****************************************/
float Tempetature_Dis(void)
{
    short TempValue = 0;
    float Value = 0;
//    int16_t Temp;
    TempValue  = Get_Tempetature();
    if((TempValue & 0XF800) > 0) 	//	当温度为零下时
    {
        TempValue = ~TempValue;
        TempValue = TempValue+1;
        Value = TempValue/16.0;
        Value = 0-Value;
    }
    else									//	当温度为零上时
    {
        Value = (TempValue & 0X07FF)/16.0;
    }
//    printf("\n Temp is :%f\r\n",Value);
    Value = ((int)(Value * 10)) / 10.0;  // 保留一位小数
    return Value;
}
//非阻塞式
short Ds18b20_Read_Result(void)
{
    unsigned char TL, TH;
    short tem;

    // 1. 复位 (这里直接调用Rst即可，不需要Init重新配GPIO时钟)
    Ds18b20_Rst(); 
    
    // 2. 发送读取命令
    Ds18b20_Write_Byte(0xcc);   // 跳过ROM
    Ds18b20_Write_Byte(0xbe);   // 读取暂存寄存器
    
    // 3. 读取高低字节
    TL = Ds18b20_Read_Byte();     // 低八位
    TH = Ds18b20_Read_Byte();     // 高八位
    
    // 4. 合成温度值
    tem = TH;
    tem <<= 8;
    tem += TL;
    
    return tem;
}

