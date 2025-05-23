#ifndef __PHASE_H
#define __PHASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"  // 让你可以访问 CubeMX 定义的 GPIO/I2C/ADC 等
#include "arm_math.h"
#include "arm_const_structs.h"

// FFT 点数
#define FFT_LENGTH 1024 
// 片上 ADC 使用的参考电压大小（单位：V）
#define Reference_Voltage 3.36

extern uint8_t ADC_COMPLETED; // ADC传输完成标志位
extern uint32_t ADC_Raw_Data[1024]; // 用于接收ADC采集的原始数据的数组
extern uint8_t ifftFlag;
extern uint8_t doBitReverse;

// ADC 初始化函数
void PhaseCalculate_ADC_Init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);
// 处理ADC原始数据的函数，如果需要获取采样电压波形信息可以调用此函数
void Process_ADC_RawData(void);
// 获取相位差，qi内部调用了 Process_ADC_RawData.
float32_t Get_PhaseDifference(void);

#ifdef __cplusplus
}
#endif

#endif /* __PHASE_H */
