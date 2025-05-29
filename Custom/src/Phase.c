/**
 ******************************************************************************
 * @file    Phase.c
 * @brief   相位差计算 源文件
 ******************************************************************************
 * @attention
 *
 * 本文件用于实现相位差计算的计算逻辑
 *
 ******************************************************************************
 */

#include "Phase.h"

uint8_t ADC_COMPLETED = 0; // ADC 采样完毕则置为 1 ，否则为 0

uint32_t ADC_Raw_Data[FFT_LENGTH];    // 用于接收双ADC采集的原始数据的数组
uint16_t ADC_1_Value_DMA[FFT_LENGTH]; // 存放 ADC1 采样的原始值，因为使用16位 ADC 所以是 uint16_t
uint16_t ADC_2_Value_DMA[FFT_LENGTH]; // 存放 ADC2 采样的原始值，因为使用16位 ADC 所以是 uint16_t

float32_t ADC_1_Real_Value[FFT_LENGTH]; // 将 ADC1 的原始数据换算后的实际电压值
float32_t ADC_2_Real_Value[FFT_LENGTH]; // 将 ADC2 的原始数据换算后的实际电压值

float32_t FFT_InputBuf[FFT_LENGTH * 2]; // FFT输入数组，大小为点数的两倍
float32_t FFT_OutputBuf[FFT_LENGTH];    // FFT输出数组，大小等于点数，调用arm_cfft_f32_app后存储的是模值大小
uint8_t ifftFlag = 0;
uint8_t doBitReverse = 1;

/**
 * @brief ADC 初始化函数，调用后完成ADC的校准、开启操作
 *
 * @param hadc1 传入 &hadc1 即可
 * @param hadc2 传入 &hadc2 即可
 */
void PhaseCalculate_ADC_Init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2)
{
  // ADC校准
  HAL_ADCEx_Calibration_Start(hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);

  // 启动从ADC
  HAL_ADC_Start(hadc2);

  // 启动多模式DMA采样，数据存到 ADC_Raw_Data
  HAL_ADCEx_MultiModeStart_DMA(hadc1, (uint32_t *)ADC_Raw_Data, FFT_LENGTH);
}

/**
 * @brief  对输入的实数数据进行 FFT 变换并计算幅度谱
 * @param  rawData: 输入的实数数组，长度为 FFT_LENGTH，表示时域信号
 * @param  fft_instance: 指向 FFT 配置结构体（如 &arm_cfft_sR_f32_len1024），可以自行决定FFT的点数
 * @retval None
 *
 * @note
 *        - FFT_LENGTH 必须匹配所选的 FFT 实例；
 *        - 输入按 [实部, 虚部, ...] 方式构造复数；
 *        - 输出结果保存在全局变量 FFT_OutputBuf[] 中；
 *        - 1024点FFT调用示例：arm_cfft_f32_app(raw_data_buffer, &arm_cfft_sR_f32_len1024);
 */
static void arm_cfft_f32_app(float *rawData, const arm_cfft_instance_f32 *fft_instance)
{
  uint16_t n;
  uint16_t fftLen = fft_instance->fftLen; // 从实例中读取 FFT 长度

  for (n = 0; n < fftLen; n++)
  {
    FFT_InputBuf[2 * n] = rawData[n]; // 实部
    FFT_InputBuf[2 * n + 1] = 0.0f;   // 虚部为 0
  }

  arm_cfft_f32(fft_instance, FFT_InputBuf, ifftFlag, doBitReverse);
  arm_cmplx_mag_f32(FFT_InputBuf, FFT_OutputBuf, fftLen);
}

/**
 * @brief  在幅值数组的前半段中查找最大值的索引（忽略直流分量 ARR[0]）
 * @param  ARR: 输入的幅值数组，长度应为 FFT_LENGTH
 * @retval 最大值所在的下标（从 1 到 FFT_LENGTH/2 - 1）
 */
int Find_nMax(const float *ARR)
{
  if (ARR == NULL)
    return -1; // 输入非法

  float aMax = ARR[1];
  uint32_t nMax = 1;

  for (uint32_t i = 2; i < FFT_LENGTH / 2; i++)
  {
    if (ARR[i] > aMax)
    {
      aMax = ARR[i];
      nMax = i;
    }
  }

  return nMax;
}

/**
 * @brief     计算输入信号的基频相位角（单位：度）
 *
 * @param[in] signal  输入的实数信号数组，长度应为 FFT_LENGTH
 *
 * @return    float32_t  频谱中主频率分量的相位角，单位为度
 *
 * @note
 *           - 使用 CMSIS DSP 库进行 FFT 和幅值计算。
 *           - 函数依赖外部全局缓冲区 FFT_InputBuf 和 FFT_OutputBuf。
 *           - 基频通过幅度谱最大值定位（跳过直流分量）。
 */
float32_t Find_PhaseAngle(float32_t *signal)
{
  // 执行 FFT 与幅值计算
  arm_cfft_f32_app(signal, &arm_cfft_sR_f32_len1024);

  // 获取最大幅值对应的下标（视为基频）
  int n_max = Find_nMax(FFT_OutputBuf);

  // 利用复数的虚部和实部计算相位角（atan2f 返回弧度）
  float32_t phase_rad = atan2f(FFT_InputBuf[2 * n_max + 1], FFT_InputBuf[2 * n_max]);

  // 转换为角度返回
  return phase_rad * 180.0f / 3.1415926f;
}

/**
 * @brief 用于处理 ADC 采样原始数据的函数，只在 ADC 采样完毕后可以被调用。
 *
 * @note  ADC1 采样原始数据存放在 ADC_1_Value_DMA
 *        ADC1 采样的实际电压值存放在 ADC_1_Real_Value
 *        ADC2 采样原始数据存放在 ADC_2_Value_DMA
 *        ADC2 采样的实际电压值存放在 ADC_2_Real_Value
 */
void Process_ADC_RawData(void)
{
  if (ADC_COMPLETED)
  {
    ADC_COMPLETED = 0;

    for (int i = 0; i < FFT_LENGTH; i++)
    {
      uint32_t raw = ADC_Raw_Data[i];
      uint16_t adc1 = raw & 0xFFFF;
      uint16_t adc2 = (raw >> 16) & 0xFFFF;

      ADC_1_Value_DMA[i] = adc1;
      ADC_2_Value_DMA[i] = adc2;

      // 将原始 ADC 值换算为实际电压
      ADC_1_Real_Value[i] = ((float32_t)adc1 - 32768.0f) * Reference_Voltage / 65536.0f;
      ADC_2_Real_Value[i] = ((float32_t)adc2 - 32768.0f) * Reference_Voltage / 65536.0f;
      // printf("%f, %f\n",(double)ADC_1_Real_Value[i],(double)ADC_2_Real_Value[i]);
    }
  }
}

/**
 * @brief 获取 ADC1 和 ADC2 的相位差（单位：度）
 *        相位差 = ADC1 相位 - ADC2 相位
 *
 * @return float32_t 相位差值（正负表示相对相位关系）
 */
float32_t Get_PhaseDifference(void)
{
  // 处理原始数据，只在采样完成后才有效
  Process_ADC_RawData();

  // 使用原始 ADC 数据计算两个通道的相位角
  float32_t phase1 = Find_PhaseAngle(ADC_1_Real_Value);
  float32_t phase2 = Find_PhaseAngle(ADC_2_Real_Value);

  // 计算相位差（相位1 - 相位2）
  float32_t phase_diff = phase1 - phase2;

  // 将相位差归一化到 [-180°, 180°] 区间
  if (phase_diff > 180.0f)
    phase_diff -= 360.0f;
  else if (phase_diff < -180.0f)
    phase_diff += 360.0f;

  return phase_diff;
}
