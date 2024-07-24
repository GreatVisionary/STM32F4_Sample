#include "Cordic.h"
CORDIC_ConfigTypeDef sCordicConfig;
extern CORDIC_HandleTypeDef hcordic;
void cordic_config(void)
{

	sCordicConfig.Function         = CORDIC_FUNCTION_SINE;     /* sine function */
  sCordicConfig.Precision        = CORDIC_PRECISION_6CYCLES; /* max precision for q1.31 sine */
 	sCordicConfig.Scale            = CORDIC_SCALE_0;           /* no scale */
 	sCordicConfig.NbWrite          = CORDIC_NBWRITE_1;         /* One input data: angle. Second input data (modulus) is 1 after cordic reset */
 	sCordicConfig.NbRead           = CORDIC_NBREAD_2;          /* One output data: sine */
 	sCordicConfig.InSize           = CORDIC_INSIZE_32BITS;     /* q1.31 format for input data */
  sCordicConfig.OutSize          = CORDIC_OUTSIZE_32BITS;    /* q1.31 format for output data */
	
	HAL_CORDIC_Configure(&hcordic,&sCordicConfig);
}

int32_t value_to_cordic31(float value, float coeff)
{
	int32_t cordic31;
	cordic31 = (int32_t)((value/coeff)*0x80000000);		//value对coeff归一化，然后扩大2^31倍，取整得到Q31定点数据
	return cordic31;
}

int32_t pInBuff;			//输入数据
int32_t pOutBuff[2];		//输出数据
void cordic_calculate_start(float arg1)			
{
	//传入的是归一化角度[0, 1.0]
	if(arg1 > 0.5f)								//将角度从[0, 1.0]转换到[-0.5, 0.5]
		arg1 -= 1.0f;
	pInBuff = value_to_cordic31(arg1, 0.5f);	//[-0.5, 0.5]对0.5f归一化，转换到[-1.0, 1.0]，并用Q31定点数据表示
	
	//传入的是弧度值[0, 6.28]
	/*if(arg1 > 3.14f)							//将角度从[0, 6.28]转换到[-3.14, 3.14]
		arg1 -= 6.28f;
	pInBuff = value_to_cordic31(arg1, 3.14f);	//[-3.14, 3.14]对3.14f归一化，转换到[-1.0, 1.0]，并用Q31定点数据表示
	*/
	
	HAL_CORDIC_Calculate_DMA(&hcordic, &pInBuff, pOutBuff, 1, CORDIC_DMA_DIR_IN_OUT);			//开始转换
}

void cordic31_to_value(int cordic31, float *res)
{
	if(cordic31&0x80000000)			//为负数
	{
		cordic31 = cordic31&0x7fffffff;
		*res = ((float)(cordic31)-0x80000000)/0x80000000;
	}
	else							//为正数
	{
		*res = (float)(cordic31)/0x80000000;
	}
}

void cordic_get_result(float *res1, float *res2)
{	
	while (HAL_CORDIC_GetState(&hcordic) != HAL_CORDIC_STATE_READY);	//等待cordic模块空闲

	cordic31_to_value(pOutBuff[0], res1);								//正弦值转浮点
	cordic31_to_value(pOutBuff[1], res2);								//余弦值转浮点
}
