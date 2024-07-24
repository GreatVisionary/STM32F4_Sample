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
	cordic31 = (int32_t)((value/coeff)*0x80000000);		//value��coeff��һ����Ȼ������2^31����ȡ���õ�Q31��������
	return cordic31;
}

int32_t pInBuff;			//��������
int32_t pOutBuff[2];		//�������
void cordic_calculate_start(float arg1)			
{
	//������ǹ�һ���Ƕ�[0, 1.0]
	if(arg1 > 0.5f)								//���Ƕȴ�[0, 1.0]ת����[-0.5, 0.5]
		arg1 -= 1.0f;
	pInBuff = value_to_cordic31(arg1, 0.5f);	//[-0.5, 0.5]��0.5f��һ����ת����[-1.0, 1.0]������Q31�������ݱ�ʾ
	
	//������ǻ���ֵ[0, 6.28]
	/*if(arg1 > 3.14f)							//���Ƕȴ�[0, 6.28]ת����[-3.14, 3.14]
		arg1 -= 6.28f;
	pInBuff = value_to_cordic31(arg1, 3.14f);	//[-3.14, 3.14]��3.14f��һ����ת����[-1.0, 1.0]������Q31�������ݱ�ʾ
	*/
	
	HAL_CORDIC_Calculate_DMA(&hcordic, &pInBuff, pOutBuff, 1, CORDIC_DMA_DIR_IN_OUT);			//��ʼת��
}

void cordic31_to_value(int cordic31, float *res)
{
	if(cordic31&0x80000000)			//Ϊ����
	{
		cordic31 = cordic31&0x7fffffff;
		*res = ((float)(cordic31)-0x80000000)/0x80000000;
	}
	else							//Ϊ����
	{
		*res = (float)(cordic31)/0x80000000;
	}
}

void cordic_get_result(float *res1, float *res2)
{	
	while (HAL_CORDIC_GetState(&hcordic) != HAL_CORDIC_STATE_READY);	//�ȴ�cordicģ�����

	cordic31_to_value(pOutBuff[0], res1);								//����ֵת����
	cordic31_to_value(pOutBuff[1], res2);								//����ֵת����
}
