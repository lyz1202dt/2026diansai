#ifndef __EMM_V5_H
#define __EMM_V5_H

#include "Driver/uart/uart.h"
#include "Driver/zdt/uartport.h"
/**********************************************************
***	Emm_V5.0�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/

#define		ABS(x)		((x) > 0 ? (x) : -(x)) 

typedef enum {
	S_VER   = 0,			/* ��ȡ�̼��汾�Ͷ�Ӧ��Ӳ���汾 */
	S_RL    = 1,			/* ��ȡ��ȡ���������� */
	S_PID   = 2,			/* ��ȡPID���� */
	S_VBUS  = 3,			/* ��ȡ���ߵ�ѹ */
	S_CPHA  = 5,			/* ��ȡ����� */
	S_ENCL  = 7,			/* ��ȡ�������Ի�У׼��ı�����ֵ */
	S_TPOS  = 8,			/* ��ȡ���Ŀ��λ�ýǶ� */
	S_VEL   = 9,			/* ��ȡ���ʵʱת�� */
	S_CPOS  = 10,			/* ��ȡ���ʵʱλ�ýǶ� */
	S_PERR  = 11,			/* ��ȡ���λ�����Ƕ� */
	S_FLAG  = 13,			/* ��ȡʹ��/��λ/��ת״̬��־λ */
	S_Conf  = 14,			/* ��ȡ�������� */
	S_State = 15,			/* ��ȡϵͳ״̬���� */
	S_ORG   = 16,     /* ��ȡ���ڻ���/����ʧ��״̬��־λ */
}SysParams_t;


/**********************************************************
*** ע�⣺ÿ�������Ĳ����ľ���˵��������Ķ�Ӧ������ע��˵��
**********************************************************/
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr); // ����ǰλ������
void Emm_V5_Reset_Clog_Pro(uint8_t addr); // �����ת����
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s); // ��ȡ����
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, uint32_t svF, uint8_t ctrl_mode); // ���������޸Ŀ���/�ջ�����ģʽ
void Emm_V5_En_Control(uint8_t addr, uint32_t state, uint32_t snF); // ���ʹ�ܿ���
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t snF); // �ٶ�ģʽ����
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint32_t raF, uint32_t snF); // λ��ģʽ����
void Emm_V5_Stop_Now(uint8_t addr, uint32_t snF); // �õ������ֹͣ�˶�
void Emm_V5_Synchronous_motion(uint8_t addr); // �������ͬ����ʼ�˶�
void Emm_V5_Origin_Set_O(uint8_t addr, uint32_t svF); // ���õ�Ȧ��������λ��
void Emm_V5_Origin_Modify_Params(uint8_t addr, uint32_t svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, uint32_t potF); // �޸Ļ������
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, uint32_t snF); // �������������
void Emm_V5_Origin_Interrupt(uint8_t addr); // ǿ���жϲ��˳�����

//���պ���
//void Emm_V5_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount);
uint32_t Emm_V5_GetPos(uint8_t addr,uint8_t* recv_buf,float* pos);

void Emm_V5_Pos_ControlEx(uint8_t addr,float angle,float cur_angle,float dt);

#endif
