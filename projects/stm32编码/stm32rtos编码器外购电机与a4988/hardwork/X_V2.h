#ifndef __X_V2_H
#define __X_V2_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f10x.h"
#include "usart.h"

/**********************************************************
***	X_V2�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/

#define					ABS(x)							((x) > 0 ? (x) : -(x)) 

typedef enum {
	S_VBUS  = 5,	// ��ȡ���ߵ�ѹ
	S_CBUS  = 6,	// ��ȡ���ߵ���
	S_CPHA  = 7,	// ��ȡ�����
	S_ENCO  = 8,	// ��ȡ������ԭʼֵ
	S_CLKC  = 9,	// ��ȡʵʱ������
	S_ENCL  = 10,	// ��ȡ�������Ի�У׼��ı�����ֵ
	S_CLKI  = 11,	// ��ȡ����������
	S_TPOS  = 12,	// ��ȡ���Ŀ��λ��
	S_SPOS  = 13,	// ��ȡ���ʵʱ�趨��Ŀ��λ��
	S_VEL   = 14,	// ��ȡ���ʵʱת��
	S_CPOS  = 15,	// ��ȡ���ʵʱλ��
	S_PERR  = 16,	// ��ȡ���λ�����
	S_VBAT  = 17,	// ��ȡ��Ȧ��������ص�ѹ��Y42��
	S_TEMP  = 18,	// ��ȡ���ʵʱ�¶ȣ�X42S/Y42��
	S_FLAG  = 19,	// ��ȡ���״̬��־λ
	S_OFLAG = 20, // ��ȡ����״̬��־λ
	S_OAF   = 21,	// ��ȡ���״̬��־λ + ����״̬��־λ��X42S/Y42��
	S_PIN   = 22,	// ��ȡ����״̬��X42S/Y42��
	S_SYS   = 23,	// ��ȡϵͳ״̬��Ϣ
}SysParams_t;

#define		MMCL_LEN		512
extern __IO uint16_t MMCL_count, MMCL_cmd[MMCL_LEN];

/**
***********************************************************
***********************************************************
*** 
***
*** @brief	��׺���У�X42S/Y42��ΪX42S/Y42�������X42�����ã�����ͨ��
***
*** 
***********************************************************
***********************************************************
***/
/**********************************************************
*** ������������
**********************************************************/
// ����������У׼
void X_V2_Trig_Encoder_Cal(uint8_t addr);
// ���������X42S/Y42��
void X_V2_Reset_Motor(uint8_t addr);
// ����ǰλ������
void X_V2_Reset_CurPos_To_Zero(uint8_t addr);
// �����ת����
void X_V2_Reset_Clog_Pro(uint8_t addr);
// �ָ���������
void X_V2_Restore_Motor(uint8_t addr);
/**********************************************************
*** �˶���������
**********************************************************/
// �������X42S/Y42��
void X_V2_Multi_Motor_Cmd(uint8_t addr); // �ο�����Ķ�������˵��ʹ�ã��������������ʹ�ã���˵����Ķ��������½�˵����ԭ����һ���Է��Ͷ�������������ȥ
// ���ʹ�ܿ���
void X_V2_En_Control(uint8_t addr, bool state, bool snF);
// ����ģʽ
void X_V2_Torque_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, bool snF);
// ����ģʽ���ٿ��ƣ�X42S/Y42��
void X_V2_Torque_LV_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, bool snF, float maxVel);
// �ٶ�ģʽ����
void X_V2_Vel_Control(uint8_t addr, uint8_t dir, uint16_t acc, float vel, bool snF);
// �ٶ�ģʽ�޵������ƣ�X42S/Y42��
void X_V2_Vel_LC_Control(uint8_t addr, uint8_t dir, uint16_t acc, float vel, bool snF, uint16_t maxCur);
// ֱͨ����λ��ģʽ����
void X_V2_Bypass_Pos_LV_Control(uint8_t addr, uint8_t dir, float vel, float pos, uint8_t raf, bool snF);
// ֱͨ����λ��ģʽ�޵������ƣ�X42S/Y42��
void X_V2_Bypass_Pos_LV_LC_Control(uint8_t addr, uint8_t dir, float vel, float pos, uint8_t raf, bool snF, uint16_t maxCur);
// �������߼Ӽ���λ��ģʽ����
void X_V2_Traj_Pos_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float vel, float pos, uint8_t raf, bool snF);
// �������߼Ӽ���λ��ģʽ�޵������ƣ�X42S/Y42��
void X_V2_Traj_Pos_LC_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float vel, float pos, uint8_t raf, bool snF, uint16_t maxCur);
// ���ÿ�����������λ��ģʽ���˶�������X42S/Y42��
void X_V2_Set_QTrajPos_Params(uint8_t addr, uint16_t acc, uint16_t dec, float vel, uint8_t raf, bool snF, uint16_t maxCur);
// ������������λ��ģʽ���ƣ�X42S/Y42��
void X_V2_QTrajPos_LC_Control(uint8_t addr, float pos);
// �õ������ֹͣ�˶�
void X_V2_Stop_Now(uint8_t addr, bool snF);
// �������ͬ����ʼ�˶�
void X_V2_Synchronous_motion(uint8_t addr);
/**********************************************************
*** ԭ���������
**********************************************************/
// ���õ�Ȧ��������λ��
void X_V2_Origin_Set_O(uint8_t addr, bool svF);
// ��������
void X_V2_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
// ǿ���жϲ��˳�����
void X_V2_Origin_Interrupt(uint8_t addr);
// ��ȡ�������
void X_V2_Origin_Read_Params(uint8_t addr);
// �޸Ļ������
void X_V2_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
// ��ȡ��ײ���㷵�ؽǶȣ�X42S/Y42��
void X_V2_Origin_Read_SL_RP(uint8_t addr);
// �޸���ײ���㷵�ؽǶȣ�X42S/Y42��
void X_V2_Origin_Modify_SL_RP(uint8_t addr, bool svF, uint16_t sl_rp);
/**********************************************************
*** ��ȡϵͳ��������
**********************************************************/
// ��ʱ������Ϣ���X42S/Y42��
void X_V2_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms);
// ��ȡϵͳ����
void X_V2_Read_Sys_Params(uint8_t addr, SysParams_t s);
/**********************************************************
*** ��д������������
**********************************************************/
// �޸ĵ��ID��ַ
void X_V2_Modify_Motor_ID(uint8_t addr, bool svF, uint8_t id);
// �޸�ϸ��ֵ
void X_V2_Modify_MicroStep(uint8_t addr, bool svF, uint8_t mstep);
// �޸ĵ����־
void X_V2_Modify_PDFlag(uint8_t addr, bool pdf);
// ��ȡѡ�����״̬��X42S/Y42��
void X_V2_Read_Opt_Param_Sta(uint8_t addr);
// �޸ĵ�����ͣ�X42S/Y42��
void X_V2_Modify_Motor_Type(uint8_t addr, bool svF, bool mottype);
// �޸Ĺ̼����ͣ�X42S/Y42��
void X_V2_Modify_Firmware_Type(uint8_t addr, bool svF, bool fwtype);
// �޸Ŀ���/�ջ�����ģʽ��X42S/Y42��
void X_V2_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode);
// �޸ĵ���˶�������X42S/Y42��
void X_V2_Modify_Motor_Dir(uint8_t addr, bool svF, bool dir);
// �޸������������ܣ�X42S/Y42��
void X_V2_Modify_Lock_Btn(uint8_t addr, bool svF, bool lockbtn);
// �޸�����λ�ýǶ��Ƿ������С10�����루X42S/Y42��
void X_V2_Modify_S_Vel(uint8_t addr, bool svF, bool s_vel);
// �޸Ŀ���ģʽ��������
void X_V2_Modify_OM_ma(uint8_t addr, bool svF, uint16_t om_ma);
// �޸ıջ�ģʽ������
void X_V2_Modify_FOC_mA(uint8_t addr, bool svF, uint16_t foc_mA);
// ��ȡPID����
void X_V2_Read_PID_Params(uint8_t addr);
// �޸�PID����
void X_V2_Modify_PID_Params(uint8_t addr, bool svF, uint32_t pTkp, uint32_t pBkp, uint32_t vkp, uint32_t vki);
// ��ȡDMX512Э�������X42S/Y42��
void X_V2_Read_DMX512_Params(uint8_t addr);
// �޸�DMX512Э�������X42S/Y42��
void X_V2_Modify_DMX512_Params(uint8_t addr, bool svF, uint16_t tch, uint8_t nch, uint8_t mode, uint16_t vel, uint16_t acc, uint16_t vel_step, uint32_t pos_step);
// ��ȡλ�õ��ﴰ�ڣ�X42S/Y42��
void X_V2_Read_Pos_Window(uint8_t addr);
// �޸�λ�õ��ﴰ�ڣ�X42S/Y42��
void X_V2_Modify_Pos_Window(uint8_t addr, bool svF, uint16_t prw);
// ��ȡ���ȹ������������ֵ��X42S/Y42��
void X_V2_Read_Otocp(uint8_t addr);
// �޸Ĺ��ȹ������������ֵ��X42S/Y42��
void X_V2_Modify_Otocp(uint8_t addr, bool svF, uint16_t otp, uint16_t ocp, uint16_t time_ms);
// ��ȡ������������ʱ�䣨X42S/Y42��
void X_V2_Read_Heart_Protect(uint8_t addr);
// �޸�������������ʱ�䣨X42S/Y42��
void X_V2_Modify_Heart_Protect(uint8_t addr, bool svF, uint32_t hp);
// ��ȡ�����޷�/����ϵ����X42S/Y42��
void X_V2_Read_Integral_Limit(uint8_t addr);
// �޸Ļ����޷�/����ϵ����X42S/Y42��
void X_V2_Modify_Integral_Limit(uint8_t addr, bool svF, uint32_t il);
/**********************************************************
*** ��ȡ����������������
**********************************************************/
// ��ȡϵͳ״̬����
void X_V2_Read_System_State_Params(uint8_t addr);
// ��ȡ�������ò���
void X_V2_Read_Motor_Conf_Params(uint8_t addr);

/**
***********************************************************
***********************************************************
*** @brief	��������˵��
***
*** @brief	�����ǰ���Ӧ������ص�X42S/Y42���������ϵĺ�����X42S/Y42��
*** @brief	���������һ���������ȷ��ͳ�ȥ���Ȱ�����浽����MMCL_cmd[]��
*** @brief	���ú�����void X_V2_Multi_Motor_Cmd(uint8_t addr); ���MMCL_cmd[]�����е���������ͳ�ȥ
***
*** 
***********************************************************
***********************************************************
***/
/**********************************************************
*** ������������
**********************************************************/
// ����������У׼ - ���ص�����ָ����
void X_V2_MMCL_Trig_Encoder_Cal(uint8_t addr);
// ������� - ���ص�����ָ����
void X_V2_MMCL_Reset_Motor(uint8_t addr);
// ����ǰλ������ - ���ص�����ָ����
void X_V2_MMCL_Reset_CurPos_To_Zero(uint8_t addr);
// �����ת���� - ���ص�����ָ����
void X_V2_MMCL_Reset_Clog_Pro(uint8_t addr);
// �ָ��������� - ���ص�����ָ����
void X_V2_MMCL_Restore_Motor(uint8_t addr);
/**********************************************************
*** �˶���������
**********************************************************/
// ���ʹ�ܿ��� - ���ص�����ָ����
void X_V2_MMCL_En_Control(uint8_t addr, bool state, bool snF);
// ����ģʽ���� - ���ص�����ָ����
void X_V2_MMCL_Torque_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, bool snF);
// ����ģʽ���ٿ��ƣ�X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Torque_LV_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, bool snF, float maxVel);
// �ٶ�ģʽ���� - ���ص�����ָ����
void X_V2_MMCL_Vel_Control(uint8_t addr, uint8_t dir, uint16_t acc, float vel, bool snF);
// ֱͨ����λ��ģʽ - ���ص�����ָ����
void X_V2_MMCL_Bypass_Pos_LV_Control(uint8_t addr, uint8_t dir, float vel, float pos, uint8_t raf, bool snF);
// ֱͨ����λ��ģʽ�޵������� - ���ص�����ָ����
void X_V2_MMCL_Bypass_Pos_LV_LC_Control(uint8_t addr, uint8_t dir, float vel, float pos, uint8_t raf, bool snF, uint16_t maxCur);
// �������߼Ӽ���λ��ģʽ���� - ���ص�����ָ����
void X_V2_MMCL_Traj_Pos_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float vel, float pos, uint8_t raf, bool snF);
// �������߼Ӽ���λ��ģʽ�޵������ƣ�X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Traj_Pos_LC_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float vel, float pos, uint8_t raf, bool snF, uint16_t maxCur);
// ���ÿ�����������λ��ģʽ���˶�������X42S/Y42��
void X_V2_MMCL_Set_QTrajPos_Params(uint8_t addr, uint16_t acc, uint16_t dec, float vel, uint8_t raf, bool snF, uint16_t maxCur);
// ������������λ��ģʽ���ƣ�X42S/Y42��
void X_V2_MMCL_QTrajPos_LC_Control(uint8_t addr, float pos);
// �õ������ֹͣ�˶� - ���ص�����ָ����
void X_V2_MMCL_Stop_Now(uint8_t addr, bool snF);
// �������ͬ����ʼ�˶� - ���ص�����ָ����
void X_V2_MMCL_Synchronous_motion(uint8_t addr);
/**********************************************************
*** ԭ���������
**********************************************************/
// ���õ�Ȧ��������λ�� - ���ص�����ָ����
void X_V2_MMCL_Origin_Set_O(uint8_t addr, bool svF);
// �������� - ���ص�����ָ����
void X_V2_MMCL_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
// ǿ���жϲ��˳����� - ���ص�����ָ����
void X_V2_MMCL_Origin_Interrupt(uint8_t addr);
// �޸Ļ������ - ���ص�����ָ����
void X_V2_MMCL_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
// ��ȡ��ײ���㷵�ؽǶȣ�X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Origin_Read_SL_RP(uint8_t addr);
// �޸���ײ���㷵�ؽǶȣ�X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Origin_Modify_SL_RP(uint8_t addr, bool svF, uint16_t sl_rp);
/**********************************************************
*** ��ȡϵͳ��������
**********************************************************/
// ��ʱ������Ϣ���X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms);
// ��ȡϵͳ���� - ���ص�����ָ����
void X_V2_MMCL_Read_Sys_Params(uint8_t addr, SysParams_t s);
/**********************************************************
*** ��д������������
**********************************************************/
// �޸ĵ��ID��ַ - ���ص�����ָ����
void X_V2_MMCL_Modify_Motor_ID(uint8_t addr, bool svF, uint8_t id);
// �޸�ϸ��ֵ - ���ص�����ָ����
void X_V2_MMCL_Modify_MicroStep(uint8_t addr, bool svF, uint8_t mstep);
// �޸ĵ����־ - ���ص�����ָ����
void X_V2_MMCL_Modify_PDFlag(uint8_t addr, bool pdf);
// ��ȡѡ�����״̬��X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Read_Opt_Param_Sta(uint8_t addr);
// �޸ĵ�����ͣ�X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Modify_Motor_Type(uint8_t addr, bool svF, bool mottype);
// �޸Ĺ̼����ͣ�X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Modify_Firmware_Type(uint8_t addr, bool svF, bool fwtype);
// �޸Ŀ���/�ջ�����ģʽ��X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode);
// �޸ĵ���˶�������X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Modify_Motor_Dir(uint8_t addr, bool svF, bool dir);
// �޸������������ܣ�X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Modify_Lock_Btn(uint8_t addr, bool svF, bool lock);
// �޸�����λ�ýǶ��Ƿ������С10�����루X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Modify_S_Vel(uint8_t addr, bool svF, bool s_vel);
// �޸Ŀ���ģʽ�������� - ���ص�����ָ����
void X_V2_MMCL_Modify_OM_mA(uint8_t addr, bool svF, uint16_t om_ma);
// �޸ıջ�ģʽ������ - ���ص�����ָ����
void X_V2_MMCL_Modify_FOC_mA(uint8_t addr, bool svF, uint16_t foc_mA);
// ��ȡPID���� - ���ص�����ָ����
void X_V2_MMCL_Read_PID_Params(uint8_t addr);
// �޸�PID���� - ���ص�����ָ����
void X_V2_MMCL_Modify_PID_Params(uint8_t addr, bool svF, uint32_t pTkp, uint32_t pBkp, uint32_t vkp, uint32_t vki);
// ��ȡDMX512Э�������X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Read_DMX512_Params(uint8_t addr);
// ��ȡDMX512Э�������X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Modify_DMX512_Params(uint8_t addr, bool svF, uint16_t tch, uint8_t nch, uint8_t mode, uint16_t vel, uint16_t acc, uint16_t vel_step, uint32_t pos_step);
// ��ȡλ�õ��ﴰ�ڣ�X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Read_Pos_Window(uint8_t addr);
// �޸�λ�õ��ﴰ�ڣ�X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Modify_Pos_Window(uint8_t addr, bool svF, uint16_t prw);
// ��ȡ���ȹ������������ֵ��X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Read_Otocp(uint8_t addr);
// �޸Ĺ��ȹ������������ֵ��X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Modify_Otocp(uint8_t addr, bool svF, uint16_t otp, uint16_t ocp, uint16_t time_ms);
// ��ȡ������������ʱ�䣨X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Read_Heart_Protect(uint8_t addr);
// �޸�������������ʱ�䣨X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Modify_Heart_Protect(uint8_t addr, bool svF, uint32_t hp);
// ��ȡ�����޷�/����ϵ����X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Read_Integral_Limit(uint8_t addr);
// �޸Ļ����޷�/����ϵ����X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Modify_Integral_Limit(uint8_t addr, bool svF, uint32_t il);
/**********************************************************
*** ��ȡ����������������
**********************************************************/
// ��ȡϵͳ״̬���� - ���ص�����ָ����
void X_V2_MMCL_Read_System_State_Params(uint8_t addr);
// ��ȡ�������ò��� - ���ص�����ָ����
void X_V2_MMCL_Read_Motor_Conf_Params(uint8_t addr);

#endif
