/*
 * relay.c
 *
 *  Created on: 2022年4月28日
 *      Author: robin.zheng
 */

#include <string.h>
#include "DataType.h"
#include "SystemMacVarDef.h"
#include "PubBasicVarDef.h"
#include "DSP2803x_Gpio.h"
#include "DSP2803x_Adc.h"
#include "relay.h"
#include "CanFuncode.h"
#include "CanVcu.h"

extern volatile struct ADC_REGS AdcRegs;
extern volatile struct ADC_RESULT_REGS AdcResult;
extern volatile struct GPIO_DATA_REGS GpioDataRegs;
extern FUNCCODE_ALL_UNI g_FuncCode;

DI_STATUS diStatus;
DO_STATUS doStatus;
int HFlag = 0;

uint16_t scopeDiValue;
uint16_t scopeDoValue[3];
uint16_t aiValue[NUM_AI];       //装AD转换成对应的电压值
uint16_t main_pp[NUM_AI+32];

RELAY_OBJ   g_RelayObj;
RELAY_DEV   relayDebug;
HVB_INFO hvbInfo;
extern VCU_INFO gVcuInfo;

extern int16_t  gScope[11];

extern uint16_t car_ready; // 中通客车XP700专用
/*
const uint16_t channelCfg1[11][4] = {
    //最大值   , 比例系数Kp%,偏移量V,   属性-->Bit0-3，小数位     Bit4-7,单位,0-电压  1-电流 2-温度 3-转速 4-压力
    10500, 102, 30, 0x01,       // 0 电压值 0.1V/bit -2 VIN1
    10500, 102, 30, 0x01,       // 0 电压值 0.1V/bit -2 VIN1
    10500, 102, 30, 0x01,       // 1 电压值0.1V/bit   VIN2
    10500, 102, 30, 0x01,       // 2 电压值0.1V/bit   VIN3
      500, 100,  0, 0x11,       // 3 电流值0.1A/bit 0-5V  BATT1
      500, 100,  0, 0x11,       // 4 电流值0.1A/bit 0-5V  BATT2
    10500, 102, 30, 0x01,       // 5 电压值0.1V/bit   VIN4
    10500, 102, 30, 0x01,       // 6 电压值0.1V/bit   VIN5
    10500, 102, 30, 0x01,       // 7 电压值0.1V/bit   VIN6
    10500, 102, 30, 0x01,       // 8 电压值0.1V/bit   VIN7
    10500, 102, 30, 0x01,       // 9 电压值0.1V/bit
};
*/

//参数初始化
void RalayParaInit(void)
{
    static    uint16_t i = 0;

    RELAY_DEV *p_relay = &g_RelayObj.MCU1;
    uint16_t  *par;

    //更新状态字

    g_RelayObj.faultStateWord &= 0x7FFF;
    g_RelayObj.faultStateWord |= hvbInfo.interLockFault<<15;

    g_RelayObj.faultStateWord &= ~(0x0001<<i);
    g_RelayObj.faultStateWord |= (p_relay[i].fault<<i);

    g_RelayObj.cmdStateWord &= ~(0x0001<<i);
    g_RelayObj.cmdStateWord |= (p_relay[i].cmd<<i);

//    g_RelayObj.vcuCmdWord   &= ~(0x0001<<i);
    g_RelayObj.vcuCmdWord   = ((uint32_t)g_RelayObj.MCU1.cmdVcu<<0)|((uint32_t)g_RelayObj.MCU2.cmdVcu<<1)|((uint32_t)g_RelayObj.ManyInOne.cmdVcu<<2)
                              |((uint32_t)g_RelayObj.Air.cmdVcu<<3)|((uint32_t)g_RelayObj.Aux1.cmdVcu<<4)|((uint32_t)g_RelayObj.Aux2.cmdVcu<<5)
                              |((uint32_t)g_RelayObj.PTC.cmdVcu<<6)|((uint32_t)g_RelayObj.Defroster.cmdVcu<<7)|((uint32_t)g_RelayObj.Spare1.cmdVcu<<8)
                              |((uint32_t)g_RelayObj.Spare2.cmdVcu<<9)|((uint32_t)g_RelayObj.Spare3.cmdVcu<<10)|((uint32_t)g_RelayObj.MCU1_PP.cmdVcu<<11)
                              |((uint32_t)g_RelayObj.MCU2_PP.cmdVcu<<12)|((uint32_t)g_RelayObj.ManyInOne_PP.cmdVcu<<13)|((uint32_t)g_RelayObj.Air_PP.cmdVcu<<14)
                              |((uint32_t)g_RelayObj.Aux1_PP.cmdVcu<<15)|((uint32_t)g_RelayObj.Aux2_PP.cmdVcu<<16);

    if(i<NUM_RELAY)
    {
        //        p_relay += i;
        par = &g_FuncCode.Group.F02[0];

        //更新功能输入的参数
        if(i < NUM_MAIN_RELAY)      //i<6
        {
            p_relay[i].cfg.relayMainPort      = par[i*10 + 0];
            p_relay[i].cfg.voltFrontPort      = par[i*10 + 1];
            p_relay[i].cfg.voltBackPort       = par[i*10 + 2];
            p_relay[i].cfg.adhParaVolt        = par[i*10 + 3];
            p_relay[i].cfg.adhParaTime        = par[i*10 + 4];
            p_relay[i].cfg.relayPrePort       = 0;
            p_relay[i].cfg.relayPreParaSel    = par[i*10 + 6];

            if(p_relay[i].cfg.relayPreParaSel !=3)
                p_relay[i].cfg.relayPrePort       = par[i*10 + 5];
        }
        else if(i < (NUM_MAIN_RELAY+2))  //i<8
        {
            p_relay[i].cfg.relayMainPort      = g_FuncCode.Group.F05[(i-NUM_MAIN_RELAY)*3 + 2 + 0];
            p_relay[i].cfg.voltFrontPort      = g_FuncCode.Group.F05[(i-NUM_MAIN_RELAY)*3 + 2 + 1];
            p_relay[i].cfg.voltBackPort       = g_FuncCode.Group.F05[(i-NUM_MAIN_RELAY)*3 + 2 + 2];
            p_relay[i].cfg.adhParaVolt        = g_FuncCode.Group.F05[0];
            p_relay[i].cfg.adhParaTime        = g_FuncCode.Group.F05[1];
            p_relay[i].cfg.relayPrePort       = 0;
            p_relay[i].cfg.relayPreParaSel    = 0;
        }
        else if(i < (NUM_MAIN_RELAY+5))
        {
            p_relay[i].cfg.relayMainPort      = g_FuncCode.Group.F05[(i-NUM_MAIN_RELAY-2)*3 + 10 + 0];
            p_relay[i].cfg.voltFrontPort      = g_FuncCode.Group.F05[(i-NUM_MAIN_RELAY-2)*3 + 10 + 1];
            p_relay[i].cfg.voltBackPort       = g_FuncCode.Group.F05[(i-NUM_MAIN_RELAY-2)*3 + 10 + 2];
            p_relay[i].cfg.adhParaVolt        = g_FuncCode.Group.F05[8];
            p_relay[i].cfg.adhParaTime        = g_FuncCode.Group.F05[9];
            p_relay[i].cfg.relayPrePort       = 0;
            p_relay[i].cfg.relayPreParaSel    = 0;
        }
        else
        {
            p_relay[i].cfg.relayMainPort      = par[(i-11)*10 + 0];
            p_relay[i].cfg.voltFrontPort      = par[(i-11)*10 + 1];
            p_relay[i].cfg.voltBackPort       = par[(i-11)*10 + 2];
            p_relay[i].cfg.adhParaVolt        = par[(i-11)*10 + 3];
            p_relay[i].cfg.adhParaTime        = par[(i-11)*10 + 4];
            p_relay[i].cfg.relayPrePort       = par[(i-11)*10 + 5];
            p_relay[i].cfg.relayPreParaSel    = par[(i-11)*10 + 6];
        }

        //更新预充参数
        if(p_relay[i].cfg.relayPreParaSel == 1)
        {
            p_relay[i].PrePara =   (RELAY_PRECHARGE_PARAMETER *)g_FuncCode.Code.PreCharge1;
        }
        else if(p_relay[i].cfg.relayPreParaSel == 2)
        {
            p_relay[i].PrePara =   (RELAY_PRECHARGE_PARAMETER *)g_FuncCode.Code.PreCharge2;
        }
        else if(p_relay[i].cfg.relayPreParaSel == 3)
        {
            p_relay[i].PrePara =   (RELAY_PRECHARGE_PARAMETER *)g_FuncCode.Code.PreCharge3;
        }
        else
        {
            p_relay[i].PrePara =   (RELAY_PRECHARGE_PARAMETER *)g_FuncCode.Code.PreCharge0;
        }
        i++;
        if( i>= NUM_RELAY)i = 0;
    }

}


uint16_t RelayIsOpen(void)
{
    return (doStatus.value & 0xff) ? 0 : 1;
}

void UpdateDi(void)
{
//    static uint16_t filter[NUM_DI];
//    DI_STATUS status;
//    int tmp, i;

    static uint16_t dif[7] = {0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF};

    if(aiValue[4]>5000)
    {
        dif[0]  = (dif[0]<<1)| GpioDataRegs.GPADAT.bit.GPIO0; // di1
        dif[1]  = (dif[1]<<1)| GpioDataRegs.GPADAT.bit.GPIO1; // di2
        dif[2]  = (dif[2]<<1)| GpioDataRegs.GPADAT.bit.GPIO3; // di3
        dif[3]  = (dif[3]<<1)| GpioDataRegs.GPADAT.bit.GPIO5; // di4
    }
    else
    {
        dif[0]  = (dif[0]<<1)| GpioDataRegs.GPADAT.bit.GPIO16; // di1
        dif[1]  = (dif[1]<<1)| GpioDataRegs.GPADAT.bit.GPIO17; // di2
        dif[2]  = (dif[2]<<1)| GpioDataRegs.GPADAT.bit.GPIO18; // di3
        dif[3]  = (dif[3]<<1)| GpioDataRegs.GPADAT.bit.GPIO29; // di4
        dif[4]  = (dif[4]<<1)| GpioDataRegs.GPADAT.bit.GPIO28; // di5
        dif[5]  = (dif[5]<<1)| GpioDataRegs.GPADAT.bit.GPIO0 ; // di6 备用
        dif[6]  = (dif[6]<<1)| GpioDataRegs.GPADAT.bit.GPIO5 ; // di7 上装
    }

    if(dif[0] == 0xFFFF)        diStatus.bits.di1 = 1;
    else if(dif[0] == 0x0)      diStatus.bits.di1 = 0;

    if(dif[1] == 0xFFFF)        diStatus.bits.di2 = 1;
    else if(dif[1] == 0x0)      diStatus.bits.di2 = 0;

    if(dif[2] == 0xFFFF)        diStatus.bits.di3 = 1;
    else if(dif[2] == 0x0)      diStatus.bits.di3 = 0;

    if(dif[3] == 0xFFFF)        diStatus.bits.di4 = 1;
    else if(dif[3] == 0x0)      diStatus.bits.di4 = 0;

    if(dif[4] == 0xFFFF)        diStatus.bits.di5 = 1;
    else if(dif[4] == 0x0)      diStatus.bits.di5 = 0;

    if(dif[5] == 0xFFFF)        diStatus.bits.di6 = 1;
    else if(dif[5] == 0x0)      diStatus.bits.di6 = 0;

    if(dif[6] == 0xFFFF)        diStatus.bits.di7 = 1;
    else if(dif[6] == 0x0)      diStatus.bits.di7 = 0;

//        gScope[0] = aiValue[4];
//        gScope[1] = GpioDataRegs.GPADAT.bit.GPIO17;
//        gScope[2] = GpioDataRegs.GPADAT.bit.GPIO18;
//    status.value = 0;
//    status.bits.di1 = GpioDataRegs.GPADAT.bit.GPIO16;
//    status.bits.di2 = GpioDataRegs.GPADAT.bit.GPIO17;
//    status.bits.di3 = GpioDataRegs.GPADAT.bit.GPIO18;
//    status.bits.di4 = GpioDataRegs.GPADAT.bit.GPIO29;
//    status.bits.di5 = GpioDataRegs.GPADAT.bit.GPIO28;

    //status.value = ~status.value;
//#if 0
//    for (i = NUM_DI - 1; i >= 0; i--) {
//        tmp = 0x01UL << i;
//        if ((diStatus.value ^ (status.value)) & tmp) {
//            if(++filter[i] >= 20) {
//                diStatus.value ^= tmp;
//                filter[i] = 0;
//            } else
//               filter[i] = 0;
//        }
//    }
//#else
//    diStatus.value = status.value;
//#endif

    scopeDiValue = diStatus.bits.di1
                + diStatus.bits.di2 * 10
                + diStatus.bits.di3 * 100
                + diStatus.bits.di4 * 1000
                + diStatus.bits.di5 * 10000;

    return;
}

void UpdateDO(void)
{
    struct GPIO_DATA_REGS regShadow;

    regShadow.GPADAT.all = GpioDataRegs.GPADAT.all;
    regShadow.GPADAT.bit.GPIO20 = doStatus.bits.do1;
    regShadow.GPADAT.bit.GPIO21 = doStatus.bits.do2;
    regShadow.GPADAT.bit.GPIO22 = doStatus.bits.do3;
    regShadow.GPADAT.bit.GPIO23 = doStatus.bits.do4;
    regShadow.GPADAT.bit.GPIO8  = doStatus.bits.do5;
    regShadow.GPADAT.bit.GPIO9  = doStatus.bits.do6;
    regShadow.GPADAT.bit.GPIO10 = doStatus.bits.do7;
    regShadow.GPADAT.bit.GPIO11 = doStatus.bits.do8;
    regShadow.GPADAT.bit.GPIO12 = doStatus.bits.do9;

    regShadow.GPADAT.bit.GPIO4 = doStatus.bits.do10;
    regShadow.GPADAT.bit.GPIO6 = doStatus.bits.do11;
    regShadow.GPADAT.bit.GPIO7 = doStatus.bits.do12;

    regShadow.GPADAT.bit.GPIO19 = 0;
    GpioDataRegs.GPADAT.all = regShadow.GPADAT.all;

    scopeDoValue[0] = doStatus.bits.do1
                + doStatus.bits.do2 * 10
                + doStatus.bits.do3 * 100
                + doStatus.bits.do4 * 1000
                + doStatus.bits.do5 * 10000;

    scopeDoValue[1] = doStatus.bits.do6
                + doStatus.bits.do7 * 10
                + doStatus.bits.do8 * 100
                + doStatus.bits.do9 * 1000;


    scopeDoValue[2] =  doStatus.bits.do10
                + doStatus.bits.do11 * 10
                + doStatus.bits.do12 * 100;


    return;
}

void do_action(uint16_t idx, uint16_t bAction)
{

    if(idx >=1 && idx <= NUM_DO)
    {
        idx = idx-1;
        if(bAction)
            doStatus.value |= (1 << idx);
        else
            doStatus.value &= (~(1 << idx));
    }
}

uint16_t di_get(uint16_t idx)
{
    if(idx < NUM_DI)
        return (diStatus.value & (1 << idx)) ? 1 : 0;
    else
        return 0;
}

/*
HVB_VERSION_2.0:

    0  ADCINA0  DSP_ADC0    3V采样
       ADCINA1
    1  ADCINA2 DSP_ADC2    U5_TJ10
    2  ADCINA3 DSP_ADC3    U6
    3  ADCINA4 DSP_ADC4    U7_TJ5_TJ7
       ADCINA5
    4  ADCINA6 DSP_ADC5    IBATT1
    5  ADCINA7 DSP_ADC6    IBATT2
       ADCINB0
       ADCINB1
    6  ADCINB2 DSP_ADC11   U8
    7  ADCINB3 DSP_ADC10   U1_VBATT
    8  ADCINB4 DSP_ADC9    U2_TJ8_TJ5A
       ADCINB5
    9  ADCINB6 DSP_ADC8    U3_TJ6
    10 ADCINB7 DSP_ADC7    U4_TJ9

HVB_VERSION_2.5:                    licht.he 2022.10.20

    序号              IO            网络标号            GS-Kit/控制板座子对应序号
    0       ADCINA0         REF_+3V             3V采样
    13      ADCINA1         DSP_ADC15           U12
    1       ADCINA2         DSP_ADC2            U5
    2       ADCINA3         DSP_ADC3            U6
    3       ADCINA4         DSP_ADC4            U7
            ADCINA5
    4       ADCINA6         DSP_HWVER           版本辨识电压采样
    5       ADCINA7         DSP_ADC12           U9
    11      ADCINB0         DSP_ADC13           U10
    12      ADCINB1         DSP_ADC14           U11
    6       ADCINB2         DSP_ADC11           U8
    7       ADCINB3         DSP_ADC10           U1
    8       ADCINB4         DSP_ADC9            U2
            ADCINB5
    9       ADCINB6         DSP_ADC8            U3
    10      ADCINB7         DSP_ADC7            U4
 */


const uint16_t channelCfg[14][2] = {
    // zeroDrift   coefficient
    {0, 2645,},         // 3v
    {0, 2645,},         // U5
    {0, 2645,},         // U6
    {0, 2645,},         // U7
    {0, 2645,},         // 版本辨识电压采样
    {0, 2645,},         // U9
    {0, 2645,},         // U8
    {0, 2645,},         // U1
    {0, 2645,},         // U2
    {0, 2645,},         // U3
    {0, 2645,},         // U4

    {0, 2645,},         // U10
    {0, 2645,},         // U11
    {0, 2645,},         // U12
};


uint16_t ADConvert(uint16_t ad, uint16_t adZeroDrift, uint16_t coefficient, uint16_t ch)
{
    uint32_t t;
    if(ad < adZeroDrift)
        t = 0;
    else
        t = ad - adZeroDrift;
    t = (t * coefficient) >> 10;
    return t;
}

void UpdateAd(void)
{
    static uint16_t cnt;
    uint16_t t;

#if 0
    static uint16_t div;
    if(++div < 10)               // 2 * 2 = 4ms sample
        return;
    div = 0;
#endif

    if(AdcRegs.ADCINTFLG.bit.ADCINT1 == 0)
    {
        if(++cnt > 2)
        {
            AdcRegs.ADCSOCFRC1.all = 0xFFFF;                        //0xDCDD // 0xD8DC;// use V8 input
            cnt = 0;
        }
    }
    else
    {
        AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
        t = ADConvert(AdcResult.ADCRESULT0, channelCfg[0][0], channelCfg[0][1], 0);
        aiValue[0] = Flt16(t, aiValue[0]);      // ai1 3V--->U
        t = ADConvert(AdcResult.ADCRESULT2, channelCfg[1][0], channelCfg[1][1], 1);
        aiValue[1] = Flt16(t, aiValue[1]);      // ai2 --->U5
        t = ADConvert(AdcResult.ADCRESULT3, channelCfg[2][0], channelCfg[2][1], 2);
        aiValue[2] = Flt16(t, aiValue[2]);      // ai3 --->U6
        t = ADConvert(AdcResult.ADCRESULT4, channelCfg[3][0], channelCfg[3][1], 3);
        aiValue[3] = Flt16(t, aiValue[3]);      // ai4 --->U7
        t = ADConvert(AdcResult.ADCRESULT6, channelCfg[4][0], channelCfg[4][1], 4);
        aiValue[4] = Flt16(t, aiValue[4]);      // ai5 --->VERSION_VOLT
        t = ADConvert(AdcResult.ADCRESULT7, channelCfg[5][0], channelCfg[5][1], 5);
        aiValue[5] = Flt16(t, aiValue[5]);      // ai6 --->U9
        t = ADConvert(AdcResult.ADCRESULT10, channelCfg[6][0], channelCfg[6][1], 6);
        aiValue[6] = Flt16(t, aiValue[6]);      // ai7--->U8
        t = ADConvert(AdcResult.ADCRESULT11, channelCfg[7][0], channelCfg[7][1], 7);
        aiValue[7] = Flt16(t, aiValue[7]);      // ai8 --->U1
        t = ADConvert(AdcResult.ADCRESULT12, channelCfg[8][0], channelCfg[8][1], 8);
        aiValue[8] = Flt16(t, aiValue[8]);      // ai9 --->U2
        t = ADConvert(AdcResult.ADCRESULT14, channelCfg[9][0], channelCfg[9][1], 9);
        aiValue[9] = Flt16(t, aiValue[9]);      // ai10 --->U3
        t = ADConvert(AdcResult.ADCRESULT15, channelCfg[10][0], channelCfg[10][1], 10);
        aiValue[10] = Flt16(t, aiValue[10]);  // ai11 --->U4

        //HVB_VERSION_2.5
        t = ADConvert(AdcResult.ADCRESULT8, channelCfg[11][0], channelCfg[11][1], 11);
        aiValue[11] = Flt16(t, aiValue[11]);  // ai12 --->U10
        t = ADConvert(AdcResult.ADCRESULT9, channelCfg[12][0], channelCfg[12][1], 12);
        aiValue[12] = Flt16(t, aiValue[12]);   // ai13 --->U11
        t = ADConvert(AdcResult.ADCRESULT1, channelCfg[13][0], channelCfg[13][1], 13);
        aiValue[13] = Flt16(t, aiValue[13]);  // ai14 --->U12

        AdcRegs.ADCSOCFRC1.all = 0xFFFF;
    }

//    gScope[1] = AdcResult.ADCRESULT14;
//    gScope[0] = AdcResult.ADCRESULT6;
//    gScope[2] = aiValue[4];
//    gScope[3] = aiValue[9];


}

void PanelDo(uint16_t idx, uint16_t action)
{
    do_action(idx, action);
}



// 0 主正采样
// 2 主正预充
// 采样 3，6

// 3 三合一预充
// 6 三合一主接
// 采样 1，2


void RelayInit(void)
{
    uint16_t i;
    RELAY_DEV *p_relay = &g_RelayObj.MCU1;

    //memset(&g_RelayObj, 0, sizeof(g_RelayObj));
    for(i=0; i<NUM_RELAY; i++)
    {
        p_relay[i].subStatus = RELAY_STATUS_OPEN_OK;
    }

    diStatus.value = 0;
}

void RelayConfigUpdate(void)
{
//    static uint16_t tick;
//    uint16_t i;
//    uint16_t *p;
//    RELAY_CFG *r;
//
//    if(++tick > TIME_1S_LOOP_2MS) {
//        tick = 0;
//        p = &g_FuncCode.Code.relay1type;
//        for(i=0; i<NUM_RELAY; i++) {
//            r = &relay[i].cfg;
//            r->type = *p++;
//            r->voltIdxBatt = *p++;
//            r->voltIdxRight = *p++;
//            r->relayRef = *p++;
//        }
//    }
}


void RelayDebugUpdate(void)
{
    uint16_t idx;
    RELAY_DEV *r = &g_RelayObj.MCU1;

    idx = g_FuncCode.Code.debugRelayIdx;
    if(idx >= NUM_RELAY)    idx = 0;

    relayDebug.cmdVcu               = r[idx].cmdVcu;                      // 50 vcu命令
    relayDebug.status               = r[idx].status;                      // 51 继电器状态
//        relayDebug.battReady            = r->battReady;                   // 52 电池电压正常
    relayDebug.voltageBeforeClose   = r[idx].voltageBeforeClose;          // 53 继电器闭合前后端电压
    relayDebug.tick                 = r[idx].tick;                        // 54 状态切换计时器
    relayDebug.tickEvent            = r[idx].tickEvent;                   // 55 事件计时器
    relayDebug.adh.diffSum          = r[idx].adh.diffSum;                 // 56
    relayDebug.adh.diffStatic       = r[idx].adh.diffStatic;              // 57 闭合后左右静态偏差
    relayDebug.adh.diffCalCnt       = r[idx].adh.diffCalCnt;              // 58
    relayDebug.adh.diffAdhCnt       = r[idx].adh.diffAdhCnt;              // 59
    relayDebug.adh.diffReal         = r[idx].adh.diffReal;                // 60 实时压差

    relayDebug.cfg.relayMainPort    =  r[idx].cfg.relayMainPort;
    relayDebug.cfg.voltFrontPort    =  r[idx].cfg.voltFrontPort;
    relayDebug.cfg.voltBackPort     =  r[idx].cfg.voltBackPort;
    relayDebug.cfg.adhParaVolt      =  r[idx].cfg.adhParaVolt;
    relayDebug.cfg.adhParaTime      =  r[idx].cfg.adhParaTime;
    relayDebug.cfg.relayPrePort     =  r[idx].cfg.relayPrePort;
    relayDebug.cfg.relayPreParaSel  =  r[idx].cfg.relayPreParaSel;

    relayDebug.PrePara              =  r[idx].PrePara;



    /*
    gScope[0] =  relayDebug.cfg.relayMainPort;
    gScope[1] =  relayDebug.cfg.voltFrontPort;
    gScope[2] =  relayDebug.cfg.voltBackPort;
    gScope[3] =  relayDebug.cfg.adhParaVolt;
    gScope[4] =  relayDebug.cfg.adhParaTime;
    gScope[5] =  relayDebug.cfg.relayPrePort;
    gScope[6] =  relayDebug.cfg.relayPreParaSel;

    gScope[7] =  relayDebug.PrePara->preChargeOverLapTime;
    gScope[8] =  relayDebug.PrePara->preChargeLoadShortTime;
    */
}

//根据继电器配置参数，将读取的电压配置到对应的继电器组
void RelayVoltageUpdate(RELAY_DEV *r)
{
    RELAY_CFG *cfg = &r->cfg;

    if(cfg->voltFrontPort < NUM_AI)
        r->voltageFront = aiValue[cfg->voltFrontPort];
    else
        r->voltageFront = 0;
    if(cfg->voltBackPort < NUM_AI)
        r->voltageBack = aiValue[cfg->voltBackPort];
    else
        r->voltageBack = 0;
}

//用主驱1检查电池电压 欠压和过压
void RelayBattCheck(void)
{
    uint16_t v1, v2, aisel;

    aisel = Min(g_FuncCode.Code.UdcPortSel,NUM_AI);
    g_RelayObj.battVolt = aiValue[aisel];

    if(g_RelayObj.battReady) {
        if(g_RelayObj.battVolt < g_FuncCode.Code.voltageUnder || g_RelayObj.battVolt > g_FuncCode.Code.voltageOver)
        {
            v1 = ((uint32_t)g_FuncCode.Code.voltageUnder * 7) >> 3;
            v2 = ((uint32_t)g_FuncCode.Code.voltageOver * 9) >> 3;
            if(g_RelayObj.battVolt < v1 || g_RelayObj.battVolt > v2)  // sudden death
                g_RelayObj.tickBatt = TIME_50MS_LOOP_2MS;
            if(++g_RelayObj.tickBatt > TIME_50MS_LOOP_2MS)
            {
                g_RelayObj.tickBatt     = 0;
                g_RelayObj.battReady    = 0;
            }
        }
        else
        {
            g_RelayObj.tickBatt = 0;
        }
    }
    else
    {
        if(g_RelayObj.battVolt > g_FuncCode.Code.voltageUnder && g_RelayObj.battVolt < g_FuncCode.Code.voltageOver)
        {
            if(++g_RelayObj.tickBatt > TIME_500MS_LOOP_2MS)
            {
                g_RelayObj.tickBatt     = 0;
                g_RelayObj.battReady    = 1;
            }
        }
        else
        {
            g_RelayObj.tickBatt = 0;
        }
    }
}


//检查VCU 指令，更新继电器状态
void RelayCmdCheck(RELAY_DEV *r)
{
    uint16_t ready = 1;

    if(r->cmdVcu == RELAY_CMD_OPEN)
    {
        if(r->cmd != RELAY_CMD_OPEN) {
            // other conditions
            r->cmd = ready ? RELAY_CMD_OPEN : RELAY_CMD_CLOSE;
            if(r->cmd == RELAY_CMD_OPEN)
            {
                r->status       = RELAY_STATUS_OPENING;
                r->subStatus    = RELAY_STATUS_OPEN_INIT;
            }
        }
    }
    else
    {
        if(r->cmd != RELAY_CMD_CLOSE)
        {
            if(!g_RelayObj.battReady)
                ready = 0;
            if(r->status == RELAY_STATUS_ERROR)
                ready = 0;
            // other conditions

            r->cmd = ready ? RELAY_CMD_CLOSE : RELAY_CMD_OPEN;
            if(r->cmd == RELAY_CMD_CLOSE)
            {
                r->status       = RELAY_STATUS_CLOSING;
                r->subStatus    = RELAY_STATUS_CLOSE_INIT;
            }
        }
        else if(!g_RelayObj.battReady)
        {
            r->cmd          = RELAY_CMD_OPEN;
            r->status       = RELAY_STATUS_OPENING;
            r->subStatus    = RELAY_STATUS_OPEN_INIT;
            r->tick         = 0;
        }
    }
}

//上电粘连检测
uint16_t RelayPoweronAdhCheck(RELAY_DEV *r)
{
    if(r->tick == 0)                    r->tickEvent = 0;

    if(r->tick > TIME_200MS_LOOP_2MS)           return 1;
    if(g_FuncCode.Code.powerUpcheckVolt == 0)   return 1;

    if(abs(r->voltageBack - r->voltageFront) < g_FuncCode.Code.powerUpcheckVolt)
    {
        if(++r->tickEvent > TIME_100MS_LOOP_2MS)
        {
            r->fault = RELAY_ERROR_POWERUP_ADH;
            r->status = RELAY_STATUS_ERROR;
            r->subStatus = 0;
            r->tick = TIME_100MS_LOOP_2MS;
            return 1;
        }
    }
    else
    {
        r->tickEvent = 0;
    }


    return 0;
}

//执行闭合命令函数
void RelayCmdClose(RELAY_DEV *r)
{
    uint32_t t;
//    RELAY_DEV                   *rm;
    RELAY_CFG                   *cfg = &r->cfg;
    RELAY_PRECHARGE_PARAMETER   *pre = r->PrePara;

    //未上高压
    if(!g_RelayObj.battReady) {
        r->cmd = RELAY_CMD_OPEN;    //断开
        return;
    }


    switch(r->subStatus) {
    case RELAY_STATUS_CLOSE_INIT:       //闭合初始化，
        if(cfg->relayPrePort != 0)      //有预充功能
        {
            r->voltageBeforeClose = r->voltageBack;
            t = (uint32_t)r->voltageFront * pre->preChargeVoltageBeforePer / 100;
            if(r->voltageBack > t)
                r->flag = 1;            // 不完整预充
            else
                r->flag = 0;            // 完整预充

            do_action(cfg->relayPrePort, 1);        //闭合预充继电器
            r->doAction        |= 0x0002;
            r->status           = RELAY_STATUS_CLOSING;     //主步骤正在闭合
            r->subStatus        = RELAY_STATUS_CLOSE_DOING; //跳到正在预充状态
        }
        else
        {
            if(main_pp[r->cfg.relayMainPort+32]==6)
            {
                r->fault     = RELAY_ERROR_LOAD_SHORTCIRCUIT_PORT_DAMAGE;
                r->status    = RELAY_STATUS_ERROR;
                r->subStatus = RELAY_STATUS_CLOSE_SHORTCIRCUIT;
                r->tick      = 0;

                do_action(r->cfg.relayMainPort, 0);
                do_action(r->cfg.relayPrePort, 0);
                r->doAction    = 0;
            }
            else if(main_pp[r->cfg.relayMainPort+32]==4)
            {
                r->fault = RELAY_ERROR_CLOSE_TIMEOUT;
                break;
            }
            r->status           = RELAY_STATUS_CLOSING;     //主步骤正在闭合
            r->subStatus        = RELAY_STATUS_CLOSE_TAIL;  //闭合主接
        }

        r->tick                 = 0;

        break;

    case RELAY_STATUS_CLOSE_DOING:      //正在预充
        if(r->flag == 0)
        {
            //完整预充
            if(abs(r->voltageBack - r->voltageBeforeClose) < (r->PrePara->preChargeLoadShortVolt) && (r->tick) > (r->PrePara->preChargeLoadShortTime>>1))
            { //判断负载短路 有两个参数
                r->fault     = RELAY_ERROR_LOAD_SHORTCIRCUIT_PORT_DAMAGE;
                r->status    = RELAY_STATUS_ERROR;
                r->subStatus = RELAY_STATUS_CLOSE_SHORTCIRCUIT;
                r->tick      = 0;

                do_action(r->cfg.relayMainPort, 0);
                do_action(r->cfg.relayPrePort, 0);
                r->doAction    = 0;
            }
            else
            {
                t = (uint32_t)r->voltageFront * pre->preChargeReadyVoltageFullPer / 100;    //预充完成
                if(r->voltageBack > t)
               {
                    if(cfg->relayPreParaSel == 3)
                    {
                        main_pp[r->cfg.relayMainPort+15] = r->cfg.relayMainPort;
                        r->subStatus =  RELAY_STATUS_WAIT_CLOSE;
                        r->tick      = 0;
                    }
                    else
                   {
                        r->subStatus = RELAY_STATUS_CLOSE_TAIL;
                        r->tick      = 0;                                                   // 原版注释该语句
                    }
                }
                else if(r->tick > (pre->preChargeTimeoutFull >> 1))
                {                   //预充超时
                    r->subStatus = RELAY_STATUS_CLOSE_TIMEOUT;
                    r->tick      = 0;
                    gScope[0]=1;
                }
            }
        }
        else
        {                          //不完整预充
            t = (uint32_t)r->voltageFront * pre->preChargeReadyVoltageHalflPer / 100;
            if(r->voltageBack > t)
               {
                    if(cfg->relayPreParaSel == 3)
                    {
                        main_pp[r->cfg.relayMainPort+15] = r->cfg.relayMainPort;
                        r->subStatus = RELAY_STATUS_WAIT_CLOSE;
                        r->tick      = 0;
                    }
                    else
                   {
                        r->subStatus = RELAY_STATUS_CLOSE_TAIL;
                        r->tick      = 0;                                                   // 原版注释该语句
                    }
                }
            else if(r->tick > (pre->preChargeTimeoutHalf >> 1))
            {
                r->subStatus = RELAY_STATUS_CLOSE_TIMEOUT;
                r->tick      = 0;
                gScope[0]=2;
            }
        }
        break;

    case RELAY_STATUS_CLOSE_TAIL:       //闭合主接
        if(cfg->relayPreParaSel == 3)
        {
            if(cfg->relayPrePort == 0 && main_pp[r->cfg.relayMainPort+15] == r->cfg.relayMainPort)
            {
                main_pp[r->cfg.relayMainPort] = r->cfg.relayMainPort;
            }
            else
                break;
        }
        if((r->tick < (pre->preChargeMinTime >> 1))&&(cfg->relayPrePort != 0)) return;  //有预充功能的 判断最小预充时间
        r->tick         = 0;
        r->status       = RELAY_STATUS_CLOSING;
        r->subStatus    = RELAY_STATUS_CLOSE_OVERLAP;
        do_action(r->cfg.relayMainPort, 1);
        r->doAction    |= 0x0001;
        break;

    case RELAY_STATUS_CLOSE_OVERLAP:
        if(r->tick > (pre->preChargeOverLapTime >> 1))
        {
            do_action(cfg->relayPrePort, 0);
            r->doAction     &= 0x0001;
            r->tick         = 0;
            r->subStatus    = RELAY_STATUS_CLOSE_OK;
            r->status       = RELAY_STATUS_CLOSED;
            //r->tryCnt       = 0;
        }
        break;

    case RELAY_STATUS_CLOSE_OK:                 //在闭合时做粘连判断
        main_pp[r->cfg.relayMainPort+32] = 0;
        if(cfg->relayPreParaSel == 3 && pre->preChargeProtectTime != 0)
        {
            if(r->tick > (pre->preChargeProtectTime >> 1))
            {
            do_action(r->cfg.relayPrePort, 0);
            r->tick      = 0;
            r->doAction    = 0;
            }
        }
        OpenAdhesionTrace(r);                   //闭合时，记录静太压差
        Closedetectionopencircuit(r);

        break;

    case RELAY_STATUS_CLOSE_TIMEOUT:            // close timeout
        do_action(cfg->relayMainPort, 0);
        do_action(cfg->relayPrePort, 0);
        r->doAction     = 0;

        if(r->tick > (pre->preChargeRetryTime >> 1) && r->tryCnt < pre->preChargeRetryCnt)
        {
            r->tick = 0;
            r->tryCnt ++;
            r->subStatus    = RELAY_STATUS_CLOSE_INIT;
            main_pp[r->cfg.relayMainPort+32] = 0;
        }
        else if(r->tick <= (pre->preChargeRetryTime >> 1))
        {
            gScope[0]=3;
        }
        else
        {
            r->fault = RELAY_ERROR_CLOSE_TIMEOUT;
            main_pp[r->cfg.relayMainPort+32] = 4;
        }
        break;

    case  RELAY_STATUS_WAIT_CLOSE:    //等待主接
        if(r->tick < (pre->preChargeOverLapTime >> 1))
        {
            if(r->voltageBack <r->voltageFront*0.9)
            {
                if(r->voltageBack <= (uint32_t)r->voltageFront *pre->relayWait/100 )
                {
                    gScope[0]=4;
                    gScope[1]=(uint32_t)r->voltageFront *pre->relayWait/100;
                    gScope[2]=r->voltageBack;
                    main_pp[r->cfg.relayMainPort+32] = 6;
                    do_action(r->cfg.relayMainPort, 0);
                    do_action(r->cfg.relayPrePort, 0);
                    r->fault     = RELAY_ERROR_LOAD_SHORTCIRCUIT_PORT_DAMAGE;
                    r->status    = RELAY_STATUS_ERROR;
                    r->subStatus = RELAY_STATUS_CLOSE_SHORTCIRCUIT;
                    r->tick      = 0;
                    r->doAction    = 0;
                }
                else if(r->voltageBack > (uint32_t)r->voltageFront *pre->relayWait/100)
                {
                    gScope[0]=5;
                    gScope[1]=r->voltageFront;
                    gScope[2]=r->voltageBack;
                    do_action(r->cfg.relayMainPort, 0);
                    do_action(r->cfg.relayPrePort, 0);
                    r->subStatus = RELAY_STATUS_CLOSE_TIMEOUT;
                    r->tick      = 0;
                }
            }
            else
            {
                Closedetectionopencircuit(r);
                if(main_pp[r->cfg.relayMainPort] == r->cfg.relayMainPort)
                {
                    r->tick         = 0;
                    r->subStatus    = RELAY_STATUS_CLOSE_OK;
                }
            }
        }
        else if(r->tick >= (pre->preChargeOverLapTime >> 1))
        {
            gScope[0]=6;
            r->subStatus = RELAY_STATUS_CLOSE_TIMEOUT;
            r->tick      = 0;
        }
        break;
    }
}

//断开继电器时执行
void RelayCmdOpen(RELAY_DEV *r)
{
    RELAY_CFG *cfg = &r->cfg;

    switch(r->subStatus)
    {
    case RELAY_STATUS_OPEN_INIT:
        if(cfg->relayPrePort != 0 )   //如果使用了预充继电器
        {
            do_action(cfg->relayPrePort, 0);
            r->doAction  &= 0x0001;
            if(cfg->relayPreParaSel == 3)
            {
                main_pp[r->cfg.relayMainPort+15] = 0;
                r->tick         = 0;
                r->status       = RELAY_STATUS_OPENING;
                r->subStatus    = RELAY_STATUS_OPEN_DOING;
                r->tickEvent    = 0;
                r->tryCnt       = 0;
                break;
            }
        }
        do_action(cfg->relayMainPort, 0);
        main_pp[r->cfg.relayMainPort] = 0;
        r->doAction    &= 0x0002;
        r->tick         = 0;
        r->status       = RELAY_STATUS_OPENING;
        r->subStatus    = RELAY_STATUS_OPEN_DOING;
        r->tickEvent    = 0;
        r->tryCnt       = 0;

        r->adh.diffAdhCnt = 0;
        r->adh.diffCalCnt = 0;
        r->adh.diffSum = 0;
        break;

    case RELAY_STATUS_OPEN_DOING:
#if 0
        t = abs(r->voltageRight - r->voltageBatt);
        if(t > 400) {
            r->subStatus = RELAY_STATUS_OPEN_OK;
            r->tick = 0;
        } else if(r->tick > (g_FuncCode.Code.openAdhesionTimeout >> 1)) {
            r->subStatus = RELAY_STATUS_OPEN_TIMEOUT;
            r->status = RELAY_STATUS_ERROR;
            r->fault = RELAY_ERROR_OPEN_ADH;
        }
#else
        if(OpenAdhesionTrace(r))        //返回 1 表示没粘连
        {
            r->subStatus = RELAY_STATUS_OPEN_OK;
            r->tick = 0;
        }
#endif
        break;

    case RELAY_STATUS_OPEN_OK:
        r->status = g_RelayObj.battReady ? RELAY_STATUS_OPENED : RELAY_STATUS_WAIT_HIGHVOLT;
        r->tryCnt       = 0;
        break;

    case RELAY_STATUS_OPEN_TIMEOUT:
        break;
    }
}

void OpenAdhesionTraceInit(RELAY_DEV *r, uint16_t initDiff, uint16_t initOpen)
{
    RELAY_ADHISION *adh;

    adh = &r->adh;
    if(initDiff) {
        adh->diffStatic = 0;
        adh->diffCalCnt = 0;
    }
    if(initOpen)
        adh->diffCalCnt = 0;
}

//断开和闭合时处理粘连    ...
uint16_t OpenAdhesionTrace(RELAY_DEV *r)
{
    int32_t t1, t2;
    uint16_t rnt;
    RELAY_ADHISION *adh;


    rnt = 0;
    adh = &r->adh;
    if(r->subStatus == RELAY_STATUS_CLOSE_OK && r->tick > TIME_1S_LOOP_2MS) //继电器已闭合 并且 时间超过1S;  保存当前静态压差 diffStatic
    {
        if(adh->diffStatic == 0)
        {
            if(adh->diffCalCnt == 0) adh->diffSum = 0;

            t1 = r->voltageFront;
            t2 = r->voltageBack;
            adh->diffSum += (t1 - t2);
            if(++adh->diffCalCnt > TIME_1S_LOOP_2MS)
            {
                adh->diffStatic = adh->diffSum / adh->diffCalCnt;
                rnt = 1;
            }
        }
        else
        {
            rnt = 1;
        }

//        gScope[0] = r->tick;
//        gScope[1] = adh->diffStatic;

    }
    else if((r->subStatus == RELAY_STATUS_OPEN_DOING) && (r->tick > TIME_200MS_LOOP_2MS))   //断开时
    {
        t1 = r->voltageFront;
        t2 = r->voltageBack;

#if 1
        if(abs(t1 - t2) > (abs(adh->diffStatic) + r->cfg.adhParaVolt))
        {
            if(++r->tickEvent > TIME_300MS_LOOP_2MS)
            {
                adh->diffAdhCnt = 0;
                return 1;
            }
        }
        else
        {
            r->tickEvent = 0;
        }

        if(!g_RelayObj.battReady)
        {
            adh->diffAdhCnt = 0;
            return 1;
        }

#endif

//        gScope[0] = abs(t1 - t2);
//        gScope[1] = g_FuncCode.Code.OpenCircuitcheckVolt;
//        gScope[2] = r->voltageBack;
//        gScope[3] = r->cfg.adhParaVolt;
//        gScope[4] = adh->diffStatic;

#if 0
        if(abs(t1 - t2) > abs(adh->diffStatic)) {
            r->tickEvent ++;
            adh->diffSum += (abs(t1 - t2) - abs(adh->diffStatic));
            if(adh->diffSum > g_FuncCode.Code.openAdhesionVoltageDiff * (uint32_t)r->tickEvent
                                && r->tickEvent > TIME_200MS_LOOP_2MS) {
                adh->diffAdhCnt = 0;
                rnt = 1;
            }
            gScope[2] = adh->diffSum;
            gScope[3] = r->tickEvent;
        }
#endif

#if 0
        adh->diffSum += abs(t1 - t2);
        adh->diffCalCnt ++;
        //if(abs(adh->diffSum / adh->diffCalCnt) > (abs(adh->diffStatic) + g_FuncCode.Code.openAdhesionVoltageDiff)) {
        if(t1 - t2 / adh->diffCalCnt) > (abs(adh->diffStatic) + g_FuncCode.Code.openAdhesionVoltageDiff)) {
            if(++adh->diffAdhCnt > TIME_200MS_LOOP_2MS) {
                adh->diffAdhCnt = 0;
                rnt = 1;
            }
        } else if(adh->diffAdhCnt > 0)
            adh->diffAdhCnt --;
#endif

        if(r->cfg.adhParaTime == 5000)     //等于5s不判断下电粘连
        {
            adh->diffAdhCnt = 0;
            return 1;
        }


        if(r->tick > (r->cfg.adhParaTime >> 1)) {
            r->fault = RELAY_ERROR_OPEN_ADH;
            r->status = RELAY_STATUS_ERROR;
            rnt = 1;
        }

    }
//    gScope[0] = r->tick;
    return rnt;
}

void ClosedetectionopencircuitInit(RELAY_DEV *r)
{
    RELAY_OPENCIRCUIT *op;
    op = &r->op;

    op->diffOpStatic = 0;
    op->diffOpSum = 0;
    op-> diffOpCalCnt = 0;                // 偏差求和计数，求静态偏差时的中间量
    op-> diffOpTick = 0;                  // 开路计时器
    op-> diffOphCnt = 0;
    op-> diffOpReal = 0;
}

// 闭合开路判断             2022.10.26    licht.he
void Closedetectionopencircuit(RELAY_DEV *r)
{
    int32_t t1, t2;
    RELAY_OPENCIRCUIT *op;
    uint16_t v = VERSION_SOFTWARE;

    op = &r->op;

    if((r->subStatus == RELAY_STATUS_CLOSE_OK) && (r->tick > (g_FuncCode.Code.OpenCircuitStartTime >>1)) && (op->diffOpStatic == 0))         // 保存继电器当前静态压差 diffStatic
    {
            if(op->diffOpCalCnt == 0) op->diffOpSum = 0;

            t1 = r->voltageFront;
            t2 = r->voltageBack;
            op->diffOpSum += (t1 - t2);
            if(++op->diffOpCalCnt > TIME_1S_LOOP_2MS)
            {
                op->diffOpStatic = op->diffOpSum / op->diffOpCalCnt;                                                // 误差值
                op->diffOpTick = 0;
            }
    }
    else if (op->diffOpStatic != 0)
    {
        t1 = r->voltageFront;
        t2 = r->voltageBack;
        op->diffOpTick ++;

        if((abs(t1 - t2) < (op->diffOpStatic + g_FuncCode.Code.OpenCircuitcheckVolt)) && (t2 > op->diffOpStatic) )                     // 开路判断电压需可改变，持续时间也需可变（暂时使用粘连时间及电压）
        {
            op->diffOphCnt = 0;
            op->diffOpTick = 0;
        }
        else
        {
            if(op->diffOpTick > TIME_300MS_LOOP_2MS)
            {
                op->diffOphCnt ++;
                op->diffOpTick = 0;
            }
        }

        if(!g_RelayObj.battReady)                                                                                   //下高压不报
        {
            op->diffOphCnt = 0;
        }

        if(g_FuncCode.Code.OpenCircuitDuration == 15000 && v >= 4000)                                                           //持续时间等于15s不判断开路
        {
            op->diffOphCnt = 0;
        }


        if( op->diffOphCnt > ((g_FuncCode.Code.OpenCircuitDuration >> 1) / TIME_300MS_LOOP_2MS))
        {
            r->fault = RELAY_ERROR_CLOSE_TIMEOUT;
            r->status = RELAY_STATUS_ERROR;
            r->subStatus = RELAY_ERROR_CLOSE_TIMEOUT;

            do_action(r->cfg.relayMainPort, 0);                       //开路判断后断开接触器状态
            do_action(r->cfg.relayPrePort, 0);
            r->doAction    = 0;
        }

    }

//    gScope[0] = r->fault;
//    gScope[1] = op->diffOpStatic;
//    gScope[2] = op->diffOphCnt;
//    gScope[3] = r->voltageFront;
//    gScope[4] = r->voltageBack;

}


//高压插头互锁 延时下电处理
void PowerDownLock(void)
{
//    uint16_t versionflag = 0;
//    if(aiValue[4]>1500)
//        versionflag = 0xEFF;
//    else
//        versionflag = 0xFF;

    //高压插头互锁滤波
    if(diStatus.bits.di1 || (g_FuncCode.Code.interLockFilterTime == 0)) {
        hvbInfo.pluginLockTick = 0;
        hvbInfo.interLockFault = 0;
    } else if(++hvbInfo.pluginLockTick > (g_FuncCode.Code.interLockFilterTime >> 1)) {
        hvbInfo.interLockFault = 1;
        hvbInfo.pluginLockTick = g_FuncCode.Code.interLockFilterTime >> 1;
    }

    //延时下电
    if((diStatus.bits.di2 || diStatus.bits.di3
           // || (gVcuInfo.bms_delayoff   == 1) //关联BMS充电状态
      )
      ||((g_FuncCode.Code.powerDownDelayCOND!=0)&&(doStatus.value & 0xFF) )        // 2.0 0XFF    2.5 0XEFF
      )
    {
        hvbInfo.powerdownTick   = 0;
        doStatus.bits.do9       = 1;
        hvbInfo.dying           = 0;
    } else {
        hvbInfo.dying = 1;
        if(++hvbInfo.powerdownTick > (g_FuncCode.Code.powerDownDelayTime >> 1))
        {
            if(doStatus.value & 0xFF)
            {
                if(hvbInfo.powerdownTick > TIME_200MS_LOOP_2MS)
                    hvbInfo.powerdownTick -= TIME_200MS_LOOP_2MS;
                else
                    hvbInfo.powerdownTick = 0;
            }
            else
                doStatus.bits.do9 = 0;
        }
    }

    //硬线使能控制上装继电器
    uint16_t car_type = g_FuncCode.Code.CanInvMotorVersion; //车型
    switch(car_type)
    {
    case HanGeWei_CARTYPE:
        if(diStatus.bits.di4 == 1)  // 上装
            g_RelayObj.Aux1.cmdVcu = RELAY_CMD_CLOSE;
        else
            g_RelayObj.Aux1.cmdVcu = RELAY_CMD_OPEN;
        break;

    case Test_D3Z1H9L5U1:
        if(diStatus.bits.di1 == 1) // 测试专用 HVLOCK 控制 TJ6
            doStatus.bits.do2 = 1;
        else
            doStatus.bits.do2 = 0;

        if( diStatus.bits.di4 == 1) // 测试专用 INPUT4 控制TJ10
            doStatus.bits.do7 = 1;
        else
            doStatus.bits.do7 = 0;
        break;

    case Test_D1Z1M9L5U1:
        if(diStatus.bits.di1 == 1) // K9+
            doStatus.bits.do7 = 1;
        else
            doStatus.bits.do7 = 0;

        if(diStatus.bits.di4 == 1) // K6+
            doStatus.bits.do2 = 1;
        else
            doStatus.bits.do2 = 0;

        if(diStatus.bits.di3 == 1) // DC_EN
            doStatus.bits.do3 = 1;
        else
            doStatus.bits.do3 = 0;
        break;


    case ZhongTong_CARTYPE_5IN1_Slaver:
        if(diStatus.bits.di1 == 1 && car_ready == 1) // 充电负1
            g_RelayObj.Aux1.cmdVcu = RELAY_CMD_CLOSE;
        else
            g_RelayObj.Aux1.cmdVcu = RELAY_CMD_OPEN;
        if(diStatus.bits.di3 == 1) // 主负
            g_RelayObj.Spare1.cmdVcu = RELAY_CMD_CLOSE;
        else
            g_RelayObj.Spare1.cmdVcu = RELAY_CMD_OPEN;
        if(diStatus.bits.di4 == 1 && car_ready == 1) // 充电负2
            g_RelayObj.Aux2.cmdVcu = RELAY_CMD_CLOSE;
        else
            g_RelayObj.Aux2.cmdVcu = RELAY_CMD_OPEN;
        break;

    case DaYun_ChengDu_CARTYPE_7AD:
        if(g_RelayObj.battReady)
            g_RelayObj.ManyInOne.cmdVcu = RELAY_CMD_CLOSE;   // 多合一
        else
            g_RelayObj.ManyInOne.cmdVcu = RELAY_CMD_OPEN;

        if(g_RelayObj.battReady)
            g_RelayObj.Aux1.cmdVcu = RELAY_CMD_CLOSE;   // 上装
        else
            g_RelayObj.Aux1.cmdVcu = RELAY_CMD_OPEN;
        break;

    default:
        break;
    }
}

void RelayManagement(void)
{
    uint16_t i;
    RELAY_DEV *p = &g_RelayObj.MCU1;
    RELAY_DEV *r = &g_RelayObj.MCU1;

//    if( ((MAX_V(g_RelayObj.ManyInOne.voltageFront,g_RelayObj.ManyInOne.voltageBack) - MIN_V(g_RelayObj.ManyInOne.voltageFront,g_RelayObj.ManyInOne.voltageBack)) > 150)
//          && ((g_RelayObj.ManyInOne.voltageBack / 10) > 150) && (g_RelayObj.ManyInOne.cmdVcu = RELAY_CMD_CLOSE))
//    {
//         g_RelayObj.ManyInOne.cmdVcu = RELAY_CMD_OPEN;
//         g_RelayObj.ManyInOne.cmd = RELAY_CMD_OPEN;
//    }
//    RalayParaInit();

//    RelayConfigUpdate();                                      //更新配置参数
//    RelayDebugUpdate();
    RelayBattCheck();                                           // check battery voltage UV/OV
    for(i=0; i<NUM_RELAY; i++)
    {
        r = &p[i];                                              //遍励每一组继电器


        r->adh.diffReal = r->voltageFront - r->voltageBack;     //取压差 用于判断粘连  ...

        //if(++r->tick > 0xfffd)  r->tick = 0xfffd;               //计时
        r->tick++;
        r->statusScope = (r->status << 8) |  r->subStatus;       //记录当前状态
        RelayVoltageUpdate(r);                                  //更新当前继电器组的 前后端电压

        if(r->cfg.relayMainPort == 0) continue;                   //没用的

        RelayCmdCheck(r);                                       // 更新当前继电器组的 VCU 命令

        switch(r->status)
        {
            case RELAY_STATUS_WAIT_HIGHVOLT:                            //等待高压
                if(g_RelayObj.battReady) {
                    r->status = RELAY_STATUS_POWERON_ADH_CHECK;
                    r->tick = 0;
                }
                r->subStatus = 0;
                break;

            case RELAY_STATUS_POWERON_ADH_CHECK:
                if(RelayPoweronAdhCheck(r))                             //已上高压，上电检查粘连
                {
                    if(r->fault == RELAY_ERROR_OK)
                    {
                        if(r->cmd == RELAY_CMD_CLOSE)
                        {
                            r->status = RELAY_STATUS_CLOSING;
                            r->subStatus = RELAY_STATUS_CLOSE_INIT;
                        }
                        else
                        {
                            r->status = RELAY_STATUS_OPENED;
                            r->subStatus = RELAY_STATUS_OPEN_OK;
                        }
                    }
                    else
                    {
                        r->status = RELAY_STATUS_ERROR;
                    }
                    r->tick = 0;
                }

                ClosedetectionopencircuitInit(r);
                break;

            case RELAY_STATUS_CLOSING:
            case RELAY_STATUS_CLOSED:
                RelayCmdClose(r);
                break;

            case RELAY_STATUS_OPENING:
            case RELAY_STATUS_OPENED:
                RelayCmdOpen(r);
                break;

            case RELAY_STATUS_ERROR:
            default:
                break;
        }
    }

}

