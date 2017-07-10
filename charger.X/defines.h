/* 
 * File:   defines.h
 * Author: Ehsan
 *
 * Created on May 1, 2016, 3:19 PM
 */

#ifndef DEFINES_H
#define	DEFINES_H

#ifdef	__cplusplus
extern "C" {
#endif

#define FOSC    80000000ULL//(7370000ULL)
#define FCY     (FOSC/4)
    
#define _NONE   0x00
    
#define _ON  0x01
#define _OFF  0x00

#define _a      10   
#define _b      11 
#define _c      12 
#define _d      13 
#define _e      14 
#define _f      15 
#define _h      16 
#define _l      17 
#define _i      18 
#define _n      19 
#define _o      20 
#define _p      21 
#define _r      22 
#define _s      23
#define _t      24
#define _u      25
#define _dot    50 
#define __OFF    60 


#define P_Key   PORTBbits.RB7
#define N_Key   PORTBbits.RB8
#define M_Key   PORTBbits.RB6
    
#define DIGIT1  LATBbits.LATB11   
#define DIGIT2  LATBbits.LATB10
#define DIGIT3  LATBbits.LATB9 
    
    
    
#define _ENTER              0x01
#define _UP                 0x02
#define _DOWN               0x03
    
//#define Debounce_Limit      0x16
    

    
#define _REGISTER_ADDRESS_SHOW          0x01 
#define _FLAG_REGISTER_ADDRESS_SHOW     0x02 
#define _BYTE_REGISTER_ADDRESS_SHOW     0x03 
#define _REAL_REGISTER_ADDRESS_SHOW     0x04 
#define _FLAG_REGISTER_VALUE_SHOW       0x05
#define _BYTE_REGISTER_VALUE_SHOW       0x06
#define _REAL_REGISTER_VALUE_SHOW       0x07 
#define _FLOAT_REGISTER_VALUE_SHOW      0x08
#define _WORD_H_REGISTER_VALUE_SHOW      0x09
#define _WORD_L_REGISTER_VALUE_SHOW      0x0A
    
#define _0_1_2  0
#define _2_0_1  1
#define _1_2_0  2
#define _0_2_1  3
#define _1_0_2  4
#define _2_1_0  5
#define _0_0_0  6
    
#define _rst     0
#define _sAu     1
#define _Fst     2
#define _clr     3
#define _Lbc     4
#define _Hbc     5
#define _LLc     6
#define _HLc     7
#define _Lou     8
#define _Hou     9
#define _LIU     10  
#define _HIU     11 
#define _Lbt     12
#define _Hbt     13
#define _LHt     14
#define _HHt     15
#define _FIt     16

//#define _PWM1    LATDbits.LATD0 
//#define _PWM2    LATDbits.LATD1
//#define _PWM3    LATDbits.LATD2
    
#define _Wdt_Status RCONbits.SWDTEN
    
//#define _OC1    _PWM1 
//#define _OC2    _PWM2
//#define _OC3    _PWM3
    
//#define _OC1    _PWM1 
//#define _OC2    _PWM2
//#define _OC3    _PWM3

//#define _OC1    LATDbits.LATD0 
//#define _OC2    LATDbits.LATD1
//#define _OC3    LATDbits.LATD2
    
#define READY  LATDbits.LATD3 
#define ERR_OUT  PORTDbits.RD1
#define DRIVE_RESET  LATDbits.LATD2   

    
    
#define _Debounce_Limit  0x3F
#define _Debounce_Limit_1  0x02
#define _MAX_Delay      39148//33619//31349//44150    
#define _MIN_Delay      21100//22309//17509//11250  
#define _Timer_Range    (_MAX_Delay - _MIN_Delay)
    
#define _PWM_Period     9999//999
#define _MAX_DutyCycle      9000//998  
#define _MIN_DutyCycle       0   
#define _DutyCycle_Range    (_MAX_DutyCycle - _MIN_DutyCycle)
    
#define _NO_PT100_VALUE     3000
    
#define _Soft_Start_Step    1
#define _ON_TIME        2500//10000
    
#define _T3_LOW     (16666 * 0.8)   
#define _T3_HIGH     (16666 * 1.15) 
    
#define clientAddress 0x01
///////////////////------------Float Type Index--------------//////////////////
////-----RW-----////////
//#define _r_VO_SP            0
//#define _r_IO_SP            1
//#define _r_IB_SP            2
//#define _r_TEMP_DEC_IB      3
//#define _r_TEMP_ZERRO_IB    4
//#define _r_TEMP_DEC_IO      5
//#define _r_TEMP_ZERRO_IO    6
//#define _r_VIN_HIGH_ALARM   7
//#define _r_VIN_LOW_ALARM    8
    
 #define _r_VO_SP            1
 #define _r_IO_SP            4
 #define _r_IB_SP            3
 #define _r_TEMP_DEC_IB      5
#define _r_TEMP_ZERRO_IB    0
 #define _r_TEMP_DEC_IO      6
#define _r_TEMP_ZERRO_IO    7
#define _r_VIN_HIGH_ALARM   8
 #define _r_VIN_LOW_ALARM    2
#define _END_OF_RW_FLOAT_ADDRESS    9    
////-----RO----/////////
#define _r_VO_PV            9
#define _r_IO_PV            10
#define _r_IB_PV            11
#define _r_VO_TEMP_SP       12
#define _r_IO_TEMP_SP       13
#define _r_BAT_TEMP         14
#define _r_IB_TEMP_SP       15
#define _r_VIN              16
#define _r_IB_fsp           17
#define _r_IO_fsp            18
#define _r_VO_fsp           19
#define _END_OF_RO_FLOAT_ADDRESS    20
///////////////////------------Byte Type Index--------------//////////////////  
//Byte Type Index
#define _b_VO_P 1
#define _b_VO_I 2
#define _b_VO_D 3
#define _b_IB_P 4
#define _b_IB_I 5
#define _b_IB_D 6
#define _b_IO_P 7 
#define _b_IO_I 8 
#define _b_IO_D 9
#define _b_CMD  10
#define _b_DATA_REAL 11
#define _b_VO_GAIN 12
#define _b_IB_GAIN 13
#define _b_IO_GAIN 14
#define _END_OF_RW_BYTE_ADDRESS   15
    
///////////////////------------WORD Type Index--------------////////////////// 
//////-----RW-----//////
#define _r_CMD          0    
#define _r_USTEP        1
#define _r_DSTEP        2
#define _U_CV_Z         3
#define _CV_G           4
#define _U_L1           5
#define _U_L2           6
#define _U_L3           7
#define _END_OF_RW_WORD_ADDRESS 8 
//////-----RO-----//////
#define _AI_VOUT        8
#define _AI_IBAT        9
#define _AI_IOUT        10
#define _AI_VIN         11
#define _AI_BAT_TEMP    12
#define _AI_HSINK_TEMP  13
#define _r_VOF          14
#define _r_IB_F         15
#define _r_IO_F         16
#define _r_VIN_F        17
#define _r_VO           18
#define _r_IB           19
#define _r_IO           20
#define _r_VO_CV        21
#define _r_IB_CV        22
#define _r_IO_CV        23
#define _r_CV           24
#define _r_HSINK        25
#define _U_CV           26
#define _U_PRE1         27
#define _U_PRE2         28
#define _U_PRE3         29  
#define _U_NOC          30
#define _U_VO_PV        31
#define _U_VI_PV        32
#define _U_IB_PV        33
#define _U_IO_PV        34
#define _U_TB_PV        35
#define _U_TH_PV        36
#define _U_FRQ_PV       37   
#define _U_VO_TEMP_SP   38
#define _r_B_TEMP_F     39
#define _r_H_TEMP_F     40
#define _r_B_TEMP       41
#define _r_H_TEMP       42
#define _U_TEST1        43
#define _U_TEST2        44
#define _U_TEST3        45
#define _U_TEST4        46
#define _END_OF_RO_WORD_ADDRESS 47  
    
///////////////////------------Flag Type Index--------------//////////////////
//Flag Type Index
#define _END_OF_FUNCTION_FLAG_ADDRESS 18  
#define _END_OF_EVENT_FLAG_ADDRESS 28 
#define _END_OF_PROPERTY_FLAG_ADDRESS 61
//////-----RW-----//////
#define _F_VOFS             62
#define _F_VO_SP_SEL        63
#define _F_IBFS             64
#define _F_IB_SP_SEL_DEC    65
#define _F_IOFS             66
#define _F_IO_SP_SEL_DEC    67
#define _F_ERU              68
#define _F_ERD              69
#define _F_IB_SP_SEL        70
#define _F_IO_SP_SEL        71
#define _F_IB_SP_ALARM_MASK     72
#define _F_IO_SP_ALARM_MASK     73
#define _F_IB_TEMP_SP_MASK      74
#define _F_IO_TEMP_SP_MASK      75
#define _F_VIN_HIGH_ALARM_MASK  76 
#define _F_rSt  77
#define _F_PHASE_LOSS_MASK      78
#define _F_FAIL_ENABLE          79
#define _F_CHARGER_DISABLE      80
#define _F_SAVE_FACTORY_RESET   81
#define _F_EQUAlIZATUIN_CHARGE   82
#define _F_MANUAL_EQUAlIZATUIN_CHARGE   83
#define _END_OF_RW_FLAG_ADDRESS  84
//////-----RO-----//////
#define _F_IB_SP_DEC        84
#define _F_IO_SP_DEC        85
#define _END_OF_RO_FLAG_ADDRESS     86
#define _END_OF_RW_FLAG_ADDRESS2    56


    

#ifdef	__cplusplus
}
#endif

#endif	/* DEFINES_H */

