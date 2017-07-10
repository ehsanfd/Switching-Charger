/*
 * File:   main.c
 * Author: Ehsan
 *
 * Created on May 1, 2016, 2:42 PM
 */



#include "defines.h"
#include <xc.h>
#include <p30fxxxx.h>
#include <dsp.h>
#include <libq.h>
#include <libpic30.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>



#include "eeprom.h"
#include "drv_uart1.h"
#include "modbus_RTU.h"

// FOSC
#pragma config FOSFPR = XT_PLL8//XT              // Oscillator (XT)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_10//WDTPSB_5        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_8       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_OFF         // POR Timer Value (Timer Disabled)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_OFF         // PBOR Enable (Disabled)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)


/* Declare a PID Data Structure named, fooPID */
tPID fooPID1, fooPID2, fooPID3;
/* The fooPID data structure contains a pointer to derived coefficients in X-space and */
/* pointer to controler state (history) samples in Y-space. So declare variables for the */
/* derived coefficients and the controller history samples */
fractional abcCoefficient1[3] __attribute__ ((section (".xbss, bss, xmemory")));
fractional controlHistory1[3] __attribute__ ((section (".ybss, bss, ymemory")));

fractional abcCoefficient2[3] __attribute__ ((section (".xbss, bss, xmemory")));
fractional controlHistory2[3] __attribute__ ((section (".ybss, bss, ymemory")));

fractional abcCoefficient3[3] __attribute__ ((section (".xbss, bss, xmemory")));
fractional controlHistory3[3] __attribute__ ((section (".ybss, bss, ymemory")));
/* The abcCoefficients referenced by the fooPID data structure */
/* are derived from the gain coefficients, Kp, Ki and Kd */
/* So, declare Kp, Ki and Kd in an array */
fractional kCoeffs1[] = {0,0,0};
fractional kCoeffs2[] = {0,0,0};
fractional kCoeffs3[] = {0,0,0};


int Current_State = 0;
//int Previous_State = 0;
char Key_Pressed_Flag = 0;
char Key_Released_Flag = 1;
char Pressed_Key = _NONE;
char Previous_Pressed_Key = _NONE;
char Detected_Key = _NONE;
char Previous_Detected_Key = _NONE;
int Debounce_Counter = 0;
char Debounce_Limit = _Debounce_Limit;
int TTCounter = 0;
int TTCounter_Limit = 250;
int iTCounter = 0;

int Register_Counter = 0;
int Register_Counter_Limit = 0;
int End_Of_Float =0;
int End_Of_Word = 0;
int End_Of_Byte = 0;
int End_Of_Flag = 0;
int DAM = 0;

char Flag_Type[_END_OF_RO_FLAG_ADDRESS];
unsigned char Byte_Type[_END_OF_RW_BYTE_ADDRESS];
unsigned int Word_Type[_END_OF_RO_WORD_ADDRESS];
float r[_END_OF_RO_FLOAT_ADDRESS];
unsigned int Real_Type[_END_OF_RO_FLOAT_ADDRESS];
unsigned char Float_Type[_END_OF_RO_FLOAT_ADDRESS];

unsigned char Segment_Digit[3];
unsigned char Segment_Dot[3];
unsigned char Set_Segment_Counter = 0;
unsigned char Over_Flow_Timer1_Flag = 0;
unsigned char Over_Flow_Timer1_Counter = 0;

int Que_Alarm[16];
unsigned char f_event[33];
unsigned char f_property[33];
unsigned char f_event_already_happened[33];

unsigned char Screen_Saver_Enable = 0;
unsigned int Screen_Saver_Counter = 0;

unsigned char Run_Function_Start_Flag = 0;
unsigned int Run_Function_Start_Counter = 0;

unsigned int Temporary_int_Register1 = 0;
unsigned long int Temporary_long_Register1 = 0;
float Temporary_float_Register = 0;

unsigned long int Equalization_Charge_Counter = 0;
unsigned long int Manual_Equalization_Charge_Counter = 0;
unsigned char Equalization_Charge_Flag = 0;
float VO_SP_Temporary = 0.0;

unsigned int Phase_Direct_Detect_Counter = 0;
float VO_PV_SUM = 0.00;
float VI_PV_SUM = 0.00;
float IB_PV_SUM = 0.00;
float IO_PV_SUM = 0.00;
float TB_PV_SUM = 0.00;
float TH_PV_SUM = 0.00;
float Frq_PV_SUM = 0.00;

unsigned char OC1_Status = 0;
unsigned char OC2_Status = 0;
unsigned char OC3_Status = 0;

//unsigned char Power_On_Flag = 1;
//unsigned int RU_Step = 0;

unsigned int Second_Filter_Counter = 0;

unsigned char VO_PID_Enable = 0;
unsigned char IB_PID_Enable = 0;
unsigned char IO_PID_Enable = 0;

unsigned char Que_Alarm_Update_Flag = 0;
unsigned int Que_Alarm_Update_Counter = 0;
unsigned char Start_Que_Alarm_Update_Flag = 0;
unsigned int Last_Event_Number = 0;
unsigned int T_INT0_INT1 = 0;
unsigned int T_INT1_INT2 = 0;
unsigned int T_INT2_INT0 = 0;

volatile uint8_t instate = 0;
volatile uint8_t outstate = 0x32;
volatile uint16_t inputRegisters[20];
volatile uint16_t holdingRegisters[20] ={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
unsigned char UART_last_Byte_Sent_Flag = 0;


unsigned char Phase_Sync_Counter[3] = {0,0,0};
unsigned char F_RIGHT = 0;
unsigned char F_LEFT = 0;
unsigned char F_L1_LOSS = 0;
unsigned char F_L2_LOSS = 0;
unsigned char F_L3_LOSS = 0;
unsigned char F_NO_LOSS = 0;
unsigned int C_NO_OF_CYCLE = 0;
unsigned char F_FILTER_UPDATE = 0;
unsigned char F_PID_CLK = 0;
//unsigned int AI_VOUT = 0;
//unsigned int AI_IBAT = 0;
//unsigned int AI_IOUT = 0;
//unsigned int AI_BAT_TEMP = 0;
//unsigned int AI_VIN = 0;
//unsigned int AI_HSINK_TEMP = 0;
unsigned long int AI_VO_SUM = 0;
unsigned long int AI_IOUT_SUM = 0;
unsigned long int AI_IBAT_SUM = 0;
unsigned long int AI_VIN_SUM = 0;
unsigned long int AI_BAT_TEMP_SUM = 0;
unsigned long int AI_HITSINK_TEMP_SUM = 0;
//unsigned int r_VOF = 0;
//unsigned int r_IB_F = 0;
//unsigned int r_VIN_F = 0;
//unsigned int r_VO = 0;
//float r_VO_Z = 0.0;
//unsigned int r_IB = 0;
//float r_IB_Z = 0.0;
unsigned char F_VO_UPDATE = 0;
unsigned char F_IO_UPDATE = 0;
unsigned char F_IB_UPDATE = 0;
unsigned char F_VIN_UPDATE = 0;
unsigned char F_B_TEMP_UPDATE = 0;
unsigned char F_H_TEMP_UPDATE = 0;
float VO_G = 0.0;
unsigned int HOU_REAL = 0.0;
unsigned int HOU_WORD = 0; 
unsigned int LOU_WORD = 0;
float TBAT_G = 0.0;
unsigned char Hbt_REAL = 0;
unsigned int Hbt_WORD = 0;
unsigned char Lbt_REAL = 0;
unsigned int Lbt_WORD = 0;
float TBAT_WZ = 0.0;
//float r_VO_PV = 0.0;
float Set_Point = 0.0;
unsigned int INT_Set_Point = 0;
float Error_VO = 0.0;
float Error_IB = 0.0;
float Error_IO = 0.0;
//float r_VO_SP = 0.0;
//float r_VO_TEMP_SP = 0.0;
//float r_BAT_TEMP = 0.0;
//unsigned int r_VO_CV = 0;
float IB_G = 0.0;
unsigned char HBC_REAL = 0;
unsigned int HBC_WORD = 0;
unsigned int LBC_WORD = 0;
//float r_IB_PV = 0.0;
//float r_IB_SP = 0.0;
//float r_IB_TEMP_SP = 0.0;
//unsigned int r_IB_CV = 0;
//unsigned int r_IO_F = 0;
//unsigned int r_IO = 0;
//float r_IO_Z = 0.0;
float IO_G = 0.0;
unsigned char HLC_REAL = 0;
unsigned int HLC_WORD = 0;
unsigned int LLC_WORD = 0;
//float r_IO_PV = 0.0;
//float r_IO_SP = 0.0;
//unsigned int r_IO_CV = 0;
//unsigned int r_CMD = 0;
//unsigned int r_CV = 0;
//float r_TEMP_DEC_IB = 0.0;
float IB_DEC_G = 0.0;
//float r_IO_TEMP_SP = 0.0;
//float r_TEMP_DEC_IO = 0.0;
float IO_DEC_G = 0.0;
float NTC_G = 0;
unsigned int HHt_WORD = 0;
unsigned int LHt_WORD = 0;
float VI_G = 0.0;
unsigned char LUT[256];
//char r_HSINK = 0;
unsigned int Ramp_Output = 0;
unsigned int Set_Point_Ramp_Output = 0;

//unsigned int r_USTEP = 0;
//unsigned int r_DSTEP = 0;
//unsigned int U_CV = 0;
//unsigned int U_CV_Z = 0;
//unsigned int CV_G = 0;
//unsigned int U_L1 = 0;
//unsigned int U_L2 = 0;
//unsigned int U_L3 = 0;
//unsigned int U_PRE1 = 0;
//unsigned int U_PRE2 = 0;
//unsigned int U_PRE3 = 0;
unsigned char FUNC_CONFIG_RIGHT = 0;
unsigned char FUNC_CONFIG_LEFT = 0;
unsigned char FUNC_CONFIG_RIGHT1 = 0;
unsigned char FUNC_CONFIG_LEFT1 = 0;
unsigned char F_FAIL = 0;
signed long int M = 0;
float M2 = 0.0;
unsigned int LIU_WORD = 0;
unsigned int HIU_WORD = 0;
unsigned char HIU_REAL = 0;
unsigned char LHt_REAL = 0;
unsigned char HHt_REAL = 0;
unsigned int HMt_WORD = 0;
unsigned char HMt_REAL = 0;
unsigned char CMD_TYPE = 1;
unsigned char F_FAIL_DATA_ENTRY = 0;
unsigned int AI_OFF_SCR_BATTERY_VOLTAGE = 0;
unsigned char F_E2ROM_FAIL = 0;
unsigned char WATCH_DOG_FAIL = 0;
unsigned char Start_Output_Over_Under_V_Flag = 0;
unsigned int Output_Over_Under_V_Counter = 0;
unsigned char Load_Battery_Over_I_Flag = 0;
unsigned int Load_Battery_Over_I_Counter = 0;
unsigned char Byte_Type_Change_Flag[_END_OF_RW_BYTE_ADDRESS];
unsigned char Word_Type_Change_Flag[_END_OF_RW_WORD_ADDRESS];
unsigned char Real_Type_Change_Flag[_END_OF_RW_FLOAT_ADDRESS];
unsigned char Float_Type_Change_Flag[_END_OF_RW_FLOAT_ADDRESS];
unsigned char Flag_Type_Change_Flag[_END_OF_RW_FLAG_ADDRESS2];

unsigned char PT100_LOSS_FLAG = 0;

unsigned int DRIVE_RESET_Counter = 0;
unsigned int READY_STATUS_Counter = 0;

unsigned char PID_Ramp_Disable = 0;

unsigned int F_FILTER_UPDATE_Counter = 0;

extern volatile unsigned char rxbuffer[MaxFrameIndex+1];
extern volatile unsigned char BusState;

void PIC_Init(void);
void Update_7Segment_Display(void);
void Perform_Command(void);
void Set_Segment(unsigned char value, unsigned char Number_of_Seg, unsigned char DOT_status);
void Show_Screen_Saver(unsigned char index_fault);
void Run_Function(unsigned char index_function);
void Restore(void);
void External_Interrupt(unsigned char status);
unsigned char CMP_Arrays(unsigned char * a1, unsigned char n);
void Event_Flag_Update(void);
void PID_Initialize(void);
void Factory_Reset(void);
void Save_Factory_Reset(void);
void Set_Limits_Value(void);
void Check_Phase_Sequence(void);
void modbusGet(void);
void SetOuts(volatile uint8_t in);
uint8_t ReadIns(void);

void __attribute__ ( ( __interrupt__ , no_auto_psv ) ) _T1Interrupt ( void )
{
/* Interrupt Service Routine code goes here */
    Over_Flow_Timer1_Counter ++;
    if(Over_Flow_Timer1_Counter == 50)
    {
        Over_Flow_Timer1_Counter = 0;
        Over_Flow_Timer1_Flag = 1;
    }
    modbusTickTimer();
    IFS0bits.T1IF = 0;  //Clear Timer1 interrupt flag
    
}

void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void)
{
    IFS0bits.INT0IF = 0;    
    IEC0bits.INT0IE = 0;    
    IFS1bits.INT1IF = 0;    
    IEC1bits.INT1IE = 1;    
    IFS1bits.INT2IF = 0;    
    IEC1bits.INT2IE = 1;
    
    T_INT2_INT0 = TMR5;
    TMR5 = 0;
    FUNC_CONFIG_LEFT = FUNC_CONFIG_LEFT1;
    FUNC_CONFIG_RIGHT = FUNC_CONFIG_RIGHT1;
    if(F_FAIL == 0 && Flag_Type[_F_CHARGER_DISABLE] == 0 && (FUNC_CONFIG_LEFT == 1 || FUNC_CONFIG_RIGHT == 1))
    {
        if((FUNC_CONFIG_LEFT == 1 && OC2_Status == 0) || (FUNC_CONFIG_RIGHT == 1 && OC3_Status == 0))
        {
//            T2CONbits.TON = 1;
//            PR2 = Word_Type[_U_PRE1];
        }
    }    
    
    //STORE NO. OF CYCLE PER INTERRUPT
    F_FILTER_UPDATE = 1;
    Phase_Sync_Counter[0] = 0; 
    Phase_Sync_Counter[1] ++;
    if(Phase_Sync_Counter[1] > 5)
        Phase_Sync_Counter[1] = 5;
    Phase_Sync_Counter[2] ++;
    if(Phase_Sync_Counter[2] > 5)
        Phase_Sync_Counter[2] = 5;
    IFS0bits.INT0IF = 0;    //Clear the INT0 interrupt flag or else
                                //the CPU will keep vectoring back to the ISR
}

void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void)
{
    IFS0bits.INT0IF = 0;    
    IEC0bits.INT0IE = 1;    
    IFS1bits.INT1IF = 0;    
    IEC1bits.INT1IE = 0;    
    IFS1bits.INT2IF = 0;    
    IEC1bits.INT2IE = 1;
    
    T_INT0_INT1 = TMR5;
    TMR5 = 0;
    FUNC_CONFIG_LEFT = FUNC_CONFIG_LEFT1;
    FUNC_CONFIG_RIGHT = FUNC_CONFIG_RIGHT1;
    if(F_FAIL == 0 && Flag_Type[_F_CHARGER_DISABLE] == 0 && (FUNC_CONFIG_LEFT == 1 || FUNC_CONFIG_RIGHT == 1))
    {
        if((FUNC_CONFIG_LEFT == 1 && OC3_Status == 0) || (FUNC_CONFIG_RIGHT == 1 && OC1_Status == 0))
        {
//            T3CONbits.TON = 1;
//            PR3 = Word_Type[_U_PRE2];
        }
    }
    Phase_Sync_Counter[1] = 0; 
    Phase_Sync_Counter[0] ++;
    if(Phase_Sync_Counter[0] > 5)
        Phase_Sync_Counter[0] = 5;
    Phase_Sync_Counter[2] ++;
    if(Phase_Sync_Counter[2] > 5)
        Phase_Sync_Counter[2] = 5;    
    IFS1bits.INT1IF = 0;    //Clear the INT1 interrupt flag or else
                                //the CPU will keep vectoring back to the ISR
}

void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void)
{ 
    IFS0bits.INT0IF = 0;    
    IEC0bits.INT0IE = 1;    
    IFS1bits.INT1IF = 0;    
    IEC1bits.INT1IE = 1;    
    IFS1bits.INT2IF = 0;    
    IEC1bits.INT2IE = 0;
    
    T_INT1_INT2 = TMR5;
    TMR5 = 0;
    FUNC_CONFIG_LEFT = FUNC_CONFIG_LEFT1;
    FUNC_CONFIG_RIGHT = FUNC_CONFIG_RIGHT1;
    if(F_FAIL == 0 && Flag_Type[_F_CHARGER_DISABLE] == 0 && (FUNC_CONFIG_LEFT == 1 || FUNC_CONFIG_RIGHT == 1))
    {
        if((FUNC_CONFIG_LEFT == 1 && OC1_Status == 0) || (FUNC_CONFIG_RIGHT == 1 && OC2_Status == 0))
        {
//            T4CONbits.TON = 1;
//            PR4 = Word_Type[_U_PRE3];
        }
    }   
    
    Phase_Sync_Counter[2] = 0; 
    Phase_Sync_Counter[0] ++;
    if(Phase_Sync_Counter[0] > 5)
        Phase_Sync_Counter[0] = 5;
    Phase_Sync_Counter[1] ++;
    if(Phase_Sync_Counter[1] > 5)
        Phase_Sync_Counter[1] = 5;      
    IFS1bits.INT2IF = 0;    //Clear the INT2 interrupt flag or else
                                //the CPU will keep vectoring back to the ISR
}

void __attribute__ ( ( __interrupt__ , no_auto_psv ) ) _T2Interrupt ( void )
{
    /*if(FUNC_CONFIG_LEFT == 1)
    {        
        if(OC2_Status == 0)
        {
            _OC2 = 1;
            OC2_Status = 1;
            PR2 = _ON_TIME;
            TMR2 = 0;
        }
        else
        {
            _OC2 = 0;
            OC2_Status = 0;
            T2CONbits.TON = 0; 
            TMR2 = 0;
        }
    }
    else if(FUNC_CONFIG_RIGHT == 1)
    {
        if(OC3_Status == 0)
        {
            OC3_Status = 1;
            _OC3 = 1;
            PR2 = _ON_TIME;
            TMR2 = 0;
            
        }
        else
        {
            OC3_Status = 0;
            _OC3 = 0;
            T2CONbits.TON = 0;  
            TMR2 = 0;
        }
    }
    IFS0bits.T2IF = 0;*/
}

void __attribute__ ( ( __interrupt__ , no_auto_psv ) ) _T3Interrupt ( void )
{
    /*if(FUNC_CONFIG_LEFT == 1)
    {        
        if(OC3_Status == 0)
        {
            OC3_Status = 1;
            _OC3 = 1;
            PR3 = _ON_TIME;
            TMR3 = 0;
        }
        else
        {
            OC3_Status = 0;
            _OC3 = 0;
            T3CONbits.TON = 0; 
            TMR3 = 0;
        }
    }
    else if(FUNC_CONFIG_RIGHT == 1)
    {
        if(OC1_Status == 0)
        {
            OC1_Status = 1;
            _OC1 = 1;
            PR3 = _ON_TIME;
            TMR3 = 0;
        }
        else
        {
            OC1_Status = 0;
            _OC1 = 0;
            T3CONbits.TON = 0; 
            TMR3 = 0;
        }
    }
    IFS0bits.T3IF = 0;*/
}

void __attribute__ ( ( __interrupt__ , no_auto_psv ) ) _T4Interrupt ( void )
{
    /*if(FUNC_CONFIG_LEFT == 1)
    {        
        if(OC1_Status == 0)
        {
            OC1_Status = 1;
            _OC1 = 1;
            PR4 = _ON_TIME;
            TMR4 = 0;
        }
        else
        {
            OC1_Status = 0;
            _OC1 = 0;
            T4CONbits.TON = 0; 
            TMR4 = 0;
        }
    }
    else if(FUNC_CONFIG_RIGHT == 1)
    {
        if(OC2_Status == 0)
        {
            OC2_Status = 1;
            _OC2 = 1;
            PR4 = _ON_TIME;
            TMR4 = 0;
        }
        else
        {
            OC2_Status = 0;
            _OC2 = 0;
            T4CONbits.TON = 0;  
            TMR4 = 0;
        }
    }
    IFS1bits.T4IF = 0;*/
}



void main(void) 
{
    unsigned char i,i1,j,j1;
    int temp = 0;
    i = i1 = j = 0;
    i = 0;
    j1 = 0;
   
    PIC_Init();
//    _OC1 = 0;
//    _OC2 = 0;
//    _OC3 = 0;
    
    WATCH_DOG_FAIL = RCONbits.WDTO;
    RCONbits.WDTO = 0;
    Register_Counter_Limit = _END_OF_RO_FLOAT_ADDRESS + _END_OF_RO_WORD_ADDRESS + 
                             _END_OF_RW_BYTE_ADDRESS + _END_OF_RO_FLAG_ADDRESS;
    End_Of_Float = _END_OF_RO_FLOAT_ADDRESS;
    End_Of_Word = _END_OF_RO_FLOAT_ADDRESS + _END_OF_RO_WORD_ADDRESS;
    End_Of_Byte = _END_OF_RO_FLOAT_ADDRESS + _END_OF_RO_WORD_ADDRESS + 
                  _END_OF_RW_BYTE_ADDRESS;
    End_Of_Flag = _END_OF_RO_FLOAT_ADDRESS + _END_OF_RO_WORD_ADDRESS + 
                  _END_OF_RW_BYTE_ADDRESS + _END_OF_RO_FLAG_ADDRESS;
    
    
    Restore(); 
    
    
    
    // <editor-fold defaultstate="collapsed" desc="ONE CYCLE PROGRAM1">
    
    // <editor-fold defaultstate="collapsed" desc="ENABLE EXTERNAL INTERRUPT">
    External_Interrupt(_ON);    
    // </editor-fold>
    __delay_ms(500);

    // </editor-fold>
    
    // <editor-fold defaultstate="collapsed" desc="ONE CYCLE PROGRAM2">
//    External_Interrupt(_OFF); 
    
    while(/*(F_LEFT == 0 && F_RIGHT == 0) && */Phase_Direct_Detect_Counter < 200)
    {
        if(Over_Flow_Timer1_Flag == 1)
        {
            Phase_Direct_Detect_Counter ++;
            Over_Flow_Timer1_Flag = 0;
        }
        // <editor-fold defaultstate="collapsed" desc="LEFT ROTATION">
        if((CMP_Arrays(Phase_Sync_Counter,_0_1_2) == 1) || 
           (CMP_Arrays(Phase_Sync_Counter,_2_0_1) == 1) ||
           (CMP_Arrays(Phase_Sync_Counter,_1_2_0) == 1))
        {
            F_LEFT = 1;
            F_RIGHT = 0;
        }

        // </editor-fold>

        // <editor-fold defaultstate="collapsed" desc="RIGHT ROTATION">
        if((CMP_Arrays(Phase_Sync_Counter,_0_2_1) == 1) || 
           (CMP_Arrays(Phase_Sync_Counter,_1_0_2) == 1) ||
           (CMP_Arrays(Phase_Sync_Counter,_2_1_0) == 1))
        {
            F_RIGHT = 1;
            F_LEFT = 0;
        }

        // </editor-fold>
    }
    //External_Interrupt(_OFF); 
    // <editor-fold defaultstate="collapsed" desc="PHASE LOSS">
    if(Phase_Sync_Counter[0] > 3)
        F_L1_LOSS = 1;
    
    if(Phase_Sync_Counter[1] > 3)
        F_L2_LOSS = 1;
    
    if(Phase_Sync_Counter[2] > 3)
        F_L3_LOSS = 1;   
    
    if((CMP_Arrays(Phase_Sync_Counter,_0_0_0) == 1))
    {
        F_L1_LOSS = 1;
        F_L2_LOSS = 1;
        F_L3_LOSS = 1;
    }
    
    if(F_L1_LOSS == 1 || F_L2_LOSS == 1 || F_L3_LOSS == 1)
    {
        F_NO_LOSS = 0;
        F_LEFT = 0;
        F_RIGHT = 0;
    }
    else
        F_NO_LOSS = 1;
    // </editor-fold>
    
    // <editor-fold defaultstate="collapsed" desc="FUNC CONFIG LEFT">
    if(F_NO_LOSS == 1 && F_LEFT == 1)
    {
        FUNC_CONFIG_LEFT1 = 1;
    }
    else
        FUNC_CONFIG_LEFT1 = 0;
    // </editor-fold>
    
    // <editor-fold defaultstate="collapsed" desc="FUNC CONFIG RIGHT">
    if(F_NO_LOSS == 1 && F_RIGHT == 1)
    {
        FUNC_CONFIG_RIGHT1 = 1;
    }
    else
        FUNC_CONFIG_RIGHT1 = 0;
    // </editor-fold>

    Word_Type[_AI_VOUT] = ADCBUF2;
    AI_OFF_SCR_BATTERY_VOLTAGE = Word_Type[_AI_VOUT];
    // </editor-fold>
    
//    Phase_Sync_Counter[0] = 0;
//    Phase_Sync_Counter[1] = 0;
//    Phase_Sync_Counter[2] = 0;
//    External_Interrupt(_ON); 

    
    PID_Initialize(); 
    _Wdt_Status = _ON;
    modbusSetAddress(clientAddress);
    modbusInit();
    READY = 0;
    while (1)
    {                    
        ClrWdt();
         
        modbusGet();
        //ADC VALUE UPDATE
//        Word_Type[_AI_VOUT] = ADCBUF0;
//        Word_Type[_AI_IOUT] = ADCBUF4;
//        Word_Type[_AI_IBAT] = ADCBUF5;
//        Word_Type[_AI_VIN] = ADCBUF1;        
//        Word_Type[_AI_BAT_TEMP] = ADCBUF2;       
//        Word_Type[_AI_HSINK_TEMP] = ADCBUF3; 
        
        Word_Type[_AI_VOUT] = ADCBUF2;
        Word_Type[_AI_IOUT] = ADCBUF1;
        Word_Type[_AI_IBAT] = ADCBUF0;
        Word_Type[_AI_VIN] = ADCBUF3;        
        Word_Type[_AI_BAT_TEMP] = ADCBUF4;       
        Word_Type[_AI_HSINK_TEMP] = ADCBUF5; 
        
        
        F_PID_CLK = 0;
        
        // <editor-fold defaultstate="collapsed" desc="FILTER UPDATE">        
        if(F_FILTER_UPDATE == 0)
        {
            C_NO_OF_CYCLE ++;            
            AI_VO_SUM = AI_VO_SUM + Word_Type[_AI_VOUT];
            AI_IOUT_SUM = AI_IOUT_SUM + Word_Type[_AI_IOUT];
            AI_IBAT_SUM = AI_IBAT_SUM + Word_Type[_AI_IBAT];
            AI_VIN_SUM = AI_VIN_SUM + Word_Type[_AI_VIN];
            AI_BAT_TEMP_SUM = AI_BAT_TEMP_SUM + Word_Type[_AI_BAT_TEMP];
            AI_HITSINK_TEMP_SUM = AI_HITSINK_TEMP_SUM + Word_Type[_AI_HSINK_TEMP];
        }
        if(F_FILTER_UPDATE == 1)
        {   
            Word_Type[_U_NOC] = C_NO_OF_CYCLE;
            Word_Type[_r_VOF] = AI_VO_SUM / C_NO_OF_CYCLE;
            F_VO_UPDATE = 1;
            
            Word_Type[_r_IO_F] = AI_IOUT_SUM / C_NO_OF_CYCLE;
            F_IO_UPDATE = 1;
            
            Word_Type[_r_IB_F] = AI_IBAT_SUM / C_NO_OF_CYCLE;
            F_IB_UPDATE = 1;
            
            Word_Type[_r_VIN_F] = AI_VIN_SUM / C_NO_OF_CYCLE;
            F_VIN_UPDATE = 1;
            
            Word_Type[_r_B_TEMP_F] = AI_BAT_TEMP_SUM / C_NO_OF_CYCLE;
            F_B_TEMP_UPDATE = 1;
            
            Word_Type[_r_H_TEMP_F] = AI_HITSINK_TEMP_SUM / C_NO_OF_CYCLE;
            F_H_TEMP_UPDATE = 1;
            
            VO_PV_SUM = r[_r_VO_PV] + VO_PV_SUM;
            VI_PV_SUM = r[_r_VIN] + VI_PV_SUM;
            IB_PV_SUM = r[_r_IB_PV] + IB_PV_SUM;
            IO_PV_SUM = r[_r_IO_PV] + IO_PV_SUM;
            TB_PV_SUM = r[_r_BAT_TEMP] + TB_PV_SUM;
            TH_PV_SUM = (float)Word_Type[_r_HSINK] + TH_PV_SUM;
            Frq_PV_SUM = (float)Word_Type[_U_TEST4] + Frq_PV_SUM;
            Second_Filter_Counter ++;
            if(Second_Filter_Counter == 50)
            {
                Word_Type[_U_VO_PV] = (unsigned int)((VO_PV_SUM / 50.00) * 10.00);
                Word_Type[_U_VI_PV] = (unsigned int)((VI_PV_SUM / 50.00) * 10.00);
                Word_Type[_U_IB_PV] = (unsigned int)((IB_PV_SUM / 50.00) * 10.00);
                Word_Type[_U_IO_PV] = (unsigned int)((IO_PV_SUM / 50.00) * 10.00);
                Word_Type[_U_TB_PV] = (unsigned int)((TB_PV_SUM / 50.00) * 10.00);
                Word_Type[_U_TH_PV] = (unsigned int)((TH_PV_SUM / 50.00) * 10.00);
                Temporary_long_Register1 = 1280900000;
                Word_Type[_U_FRQ_PV] = (unsigned int)(Temporary_long_Register1 / (Frq_PV_SUM));
                VO_PV_SUM = 0.00;
                VI_PV_SUM = 0.00;
                IB_PV_SUM = 0.00;
                IO_PV_SUM = 0.00;
                TB_PV_SUM = 0.00;
                TH_PV_SUM = 0.00;
                Frq_PV_SUM = 0;
                Second_Filter_Counter = 0;
            }
            
        }
        if(F_VO_UPDATE == 1 && F_IO_UPDATE == 1 && F_IB_UPDATE == 1 && F_VIN_UPDATE == 1)
        {
            AI_VO_SUM = 0;
            AI_IOUT_SUM = 0;
            AI_IBAT_SUM = 0;
            AI_VIN_SUM = 0;
            AI_BAT_TEMP_SUM = 0;
            AI_HITSINK_TEMP_SUM = 0;
            F_PID_CLK = 1;
            F_FILTER_UPDATE = 0;
            
            C_NO_OF_CYCLE = 0;
            F_VO_UPDATE = 0;
            F_IO_UPDATE = 0;
            F_IB_UPDATE = 0;
            F_VIN_UPDATE = 0;
            F_B_TEMP_UPDATE = 0;
            F_H_TEMP_UPDATE = 0;
        }
        // </editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="OUTPUT VOLTAGE">         
        //FILTER SELECT
        if(Flag_Type[_F_VOFS] == 1)
        {
            Word_Type[_r_VO] = Word_Type[_r_VOF];
            Word_Type[_r_B_TEMP] = Word_Type[_r_B_TEMP_F];
        }
        else if(Flag_Type[_F_VOFS] == 0)
        {
            Word_Type[_r_VO] = Word_Type[_AI_VOUT];
            Word_Type[_r_B_TEMP] = Word_Type[_AI_BAT_TEMP];
        }       
        
        //SIGNAL CONDITIONER
        if(Word_Type[_AI_BAT_TEMP] > _NO_PT100_VALUE && PT100_LOSS_FLAG == 0)
        {
            PT100_LOSS_FLAG = 1;
            r[_r_VO_SP] = 26.5;
        }
        else if(Word_Type[_AI_BAT_TEMP] < _NO_PT100_VALUE)
            PT100_LOSS_FLAG = 0;
        if(Equalization_Charge_Flag == 1)
            VO_SP_Temporary = r[_r_VO_SP] * 1.04;
        else
            VO_SP_Temporary = r[_r_VO_SP];
            
        r[_r_VO_PV] = VO_G * ((float)Word_Type[_r_VO] - LOU_WORD); 
        //Word_Type[_U_VO_PV] = (unsigned int)(r[_r_VO_PV] * 10);
        //SETPOINT SELECT
        r[_r_BAT_TEMP] = TBAT_G * ((float)Word_Type[_r_B_TEMP] - TBAT_WZ) + Lbt_REAL;
        r[_r_VO_TEMP_SP] = (0 - r[_r_BAT_TEMP]) * 0.06 + VO_SP_Temporary;
        Word_Type[_U_VO_TEMP_SP] = (unsigned int)(r[_r_VO_TEMP_SP] * 10.00);
        
        if(Flag_Type[_F_VO_SP_SEL] == 0 || (Word_Type[_AI_BAT_TEMP] > _NO_PT100_VALUE))
        {
            Set_Point = VO_SP_Temporary;
        }
        else
        {
            //SIGNAL CONDITIONER2            
            Set_Point = r[_r_VO_TEMP_SP];
        }
        
        // <editor-fold defaultstate="collapsed" desc="SET POINT RAMP UP/DOWN"> 
        INT_Set_Point = Set_Point * 1000;
        if(INT_Set_Point > M)
        {
            if(Flag_Type[_F_ERU] == 0)
            {
                Set_Point_Ramp_Output = INT_Set_Point;
            }
            else
            {
                if(READY_STATUS_Counter == 2000)
                    M = M + Word_Type[_r_USTEP];
                if(INT_Set_Point > M)
                    Set_Point_Ramp_Output = M;
                else if(INT_Set_Point < M)
                    Set_Point_Ramp_Output = INT_Set_Point;
            }
        }
        else if((INT_Set_Point < M))
        {
            if(Flag_Type[_F_ERD] == 0)
            {
                Set_Point_Ramp_Output = INT_Set_Point;
            }
            else
            {
                M = M - Word_Type[_r_DSTEP];
                if(INT_Set_Point > M)
                    Set_Point_Ramp_Output = INT_Set_Point;
                else if(INT_Set_Point < M)
                    Set_Point_Ramp_Output = M;
            }
        }
        Set_Point = (float)Set_Point_Ramp_Output / 1000.0;
        // </editor-fold>
        
        r[_r_VO_fsp] = Set_Point;
               
        
        //PID VOLTAGE REGULATOR
        Error_VO = Set_Point - r[_r_VO_PV];
        if(f_event[24] == 1 && (F_PID_CLK == 1 || Flag_Type[_F_VOFS] == 0) && READY_STATUS_Counter == 2000)
        {
            
            fooPID1.controlReference = Float2Fract(Set_Point / (VO_G * (4096.00 - LOU_WORD)));//Q15(0.74) ; //          
            fooPID1.measuredOutput = Float2Fract(r[_r_VO_PV] / (VO_G * (4096.00 - LOU_WORD)));//Q15(0.453); // 
            
            PID(&fooPID1);           
            Temporary_float_Register = (float)((float)(Fract2Float(fooPID1.controlOutput) + 1.00) * (float)_DutyCycle_Range) / 2.00;
            Temporary_long_Register1 = (float)(Temporary_float_Register / 10.00) * Byte_Type[_b_VO_GAIN];
            Temporary_int_Register1 = _DutyCycle_Range;
            if(Temporary_long_Register1 > Temporary_int_Register1)
                Temporary_long_Register1  = Temporary_int_Register1;
            Word_Type[_r_VO_CV] = Temporary_long_Register1;
        }
        //Word_Type[_r_VO_CV] = (int)(Fract2Float(fooPID1.controlOutput) * 1000);
        
        // </editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="CHARGE CURRENT">
        //FILTER SELECT
        if(Flag_Type[_F_IBFS] == 1)
        {
            Word_Type[_r_IB] = Word_Type[_r_IB_F];
            Word_Type[_r_B_TEMP] = Word_Type[_r_B_TEMP_F];
        }
        else if(Flag_Type[_F_IBFS] == 0)
        {
            Word_Type[_r_IB] = Word_Type[_AI_IBAT];
            Word_Type[_r_B_TEMP] = Word_Type[_AI_BAT_TEMP];
        }
        
        //SIGNAL CONDITIONER
        r[_r_IB_PV] = IB_G * ((float)Word_Type[_r_IB] - LBC_WORD); 
        
        //SETPOINT SELECT
        r[_r_BAT_TEMP] = TBAT_G * ((float)Word_Type[_r_B_TEMP] - TBAT_WZ) + Lbt_REAL;
        r[_r_IB_TEMP_SP] = (r[_r_TEMP_DEC_IB] - r[_r_BAT_TEMP]) * IB_DEC_G + r[_r_IB_SP];
        
        if(r[_r_BAT_TEMP] > r[_r_TEMP_DEC_IB])
        {
            Flag_Type[_F_IB_SP_DEC] = 1;
        }
        else
            Flag_Type[_F_IB_SP_DEC] = 0;
        
        if(Flag_Type[_F_IB_SP_DEC] == 1 && Flag_Type[_F_IB_SP_SEL] == 1)
        {
            Flag_Type[_F_IB_SP_SEL_DEC] = 1;
        }
        else
            Flag_Type[_F_IB_SP_SEL_DEC] = 0;
        
        if(Flag_Type[_F_IB_SP_SEL_DEC] == 0 || (Word_Type[_AI_BAT_TEMP] > _NO_PT100_VALUE))
        {
            Set_Point = r[_r_IB_SP];
        }
        else
        {
            //TBAT_G = (float)(Hbt_REAL - Hbt_WORD) / (Lbt_REAL - Lbt_WORD);
            //TBAT_WZ = Lbt_WORD;
            
            Set_Point = r[_r_IB_TEMP_SP];            
        }
        
        //ZERRO CHARGE SELECT
        if(f_event[12] == 1)
        {
            Set_Point = r[_r_IB_SP] * 0.05;
        }
        r[_r_IB_fsp] = Set_Point;
        
        //PID VOLTAGE REGULATOR
        Error_IB = Set_Point - r[_r_IB_PV]; 
        if(f_event[25] == 1 && (F_PID_CLK == 1 || Flag_Type[_F_IBFS] == 0) && READY_STATUS_Counter == 2000)
        {                                           
            
            fooPID2.controlReference = Float2Fract(Set_Point / (IB_G * (4096 - LBC_WORD)));//Q15(0.74) ;           
            fooPID2.measuredOutput = Float2Fract(r[_r_IB_PV] / (IB_G * (4096 - LBC_WORD)));//Q15(0.453);
            
            PID(&fooPID2);
            Temporary_float_Register = (float)((float)(Fract2Float(fooPID2.controlOutput) + 1.00) * (float)_DutyCycle_Range) / 2.00;
            Temporary_long_Register1 = (float)(Temporary_float_Register / 10.00) * Byte_Type[_b_IB_GAIN];
            Temporary_int_Register1 = _DutyCycle_Range;
            if(Temporary_long_Register1 > Temporary_int_Register1)
                Temporary_long_Register1  = Temporary_int_Register1;
            Word_Type[_r_IB_CV] = Temporary_long_Register1;
            //Word_Type[_r_IB_CV] = (int)(Fract2Float(fooPID2.controlOutput) * 1000);
        }
        // </editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="LOAD CURRENT">
        if(Flag_Type[_F_IOFS] == 1)
        {
            Word_Type[_r_IO] = Word_Type[_r_IO_F];
            Word_Type[_r_H_TEMP] = Word_Type[_r_H_TEMP_F];
        }
        else if(Flag_Type[_F_IOFS] == 0)
        {
            Word_Type[_r_IO] = Word_Type[_AI_IOUT];
            Word_Type[_r_H_TEMP] = Word_Type[_AI_HSINK_TEMP];
        }
        
        //SIGNAL CONDITIONER
        r[_r_IO_PV] = IO_G * ((float)Word_Type[_r_IO] - LLC_WORD);
        //HEATSINK TEMPRATURE LUT
        Temporary_float_Register = (float)((float)Word_Type[_r_H_TEMP] - LHt_WORD) * NTC_G;
        if(Temporary_float_Register < 0)
            Temporary_float_Register = 0;
        if(Temporary_float_Register > 254)
            Temporary_float_Register = 254;
        Word_Type[_r_HSINK] = LUT[(unsigned char)(Temporary_float_Register + 0.5)];
        
        //LOAD CURRENT COMPENSATE
        r[_r_IO_TEMP_SP] = (r[_r_TEMP_DEC_IO] - Word_Type[_r_HSINK]) * IO_DEC_G + r[_r_IO_SP];
        
        if(Word_Type[_r_HSINK] > r[_r_TEMP_DEC_IO])
        {
            Flag_Type[_F_IO_SP_DEC] = 1;
        }
        else
            Flag_Type[_F_IO_SP_DEC] = 0; 
        if(Flag_Type[_F_IO_SP_DEC] == 1 && Flag_Type[_F_IO_SP_SEL] == 1)
        {
            Flag_Type[_F_IO_SP_SEL_DEC] = 1;
        }
        else
            Flag_Type[_F_IO_SP_SEL_DEC] = 0;
        
        if(Flag_Type[_F_IO_SP_SEL_DEC] == 0)
        {
            Set_Point = r[_r_IO_SP];
        }
        else
        {            
           Set_Point = r[_r_IO_TEMP_SP];                               
        }
        r[_r_IO_fsp] = Set_Point;
        
        //PID VOLTAGE REGULATOR
        Error_IO = Set_Point - r[_r_IO_PV];
        if(f_event[26] == 1 && (F_PID_CLK == 1 || Flag_Type[_F_IOFS] == 0) && READY_STATUS_Counter == 2000)
        {           
            
            fooPID3.controlReference = Float2Fract(Set_Point / (IO_G * (4096 - LLC_WORD)));//Q15(0.74) ;           
            fooPID3.measuredOutput = Float2Fract(r[_r_IO_PV] / (IO_G * (4096 - LLC_WORD)));//Q15(0.453); 
            
            PID(&fooPID3);
            Temporary_float_Register = (float)((float)(Fract2Float(fooPID3.controlOutput) + 1.00) * (float)_DutyCycle_Range) / 2.00;
            Temporary_long_Register1 = (float)(Temporary_float_Register / 10.00) * Byte_Type[_b_IO_GAIN];
            Temporary_int_Register1 = _DutyCycle_Range;
            if(Temporary_long_Register1 > Temporary_int_Register1)
                Temporary_long_Register1  = Temporary_int_Register1;
            Word_Type[_r_IO_CV] = Temporary_long_Register1;
            //Word_Type[_r_IO_CV] = (int)(Fract2Float(fooPID3.controlOutput) * 1000);
        }
        // </editor-fold>
        
        //INPUT VOLTAGE
        r[_r_VIN]= VI_G * ((float)Word_Type[_r_VIN_F] - LIU_WORD);
        
        // <editor-fold defaultstate="collapsed" desc="MINIMUM SELECT"> 
        
        if(Byte_Type[_b_CMD] != 0)
        {
            CMD_TYPE = Byte_Type[_b_CMD];
        }
        if(Byte_Type[_b_CMD] == 0)
        {
            /*if(Error_VO < 0)
                CMD_TYPE = 1; 
            if(Error_IB < 0)
                CMD_TYPE = 2;
            if(Error_IO < 0 && Error_IO < Error_IB)
                CMD_TYPE = 3;                        
            if(CMD_TYPE != 1 && CMD_TYPE != 2 && CMD_TYPE != 3)
                CMD_TYPE = 1;*/
            if(Error_VO < 0)
                CMD_TYPE = 1; 
            if(Error_IO < 0 /*&& Error_IO < Error_IB*/)
                CMD_TYPE = 3;
            if(Error_IB < 0)
                CMD_TYPE = 2;                                   
            /*if(CMD_TYPE != 1 && CMD_TYPE != 2 && CMD_TYPE != 3)
                CMD_TYPE = 1;*/
        }
        switch(CMD_TYPE)
        {
            case 1:
                Word_Type[_r_CV] = Word_Type[_r_VO_CV];               
            break;
            case 2:
                Word_Type[_r_CV] = Word_Type[_r_IB_CV];
            break;
            case 3:
                Word_Type[_r_CV] = Word_Type[_r_IO_CV];
            break;
            case 4:
                Word_Type[_r_CV] = Word_Type[_r_CMD];
            break;
            case 5:
                Word_Type[_r_CV] = 0;
            break;
        }
        
        /*if(Error_VO < 0)
        {
            VO_PID_Enable = 1;
            IB_PID_Enable = 0;
            IO_PID_Enable = 0;           
        }
        if(Error_IO < 0)
        {
            VO_PID_Enable = 0;
            IB_PID_Enable = 0;
            IO_PID_Enable = 1;
        }
        if(Error_IB < 0)
        {
            VO_PID_Enable = 0;
            IB_PID_Enable = 1;
            IO_PID_Enable = 0;
        }
        if(VO_PID_Enable == 0 && IB_PID_Enable == 0 && IO_PID_Enable == 0)
            VO_PID_Enable = 1;*/
                
        /*if(Error_VO <= Error_IB && Error_VO <= Error_IO)
        {
            VO_PID_Enable = 1;
            IB_PID_Enable = 0;
            IO_PID_Enable = 0;           
        }
        else if(Error_IB <= Error_VO && Error_IB <= Error_IO)
        {
            VO_PID_Enable = 0;
            IB_PID_Enable = 1;
            IO_PID_Enable = 0;
        }
        else if(Error_IO <= Error_IB && Error_IO <= Error_VO)
        {
            VO_PID_Enable = 0;
            IB_PID_Enable = 0;
            IO_PID_Enable = 1;
        }*/
                       
        /*if(VO_PID_Enable == 1)
        {
            Word_Type[_r_CV] = Word_Type[_r_VO_CV];
            CMD_TYPE = 1;
        }
        else if(IB_PID_Enable == 1)
        {
            Word_Type[_r_CV] = Word_Type[_r_IB_CV];
            CMD_TYPE = 2;
        }
        else if(IO_PID_Enable == 1)
        {
            Word_Type[_r_CV] = Word_Type[_r_IO_CV];
            CMD_TYPE = 3;
        }*/
        
        /*switch(Byte_Type[_b_CMD])
        {
            case 0:
                if(Word_Type[_r_VO_CV] <= Word_Type[_r_IB_CV] && Word_Type[_r_VO_CV] <= Word_Type[_r_IO_CV])
                {
                    Word_Type[_r_CV] = Word_Type[_r_VO_CV];
                    CMD_TYPE = 1;
                }
                else if(Word_Type[_r_IB_CV] <= Word_Type[_r_VO_CV] && Word_Type[_r_IB_CV] <= Word_Type[_r_IO_CV])
                {
                    Word_Type[_r_CV] = Word_Type[_r_IB_CV];
                    CMD_TYPE = 2;
                }
                else if(Word_Type[_r_IO_CV] <= Word_Type[_r_IB_CV] && Word_Type[_r_IO_CV] <= Word_Type[_r_VO_CV])
                {
                    Word_Type[_r_CV] = Word_Type[_r_IO_CV]; 
                    CMD_TYPE = 3;
                }
            break;
            case 1:
                Word_Type[_r_CV] = Word_Type[_r_VO_CV];
                CMD_TYPE = 1;
            break;
            case 2:
                Word_Type[_r_CV] = Word_Type[_r_IB_CV];
                CMD_TYPE = 2;
            break;
            case 3:
                Word_Type[_r_CV] = Word_Type[_r_IO_CV];
                CMD_TYPE = 3;
            break;
            case 4:
                Word_Type[_r_CV] = Word_Type[_r_CMD];
                CMD_TYPE = 0;
            break;
        }*/
        
        // </editor-fold>
        
        if(ERR_OUT == 0)
        {
            OC1RS = 0;
            //CMD_TYPE = 5;
            Word_Type[_r_VO_CV] = 0;                           
            Word_Type[_r_IB_CV] = 0;
            Word_Type[_r_IO_CV] = 0;
            Word_Type[_r_CV] = 0;
            PID_Ramp_Disable = 0;
            M2 = 0;
        }
        
        // <editor-fold defaultstate="collapsed" desc="RAMP UP/DOWN">        
        if(Word_Type[_r_CV] > M2)
        {
            if(PID_Ramp_Disable == 1)
            {
                Ramp_Output = Word_Type[_r_CV];
            }
            else
            {
                if(READY_STATUS_Counter == 2000)
                    M2 = M2 + 1/*Word_Type[_r_USTEP]*/;
                if(Word_Type[_r_CV] > M2)
                    Ramp_Output = M2;
                else if(Word_Type[_r_CV] < M2)
                {
                    Ramp_Output = Word_Type[_r_CV];
                    PID_Ramp_Disable = 1;
                }
            }
        }
        else if((Word_Type[_r_CV] < M2))
        {
            if(1/*Flag_Type[_F_ERD] == 0*/)
            {
                Ramp_Output = Word_Type[_r_CV];
            }
            else
            {
                M2 = M2 - Word_Type[_r_DSTEP];
                if(Word_Type[_r_CV] > M2)
                    Ramp_Output = Word_Type[_r_CV];
                else if(Word_Type[_r_CV] < M2)
                    Ramp_Output = M2;
            }
        }
        //Ramp_Output = Word_Type[_r_CV];
        // </editor-fold>
        
        //PRESET PWM
        Word_Type[_U_CV] = Ramp_Output;        
        
        
        //L1,L2,L3 CURRENT BALANCE
        if(Word_Type[_U_CV] > _DutyCycle_Range)
            Word_Type[_U_CV] = _DutyCycle_Range;
        
        if(ERR_OUT == 0)
        {
            OC1RS = 0;
            //CMD_TYPE = 5;
            Word_Type[_r_VO_CV] = 0;                           
            Word_Type[_r_IB_CV] = 0;
            Word_Type[_r_IO_CV] = 0;
            Word_Type[_U_CV] = 0;
            PID_Ramp_Disable = 0;
            M2 = 0;
        }
        else if(F_FAIL == 0 && Flag_Type[_F_CHARGER_DISABLE] == 0 && (FUNC_CONFIG_LEFT == 1 || FUNC_CONFIG_RIGHT == 1 || Flag_Type[_F_PHASE_LOSS_MASK] == 1))
        {
            OC1RS = Word_Type[_U_CV];

        }
        else
        {
            OC1RS = 0;
            //READY = 0;
            DRIVE_RESET_Counter = 0;
            //READY_STATUS_Counter = 0;
            //CMD_TYPE = 5;
            Word_Type[_r_VO_CV] = 0;                           
            Word_Type[_r_IB_CV] = 0;
            Word_Type[_r_IO_CV] = 0;
            Word_Type[_U_CV] = 0;

        }
                             
        /*
        if(Word_Type[_U_L1] < Word_Type[_U_CV])
            Word_Type[_U_PRE1] = (Word_Type[_U_L1] - _Timer_Range);
        else
            Word_Type[_U_PRE1] = Word_Type[_U_L1] - Word_Type[_U_CV];        
        
        if(Word_Type[_U_L2] < Word_Type[_U_CV])
            Word_Type[_U_PRE2] = (Word_Type[_U_L2] - _Timer_Range);
        else
            Word_Type[_U_PRE2] = Word_Type[_U_L2] - Word_Type[_U_CV];        
        
        if(Word_Type[_U_L3] < Word_Type[_U_CV])
            Word_Type[_U_PRE3] = (Word_Type[_U_L3] - _Timer_Range);
        else
            Word_Type[_U_PRE3] = Word_Type[_U_L3] - Word_Type[_U_CV];   */     
        
        if(Key_Pressed_Flag == 1)
        {
            // <editor-fold defaultstate="collapsed" desc="Key">
            TTCounter = 0;
            Key_Pressed_Flag = 0;
            if(Screen_Saver_Enable == 0)
                Perform_Command();
            //Update_7Segment_Display();
            Screen_Saver_Enable = 0;
            Screen_Saver_Counter = 0;
                   
            //DRV_UART1_WriteByte(0x07);
           
            // </editor-fold>
        }

        if(Over_Flow_Timer1_Flag == 1)
        {
            Over_Flow_Timer1_Flag = 0;                                                        
            iTCounter ++;
            // <editor-fold defaultstate="collapsed" desc="Over Flow">  
            Set_Segment(Segment_Digit[Set_Segment_Counter], Set_Segment_Counter, Segment_Dot[Set_Segment_Counter]);
            Set_Segment_Counter ++;
            if(Set_Segment_Counter == 3)
                Set_Segment_Counter = 0;           
            
            if(Screen_Saver_Enable == 0)
            {
                Screen_Saver_Counter ++;
                if(Screen_Saver_Counter > 1000)
                {
                    Screen_Saver_Counter = 0;
                    for(j = 0; j < 33; j ++)
                    {
                        if((f_property[j] == 1 || f_property[j] == 3) && f_event[j] == 1)
                        {
                            Screen_Saver_Enable = 1;
                            break;
                        }                         
                    }
                    
                }                    
            }
            
            if(iTCounter % 800 == 0 && Screen_Saver_Enable == 1)
            {                          
                // <editor-fold defaultstate="collapsed" desc="Screen_Saver">
                for(j = 0; j < 33; j ++)
                {
                    if(f_property[j] == 1 || f_property[j] == 3)
                        temp = temp + f_event[j];
                }
                if(temp == 0)
                {
                    Screen_Saver_Enable = 0;
                    //Update_7Segment_Display();
                }
                else
                    temp = 0;
                    
                while(f_event[i1] == 0 || f_property[i1] == 2 || f_property[i1] == 0)
                {
                    i1 ++;
                    if(i1 == 33)
                    {
                        iTCounter = 788;
                        break;
                    }
                }
                if(i1 < 33)
                    Show_Screen_Saver(i1 + 1);
                i1 ++;
                if(i1 >= 33)
                    i1 = 0;
                // </editor-fold>
            }
            
            if(iTCounter % 20 == 0)
            {
                // <editor-fold defaultstate="collapsed" desc="Fault">
                if(f_event[i] == 1 && f_event_already_happened[i] == 0 && (f_property[i] == 2 || f_property[i] == 3))
                {
                    
                    f_event_already_happened[i] = 1;                    
                    _Wdt_Status = _OFF;
                    Que_Alarm[Last_Event_Number] = i+1;
                    inputRegisters[7] = i+1; 
                    ee_set_Que_Alarm_ADRESS(Que_Alarm[Last_Event_Number],Last_Event_Number);
                    Last_Event_Number = (Last_Event_Number + 1) % 10;
                    ee_set_POINTER_OF_LAST_EVENT(Last_Event_Number);
                    /*for(j = 8; j > 0; j--)
                    {
                        Que_Alarm[j + 1] = Que_Alarm[j];
                        //ee_set_Que_Alarm_ADRESS(Que_Alarm[j + 1],j + 1);
                    }
                    Que_Alarm[1] = Que_Alarm[0];
                    //ee_set_Que_Alarm_ADRESS(Que_Alarm[1],1);
                    Que_Alarm[0] = i+1;
                    //ee_set_Que_Alarm_ADRESS(Que_Alarm[0],0);
                    Que_Alarm_Update_Flag = 1;*/
                    _Wdt_Status = _ON;
                }
                i ++;
                    if(i == 33)
                        i = 0;
                // </editor-fold>
            }            
            /*if(Que_Alarm_Update_Flag == 1)
            {
                Start_Que_Alarm_Update_Flag = 1;
                Que_Alarm_Update_Flag = 0;
                Que_Alarm_Update_Counter = 0;
            }
            if(Start_Que_Alarm_Update_Flag == 1)
            {
                ee_set_Que_Alarm_ADRESS(Que_Alarm[Que_Alarm_Update_Counter],Que_Alarm_Update_Counter);
                Que_Alarm_Update_Counter ++;
                if(Que_Alarm_Update_Counter == 33)
                {
                    Que_Alarm_Update_Counter = 0;
                    Start_Que_Alarm_Update_Flag = 0;
                }
            }*/
            
            if(Run_Function_Start_Flag == 1)
            {
                Run_Function_Start_Counter ++;
                if(Run_Function_Start_Counter == 200)
                {
                    Run_Function_Start_Counter = 0;
                    Flag_Type[Register_Counter - End_Of_Byte] = 0;
                    Run_Function_Start_Flag = 0;
                    //Update_7Segment_Display();
                }
                
            }
            if(Screen_Saver_Enable == 0)
                Update_7Segment_Display();
            
            
            if(Flag_Type[_F_MANUAL_EQUAlIZATUIN_CHARGE] == 1)
            {
                Manual_Equalization_Charge_Counter++;
                Flag_Type[_F_EQUAlIZATUIN_CHARGE] = 0;
                Equalization_Charge_Flag = 1;
                if(Manual_Equalization_Charge_Counter > (60000))
                {
                    Manual_Equalization_Charge_Counter = (60000 + 1);
                    if(f_event[24] == 1)
                    {
                        Manual_Equalization_Charge_Counter = 0;
                        Equalization_Charge_Flag = 0;
                        Flag_Type[_F_MANUAL_EQUAlIZATUIN_CHARGE] = 0;
                    }

                }
                
            }
            else               
            {
                Manual_Equalization_Charge_Counter = 0;
                Equalization_Charge_Flag = 0;
            }
            if(Flag_Type[_F_EQUAlIZATUIN_CHARGE] == 1)
            {
                Equalization_Charge_Counter++;
                if(Equalization_Charge_Counter > 17280000)
                {
                    Equalization_Charge_Flag = 1;
                    if(Equalization_Charge_Counter > (17280000 + 60000))
                    {
                        Equalization_Charge_Counter = (17280000 + 60000 + 1);
                        if(f_event[24] == 1)
                        {
                            Equalization_Charge_Counter = 0;
                            Equalization_Charge_Flag = 0;
                        }

                    }


                }
            }
            else if(Flag_Type[_F_MANUAL_EQUAlIZATUIN_CHARGE] == 0)               
            {
                Equalization_Charge_Counter = 0;
                Equalization_Charge_Flag = 0;
            }
            
            if(F_NO_LOSS == 0)
            {
                F_FILTER_UPDATE_Counter ++;
                if(F_FILTER_UPDATE_Counter == 4)
                {
                    F_FILTER_UPDATE_Counter = 0;
                    F_FILTER_UPDATE = 1;
                }
            }
            // </editor-fold> 
            
            if(Start_Output_Over_Under_V_Flag == 1)
                Output_Over_Under_V_Counter ++;
            if(Load_Battery_Over_I_Flag == 1)
                Load_Battery_Over_I_Counter ++;
            
            
            if(F_FAIL == 0 && Flag_Type[_F_CHARGER_DISABLE] == 0 && (FUNC_CONFIG_LEFT == 1 || FUNC_CONFIG_RIGHT == 1 || Flag_Type[_F_PHASE_LOSS_MASK] == 1))
            {
                // <editor-fold defaultstate="collapsed" desc="Drive Reset"> 

                DRIVE_RESET_Counter ++;
                if(DRIVE_RESET_Counter == 2000)
                    DRIVE_RESET = 1;
                if(DRIVE_RESET_Counter == 2001)
                {
                    DRIVE_RESET = 0;
                    DRIVE_RESET_Counter = 0;
                }

                // </editor-fold>

                // <editor-fold defaultstate="collapsed" desc="READY STATUS"> 
                if(READY_STATUS_Counter != 2000)
                {
                    if((Word_Type[_AI_VIN] > (0.6 * 4096)))
                    {
                        READY = 1;
                        READY_STATUS_Counter = 2000;
                    }
                    else
                    {                
                        READY_STATUS_Counter ++;
                        if(READY_STATUS_Counter == 1000)
                        {
                            READY = 1;
                            READY_STATUS_Counter = 2000;
                        }

                    }
                }
                // </editor-fold>

            }
        }
        
        TTCounter++;            
            
        // <editor-fold defaultstate="collapsed" desc="KEY">
        if((TTCounter % 50) == 0)
        {
            // <editor-fold defaultstate="collapsed" desc="KEY Detector">
            switch(Previous_Pressed_Key)
            {
                  case _ENTER:
                          if(!(M_Key == 0))
                          {
                                  Key_Released_Flag = 1;
                                  Detected_Key = _NONE;
                                  TTCounter_Limit = 250;
                          }
                  break;
                  case _UP:
                          if(!(P_Key == 0))
                          {
                                  Key_Released_Flag = 1;
                                  Detected_Key = _NONE;
                                  Debounce_Limit = _Debounce_Limit;
                                  TTCounter_Limit = 250;
                          }
                  break;
                  case _DOWN:
                          if(!(N_Key == 0))
                          {
                                  Key_Released_Flag = 1;
                                  Detected_Key = _NONE;
                                  Debounce_Limit = _Debounce_Limit;
                                  TTCounter_Limit = 250;
                          }
                  break;

            }
            // </editor-fold>
        }
    //    if(TTCounter < TTCounter_Limit)
    //          return;
        if(TTCounter >= TTCounter_Limit)//else
        {
            TTCounter = TTCounter_Limit - 3;
            // <editor-fold defaultstate="collapsed" desc="M_Key">
            if((M_Key == 0)  && (Key_Pressed_Flag == 0)  && (Key_Released_Flag == 1))
            {
                  Pressed_Key = _ENTER;

                  if(Pressed_Key == Previous_Pressed_Key)
                  {
                          Debounce_Counter++;

                          if(Debounce_Counter == Debounce_Limit)
                          {

                              Key_Pressed_Flag = 1;
                                  Previous_Detected_Key = Detected_Key;
                                  Detected_Key = _ENTER;
    //                                Key_Released_Flag = 0;
                                  Debounce_Counter = 0;
                          }
                  }
                  else
                  {
                          Debounce_Counter = 0;
                  }
                  Previous_Pressed_Key = _ENTER;
            }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="P_Key">
            if((P_Key == 0)  && (Key_Pressed_Flag == 0)  && (Key_Released_Flag == 1))
            {
                  Pressed_Key = _UP;
                  if(Pressed_Key == Previous_Pressed_Key)
                  {
                          Debounce_Counter++;
                          if(Debounce_Counter == Debounce_Limit)
                          {
                                  Key_Pressed_Flag = 1;
                                  Previous_Detected_Key = Detected_Key;
                                  Detected_Key = _UP;
    //                                Key_Released_Flag = 0;
                                  Debounce_Counter = 0;
                          }
                  }
                  else
                  {
                          Debounce_Counter = 0;
                  }
                  Previous_Pressed_Key = _UP;
            }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="N_Key">
            if((N_Key == 0)  && (Key_Pressed_Flag == 0)  && (Key_Released_Flag == 1))
            {
                  Pressed_Key = _DOWN;
                  if(Pressed_Key == Previous_Pressed_Key)
                  {
                          Debounce_Counter++;
                          if(Debounce_Counter == Debounce_Limit)
                          {
                                  Key_Pressed_Flag = 1;
                                  Previous_Detected_Key = Detected_Key;
                                  Detected_Key = _DOWN;
    //                                Key_Released_Flag = 0;
                                  Debounce_Counter = 0;
                          }
                  }
                  else
                  {
                          Debounce_Counter = 0;
                  }
                  Previous_Pressed_Key = _DOWN;
            }
            // </editor-fold>  
        }
        // </editor-fold>
                
        
        if(F_FAIL == 0)
        {
            Check_Phase_Sequence();
            Event_Flag_Update();
        }
        
        
        if(TTCounter > 6000)
            TTCounter = 0;
        if(iTCounter > 6000)
            iTCounter = 0;
        if(Output_Over_Under_V_Counter > 6000)
            Output_Over_Under_V_Counter = 0;
        if(Load_Battery_Over_I_Counter > 6000)
            Load_Battery_Over_I_Counter = 0;
        
                         
    }
    
}


void PIC_Init(void)
{
    //---------------PORT Init-------------------------------------------------
    ADPCFGbits.PCFG6 = 1;       
    ADPCFGbits.PCFG7 = 1;    
    ADPCFGbits.PCFG8 = 1;  
    ADPCFGbits.PCFG9 = 1;       
    ADPCFGbits.PCFG10 = 1;    
    ADPCFGbits.PCFG11 = 1;
    ADPCFGbits.PCFG12= 1;
    TRISBbits.TRISB6 = 1;//using pin as input    
    TRISBbits.TRISB7 = 1;    
    TRISBbits.TRISB8 = 1;    
    TRISBbits.TRISB12 = 0;  
    TRISBbits.TRISB11 = 0; 
    TRISBbits.TRISB10 = 0;  
    TRISBbits.TRISB9 = 0; 
    
    TRISFbits.TRISF0 = 0;//using pin as output   
    TRISFbits.TRISF1 = 0;    
    TRISFbits.TRISF2 = 0;    
    TRISFbits.TRISF3 = 0;   
    TRISFbits.TRISF4 = 0;    
    TRISFbits.TRISF5 = 0;    
    TRISFbits.TRISF6 = 0;    
    //TRISCbits.TRISC13 = 0;   
    //TRISCbits.TRISC14 = 0;    
    
    TRISDbits.TRISD0 = 0;
    TRISDbits.TRISD1 = 1;
    TRISDbits.TRISD2 = 0;
    TRISDbits.TRISD3 = 0;
           
    //---------------Timer Init-------------------------------------------------
    T1CON = 0x8000;
    IEC0bits.T1IE = 1;
    IFS0bits.T1IF = 0;
    //IPC0 = IPC0 | 0x1000;
    PR1 = 2000;//12500;
    IPC0bits.T1IP = 5;
      
    T2CON	 = 0x0000;
    //T2CONbits.TCKPS = 1;
    //IEC0bits.T2IE = 1;
    IFS0bits.T2IF = 0;
    //IPC0 = IPC0 | 0x1000;
    //IPC1bits.T2IP = 6;
    PR2 = _PWM_Period;
    OC1RS = 0;
    OC1R = 0;
    OC1CONbits.OCTSEL = 0;
    OC1CONbits.OCM = 6;
    T2CONbits.TON = 1;
    
    /*//T3CON	 = 0x8000;
    T3CONbits.TCKPS = 1;
    IEC0bits.T3IE = 1;
    IFS0bits.T3IF = 0;
    //IPC0 = IPC0 | 0x1000;
    IPC1bits.T3IP = 6;
    //PR3 = 25000;
    
    //T4CON	 = 0x8000;
    T4CONbits.TCKPS = 1;
    IEC1bits.T4IE = 1;
    IFS1bits.T4IF = 0;
    //IPC0 = IPC0 | 0x1000;
    IPC5bits.T4IP = 6;
    //PR4 = 25000;*/
    
    T5CON = 0x8010;
    //IPC0 = IPC0 | 0x1000;
    PR5	= 60000;
    
    //++++++++++++++++++ADC++++++++++++++++++++++++++++++++++++++++++++
    
    ADCON1bits.FORM = 0;
    ADCON1bits.SSRC = 7;
    

    ADCON2bits.VCFG = 7;
    ADCON2bits.CSCNA = 1;
    ADCON2bits.SMPI = 5;
    ADCON2bits.BUFM=0;
    ADCON2bits.ALTS=0;

    ADCON3bits.SAMC=16;
    ADCON3bits.ADRC=0;
    ADCON3bits.ADCS=31;

    ADCHSbits.CH0NB=0;
    ADCHSbits.CH0NA=0;
    ADCHSbits.CH0SA=0;
    ADCHSbits.CH0SB=0;

    ADPCFGbits.PCFG0 = 0;
    ADPCFGbits.PCFG1 = 0;
    ADPCFGbits.PCFG2 = 0;
    ADPCFGbits.PCFG3 = 0;
    ADPCFGbits.PCFG4 = 0;
    ADPCFGbits.PCFG5 = 0;    

    ADCSSLbits.CSSL0 = 1;
    ADCSSLbits.CSSL1 = 1;
    ADCSSLbits.CSSL2 = 1;
    ADCSSLbits.CSSL3 = 1;
    ADCSSLbits.CSSL4 = 1;
    ADCSSLbits.CSSL5 = 1;    
   
    ADCON1bits.ASAM = 1;
    ADCON1bits.SAMP = 1;
    ADCON1bits.ADON = 1;
    
//    ADCON2bits.VCFG = 7;
//    ADCON1bits.SSRC = 7;
//    ADCON2bits.SMPI = 2;
//    ADCON2bits.CSCNA = 1;
//    ADCSSLbits.CSSL2 = 1;
//    ADCSSLbits.CSSL10 = 1;
    
    ADCON1bits.ASAM = 1;
    ADCON1bits.SAMP = 1;

    ADCON3bits.ADCS = 30;  // Tad = 266ns, conversion time is 12*Tad
    ADCON1bits.ADON = 1;   // Turn ADC ON
    
    DRV_UART1_InitializerDefault();
}

void Show_Screen_Saver(unsigned char index)
{
    inputRegisters[6] = index;
    
    switch(index)
    {
        // <editor-fold defaultstate="collapsed" desc="Alarm">
        case 0:

        break;
        case 1:
            Segment_Digit[2] = _l;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _e;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _f;
            Segment_Dot[0] = _OFF;
        break;
        case 2:
            Segment_Digit[2] = _r;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _i;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _t;
            Segment_Dot[0] = _OFF;
        break;
        case 3:
            Segment_Digit[2] = _l;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = 1;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _l;
            Segment_Dot[0] = _OFF;
        break;
        case 4:
            Segment_Digit[2] = _l;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = 2;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _l;
            Segment_Dot[0] = _OFF;
        break;
        case 5:
            Segment_Digit[2] = _l;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = 3;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _l;
            Segment_Dot[0] = _OFF;
        break;
        case 6:
            Segment_Digit[2] = _f;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _d;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _e;
            Segment_Dot[0] = _OFF;
        break;
        case 7:
            Segment_Digit[2] = _o;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _u;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _o;
            Segment_Dot[0] = _OFF;
        break;
        case 8:
            Segment_Digit[2] = _u;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _u;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _o;
            Segment_Dot[0] = _OFF;
        break;
        case 9:
            Segment_Digit[2] = _o;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _c;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _b;
            Segment_Dot[0] = _OFF;
        break; 
        case 10:
            Segment_Digit[2] = _u;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _c;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _b;
            Segment_Dot[0] = _OFF;
        break;
        case 11:
            Segment_Digit[2] = _o;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _c;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _l;
            Segment_Dot[0] = _OFF;
        break;
        case 12:
            Segment_Digit[2] = _u;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _c;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _l;
            Segment_Dot[0] = _OFF;
        break;
        case 13:
            Segment_Digit[2] = _o;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _t;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _b;
            Segment_Dot[0] = _OFF;
        break;
        case 14:
            Segment_Digit[2] = _d;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _c;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _b;
            Segment_Dot[0] = _OFF;
        break;
        case 15:
            Segment_Digit[2] = _o;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _t;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _s;
            Segment_Dot[0] = _OFF;
        break;
        case 16:
            Segment_Digit[2] = _d;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _c;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _l;
            Segment_Dot[0] = _OFF;
        break;
        case 17:
            Segment_Digit[2] = _s;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _c;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _o;
            Segment_Dot[0] = _OFF;
        break;
        case 18:
            Segment_Digit[2] = _o;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _u;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _i;
            Segment_Dot[0] = _OFF;
        break;
        case 19:
            Segment_Digit[2] = _u;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _u;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _i;
            Segment_Dot[0] = _OFF;
        break;
        case 20:
            Segment_Digit[2] = _n;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _o;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _b;
            Segment_Dot[0] = _OFF;
        break;
        case 21:
            Segment_Digit[2] = _n;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _o;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _p;
            Segment_Dot[0] = _OFF;
        break;
        case 22:
            Segment_Digit[2] = _n;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _o;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _n;
            Segment_Dot[0] = _OFF;
        break;
        case 23:
            Segment_Digit[2] = _e;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = 2;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _e;
            Segment_Dot[0] = _OFF;
        break;
        case 24:
            Segment_Digit[2] = _h;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _a;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _e;
            Segment_Dot[0] = _OFF;
        break;
        case 25:
            Segment_Digit[2] = _f;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _l;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _o;
            Segment_Dot[0] = _OFF;
        break;
        case 26:
            Segment_Digit[2] = _c;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _h;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _a;
            Segment_Dot[0] = _OFF;
        break;
        case 27:
            Segment_Digit[2] = _l;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _c;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _l;
            Segment_Dot[0] = _OFF;
        break;
        case 28:
            Segment_Digit[2] = _t;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _s;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _t;
            Segment_Dot[0] = _OFF;
        break;
        case 29:
            Segment_Digit[2] = _o;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _u;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _f;
            Segment_Dot[0] = _OFF;
        break;
        case 30:
            Segment_Digit[2] = _u;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _u;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _f;
            Segment_Dot[0] = _OFF;
        break;
        case 31:
            Segment_Digit[2] = _c;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _b;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _f;
            Segment_Dot[0] = _OFF;
        break;
        case 32:
            Segment_Digit[2] = _c;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _l;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _f;
            Segment_Dot[0] = _OFF;
        break;
        case 33:
            Segment_Digit[2] = _s;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = _h;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = _d;
            Segment_Dot[0] = _OFF;
        break;
        // </editor-fold>
    }
}

void  Update_7Segment_Display(void)
{
    unsigned int Temp;
    if(Current_State == _REGISTER_ADDRESS_SHOW)
    {
        // <editor-fold defaultstate="collapsed" desc="Current_State == _REGISTER_ADDRESS_SHOW">
        if(Register_Counter < End_Of_Float)
        {
            Segment_Digit[2] = _r;
            Segment_Dot[2] = _ON;
            Segment_Digit[1] = (unsigned int)Register_Counter / 10;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = (unsigned int)Register_Counter % 10;
            Segment_Dot[0] = _OFF;
        }
        else if(Register_Counter >= End_Of_Float && Register_Counter < End_Of_Word)
        {
            Segment_Digit[2] = _u;
            Segment_Dot[2] = _ON;
            Segment_Digit[1] = (unsigned int)(Register_Counter - End_Of_Float) / 10;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = (unsigned int)(Register_Counter - End_Of_Float) % 10;
            Segment_Dot[0] = _OFF;
        }
        else if(Register_Counter >= End_Of_Word && Register_Counter < End_Of_Byte)
        {
            Segment_Digit[2] = _b;
            Segment_Dot[2] = _ON;
            Segment_Digit[1] = (unsigned int)(Register_Counter - End_Of_Word) / 10;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = (unsigned int)(Register_Counter - End_Of_Word) % 10;
            Segment_Dot[0] = _OFF;
        }
        else if(Register_Counter >= End_Of_Byte)
        {
            Segment_Digit[2] = _f;
            Segment_Dot[2] = _ON;
            Segment_Digit[1] = (unsigned int)(Register_Counter - End_Of_Byte) / 10;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = (unsigned int)(Register_Counter - End_Of_Byte) % 10;
            Segment_Dot[0] = _OFF;
        }
        // </editor-fold>
    }
    else if(Current_State == _FLAG_REGISTER_VALUE_SHOW)
    {
        // <editor-fold defaultstate="collapsed" desc="Current_State == _FLAG_REGISTER_VALUE_SHOW">
        if((Register_Counter - End_Of_Byte) < _END_OF_FUNCTION_FLAG_ADDRESS)
        {
            switch((Register_Counter - End_Of_Byte))
            {
                // <editor-fold defaultstate="collapsed" desc="Control Flag">                
                case 0:
                    if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    {
                        Segment_Digit[2] = _r;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _s;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _t;
                        Segment_Dot[0] = _OFF;
                    }
                    else
                    {
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    }               
                break;
                case 1:
                    if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    {
                        Segment_Digit[2] = _s;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _a;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _u;
                        Segment_Dot[0] = _OFF;
                    }
                    else
                    {
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    }               
                break;
                case 2:
                    if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    {
                        Segment_Digit[2] = _f;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _s;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _t;
                        Segment_Dot[0] = _OFF;
                    }
                    else
                    {
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    }               
                break;
                case 3:
                    if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    {
                        Segment_Digit[2] = _c;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _l;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _r;
                        Segment_Dot[0] = _OFF;
                    }
                    else
                    {
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    }               
                break;
                case 4:
                    if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    {
                        Segment_Digit[2] = _l;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _b;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _c;
                        Segment_Dot[0] = _OFF;
                    }
                    else
                    {
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    }               
                break;
                case 5:
                    if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    {
                        Segment_Digit[2] = _h;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _b;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _c;
                        Segment_Dot[0] = _OFF;
                    }
                    else
                    {
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    }               
                break;
                case 6:
                    if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    {
                        Segment_Digit[2] = _l;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _l;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _c;
                        Segment_Dot[0] = _OFF;
                    }
                    else
                    {
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    }               
                break;
                case 7:
                    if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    {
                        Segment_Digit[2] = _h;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _l;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _c;
                        Segment_Dot[0] = _OFF;
                    }
                    else
                    {
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    }               
                break;
                case 8:
                    if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    {
                        Segment_Digit[2] = _l;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _u;
                        Segment_Dot[0] = _OFF;
                    }
                    else
                    {
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    }               
                break;
                case 9:
                    if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    {
                        Segment_Digit[2] = _h;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _u;
                        Segment_Dot[0] = _OFF;
                    }
                    else
                    {
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    }               
                break;
                case 10:
                    if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    {
                        Segment_Digit[2] = _l;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _i;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _u;
                        Segment_Dot[0] = _OFF;
                    }
                    else
                    {
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    }               
                break;
                case 11:
                    if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    {
                        Segment_Digit[2] = _h;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _i;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _u;
                        Segment_Dot[0] = _OFF;
                    }
                    else
                    {
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    }               
                break;
                case 12:
                    if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    {
                        Segment_Digit[2] = _l;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _b;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _t;
                        Segment_Dot[0] = _OFF;
                    }
                    else
                    {
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    }               
                break;
                case 13:
                    if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    {
                        Segment_Digit[2] = _h;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _b;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _t;
                        Segment_Dot[0] = _OFF;
                    }
                    else
                    {
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    }               
                break;
                case 14:
                    if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    {
                        Segment_Digit[2] = _l;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _h;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _t;
                        Segment_Dot[0] = _OFF;
                    }
                    else
                    {
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    }               
                break;
                case 15:
                    if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    {
                        Segment_Digit[2] = _h;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _h;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _t;
                        Segment_Dot[0] = _OFF;
                    }
                    else
                    {
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    }               
                break;
                case 16:
                    if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    {
                        Segment_Digit[2] = _f;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _i;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _t;
                        Segment_Dot[0] = _OFF;
                    }
                    else
                    {
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    }               
                break;
                case 17:
                        Segment_Digit[2] = _n;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _u;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _l;
                        Segment_Dot[0] = _OFF;
                    
                                  
                break;

                // </editor-fold>    
            }
        }
        else if((Register_Counter - End_Of_Byte) >= _END_OF_FUNCTION_FLAG_ADDRESS && (Register_Counter - End_Of_Byte) < _END_OF_EVENT_FLAG_ADDRESS)
        {
            if(Last_Event_Number == 0)
                Temp = 9;
            else
                Temp = Last_Event_Number - 1;
            
            if((Register_Counter - End_Of_Byte - _END_OF_FUNCTION_FLAG_ADDRESS) > Temp)
                Temp = 10 - (Register_Counter - End_Of_Byte - _END_OF_FUNCTION_FLAG_ADDRESS) + Temp;
            else
                Temp = Temp - (Register_Counter - End_Of_Byte - _END_OF_FUNCTION_FLAG_ADDRESS);
            if(Que_Alarm[Temp] == 0)
            {
                Segment_Digit[2] = _n;
                Segment_Dot[2] = _OFF;
                Segment_Digit[1] = _a;
                Segment_Dot[1] = _OFF;
                Segment_Digit[0] = _l;
                Segment_Dot[0] = _OFF;
            }
            else
            {                
                switch(Que_Alarm[Temp])
                {
                    // <editor-fold defaultstate="collapsed" desc="Alarm">
                    case 0:
                        Segment_Digit[2] = _n;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _a;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _l;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 1:
                        Segment_Digit[2] = _l;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _e;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _f;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 2:
                        Segment_Digit[2] = _r;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _i;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _t;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 3:
                        Segment_Digit[2] = _l;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = 1;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _l;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 4:
                        Segment_Digit[2] = _l;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = 2;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _l;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 5:
                        Segment_Digit[2] = _l;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = 3;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _l;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 6:
                        Segment_Digit[2] = _f;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _d;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _e;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 7:
                        Segment_Digit[2] = _o;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _u;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _o;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 8:
                        Segment_Digit[2] = _u;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _u;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _o;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 9:
                        Segment_Digit[2] = _o;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _c;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _b;
                        Segment_Dot[0] = _OFF;
                    break; 
                    case 10:
                        Segment_Digit[2] = _u;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _c;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _b;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 11:
                        Segment_Digit[2] = _o;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _c;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _l;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 12:
                        Segment_Digit[2] = _u;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _c;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _l;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 13:
                        Segment_Digit[2] = _o;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _t;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _b;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 14:
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _c;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _b;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 15:
                        Segment_Digit[2] = _o;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _t;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _s;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 16:
                        Segment_Digit[2] = _d;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _c;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _l;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 17:
                        Segment_Digit[2] = _s;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _c;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _o;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 18:
                        Segment_Digit[2] = _o;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _u;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _i;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 19:
                        Segment_Digit[2] = _u;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _u;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _i;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 20:
                        Segment_Digit[2] = _n;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _b;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 21:
                        Segment_Digit[2] = _n;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _p;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 22:
                        Segment_Digit[2] = _n;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _o;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _n;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 23:
                        Segment_Digit[2] = _e;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = 2;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _e;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 24:
                        Segment_Digit[2] = _h;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _a;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _e;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 25:
                        Segment_Digit[2] = _f;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _l;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _o;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 26:
                        Segment_Digit[2] = _c;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _h;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _a;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 27:
                        Segment_Digit[2] = _l;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _c;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _l;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 28:
                        Segment_Digit[2] = _t;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _s;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _t;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 29:
                        Segment_Digit[2] = _o;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _u;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _f;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 30:
                        Segment_Digit[2] = _u;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _u;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _f;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 31:
                        Segment_Digit[2] = _c;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _b;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _f;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 32:
                        Segment_Digit[2] = _c;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _l;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _f;
                        Segment_Dot[0] = _OFF;
                    break;
                    case 33:
                        Segment_Digit[2] = _s;
                        Segment_Dot[2] = _OFF;
                        Segment_Digit[1] = _h;
                        Segment_Dot[1] = _OFF;
                        Segment_Digit[0] = _d;
                        Segment_Dot[0] = _OFF;
                    break;
                    // </editor-fold>
                }
            }  
        }
        else if((Register_Counter - End_Of_Byte) >= _END_OF_EVENT_FLAG_ADDRESS && (Register_Counter - End_Of_Byte) < _END_OF_PROPERTY_FLAG_ADDRESS)
        {
            Segment_Digit[2] = (unsigned int)Flag_Type[Register_Counter - End_Of_Byte] / 100;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = ((unsigned int)Flag_Type[Register_Counter - End_Of_Byte] % 100) / 10;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = (unsigned int)Flag_Type[Register_Counter - End_Of_Byte] % 10;
            Segment_Dot[0] = _OFF;
        }
        else
        {
            if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
            {
                Segment_Digit[2] = _f;
                Segment_Dot[2] = _OFF;
                Segment_Digit[1] = _l;
                Segment_Dot[1] = _OFF;
                Segment_Digit[0] = _s;
                Segment_Dot[0] = _OFF;
            }
            else
            {
                Segment_Digit[2] = _t;
                Segment_Dot[2] = _OFF;
                Segment_Digit[1] = _r;
                Segment_Dot[1] = _OFF;
                Segment_Digit[0] = _u;
                Segment_Dot[0] = _OFF;
            }
        }
        // </editor-fold>        
    }    
    else if(Current_State == _BYTE_REGISTER_VALUE_SHOW)
    {
        // <editor-fold defaultstate="collapsed" desc="Current_State == _BYTE_REGISTER_VALUE_SHOW">
        Segment_Digit[2] = (unsigned int)Byte_Type[Register_Counter - End_Of_Word] / 100;
        Segment_Dot[2] = _OFF;
        Segment_Digit[1] = ((unsigned int)Byte_Type[Register_Counter - End_Of_Word] % 100) / 10;
        Segment_Dot[1] = _OFF;
        Segment_Digit[0] = (unsigned int)Byte_Type[Register_Counter - End_Of_Word] % 10;
        Segment_Dot[0] = _OFF;
        // </editor-fold>
    }   
    else if(Current_State == _REAL_REGISTER_VALUE_SHOW)
    {       
        // <editor-fold defaultstate="collapsed" desc="Current_State == _REAL_REGISTER_VALUE_SHOW">        
        Real_Type[Register_Counter] = (r[Register_Counter] * 10) / 10;
        Segment_Digit[2] = (unsigned int)Real_Type[Register_Counter] / 100;
        Segment_Dot[2] = _OFF;
        Segment_Digit[1] = ((unsigned int)Real_Type[Register_Counter] % 100) / 10;
        Segment_Dot[1] = _OFF;
        Segment_Digit[0] = (unsigned int)Real_Type[Register_Counter] % 10;
        Segment_Dot[0] = _OFF;
        // </editor-fold>
    }        
    else if(Current_State == _FLOAT_REGISTER_VALUE_SHOW)
    {
        // <editor-fold defaultstate="collapsed" desc="Current_State == _FLOAT_REGISTER_VALUE_SHOW">        
        Float_Type[Register_Counter] = (int)(r[Register_Counter] * 10) % 10;
        Segment_Digit[2] = __OFF;
        Segment_Dot[2] = _OFF;
        Segment_Digit[1] = __OFF;
        Segment_Dot[1] = _ON;
        Segment_Digit[0] = (unsigned int)Float_Type[Register_Counter];
        Segment_Dot[0] = _OFF;
        // </editor-fold>
    }
    else if(Current_State == _WORD_H_REGISTER_VALUE_SHOW)
    {
        // <editor-fold defaultstate="collapsed" desc="WORD_H_REGISTER_VALUE_SHOW">
        if(Word_Type[Register_Counter - End_Of_Float] < 1000)
        {
            Segment_Digit[2] = (unsigned int)Word_Type[Register_Counter - End_Of_Float] / 100;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = ((unsigned int)Word_Type[Register_Counter - End_Of_Float] % 100) / 10;
            if((Register_Counter - End_Of_Float) >= _U_VO_PV && (Register_Counter - End_Of_Float) <= _U_FRQ_PV)
                Segment_Dot[1] = _ON;
            else
                Segment_Dot[1] = _OFF;
            Segment_Digit[0] = (unsigned int)Word_Type[Register_Counter - End_Of_Float] % 10;
            Segment_Dot[0] = _OFF;
        }
        else if(Word_Type[Register_Counter - End_Of_Float] < 10000)
        {
            Segment_Digit[2] = (unsigned int)Word_Type[Register_Counter - End_Of_Float] / 1000;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = ((unsigned int)Word_Type[Register_Counter - End_Of_Float] % 1000) / 100;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = ((unsigned int)Word_Type[Register_Counter - End_Of_Float] % 100) / 10;
            Segment_Dot[0] = _OFF;
        }
        else
        {
            Segment_Digit[2] = (unsigned int)Word_Type[Register_Counter - End_Of_Float] / 10000;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = ((unsigned int)Word_Type[Register_Counter - End_Of_Float] % 10000) / 1000;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = ((unsigned int)Word_Type[Register_Counter - End_Of_Float] % 1000) / 100;
            Segment_Dot[0] = _OFF;
        }
        // </editor-fold>
    }
    else if(Current_State == _WORD_L_REGISTER_VALUE_SHOW)
    {
        // <editor-fold defaultstate="collapsed" desc="WORD_L_REGISTER_VALUE_SHOW">
        if(Word_Type[Register_Counter - End_Of_Float] < 1000)
        {
            Segment_Digit[2] = __OFF;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = __OFF;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = __OFF;
            Segment_Dot[0] = _OFF;
        }
        else if(Word_Type[Register_Counter - End_Of_Float] < 10000)
        {
            Segment_Digit[2] = __OFF;
            Segment_Dot[2] = _OFF;            
            Segment_Digit[1] = __OFF;
            if((Register_Counter - End_Of_Float) == _U_IO_PV)
                Segment_Dot[1] = _ON;
            else
                Segment_Dot[1] = _OFF;
            Segment_Digit[0] = ((unsigned int)Word_Type[Register_Counter - End_Of_Float] % 10);
            Segment_Dot[0] = _OFF;
        }
        else
        {
            Segment_Digit[2] = __OFF;
            Segment_Dot[2] = _OFF;
            Segment_Digit[1] = ((unsigned int)Word_Type[Register_Counter - End_Of_Float] % 100) / 10;
            Segment_Dot[1] = _OFF;
            Segment_Digit[0] = ((unsigned int)Word_Type[Register_Counter - End_Of_Float] % 10);
            Segment_Dot[0] = _OFF;
        }
        // </editor-fold>
    }
    
}

void Set_Segment(unsigned char value, unsigned char Number_of_Seg, unsigned char DOT_status)
{
    switch(Number_of_Seg)
    {
        case 0:
            DIGIT1 = 0;
            DIGIT2 = 0;
            DIGIT3 = 1;
        break;
        case 1:
            DIGIT2 = 1;
            DIGIT1 = 0;
            DIGIT3 = 0;
        break;
        case 2:
            DIGIT3 = 0;
            DIGIT1 = 1;
            DIGIT2 = 0;
        break;
            
    }
    switch(value)
    {
        case 0:
        PORTF = 0x77;
        break;
        case 1:
        PORTF = 0x12;
        break;
        case 2:
        PORTF = 0x6B;
        break;
        case 3:
        PORTF = 0x3B;
        break;
        case 4:
        PORTF = 0x1E;
        break;
        case 5:
        PORTF = 0x3D;
        break;
        case 6:
        PORTF = 0x7D;
        break;
        case 7:
        PORTF = 0x13;
        break;
        case 8:
        PORTF = 0x7F;
        break;
        case 9:
        PORTF = 0x3F;
        break;
        case _a:
        PORTF = 0x5F;
        break;
        case _b:
        PORTF = 0x7C;
        break;
        case _c:
        PORTF = 0x65;
        break;
        case _d:
        PORTF = 0x7A;
        break;
        case _e:
        PORTF = 0x6F;
        break;
        case _f:
        PORTF = 0x4D;
        break;
        case _h:
        PORTF = 0x5E;
        break;
        case _l:
        PORTF = 0x64;
        break;
        case _i:
        PORTF = 0x12;
        break;
        case _n:
        PORTF = 0x57;
        break;
        case _o:
        PORTF = 0x77;
        break;
        case _p:
        PORTF = 0x4F;
        break;
        case _r:
        PORTF = 0x45;
        break;
        case _s:
        PORTF = 0x3D;
        break;
        case _t:
        PORTF = 0x6C;
        break;
        case _u:
        PORTF = 0x76;
        break;
        case __OFF:
        PORTF = 0x00;
        break;
    }
    if(DOT_status == _ON)
        LATBbits.LATB12 = 1;
    else
        LATBbits.LATB12 = 0;
}

void Perform_Command(void)
{   
    unsigned int i = 1;
    if(Current_State == _REGISTER_ADDRESS_SHOW)
    {
        // <editor-fold defaultstate="collapsed" desc="Current_State == _REGISTER_ADDRESS_SHOW">
        if(Pressed_Key == _ENTER)
        {
            if(1/*Previous_Detected_Key != _ENTER*/)
            {
                if(Register_Counter < End_Of_Float)
                    Current_State = _REAL_REGISTER_VALUE_SHOW;
                else if(Register_Counter >= End_Of_Float && Register_Counter < End_Of_Word)
                    Current_State = _WORD_H_REGISTER_VALUE_SHOW;
                else if(Register_Counter >= End_Of_Word && Register_Counter < End_Of_Byte)
                    Current_State = _BYTE_REGISTER_VALUE_SHOW;
                else if(Register_Counter >= End_Of_Byte)
                    Current_State = _FLAG_REGISTER_VALUE_SHOW;
            }
        }
        else if(Pressed_Key == _UP)
        {
            if(Previous_Detected_Key == _UP)
            {
                Debounce_Limit = _Debounce_Limit_1;
                TTCounter_Limit = 100;
            }
            
                Register_Counter ++;
            if(Register_Counter >= Register_Counter_Limit)
                Register_Counter = 0;                
        }
        else if(Pressed_Key == _DOWN)
        {
            if(Previous_Detected_Key == _DOWN)
            {
                Debounce_Limit = _Debounce_Limit_1;
                TTCounter_Limit = 100;
            }
            if(Register_Counter == 0)
                Register_Counter = Register_Counter_Limit - 1;
            else
                Register_Counter --;
        }
        // </editor-fold>
    }   
    else if(Current_State == _FLAG_REGISTER_VALUE_SHOW)
    {
        // <editor-fold defaultstate="collapsed" desc="Current_State == _FLAG_REGISTER_VALUE_SHOW">
        if(Pressed_Key == _ENTER)
        {
            Current_State = _REGISTER_ADDRESS_SHOW;
            if((Register_Counter - End_Of_Byte) < _END_OF_FUNCTION_FLAG_ADDRESS)
            {
                Run_Function_Start_Flag = 0;
                Run_Function_Start_Counter = 0;
                Flag_Type[Register_Counter - End_Of_Byte] = 0;
            }
        }
        else if(Pressed_Key == _UP)
        {           
            if((Register_Counter - End_Of_Byte) < _END_OF_FUNCTION_FLAG_ADDRESS)
            {
                Flag_Type[Register_Counter - End_Of_Byte] = 1;
                Run_Function_Start_Flag = 1;
                Run_Function(Register_Counter - End_Of_Byte);                
            }
            else if((Register_Counter - End_Of_Byte) >= _END_OF_EVENT_FLAG_ADDRESS && (Register_Counter - End_Of_Byte) < _END_OF_PROPERTY_FLAG_ADDRESS)
            {
                Flag_Type_Change_Flag[Register_Counter - End_Of_Byte - _END_OF_EVENT_FLAG_ADDRESS] = 1;
                Flag_Type[Register_Counter - End_Of_Byte] ++;
                if(Flag_Type[Register_Counter - End_Of_Byte] == 4)
                    Flag_Type[Register_Counter - End_Of_Byte] = 0;
                f_property[Register_Counter - End_Of_Byte - _END_OF_EVENT_FLAG_ADDRESS] = Flag_Type[Register_Counter - End_Of_Byte];
            }
            else if((Register_Counter - End_Of_Byte) >= _END_OF_PROPERTY_FLAG_ADDRESS && (Register_Counter - End_Of_Byte) < _END_OF_RW_FLAG_ADDRESS)
            {
                Flag_Type_Change_Flag[Register_Counter - End_Of_Byte - _END_OF_EVENT_FLAG_ADDRESS] = 1;
                Flag_Type[Register_Counter - End_Of_Byte] = 1 - Flag_Type[Register_Counter - End_Of_Byte];
            }
            
        }
        else if(Pressed_Key == _DOWN)
        {
            if((Register_Counter - End_Of_Byte) >= _END_OF_EVENT_FLAG_ADDRESS && (Register_Counter - End_Of_Byte) < _END_OF_PROPERTY_FLAG_ADDRESS)
            {                
                Flag_Type_Change_Flag[Register_Counter - End_Of_Byte - _END_OF_EVENT_FLAG_ADDRESS] = 1;
                if(Flag_Type[Register_Counter - End_Of_Byte] == 0)
                    Flag_Type[Register_Counter - End_Of_Byte] = 3;
                else
                    Flag_Type[Register_Counter - End_Of_Byte] --;
                f_property[Register_Counter - End_Of_Byte - _END_OF_EVENT_FLAG_ADDRESS] = Flag_Type[Register_Counter - End_Of_Byte];
            }
            else if((Register_Counter - End_Of_Byte) >= _END_OF_PROPERTY_FLAG_ADDRESS && (Register_Counter - End_Of_Byte) < _END_OF_RW_FLAG_ADDRESS)
            {
                Flag_Type_Change_Flag[Register_Counter - End_Of_Byte - _END_OF_EVENT_FLAG_ADDRESS] = 1;
                Flag_Type[Register_Counter - End_Of_Byte] = 1 - Flag_Type[Register_Counter - End_Of_Byte];
            }
        }
        // </editor-fold>
    }    
    else if(Current_State == _BYTE_REGISTER_VALUE_SHOW)
    {
        // <editor-fold defaultstate="collapsed" desc="Current_State == _BYTE_REGISTER_VALUE_SHOW">
        if(Pressed_Key == _ENTER)
        {
            Current_State = _REGISTER_ADDRESS_SHOW;
        }
        else if(Pressed_Key == _UP)
        {
            
           Byte_Type_Change_Flag[Register_Counter - End_Of_Word] = 1;
            if(Previous_Detected_Key == _UP)
            {
                Debounce_Limit = _Debounce_Limit_1;
                TTCounter_Limit = 100;
            }
            if(Byte_Type[Register_Counter - End_Of_Word] == 255)
                Byte_Type[Register_Counter - End_Of_Word] = 0;
            else
                Byte_Type[Register_Counter - End_Of_Word] ++;                           
        }
        else if(Pressed_Key == _DOWN)
        {
            Byte_Type_Change_Flag[Register_Counter - End_Of_Word] = 1;
            if(Previous_Detected_Key == _DOWN)
            {
                Debounce_Limit = _Debounce_Limit_1;
                TTCounter_Limit = 100;
            }
            if(Byte_Type[Register_Counter - End_Of_Word] == 0)
                Byte_Type[Register_Counter - End_Of_Word] = 255;
            else
                Byte_Type[Register_Counter - End_Of_Word] --;
        }
        // </editor-fold>
    }   
    else if(Current_State == _REAL_REGISTER_VALUE_SHOW)
    {
        // <editor-fold defaultstate="collapsed" desc="Current_State == _REAL_REGISTER_VALUE_SHOW">
        if(Pressed_Key == _ENTER)
        {
            Current_State = _FLOAT_REGISTER_VALUE_SHOW;
        }
        else if(Pressed_Key == _UP)
        {
            if(Register_Counter < _END_OF_RW_FLOAT_ADDRESS)
            {
                Real_Type_Change_Flag[Register_Counter] = 1;
                if(Previous_Detected_Key == _UP)
                {
                    Debounce_Limit = _Debounce_Limit_1;
                    TTCounter_Limit = 100;
                }
//                if(r[Register_Counter] >= 999)
//                    r[Register_Counter] = 0;
//                else
//                    r[Register_Counter] = r[Register_Counter] + 1;
                if(Real_Type[Register_Counter] == 999)
                    Real_Type[Register_Counter] = 999;
                else
                    Real_Type[Register_Counter] ++; 
                r[Register_Counter] = (float)(Real_Type[Register_Counter] * 10 + Float_Type[Register_Counter]) / 10.0;
                
            }
        }
        else if(Pressed_Key == _DOWN)
        {
            if(Register_Counter < _END_OF_RW_FLOAT_ADDRESS)
            {
                Real_Type_Change_Flag[Register_Counter] = 1;
                if(Previous_Detected_Key == _DOWN)
                {
                    Debounce_Limit = _Debounce_Limit_1;
                    TTCounter_Limit = 100;
                }
//                if(r[Register_Counter] == 0)
//                    r[Register_Counter] = 999;
//                else
//                    r[Register_Counter] = r[Register_Counter] - 1;
                if(Real_Type[Register_Counter] == 0)
                    Real_Type[Register_Counter] = 0;
                else
                    Real_Type[Register_Counter] --; 
                r[Register_Counter] = (float)(Real_Type[Register_Counter] * 10 + Float_Type[Register_Counter]) / 10.0;
                
            }
        }
        // </editor-fold>
    }        
    else if(Current_State == _FLOAT_REGISTER_VALUE_SHOW)
    {
        // <editor-fold defaultstate="collapsed" desc="Current_State == _FLOAT_REGISTER_VALUE_SHOW">
        if(Pressed_Key == _ENTER)
        {
            Current_State = _REGISTER_ADDRESS_SHOW;
        }
        else if(Pressed_Key == _UP)
        {
            if(Register_Counter < _END_OF_RW_FLOAT_ADDRESS)
            {
                Float_Type_Change_Flag[Register_Counter] = 1;
                if(Previous_Detected_Key == _UP)
                {
                    Debounce_Limit = _Debounce_Limit_1;
                    TTCounter_Limit = 100;
                }
//                if(r[Register_Counter] >= 999)
//                    r[Register_Counter] = 0;
//                else
//                    r[Register_Counter] = r[Register_Counter] + 0.1;;
                if(Float_Type[Register_Counter] == 9)
                    Float_Type[Register_Counter] = 9;
                else
                    Float_Type[Register_Counter] ++; 
                r[Register_Counter] = (float)((float)Real_Type[Register_Counter] * 10 + Float_Type[Register_Counter]) / 10.0;
                
            }
        }
        else if(Pressed_Key == _DOWN)
        {
            if(Register_Counter < _END_OF_RW_FLOAT_ADDRESS)
            {
                Float_Type_Change_Flag[Register_Counter] = 1;
                if(Previous_Detected_Key == _DOWN)
                {
                    Debounce_Limit = _Debounce_Limit_1;
                    TTCounter_Limit = 100;
                }
//                if(r[Register_Counter] == 0)
//                    r[Register_Counter] = 999;
//                else
//                    r[Register_Counter] = r[Register_Counter] - 0.1;
                if(Float_Type[Register_Counter] == 0)
                    Float_Type[Register_Counter] = 0;
                else
                    Float_Type[Register_Counter] --; 
                r[Register_Counter] = (float)((float)Real_Type[Register_Counter] * 10 + Float_Type[Register_Counter]) / 10.0;
                
            }
        }
        // </editor-fold>
    }
    else if(Current_State == _WORD_H_REGISTER_VALUE_SHOW)
    {
        // <editor-fold defaultstate="collapsed" desc="WORD_H_REGISTER_VALUE_SHOW">
        if(Pressed_Key == _ENTER)
        {
            if((Register_Counter - End_Of_Float) >= _U_VO_PV && (Register_Counter - End_Of_Float) <= _U_FRQ_PV)
            {
                if((Register_Counter - End_Of_Float) == _U_IO_PV && Word_Type[Register_Counter - End_Of_Float] > 999)
                    Current_State = _WORD_L_REGISTER_VALUE_SHOW;
                else
                    Current_State = _REGISTER_ADDRESS_SHOW;
            }
            else
                Current_State = _WORD_L_REGISTER_VALUE_SHOW;
        }
        else if(Pressed_Key == _UP)
        {
            if((Register_Counter - End_Of_Float) < _END_OF_RW_WORD_ADDRESS)
            {
                Word_Type_Change_Flag[Register_Counter - End_Of_Float] = 1;
                if(Previous_Detected_Key == _UP)
                {
                    Debounce_Limit = _Debounce_Limit_1;
                    TTCounter_Limit = 100;
                    i = 10;
                }
                if(Word_Type[Register_Counter - End_Of_Float] < 1000)
                {
                    if(Word_Type[Register_Counter - End_Of_Float] <= 65535 - (1*i))
                        Word_Type[Register_Counter - End_Of_Float] = Word_Type[Register_Counter - End_Of_Float] + 1*i;
                    else
                        Word_Type[Register_Counter - End_Of_Float] = 65535;
                }
                else if(Word_Type[Register_Counter - End_Of_Float] < 10000)
                {
                    if(Word_Type[Register_Counter - End_Of_Float] <= 65535 - (10*i))
                        Word_Type[Register_Counter - End_Of_Float] = Word_Type[Register_Counter - End_Of_Float] + 10*i;
                    else
                        Word_Type[Register_Counter - End_Of_Float] = 65535;
                }
                else
                {
                    if(Word_Type[Register_Counter - End_Of_Float] <= 65535 - (100*i))
                        Word_Type[Register_Counter - End_Of_Float] = Word_Type[Register_Counter - End_Of_Float] + 100*i;
                    else
                        Word_Type[Register_Counter - End_Of_Float] = 65535;
                }

            }
        }
        else if(Pressed_Key == _DOWN)
        {
            if((Register_Counter - End_Of_Float) < _END_OF_RW_WORD_ADDRESS)
            {
                Word_Type_Change_Flag[Register_Counter - End_Of_Float] = 1;
                if(Previous_Detected_Key == _DOWN)
                {
                    Debounce_Limit = _Debounce_Limit_1;  
                    TTCounter_Limit = 100;
                    i = 10;
                }
                if(Word_Type[Register_Counter - End_Of_Float] < 1000)
                {
                    if(Word_Type[Register_Counter - End_Of_Float] >= (1*i))
                        Word_Type[Register_Counter - End_Of_Float] = Word_Type[Register_Counter - End_Of_Float] - 1*i;
                    else
                        Word_Type[Register_Counter - End_Of_Float] = 0;
                }
                else if(Word_Type[Register_Counter - End_Of_Float] < 10000)
                {
                    if(Word_Type[Register_Counter - End_Of_Float] >= (10*i))
                        Word_Type[Register_Counter - End_Of_Float] = Word_Type[Register_Counter - End_Of_Float] - 10*i;
                    else
                        Word_Type[Register_Counter - End_Of_Float] = 0;
                }
                else
                {
                    if(Word_Type[Register_Counter - End_Of_Float] >= (100*i))
                        Word_Type[Register_Counter - End_Of_Float] = Word_Type[Register_Counter - End_Of_Float] - 100*i;
                    else
                        Word_Type[Register_Counter - End_Of_Float] = 0;
                }

            }
        }
        // </editor-fold>
    }
    else if(Current_State == _WORD_L_REGISTER_VALUE_SHOW)
    {
        // <editor-fold defaultstate="collapsed" desc="WORD_L_REGISTER_VALUE_SHOW">
        if(Pressed_Key == _ENTER)
        {
            Current_State = _REGISTER_ADDRESS_SHOW;
        }
        else if(Pressed_Key == _UP)
        {
            if((Register_Counter - End_Of_Float) < _END_OF_RW_WORD_ADDRESS)
            {
                Word_Type_Change_Flag[Register_Counter - End_Of_Float] = 1;
                if(Previous_Detected_Key == _UP)
                {
                    Debounce_Limit = _Debounce_Limit_1;
                    TTCounter_Limit = 100;
                }
                Word_Type[Register_Counter - End_Of_Float] ++;
                if(Word_Type[Register_Counter - End_Of_Float] > 65534)
                    Word_Type[Register_Counter - End_Of_Float] = 65534;
            }
            
        }
        else if(Pressed_Key == _DOWN)
        {
            if((Register_Counter - End_Of_Float) < _END_OF_RW_WORD_ADDRESS)
            {
                Word_Type_Change_Flag[Register_Counter - End_Of_Float] = 1;
                if(Previous_Detected_Key == _DOWN)
                {
                    Debounce_Limit = _Debounce_Limit_1;
                    TTCounter_Limit = 100;
                }
                if(Word_Type[Register_Counter - End_Of_Float] > 0)
                    Word_Type[Register_Counter - End_Of_Float] --;
            }
        }
        // </editor-fold>
    }
    
    Set_Limits_Value();
   
}

void Run_Function(unsigned char index_function)
{
    unsigned int ip,i = 0;
    unsigned int j = 0;
    unsigned int W,WP,WL,WH,T,TL,TH,TP;
    float D, f_ip, f_i;
    switch(index_function)
    {
        case _rst:             
            asm ("RESET"); 
        break;
        case _sAu:            
            _Wdt_Status = _OFF;
            ee_set_DAM(Byte_Type[0]); 
            for(i = 0; i < 10; i++)
            {
                //Que_Alarm[i] = 0;
                ee_set_Que_Alarm_ADRESS(Que_Alarm[i],i);
            }
            for(i = 0; i < _END_OF_RW_BYTE_ADDRESS; i++)
            {
                if(Byte_Type_Change_Flag[i] == 1)
                {
                    ee_set_Byte_Type_ADRESS(Byte_Type[i], i);
                    Byte_Type_Change_Flag[i] = 0;
                }
            }
            for(i = 0; i < _END_OF_RW_WORD_ADDRESS; i++)
            {
                if(Word_Type_Change_Flag[i] == 1)
                {
                    ee_set_Word_Type_ADRESS(Word_Type[i], i);
                    Word_Type_Change_Flag[i] = 0;
                }
            }
            for(i = 0; i < _END_OF_RW_FLOAT_ADDRESS; i++)
            {
                if(Real_Type_Change_Flag[i] == 1)
                {
                    ee_set_Real_Type_ADRESS(Real_Type[i], i);
                    Real_Type_Change_Flag[i] = 0;
                }
            }
            for(i = 0; i < _END_OF_RW_FLOAT_ADDRESS; i++)
            {
                if(Float_Type_Change_Flag[i] == 1)
                {
                    ee_set_Float_Type_ADRESS(Float_Type[i], i);
                    Float_Type_Change_Flag[i] = 0;
                }
            }
            for(i = 0; i < _END_OF_RW_FLAG_ADDRESS2; i++)
            {
                if(Flag_Type_Change_Flag[i] == 1)
                {
                    ee_set_Flag_Type_ADRESS(Flag_Type[i+_END_OF_EVENT_FLAG_ADDRESS], i);
                    Flag_Type_Change_Flag[i] = 0;
                }
            }
            _Wdt_Status = _ON;
        break;
        case _Fst:
            if(Flag_Type[_F_SAVE_FACTORY_RESET] == 0)
                Factory_Reset();
            else
                Save_Factory_Reset();
        break;
        case _clr:            
            _Wdt_Status = _OFF;
            Last_Event_Number = 0;
            ee_set_POINTER_OF_LAST_EVENT(Last_Event_Number);
            for(i = 0; i < 10; i++)
            {
                Que_Alarm[i] = 0;
                ee_set_Que_Alarm_ADRESS(Que_Alarm[i],i);
            }
            _Wdt_Status = _ON;
        break;
        case _Lbc:
            LBC_WORD = Word_Type[_r_IB];
            ee_set_LbC_ADRESS_WORD(LBC_WORD);
        break;
        case _Hbc:
            HBC_WORD = Word_Type[_r_IB];
            HBC_REAL = Byte_Type[_b_DATA_REAL];
            ee_set_HbC_ADRESS_WORD(HBC_WORD);
            ee_set_HbC_ADRESS_REAL(HBC_REAL);
        break;
        case _LLc:
            LLC_WORD = Word_Type[_r_IO];
            ee_set_LLC_ADRESS_WORD(LLC_WORD);
        break;
        case _HLc:
            HLC_WORD = Word_Type[_r_IO];
            HLC_REAL = Byte_Type[_b_DATA_REAL];
            ee_set_HLC_ADRESS_WORD(HLC_WORD);
            ee_set_HLC_ADRESS_REAL(HLC_REAL);
        break;
        case _Lou:
            LOU_WORD = Word_Type[_r_VO];
            ee_set_LOU_ADRESS_WORD(LOU_WORD);
        break;
        case _Hou:
            HOU_WORD = Word_Type[_r_VO];
            HOU_REAL = Byte_Type[_b_DATA_REAL];
            ee_set_HOU_ADRESS_WORD(HOU_WORD);
            ee_set_HOU_ADRESS_REAL(HOU_REAL);//???????
        break;
        case _LIU:
            LIU_WORD = Word_Type[_r_VIN_F];
            ee_set_LIU_ADRESS_WORD(LIU_WORD);
        break;
        case _HIU:
            HIU_WORD = Word_Type[_r_VIN_F];
            HIU_REAL = Byte_Type[_b_DATA_REAL];
            ee_set_HIU_ADRESS_WORD(HIU_WORD);
            ee_set_HIU_ADRESS_REAL(HIU_REAL);
        break;
        case _Lbt:
            Lbt_WORD = Word_Type[_AI_BAT_TEMP];
            Lbt_REAL = Byte_Type[_b_DATA_REAL];
            ee_set_Lbt_ADRESS_WORD(Lbt_WORD);
            ee_set_Lbt_ADRESS_REAL(Lbt_REAL);
        break;
        case _Hbt:
            Hbt_WORD = Word_Type[_AI_BAT_TEMP];
            Hbt_REAL = Byte_Type[_b_DATA_REAL];
            ee_set_Hbt_ADRESS_WORD(Hbt_WORD);
            ee_set_Hbt_ADRESS_REAL(Hbt_REAL);
        break;
        case _LHt:
            LHt_WORD = Word_Type[_AI_HSINK_TEMP];
            LHt_REAL = Byte_Type[_b_DATA_REAL];
            ee_set_LHt_ADRESS_WORD(LHt_WORD);
            ee_set_LHt_ADRESS_REAL(LHt_REAL);
            
            HMt_WORD = Word_Type[_AI_HSINK_TEMP];
            HMt_REAL = Byte_Type[_b_DATA_REAL];
            ee_set_HMt_ADRESS_WORD(HMt_WORD);
            ee_set_HMt_ADRESS_REAL(HMt_REAL);
            
            LUT[0] = Byte_Type[_b_DATA_REAL];
            ee_set_LUT_ADRESS(LUT[0], 0);
        break;
        case _HHt:
            HHt_WORD = Word_Type[_AI_HSINK_TEMP];
            HHt_REAL = Byte_Type[_b_DATA_REAL];
            ee_set_HHt_ADRESS_WORD(HHt_WORD);
            ee_set_HHt_ADRESS_REAL(HHt_REAL);
            
            HMt_WORD = Word_Type[_AI_HSINK_TEMP];
            HMt_REAL = Byte_Type[_b_DATA_REAL];
            ee_set_HMt_ADRESS_WORD(HMt_WORD);
            ee_set_HMt_ADRESS_REAL(HMt_REAL);
            
            LUT[255] = Byte_Type[_b_DATA_REAL];
            ee_set_LUT_ADRESS(LUT[255], 255);
        break;
        case _FIt:
            _Wdt_Status = _OFF;
            W = Word_Type[_AI_HSINK_TEMP];
            T = Byte_Type[_b_DATA_REAL];
            WL = LHt_WORD;
            TL = LHt_REAL;
            WH = HHt_WORD;
            TH = HHt_REAL;
            TP = HMt_REAL;
            WP = HMt_WORD;
            if((W > WP && T > TP) || (W < WP && T < TP))
            {
                Flag_Type[Register_Counter - End_Of_Byte] = 0;
                Run_Function_Start_Flag = 0;
                F_FAIL_DATA_ENTRY = 1;
            }
            else
            {
                f_i = (float)((float)WL - W) * 255.00 / (float)((float)WL - WH);
                i = (unsigned int)(f_i + 0.5);
                LUT[i] = T;
                ee_set_LUT_ADRESS(LUT[i], i);
                f_ip = (float)((float)WL - WP) * 255.00 / (float)((float)WL - WH);
                ip = (unsigned int)(f_ip + 0.5);
                //D = (float)(T - TP) / (float)(f_i - f_ip);
                D = (float)((float)T - TP) / (float)((float)i - ip);
                //Word_Type[_U_TEST1] = i;
                //Word_Type[_U_TEST2] = ip;
                for(j = i; j > (ip); j--)
                {
                    LUT[j] = (unsigned char)((float)((float)T + ((float)((float)j - i) * D)) /** 10.00*/);
                    ee_set_LUT_ADRESS(LUT[j], j);                   
                }
                LUT[ip] = (unsigned char)((float)((float)T + ((float)((float)ip - i) * D)) /** 10.00*/);
                ee_set_LUT_ADRESS(LUT[ip], ip);
                    
                HMt_REAL = T;
                HMt_WORD = W;                        
                ee_set_HMt_ADRESS_WORD(HMt_WORD);
                ee_set_HMt_ADRESS_REAL(HMt_REAL);
                F_FAIL_DATA_ENTRY = 0;
                Temporary_float_Register = 255;
                NTC_G = Temporary_float_Register / (float)((float)HHt_WORD - LHt_WORD);
            }
            _Wdt_Status = _ON;
        break;
    }
}

void Restore(void)
{    
    int i;
    
    Last_Event_Number = ee_get_POINTER_OF_LAST_EVENT();
    
    DAM = ee_get_DAM();
    Byte_Type[0] = DAM;
    if((DAM >> 6) == 0)
    {
        Register_Counter = DAM & 0x3F;
        if(Register_Counter >= _END_OF_RO_FLOAT_ADDRESS)
            Register_Counter = 0;
        Current_State = _REGISTER_ADDRESS_SHOW;
    }
    else if((DAM >> 6) == 1)
    {
        Register_Counter = End_Of_Float + (DAM & 0x3F);
        if(Register_Counter >= End_Of_Float + _END_OF_RO_WORD_ADDRESS)
            Register_Counter = End_Of_Float;
        if(Register_Counter >= (End_Of_Float + _U_VO_PV) && Register_Counter <= (End_Of_Float + _U_TH_PV))
            Current_State = _WORD_H_REGISTER_VALUE_SHOW;
        else
            Current_State = _REGISTER_ADDRESS_SHOW;
    }
    else if((DAM >> 6) == 2)
    {
        Register_Counter = End_Of_Word + (DAM & 0x3F);
        if(Register_Counter >= End_Of_Word + _END_OF_RW_BYTE_ADDRESS)
            Register_Counter = End_Of_Word;
        Current_State = _REGISTER_ADDRESS_SHOW;
    }
    else if((DAM >> 6) == 3)
    {
        Register_Counter = End_Of_Byte + (DAM & 0x3F);
        if(Register_Counter >= End_Of_Byte + _END_OF_RW_BYTE_ADDRESS)
            Register_Counter = End_Of_Byte;
        Current_State = _REGISTER_ADDRESS_SHOW;
    }
    
    LHt_WORD = ee_get_LHt_ADRESS_WORD();
    LHt_REAL = ee_get_LHt_ADRESS_REAL();
    HHt_WORD = ee_get_HHt_ADRESS_WORD();
    HHt_REAL = ee_get_HHt_ADRESS_REAL();
    Temporary_float_Register = 255;
    NTC_G = Temporary_float_Register / (float)((float)HHt_WORD - LHt_WORD);
    
    LOU_WORD = ee_get_LOU_ADRESS_WORD();
    HOU_WORD = ee_get_HOU_ADRESS_WORD();
    HOU_REAL = ee_get_HOU_ADRESS_REAL();
    VO_G = (float)HOU_REAL / ((float)((float)HOU_WORD -LOU_WORD) * 10.00) ;  
    
    LBC_WORD = ee_get_LbC_ADRESS_WORD();
    HBC_WORD = ee_get_HbC_ADRESS_WORD();
    HBC_REAL = ee_get_HbC_ADRESS_REAL();
    IB_G = (float)HBC_REAL / ((float)HBC_WORD -LBC_WORD); 
    
    LLC_WORD = ee_get_LLC_ADRESS_WORD();
    HLC_WORD = ee_get_HLC_ADRESS_WORD();
    HLC_REAL = ee_get_HLC_ADRESS_REAL();
    IO_G = (float)HLC_REAL / ((float)HLC_WORD -LLC_WORD); 
    
    Lbt_WORD = ee_get_Lbt_ADRESS_WORD();
    Lbt_REAL = ee_get_Lbt_ADRESS_REAL();
    Hbt_WORD = ee_get_Hbt_ADRESS_WORD();
    Hbt_REAL = ee_get_Hbt_ADRESS_REAL();
    TBAT_G = (float)((float)Hbt_REAL - Lbt_REAL) / ((float)Hbt_WORD - Lbt_WORD);
    TBAT_WZ = Lbt_WORD;
    
    LIU_WORD = ee_get_LIU_ADRESS_WORD();
    HIU_WORD = ee_get_HIU_ADRESS_WORD();
    HIU_REAL = ee_get_HIU_ADRESS_REAL();
    VI_G = (float)HIU_REAL * 4.0 / ((float)HIU_WORD -LIU_WORD); 
    
    for(i = 0; i < 256; i ++)
    {
        LUT[i] = ee_get_LUT_ADRESS(i);
    }
    
    for(i = 0; i < 10; i ++)
    {
        Que_Alarm[i] = ee_get_Que_Alarm_ADRESS(i);
    }
    
    //---------------------------------------------------------------
    for(i = 0; i < _END_OF_RW_BYTE_ADDRESS; i++)
    {
        Byte_Type[i] = ee_get_Byte_Type_ADRESS(i);
    }
    for(i = 0; i < _END_OF_RW_WORD_ADDRESS; i++)
    {
        Word_Type[i] = ee_get_Word_Type_ADRESS(i);
    }
    for(i = 0; i < _END_OF_RW_FLOAT_ADDRESS; i++)
    {
        Real_Type[i] = ee_get_Real_Type_ADRESS(i);
    }
    for(i = 0; i < _END_OF_RW_FLOAT_ADDRESS; i++)
    {
        Float_Type[i] = ee_get_Float_Type_ADRESS(i);
        r[i] = (float)((float)Real_Type[i] * 10 + Float_Type[i]) / 10.0;
    }    
    for(i = 0; i < _END_OF_RW_FLAG_ADDRESS2; i++)
    {
        Flag_Type[i+_END_OF_EVENT_FLAG_ADDRESS] = ee_get_Flag_Type_ADRESS(i);        
    }
    
    for(i = 0; i < 33; i ++)
    {
        f_property[i] = Flag_Type[i+_END_OF_EVENT_FLAG_ADDRESS];
    }
    //---------------------------------------------------------------
    
    
    IB_DEC_G = r[_r_IB_SP] / (r[_r_TEMP_ZERRO_IB] - r[_r_TEMP_DEC_IB]);
    IO_DEC_G = r[_r_IO_SP] / (r[_r_TEMP_ZERRO_IO] - r[_r_TEMP_DEC_IO]);
    
    Set_Limits_Value();
}

void External_Interrupt(unsigned char status)
{
    if(status == _ON)
    {
        INTCON2bits.INT0EP = 1;     // Interrupt on negative edge
        INTCON2bits.INT1EP = 1;     // Interrupt on negative edge
        INTCON2bits.INT2EP = 1;     // Interrupt on negative edge

        IFS0bits.INT0IF = 0;    /*Reset INT0 interrupt flag */
        IEC0bits.INT0IE = 1;    /*Enable INT0 Interrupt Service Routine */
        IPC0bits.INT0IP = 7;

        IFS1bits.INT1IF = 0;    /*Reset INT1 interrupt flag */
        IEC1bits.INT1IE = 1;    /*Enable INT1 Interrupt Service Routine */
        IPC4bits.INT1IP = 7;

        IFS1bits.INT2IF = 0;    /*Reset INT0 interrupt flag */
        IEC1bits.INT2IE = 1;    /*Enable INT0 Interrupt Service Routine */
        IPC5bits.INT2IP = 7;
        
        
    }
    else
    {
        IFS0bits.INT0IF = 0;    /*Reset INT0 interrupt flag */
        IEC0bits.INT0IE = 0;    /*Enable INT0 Interrupt Service Routine */
        IFS1bits.INT1IF = 0;    /*Reset INT1 interrupt flag */
        IEC1bits.INT1IE = 0;    /*Enable INT1 Interrupt Service Routine */
        IFS1bits.INT2IF = 0;    /*Reset INT0 interrupt flag */
        IEC1bits.INT2IE = 0;    /*Enable INT0 Interrupt Service Routine */
    }
}

unsigned char CMP_Arrays(unsigned char * a1, unsigned char n) 
{
    unsigned char temp_array1[3] = {0,1,2};
    unsigned char temp_array2[3] = {2,0,1};
    unsigned char temp_array3[3] = {1,2,0};
    unsigned char temp_array4[3] = {0,2,1};
    unsigned char temp_array5[3] = {1,0,2};
    unsigned char temp_array6[3] = {2,1,0};
    unsigned char temp_array7[3] = {0,0,0};
    int i;
    
    switch(n)
    {
        case 0:
        for (i=0; i<3; ++i)
        {
            if (a1[i] != temp_array1[i])
                                return 0;
        }
        break;
        case 1:
        for (i=0; i<3; ++i)
        {
            if (a1[i] != temp_array2[i])
                                return 0;
        }
        break;
        case 2:
        for (i=0; i<3; ++i)
        {
            if (a1[i] != temp_array3[i])
                                return 0;
        }
        break;
        case 3:
        for (i=0; i<3; ++i)
            if (a1[i] != temp_array4[i])
                                return 0;
        break;
        case 4:
        for (i=0; i<3; ++i)
        {
            if (a1[i] != temp_array5[i])
                                return 0;
        }
        break;
        case 5:
        for (i=0; i<3; ++i)
        {
            if (a1[i] != temp_array6[i])
                                return 0;
        }
        break;
        case 6:
        for (i=0; i<3; ++i)
        {
            if (a1[i] != temp_array7[i])
                                return 0;
        }
        break;
    }
    return (1);
}

void Event_Flag_Update(void)
{
    int i = 0;
//    Word_Type[_U_TEST1] = Phase_Sync_Counter[0];
//    Word_Type[_U_TEST2] = Phase_Sync_Counter[1];
//    Word_Type[_U_TEST3] = Phase_Sync_Counter[2];
//    if(Phase_Sync_Counter[0] > 3)
//        F_L1_LOSS = 1;
//    else
//        F_L1_LOSS = 0;
//    
//    if(Phase_Sync_Counter[1] > 3)
//        F_L2_LOSS = 1;
//    else
//        F_L2_LOSS = 0;
//    
//    if(Phase_Sync_Counter[2] > 3)
//        F_L3_LOSS = 1;
//    else
//        F_L3_LOSS = 0;
    f_event[0] = F_LEFT;
    f_event[1] = F_RIGHT;
    f_event[2] = F_L1_LOSS;
    f_event[3] = F_L2_LOSS;
    f_event[4] = F_L3_LOSS;
    f_event[5] = F_FAIL_DATA_ENTRY;
    f_event[6] = (r[_r_VO_PV] > (r[_r_VO_SP] * 1.2));
    f_event[7] = ((r[_r_VO_SP] * 0.5) > r[_r_VO_PV]);
    f_event[8] = (r[_r_IB_PV] > (r[_r_IB_SP] * 1.2));
    f_event[9] = ((r[_r_IB_SP] * 0.1) > r[_r_IB_PV]);
    f_event[10] = (r[_r_IO_PV] > (r[_r_IO_SP] * 1.2));
    f_event[11] = ((r[_r_IO_SP] * 0.1) > r[_r_IO_PV]);
    if(r[_r_BAT_TEMP] > r[_r_TEMP_ZERRO_IB] && Word_Type[_AI_BAT_TEMP] < _NO_PT100_VALUE)
        f_event[12] = 1;
    else if(r[_r_BAT_TEMP] < r[_r_TEMP_ZERRO_IB] * 0.9)
        f_event[12] = 0;
    f_event[13] = Flag_Type[_F_IB_SP_SEL_DEC];
    if(Word_Type[_r_HSINK] > r[_r_TEMP_ZERRO_IO])
        f_event[14] = 1;
    else if(Word_Type[_r_HSINK] < r[_r_TEMP_ZERRO_IO] * 0.9)
        f_event[14] = 0;
    f_event[15] = Flag_Type[_F_IO_SP_SEL_DEC];
    if((Word_Type[_AI_IBAT] > 100 || Word_Type[_AI_IOUT] > 100) && 1200 > Word_Type[_AI_VOUT])
        f_event[16] = 1;
    f_event[17] = (r[_r_VIN] > r[_r_VIN_HIGH_ALARM]);
    f_event[18] = (r[_r_VIN] < r[_r_VIN_LOW_ALARM]);
    f_event[19] = (1000 > AI_OFF_SCR_BATTERY_VOLTAGE);
    f_event[20] = (Word_Type[_AI_BAT_TEMP] > _NO_PT100_VALUE);
    f_event[21] = (Word_Type[_AI_HSINK_TEMP] > 3700);
    F_E2ROM_FAIL = (ERR_OUT == 0);
    f_event[22] = F_E2ROM_FAIL;
    f_event[23] = WATCH_DOG_FAIL;
    f_event[24] = (CMD_TYPE == 1);
    f_event[25] = (CMD_TYPE == 2);
    f_event[26] = (CMD_TYPE == 3);
    f_event[27] = (CMD_TYPE == 4);
    
    if( F_NO_LOSS == 0 || (ERR_OUT == 0))
    {
        f_event[24] = 0;
        f_event[25] = 0;
        f_event[26] = 0;
    }
    
    if(f_event[6] == 1 || f_event[7] == 1)
    {
       Start_Output_Over_Under_V_Flag = 1;
       if(Output_Over_Under_V_Counter > 200)
       {
           if(f_event[6] == 1)
               f_event[28] = 1;
           if(f_event[7] == 1)
               f_event[29] = 1;
           Start_Output_Over_Under_V_Flag = 0;
           Output_Over_Under_V_Counter = 0;
       }
    }
    else
    {
        Start_Output_Over_Under_V_Flag = 0;
        Output_Over_Under_V_Counter = 0;
    }
    
    if(f_event[8] == 1 || f_event[10] == 1)
    {
       Load_Battery_Over_I_Flag = 1;
       if(Load_Battery_Over_I_Counter > 200)
       {
           if(f_event[8] == 1)
               f_event[30] = 1;
           if(f_event[10] == 1)
               f_event[31] = 1;
           Load_Battery_Over_I_Flag = 0;
           Load_Battery_Over_I_Counter = 0;
       }
    }
    else
    {
        Load_Battery_Over_I_Flag = 0;
        Load_Battery_Over_I_Counter = 0;
    }
    
    if(Flag_Type[_F_rSt] == 1)
    {
        f_event[28] = 0;
        f_event[29] = 0;
        f_event[30] = 0;
        f_event[31] = 0;
    }
    
    if(Flag_Type[_F_FAIL_ENABLE] == 0)
    {
        F_FAIL = 0;
        f_event[32] = 0;
    }
    else if((f_event[2] == 1 || f_event[3] == 1 || f_event[4] == 1) && Flag_Type[_F_PHASE_LOSS_MASK] == 0)
    {
        F_FAIL = 1;
        f_event[32] = 1;
    }
    else if(f_event[28] == 1 || f_event[29] == 1 || f_event[30] == 1 ||
            f_event[31] == 1 || f_event[16] == 1 || f_event[14] == 1)
    {
        F_FAIL = 1;
        f_event[32] = 1;
    }
    else
    {
        F_FAIL = 0;
        f_event[32] = 0;
    }   
    
    
    for(i = 0; i < 33; i ++)
    {
        if(f_event[i] == 0)
            f_event_already_happened[i] = 0;
    }
    //F_FAIL = 1;
    //READY = 1 - F_FAIL;
}

void PID_Initialize(void)
{
    fooPID1.abcCoefficients = &abcCoefficient1[0];    
    fooPID1.controlHistory = &controlHistory1[0];     
    PIDInit(&fooPID1);                               
    kCoeffs1[0] = Q15((float)Byte_Type[_b_VO_P] / 255.00);//Byte_Type[_b_VO_P];//Q15(0.7);
    
    if(Byte_Type[_b_VO_I] == 0)
        kCoeffs1[1] = Q15(0.00);
    else
        kCoeffs1[1] = Q15(((float)Byte_Type[_b_VO_I] * 0.0039) - 0.0014);//Byte_Type[_b_VO_I];//Q15(0.2);
    
    kCoeffs1[2] = Q15((float)Byte_Type[_b_VO_D] / 255.00);//Byte_Type[_b_VO_D];//Q15(0.07);
    PIDCoeffCalc(&kCoeffs1[0], &fooPID1); 
          
    fooPID2.abcCoefficients = &abcCoefficient2[0];    
    fooPID2.controlHistory = &controlHistory2[0];     
    PIDInit(&fooPID2);                               
    kCoeffs2[0] = Q15((float)Byte_Type[_b_IB_P] / 255.00);//Byte_Type[_b_IB_P];//Q15(0.7);
    if(Byte_Type[_b_IB_I] == 0)
        kCoeffs2[1] = Q15(0.00);
    else
        kCoeffs2[1] = Q15(((float)Byte_Type[_b_IB_I] * 0.0039) - 0.0014);//Byte_Type[_b_IB_I];//Q15(0.2);
    kCoeffs2[2] = Q15((float)Byte_Type[_b_IB_D] / 255.00);//Byte_Type[_b_IB_D];//Q15(0.07);
    PIDCoeffCalc(&kCoeffs2[0], &fooPID2); 
    
    fooPID3.abcCoefficients = &abcCoefficient3[0];    
    fooPID3.controlHistory = &controlHistory3[0];     
    PIDInit(&fooPID3);                               
    kCoeffs3[0] = Q15((float)Byte_Type[_b_IO_P] / 255.00);//Byte_Type[_b_IO_P];//Q15(0.7);
    if(Byte_Type[_b_IO_I] == 0)
        kCoeffs3[1] = Q15(0.00);
    else
        kCoeffs3[1] = Q15(((float)Byte_Type[_b_IO_I] * 0.0039) - 0.0014);//Byte_Type[_b_IO_I];//Q15(0.2);
    kCoeffs3[2] = Q15((float)Byte_Type[_b_IO_D] / 255.00);//Byte_Type[_b_IO_D];//Q15(0.07);
    PIDCoeffCalc(&kCoeffs3[0], &fooPID3); 
}

void Factory_Reset(void)
{
    unsigned int i = 0;
    unsigned char Temp_LUT[256] = { 0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,
                                    0x02,0x02,0x02,0x03,0x03,0x03,0x03,0x04,
                                    0x04,0x04,0x05,0x05,0x05,0x05,0x06,0x06,
                                    0x06,0x07,0x07,0x07,0x07,0x08,0x08,0x08,
                                    0x09,0x09,0x09,0x09,0x0A,0x0A,0x0A,0x0B,
                                    0x0B,0x0B,0x0B,0x0C,0x0C,0x0C,0x0C,0x0D,
                                    0x0D,0x0D,0x0E,0x0E,0x0E,0x0E,0x0F,0x0F,
                                    0x0F,0x10,0x10,0x10,0x10,0x11,0x11,0x11,
                                    0x12,0x12,0x12,0x12,0x13,0x13,0x13,0x14,
                                    0x14,0x14,0x14,0x15,0x15,0x15,0x16,0x16,
                                    0x16,0x16,0x17,0x17,0x17,0x18,0x18,0x18,
                                    0x18,0x19,0x19,0x19,0x19,0x1A,0x1A,0x1A,
                                    0x1B,0x1B,0x1B,0x1B,0x1C,0x1C,0x1C,0x1D,
                                    0x1D,0x1D,0x1D,0x1E,0x1E,0x1E,0x1F,0x1F,
                                    0x1F,0x1F,0x20,0x20,0x20,0x21,0x21,0x21,
                                    0x21,0x22,0x22,0x22,0x23,0x23,0x23,0x23,
                                    0x24,0x24,0x24,0x25,0x25,0x25,0x25,0x26,
                                    0x26,0x26,0x26,0x27,0x27,0x27,0x28,0x28,
                                    0x28,0x28,0x29,0x29,0x29,0x2A,0x2A,0x2A,
                                    0x2A,0x2B,0x2B,0x2B,0x2C,0x2C,0x2C,0x2C,
                                    0x2D,0x2D,0x2D,0x2E,0x2E,0x2E,0x2E,0x2F,
                                    0x2F,0x2F,0x30,0x30,0x30,0x30,0x31,0x31,
                                    0x31,0x32,0x32,0x33,0x33,0x34,0x35,0x35,
                                    0x36,0x37,0x37,0x38,0x39,0x39,0x3A,0x3A,
                                    0x3B,0x3C,0x3C,0x3D,0x3E,0x3E,0x3F,0x40,
                                    0x40,0x41,0x42,0x42,0x43,0x43,0x44,0x45,
                                    0x45,0x46,0x47,0x47,0x48,0x49,0x49,0x4A,
                                    0x4B,0x4B,0x4C,0x4C,0x4D,0x4E,0x4E,0x4F,
                                    0x50,0x50,0x51,0x52,0x52,0x53,0x53,0x54,
                                    0x55,0x55,0x56,0x57,0x57,0x58,0x59,0x59,
                                    0x5A,0x5B,0x5B,0x5C,0x5C,0x5D,0x5E,0x5E,
                                    0x5F,0x60,0x60,0x61,0x62,0x62,0x63,0x64};
    Byte_Type[0] = 0;
    
    LBC_WORD = 0;
    
    
    HBC_WORD = 2048;
    HBC_REAL = 100;
    

    LLC_WORD = 0;
    

    HLC_WORD = 2048;
    HLC_REAL = 100;
  

    LOU_WORD = 0;
  

    HOU_WORD = 4096;
    HOU_REAL = 367;


    LIU_WORD = 0;
 

    HIU_WORD = 4096;
    HIU_REAL = 137;


    Lbt_WORD = 260;
    Lbt_REAL = 0;
 

    Hbt_WORD = 3667;
    Hbt_REAL = 95;


    LHt_WORD = 3355;
    LHt_REAL = 0;

//    HMt_WORD = 2000;
//    HMt_REAL = 150;


    HHt_WORD = 298;
    HHt_REAL = 125;

    
    Temporary_float_Register = 255;
    NTC_G = Temporary_float_Register / (float)((float)HHt_WORD - LHt_WORD);
    VO_G = (float)HOU_REAL / ((float)((float)HOU_WORD -LOU_WORD) * 10.00) ;  
    IB_G = (float)HBC_REAL / ((float)HBC_WORD -LBC_WORD);     
    IO_G = (float)HLC_REAL / ((float)HLC_WORD -LLC_WORD);    
    TBAT_G = (float)((float)Hbt_REAL - Lbt_REAL) / ((float)Hbt_WORD - Lbt_WORD);
    TBAT_WZ = Lbt_WORD;    
    VI_G = (float)HIU_REAL  * 4.0 / ((float)HIU_WORD -LIU_WORD); 
    
    for(i = 0; i < 256; i++)
    {
        LUT[i] = Temp_LUT[i];                   
    }
    
    Last_Event_Number = 0;
    for(i = 0; i < 10; i++)
    {
        Que_Alarm[i] = 0;
    }
    Byte_Type[_b_VO_P] = 100;
    Byte_Type[_b_VO_I] = 100;
    Byte_Type[_b_VO_D] = 10;
    Byte_Type[_b_IB_P] = 100;
    Byte_Type[_b_IB_I] = 100;
    Byte_Type[_b_IB_D] = 10;
    Byte_Type[_b_IO_P] = 100;
    Byte_Type[_b_IO_I] = 100;
    Byte_Type[_b_IO_D] = 10;
    Byte_Type[_b_CMD] = 0;
    Byte_Type[_b_DATA_REAL] = 0;
    Byte_Type[_b_VO_GAIN] = 10;
    Byte_Type[_b_IB_GAIN] = 10;
    Byte_Type[_b_IO_GAIN] = 10;    
    
    Word_Type[_r_CMD] = 0;
    Word_Type[_r_USTEP] = 0;
    Word_Type[_r_DSTEP] = 0;
    Word_Type[_U_CV_Z] = 0;
    Word_Type[_CV_G] = 0;
    Word_Type[_U_L1] = 39148;//23319;//31233;
    Word_Type[_U_L2] = 39148;//22619;//30062;//30308;
    Word_Type[_U_L3] = 39148;//31519;//30482;//30382;
    
    Real_Type[_r_VO_SP] = 27;
    Real_Type[_r_IO_SP] = 100;
    Real_Type[_r_IB_SP] = 25;
    Real_Type[_r_TEMP_DEC_IB] = 40;
    Real_Type[_r_TEMP_ZERRO_IB] = 50;
    Real_Type[_r_TEMP_DEC_IO] = 80;
    Real_Type[_r_TEMP_ZERRO_IO] = 85;
    Real_Type[_r_VIN_HIGH_ALARM] = 660;
    Real_Type[_r_VIN_LOW_ALARM] = 370;
    
    Float_Type[_r_VO_SP] = 0;
    Float_Type[_r_IO_SP] = 0;
    Float_Type[_r_IB_SP] = 0;
    Float_Type[_r_TEMP_DEC_IB] = 0;
    Float_Type[_r_TEMP_ZERRO_IB] = 0;
    Float_Type[_r_TEMP_DEC_IO] = 0;
    Float_Type[_r_TEMP_ZERRO_IO] = 0;
    Float_Type[_r_VIN_HIGH_ALARM] = 0;
    Float_Type[_r_VIN_LOW_ALARM] = 0;
    
    for(i = 0; i < _END_OF_RW_FLOAT_ADDRESS; i++)
    {
        r[i] = (float)((float)Real_Type[i] * 10.00 + Float_Type[i]) / 10.0;
    }
    
    Flag_Type[2 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[3 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[4 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[16 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[28 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[29 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[30 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[31 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[32 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[6 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[7 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[8 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[10 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[12 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[14 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[17 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[18 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[23 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[22 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[0 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[1 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[5 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[9 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[10 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[13 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[15 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[24 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[25 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[33 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[26 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[20 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[21 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[19 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    
    Flag_Type[_F_VOFS] = 1;
    Flag_Type[_F_VO_SP_SEL] = 0;
    Flag_Type[_F_IBFS] = 1;
    Flag_Type[_F_IB_SP_SEL_DEC] = 0;
    Flag_Type[_F_IOFS] = 1;
    Flag_Type[_F_IO_SP_SEL_DEC] = 0;
    Flag_Type[_F_ERU] = 0;
    Flag_Type[_F_ERD] = 0;
    Flag_Type[_F_IB_SP_SEL] = 0;
    Flag_Type[_F_IO_SP_SEL] = 0;
    Flag_Type[_F_IB_SP_ALARM_MASK] = 0;
    Flag_Type[_F_IO_SP_ALARM_MASK] = 0;
    Flag_Type[_F_IB_TEMP_SP_MASK] = 0;
    Flag_Type[_F_IO_TEMP_SP_MASK] = 0;
    Flag_Type[_F_VIN_HIGH_ALARM_MASK] = 0;
    Flag_Type[_F_rSt] = 0;
    Flag_Type[_F_PHASE_LOSS_MASK] = 0;   
    Flag_Type[_F_FAIL_ENABLE] = 1;
    Flag_Type[_F_CHARGER_DISABLE] = 0;

}

void Save_Factory_Reset(void)
{
    unsigned int i = 0;
    unsigned char Temp_LUT[256] = { 0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,
                                    0x02,0x02,0x02,0x03,0x03,0x03,0x03,0x04,
                                    0x04,0x04,0x05,0x05,0x05,0x05,0x06,0x06,
                                    0x06,0x07,0x07,0x07,0x07,0x08,0x08,0x08,
                                    0x09,0x09,0x09,0x09,0x0A,0x0A,0x0A,0x0B,
                                    0x0B,0x0B,0x0B,0x0C,0x0C,0x0C,0x0C,0x0D,
                                    0x0D,0x0D,0x0E,0x0E,0x0E,0x0E,0x0F,0x0F,
                                    0x0F,0x10,0x10,0x10,0x10,0x11,0x11,0x11,
                                    0x12,0x12,0x12,0x12,0x13,0x13,0x13,0x14,
                                    0x14,0x14,0x14,0x15,0x15,0x15,0x16,0x16,
                                    0x16,0x16,0x17,0x17,0x17,0x18,0x18,0x18,
                                    0x18,0x19,0x19,0x19,0x19,0x1A,0x1A,0x1A,
                                    0x1B,0x1B,0x1B,0x1B,0x1C,0x1C,0x1C,0x1D,
                                    0x1D,0x1D,0x1D,0x1E,0x1E,0x1E,0x1F,0x1F,
                                    0x1F,0x1F,0x20,0x20,0x20,0x21,0x21,0x21,
                                    0x21,0x22,0x22,0x22,0x23,0x23,0x23,0x23,
                                    0x24,0x24,0x24,0x25,0x25,0x25,0x25,0x26,
                                    0x26,0x26,0x26,0x27,0x27,0x27,0x28,0x28,
                                    0x28,0x28,0x29,0x29,0x29,0x2A,0x2A,0x2A,
                                    0x2A,0x2B,0x2B,0x2B,0x2C,0x2C,0x2C,0x2C,
                                    0x2D,0x2D,0x2D,0x2E,0x2E,0x2E,0x2E,0x2F,
                                    0x2F,0x2F,0x30,0x30,0x30,0x30,0x31,0x31,
                                    0x31,0x32,0x32,0x33,0x33,0x34,0x35,0x35,
                                    0x36,0x37,0x37,0x38,0x39,0x39,0x3A,0x3A,
                                    0x3B,0x3C,0x3C,0x3D,0x3E,0x3E,0x3F,0x40,
                                    0x40,0x41,0x42,0x42,0x43,0x43,0x44,0x45,
                                    0x45,0x46,0x47,0x47,0x48,0x49,0x49,0x4A,
                                    0x4B,0x4B,0x4C,0x4C,0x4D,0x4E,0x4E,0x4F,
                                    0x50,0x50,0x51,0x52,0x52,0x53,0x53,0x54,
                                    0x55,0x55,0x56,0x57,0x57,0x58,0x59,0x59,
                                    0x5A,0x5B,0x5B,0x5C,0x5C,0x5D,0x5E,0x5E,
                                    0x5F,0x60,0x60,0x61,0x62,0x62,0x63,0x64};
    _Wdt_Status = _OFF;
    Byte_Type[0] = 0;
    ee_set_DAM(Byte_Type[0]);
    
    LBC_WORD = 0;
    ee_set_LbC_ADRESS_WORD(LBC_WORD);
    
    HBC_WORD = 2048;
    HBC_REAL = 100;
    ee_set_HbC_ADRESS_WORD(HBC_WORD);
    ee_set_HbC_ADRESS_REAL(HBC_REAL);

    LLC_WORD = 0;
    ee_set_LLC_ADRESS_WORD(LLC_WORD);

    HLC_WORD = 2048;
    HLC_REAL = 100;
    ee_set_HLC_ADRESS_WORD(HLC_WORD);
    ee_set_HLC_ADRESS_REAL(HLC_REAL);

    LOU_WORD = 0;
    ee_set_LOU_ADRESS_WORD(LOU_WORD);

    HOU_WORD = 4096;
    HOU_REAL = 367;
    ee_set_HOU_ADRESS_WORD(HOU_WORD);
    ee_set_HOU_ADRESS_REAL(HOU_REAL);//???????

    LIU_WORD = 0;
    ee_set_LIU_ADRESS_WORD(LIU_WORD);

    HIU_WORD = 4096;
    HIU_REAL = 137;
    ee_set_HIU_ADRESS_WORD(HIU_WORD);
    ee_set_HIU_ADRESS_REAL(HIU_REAL);

    Lbt_WORD = 260;
    Lbt_REAL = 0;
    ee_set_Lbt_ADRESS_WORD(Lbt_WORD);
    ee_set_Lbt_ADRESS_REAL(Lbt_REAL);

    Hbt_WORD = 3667;
    Hbt_REAL = 95;
    ee_set_Hbt_ADRESS_WORD(Hbt_WORD);
    ee_set_Hbt_ADRESS_REAL(Hbt_REAL);

    LHt_WORD = 3355;
    LHt_REAL = 0;
    ee_set_LHt_ADRESS_WORD(LHt_WORD);
    ee_set_LHt_ADRESS_REAL(LHt_REAL);

//    HMt_WORD = 2000;
//    HMt_REAL = 150;
//    ee_set_HMt_ADRESS_WORD(HMt_WORD);
//    ee_set_HMt_ADRESS_REAL(HMt_REAL);

    HHt_WORD = 298;
    HHt_REAL = 125;
    ee_set_HHt_ADRESS_WORD(HHt_WORD);
    ee_set_HHt_ADRESS_REAL(HHt_REAL);
    
    Temporary_float_Register = 255;
    NTC_G = Temporary_float_Register / (float)((float)HHt_WORD - LHt_WORD);
    VO_G = (float)HOU_REAL / ((float)((float)HOU_WORD -LOU_WORD) * 10.00) ;  
    IB_G = (float)HBC_REAL / ((float)HBC_WORD -LBC_WORD);     
    IO_G = (float)HLC_REAL / ((float)HLC_WORD -LLC_WORD);    
    TBAT_G = (float)((float)Hbt_REAL - Lbt_REAL) / ((float)Hbt_WORD - Lbt_WORD);
    TBAT_WZ = Lbt_WORD;    
    VI_G = (float)HIU_REAL  * 4.0 / ((float)HIU_WORD -LIU_WORD); 
    
    for(i = 0; i < 256; i++)
    {
        LUT[i] = Temp_LUT[i];
        ee_set_LUT_ADRESS(LUT[i], i);                   
    }
    
    Last_Event_Number = 0;
    ee_set_POINTER_OF_LAST_EVENT(Last_Event_Number);
    for(i = 0; i < 10; i++)
    {
        Que_Alarm[i] = 0;
        ee_set_Que_Alarm_ADRESS(Que_Alarm[i],i);
    }
    Byte_Type[_b_VO_P] = 100;
    Byte_Type[_b_VO_I] = 100;
    Byte_Type[_b_VO_D] = 10;
    Byte_Type[_b_IB_P] = 100;
    Byte_Type[_b_IB_I] = 100;
    Byte_Type[_b_IB_D] = 10;
    Byte_Type[_b_IO_P] = 100;
    Byte_Type[_b_IO_I] = 100;
    Byte_Type[_b_IO_D] = 10;
    Byte_Type[_b_CMD] = 0;
    Byte_Type[_b_DATA_REAL] = 0;
    Byte_Type[_b_VO_GAIN] = 10;
    Byte_Type[_b_IB_GAIN] = 10;
    Byte_Type[_b_IO_GAIN] = 10;    
    for(i = 0; i < _END_OF_RW_BYTE_ADDRESS; i++)
    {
        ee_set_Byte_Type_ADRESS(Byte_Type[i], i);
    }
    
    Word_Type[_r_CMD] = 0;
    Word_Type[_r_USTEP] = 0;
    Word_Type[_r_DSTEP] = 0;
    Word_Type[_U_CV_Z] = 0;
    Word_Type[_CV_G] = 0;
    Word_Type[_U_L1] = 39148;//23319;//31233;
    Word_Type[_U_L2] = 39148;//22619;//30062;//30308;
    Word_Type[_U_L3] = 39148;//31519;//30482;//30382;
    for(i = 0; i < _END_OF_RW_WORD_ADDRESS; i++)
    {
        ee_set_Word_Type_ADRESS(Word_Type[i], i);
    }
    
    Real_Type[_r_VO_SP] = 27;
    Real_Type[_r_IO_SP] = 100;
    Real_Type[_r_IB_SP] = 25;
    Real_Type[_r_TEMP_DEC_IB] = 40;
    Real_Type[_r_TEMP_ZERRO_IB] = 50;
    Real_Type[_r_TEMP_DEC_IO] = 80;
    Real_Type[_r_TEMP_ZERRO_IO] = 85;
    Real_Type[_r_VIN_HIGH_ALARM] = 660;
    Real_Type[_r_VIN_LOW_ALARM] = 370;
    for(i = 0; i < _END_OF_RW_FLOAT_ADDRESS; i++)
    {
        ee_set_Real_Type_ADRESS(Real_Type[i], i);
    }
    
    Float_Type[_r_VO_SP] = 0;
    Float_Type[_r_IO_SP] = 0;
    Float_Type[_r_IB_SP] = 0;
    Float_Type[_r_TEMP_DEC_IB] = 0;
    Float_Type[_r_TEMP_ZERRO_IB] = 0;
    Float_Type[_r_TEMP_DEC_IO] = 0;
    Float_Type[_r_TEMP_ZERRO_IO] = 0;
    Float_Type[_r_VIN_HIGH_ALARM] = 0;
    Float_Type[_r_VIN_LOW_ALARM] = 0;
    for(i = 0; i < _END_OF_RW_FLOAT_ADDRESS; i++)
    {
        ee_set_Float_Type_ADRESS(Float_Type[i], i);
        r[i] = (float)((float)Real_Type[i] * 10.00 + Float_Type[i]) / 10.0;
    }
    
    Flag_Type[2 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[3 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[4 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[16 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[28 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[29 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[30 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[31 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[32 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[6 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[7 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[8 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[10 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[12 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[14 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[17 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[18 + _END_OF_EVENT_FLAG_ADDRESS] = 3;
    Flag_Type[23 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[22 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[0 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[1 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[5 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[9 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[10 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[11 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[13 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[15 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[24 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[25 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[33 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[26 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[20 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[21 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    Flag_Type[19 + _END_OF_EVENT_FLAG_ADDRESS] = 1;
    
    Flag_Type[_F_VOFS] = 1;
    Flag_Type[_F_VO_SP_SEL] = 0;
    Flag_Type[_F_IBFS] = 1;
    Flag_Type[_F_IB_SP_SEL_DEC] = 0;
    Flag_Type[_F_IOFS] = 1;
    Flag_Type[_F_IO_SP_SEL_DEC] = 0;
    Flag_Type[_F_ERU] = 0;
    Flag_Type[_F_ERD] = 0;
    Flag_Type[_F_IB_SP_SEL] = 0;
    Flag_Type[_F_IO_SP_SEL] = 0;
    Flag_Type[_F_IB_SP_ALARM_MASK] = 0;
    Flag_Type[_F_IO_SP_ALARM_MASK] = 0;
    Flag_Type[_F_IB_TEMP_SP_MASK] = 0;
    Flag_Type[_F_IO_TEMP_SP_MASK] = 0;
    Flag_Type[_F_VIN_HIGH_ALARM_MASK] = 0;
    Flag_Type[_F_rSt] = 0;
    Flag_Type[_F_PHASE_LOSS_MASK] = 0;   
    Flag_Type[_F_FAIL_ENABLE] = 1;
    Flag_Type[_F_CHARGER_DISABLE] = 0;
    for(i = 0; i < _END_OF_RW_FLAG_ADDRESS2; i++)
    {
        ee_set_Flag_Type_ADRESS(Flag_Type[i+_END_OF_EVENT_FLAG_ADDRESS], i);
    }
    _Wdt_Status = _ON;
}

void Set_Limits_Value(void)
{    
    /*if(Word_Type[_U_L1] < 50000)
        Word_Type[_U_L1] = 50000;
    if(Word_Type[_U_L2] < 50000)
        Word_Type[_U_L2] = 50000;
    if(Word_Type[_U_L3] < 50000)
        Word_Type[_U_L3] = 50000;*/
    
    if(Word_Type[_U_L1] > _MAX_Delay)
        Word_Type[_U_L1] = _MAX_Delay;
    if(Word_Type[_U_L2] > _MAX_Delay)
        Word_Type[_U_L2] = _MAX_Delay;
    if(Word_Type[_U_L3] > _MAX_Delay)
        Word_Type[_U_L3] = _MAX_Delay;
    
    if(Word_Type[_U_L1] < _MIN_Delay)
        Word_Type[_U_L1] = _MIN_Delay;
    if(Word_Type[_U_L2] < _MIN_Delay)
        Word_Type[_U_L2] = _MIN_Delay;
    if(Word_Type[_U_L3] < _MIN_Delay)
        Word_Type[_U_L3] = _MIN_Delay;
    
    
    if(Word_Type[_r_CMD] > 9000)
        Word_Type[_r_CMD] = 9000;
//    if(Word_Type[_r_CMD] < 60)
//        Word_Type[_r_CMD] = 60;
    
    if(r[_r_VO_SP] > 29)
        r[_r_VO_SP] = 29;
    /*if(r[_r_VO_SP] < 24)
        r[_r_VO_SP] = 24;*/
    
    if(r[_r_IB_SP] > 70)
        r[_r_IB_SP] = 70;
    
    if(r[_r_IO_SP] > 150)
        r[_r_IO_SP] = 150;
    
    if(r[_r_TEMP_DEC_IB] > 55)
        r[_r_TEMP_DEC_IB] = 55;
    if(r[_r_TEMP_DEC_IB] < 25)
        r[_r_TEMP_DEC_IB] = 25;
    
    if(r[_r_TEMP_DEC_IO] > 85)
        r[_r_TEMP_DEC_IO] = 85;
    if(r[_r_TEMP_DEC_IO] < 50)
        r[_r_TEMP_DEC_IO] = 50;
    
    if(r[_r_TEMP_ZERRO_IB] > 55)
        r[_r_TEMP_ZERRO_IB] = 55;
    if(r[_r_TEMP_ZERRO_IB] < 25)
        r[_r_TEMP_ZERRO_IB] = 25;
    
    if(r[_r_TEMP_ZERRO_IO] > 100)
        r[_r_TEMP_ZERRO_IO] = 100;
    if(r[_r_TEMP_ZERRO_IO] < 80)
        r[_r_TEMP_ZERRO_IO] = 80;
    
    if(r[_r_VIN_HIGH_ALARM] > 800)
        r[_r_VIN_HIGH_ALARM] = 800;
    if(r[_r_VIN_HIGH_ALARM] < 400)
        r[_r_VIN_HIGH_ALARM] = 400;
    
    if(r[_r_VIN_LOW_ALARM] > 500)
        r[_r_VIN_LOW_ALARM] = 500;
    if(r[_r_VIN_LOW_ALARM] < 200)
        r[_r_VIN_LOW_ALARM] = 200;
    
}

void Check_Phase_Sequence(void)
{
    // <editor-fold defaultstate="collapsed" desc="ONE CYCLE PROGRAM2">
    if( F_NO_LOSS == 1)
    {
        if(F_RIGHT == 0 && F_LEFT == 0)
        {
            if((CMP_Arrays(Phase_Sync_Counter,_0_1_2) == 1) || 
               (CMP_Arrays(Phase_Sync_Counter,_2_0_1) == 1) ||
               (CMP_Arrays(Phase_Sync_Counter,_1_2_0) == 1))
            {
                F_LEFT = 1;
                F_RIGHT = 0;
            }        
            else if((CMP_Arrays(Phase_Sync_Counter,_0_2_1) == 1) || 
                    (CMP_Arrays(Phase_Sync_Counter,_1_0_2) == 1) ||
                    (CMP_Arrays(Phase_Sync_Counter,_2_1_0) == 1))
            {
                F_RIGHT = 1;
                F_LEFT = 0;
            }
            else
            {
                F_RIGHT = 0;
                F_LEFT = 0;
            }
        }
    }
    
    /*if(Phase_Sync_Counter[0] > 3)
        F_L1_LOSS = 1;
    
    if(Phase_Sync_Counter[1] > 3)
        F_L2_LOSS = 1;
    
    if(Phase_Sync_Counter[2] > 3)
        F_L3_LOSS = 1; */ 
    
    if(T_INT0_INT1 < _T3_LOW || Phase_Sync_Counter[1] > 3/*|| T_INT0_INT1 > _T3_HIGH*/)
        F_L2_LOSS = 1;
    else
        F_L2_LOSS = 0;

    if(T_INT1_INT2 < _T3_LOW || Phase_Sync_Counter[2] > 3/*|| T_INT1_INT2 > _T3_HIGH*/)
        F_L3_LOSS = 1;
    else
        F_L3_LOSS = 0;

    if(T_INT2_INT0 < _T3_LOW || Phase_Sync_Counter[0] > 3/*|| T_INT2_INT0 > _T3_HIGH*/)
        F_L1_LOSS = 1;
    else
        F_L1_LOSS = 0;
    
    
    Word_Type[_U_TEST1] = T_INT0_INT1;
    Word_Type[_U_TEST2] = T_INT1_INT2;
    Word_Type[_U_TEST3] = T_INT2_INT0;
    Word_Type[_U_TEST4] = T_INT0_INT1 + T_INT1_INT2 + T_INT2_INT0;
    
    if((CMP_Arrays(Phase_Sync_Counter,_0_0_0) == 1))
    {
        F_L1_LOSS = 1;
        F_L2_LOSS = 1;
        F_L3_LOSS = 1;
    }
    
    if(1/*Flag_Type[_F_FAIL_ENABLE] == 1*/)
    {
        if(F_L1_LOSS == 1 || F_L2_LOSS == 1 || F_L3_LOSS == 1)
        {
            F_NO_LOSS = 0;
            F_LEFT = 0;
            F_RIGHT = 0;
            
        }
        else
            F_NO_LOSS = 1;
    }
    
    
    
    if(F_NO_LOSS == 1 && F_LEFT == 1)
    {
        FUNC_CONFIG_LEFT1 = 1;
    }
    else
        FUNC_CONFIG_LEFT1 = 0;
    
    
    
    if(F_NO_LOSS == 1 && F_RIGHT == 1)
    {
        FUNC_CONFIG_RIGHT1 = 1;
    }
    else
        FUNC_CONFIG_RIGHT1 = 0;
    
}

void modbusGet(void) 
{
	unsigned int requestedAdrress;
    if(UART_last_Byte_Sent_Flag == 1 && U1STAbits.TRMT == 1)
    {           
        //READY = 0;
        modbusReset();
        UART_last_Byte_Sent_Flag = 0;
        
    }
    if (modbusGetBusState() & (1<<ReceiveCompleted))
	{

        switch(rxbuffer[1]) {
			case fcReadCoilStatus: {
				modbusExchangeBits(&outstate,0,8);
			}
			break;
			
			case fcReadInputStatus: {
				volatile uint8_t inps = ReadIns();
				modbusExchangeBits(&inps,0,8);
			}
			break;
			
			case fcReadHoldingRegisters: {
                
                holdingRegisters[0] = r[_r_VO_SP]*10;
                holdingRegisters[1] = r[_r_VIN_LOW_ALARM]*10;
                holdingRegisters[2] = r[_r_IB_SP]*10;
                holdingRegisters[3] = r[_r_IO_SP]*10;
                holdingRegisters[4] = r[_r_TEMP_DEC_IB]*10;
                holdingRegisters[5] = r[_r_TEMP_DEC_IO]*10;
				modbusExchangeRegisters(holdingRegisters,0,20);
			}
			break;
			
			case fcReadInputRegisters: {
				inputRegisters[0] = Word_Type[_U_VO_PV];
                inputRegisters[1] = Word_Type[_U_VI_PV];
                inputRegisters[2] = Word_Type[_U_IB_PV];
                inputRegisters[3] = Word_Type[_U_IO_PV];
                inputRegisters[4] = Word_Type[_U_TB_PV];
                inputRegisters[5] = Word_Type[_U_TH_PV];
                modbusExchangeRegisters(inputRegisters,0,20);
			}
			break;
			
			case fcForceSingleCoil: {
				modbusExchangeBits(&outstate,0,8);
                requestedAdrress = modbusRequestedAddress();
                if((outstate & 0x01) == 0x01 && requestedAdrress == 0)
                {
                    Run_Function(_sAu);
                }
                if((outstate & 0x20) == 0x20 && requestedAdrress == 5)
                {
                    Run_Function(_rst);
                }
                if((outstate & 0x40) == 0x40 && requestedAdrress == 6)
                {
                    Run_Function(_Fst);
                }
                if((outstate & 0x02) == 0x02 && requestedAdrress == 1)
                {
                    Flag_Type[_F_VO_SP_SEL] = 1;
                    Flag_Type_Change_Flag[_F_VO_SP_SEL - _END_OF_EVENT_FLAG_ADDRESS] = 1;
                }
                else if((outstate & 0x02) == 0x00 && requestedAdrress == 1)
                {
                    Flag_Type[_F_VO_SP_SEL] = 0;
                    Flag_Type_Change_Flag[_F_VO_SP_SEL - _END_OF_EVENT_FLAG_ADDRESS] = 1;
                }
                if((outstate & 0x04) == 0x04 && requestedAdrress == 2)
                {
                    Flag_Type[_F_IB_SP_SEL] = 1;
                    Flag_Type_Change_Flag[_F_IB_SP_SEL - _END_OF_EVENT_FLAG_ADDRESS] = 1;
                }
                else if((outstate & 0x04) == 0x00 && requestedAdrress == 2)
                {
                    Flag_Type[_F_IB_SP_SEL] = 0;
                    Flag_Type_Change_Flag[_F_IB_SP_SEL - _END_OF_EVENT_FLAG_ADDRESS] = 1;
                }
                if((outstate & 0x08) == 0x08 && requestedAdrress == 3)
                {
                    Flag_Type[_F_IO_SP_SEL] = 1;
                    Flag_Type_Change_Flag[_F_IO_SP_SEL - _END_OF_EVENT_FLAG_ADDRESS] = 1;
                }
                else if((outstate & 0x08) == 0x00 && requestedAdrress == 3)
                {
                    Flag_Type[_F_IO_SP_SEL] = 0;
                    Flag_Type_Change_Flag[_F_IO_SP_SEL - _END_OF_EVENT_FLAG_ADDRESS] = 1;
                }
                if((outstate & 0x10) == 0x10 && requestedAdrress == 4)
                {
                    Flag_Type[_F_SAVE_FACTORY_RESET] = 1;
                    Flag_Type_Change_Flag[_F_SAVE_FACTORY_RESET - _END_OF_EVENT_FLAG_ADDRESS] = 1;
                }
                else if((outstate & 0x10) == 0x00 && requestedAdrress == 4)
                {
                    Flag_Type[_F_SAVE_FACTORY_RESET] = 0;
                    Flag_Type_Change_Flag[_F_SAVE_FACTORY_RESET - _END_OF_EVENT_FLAG_ADDRESS] = 1;
                }
                
				SetOuts(outstate);
			}
			break;
			
			case fcPresetSingleRegister: {
				modbusExchangeRegisters(holdingRegisters,0,20);
                r[_r_VO_SP] = (float)holdingRegisters[0]/10.0;
                r[_r_VIN_LOW_ALARM] = (float)holdingRegisters[1]/10.0;
                r[_r_IB_SP] = (float)holdingRegisters[2]/10.0;
                r[_r_IO_SP] = (float)holdingRegisters[3]/10.0;
                r[_r_TEMP_DEC_IB] = (float)holdingRegisters[4]/10.0;
                r[_r_TEMP_DEC_IO] = (float)holdingRegisters[5]/10.0;
                Real_Type[_r_VO_SP] = (r[_r_VO_SP] * 10) / 10;
                Float_Type[_r_VO_SP] = (int)(r[_r_VO_SP] * 10) % 10;
                Real_Type_Change_Flag[_r_VO_SP] = 1;
                Float_Type_Change_Flag[_r_VO_SP] = 1;
                Real_Type[_r_VIN_LOW_ALARM] = (r[_r_VIN_LOW_ALARM] * 10) / 10;
                Float_Type[_r_VIN_LOW_ALARM] = (int)(r[_r_VIN_LOW_ALARM] * 10) % 10;
                Real_Type_Change_Flag[_r_VIN_LOW_ALARM] = 1;
                Float_Type_Change_Flag[_r_VIN_LOW_ALARM] = 1;
                Real_Type[_r_IB_SP] = (r[_r_IB_SP] * 10) / 10;
                Float_Type[_r_IB_SP] = (int)(r[_r_IB_SP] * 10) % 10;
                Real_Type_Change_Flag[_r_IB_SP] = 1;
                Float_Type_Change_Flag[_r_IB_SP] = 1;
                Real_Type[_r_IO_SP] = (r[_r_IO_SP] * 10) / 10;
                Float_Type[_r_IO_SP] = (int)(r[_r_IO_SP] * 10) % 10;
                Real_Type_Change_Flag[_r_IO_SP] = 1;
                Float_Type_Change_Flag[_r_IO_SP] = 1;
                Real_Type[_r_TEMP_DEC_IB] = (r[_r_TEMP_DEC_IB] * 10) / 10;
                Float_Type[_r_TEMP_DEC_IB] = (int)(r[_r_TEMP_DEC_IB] * 10) % 10;
                Real_Type_Change_Flag[_r_TEMP_DEC_IB] = 1;
                Float_Type_Change_Flag[_r_TEMP_DEC_IB] = 1;
                Real_Type[_r_TEMP_DEC_IO] = (r[_r_TEMP_DEC_IO] * 10) / 10;
                Float_Type[_r_TEMP_DEC_IO] = (int)(r[_r_TEMP_DEC_IO] * 10) % 10;
                Real_Type_Change_Flag[_r_TEMP_DEC_IO] = 1;
                Float_Type_Change_Flag[_r_TEMP_DEC_IO] = 1;
			}
			break;
			
			case fcForceMultipleCoils: {
				modbusExchangeBits(&outstate,0,8);
				SetOuts(outstate);
			}
			break;
			
			case fcPresetMultipleRegisters: {
				modbusExchangeRegisters(holdingRegisters,0,4);
			}
			break;
			
			default: {
				modbusSendException(ecIllegalFunction);
			}
			break;
		}
	}
}

/*
*   Modify the following 3 functions to implement your own pin configurations...
*/
void SetOuts(volatile uint8_t in) 
{
	
}

uint8_t ReadIns(void) 
{
	uint8_t ins=0x31;
	
	return ins;
}