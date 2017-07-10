/* 
 * File:   eeprom.h
 * Author: Ehsan
 *
 * Created on May 6, 2016, 11:55 AM
 */


#ifndef EEPROM_H
#define	EEPROM_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>
#include <p30fxxxx.h>
#include <libpic30.h>
    
void write_eeprom(_prog_addressT location, int value);
int read_eeprom(_prog_addressT location);
void ee_set_DAM(int newval);
int ee_get_DAM(void);
int ee_get_Lbt_ADRESS_WORD(void);
void ee_set_Lbt_ADRESS_WORD(int newval);
int ee_get_Lbt_ADRESS_REAL(void);
void ee_set_Lbt_ADRESS_REAL(int newval);
int ee_get_Hbt_ADRESS_WORD(void);
void ee_set_Hbt_ADRESS_WORD(int newval);
int ee_get_Hbt_ADRESS_REAL(void);
void ee_set_Hbt_ADRESS_REAL(int newval);
int ee_get_LOU_ADRESS_WORD(void);
void ee_set_LOU_ADRESS_WORD(int newval);
int ee_get_HOU_ADRESS_WORD(void);
void ee_set_HOU_ADRESS_WORD(int newval);
int ee_get_HOU_ADRESS_REAL(void);
void ee_set_HOU_ADRESS_REAL(int newval);
int ee_get_LIU_ADRESS_WORD(void);
void ee_set_LIU_ADRESS_WORD(int newval);
int ee_get_HIU_ADRESS_WORD(void);
void ee_set_HIU_ADRESS_WORD(int newval);
int ee_get_HIU_ADRESS_REAL(void);
void ee_set_HIU_ADRESS_REAL(int newval);
int ee_get_LbC_ADRESS_WORD(void);
void ee_set_LbC_ADRESS_WORD(int newval);
int ee_get_HbC_ADRESS_WORD(void);
void ee_set_HbC_ADRESS_WORD(int newval);
int ee_get_HbC_ADRESS_REAL(void);
void ee_set_HbC_ADRESS_REAL(int newval);
int ee_get_LLC_ADRESS_WORD(void);
void ee_set_LLC_ADRESS_WORD(int newval);
int ee_get_HLC_ADRESS_WORD(void);
void ee_set_HLC_ADRESS_WORD(int newval);
int ee_get_HLC_ADRESS_REAL(void);
void ee_set_HLC_ADRESS_REAL(int newval);
int ee_get_LHt_ADRESS_WORD(void);
void ee_set_LHt_ADRESS_WORD(int newval);
int ee_get_LHt_ADRESS_REAL(void);
void ee_set_LHt_ADRESS_REAL(int newval);
int ee_get_HHt_ADRESS_WORD(void);
void ee_set_HHt_ADRESS_WORD(int newval);
int ee_get_HHt_ADRESS_REAL(void);
void ee_set_HHt_ADRESS_REAL(int newval);
int ee_get_HMt_ADRESS_WORD(void);
void ee_set_HMt_ADRESS_WORD(int newval);
int ee_get_HMt_ADRESS_REAL(void);
void ee_set_HMt_ADRESS_REAL(int newval);
int ee_get_LUT_ADRESS(int i);
void ee_set_LUT_ADRESS(int newval, int i);
int ee_get_Que_Alarm_ADRESS(int i);
void ee_set_Que_Alarm_ADRESS(int newval, int i);
int ee_get_Byte_Type_ADRESS(int i);
void ee_set_Byte_Type_ADRESS(int newval, int i);
int ee_get_Word_Type_ADRESS(int i);
void ee_set_Word_Type_ADRESS(int newval, int i);
int ee_get_Real_Type_ADRESS(int i);
void ee_set_Real_Type_ADRESS(int newval, int i);
int ee_get_Float_Type_ADRESS(int i);
void ee_set_Float_Type_ADRESS(int newval, int i);
int ee_get_Flag_Type_ADRESS(int i);
void ee_set_Flag_Type_ADRESS(int newval, int i);
int ee_get_POINTER_OF_LAST_EVENT(void);
void ee_set_POINTER_OF_LAST_EVENT(int newval);




#ifdef	__cplusplus
}
#endif

#endif	/* EEPROM_H */

