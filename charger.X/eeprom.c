/* 
 * File:   eeprom.c
 * Author: Ehsan
 *
 * Created on May 6, 2016, 11:54 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include "eeprom.h"
#include "defines.h"

/*
 * 
 */
int _EEDATA(2) EEPROM_DAM = 0;
int _EEDATA(2) Lbt_ADRESS_WORD = 0;
int _EEDATA(2) Lbt_ADRESS_REAL = 0;
int _EEDATA(2) Hbt_ADRESS_WORD = 0;
int _EEDATA(2) Hbt_ADRESS_REAL = 0;
int _EEDATA(2) LOU_ADRESS_WORD = 0;
int _EEDATA(2) HOU_ADRESS_WORD = 0;
int _EEDATA(2) HOU_ADRESS_REAL = 0;
int _EEDATA(2) LIU_ADRESS_WORD = 0;
int _EEDATA(2) HIU_ADRESS_WORD = 0;
int _EEDATA(2) HIU_ADRESS_REAL = 0;
int _EEDATA(2) LbC_ADRESS_WORD = 0;
int _EEDATA(2) HbC_ADRESS_WORD = 0;
int _EEDATA(2) HbC_ADRESS_REAL = 0;
int _EEDATA(2) LLC_ADRESS_WORD = 0;
int _EEDATA(2) HLC_ADRESS_WORD = 0;
int _EEDATA(2) HLC_ADRESS_REAL = 0;
int _EEDATA(2) LHt_ADRESS_WORD = 0;
int _EEDATA(2) LHt_ADRESS_REAL = 0;
int _EEDATA(2) HHt_ADRESS_WORD = 0;
int _EEDATA(2) HHt_ADRESS_REAL = 0;
int _EEDATA(2) HMt_ADRESS_WORD = 0;
int _EEDATA(2) HMt_ADRESS_REAL = 0;
int _EEDATA(2) POINTER_OF_LAST_EVENT = 0;
int _EEDATA(2) LUT_ADRESS[255];
int _EEDATA(2) Que_Alarm_ADRESS[16];
int _EEDATA(2) Byte_Type_ADRESS[_END_OF_RW_BYTE_ADDRESS];
int _EEDATA(2) Word_Type_ADRESS[_END_OF_RW_WORD_ADDRESS];
int _EEDATA(2) Real_Type_ADRESS[_END_OF_RW_FLOAT_ADDRESS];
int _EEDATA(2) Float_Type_ADRESS[_END_OF_RW_FLOAT_ADDRESS];
int _EEDATA(2) Flag_Type_ADRESS[_END_OF_RW_FLAG_ADDRESS2];

_prog_addressT tempLoc;

//the functions which actually touch the EEPROM
void write_eeprom(_prog_addressT location, int value) {
	_erase_eedata(location, _EE_WORD);
	_wait_eedata();
	_write_eedata_word(location, value);	
	_wait_eedata();
}

int read_eeprom(_prog_addressT location) {
	int temp;
	_memcpy_p2d16(&temp, location, 2);
	return temp;
}

int ee_get_DAM(void) {
	_init_prog_address(tempLoc,EEPROM_DAM);
	return(read_eeprom(tempLoc));
}

void ee_set_DAM(int newval) {
	_init_prog_address(tempLoc,EEPROM_DAM);
	write_eeprom(tempLoc, newval);
}

int ee_get_Lbt_ADRESS_WORD(void) {
	_init_prog_address(tempLoc,Lbt_ADRESS_WORD);
	return(read_eeprom(tempLoc));
}

void ee_set_Lbt_ADRESS_WORD(int newval) {
	_init_prog_address(tempLoc,Lbt_ADRESS_WORD);
	write_eeprom(tempLoc, newval);
}

int ee_get_Lbt_ADRESS_REAL(void) {
	_init_prog_address(tempLoc,Lbt_ADRESS_REAL);
	return(read_eeprom(tempLoc));
}

void ee_set_Lbt_ADRESS_REAL(int newval) {
	_init_prog_address(tempLoc,Lbt_ADRESS_REAL);
	write_eeprom(tempLoc, newval);
}

int ee_get_Hbt_ADRESS_WORD(void) {
	_init_prog_address(tempLoc,Hbt_ADRESS_WORD);
	return(read_eeprom(tempLoc));
}

void ee_set_Hbt_ADRESS_WORD(int newval) {
	_init_prog_address(tempLoc,Hbt_ADRESS_WORD);
	write_eeprom(tempLoc, newval);
}

int ee_get_Hbt_ADRESS_REAL(void) {
	_init_prog_address(tempLoc,Hbt_ADRESS_REAL);
	return(read_eeprom(tempLoc));
}

void ee_set_Hbt_ADRESS_REAL(int newval) {
	_init_prog_address(tempLoc,Hbt_ADRESS_REAL);
	write_eeprom(tempLoc, newval);
}

int ee_get_LOU_ADRESS_WORD(void) {
	_init_prog_address(tempLoc,LOU_ADRESS_WORD);
	return(read_eeprom(tempLoc));
}

void ee_set_LOU_ADRESS_WORD(int newval) {
	_init_prog_address(tempLoc,LOU_ADRESS_WORD);
	write_eeprom(tempLoc, newval);
}

int ee_get_HOU_ADRESS_WORD(void) {
	_init_prog_address(tempLoc,HOU_ADRESS_WORD);
	return(read_eeprom(tempLoc));
}

void ee_set_HOU_ADRESS_WORD(int newval) {
	_init_prog_address(tempLoc,HOU_ADRESS_WORD);
	write_eeprom(tempLoc, newval);
}

int ee_get_HOU_ADRESS_REAL(void) {
	_init_prog_address(tempLoc,HOU_ADRESS_REAL);
	return(read_eeprom(tempLoc));
}

void ee_set_HOU_ADRESS_REAL(int newval) {
	_init_prog_address(tempLoc,HOU_ADRESS_REAL);
	write_eeprom(tempLoc, newval);
}

int ee_get_LIU_ADRESS_WORD(void) {
	_init_prog_address(tempLoc,LIU_ADRESS_WORD);
	return(read_eeprom(tempLoc));
}

void ee_set_LIU_ADRESS_WORD(int newval) {
	_init_prog_address(tempLoc,LIU_ADRESS_WORD);
	write_eeprom(tempLoc, newval);
}

int ee_get_HIU_ADRESS_WORD(void) {
	_init_prog_address(tempLoc,HIU_ADRESS_WORD);
	return(read_eeprom(tempLoc));
}

void ee_set_HIU_ADRESS_WORD(int newval) {
	_init_prog_address(tempLoc,HIU_ADRESS_WORD);
	write_eeprom(tempLoc, newval);
}

int ee_get_HIU_ADRESS_REAL(void) {
	_init_prog_address(tempLoc,HIU_ADRESS_REAL);
	return(read_eeprom(tempLoc));
}

void ee_set_HIU_ADRESS_REAL(int newval) {
	_init_prog_address(tempLoc,HIU_ADRESS_REAL);
	write_eeprom(tempLoc, newval);
}

int ee_get_LbC_ADRESS_WORD(void) {
	_init_prog_address(tempLoc,LbC_ADRESS_WORD);
	return(read_eeprom(tempLoc));
}

void ee_set_LbC_ADRESS_WORD(int newval) {
	_init_prog_address(tempLoc,LbC_ADRESS_WORD);
	write_eeprom(tempLoc, newval);
}

int ee_get_HbC_ADRESS_WORD(void) {
	_init_prog_address(tempLoc,HbC_ADRESS_WORD);
	return(read_eeprom(tempLoc));
}

void ee_set_HbC_ADRESS_WORD(int newval) {
	_init_prog_address(tempLoc,HbC_ADRESS_WORD);
	write_eeprom(tempLoc, newval);
}

int ee_get_HbC_ADRESS_REAL(void) {
	_init_prog_address(tempLoc,HbC_ADRESS_REAL);
	return(read_eeprom(tempLoc));
}

void ee_set_HbC_ADRESS_REAL(int newval) {
	_init_prog_address(tempLoc,HbC_ADRESS_REAL);
	write_eeprom(tempLoc, newval);
}

int ee_get_LLC_ADRESS_WORD(void) {
	_init_prog_address(tempLoc,LLC_ADRESS_WORD);
	return(read_eeprom(tempLoc));
}

void ee_set_LLC_ADRESS_WORD(int newval) {
	_init_prog_address(tempLoc,LLC_ADRESS_WORD);
	write_eeprom(tempLoc, newval);
}

int ee_get_HLC_ADRESS_WORD(void) {
	_init_prog_address(tempLoc,HLC_ADRESS_WORD);
	return(read_eeprom(tempLoc));
}

void ee_set_HLC_ADRESS_WORD(int newval) {
	_init_prog_address(tempLoc,HLC_ADRESS_WORD);
	write_eeprom(tempLoc, newval);
}

int ee_get_HLC_ADRESS_REAL(void) {
	_init_prog_address(tempLoc,HLC_ADRESS_REAL);
	return(read_eeprom(tempLoc));
}

void ee_set_HLC_ADRESS_REAL(int newval) {
	_init_prog_address(tempLoc,HLC_ADRESS_REAL);
	write_eeprom(tempLoc, newval);
}

int ee_get_LHt_ADRESS_WORD(void) {
	_init_prog_address(tempLoc,LHt_ADRESS_WORD);
	return(read_eeprom(tempLoc));
}

void ee_set_LHt_ADRESS_WORD(int newval) {
	_init_prog_address(tempLoc,LHt_ADRESS_WORD);
	write_eeprom(tempLoc, newval);
}

int ee_get_LHt_ADRESS_REAL(void) {
	_init_prog_address(tempLoc,LHt_ADRESS_REAL);
	return(read_eeprom(tempLoc));
}

void ee_set_LHt_ADRESS_REAL(int newval) {
	_init_prog_address(tempLoc,LHt_ADRESS_REAL);
	write_eeprom(tempLoc, newval);
}

int ee_get_HHt_ADRESS_WORD(void) {
	_init_prog_address(tempLoc,HHt_ADRESS_WORD);
	return(read_eeprom(tempLoc));
}

void ee_set_HHt_ADRESS_WORD(int newval) {
	_init_prog_address(tempLoc,HHt_ADRESS_WORD);
	write_eeprom(tempLoc, newval);
}

int ee_get_HHt_ADRESS_REAL(void) {
	_init_prog_address(tempLoc,HHt_ADRESS_REAL);
	return(read_eeprom(tempLoc));
}

void ee_set_HHt_ADRESS_REAL(int newval) {
	_init_prog_address(tempLoc,HHt_ADRESS_REAL);
	write_eeprom(tempLoc, newval);
}

int ee_get_HMt_ADRESS_WORD(void) {
	_init_prog_address(tempLoc,HMt_ADRESS_WORD);
	return(read_eeprom(tempLoc));
}

void ee_set_HMt_ADRESS_WORD(int newval) {
	_init_prog_address(tempLoc,HMt_ADRESS_WORD);
	write_eeprom(tempLoc, newval);
}

int ee_get_HMt_ADRESS_REAL(void) {
	_init_prog_address(tempLoc,HMt_ADRESS_REAL);
	return(read_eeprom(tempLoc));
}

void ee_set_HMt_ADRESS_REAL(int newval) {
	_init_prog_address(tempLoc,HMt_ADRESS_REAL);
	write_eeprom(tempLoc, newval);
}

int ee_get_LUT_ADRESS(int i) {
	_init_prog_address(tempLoc,LUT_ADRESS);
	return(read_eeprom(tempLoc+2*i));
}

void ee_set_LUT_ADRESS(int newval, int i) {
	_init_prog_address(tempLoc,LUT_ADRESS);
	write_eeprom(tempLoc+2*i, newval);
}

int ee_get_Que_Alarm_ADRESS(int i) {
	_init_prog_address(tempLoc,Que_Alarm_ADRESS);
	return(read_eeprom(tempLoc+2*i));
}

void ee_set_Que_Alarm_ADRESS(int newval, int i) {
	_init_prog_address(tempLoc,Que_Alarm_ADRESS);
	write_eeprom(tempLoc+2*i, newval);
}

int ee_get_Byte_Type_ADRESS(int i) {
	_init_prog_address(tempLoc,Byte_Type_ADRESS);
	return(read_eeprom(tempLoc+2*i));
}

void ee_set_Byte_Type_ADRESS(int newval, int i) {
	_init_prog_address(tempLoc,Byte_Type_ADRESS);
	write_eeprom(tempLoc+2*i, newval);
}

int ee_get_Word_Type_ADRESS(int i) {
	_init_prog_address(tempLoc,Word_Type_ADRESS);
	return(read_eeprom(tempLoc+2*i));
}

void ee_set_Word_Type_ADRESS(int newval, int i) {
	_init_prog_address(tempLoc,Word_Type_ADRESS);
	write_eeprom(tempLoc+2*i, newval);
}

int ee_get_Real_Type_ADRESS(int i) {
	_init_prog_address(tempLoc,Real_Type_ADRESS);
	return(read_eeprom(tempLoc+2*i));
}

void ee_set_Real_Type_ADRESS(int newval, int i) {
	_init_prog_address(tempLoc,Real_Type_ADRESS);
	write_eeprom(tempLoc+2*i, newval);
}

int ee_get_Float_Type_ADRESS(int i) {
	_init_prog_address(tempLoc,Float_Type_ADRESS);
	return(read_eeprom(tempLoc+2*i));
}

void ee_set_Float_Type_ADRESS(int newval, int i) {
	_init_prog_address(tempLoc,Float_Type_ADRESS);
	write_eeprom(tempLoc+2*i, newval);
}

int ee_get_Flag_Type_ADRESS(int i) {
	_init_prog_address(tempLoc,Flag_Type_ADRESS);
	return(read_eeprom(tempLoc+2*i));
}

void ee_set_Flag_Type_ADRESS(int newval, int i) {
	_init_prog_address(tempLoc,Flag_Type_ADRESS);
	write_eeprom(tempLoc+2*i, newval);
}

int ee_get_POINTER_OF_LAST_EVENT(void) {
	_init_prog_address(tempLoc,POINTER_OF_LAST_EVENT);
	return(read_eeprom(tempLoc));
}

void ee_set_POINTER_OF_LAST_EVENT(int newval) {
	_init_prog_address(tempLoc,POINTER_OF_LAST_EVENT);
	write_eeprom(tempLoc, newval);
}



