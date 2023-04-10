/*
 * rtc.c
 *
 *  Created on: 6 Apr 2023
 *      Author: shiba
 */

#include "rtc_ds1307.h"
#include "stdio.h"

#define SYSTICK_TIM_CLK		16000000UL

void date_to_string(RTC_Date_t *pDate);
void time_to_string(RTC_Time_t *pTime);
char *get_day_of_week(uint8_t i);
extern void initialise_monitor_handles(void);

char date[10];
char time[10];

void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz) - 1;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value into SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}

int main(void)
{
	initialise_monitor_handles();

	init_systick_timer(1);

	while(1);

//	RTC_Time_t current_time;
//	RTC_Date_t current_date;
//
//	printf("RTC test\n");
//	if(ds1307_init())
//	{
//		printf("RTC init has failed\n");
//		while(1);
//	}
//
//	current_time.seconds = 30;
//	current_time.minutes = 38;
//	current_time.hours = 4;
//	current_time.time_format = TIME_FORMAT_12HRS_PM;
//
//	current_date.date = THURSDAY;
//	current_date.day = 6;
//	current_date.month = 4;
//	current_date.year = 21;
//
//	ds1307_set_current_date(&current_date);
//	ds1307_set_current_time(&current_time);
//
//	ds1307_get_current_time(&current_time);
//	ds1307_get_current_date(&current_date);
//
//	char* am_pm;
//
//	if(current_time.time_format != TIME_FORMAT_24HRS)
//	{
//		am_pm = (current_time.time_format) ? "PM" : "AM";
//		time_to_string(&current_time);
//		printf("Current time = %s %s\n", time, am_pm);
//	} else
//	{
//		time_to_string(&current_time);
//		printf("Current time = %s\n", time);
//	}
//	date_to_string(&current_date);
//	printf("Current date = %s <%s> \n", date, get_day_of_week(current_date.date));


	return 0;
}

void time_to_string(RTC_Time_t *pTime)
{
	sprintf(date, "%d:%d:%d", pTime->hours, pTime->minutes, pTime->seconds);
}

void date_to_string(RTC_Date_t *pDate)
{
	sprintf(time, "%d/%d/%d", pDate->day, pDate->month, pDate->year);
}

char *get_day_of_week(uint8_t i)
{
	char *days[] = {"SUNDAY", "MONDAY", "TUSEDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "STATURDAY"};
	return days[i - 1];
}

void SysTick_Handler(void){
	printf("Come\n");
}
