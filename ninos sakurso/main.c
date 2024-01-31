#define F_CPU 8000000UL
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

const uint8_t Set_Point = 127;

/* Global Variables for PID */
float d_Temp = 0;
float i_Temp = 0;
float PWM_Temp = 0;

uint8_t new_ADC_value;

float X = 100;//90
float Y = 90;//60
float Z = 50;//50


void InitADC()
{
	
	//and set left adjust result
	ADMUX |= (1<<REFS0)|(1<<ADLAR);
	ADCSRA |= (1<<ADPS2)|(1<<ADPS0)|(1<<ADATE)|(1<<ADIE)|(1<<ADEN);
	//set ADC trigger source - Timer0 compare match A
	ADCSRB |= (1<<ADTS1)|(1<<ADTS0);
}
void SetADCChannel(uint8_t ADCchannel)
{
	//select ADC channel with safety mask
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
}
void StartADC(void)
{
	ADCSRA |= (1<<ADSC);
}
//disable ADC
void DisableADC(void)
{
	ADCSRA &= ~((1<<ADEN)|(1<<ADIE));
}

void InitPort(void)
{
	//set PD6 and PD2 as output
	DDRD |= (1<<PIND2)|(1<<PIND3)|(1<<PIND4)|(1<<PIND5)|(1<<PIND6)|(1<<PIND7);
}
//initialize timer0
void InitTimer0(void)
{
	//Set Initial Timer value
	TCNT0=0;
	//Place TOP timer value to Output compare register
	OCR0A=99;
	TCCR0A |=(1<<COM0A0)|(1<<WGM01);
}

void StartTimer0(void)
{

	TCCR0B |=(1<<CS01);
}
void StopTimer0(void)
{
	TCCR0B &=~(1<<CS01);
	TIMSK0 &=~(1<<OCIE0A);
}

void pwmfunc(int dc)
{
	OCR2B = dc;
	TCCR2A |= (1<<COM2B1) | (1<<WGM21) | (1<<WGM20);
	TCCR2B |= (1<<CS21);
}

ISR(ADC_vect)
{
	//clear timer compare match flag
	TIFR0=(1<<OCF0A);
	//toggle pin PD2 to track the end of ADC conversion
	PIND = (1<<PIND2);
	new_ADC_value = ADCH;
}

float PID_foo(float *p)
{
	float Kp = p[0];
	float Ki = p[1];
	float Kd = p[2];
	float iMax = 100;
	float iMin = -100;
	float Err_Value;
	float P_Term;
	float I_Term;
	float D_Term;
	
	Err_Value = (Set_Point - new_ADC_value);
	P_Term = Kp * Err_Value;
	i_Temp += Err_Value;
	
	if (i_Temp > iMax)
	{i_Temp = iMax;}
	else if (i_Temp < iMin)
	{i_Temp = iMin;}
	
	I_Term = Ki * i_Temp;
	
	D_Term = Kd * (d_Temp - Err_Value);
	d_Temp = Err_Value;
	
	return (P_Term + I_Term + D_Term);	
}


int main()
{
	InitPort();

	InitTimer0();

	sei();
	
	while(1)
	{
		InitADC();
	
		SetADCChannel(0);

		StartTimer0();
		//start conversion
		StartADC();
		
		float coef[3] = {0, 0, 0};//{0, 0, 0};
		float dcoef[3] = {1, 0.5, 0.5};
		//float threshold = 0.001;
		
		float best_error = PID_foo(coef);	
		float error = 0;
		
		//while((dcoef[0] + dcoef[1] + dcoef[2]) > threshold)
		while(1)
		{
			for(int i =0; i < 3; i++)
			{
				coef[i] += dcoef[i];
				error = PID_foo(coef);
				if(error < best_error)
				{
					best_error = error;
					dcoef[i] *= 1.1; //1.1
				}
				else
				{
					coef[i] -= 0.1*dcoef[i];
					error = PID_foo(coef);
					if(error < best_error)
					{
						best_error = error;
						dcoef[i] *= 1.05;
					} 
					else
					{
						coef[i] += dcoef[i];
						dcoef[i] *= 0.95;
					}
				}
			}
				if((new_ADC_value >= (Set_Point + X)) && (new_ADC_value < 256))
				{
					PORTD = PORTD & 0x207;
					PORTD |= (0 << PIND4) | (1 << PIND5);
					//pwmfunc(PWM_Temp - PID_foo(coef));
					pwmfunc(110);
					//pwmfunc(0.013*PID_foo(coef));
				}
				if((new_ADC_value < (Set_Point - X)) && (new_ADC_value >= 0))
				{
					PORTD = PORTD & 0x207;
					PORTD |= (1 << PIND4) | (0 << PIND5);
					pwmfunc(110);
					//pwmfunc(0.5*PID_foo(coef));
				}
				if((new_ADC_value < (Set_Point + X)) && (new_ADC_value > (Set_Point + Y)))
				{
					PORTD = PORTD & 0x207;
					PORTD |= (0 << PIND4) | (1 << PIND5);
					pwmfunc(0.8*PID_foo(coef));//0.85
					
				}
				if((new_ADC_value > (Set_Point - X)) && (new_ADC_value < (Set_Point - Y)))
				{
					PORTD = PORTD & 0x207;
					PORTD |= (1 << PIND4) | (0 << PIND5);
					//pwmfunc(15*PWM_Temp - 15*PID_foo(coef));
					pwmfunc(0.8*PID_foo(coef));//0.85
				}
				if ((new_ADC_value < (Set_Point + Z)) && (new_ADC_value > (Set_Point - Z)))
				{
					PORTD = PORTD & 0x207;
					PORTD |= (0 << PIND4) | (0 << PIND5);
					pwmfunc(0);
				}
			
		}
	}
}