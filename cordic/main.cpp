#include <stdint.h>
#include "serial.h"
#include "../include/STM32G491xx.h"
void delay(uint32_t dly)
{
    while(dly--);
}
void setup()
{
    GPIOA->MODER &= ~(1 << 1); // Make bit 0 an output
    GPIOA->MODER |= (1 << 0);
}

void initDACS()
{
    // DACs are on PINS 9 (PA4) and 10 (PA5) (LQFP32)
    // These are DAC1_OUT1 and DAC1_OUT2 resp.
    RCC->AHB2ENR |= (1 << 16); // enable DAC1
    RCC->AHB2ENR |= (1 << 0); // enable Port A    
    DAC1->DAC_CR |= (1 << 16) + (1 << 0); // enable both outputs;
    DAC1->DAC_DHR12R1 = 0;
    DAC1->DAC_DHR12R2 = 0;
    // Now configure PA4 and PA5 as analog ouputs
    GPIOA->MODER |= ((1 << 8) + (1 << 9) + (1 << 10) + (1 << 11));
}
class cordic {
public:

	cordic(){};
	void begin(void);
	void set_arg_size(int size);
	int get_arg_size(void);
	void set_n_arg(int N);
	int get_n_arg(void);
	
	void set_res_size(int size);
	int get_res_size(void);
	void set_n_res(int N);
	int get_n_res(void);
	
	void set_scale(unsigned int scale);
	unsigned int get_scale(void);
	
	void set_precision(unsigned int precision);
	unsigned int get_precision(void);
	
	void set_function(uint32_t function);
	uint32_t get_function(void);
	
	void get_results(int32_t *Results);
	
	void write_args(int32_t *args);

	
	static const uint32_t cos=0;
	static const uint32_t sin=1;
	static const uint32_t phase=2;
	static const uint32_t mod=3;
	static const uint32_t atan=4;
	static const uint32_t cosh=5;
	static const uint32_t sinh=6;
	static const uint32_t atanh=7;
	static const uint32_t ln=8;
	static const uint32_t sqrt=9;

};
void cordic::begin(void)
{
	RCC->AHB1ENR |= (1 << 3); // turn on the clock for the CORDIC subsystem
}

void cordic::set_arg_size(int size)
{
	if (size == 16)
	{
		CORDIC->CSR |= (1 << 22);
	}
	if (size == 32)
	{
		CORDIC->CSR &= ~(1 << 22);
	}
}
int cordic::get_arg_size(void)
{
	if (CORDIC->CSR & (1<<22))
	{
		return 16;
	}
	else
	{
		return 32;
	}
}
void cordic::set_n_arg(int N)
{
	if (N == 1)
	{
		CORDIC->CSR &=~(1 << 20);
	}
	if (N == 2)
	{
		CORDIC->CSR |=(1 << 20);
	}
}
int cordic::get_n_arg(void)
{
	if (CORDIC->CSR & (1 << 20))
	{
		return 2;
	}
	else
	{
		return 1;
	}
}
void cordic::set_res_size(int size)
{
	if (size == 16)
	{
		CORDIC->CSR |= (1 << 21);
	}
	if (size == 32)
	{
		CORDIC->CSR &= ~(1 << 21);
	}	
}

int cordic::get_res_size(void)
{
	if (CORDIC->CSR & (1<<21))
	{
		return 16;
	}
	else
	{
		return 32;
	}
}
void cordic::set_n_res(int N)
{
	if (N == 1)
	{
		CORDIC->CSR &=~(1 << 19);
	}
	if (N == 2)
	{
		CORDIC->CSR |=(1 << 19);
	}
}
int cordic::get_n_res(void)
{
	if (CORDIC->CSR & (1 << 19))
	{
		return 2;
	}
	else
	{
		return 1;
	}
}
void cordic::set_scale(unsigned int scale)
{
	CORDIC->CSR = (CORDIC->CSR & ~((1<<10)+(1<<9)+(1<<8)) );
	CORDIC->CSR = (CORDIC->CSR | (scale << 8));	
}
unsigned int cordic::get_scale(void)
{
	int scale = (CORDIC->CSR & ((1<<10)+(1<<9)+(1<<8)) );
	scale = scale >> 8;
	return scale;
}
void cordic::set_precision(unsigned int precision)
{
	CORDIC->CSR = (CORDIC->CSR & ~((1<<7)+(1<<6)+(1<<5)+(1<<4)) );
	CORDIC->CSR = (CORDIC->CSR | (precision << 4));	
}
unsigned int cordic::get_precision(void)
{
	unsigned int precision = (CORDIC->CSR & ((1<<7)+(1<<6)+(1<<5)+(1<<4)) );
	precision = precision >> 4;
	return precision;
}
void cordic::set_function(uint32_t function)
{
	CORDIC->CSR &=~(0x0f); // clear out lower bits
	CORDIC->CSR |= function; // set the appropriate ones
}
uint32_t cordic::get_function(void)
{
	return CORDIC->CSR & 0x0f;
}

void cordic::get_results(int32_t *Results)
{	
	if (this->get_res_size()==32)
	{
		Results[0]= CORDIC->RDATA;
		if (this->get_n_res()==2)	
		{
			Results[1]= CORDIC->RDATA;
		}
	}
	else
	{
		uint32_t result=CORDIC->RDATA;
		Results[0]=result & 0xffff;
		if (this->get_n_res()==2)
		{
			Results[1]=result >> 16;
		}
	}
}
	
void cordic::write_args(int32_t *arg)
{
	if (this->get_res_size()==32)
	{
		CORDIC->WDATA = arg[0];
		if (this->get_n_arg()==2)	
		{
			CORDIC->WDATA = arg[1];
		}
	}
	else
	{
		
		uint32_t arg1=arg[0] & 0xffff;
		
		if (this->get_n_arg()==2)
		{
			uint32_t arg2=arg[1];
			CORDIC->WDATA = (arg2 << 16)  + arg1;
		}
		else
		{
			CORDIC->WDATA = (uint32_t)arg;
		}	
	}
}

serial Serial;
cordic Cordic;
int32_t args[2];
int32_t results[2];

int main()
{
    char c;
	int16_t s_int=0;		
	unsigned output = 0;    
    setup();  
	initDACS();
    Serial.begin();
	Cordic.begin();
	Cordic.set_scale(0);
	Cordic.set_precision(5);
	Cordic.set_n_arg(2);
	Cordic.set_n_res(2);
	Cordic.set_arg_size(32);
	Cordic.set_res_size(32);
	Cordic.set_function(Cordic.sin);
	args[0]=0;
	args[1]=0x7fffff00;
    enable_interrupts();
    Serial.print("Cordic signal generator\r\n");
	
	while(1)
    {        
		s_int++;
		output = (s_int-(int16_t)0x8000); 
		args[0]=output<<16;
		Cordic.write_args(args);
		Cordic.get_results(results);		
        DAC1->DAC_DHR12R1 = (results[0]+0x80000000)>>20;
        DAC1->DAC_DHR12R2 = output>>4;
    }    
}
