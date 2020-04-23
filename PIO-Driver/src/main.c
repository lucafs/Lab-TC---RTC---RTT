/**
 * 5 semestre - Eng. da Computação - Insper
 * Rafael Corsi - rafael.corsi@insper.edu.br
 *
 * Projeto 0 para a placa SAME70-XPLD
 *
 * Objetivo :
 *  - Introduzir ASF e HAL
 *  - Configuracao de clock
 *  - Configuracao pino In/Out
 *
 * Material :
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 */

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

	#define LED_PIO           PIOC                 // periferico que controla o LED
	#define LED_PIO_ID        12                  // ID do periférico PIOC (controla LED)
	#define LED_PIO_IDX       8                    // ID do LED no PIO
	#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)   // Mascara para CONTROLARMOS o LED

	// Configuracoes do botao
	#define BUT_PIO			  PIOA
	#define BUT_PIO_ID		  10
	#define BUT_PIO_IDX		  11
	#define BUT_PIO_IDX_MASK (1u << BUT_PIO_IDX)
	
	
	/*  Default pin configuration (no attribute). */
	#define _PIO_DEFAULT             (0u << 0)
	/*  The internal pin pull-up is active. */
	#define _PIO_PULLUP              (1u << 0)
	/*  The internal glitch filter is active. */
	#define _PIO_DEGLITCH            (1u << 1)
	/*  The internal debouncing filter is active. */
	#define _PIO_DEBOUNCE            (1u << 3)

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);
void _pio_set(Pio *p_pio, const uint32_t ul_mask);
void _pio_clear(Pio *p_pio, const uint32_t ul_mask);
void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask,
void _pio_set_input(Pio *p_pio, const uint32_t ul_mask,
const uint32_t ul_attribute);

const uint32_t ul_pull_up_enable);
/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

// Função de inicialização do uC e config correta dos perifericos e pinos
void init(void)
{
	//inicializa board clock
	sysclk_init();
	
	//desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	//Ativa o PIO na qual o LED foi conectado
	// para que possamos controlar o LED
	pmc_enable_periph_clk(LED_PIO_ID);
	
	//Inicializa o PC8 como saida
	pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);
	
	// Inicializa PIO do botao
	pmc_enable_periph_clk(BUT_PIO_ID);
	
	// configura pino ligado ao botão como entrada com um pull-up.
//	pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK, PIO_DEFAULT);
	
	//Ativar o pull-up
//	_pio_pull_up(BUT_PIO, BUT_PIO_IDX_MASK, 1);
	
	_pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);

	


}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

void _pio_set(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio ->PIO_SODR = ul_mask;
}

void _pio_clear(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio -> PIO_CODR = ul_mask;
}

void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask,
const uint32_t ul_pull_up_enable)
{
	if (ul_pull_up_enable == 1){
		p_pio -> PIO_PUER = ul_mask;
	}
	else {
		p_pio -> PIO_PUDR = ul_mask;
	}
	
}

void _pio_set_input(Pio *p_pio, const uint32_t ul_mask,
const uint32_t ul_attribute)
{
	p_pio -> PIO_ODR = ul_mask;
	
	_pio_pull_up(p_pio, ul_mask, ul_attribute & _PIO_PULLUP);
	
// 	if(ul_attribute & _PIO_PULLUP)
// 		_pio_pull_up(p_pio, ul_mask, 1);
// 		else
// 		_pio_pull_up(p_pio, ul_mask, 0);

	if(ul_attribute & (_PIO_DEBOUNCE | _PIO_DEGLITCH))
		p_pio -> PIO_IFER = ul_mask; //enable
	else p_pio -> PIO_IFDR = ul_mask; //disable
		//checar se tem debouncing e ativa
	
	
	//checa se tem glitch e ativa

}	


// Funcao principal chamada na inicalizacao do uC.
int main(void)
{
	// inicializa sistema e IOs
	init();

	// super loop
	// aplicacoes embarcadas não devem sair do while(1).
	
	while(1){
	//botao apertdado retorna zero
	if (pio_get(BUT_PIO, PIO_INPUT, BUT_PIO_IDX_MASK)==0)
	{
		int i = 0;
		while (i<5)
		{
			_pio_set(PIOC, LED_PIO_IDX_MASK);      // Coloca 1 no pino LED
			delay_ms(200);                        // Delay por software de 200 ms
			_pio_clear(PIOC, LED_PIO_IDX_MASK);    // Coloca 0 no pino do LED
			delay_ms(200);                        // Delay por software de 200 ms
			i++;
		}
	}
	
	}
	
	
	
	return 0;
}
