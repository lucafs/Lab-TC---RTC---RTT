/************************************************************************
 * 5 semestre - Eng. da Computao - Insper
 * Rafael Corsi - rafael.corsi@insper.edu.br
 *
 * Material:
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 *
 * Objetivo:
 *  - Demonstrar interrupção do PIO
 *
 * Periféricos:
 *  - PIO
 *  - PMC
 *
 * Log:
 *  - 10/2018: Criação
 ************************************************************************/

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

// LED
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

// Botão
#define BUT_PIO      PIOA
#define BUT_PIO_ID   ID_PIOA
#define BUT_IDX  11
#define BUT_IDX_MASK (1 << BUT_IDX)

#define BUT1_PIO            PIOD
#define BUT1_PIO_ID        16
#define BUT1_PIO_IDX        28
#define BUT1_PIO_IDX_MASK    (1u << BUT1_PIO_IDX)

#define BUT2_PIO            PIOC
#define BUT2_PIO_ID        12
#define BUT2_PIO_IDX        31
#define BUT2_PIO_IDX_MASK    (1u << BUT2_PIO_IDX)

#define BUT3_PIO            PIOA
#define BUT3_PIO_ID        10
#define BUT3_PIO_IDX        19
#define BUT3_PIO_IDX_MASK    (1u << BUT3_PIO_IDX)

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* prototype                                                            */
/************************************************************************/
void io_init(void);
void pisca_led(int n, int t);
volatile char but_flag;


/************************************************************************/
/* handler / callbacks                                                  */
/************************************************************************/

/*
 * Exemplo de callback para o botao, sempre que acontecer
 * ira piscar o led por 5 vezes
 *
 * !! Isso é um exemplo ruim, nao deve ser feito na pratica, !!
 * !! pois nao se deve usar delays dentro de interrupcoes    !!
 */
void but_callback(void)
{
	 but_flag = 1;
}

/************************************************************************/
/* funções                                                              */
/************************************************************************/

// pisca led N vez no periodo T
void pisca_led(int n, int t){
  for (int i=0;i<n;i++){
    pio_clear(LED_PIO, LED_IDX_MASK);
    delay_ms(t);
    pio_set(LED_PIO, LED_IDX_MASK);
    delay_ms(t);
  }
}

// Inicializa botao SW0 do kit com interrupcao
void io_init(void)
{
	//Inicializa o PC8 como saida
// 	pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);
// 	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 0, 0, 0);
// 	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 0, 0, 0);
// 	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 0, 0, 0);
  // Configura led
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);

  // Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT_PIO_ID);

  // Configura PIO para lidar com o pino do botão como entrada
  // com pull-up
	pio_configure(BUT_PIO, PIO_INPUT, BUT_IDX_MASK, PIO_PULLUP);

  // Configura interrupção no pino referente ao botao e associa
  // função de callback caso uma interrupção for gerada
  // a função de callback é a: but_callback()
  pio_handler_set(BUT_PIO,
                  BUT_PIO_ID,
                  BUT_IDX_MASK,
                  PIO_IT_EDGE,
                  but_callback);

  // Ativa interrupção
  pio_enable_interrupt(BUT_PIO, BUT_IDX_MASK);

  // Configura NVIC para receber interrupcoes do PIO do botao
  // com prioridade 4 (quanto mais próximo de 0 maior)
  NVIC_EnableIRQ(BUT_PIO_ID);
  NVIC_SetPriority(BUT_PIO_ID, 4); // Prioridade 4
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/
/* flag */

// Funcao principal chamada na inicalizacao do uC.
void main(void)
{
	// Inicializa clock
	sysclk_init();

	// Desativa watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;

  // configura botao com interrupcao
  io_init();

	// super loop
	// aplicacoes embarcadas no devem sair do while(1).
	while(1)
  {
	     if(but_flag){
			 pisca_led(5, 200);
		     // zera flag
		     but_flag = 0;
	     }
		 pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);

	}
}
