/* -------------------------------------------------------------------------
 * Monitoramento e Controle Inteligente de Temperatura  -  Rev 1.1
 * ---------------------------------------------------------------------
 * Firmware para STM8S003F3P6 que le a temperatura com o sensor LM35 e
 * controla a velocidade de uma ventoinha de 5 V por PWM.
 *
 * Funcionamento resumido:
 *   1. Le o canal analogico PD2 (AIN3) onde esta ligada a saida do LM35
 *      (10 mV por grau Celsius), fazendo a media de varias leituras.
 *   2. Converte a media para decimos de grau Celsius (sem ponto flutuante).
 *   3. Calcula o duty cycle do PWM de acordo com a temperatura:
 *      - abaixo de ~28 C ........ ventoinha desligada
 *      - de 30 C a 50 C ......... rampa linear de 35% a 100%
 *      - acima de 50 C .......... 100%
 *      Com histerese entre ligar/desligar para a ventoinha nao ficar
 *      oscilando perto do limite.
 *   4. Gera o PWM no pino PC3 (TIM1_CH3), que chaveia o MOSFET 2N7000
 *      em low-side (nivel alto = ventoinha ligada).
 *   5. Mostra temperatura e situacao no LCD 16x2 via I2C (PCF8574).
 *   6. Botao "Max_Cooler" em PA3: enquanto pressionado, ventoinha em 100%
 *      independentemente da temperatura (pull-up interno, fecha para GND).
 *   7. LED de status em PD4:
 *        - aceso fixo ......... operacao normal
 *        - piscando rapido .... falha na leitura do sensor (ventoinha em
 *                               100% por seguranca, "esfria por duvida")
 *
 * ================== ALTERACOES DA REV 1.1 (em relacao a 1.0) ===========
 *   [1] BASE DE TEMPO POR INTERRUPCAO: o TIM4 gera uma interrupcao a cada
 *       1 ms. A ISR sinaliza o ciclo de controle (LOOP_MS) e o pisca do
 *       LED (PISCA_MS) por flags. O laco principal NAO usa mais
 *       delayMs(): processa quando a flag chega e dorme com wfi().
 *       -> atende o requisito "tratamento de interrupcoes / sem delays
 *          travantes no main" proposto no TDE01.
 *   [2] FALHA DO SENSOR SEM BLOQUEIO: Tratar_FalhaSensor() tinha um
 *       while(1) com delayMs(150). Agora a falha e um estado: ventoinha
 *       em 100%, LED piscando pelo TIM4 e o sistema continua lendo o
 *       sensor a cada ciclo, voltando sozinho ao normal.
 *   [3] CORRECAO - PWM DO CANAL 3: o modo do canal 3 do TIM1 fica no
 *       registrador CCMR3 (CCMR1..CCMR4 = canais 1..4). A Rev 1.0
 *       escrevia em CCMR2 (canal 2), deixando o CH3 em modo "frozen":
 *       a saida PC3 nao gerava PWM e a ventoinha nao ligava.
 *   [4] CORRECAO - PRESCALER DO ADC: SPSEL fica nos bits 6:4 do ADC1_CR1.
 *       O valor 0x10 resultava em SPSEL=001 (fMASTER/3 = 5,3 MHz) e nao
 *       em /8 como dizia o comentario. Agora 0x40 -> fADC = 2 MHz.
 *   [5] CORRECAO - HISTERESE: com a ventoinha ligada e a temperatura
 *       entre 28 e 30 C, (tempX10 - TEMP_LIGA_X10) era negativo em
 *       variavel sem sinal -> estouro e duty "lixo". Agora, nessa faixa,
 *       o duty fica no minimo (DUTY_MINIMO_PCT).
 *   [6] ORDEM DE LEITURA DO ADC: com alinhamento a direita o RM0016 manda
 *       ler DRL antes de DRH. A expressao unica da Rev 1.0 nao garantia a
 *       ordem (ordem de avaliacao em C nao e definida).
 *   [7] Schmitt trigger do PD2/AIN3 desligado (ADC1_TDRL): menos ruido
 *       e menor consumo na entrada analogica.
 *   [8] WATCHDOG INDEPENDENTE (IWDG, ~1 s): se o firmware travar (ex.: I2C
 *       preso), o MCU reinicia e a ventoinha nao fica parada. Pode ser
 *       desligado em USAR_WATCHDOG para depuracao.
 *   [9] Duty 100% / CCR calculado com (ARR + 1), coerente com a formula
 *       duty = CCR3 / (ARR + 1).
 *
 * Ambiente de compilacao (mesmo do projeto da aula 07 - Ex02):
 *   ST Visual Develop + Cosmic CXSTM8 + STM8S Standard Peripheral Library
 *   Projeto compilado com -dSTM8S103 (registradores identicos para os
 *   perifericos usados aqui: GPIO, ADC1, TIM1, TIM4, IWDG, I2C e CLK).
 *
 * OBSERVACAO SOBRE O ESTILO DO CODIGO:
 *   GPIO, clock e I2C/LCD usam as funcoes da SPL (stm8s_gpio.c, stm8s_clk.c
 *   e stm8s_i2c.c ja fazem parte do projeto). ADC1, TIM1, TIM4 e IWDG sao
 *   configurados por escrita DIRETA nos registradores. Motivo: o STM8S003
 *   so tem 8 KB de flash e, no Cosmic, cada arquivo .c adicionado ao
 *   projeto entra inteiro no executavel. NAO adicione stm8s_adc1.c,
 *   stm8s_tim1.c, stm8s_tim4.c ou stm8s_iwdg.c ao projeto.
 *
 *   UNICA ALTERACAO FORA DO main.c: registrar a ISR do TIM4 (IRQ 23) no
 *   arquivo stm8_interrupt_vector.c - ver stm8_interrupt_vector.c enviado.
 *
 * Clock: HSI de 16 MHz sem divisao (CLK_PRESCALER_HSIDIV1). As rotinas de
 * delay (delay.c) e o driver do LCD estao calibrados para esse clock.
 *
 * Mapa de pinos (esquematico Rev 1.1, TSSOP-20):
 *   PD2  pino 19  AIN3     <- TEMP  (saida do LM35)
 *   PC3  pino 13  TIM1_CH3 -> PWM   (gate do 2N7000 via resistor 100 R)
 *   PD4  pino  1           -> LED   (1 kR + LED ate GND; alto acende)
 *   PA3  pino 10           <- BTN   (Max_Cooler, fecha para GND)
 *   PB4  pino 12  SCL      <-> LCD I2C
 *   PB5  pino 11  SDA      <-> LCD I2C
 *   PD1  pino 18  SWIM     <-> gravador ST-Link (conector J1)
 *   VCAP pino  8           -- 1 uF ceramico para GND (C5)
 *
 * ATENCAO - parametros provisorios:
 *   As faixas de temperatura, o duty minimo e a frequencia de PWM abaixo
 *   sao valores de partida para os testes, NAO especificacao final.
 *   Ajuste os #define e recompile conforme o comportamento real do
 *   sistema na bancada (cada um tem comentario explicando o efeito).
 * --------------------------------------------------------------------- */

#include "stm8s.h"
#include "delay.h"
#include "LCD_PCF8574_I2C.h"

/* ================================ PINOS ================================ */

/* LED de status (PD4 = pino 1 no TSSOP-20, conferido no esquematico 1.1) */
#define LED_GPIO_PORT     GPIOD
#define LED_GPIO_PIN      GPIO_PIN_4

/* Botao Max_Cooler: pressionado = nivel baixo (fecha para GND) */
#define BTN_GPIO_PORT     GPIOA
#define BTN_GPIO_PIN      GPIO_PIN_3

/* Entrada analogica: PD2 = AIN3 do ADC1. No registrador ADC1_CSR, os
   bits 3:0 recebem o numero do canal: 3. */
#define TEMP_ADC_CANAL  3U

/* Saida PWM: PC3 = canal 3 do TIM1 (TIM1_CH3) */

/* ========================= PARAMETROS PROVISORIOS ====================== */

/* Tensao de referencia do ADC em milivolts. No STM8S003 a referencia e a
   propria alimentacao (VDD). Meca os 5 V reais com multimetro e ajuste:
   cada 1% de erro aqui vira ~1% de erro na temperatura lida. */
#define VREF_MV                5000U

/* Faixa de controle em decimos de grau (evita ponto flutuante). */
#define TEMP_DESLIGA_X10        280U  /* 28.0 C: abaixo disso desliga     */
#define TEMP_LIGA_X10           300U  /* 30.0 C: limiar para ligar        */
#define TEMP_MAXIMA_X10         500U  /* 50.0 C: dai para cima = 100%     */
/* A diferenca entre LIGA e DESLIGA e a histerese (2 C). Aumente se a
   ventoinha ainda oscilar liga/desliga perto do limite. */

/* Duty minimo (%) com a ventoinha ligada. Abaixo disso muitas ventoinhas
   nao giram (travam) - ajuste segundo o modelo usado. */
#define DUTY_MINIMO_PCT          35U

/* Frequencia do PWM em Hz. 1 kHz e um meio-termo. Se preferir silencio
   total, teste 25000 (o capacitor de 100 uF em paralelo com a ventoinha
   passa a filtrar o PWM, gerando uma tensao quase continua). */
#define PWM_FREQUENCIA_HZ      1000U

/* Numero de amostras do ADC usadas na media (filtragem do ruido). */
#define ADC_AMOSTRAS              16U

/* Periodo do ciclo de controle, em milissegundos (contado pelo TIM4). */
#define LOOP_MS                  250U

/* Meio periodo do pisca-pisca do LED em falha, em milissegundos. */
#define PISCA_MS                 150U

/* Limites de leitura plausivel (LM35 alimentado a 5 V mede 2 a 150 C).
   Fora disso considera-se falha do sensor (fio solto, curto etc.). */
#define TEMP_MINIMA_VALIDA_X10    20U
#define TEMP_MAXIMA_VALIDA_X10  1500U

/* Watchdog: 1 = ligado (uso normal), 0 = desligado (depuracao/bancada,
   ex.: ao investigar um LCD que nao responde no I2C). */
#define USAR_WATCHDOG              1

/* Periodo do PWM: fMASTER = 16 MHz, prescaler do TIM1 = 1.
   ARR = (16 MHz / frequencia) - 1  (ex.: 1 kHz -> 15999) */
#define PWM_ARR  ((uint16_t)((16000000UL / PWM_FREQUENCIA_HZ) - 1UL))

/* ============================ TIPOS E ESTADOS ========================== */

typedef enum
{
    TEMP_OK   = 0,
    TEMP_ERRO = 1
} TempStatus;

/* Flags e contadores compartilhados com a ISR do TIM4 (volatile: sao
   alterados dentro da interrupcao e lidos no laco principal). */
static volatile uint8_t  flagCiclo  = 0U;   /* 1 a cada LOOP_MS  */
static volatile uint8_t  flagPisca  = 0U;   /* 1 a cada PISCA_MS */
static volatile uint16_t contCiclo  = 0U;
static volatile uint8_t  contPisca  = 0U;

/* ==================== INTERRUPCAO DO TIM4 (IRQ 23) ===================== */

/* Chamada a cada 1 ms (overflow/update do TIM4). Mantida curta: apenas
   conta e sinaliza; todo o processamento acontece no laco principal.
   Registrada na posicao irq23 do stm8_interrupt_vector.c. */
@far @interrupt void TIM4_UPD_OVF_IRQHandler(void)
{
    TIM4->SR1 = (uint8_t)(~TIM4_SR1_UIF);   /* limpa a flag de update */

    if (++contCiclo >= LOOP_MS)
    {
        contCiclo = 0U;
        flagCiclo = 1U;
    }
    if (++contPisca >= PISCA_MS)
    {
        contPisca = 0U;
        flagPisca = 1U;
    }
}

/* ======================= INICIALIZACOES DO SISTEMA ===================== */

/* Clock: HSI interno de 16 MHz, sem divisao. delay.c e o LCD dependem. */
static void Clock_Config(void)
{
    CLK_DeInit();
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
}

/* Configuracao dos pinos de uso geral. */
static void GPIO_Config(void)
{
    /* LED: saida push-pull, comeca desligado (nivel baixo) */
    GPIO_Init(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

    /* Botao: entrada com pull-up interno (o esquematico nao tem pull-up
       externo; pressionado = pino em GND) */
    GPIO_Init(BTN_GPIO_PORT, BTN_GPIO_PIN, GPIO_MODE_IN_PU_NO_IT);

    /* PD2/AIN3: entrada flutuante (modo analogico) */
    GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_IN_FL_NO_IT);

    /* PC3: saida push-pull rapida - o periferico TIM1_CH3 assume o
       controle do nivel do pino quando o PWM e habilitado */
    GPIO_Init(GPIOC, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);
}

/* -------------------------------------------------------------------------
 * ADC1 por registradores.
 *
 *   ADC1_CR1:  bits 6:4 = SPSEL (prescaler do clock do ADC)
 *              bit 0    = ADON (liga o bloco / inicia conversao)
 *   ADC1_CR2:  bit 3    = ALIGN (1 = alinhado a direita)
 *   ADC1_CSR:  bits 3:0 = canal (3 = AIN3 = PD2); bit 7 = EOC
 *   ADC1_TDRL: bit n = 1 desliga o Schmitt trigger do canal n
 *   ADC1_DRH/DRL: resultado (10 bits). Alinhado a direita: ler DRL
 *                 PRIMEIRO e depois DRH (RM0016).
 * --------------------------------------------------------------------- */
static void ADC_Config(void)
{
    /* [4] SPSEL = 100 (bits 6:4) -> fADC = fMASTER/8 = 2 MHz */
    ADC1->CR1 = (uint8_t)0x40;        /* ADC ainda desligado (ADON = 0)  */

    ADC1->CR2 = ADC1_CR2_ALIGN;       /* resultado alinhado a direita    */

    ADC1->CSR = TEMP_ADC_CANAL;       /* canal 3 (PD2/AIN3); zera EOC    */

    /* [7] desliga o Schmitt trigger da entrada AIN3 (uso analogico) */
    ADC1->TDRL = (uint8_t)(1U << TEMP_ADC_CANAL);

    ADC1->CR1 |= ADC1_CR1_ADON;       /* liga o bloco do ADC             */
    delayUs(10);                      /* tempo de estabilizacao (tSTAB)  */
}

/* -------------------------------------------------------------------------
 * TIM1 por registradores: PWM no canal 3, que sai no pino PC3.
 *
 *   frequencia = 16 MHz / (PSC + 1) / (ARR + 1)
 *   duty (%)   = 100 * CCR3 / (ARR + 1)
 *
 *   CCMR3 - modo do canal 3 (OC3M) e preload do CCR3 (OC3PE)
 *   CCER2 - habilita a saida do canal 3 (CC3E) e polaridade (CC3P)
 *   BKR   - bit MOE: saidas do timer "avancado" (obrigatorio!)
 *   CR1   - ARPE (preload do ARR) e CEN (liga o contador)
 * --------------------------------------------------------------------- */
static void PWM_Config(void)
{
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER1, ENABLE);

    TIM1->PSCRH = 0x00;                          /* prescaler = 1         */
    TIM1->PSCRL = 0x00;                          /* -> contador a 16 MHz  */

    TIM1->ARRH = (uint8_t)(PWM_ARR >> 8);        /* periodo do PWM        */
    TIM1->ARRL = (uint8_t)(PWM_ARR & 0xFFU);

    TIM1->CCR3H = 0x00;                          /* comeca em 0%          */
    TIM1->CCR3L = 0x00;                          /* (ventoinha desligada) */

    /* [3] CCMR3 (canal 3) - na Rev 1.0 estava CCMR2 (canal 2):
       OC3M  (bits 6:4) = 110 -> modo PWM 1
       OC3PE (bit 3)    = 1   -> CCR3 atualizado so no fim do periodo   */
    TIM1->CCMR3 = (uint8_t)(0x60U | 0x08U);

    /* CC3E = 1 habilita a saida no PC3; CC3P = 0 -> ativo em nivel alto */
    TIM1->CCER2 = TIM1_CCER2_CC3E;

    /* TIM1 e timer "avancado": sem MOE=1 as saidas ficam mortas. */
    TIM1->BKR = TIM1_BKR_MOE;

    /* ARPE = 1 (ARR com preload) + CEN = 1 (comeca a contar) */
    TIM1->CR1 = (uint8_t)(TIM1_CR1_ARPE | TIM1_CR1_CEN);
}

/* -------------------------------------------------------------------------
 * [1] TIM4 por registradores: base de tempo de 1 ms com interrupcao.
 *
 *   16 MHz / 64 (PSCR = 6) = 250 kHz -> 4 us por contagem
 *   ARR = 249 -> 250 contagens = 1 ms por interrupcao de update
 * --------------------------------------------------------------------- */
static void TIM4_Config(void)
{
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);

    TIM4->CR1  = 0x00;                           /* parado durante config */
    TIM4->PSCR = 0x06;                           /* prescaler = 2^6 = 64  */
    TIM4->ARR  = 249U;                           /* 1 ms                  */
    TIM4->CNTR = 0x00;
    TIM4->SR1  = 0x00;                           /* limpa flags           */
    TIM4->IER  = TIM4_IER_UIE;                   /* interrupcao de update */
    TIM4->CR1  = (uint8_t)(TIM4_CR1_ARPE | TIM4_CR1_CEN);
}

/* -------------------------------------------------------------------------
 * [8] IWDG por registradores: ~1,02 s (LSI 128 kHz / 2 / 256 * 256).
 *   KR = 0xCC inicia | 0x55 libera PR/RLR | 0xAA recarrega ("alimenta")
 *   Uma vez iniciado, o IWDG nao pode ser desligado ate o proximo reset.
 * --------------------------------------------------------------------- */
static void IWDG_Config(void)
{
#if USAR_WATCHDOG
    IWDG->KR  = 0xCC;          /* inicia o watchdog                 */
    IWDG->KR  = 0x55;          /* libera escrita em PR e RLR        */
    IWDG->PR  = 0x06;          /* divisor /256                      */
    IWDG->RLR = 0xFF;          /* contagem maxima -> ~1,02 s        */
    IWDG->KR  = 0xAA;          /* recarrega e protege os registros  */
#endif
}

static void IWDG_Alimentar(void)
{
#if USAR_WATCHDOG
    IWDG->KR = 0xAA;
#endif
}

/* ====================== LEITURA DA TEMPERATURA ========================= */

/* Le ADC_AMOSTRAS conversoes, tira a media e converte para decimos de
   grau Celsius. Retorna TEMP_ERRO se o ADC travar ou o resultado for
   implausivel (sensor fora da faixa, em curto ou desconectado).

   Formula: o LM35 gera 10 mV por grau, entao a temperatura em decimos
   de grau coincide numericamente com a tensao em milivolts:

     T [0.1 C] = V [mV] = bruto * VREF_MV / 1023                      */
static TempStatus Temperatura_Ler(uint16_t *tempX10)
{
    uint8_t  i;
    uint8_t  baixo, alto;
    uint16_t guarda;
    uint32_t media;

    media = 0;
    for (i = 0; i < ADC_AMOSTRAS; i++)
    {
        /* reescreve o canal e zera o EOC; ADON=1 inicia a conversao */
        ADC1->CSR = TEMP_ADC_CANAL;
        ADC1->CR1 |= ADC1_CR1_ADON;

        /* espera EOC com protecao contra travamento (~7 us normalmente) */
        guarda = 0;
        while ((ADC1->CSR & ADC1_CSR_EOC) == 0U)
        {
            if (++guarda > 5000U)
            {
                return TEMP_ERRO;   /* ADC nao respondeu */
            }
        }

        /* [6] alinhado a direita: DRL primeiro, depois DRH */
        baixo = ADC1->DRL;
        alto  = ADC1->DRH;
        media += (uint16_t)(((uint16_t)alto << 8) | (uint16_t)baixo);
    }
    media /= ADC_AMOSTRAS;

    /* converte para milivolts = decimos de grau */
    media = (media * (uint32_t)VREF_MV) / 1023UL;

    if (media > 65535UL)
    {
        return TEMP_ERRO;
    }

    *tempX10 = (uint16_t)media;

    if ((*tempX10 < TEMP_MINIMA_VALIDA_X10) ||
        (*tempX10 > TEMP_MAXIMA_VALIDA_X10))
    {
        return TEMP_ERRO;   /* leitura fora da faixa do LM35 */
    }
    return TEMP_OK;
}

/* ================= CONTROLE DA VENTOINHA (logica) ====================== */

/* Estado da histerese (fora da funcao para a falha poder ajusta-lo). */
static uint8_t ventoinhaLigada = 0U;

/* Define o duty cycle (0 a 100) segundo a temperatura:
   - botao pressionado -> 100% (prioridade maxima, ignora temperatura)
   - histerese de ligar/desligar entre TEMP_DESLIGA_X10 e TEMP_LIGA_X10
   - entre TEMP_LIGA_X10 e TEMP_MAXIMA_X10 -> rampa linear
     DUTY_MINIMO -> 100% */
static uint8_t Controle_Duty(uint16_t tempX10, uint8_t botao)
{
    if (botao != 0U)
    {
        return 100U;
    }

    /* histerese: so liga acima de TEMP_LIGA_X10 e so desliga abaixo de
       TEMP_DESLIGA_X10 (evita liga/desliga rapido perto do limite) */
    if ((ventoinhaLigada == 0U) && (tempX10 >= TEMP_LIGA_X10))
    {
        ventoinhaLigada = 1U;
    }
    else if ((ventoinhaLigada != 0U) && (tempX10 <= TEMP_DESLIGA_X10))
    {
        ventoinhaLigada = 0U;
    }
    else
    {
        /* mantem o estado atual */
    }

    if (ventoinhaLigada == 0U)
    {
        return 0U;
    }

    /* [5] ligada e ainda dentro da faixa de histerese (28 a 30 C):
       mantem o duty minimo - evita subtracao negativa sem sinal */
    if (tempX10 <= TEMP_LIGA_X10)
    {
        return (uint8_t)DUTY_MINIMO_PCT;
    }

    if (tempX10 >= TEMP_MAXIMA_X10)
    {
        return 100U;
    }

    return (uint8_t)(DUTY_MINIMO_PCT +
                     (((uint32_t)(tempX10 - TEMP_LIGA_X10) *
                       (100UL - DUTY_MINIMO_PCT)) /
                      (TEMP_MAXIMA_X10 - TEMP_LIGA_X10)));
}

/* ======================= SAIDAS: PWM, LED, BOTAO ======================= */

/* Aplica o duty (0 a 100) no PWM da ventoinha. */
static void Fan_SetDuty(uint8_t duty)
{
    uint16_t ccr;

    if (duty > 100U)
    {
        duty = 100U;
    }
    if (duty == 100U)
    {
        ccr = (uint16_t)(PWM_ARR + 1U);  /* comparacao nunca ocorre: 100% */
    }
    else
    {
        /* [9] duty = CCR3 / (ARR + 1) */
        ccr = (uint16_t)(((uint32_t)duty * ((uint32_t)PWM_ARR + 1UL)) / 100UL);
    }
    TIM1->CCR3H = (uint8_t)(ccr >> 8);   /* com OC3PE=1 o valor novo so  */
    TIM1->CCR3L = (uint8_t)(ccr & 0xFFU);/* vale no fim do periodo atual */
}

/* Retorna 1 se o botao Max_Cooler estiver pressionado (pino em GND). */
static uint8_t Botao_Pressionado(void)
{
    return (GPIO_ReadInputPin(BTN_GPIO_PORT, BTN_GPIO_PIN) == RESET) ? 1U : 0U;
}

/* ============================== DISPLAY ================================ */

/* Escreve "valor" (0 a 999) como texto, sem zeros a esquerda, em dst.
   Retorna o numero de caracteres escritos. */
static uint8_t NumeroParaTexto(uint16_t valor, char *dst)
{
    char    tmp[3];
    uint8_t n = 0U;
    uint8_t i;

    if (valor > 999U)
    {
        valor = 999U;
    }
    if (valor >= 100U)
    {
        tmp[n++] = (char)('0' + ((valor / 100U) % 10U));
    }
    if (valor >= 10U)
    {
        tmp[n++] = (char)('0' + ((valor / 10U) % 10U));
    }
    tmp[n++] = (char)('0' + (valor % 10U));

    for (i = 0U; i < n; i++)
    {
        dst[i] = tmp[i];
    }
    return n;
}

/* Completa a linha com espacos ate 16 colunas e encerra a string. */
static void CompletarLinha(char *linha, uint8_t pos)
{
    while (pos < 16U)
    {
        linha[pos++] = ' ';
    }
    linha[16] = '\0';
}

/* Linha 1 do LCD: "Temp: 25.4" + grau + "C" (16 colunas).
   0xDF e o simbolo de grau na ROM padrao do HD44780. */
static void LCD_LinhaTemperatura(uint16_t tempX10)
{
    char       linha[17];
    uint8_t    pos = 0U;
    uint16_t   parteInteira = tempX10 / 10U;

    linha[pos++] = 'T';
    linha[pos++] = 'e';
    linha[pos++] = 'm';
    linha[pos++] = 'p';
    linha[pos++] = ':';
    linha[pos++] = ' ';
    pos += NumeroParaTexto(parteInteira, &linha[pos]);
    linha[pos++] = '.';
    linha[pos++] = (char)('0' + (tempX10 % 10U));
    linha[pos++] = (char)0xDF;   /* grau */
    linha[pos++] = 'C';
    CompletarLinha(linha, pos);

    LCD_SetCursor(0, 0);
    LCD_WriteString(linha);
}

/* Linha 2 do LCD: "Fan: 45% AUTO" / "Fan: 100% MAX" / "Fan: 0% OFF". */
static void LCD_LinhaVentoinha(uint8_t duty, uint8_t botao)
{
    char    linha[17];
    uint8_t pos = 0U;

    linha[pos++] = 'F';
    linha[pos++] = 'a';
    linha[pos++] = 'n';
    linha[pos++] = ':';
    linha[pos++] = ' ';
    pos += NumeroParaTexto(duty, &linha[pos]);
    linha[pos++] = '%';
    linha[pos++] = ' ';
    if (botao != 0U)
    {
        linha[pos++] = 'M';
        linha[pos++] = 'A';
        linha[pos++] = 'X';
    }
    else if (duty == 0U)
    {
        linha[pos++] = 'O';
        linha[pos++] = 'F';
        linha[pos++] = 'F';
    }
    else
    {
        linha[pos++] = 'A';
        linha[pos++] = 'U';
        linha[pos++] = 'T';
        linha[pos++] = 'O';
    }
    CompletarLinha(linha, pos);

    LCD_SetCursor(0, 1);
    LCD_WriteString(linha);
}

/* Tela exibida quando a leitura do sensor falha. */
static void LCD_MostrarErro(void)
{
    LCD_SetCursor(0, 0);
    LCD_WriteString("Falha no sensor!");
    LCD_SetCursor(0, 1);
    LCD_WriteString("Ventoinha: 100% ");
}

/* ================================ MAIN ================================= */

void main(void)
{
    uint16_t tempX10 = 0U;
    uint8_t  duty;
    uint8_t  botao;
    uint8_t  emFalha = 0U;   /* [2] estado de falha do sensor (0/1) */

    disableInterrupts();

    Clock_Config();
    GPIO_Config();
    PWM_Config();            /* ventoinha em 0% o quanto antes */
    ADC_Config();

    /* Tela inicial: ainda usa delayMs, mas apenas uma vez, antes de
       ligar a base de tempo e o watchdog (nao afeta o laco principal) */
    LCD_Init();
    LCD_SetCursor(0, 0);
    LCD_WriteString("Controle Temp.  ");
    LCD_SetCursor(0, 1);
    LCD_WriteString("Iniciando...    ");
    delayMs(1000);

    /* LED aceso fixo = sistema em operacao normal */
    GPIO_WriteHigh(LED_GPIO_PORT, LED_GPIO_PIN);

    TIM4_Config();           /* [1] base de tempo de 1 ms              */
    IWDG_Config();           /* [8] watchdog de ~1 s                   */
    enableInterrupts();

    while (1)
    {
        /* ---------- ciclo de controle: a cada LOOP_MS (flag da ISR) ---- */
        if (flagCiclo != 0U)
        {
            flagCiclo = 0U;
            IWDG_Alimentar();

            if (Temperatura_Ler(&tempX10) != TEMP_OK)
            {
                /* [2] falha: ventoinha em 100% ("esfria por duvida") */
                Fan_SetDuty(100U);
                ventoinhaLigada = 1U;   /* ao normalizar, sai "esfriando" */
                if (emFalha == 0U)
                {
                    emFalha = 1U;
                    LCD_MostrarErro();  /* so escreve ao entrar na falha */
                }
            }
            else
            {
                if (emFalha != 0U)
                {
                    emFalha = 0U;       /* sensor voltou ao normal */
                    GPIO_WriteHigh(LED_GPIO_PORT, LED_GPIO_PIN);
                }

                botao = Botao_Pressionado();
                duty  = Controle_Duty(tempX10, botao);
                Fan_SetDuty(duty);

                LCD_LinhaTemperatura(tempX10);
                LCD_LinhaVentoinha(duty, botao);
            }
        }

        /* ---------- pisca do LED em falha: a cada PISCA_MS ------------- */
        if (flagPisca != 0U)
        {
            flagPisca = 0U;
            if (emFalha != 0U)
            {
                GPIO_WriteReverse(LED_GPIO_PORT, LED_GPIO_PIN);
            }
        }

        /* dorme ate a proxima interrupcao (TIM4 acorda a cada 1 ms) */
        wfi();
    }
}
