;===============================================================================;
; Датчик Емкостной с Температурной Коррекцией Интеллектуальный			;
; ATtiny44									;
; Шульц Валерий,  +7 903 640-70-36. г.Рязань					;
;  2016 Aug 10  Ver 0.10 Доработка v0.32 ros101pp под реле			;
;===============================================================================;
;
; Концепция:
; При калибровке подбирается коэфф.деления опорного и измерительного каналов так,
; чтобы период составлял около 300 мкс, тогда за 20 мс опоры будет около 67 периодов,
; а за 180 мс измерения будет около 600 периодов.

.include "tn44def.inc"
; Порты процессора
.equ	Chnl	=PA0	; "1" - измер.канал, "0" - опорный канал
.equ	KeyZero	=PA1	; Кнопка 1 - Установка нуля
.equ	KeyMax	=PA2	; Кнопка 2 - Установка максимума
.equ	Freq	=PA3	; входная частота
.equ	Sck	=PA4
.equ	Miso	=PA5
.equ	 Rele	=Miso	; Выходной ключ, "1" - Реле включено
.equ	Mosi	=PA6
.equ	 Leds	=Mosi
.equ	Fxi	=PA7
.equ	Xt1	=PB0
.equ	Xt2	=PB1
.equ	Jp1	=PB2
.equ	Res	=PB3


.equ	p_Key	=PORTA
.equ	i_Key	=PINA
.equ	d_Key	=DDRA
.equ	p_Led	=PORTA
.equ	i_Led	=PINA
.equ	d_Led	=DDRA
.equ	p_Chnl	=PORTA
.equ	i_Chnl	=PINA
.equ	d_Chnl	=DDRA
.equ	p_Freq	=PORTA
.equ	i_Freq	=PINA
.equ	d_Freq	=DDRA
.equ	p_Sck	=PORTA
.equ	i_Sck	=PINA
.equ	d_Sck	=DDRA
.equ	p_Tok	=PORTA
.equ	i_Tok	=PINA
.equ	d_Tok	=DDRA
.equ	p_Mosi	=PORTA
.equ	i_Mosi	=PINA
.equ	d_Mosi	=DDRA
.equ	p_Fxi	=PORTA
.equ	i_Fxi	=PINA
.equ	d_Fxi	=DDRA
.equ	p_Jp1	=PORTB
.equ	i_Jp1	=PINB
.equ	d_Jp1	=DDRB
;---------------------------------------
 .dseg
;r0-r3 - локальные регистры в процедурах
.def	R_sreg		=r4
.def	iG		=r5
.def	Ag		=r6
.def	Bg		=r7
.def	Dg		=r8
.def	iSendByte	=r9

.def	oICRl		=r10
.def	oICRh		=r11
.def	rICRl		=r12
.def	rICRh		=r13
.def	rICRg		=r14
.def	KS		=r15

.def	A		=r16
.def	Ah		=r17
.def	B		=r18
.def	Bh		=r19
.def	D		=r20
.def	Dh		=r21
.def	status		=r22
.equ	 f_start	=0	;признак нового цикла измерения
.equ	 f_BufReady	=1
;.equ	 f_SendBit	=2
.equ	 f_CycleComplit	=3
.equ	 f_KeyZero	=4
.equ	 f_KeyMax	=5
.equ	 f_test		=6
.def	stl		=r23
.def	Fl		=r24
.def	Fh		=r25
;==================
	.org 0x60
BufDataSend:	.byte	16
BufObmen:	.byte	16
BufLed:		.byte	6
;блок копии установок EEPROM 
eDelitOpora:	.byte	1	;делитель частоты для опорного канала
eDelitIzmer:	.byte	1	;делитель частоты для измерительного канала
eOporaNormd:	.byte	1	;период опоры при температуре калибровки
eOporaNorml:	.byte	1
eOporaNormh:	.byte	1
eIzmerNormd:	.byte	1	;период сухого изм.канала при температуре калибровки
eIzmerNorml:	.byte	1
eIzmerNormh:	.byte	1
eTempNorml:	.byte	1	;температура калибровки
eTempNormh:	.byte	1
;блок вычисляемых переменных
vDlitOporal:	.byte	1	;длительность интервала вычисления опоры (чуть меньше 20мс), размерность мкс*8
vDlitOporah:	.byte	1
vDlitOporag:	.byte	1
vNimpOporal:	.byte	1	;число импульсов опоры за интервал
vNimpOporah:	.byte	1
vPeriodOporad:	.byte	1	;период опоры, дробный, = vDlitOpora / vNimpOpora, размерность мкс*8
vPeriodOporal:	.byte	1
vPeriodOporah:	.byte	1
vDlitIzmerl:	.byte	1	;длительность интервала вычисления изм.канала (чуть меньше 160мс), размерность мкс*8
vDlitIzmerh:	.byte	1
vDlitIzmerg:	.byte	1
vNimpIzmerl:	.byte	1	;число импульсов изм.канала за интервал
vNimpIzmerh:	.byte	1
vPeriodIzmerd:	.byte	1	;период изм.канала, дробный, = vDlitIzmer / vNimpIzmer, размерность мкс*8
vPeriodIzmerl:	.byte	1
vPeriodIzmerh:	.byte	1
vPeriodCalibd:	.byte	1	;калиброванный с учетом ухода опоры период изм.канала
vPeriodCalibl:	.byte	1	;vPeriodCalib = vPeriodIzmer * eOporaNorm / vPeriodOpora, размерность мкс*8
vPeriodCalibh:	.byte	1
vPeriodTokl:	.byte	1	;значение периода частоты датчика, мкс*8
vPeriodTokh:	.byte	1
vCodTempl:	.byte	1	;усредненный 256 раз код температуры
vCodTemph:	.byte	1
vCodVccl:	.byte	1	;усредненный 256 раз код канала напряжения питания
vCodVcch:	.byte	1
vN_Part:	.byte	1	;номер передаваемой порции данных
;блок вспомогательных переменных
StartICR1l:	.byte	1
StartICR1h:	.byte	1
vNextTCNT1h:	.byte	1	;время начала следующего периода частоты датчика
vNextTCNT1l:	.byte	1
vFazIzm:	.byte	1	;Фаза цикла измерения опоры/сигнала
vst_x5mc:	.byte	1	;остаток длительности фазы измерения
vNewKeys:	.byte	1	;
;vCod__l:	.byte	1
vStMorg:	.byte	1	;счетчики моргания светодиода
vTimeMorg:	.byte	1
vSumTmpg:	.byte	1	;сумма кода температуры
vSumTmph:	.byte	1
vSumTmpl:	.byte	1
vSumVccg:	.byte	1	;сумма кода канала напряжения питания
vSumVcch:	.byte	1
vSumVccl:	.byte	1
stSumAdc:	.byte	1
N1l:		.byte	1
N1h:		.byte	1
N1g:		.byte	1
N2l:		.byte	1
N2h:		.byte	1
N2g:		.byte	1
Nxl:		.byte	1
Nxh:		.byte	1
Nxg:		.byte	1
M1l:		.byte	1
M1h:		.byte	1
M1g:		.byte	1
M2l:		.byte	1
M2h:		.byte	1
M2g:		.byte	1
stSendBit:	.byte	1
st5mc:		.byte	1
stStateKey:	.byte	1
oldKeys:	.byte	1
Keys:		.byte	1
LastPressKey:	.byte	1
NowPressKey:	.byte	1
stPressKey:	.byte	1
st100mc:	.byte	1
stEscape:	.byte	1
st02c:		.byte	1
vTemp1:		.byte	1
vTemp2:		.byte	1
vTemp3:		.byte	1
vTemp4:		.byte	1
vTemp5:		.byte	1



;==================
;vNewKeys bit define:
.equ	bitKnZero	=0
.equ	bitKnMax	=1
;.equ	bitKn3		=2
;.equ	bitKn4		=3
;--------------------------------------------;
.equ	cKeyZero_on	=8	;нажатие "KeyZero"
.equ	cKeyMax_on	=9	;нажатие "KeyMax"
.equ	cKeyZero_on_off	=12	;нажатие/отпускание "KeyZero" менее 4 сек
.equ	cKeyMax_on_off	=13	;нажатие/отпускание "KeyMax" менее 4 сек
.equ	cKeyZero_on4c	=16	;нажатие "KeyZero" и удержание более 4 сек
.equ	cKeyMax_on4c	=17	;нажатие "KeyMax" и удержание более 4 сек
.equ	cKeyZero_off	=8 +32	;отпускание "KeyZero"
.equ	cKeyMax_off	=9 +32	;отпускание "KeyMax"
.equ	cCompKeyZero	=20	;с компьютера. нажатие "KeyZero" и удержание более 4 сек
.equ	cCompKeyMax	=21	;с компьютера. нажатие "KeyMax" и удержание более 4 сек
.equ	cKeyTimeOut	=24	;минута без нажатий кнопок

	.org 0xE0
;STACK - последние 16 байт
;==========================================================================================
	.eseg
.equ	cNdevice	=5		;номер прибора в сети
.equ	cDelitOpora	=22
.equ	cDelitIzmer	=44
.equ	cOporaNorm	=300*8*256
.equ	cIzmerNorm	=200*8*256
.equ	cTempNorm	=39010	;=(300*64)*256/126


.equ	cSerial		=cNdevice
.equ	cVer		=01
.equ	cID		=7
;------------------------------------
	.org 0
;буфер перезаписываемых переменных - 64 слов (128 байт)
svNdeviceh:	.db	high(cNdevice)
svNdevicel:	.db	low (cNdevice)

svDelitOpora:	.db	low (cDelitOpora)
svDelitIzmer:	.db	low (cDelitIzmer)
svOporaNorm:	.db	low(cOporaNorm), high(cOporaNorm), byte3(cOporaNorm)
svIzmerNorm:	.db	low(cIzmerNorm), high(cIzmerNorm), byte3(cIzmerNorm)
svTempNorm:	.db	low(cTempNorm), high(cTempNorm)

	.org 122
svSerial:	.db	byte4(cSerial), byte3(cSerial), high(cSerial), low(cSerial)
svID:		.db	cID
svVer:		.db	cVer
;------------------------------------
;Constants
;==========================================================================================
.cseg
;==========================================================================================
.org	$000
		rjmp	RESET
		reti	;rjmp	EXT_INT0	; External Interrupt Request 0
		reti	;rjmp	PCINT_0			; Pin Change Interrupt Request 0
		reti	;rjmp	PCINT_1		; Pin Change Interrupt Request 1
		reti	;rjmp	WDT_INT		; Watchdog Time-out
		rjmp	TIM1_CAPT		; Timer/Counter1 Capture Event
		rjmp	TIM1_COMPA		; Timer/Counter1 Compare Match A
		reti	;rjmp	TIM1_COMPB		; Timer/Counter1 Compare Match B
		reti	;rjmp	TIM1_OVF	; Timer/Counter1 Overflow
		reti	;rjmp	TIM0_COMPA	; Timer/Counter0 Compare Match A
		reti	;rjmp	TIM0_COMPB	; Timer/Counter0 Compare Match B
		reti	;rjmp	TIM0_OVF	; Timer/Counter0 Overflow
		reti	;rjmp	ANA_COMP	; Analog Comparator
		reti	;rjmp	ADC_RDY		; ADC Conversion Complete
		reti	;rjmp	EE_RDY EEPROM	; Ready
		reti	;rjmp	USI_STR		; USI START
		reti	;rjmp	USI_OVF		; USI Overflow
;================================================================================
		.db	"--------------"
TblModel:	.db	"Smart101pp      "
		.db	"v0.10 10.08.2016"
		.db	"----------------"
;================================================================================
RESET:		cli	;запрет всех прерываний
		ldi	A, (1<<WDCE)|(1<<WDE)
		out	WDTCSR, A
		ldi	A, (1<<WDE)|(1<<WDP2)|(1<<WDP1)|(1<<WDP0)	;2 sec
		out	WDTCSR, A
		ldi	A, low(RAMEND)
		ldi	Ah, high(RAMEND)
		out	SPL, A
		out	SPH, Ah
		ldi	A, (1<<Fxi)|(1<<Leds)|(0<<Rele)|(1<<Sck)|(0<<Freq)|(1<<KeyMax)|(1<<KeyZero)|(1<<Chnl)
		out	PORTA, A
		ldi	A, (1<<Fxi)|(1<<Leds)|(1<<Rele)|(1<<Sck)|(0<<Freq)|(0<<KeyMax)|(0<<KeyZero)|(1<<Chnl)
		out	DDRA, A
		ldi	A, (1<<Res)|(1<<Jp1)
		out	PORTB, A
		ldi	A, (0<<Res)|(1<<Jp1)
		out	DDRB, A
		;очищаем ОЗУ
		clr	Zh
		clr	Zl
		clr	r0
strt01:		st	Z+, r0
		cpi	Zl, 30
		brne	strt01
		ldi	Zl, low(SRAM_START)
strt02:		st	Z+, r0			;очистка ОЗУ
		cpi	Zl, low(RAMEND-1)
		brne	strt02
		cpi	Zh, high(RAMEND-1)
		brne	strt02
		;
		ldi	A, 1<<CLKPCE
		out	CLKPR, A
		ldi	A, 0
		out	CLKPR, A
		ldi	A, 0
		out	MCUCR, A
		;начальное значение переменных
		ldi	Ah, svDelitOpora	; Ah - адрес EEPROM, A - данные, после операции Ah= Ah+1
		ldi	Zl, low (eDelitOpora)
		ldi	Zh, high(eDelitOpora)
strt03:		rcall	ReadEE
		st	Z+, A
		cpi	Zl, low (eDelitOpora+10)
		brne	strt03
; Таймер 0 считает импульсы на входе Т0 (пин 10) и выдает меандр на OC0B с периодом F(T0) / (2 х OCR0A)
		ldi	A, (0<<COM0A1)|(0<<COM0A0)|(0<<COM0B1)|(1<<COM0B0)|(1<<WGM01)|(0<<WGM00)
		out	TCCR0A, A	;Pin OC0A = off, pin OC0B = Toggle OC0B on Compare Match, CTC mode.
		ldi	A, (0<<WGM02)|(1<<CS02)|(1<<CS01)|(1<<CS00)
		out	TCCR0B, A	;External clock source on T0 pin. Clock on rising edge.
		ldi	A, 81-1
		out	OCR0A, A
		ldi	A, 0		;прерывания запрещены
		out	TIMSK0, A
; Таймер 1 в свободном счете на частоте 8 МГц. 
		ldi	A, 0
		out	TCCR1B, A
		out	TCNT1H, A
		out	TCNT1L, A
		ldi	A,  low (5000*8)
		ldi	Ah, high(5000*8)
		out	OCR1AH, Ah
		out	OCR1AL, A
		ldi	A,  low (100*8)
		ldi	Ah, high(100*8)
		out	OCR1BH, Ah
		out	OCR1BL, A
		ldi	A,  low (1000*8)
		ldi	Ah, high(1000*8)
		sts	vPeriodTokl, A	;длительность периода по умолчанию
		sts	vPeriodTokh, Ah
		sts	vNextTCNT1l, A	;время следующего совпадения
		sts	vNextTCNT1h, Ah
;		sbi	p_Tok, Tok
;		ldi	A, (0<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(1<<COM1B0)|(0<<WGM11)|(0<<WGM10)
		ldi	A, (0<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<WGM11)|(0<<WGM10)
		out	TCCR1A, A	;Pin OC1A = off, pin OC1B = Toggle OC1B on Compare Match, Normal mode.
		ldi	A, (0<<ICNC1)|(1<<ICES1)|(0<<WGM13)|(0<<WGM12)|(0<<CS12)|(0<<CS11)|(1<<CS10)
		out	TCCR1B, A	;Ftact - 8MHz, Rising Edge Input Capture
;		ldi	A, (1<<ICIE1)|(1<<OCIE1B)|(1<<OCIE1A)
		ldi	A, (1<<ICIE1)|(0<<OCIE1B)|(1<<OCIE1A)
		out	TIMSK1, A
		ldi	A, (1<<ICF1)|(1<<OCF1B)|(1<<OCF1A)
		out	TIFR1, A
		ldi	A, 0
		sts	vFazIzm, A	;запуск цикла измерения
		sei
		rcall	SetMorg5
		rjmp	Main
;================================================================================
; 10пФ * 330кОм = 3,3 мкс  (х256 = 844,8 мкс = 6758,4 тактов)
; 65536 / 8 = 8192 мкс максимальный период
;================================================================================
;Режим:		Зеленый	- питание есть, реле выключено
;		Желтый	- питание есть, реле включено
;		Синий	- режим "Тест"
;		Красный миг.2Гц - обрыв датчика
;
;Значение:	синий мигает - ниже "0"
;		от синего до зеленого - от "0" до 50%
;		зеленый - 50%
;		от зеленого до красного - значение от 50% до "Мах"
;		красный мигает - выше "Мах"
;================================================================================
Main:		;калибровка
		wdr
		sbrc	status, f_test
		rjmp	TestMode		;если установлен признак теста, переход в 
		rcall	Inkey
		cpi	A, cKeyZero_on_off
		brne	ma01
		;нажатие-отпускание Key1
		rcall	SetMorg3
		rjmp	ma10

ma01:		cpi	A, cKeyZero_on4c
		brne	ma02
		;нажатие 3 секунды KeyZero
		rcall	SetMorg4
		rcall	CalibrateZero
		rjmp	ma10

ma02:		cpi	A, cKeyMax_on4c
		brne	ma03
		;нажатие 3 секунды KeyMax
		rcall	SetMorg4
		rcall	CalibrateMax
		rjmp	ma10

ma03:		rcall	isCalculate
;		rcall	isSetBufObmen
		
ma10:		rjmp	Main

;---------------------------------------------------------------------------------------------------------------
.macro Kt1_1
		sbi	p_Jp1, Jp1
.endmacro
.macro Kt1_0
		cbi	p_Jp1, Jp1
.endmacro
;.macro Kt2_1
;		sbi	p_Mosi, Mosi
;.endmacro
;.macro Kt2_0
;		cbi	p_Mosi, Mosi
;.endmacro
.macro Kt3_1
		sbi	p_Sck, Sck	;UART
.endmacro
.macro Kt3_0
		cbi	p_Sck, Sck
.endmacro
;---------------------------------------------------------------------------------------------------------------
; Мигаем светодиодиками
SetBufLed:	lds	A, st02c
		inc	A
		andi	A, 31
		sts	st02c, A
		lsr	A
		lsr	A
		cpi	A, 0
		brne	sbl01
		ldi	Ah, 0b00001001	;Зел
		rjmp	sbl08
sbl01:		cpi	A, 1
		brne	sbl02
		ldi	Ah, 0b00010010	;Кр
		rjmp	sbl08
sbl02:		cpi	A, 2
		brne	sbl03
		ldi	Ah, 0b00011011	;Жел
		rjmp	sbl08
sbl03:		cpi	A, 3
		brne	sbl04
		ldi	Ah, 0b00100100	;Зел Син
		rjmp	sbl08
sbl04:		cpi	A, 4
		brne	sbl05
		ldi	Ah, 0b00101101	;Кр -
		rjmp	sbl08
sbl05:		cpi	A, 5
		brne	sbl06
		ldi	Ah, 0b00110110	; - 
		rjmp	sbl08
sbl06:		cpi	A, 6
		brne	sbl07
		ldi	Ah, 0b00111111	;Малин Жел
		rjmp	sbl08
sbl07:		cpi	A, 7
		brne	sbl08
		ldi	Ah, 0b00000000	;Жел Зел
		rjmp	sbl08
		
sbl08:		ldi	A, 0
		sbrc	Ah, 0
		ldi	A, 128
		sts	BufLed+0, A
		ldi	A, 0
		sbrc	Ah, 1
		ldi	A, 128
		sts	BufLed+1, A
		ldi	A, 0
		sbrc	Ah, 2
		ldi	A, 128
		sts	BufLed+2, A
		ldi	A, 0
		sbrc	Ah, 3
		ldi	A, 128
		sts	BufLed+3, A
		ldi	A, 0
		sbrc	Ah, 4
		ldi	A, 128
		sts	BufLed+4, A
		ldi	A, 0
		sbrc	Ah, 5
		ldi	A, 128
		sts	BufLed+5, A
		ret		
;---------------------------------------------------------------------------------------------------------------
isCalculate:	sbrs	status, f_CycleComplit
		ret
		rcall	SetBufLed		; отработка 
		clr	KS
		ldi	A, 0x5a
		rcall	UART_send
		cbr	status, 1<<f_CycleComplit
		;находим длительность периода опоры vPeriodOpora = vDlitOpora / vNimpOpora
		lds	A,  vDlitOporal
		lds	Ah, vDlitOporah		;на 8 МГц интервал 1 секунды равем 0x7A1200
		lds	Ag, vDlitOporag
		lds	B,  vNimpOporal
		lds	Bh, vNimpOporah
		clr	Bg
		rcall	Divs24_24f
		sts	vPeriodOporah, Ah
		sts	vPeriodOporal, A	;2 байта целых и один дробный
		sts	vPeriodOporad, Dg
		;находим длительность периода изм.сигнала vPeriodIzmer = vDlitIzmer / vNimpIzmer
		lds	A,  vDlitIzmerl
		lds	Ah, vDlitIzmerh
		lds	Ag, vDlitIzmerg
		lds	B,  vNimpIzmerl
		lds	Bh, vNimpIzmerh
		clr	Bg
		rcall	Divs24_24f
		sts	vPeriodIzmerh, Ah
		sts	vPeriodIzmerl, A	;2 байта целых и один дробный
		sts	vPeriodIzmerd, Dg
		;корректируем длит.изм.сигнала с учетом ухода опоры vPeriodCalib = vPeriodIzmer * eOporaNorm / vPeriodOpora
		mov	Ag, Ah
		mov	Ah, A
		mov	A,  Dg
		lsr	Ag			;делим на 2, получая гарантированно положительное число Ag.7=0
		ror	Ah
		ror	A
		lds	B,  eOporaNormd
		lds	Bh, eOporaNorml
		lds	Bg, eOporaNormh
		rcall	Mul24s
		lds	B,  vPeriodOporad
		lds	Bh, vPeriodOporal
		lds	Bg, vPeriodOporah
		rcall	Divs48_24f
		;lsl	D			;умножаем на 2, компенсируя прежнее деление
		;rol	Dh
		;rol	Dg
		sts	vPeriodCalibh, Dg
		sts	vPeriodCalibl, Dh	;2 байта целых и один дробный
		sts	vPeriodCalibd, D
		;соотнести с текущей температурой
		;
		;
.equ	cIzmNormMax = 4000*4*256	;4000мкс * 8 / 2 и байт после запятой (*256)
.equ	cFreqMin =  400*8*16
.equ	cFreqMax = 4000*8*16
		;рассчитать период выходного сигнала
		sts	Nxl, D
		sts	Nxh, Dh
		sts	Nxg, Dg
		lds	A,  eIzmerNormd
		lds	Ah, eIzmerNorml
		lds	Ag, eIzmerNormh
		lsr	Ag			;делим на 2, получая гарантированно положительное число Ag.7=0
		ror	Ah
		ror	A
		sts	N1l, A
		sts	N1h, Ah
		sts	N1g, Ag
		ldi	A, low  (cIzmNormMax)
		sts	N2l, A
		ldi	A, high (cIzmNormMax)
		sts	N2h, A
		ldi	A, byte3(cIzmNormMax)
		sts	N2g, A
		ldi	A, low  (cFreqMin)
		sts	M1l, A
		ldi	A, high (cFreqMin)
		sts	M1h, A
		ldi	A, byte3(cFreqMin)
		sts	M1g, A
		ldi	A, low  (cFreqMax)
		sts	M2l, A
		ldi	A, high (cFreqMax)
		sts	M2h, A
		ldi	A, byte3(cFreqMax)
		sts	M2g, A
		rcall	Interpolat
		rcall	asr4
		;проверка на допустимые границы
		sbrs	Ag, 7
		rjmp	ica01
		ldi	A,  low (300*8)		;если отрицательное, берем 300 мкс
		ldi	Ah, high(300*8)
		rjmp	ica04
ica01:		tst	Ag
		breq	ica02
		ldi	A,  low (4000*8)	;если положительное и >65536, берем 4000 мкс
		ldi	Ah, high(4000*8)
		rjmp	ica04
ica02:		ldi	B,  low (300*8)		;проверка нижней границы
		ldi	Bh, high(300*8)
		cp	A,  B
		cpc	Ah, Bh
		brcs	ica03			;если меньше 300 мкс, берем 300 мкс
		ldi	B,  low (4000*8)	;проверка верхней границы
		ldi	Bh, high(4000*8)
		cp	A,  B
		cpc	Ah, Bh			;если больше 4000 мкс, берем 4000 мкс
		brcs	ica04
ica03:		mov	A,  B
		mov	Ah, Bh
ica04:		sts	vPeriodTokl, A
		sts	vPeriodTokh, Ah
		;передаем посылку из 10+21 байт, +KS
		ldi	Zl, low (eDelitOpora)
		ldi	Zh, high(eDelitOpora)
ica05:		ld	A, Z+
		rcall	UART_send
		cpi	Zl, low(eDelitOpora+10+25)
		brne	ica05
		mov	A, KS
		rcall	UART_send
		ret
;--------------------------------------------------------------------------------
;Передача байта A со стартовым и стоповым битом по 8 тактов на бит.
UART_send:	sub	KS, A
		sec	;формируем стоп-бит и передаем старт-бит
		;in	Ah, SREG
		cli
ua01:		Kt3_0	;передача "0"
		ror	A
		rjmp	ua02
ua02:		nop
		brcc	ua01
		clc
		Kt3_1	;передача "1"
		ror	A
		brne	ua02
		sei	;out	SREG, Ah
		rcall	ua03		;Stop-bit второй
ua03:		nop
		ret
;---------------------------------------------------------------------------------------------------------------
;isSetBufObmen:	sbrs	status, f_BufReady
;		ret			;выход, если буфер занят передачей, f_BufReady=0
;;Если буфер пустой, записываем очередную порцию данных
;; Буфер передается за 2+10+(10*9)=102 такта, при max=4мс это 0,408с
;		lds	A, vPeriodTokl	;период первичника вычисленный
;		sts	BufObmen+1, A
;		lds	A, vPeriodTokh
;		sts	BufObmen+2, A
;		lds	A, vN_Part
;		inc	A
;		andi	A, 0b00000011
;		sts	vN_Part, A
;		sts	BufObmen+0, A
;		cpi	A, 1
;		breq	SetPart2
;		cpi	A, 2
;		breq	SetPart3
;		cpi	A, 3
;		breq	SetPart4
;		;-------
;SetPart1:	lds	A,  eDelitIzmer		;делитель измерительного канала
;		lds	Ah, vPeriodIzmerd	;период измерительный канал, мкс/2048
;		lds	Ag, vPeriodIzmerl
;		lds	B,  vPeriodIzmerh
;		lds	Bh, vCodTempl		;код канала температуры
;		lds	Bg, vCodTemph
;		lds	D,  vCodVccl		;код канала напряжения питания
;		lds	Dh, vCodVcch
;		rjmp	sbo01
;		;-------
;SetPart2:	lds	A,  eDelitOpora		;делитель опорного канала
;		lds	Ah, vPeriodOporad	;период опорный канал, мкс/2048
;		lds	Ag, vPeriodOporal
;		lds	B,  vPeriodOporah
;		lds	Bh, vPeriodCalibd	;калиброванный с учетом ухода опоры период изм.канала, мкс/2048
;		lds	Bg, vPeriodCalibl
;		lds	D,  vPeriodCalibh
;		ldi	Dh, 0
;		rjmp	sbo01
;		;-------
;SetPart3:	lds	A,  eOporaNormd		;период опоры при температуре калибровки
;		lds	Ah, eOporaNorml
;		lds	Ag, eOporaNormh
;		lds	B,  eIzmerNormd		;
;		lds	Bh, eIzmerNorml
;		lds	Bg, eIzmerNormh
;		lds	D,  eTempNorml		;код температуры калибровки
;		lds	Dh, eTempNormh
;		rjmp	sbo01
;		;-------
;SetPart4:	clr	A
;		clr	Ah
;		clr	Ag
;		clr	B
;		clr	Bh
;		clr	Bg
;		clr	D
;		clr	Dh
;		;-------
;sbo01:		sts	BufObmen+3, A
;		sts	BufObmen+4, Ah
;		sts	BufObmen+5, Ag
;		sts	BufObmen+6, B
;		sts	BufObmen+7, Bh
;		sts	BufObmen+8, Bg
;		sts	BufObmen+9, D
;		sts	BufObmen+10,Dh
;		cbr	status, 1<<f_BufReady
;		ret
;---------------------------------------------------------------------------------------------------------------
; При нажатии кнопки вычисляется длительность периода и подбирается такой делитель OCR0A, чтобы период на выходе
; ОС0В равный ((OCR0A+1) * 2 * tимп), был около 312,5 мкс - 2500 тактов (3200 Гц) !!!!!! 300 mc
;
.equ	c300mkc	=1228800	;=300*8*2*256
Calibrate:	;запускаем цикл измерений с минимальными коэффициентами
		ldi	A, 2
		sts	eDelitOpora, A
		sts	eDelitIzmer, A
		ldi	A, 0
		sts	vFazIzm, A
		cbr	status, 1<<f_CycleComplit
		;ждем окончания измерения 180 мс
ca01:		sbrs	status, f_CycleComplit
		rjmp	ca01
		;находим длительность периода опоры vPeriodOpora = vDlitOpora / vNimpOpora
		lds	A,  vDlitOporal
		lds	Ah, vDlitOporah		;на 8 МГц интервал 1 секунды равем 0x7A1200
		lds	Ag, vDlitOporag
		lds	B,  vNimpOporal
		lds	Bh, vNimpOporah
		clr	Bg
		rcall	Divs24_24f
		;sts	eOporaNormh, Ah
		;sts	eOporaNorml, A		;2 байта целых и один дробный
		;sts	eOporaNormd, Dg
		mov	B,  Dg			
		mov	Bh, A
		mov	Bg, Ah
		ldi	A, byte3(c300mkc)
		mov	Ag, A
		ldi	Ah, high(c300mkc)
		ldi	A,  low (c300mkc)
		rcall	Divu24_24fo		;A = eDelitOpora = c300mkc / vPeriodOpora
		tst	Ag
		brne	ca02			;если больше 255, берем 255
		tst	Ah
		brne	ca02
		cpi	A, 2			;если меньше 2, берем 2
		brcc	ca03
		ldi	A, 2
		rjmp	ca03
ca02:		ldi	A, 255
ca03:		sts	eDelitOpora, A
		;находим длительность периода изм.сигнала vPeriodIzmer = vDlitIzmer / vNimpIzmer
		lds	A,  vDlitIzmerl
		lds	Ah, vDlitIzmerh
		lds	Ag, vDlitIzmerg
		lds	B,  vNimpIzmerl
		lds	Bh, vNimpIzmerh
		clr	Bg
		rcall	Divs24_24f
		;sts	eIzmerNormh, Ah
		;sts	eIzmerNorml, A		;2 байта целых и один дробный
		;sts	eIzmerNormd, Dg
		mov	B,  Dg			
		mov	Bh, A
		mov	Bg, Ah
		ldi	A, byte3(c300mkc)
		mov	Ag, A
		ldi	Ah, high(c300mkc)
		ldi	A,  low (c300mkc)
		rcall	Divu24_24fo		;A = eDelitIzmer = c300mkc / vPeriodOpora
		tst	Ag
		brne	ca04			;если больше 255, берем 255
		tst	Ah
		brne	ca04
		cpi	A, 2			;если меньше 2, берем 2
		brcc	ca05
		ldi	A, 2
		rjmp	ca05
ca04:		ldi	A, 255
ca05:		sts	eDelitIzmer, A
		;повторяем цикл измерения с вычисленными делителями
		ldi	A, 0
		sts	vFazIzm, A
		cbr	status, 1<<f_CycleComplit
		;ждем окончания измерения 180 мс
ca06:		sbrs	status, f_CycleComplit
		rjmp	ca06
		;находим длительность периода опоры vPeriodOpora = vDlitOpora / vNimpOpora
		lds	A,  vDlitOporal
		lds	Ah, vDlitOporah		;на 8 МГц интервал 1 секунды равем 0x7A1200
		lds	Ag, vDlitOporag
		lds	B,  vNimpOporal
		lds	Bh, vNimpOporah
		clr	Bg
		rcall	Divs24_24f
		sts	eOporaNormh, Ah
		sts	eOporaNorml, A		;2 байта целых и один дробный
		sts	eOporaNormd, Dg
		;находим длительность периода изм.сигнала vPeriodIzmer = vDlitIzmer / vNimpIzmer
		lds	A,  vDlitIzmerl
		lds	Ah, vDlitIzmerh
		lds	Ag, vDlitIzmerg
		lds	B,  vNimpIzmerl
		lds	Bh, vNimpIzmerh
		clr	Bg
		rcall	Divs24_24f
		sts	eIzmerNormh, Ah
		sts	eIzmerNorml, A		;2 байта целых и один дробный
		sts	eIzmerNormd, Dg
		;сохраняем текущую температуру как опорную
		lds	A,  vCodTempl
		lds	Ah, vCodTemph
		sts	eTempNorml, A
		sts	eTempNormh, Ah
		;запускаем обычный цикл измерений
		ldi	A, 0
		sts	vFazIzm, A
		cbr	status, 1<<f_CycleComplit
		;записываем как калибровочные: текущие делители, периоды и температуру.
		ldi	Ah, svDelitOpora	; Ah - адрес EEPROM, A - данные, после операции Ah= Ah+1
		ldi	Zl, low (eDelitOpora)
		ldi	Zh, high(eDelitOpora)
ca07:		ld	A, Z+
		rcall	WriteEE
		cpi	Zl, low (eDelitOpora+10)
		brne	ca07
		ret
;---------------------------------------------------------------------------------------------------------------
FlashErr:	rjmp	SetMorg

SetMorg1:	ldi	A, 1
		rjmp	SetMorg	
SetMorg2:	ldi	A, 2
		rjmp	SetMorg	
SetMorg3:	ldi	A, 3
		rjmp	SetMorg	
SetMorg4:	ldi	A, 4
		rjmp	SetMorg	
SetMorg5:	ldi	A, 5
SetMorg:	sts	vStMorg, A
		;ldi	A, 100
		;sts	vTimeMorg, A
		;nop
		ret
;----------------------------------------------------------------------------------------------
;		lds	A,  vNewKeys
;		lds	Ah, Keys
;		ldi	B,  0b00000001
;		eor	A, Ah
;		breq	


Inkey:		push	Ah
		push	B
		lds	A,  vNewKeys
		lds	Ah, Keys
		lds	B,  LastPressKey

		sbrc	A, bitKnZero
		rjmp	ink1
		sbrs	Ah, bitKnZero
		rjmp	ink2
		cbr	Ah, 1<<bitKnZero
		cpi	B, cKeyZero_on
		breq	ink1a
		ldi	A, cKeyZero_off	;отпускание KeyZero
		rjmp	ink14
ink1a:		ldi	A, cKeyZero_on_off ;нажатие-отпускание KeyZero до 4х секунд
		rjmp	ink14
ink1:		sbrc	Ah, bitKnZero
		rjmp	ink2
		sbr	Ah, 1<<bitKnZero
		;call	Beep01c
		ldi	A, cKeyZero_on	;нажатие KeyZero
		rjmp	ink14

ink2:		sbrc	A, bitKnMax
		rjmp	ink3
		sbrs	Ah, bitKnMax
		rjmp	ink4
		cbr	Ah, 1<<bitKnMax
		cpi	B, cKeyMax_on
		breq	ink3a
		ldi	A, cKeyMax_off	;отпускание KeyMax
		rjmp	ink14
ink3a:		ldi	A, cKeyMax_on_off ;нажатие-отпускание KeyMax до 4х секунд
		rjmp	ink14
ink3:		sbrc	Ah, bitKnMax
		rjmp	ink4
		sbr	Ah, 1<<bitKnMax
		;call	Beep01c
		ldi	A, cKeyMax_on	;нажатие KeyMax
		rjmp	ink14

ink4:		;изменений кнопок не было
		lds	A, stPressKey
		cpi	A, 30	;100мс * 30 = 3000 mc
		brcs	ink12
		cpi	B, cKeyMax_on
		brne	ink10
;		call	Beep2t
		ldi	A, cKeyMax_on4c
		rjmp	ink15
ink10:		cpi	B, cKeyZero_on
		brne	ink11
;		call	Beep2t
		ldi	A, cKeyZero_on4c
		rjmp	ink15
ink11:
ink12:		lds	A, stEscape	;если не было нажатий более 120 cек, выдаем код "Escape"
		cpi	A, 120
		brcs	ink13
		ldi	A, cKeyTimeOut
		rjmp	ink15
ink13:		clr	A
		rjmp	ink20

ink14:		ldi	B, 1
		sts	stPressKey, B	;при изменении кнопок включение счетчика времени нажатия
ink15:		sts	Keys, Ah
		clr	Ah
		sts	stEscape, Ah	;сбрасываем счетчик ненажатия 60 секунд
ink20:		sts	NowPressKey, A	;запоминаем текущее состояния нажатия (код) или нет (0)
		tst	A
		breq	ink21
		sts	LastPressKey, A	;если не ноль, запоминаем как последнюю нажатую
ink21:		pop	B
		pop	Ah
		ret

;--------------------------------------------------------------------
;Mx=M1+(Nx-N1)*(M2-M1)/(N2-N1)      где все переменные - integer_3_byte
;Результат в Ag:Ah:A. C=1, если переполнение.
Interpolat:	lds	A, Nxl
		lds	Ah, Nxh
		lds	Ag, Nxg
		lds	r0, N1l
		sub	A, r0
		lds	r0, N1h
		sbc	Ah, r0
		lds	r0, N1g
		sbc	Ag, r0
		clc
		brge	ia03
		sec
ia03:		ror	Ag
		ror	Ah
		ror	A	;Ag:Ah:A = (Nx-N1)/2 (знаково)
		
		lds	B, M2l
		lds	Bh, M2h
		lds	Bg, M2g
		lds	r0, M1l
		sub	B, r0
		lds	r0, M1h
		sbc	Bh, r0
		lds	r0, M1g
		sbc	Bg, r0	;Bg:Bh:B = (M2-M1)
		rcall	Mul24s

		lds	B, N2l
		lds	Bh, N2h
		lds	Bg, N2g
		lds	r0, N1l
		sub	B, r0
		lds	r0, N1h
		sbc	Bh, r0
		lds	r0, N1g
		sbc	Bg, r0
		clc
		brge	ia04
		sec
ia04:		ror	Bg
		ror	Bh
		ror	B	;Bg:Bh:B = (N2-N1)/2 (знаково)
		rcall	Divs48_24f

		lds	B, M1l
		lds	Bh, M1h
		lds	Bg, M1g
		clr	r0
		sbrc	Bg, 7
		dec	r0	;если М1 отрицательное, то расширяем FFFFFF
		add	D, B	;прибавляем M1
		adc	Dh, Bh
		adc	Dg, Bg
		adc	A, r0
		adc	Ah, r0
		adc	Ag, r0
		mov	r0, A	;xch Ag:Ah:A, Dg:Dh:D
		mov	A, D
		mov	D, r0
		mov	r0, Ah
		mov	Ah, Dh
		mov	Dh, r0
		mov	r0, Ag
		mov	Ag, Dg
		mov	Dg, r0
		
		sbrs	Ag,7
                rjmp    ia01    ;если Ag:A < 0 и Dg:D = 0xffffff, то выход. Иначе Ag:A = 0x800000
		mov	B, Dg
		cpi	B, 255
		brne	ia02
		cpi	Dh, 255
		brne	ia02
		cpi	D, 255
		brne	ia02
		clc
		ret

ia01:           mov     B, Dg   ;если Ag:A >= 0 и Dg:D = 0x000000 то выход. Иначе Ag:A = 0x7fffff
		cpi	B, 0
		brne	ia02
		cpi	Dh, 0
		brne	ia02
		cpi	D, 0
		brne	ia02
		clc
		ret

ia02:		mov	B, Dg
		sbrc	B, 7
		rjmp	ia05
		ldi	B, 0x7F
		mov	Ag, B
		ldi	Ah, 0xFF
		ldi	A, 0xFF
		clc
		ret
		
ia05:		ldi	B, 0x80
		mov	Ag, B
		ldi	Ah, 0
		ldi	A, 0
		clc
		ret
		;-----------
lsl4:		lsl	A
		rol	Ah
		rol	Ag
		lsl	A
		rol	Ah
		rol	Ag
		lsl	A
		rol	Ah
		rol	Ag
		lsl	A
		rol	Ah
		rol	Ag
		ret
		;-----------
asr4:		asr	Ag
		ror	Ah
		ror	A
		asr	Ag
		ror	Ah
		ror	A
		asr	Ag
		ror	Ah
		ror	A
		asr	Ag
		ror	Ah
		ror	A
		ret
;---------------------------------------------------------------------------
; "mpy16s" - 16x16 bit Signed Multiplication
;
; This subroutine multiplies signed the two 16-bit register variables 
; Ah:A and Bh:B.
; The result is placed in r1:r0:Ah:A.
; The routine is an implementation of Booth's algorithm. If all 32 bits
; in the result are needed, avoid calling the routine with
; -32768 ($8000) as multiplicand
;  
; Number of words	:16 + return
; Number of cycles	:210/226 (Min/Max) + return
; Low registers used	:None
; High registers used  :7 (A,Ah,B/A,Bh/Ah,r0,r1,st_mul)	
;---------------------------------------------------------------------------
Mul16s:		clr	r1		;clear result byte 3
		sub	r0, r0		;clear result byte 2 and carry
		ldi	stl,16		;init loop counter
m16s_1:		brcc	m16s_2		;if carry (previous bit) set
		add	r0, B		;    add multiplicand Low to result byte 2
		adc	r1, Bh		;    add multiplicand High to result byte 3
m16s_2:		sbrc	A, 0		;if current bit set
		sub	r0, B		;    sub multiplicand Low from result byte 2
		sbrc	A, 0		;if current bit set
		sbc	r1, Bh		;    sub multiplicand High from result byte 3
		asr	r1		;shift right result and multiplier
		ror	r0
		ror	Ah
		ror	A
		dec	stl		;decrement counter
		brne	m16s_1		;if not done, loop more	
		ret
;---------------------------------------------------------------------------
; "mpy24s" - 24x24 bit Signed Multiplication
;
; This subroutine multiplies signed the two 24-bit register variables 
; Ag:Ah:A and Bg:Bh:B.
; The result is placed in Ag:Ah:A:Dg:Dh:D
; The routine is an implementation of Booth's algorithm. If all 32 bits
; in the result are needed, avoid calling the routine with
; -32768 ($8000) as multiplicand
;---------------------------------------------------------------------------
Mul24s:		push	stl
		mov	D,  A
		mov	Dh, Ah
		mov	Dg, Ag
		clr	Ag		;clear result byte 5
		clr	Ah		;clear result byte 4
		sub	A,  A		;clear result byte 3 and carry
		ldi	stl,24		;init loop counter
m24s_1:		brcc	m24s_2		;if carry (previous bit) set
		add	A,  B		;    add multiplicand Low to result byte 3
		adc	Ah, Bh		;    add multiplicand High to result byte 4
		adc	Ag, Bg		;    add multiplicand High to result byte 5
m24s_2:		sbrs	D, 0		;if current bit set
		rjmp	m24s_3
		sub	A, B		;    sub multiplicand Low from result byte 3
		sbc	Ah, Bh		;    sub multiplicand High from result byte 4
		sbc	Ag, Bg		;    sub multiplicand High from result byte 5
m24s_3:		asr	Ag		;shift right result and multiplier
		ror	Ah
		ror	A
		ror	Dg
		ror	Dh
		ror	D
		dec	stl		;decrement counter
		brne	m24s_1		;if not done, loop more
		pop	stl
		ret
;---------------------------------------------------------------------------
Divu24_24fo:	clr	D		;деление положительных с округлением
		mov	Dh, D
		mov	Dg, D
		rcall	Divs48_24f
		sbrs	Dg, 7
		ret
		ldi	D, -1		;если после А следующий младший бит =1, то прибавляем 1
		sub	A,  D
		sbc	Ah, D
		sbc	Ag, D
		ret
;---------------------------------------------------------------------------
Divs24_24f:	clr	D
		mov	Dh, D
		mov	Dg, D
		sbrs	Ag, 7
		rjmp	Divs48_24f
		dec	D
		dec	Dh
		dec	Dg
;---------------------------------------------------------------------------
; Ag:Ah:A:Dg:Dh:D (dividend) and Bg:Bh:B (divisor). 
; The result is placed in Ag:Ah:A:Dg:Dh:D and the remainder in r2:r1,r0
;
Divs48_24f:	push	stl
		ser	stl
		mov	r0, Ag		;move dividend High to sign register
		eor	r0, Bg		;xor divisor High with sign register
		bst	r0, 7
		mov	r0, Ag
		rol	r0
		brcc	d24s_1		;if MSB in dividend set
		com	Ag		;    change sign of dividend
		com	Ah
		com	A
		com	Dg
		com	Dh
		com	D
		sub	D, stl
		sbc	Dh, stl
		sbc	Dg, stl
		sbc	A, stl
		sbc	Ah, stl
		sbc	Ag, stl
d24s_1:		mov	r0, Bg
		rol	r0
		brcc	d24s_2		;if MSB in divisor set
		com	Bg		;    change sign of divisor
		com	Bh
		com	B
		sub	B, stl
		sbc	Bh, stl
		sbc	Bg, stl
d24s_2:		clr	r0		;Clear remainder Low byte
		clr	r1
		sub	r2, r2		;Clear remainder High byte and carry
		push	A
		ldi	A, 49		;init loop counter
		mov	r3, A
		pop	A

d24s_3:		rol	D		;shift left dividend
		rol	Dh
		rol	Dg
		rol	A
		rol	Ah
		rol	Ag
		dec	r3		;decrement counter
		brne	d24s_5		;if done
		brtc	d24s_4		;if T=1, then change sign of result
		com	Ag
		com	Ah
		com	A
		com	Dg
		com	Dh
		com	D
		sub	D, stl
		sbc	Dh, stl
		sbc	Dg, stl
		sbc	A, stl
		sbc	Ah, stl
		sbc	Ag, stl
d24s_4:		pop	stl
		ret

d24s_5:		rol	r0		;shift dividend into remainder
		rol	r1
		rol	r2
		sub	r0, B		;remainder = remainder - divisor
		sbc	r1, Bh		;
		sbc	r2, Bg		;
		brcc	d24s_6		;if result negative
		add	r0, B		;    restore remainder
		adc	r1, Bh
		adc	r2, Bg
		clc			;    сlear carry to be shifted into result
		rjmp	d24s_3		;else
d24s_6:		sec			;    set carry to be shifted into result
		rjmp	d24s_3

;-----------------------------------------------------------------
; Ah - адрес EEPROM, A - данные, после операции Ah=+1
WriteEE:	sbic	EECR, EEPE
		rjmp	WriteEE
		out	EEARL, Ah
		push	Ah
		clr	Ah
		out	EECR, Ah
		sbi	EECR, EERE
		nop
		in	Ah, EEDR
		cp	A, Ah
		breq	wee1		;запись, только если значения не совпадают
		wdr
		out	EEDR, A
		in	Ah, SREG
		cli
		sbi	EECR, EEMPE
		sbi	EECR, EEPE
		out	SREG, Ah
wee1:		pop	Ah
		inc	Ah
		ret
;-----------------------------------------------------------------
; Ah - адрес EEPROM, A - данные, после операции Ah=+1
ReadEE:		sbic	EECR, EEPE
		rjmp	ReadEE
		out	EEARL, Ah
		sbi	EECR, EERE
		nop
		in	A, EEDR
		inc	Ah
		ret
;===============================================================================================================		
;
; ПРОЦЕДУРЫ ПРЕРЫВАНИЙ И ИХ ПОДПРОГРАММЫ
;
;---------------------------------------------------------------------------------------------------------------
; Прерывания с частотой 5 мс
TIM1_COMPA:	in	R_sreg, SREG
		in	Fl, OCR1AL
		in	Fh, OCR1AH
		subi	Fl, low (-5000*8)
		sbci	Fh, high(-5000*8)
		out	OCR1AH, Fh
		out	OCR1AL, Fl
		push	R_sreg
		sei
		push	Ah
		push	A
		rcall	CtrlIzm		;отрабатываем цикл измерений
		rcall	Int_5mc		;переход на обработчик временных интервалов (с низким приоритетом)
		pop	A
		pop	Ah
		cli
		pop	R_sreg
		out	SREG, R_sreg
		reti
;---------------------------------------------------------------------------------------------------------------
;TIM1_COMPB:	in	R_sreg, SREG
;;		Kt2_0
;		sbic	i_Tok, Tok
;		rjmp	setPause
;setActive:	;бит переключился в "0" - задаем длительность периода без защитной паузы
;		lds	Fl, vNextTCNT1l
;		lds	Fh, vNextTCNT1h
;		out	OCR1BH, Fh
;		out	OCR1BL, Fl
;		
;		rjmp	endTim1
;setPause:	;бит переключился в "1" - задаем длительность защитной паузы (+передача цифры)
;		in	Fl, OCR1BL 
;		in	Fh, OCR1BH
;		lds	iG, vPeriodTokl	;длительность периода
;		add	iG, Fl
;		sts	vNextTCNT1l, iG	;время следующего совпадения
;		lds	iG, vPeriodTokh
;		adc	iG, Fh
;		sts	vNextTCNT1h, iG
;		;задаем длительность защитной паузы (ключ открыт)
;		sbrc	status, f_SendBit
;		rjmp	T1cb1
;		subi	Fl, low (-100*8) ;если передаваемый бит=0, то защитная пауза = 100 мкс
;		sbci	Fh, high(-100*8)
;		rjmp	T1cb2
;T1cb1:		subi	Fl, low (-150*8) ;если передаваемый бит=1, то защитная пауза = 150 мкс
;		sbci	Fh, high(-150*8)
;T1cb2:		out	OCR1BH, Fh
;		out	OCR1BL, Fl
;		;вычисляем следующий передаваемый бит
;		cbr	status, 1<<f_SendBit	;передаваемый бит в ноль
;		lds	Fh, stSendBit
;		cpi	Fh, ((10+1)*16)	;11 байт
;		brcc	T1cb5		;если все биты переданы, начало цикла
;		inc	Fh
;		sts	stSendBit, Fh	;иначе инкрементируем счетчик 
;		cpi	Fh, 12	;!!!!
;		brcs	T1cb9		;в первых 12 битах передаем номер порции 2 бит и нули 10 бит
;		breq	T1cb4		;затем передача межбайтового бита "1"
;		mov	Fl, Fh
;		andi	Fl, 0b00001111	;для остальных байт передаем 8 бит и 1 межбайтовый "1"
;		cpi	Fl, 9
;		brcs	T1cb9
;T1cb4:		;передача межбайтового бита "1"
;		sbr	status, 1<<f_SendBit	;передаваемый бит в единицу
;		andi	Fh, 0b11110000
;		subi	Fh, -16
;		sts	stSendBit, Fh
;		ld	iSendByte, X+
;		rjmp	endTim1
;T1cb5:		;начало цикла
;		sbrc	status, f_BufReady
;		rjmp	T1cb8
;		;если готовы данные для передачи (f_BufReady=0), переписываем их в BufDataSend из BufObmen
;		push	Zh			;иначе повторно передаем прошлые данные
;		push	Zl
;		ldi	Zl, low (BufObmen)
;		ldi	Zh, high(BufObmen)
;		ldi	Xl, low (BufDataSend)
;		ldi	Xh, high(BufDataSend)
;T1cb6:		ld	Fl, Z+
;		st	X+, Fl
;		cpi	Xl, low(BufDataSend+11)
;		brne	T1cb6
;		pop	Zl
;		pop	Zh
;
;		sbr	status, 1<<f_BufReady
;T1cb8:		;инициализация передачи
;		ldi	Fh, 0
;		sts	stSendBit, Fh
;		ldi	Xl, low (BufDataSend)
;		ldi	Xh, high(BufDataSend)
;		ld	iSendByte, X+
;T1cb9:		;передача байтов из буфера
;		sbrc	iSendByte, 0
;		sbr	status, 1<<f_SendBit	;передаваемый бит в единицу
;		lsr	iSendByte
;;		rjmp	endTim1
;endTim1:	out	SREG, R_sreg
;;		Kt2_1
;		reti
;---------------------------------------------------------------------------------------------------------------
; Низкоуровневые прерывания с частотой 5 мс
Int_5mc:	lds	A, st5mc
		dec	A
		brne	i501
		;раз в секунду
		lds	Ah, stEscape
		inc	Ah
		sts	stEscape, Ah
		;конец блока раз в секунду
		ldi	A, 200		;1000 мс
i501:		sts	st5mc, A
		;отработка моргания светодиода
		lds	A, vTimeMorg
		cpi	A, 0
		brne	it02		;если vTimeMorg>0 то decrement (1 раз в 1,28 секунды)
		lds	Ah, vStMorg
		cpi	Ah, 0
		breq	it03
		dec	Ah		;если vStMorg>0 то dec vStMorg, vTimeMorg=127
		sts	vStMorg, Ah
		ldi	A, 128
it02:		dec	A
		sts	vTimeMorg, A
it03:
		;отработка дребезга кнопок
		clr	A
		sbis	i_Key, KeyZero	 ;читаем порты кнопок
		sbr	A, 1<<bitKnZero
		sbis	i_Key, KeyMax
		sbr	A, 1<<bitKnMax
		lds	Ah, oldKeys
		cp	A, Ah
		breq	it07
		;состояние кнопок отличается от запомненного
		sts	oldKeys, A	;запоминаем измененное состояние кнопок в качестве предыдущего
		ldi	Ah, 0		;сбрасываем счетчик совпадений 
		rjmp	it08
it07:		;состояние кнопок не изменилось
		lds	Ah, stStateKey
		cpi	Ah, 4		;4 = 20 мс
		brcs	it08
		sts	vNewKeys, A	;если состояние кнопок не менялось 20mc, запоминаем их в vNewKeys
		rjmp	it09
it08:		inc	Ah
		sts	stStateKey, Ah
it09:
		;инкремент stKeyPress раз в 100 мс, если не равен 0 и 255
		lds	A, st100mc
		inc	A
		cpi	A, 20
		brcs	it11
		ldi	A, 0
		lds	Ah, stPressKey
		cpi	Ah, 0
		breq	it11
		inc	Ah
		breq	it11
		sts	stPressKey, Ah
it11:		sts	st100mc, A
		;отработка АЦП температуры и напряжения
		lds	Ah, stSumAdc
		mov	A, Ah
		andi	A, 0b01111111
		cpi	A, 2
		brcs	it13		;два первых преобразования пропускаем
		sbrc	Ah, 7		;старший бит stSumAdc: 0-Temp, 1-Vcc
		rjmp	it12
		;читаем результат измерения температуры
		in	A,  ADCL
	;sts	vCodTempl, A
		lds	Ah, vSumTmpl
		add	A, Ah
		sts	vSumTmpl, A
		in	A,  ADCH
	;sts	vCodTemph, A
		lds	Ah, vSumTmph
		adc	A, Ah
		sts	vSumTmph, A
		ldi	A, 0
		lds	Ah, vSumTmpg
		adc	A, Ah
		sts	vSumTmpg, A
		rjmp	it13
it12:		;читаем результат измерения Vcc
		in	A,  ADCL
	;sts	vCodVccl, A
		lds	Ah, vSumVccl
		add	A, Ah
		sts	vSumVccl, A
		in	A,  ADCH
	;sts	vCodVcch, A
		lds	Ah, vSumVcch
		adc	A, Ah
		sts	vSumVcch, A
		ldi	A, 0
		lds	Ah, vSumVccg
		adc	A, Ah
		sts	vSumVccg, A
it13:		;запускаем новое измерение
		lds	Ah, stSumAdc
		inc	Ah
		sts	stSumAdc, Ah
		cpi	Ah, 0
		breq	it14
		cpi	Ah, 128
		brne	it15
		;переключаем на измерение Vcc
		ldi	A, (1<<MUX5)|(1<<MUX0)	;канал intRef, Ref=Vcc
		out	ADMUX, A
		rjmp	it15
it14:		;переключаем на измерение температуры
		ldi	A, (1<<REFS1)|(1<<MUX5)|(1<<MUX1)	;канал 8 (t), Ref=int (1,1 B)
		out	ADMUX, A
		;через 256 циклов переписываем усредненный результат
	;rjmp	it15
		lds	A,  vSumTmph
		lds	Ah, vSumTmpg
		sts	vCodTempl, A
		sts	vCodTemph, Ah
		lds	A,  vSumVcch
		lds	Ah, vSumVccg
		sts	vCodVccl, A
		sts	vCodVcch, Ah
		ldi	A, 0
		sts	vSumTmpl, A
		sts	vSumTmph, A
		sts	vSumTmpg, A
		sts	vSumVccl, A
		sts	vSumVcch, A
		sts	vSumVccg, A
it15:		;запуск преобразования
		ldi	A, 1<<ADLAR
		out	ADCSRB, A
		ldi	A, (1<<ADEN)|(1<<ADSC)|(1<<ADIF)|(1<<ADPS2)|(1<<ADPS1)	;одиночное преобразование
		out	ADCSRA, A	;Fтакт=125кГц, Fadc=9,6 кГц, сбрасываем флаг готовности
		ret
;---------------------------------------------------------------------------------------------------------------
; отрабатываем цикл измерений с периодом 5 мс
;---------------------------------------------------------------------------------------------------------------
; Фаза измерения vFazIzm: 0-запуск цикла, 1-измерение опоры, 2-измерение сигнала, 3-конец измерения
; - для запуска измерения надо присвоить vFazIzm=0 и ждать, когда vFazIzm=3. 
; - результат в vDlitOpora / vNimpOpora  и в vDlitIzmer / vNimpIzmer
CtrlIzm:	lds	A, vFazIzm
		cpi	A, 0
		brne	ci02

ci01:		;Начало цикла измерения опорного канала. Полный цикл измерения 20 + 180 = 200 мс
		ldi	A, 1
		sts	vFazIzm, A
		sbi	d_Fxi, Fxi
		sbi	d_Chnl, Chnl
		cbi	p_Chnl, Chnl	;Chnl = 0 - опорный канал
		ldi	A, 4		; 4 x 5мс = 20 мс (время измерений опорного канала)
		sts	vst_x5mc, A
		lds	A, eDelitOpora	;>0
		rjmp	cikl_new

ci02:		cpi	A, 1
		brne	ci05
		;Цикл измерения опорного канала.
		lds	A, vst_x5mc
		dec	A
		sts	vst_x5mc, A
		breq	ci03
		rjmp	ci10

ci03:		;конец цикла измерения опорного канала
		in	Ah, SREG	;запоминаем флаг I
		cli
		lds	A, StartICR1l	;находим rICR - StartICR1 
		sub	rICRl, A
		lds	A, StartICR1h
		sbc	rICRh, A
		ldi	A, 0
		sbc	rICRg, A	
		sts	vTemp1, rICRl	;временно запоминаем длительность импульсов опоры
		sts	vTemp2, rICRh
		sts	vTemp3, rICRg
		sts	vTemp4, Yl	;временно запоминаем число импульсов опоры
		sts	vTemp5, Yh
		;
ci04:		;Начало цикла измерения измерительного канала.
		ldi	A, 2
		sts	vFazIzm, A
		sbi	p_Chnl, Chnl	;Chnl = 1 - измерительный канал
		ldi	A, 36		;36 x 5мс = 180 мс (во время измерений измерительного канала)
		sts	vst_x5mc, A
		lds	A, eDelitIzmer	;>0
		rjmp	cikl_new

ci05:		cpi	A, 2
		brne	ci06
		;Цикл измерения измерительного канала.
		lds	A, vst_x5mc
		dec	A
		sts	vst_x5mc, A
		breq	ci07
ci06:		rjmp	ci10

		;конец цикла измерения измерительного канала
ci07:		in	Ah, SREG	;запоминаем флаг I
		cli
		lds	A, StartICR1l	;находим rICR - StartICR1 
		sub	rICRl, A
		lds	A, StartICR1h
		sbc	rICRh, A
		ldi	A, 0
		sbc	rICRg, A	
		sts	vDlitIzmerl, rICRl ;запоминаем длительность импульсов измерительного канала
		sts	vDlitIzmerh, rICRh
		sts	vDlitIzmerg, rICRg
		sts	vNimpIzmerl, Yl	   ;запоминаем число импульсов измерительного канала
		sts	vNimpIzmerh, Yh
		out	SREG, Ah	;восстанавливаем прерывания

		lds	A, vTemp1	;запоминаем длительность импульсов опоры
		sts	vDlitOporal, A
		lds	A, vTemp2
		sts	vDlitOporah, A
		lds	A, vTemp3
		sts	vDlitOporag, A
		lds	A, vTemp4
		sts	vNimpOporal, A	;запоминаем число импульсов опоры
		lds	A, vTemp5
		sts	vNimpOporah, A

		sbr	status, 1<<f_CycleComplit ;цикл измерения закончен, данные сохранены (10 байт)
		rjmp	ci01		;начинаем новый цикл измерения опорного и изм.каналов

;---------------------------------------
; Таймер 0 считает импульсы на входе Т0 (пин 10) и выдает меандр на OC0B с периодом F(T0) / (2 х OCR0A)
; в A - коэфф.деления
cikl_new:	in	Ah, SREG	;запоминаем флаг I
		cli			;запрещаем прерывания - критическая зона кода
		sbis	p_Chnl, Chnl
		rcall	SendBufLed
		dec	A
		out	OCR0A, A	;на выходе OC0B и ICP1 меандр частотой Fвх / (2 * vDelitOporn)
		dec	A
		out	TCNT0, A	;push	A
		ldi	A, (0<<COM0A1)|(0<<COM0A0)|(0<<COM0B1)|(1<<COM0B0)|(1<<WGM01)|(0<<WGM00)
		out	TCCR0A, A	;Pin OC0A = off, pin OC0B = Toggle OC0B on Compare Match, CTC mode.
		ldi	A, (0<<FOC0B)|(0<<WGM02)|(1<<CS02)|(1<<CS01)|(1<<CS00)
		sbic	i_Fxi, Fxi	;если Fxi=1, то сбросить в 0 путем установки FOC0B=1
		ldi	A, (1<<FOC0B)|(0<<WGM02)|(1<<CS02)|(1<<CS01)|(1<<CS00)
		out	TCCR0B, A	;External clock source on T0 pin. Clock on rising edge.
		sbr	status, 1<<f_start ;для прерываний TIM1_CAPT признак первоначального захвата фронта ICP
		sbi	TIFR1, ICF1	;сбрасываем флаг прерывания
;		Kt2_0
		out	SREG, Ah	;восстанавливаем прерывания
ci10:		
		ret
;-----------------------------------------------------------------------------
; Передача буфера BufLed в линейку светодиодов WS2812b
; 0 передается 3 такта "1" + 7 тактов "0" на частоте 8.000 MГц
; 1 передается 7 тактов "1" + 3 такта "0"
.equ	LenLn=2	;число светодиодов в цепочке
SendBufLed:	;push	Zh
		;push	Zl
		push	Yh
		push	Yl
		push	r0
		push	A
		sbi	d_Led, Leds
		ldi	Yl, low (BufLed);начало буфера
		ldi	Yh, high(BufLed)
		;ldi	Zl, low (LenLn*3);длина буфера, байт
		;ldi	Zh, high(LenLn*3)
		ldi	A, 1<<Leds	;маска инверсии бит
		;цикл передачи байта
SendByteLed:	ld	r0, Y+		;Fosc = 8MHz
		sec
		rol	r0
SendBitLed:	;цикл передачи бита
		out	i_Led, A	;0.125
		nop			;0.250 для 16МГц еще 3 nop
		brcs	sbled		;0.375
		out	i_Led, A	;0.500
sbled:		nop			;0.625 для 16МГц еще 4 nop
		lsl	r0		;0.750
		cbi	p_Led, Leds	;1.000
		brne	SendBitLed	;1.250
		 cpi	Yl, low (BufLed+LenLn*3) ;конец буфера?
		 brne	SendByteLed
		 cpi	Yh, high(BufLed+LenLn*3)
;		sbiw	Zh:Zl, 1
		brne	SendByteLed
		pop	A
		pop	r0
		pop	Yl
		pop	Yh
		;pop	Zl
		;pop	Zh
		ret

;---------------------------------------------------------------------------------------------------------------
; Прерывание по ЗАХВАТУ ICP
; Запоминаем старое oIcr1=rIcr1, читаем новое rIcr1=ICR1
; если f_start=1, то StartICR1=rIcr1, rIcrg=0, счетчик периодов Yh:Yl=0, f_start=0
; иначе Y=Y+1. Если rIcr1<oIcr1, то inc(rIcrg)
;
TIM1_CAPT:	in	R_sreg, SREG		;выполняется 3.5 мкс первое, 3.0 мкс последующие
		Kt1_0
		movw	oICRh:oICRl,rICRh:rICRl	;запоминаем старое значение
		in	rICRl, ICR1L		;читаем новое значение
		in	rICRh, ICR1H
		sbrs	status, f_start
		rjmp	t1cp1
		;первоначальный захват фронта ICP
;		Kt2_1
		cbr	status, 1<<f_start
		clr	rICRg
		clr	Yh
		clr	Yl
		sts	StartICR1l, rICRl
		sts	StartICR1h, rICRh
		rjmp	t1cp2
t1cp1:		;последующие захваты фронта ICP
		adiw	Yh:Yl, 1		; Yh:Yl - число периодов
		cp	rICRl, oICRl		; rICR-StartICR1 = длительность Y периодов
		cpc	rICRh, oICRh
		brcc	t1cp2
		inc	rICRg
t1cp2:		out	SREG, R_sreg
		Kt1_1
		reti
; На интервале 160мс (8 периодов сети) при частоте ниже 50 кГц (обеспечиваается делителем Т0)
; будет максимум 8000 периодов
