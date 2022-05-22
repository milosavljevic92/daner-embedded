
; PICBASIC PRO(TM) Compiler 3.0.6.1, (c) 1998, 2012 microEngineering Labs, Inc. All Rights Reserved. 
MPASMX_USED			EQU	1

#define	PBP_PATH	C:\PBP3\




        NOLIST
    ifdef PM_USED
        LIST
        "Error: PM does not support this device.  Use MPASM."
        NOLIST
    else
        LIST
        LIST p = 18F2550, r = dec, w = -311, w = -230, f = inhx32
        INCLUDE "P18F2550.INC"	; MPASM  Header
        NOLIST
    endif
        LIST
EEPROM_START	EQU	0F00000h
BLOCK_SIZE	EQU	32


PBP_HARDWAREDEF  macro
  endm


      ;  __CONFIG    _CONFIG1L, _PLLDIV_5_1L & _CPUDIV_OSC1_PLL2_1L & _USBDIV_2_1L
      ;  __CONFIG    _CONFIG1H, _FOSC_HSPLL_HS_1H & _FCMEN_OFF_1H & _IESO_OFF_1H
      ;  __CONFIG    _CONFIG2L, _PWRT_OFF_2L & _BOR_ON_2L & _BORV_3_2L & _VREGEN_ON_2L
      ;  __CONFIG    _CONFIG2H, _WDT_ON_2H & _WDTPS_512_2H
      ;  __CONFIG    _CONFIG3H, _CCP2MX_ON_3H & _PBADEN_OFF_3H & _LPT1OSC_OFF_3H & _MCLRE_ON_3H
      ;  __CONFIG    _CONFIG4L, _STVREN_ON_4L & _LVP_OFF_4L & _XINST_OFF_4L



; Define statements.
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00013	DEFINE OSC 20
#define		OSC		 20
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00003	DEFINE  INTHAND       INT_ENTRY_H  ' Context saving for High Priority Ints
#define		INTHAND		       INT_ENTRY_H  
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00004	DEFINE  INTLHAND      INT_ENTRY_L  ' Context saving for Low Priority Ints
#define		INTLHAND		      INT_ENTRY_L  
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00020	DEFINE  LCD_BITS 4
#define		LCD_BITS		 4
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00021	DEFINE  LCD_DREG PORTB
#define		LCD_DREG		 PORTB
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00022	DEFINE  LCD_DBIT 0
#define		LCD_DBIT		 0
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00023	DEFINE  LCD_RSREG PORTB
#define		LCD_RSREG		 PORTB
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00024	DEFINE  LCD_RSBIT 4
#define		LCD_RSBIT		 4
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00025	DEFINE  LCD_EREG PORTB
#define		LCD_EREG		 PORTB
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00026	DEFINE  LCD_EBIT 5
#define		LCD_EBIT		 5
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00027	DEFINE  LCD_LINES 2
#define		LCD_LINES		 2
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00028	DEFINE  LCD_COMMANDUS 2000
#define		LCD_COMMANDUS		 2000
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00029	DEFINE  LCD_DATAUS 40
#define		LCD_DATAUS		 40
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00032	DEFINE  SPWM_FREQ  170        
#define		SPWM_FREQ		  170        
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00033	DEFINE  SPWM_RES   100
#define		SPWM_RES		   100

RAM_START       		EQU	00000h
RAM_END         		EQU	007FFh
RAM_BANKS       		EQU	00008h
BANK0_START     		EQU	00060h
BANK0_END       		EQU	000FFh
BANK1_START     		EQU	00100h
BANK1_END       		EQU	001FFh
BANK2_START     		EQU	00200h
BANK2_END       		EQU	002FFh
BANK3_START     		EQU	00300h
BANK3_END       		EQU	003FFh
BANK4_START     		EQU	00400h
BANK4_END       		EQU	004FFh
BANK5_START     		EQU	00500h
BANK5_END       		EQU	005FFh
BANK6_START     		EQU	00600h
BANK6_END       		EQU	006FFh
BANK7_START     		EQU	00700h
BANK7_END       		EQU	007FFh
BANKA_START     		EQU	00000h
BANKA_END       		EQU	0005Fh

; C:\PBP3\PBPPI18L.RAM     	00028	A00000	FLAGS   VAR     BYTE BANKA SYSTEM       ' Static flags
FLAGS           		EQU	RAM_START + 000h
; C:\PBP3\PBPPI18L.RAM     	00027	A00001	GOP     VAR     BYTE BANKA SYSTEM       ' Gen Op Parameter
GOP             		EQU	RAM_START + 001h
; C:\PBP3\PBPPI18L.RAM     	00016	A00002	R4      VAR     WORD BANKA SYSTEM       ' System Register
R4              		EQU	RAM_START + 002h
; C:\PBP3\PBPPI18L.RAM     	00017	A00004	R5      VAR     WORD BANKA SYSTEM       ' System Register
R5              		EQU	RAM_START + 004h
; C:\PBP3\PBPPI18L.RAM     	00018	A00006	R6      VAR     WORD BANKA SYSTEM       ' System Register
R6              		EQU	RAM_START + 006h
; C:\PBP3\PBPPI18L.RAM     	00019	A00008	R7      VAR     WORD BANKA SYSTEM       ' System Register
R7              		EQU	RAM_START + 008h
; C:\PBP3\PBPPI18L.RAM     	00020	A0000A	R8      VAR     WORD BANKA SYSTEM       ' System Register
R8              		EQU	RAM_START + 00Ah
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00021	A0000C	INT_Flags   VAR BYTE  BANKA  SYSTEM
INT_Flags       		EQU	RAM_START + 00Ch
; C:\PBP3\PBPPI18L.RAM     	00023	A0000D	RM1     VAR     BYTE BANKA SYSTEM       ' Pin 1 Mask
RM1             		EQU	RAM_START + 00Dh
; C:\PBP3\PBPPI18L.RAM     	00026	A0000E	RM2     VAR     BYTE BANKA SYSTEM       ' Pin 2 Mask
RM2             		EQU	RAM_START + 00Eh
; C:\PBP3\PBPPI18L.RAM     	00021	A0000F	RR1     VAR     BYTE BANKA SYSTEM       ' Pin 1 Register
RR1             		EQU	RAM_START + 00Fh
; C:\PBP3\PBPPI18L.RAM     	00024	A00010	RR2     VAR     BYTE BANKA SYSTEM       ' Pin 2 Register
RR2             		EQU	RAM_START + 010h
; C:\PBP3\PBPPI18L.RAM     	00022	A00011	RS1     VAR     BYTE BANKA SYSTEM       ' Pin 1 Bank
RS1             		EQU	RAM_START + 011h
; C:\PBP3\PBPPI18L.RAM     	00025	A00012	RS2     VAR     BYTE BANKA SYSTEM       ' Pin 2 Bank
RS2             		EQU	RAM_START + 012h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00006	A00013	wsave       var byte  BANKA  SYSTEM   ' location for WREG
wsave           		EQU	RAM_START + 013h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00019	A00014	RetAddrH    VAR BYTE[3]  BANKA  SYSTEM  ' 21-bit return address Hi Pri.
RetAddrH        		EQU	RAM_START + 014h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00020	A00017	RetAddrL    VAR BYTE[3]  BANKA  SYSTEM  '                       Lo Pri.
RetAddrL        		EQU	RAM_START + 017h
; C:\PBP3\PBPPI18L.RAM     	00012	A0001A	R0      VAR     LONG BANKA SYSTEM       ' System Register
R0              		EQU	RAM_START + 01Ah
; C:\PBP3\PBPPI18L.RAM     	00013	A0001E	R1      VAR     LONG BANKA SYSTEM       ' System Register
R1              		EQU	RAM_START + 01Eh
; C:\PBP3\PBPPI18L.RAM     	00014	A00022	R2      VAR     LONG BANKA SYSTEM       ' System Register
R2              		EQU	RAM_START + 022h
; C:\PBP3\PBPPI18L.RAM     	00015	A00026	R3      VAR     LONG BANKA SYSTEM       ' System Register
R3              		EQU	RAM_START + 026h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00001	A0002A	'----------------------------------Podesavanje FUSOVA--------------------------------------------
T1              		EQU	RAM_START + 02Ah
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00001	A0002E	'----------------------------------Podesavanje FUSOVA--------------------------------------------
T2              		EQU	RAM_START + 02Eh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00001	A00032	'----------------------------------Podesavanje FUSOVA--------------------------------------------
PB01            		EQU	RAM_START + 032h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00087	A00033	BezTastera            var word      
_BezTastera      		EQU	RAM_START + 033h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00089	A00035	KMinuta               var word
_KMinuta         		EQU	RAM_START + 035h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00088	A00037	PMinuta               var word
_PMinuta         		EQU	RAM_START + 037h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00090	A00039	TMinuta               var word
_TMinuta         		EQU	RAM_START + 039h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00112	A0003B	DECmin                var byte
_DECmin          		EQU	RAM_START + 03Bh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00113	A0003C	DECsat                var byte
_DECsat          		EQU	RAM_START + 03Ch
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00114	A0003D	DECSec                var byte
_DECSec          		EQU	RAM_START + 03Dh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\SPWM_INT.BAS	00001	A0003E	DutyCount   VAR byte
_DutyCount       		EQU	RAM_START + 03Eh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00109	A0003F	DveTacke              var byte
_DveTacke        		EQU	RAM_START + 03Fh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00103	A00040	EPROM                 var byte
_EPROM           		EQU	RAM_START + 040h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00101	A00041	i                     var byte
_i               		EQU	RAM_START + 041h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00105	A00042	Kanal                 var byte
_Kanal           		EQU	RAM_START + 042h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00108	A00043	Kursor                var byte
_Kursor          		EQU	RAM_START + 043h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00104	A00044	Meni                  var byte           
_Meni            		EQU	RAM_START + 044h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00111	A00045	minut                 var byte
_minut           		EQU	RAM_START + 045h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00123	A00046	MinutKraj             var byte
_MinutKraj       		EQU	RAM_START + 046h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00122	A00047	MinutPocetak          var byte
_MinutPocetak    		EQU	RAM_START + 047h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00132	A00048	NoviMinut             var byte
_NoviMinut       		EQU	RAM_START + 048h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00128	A00049	NovostanjeRTC         var byte
_NovostanjeRTC   		EQU	RAM_START + 049h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00130	A0004A	osvetljenje           var byte
_osvetljenje     		EQU	RAM_START + 04Ah
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00131	A0004B	osvetljenjeOdrzavanje var byte
_osvetljenjeOdrzavanje		EQU	RAM_START + 04Bh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00127	A0004C	OsvKraj               var byte
_OsvKraj         		EQU	RAM_START + 04Ch
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00126	A0004D	OsvPocetak            var byte
_OsvPocetak      		EQU	RAM_START + 04Dh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00107	A0004E	Pali                  var byte
_Pali            		EQU	RAM_START + 04Eh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00117	A0004F	RTCDate	              Var Byte
_RTCDate         		EQU	RAM_START + 04Fh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00118	A00050	RTCDay	              Var Byte
_RTCDay          		EQU	RAM_START + 050h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00119	A00051	RTCHour	              Var Byte
_RTCHour         		EQU	RAM_START + 051h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00120	A00052	RTCMin	              Var Byte
_RTCMin          		EQU	RAM_START + 052h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00116	A00053	RTCMonth              Var Byte
_RTCMonth        		EQU	RAM_START + 053h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00121	A00054	RTCSec	              Var Byte
_RTCSec          		EQU	RAM_START + 054h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00115	A00055	RTCYear	              Var Byte
_RTCYear         		EQU	RAM_START + 055h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00110	A00056	sat                   var byte
_sat             		EQU	RAM_START + 056h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00125	A00057	SatKraj               var byte
_SatKraj         		EQU	RAM_START + 057h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00124	A00058	SatPocetak            var byte
_SatPocetak      		EQU	RAM_START + 058h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00097	A00059	StaraV1               var byte
_StaraV1         		EQU	RAM_START + 059h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00098	A0005A	StaraV2               var byte
_StaraV2         		EQU	RAM_START + 05Ah
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00099	A0005B	StaraV3               var byte
_StaraV3         		EQU	RAM_START + 05Bh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00100	A0005C	StaraV4               var byte
_StaraV4         		EQU	RAM_START + 05Ch
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00129	A0005D	StarostanjeRTC        var byte
_StarostanjeRTC  		EQU	RAM_START + 05Dh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00106	A0005E	Svitanje              var byte
_Svitanje        		EQU	RAM_START + 05Eh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00102	A0005F	x                     var byte
_x               		EQU	RAM_START + 05Fh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00013	A00060	fsave0H     var WORD  BANK0  SYSTEM   ' locations for FSR registers
fsave0H         		EQU	RAM_START + 060h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00016	A00062	fsave0L     var WORD  BANK0  SYSTEM   ' low priority FSR locations
fsave0L         		EQU	RAM_START + 062h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00014	A00064	fsave1H     var WORD  BANK0  SYSTEM
fsave1H         		EQU	RAM_START + 064h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00017	A00066	fsave1L     var WORD  BANK0  SYSTEM
fsave1L         		EQU	RAM_START + 066h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00015	A00068	fsave2H     var WORD  BANK0  SYSTEM
fsave2H         		EQU	RAM_START + 068h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00018	A0006A	fsave2L     var WORD  BANK0  SYSTEM
fsave2L         		EQU	RAM_START + 06Ah
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00008	A0006C	bsave       var byte  BANK0  SYSTEM   ' location for BSR register
bsave           		EQU	RAM_START + 06Ch
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00011	A0006D	psaveH      VAR BYTE  BANK0  SYSTEM   ' PCLATH Hi Pri.
psaveH          		EQU	RAM_START + 06Dh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00012	A0006E	psaveL      VAR BYTE  BANK0  SYSTEM   '        Lo Pri.
psaveL          		EQU	RAM_START + 06Eh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00009	A0006F	psaveUH     VAR BYTE  BANK0  SYSTEM   ' PCLATU Hi Pri.
psaveUH         		EQU	RAM_START + 06Fh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00010	A00070	psaveUL     VAR BYTE  BANK0  SYSTEM   '        Lo Pri.
psaveUL         		EQU	RAM_START + 070h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00007	A00071	ssave       var byte  BANK0  SYSTEM   ' location for STATUS register
ssave           		EQU	RAM_START + 071h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00092	A00072	DutyVars              VAR BYTE[4]
_DutyVars        		EQU	RAM_START + 072h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00072	A00076	STKanal1              var long
_STKanal1        		EQU	RAM_START + 076h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00073	A0007A	STKanal2              var long
_STKanal2        		EQU	RAM_START + 07Ah
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00074	A0007E	STKanal3              var long
_STKanal3        		EQU	RAM_START + 07Eh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00075	A00082	STKanal4              var long
_STKanal4        		EQU	RAM_START + 082h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00093	A00086	Displej               var byte[5]
_Displej         		EQU	RAM_START + 086h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00095	A0008B	Prijem                 var byte[5]
_Prijem          		EQU	RAM_START + 08Bh
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00094	A00090	Memorija              var byte[75]
_Memorija        		EQU	RAM_START + 090h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00077	Kanal1                VAR DutyVars[0]       
_Kanal1          		EQU	_DutyVars
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00078	Kanal2                VAR DutyVars[1]          
_Kanal2          		EQU	_DutyVars + 001h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00079	Kanal3                VAR DutyVars[2]
_Kanal3          		EQU	_DutyVars + 002h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00080	Kanal4                VAR DutyVars[3]
_Kanal4          		EQU	_DutyVars + 003h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00082	RKanal1               VAR DutyVars[0]       
_RKanal1         		EQU	_DutyVars
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00083	RKanal2               VAR DutyVars[1]          
_RKanal2         		EQU	_DutyVars + 001h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00084	RKanal3               VAR DutyVars[2]
_RKanal3         		EQU	_DutyVars + 002h
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00085	RKanal4               VAR DutyVars[3]
_RKanal4         		EQU	_DutyVars + 003h
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00259	PORTL   VAR     PORTB
_PORTL           		EQU	 PORTB
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00260	PORTH   VAR     PORTC
_PORTH           		EQU	 PORTC
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00261	TRISL   VAR     TRISB
_TRISL           		EQU	 TRISB
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00262	TRISH   VAR     TRISC
_TRISH           		EQU	 TRISC
#define _Serviced_H      	_INT_Flags??0
#define _Serviced_L      	_INT_Flags??1
#define _InHPint         	_INT_Flags??2
#define _NoPriority      	_INT_Flags??3
#define _TMR1ON          	_T1CON??0
#define _BCKLIGHT        	_PORTB??6
#define _TLEFT           	_PORTC??4
#define _TRIGHT          	_PORTC??6
#define _TDOWN           	_PORTC??5
#define _TUP             	_PORTC??7
#define _ODRZAVANJE      	_PORTC??0
#define _SDA             	_PORTA??1
#define _SCL             	_PORTA??0
#define _NovoStanjeDesno 	 PB01, 000h
#define _StaroStanjeDesno	 PB01, 004h
#define _NovoStanjeLevo  	 PB01, 003h
#define _StaroStanjeLevo 	 PB01, 007h
#define _NovoStanjeGore  	 PB01, 002h
#define _StaroStanjeGore 	 PB01, 006h
#define _NovoStanjeDole  	 PB01, 001h
#define _StaroStanjeDole 	 PB01, 005h
#define _INT_Flags??0    	 INT_Flags, 000h
#define _INT_Flags??1    	 INT_Flags, 001h
#define _INT_Flags??2    	 INT_Flags, 002h
#define _INT_Flags??3    	 INT_Flags, 003h
#define _T1CON??0        	 T1CON, 000h
#define _PORTB??6        	 PORTB, 006h
#define _PORTC??4        	 PORTC, 004h
#define _PORTC??6        	 PORTC, 006h
#define _PORTC??5        	 PORTC, 005h
#define _PORTC??7        	 PORTC, 007h
#define _PORTC??0        	 PORTC, 000h
#define _PORTA??1        	 PORTA, 001h
#define _PORTA??0        	 PORTA, 000h
#define _UCFG??3         	 UCFG, 003h

; Constants.
_USBMEMORYADDRESS		EQU	00400h
_RTC             		EQU	000D0h
_SecReg          		EQU	00000h

; EEPROM data.


	INCLUDE	"DANER V2.MAC"
	INCLUDE	"C:\PBP3\PBPPI18L.LIB"


; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00036	#HEADER
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00062	#CONFIG
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00240	BANKA   $0000, $005F
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00241	BANK0   $0060, $00FF
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00242	BANK1   $0100, $01FF
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00243	BANK2   $0200, $02FF
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00244	BANK3   $0300, $03FF
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00245	BANK4   $0400, $04FF
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00246	BANK5   $0500, $05FF
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00247	BANK6   $0600, $06FF
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00248	BANK7   $0700, $07FF
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00250	'EEPROM  $F00000, $F000FF
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00252	    #MSG "LONG Variables enabled (PBPL used)"
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00253	    LIBRARY  "PBPPI18L"

; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00254	    INCLUDE "PBPPI18L.RAM"
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00257	    INCLUDE "PBPPIC18.RAM"
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00258	#ENDIF
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00263	USBMEMORYADDRESS Con	$400	' USB RAM starts here
; C:\PBP3\DEVICES\PIC18F2550.PBPINC	00267	#HEADER

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00002	asm

	ASM?

        __CONFIG    _CONFIG1L, _PLLDIV_1_1L & _CPUDIV_OSC1_PLL2_1L & _USBDIV_2_1L
        __CONFIG    _CONFIG1H, _FOSC_HS_1H & _FCMEN_OFF_1H & _IESO_OFF_1H
        __CONFIG    _CONFIG2L, _PWRT_ON_2L & _BOR_ON_2L & _BORV_3_2L & _VREGEN_ON_2L
        __CONFIG    _CONFIG2H, _WDT_ON_2H
        __CONFIG    _CONFIG3H, _CCP2MX_ON_3H & _PBADEN_OFF_3H & _LPT1OSC_OFF_3H & _MCLRE_OFF_3H
        __CONFIG    _CONFIG4L, _STVREN_ON_4L & _LVP_OFF_4L & _XINST_OFF_4L
        __CONFIG    _CONFIG5L, _CP1_ON_5L & _CP0_ON_5L
        __CONFIG    _CONFIG5H, _CPB_ON_5H & _CPD_ON_5H


	ENDASM?

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00013	DEFINE OSC 20

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00015	INCLUDE "DT_INTS-18.bas"            

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00001	DISABLE DEBUG
	DDISABLE?	
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00003	DEFINE  INTHAND       INT_ENTRY_H  ' Context saving for High Priority Ints
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00004	DEFINE  INTLHAND      INT_ENTRY_L  ' Context saving for Low Priority Ints

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00027	INT_Flags = 0 
	MOVE?CB	000h, INT_Flags

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00029	goto  OVER_DT_INTS_18
	GOTO?L	_OVER_DT_INTS_18

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00031	ASM

	ASM?

;____ Check for PBPL (PBP ver. 2.50 and above only)__________________________
PBPLongs_Used = 0
  if (R1 - R0 == 4)
PBPLongs_Used = 1
;    messg PBPL Used = 1
  endif

  nolist
;____ DEFINE available interrupt sources_____________________________________

  #define INT_INT   INTCON, INT0IF  ;-- INT External, 16F compatible
  #define INT0_INT  INTCON, INT0IF  ;-- INT0 External 
  #define INT1_INT  INTCON3,INT1IF  ;-- INT1 External 
  #define INT2_INT  INTCON3,INT2IF  ;-- INT2 External 
  #define INT3_INT  INTCON3,INT3IF  ;-- INT3 External 
  #define RBC_INT   INTCON, RBIF    ;-- RB Port Change 
  #define TMR0_INT  INTCON, TMR0IF  ;-- TMR0 Overflow  18F
  #define TMR1_INT  PIR1, TMR1IF    ;-- TMR1 Overflow 
  #define TMR2_INT  PIR1, TMR2IF    ;-- TMR2 to PR2 Match 
  #define TMR3_INT  PIR2, TMR3IF    ;-- TMR3 Overflow 
  #define TMR4_INT  PIR3, TMR4IF    ;-- TMR4 Overflow 
  #define TX_INT    PIR1, TXIF      ;-- USART Transmit 
  #define TX1_INT   PIR1, TX1IF     ;-- USART1 Transmit 
  #define TX2_INT   PIR3, TX2IF     ;-- USART2 Transmit 
  #define RX_INT    PIR1, RCIF      ;-- USART Receive 
  #define RX1_INT   PIR1, RC1IF     ;-- USART1 Receive 
  #define RX2_INT   PIR3, RC2IF     ;-- USART2 Receive 
  #define CMP_INT   PIR2, CMIF      ;-- Comparator 
  #define EE_INT    PIR2, EEIF      ;-- EEPROM/FLASH Write Operation 
  #define BUS_INT   PIR2, BCLIF     ;-- Bus Collision 
  #define LVD_INT   PIR2, LVDIF     ;-- Low Voltage Detect 
  #define HLVD_INT  PIR2, HLVDIF    ;-- High/Low Voltage Detect 
  #define PSP_INT   PIR1, PSPIF     ;-- Parallel Slave Port Read/Write 
  #define AD_INT    PIR1, ADIF      ;-- A/D Converter 
  #define SSP_INT   PIR1, SSPIF     ;-- Master Synchronous Serial Port 
  #define CCP1_INT  PIR1, CCP1IF    ;-- CCP1 
  #define CCP2_INT  PIR2, CCP2IF    ;-- CCP2 
  #define CCP3_INT  PIR3, CCP3IF    ;-- CCP3 
  #define CCP4_INT  PIR3, CCP4IF    ;-- CCP4 
  #define CCP5_INT  PIR3, CCP5IF    ;-- CCP5 
  #define OSC_INT   PIR2, OSCFIF    ;-- Oscillator Fail 

; -- Added for ver. 3.2 --
  #define SPP_INT   PIR1, SPPIF     ;-- Streaming Parallel Port Read/Write
  #define BUS1_INT  PIR2, BCL1IF    ;-- Bus 1 Collision 
  #define BUS2_INT  PIR3, BCL2IF    ;-- Bus 2 Collision
  #define ECCP1_INT PIR2, ECCP1IF   ;-- ECCP1
  #define LCD_INT   PIR3, LCDIF     ;-- LCD Driver
  #define PMP_INT   PIR1, PMPIF     ;-- Parallel Master Port
  #define SSP1_INT  PIR1, SSP1IF    ;-- Synchronous Serial Port 1
  #define SSP2_INT  PIR3, SSP2IF    ;-- Synchronous Serial Port 2
  #define TMR5_INT  PIR3, TMR5IF    ;-- Timer 5
  #define PT_INT    PIR3, PTIF      ;-- PWM Time Base
  #define IC1IF_INT  PIR3, IC1IF    ;-- Motion Feedback
  #define IC2QEIF_INT  PIR3, IC2QEIF  ;-- Motion Feedback
  #define IC3DRIF_INT  PIR3, IC3DRIF  ;-- Motion Feedback

;_____ Comparators __________________________________________________________
  #define CMP0_INT  PIR1, CMP0IF    ;-- Comparator 0 - 1230/1330 only

CMPIFREG = PIR2                     ;-- Comparator 1
CMPIEREG = PIE2
CMPIPREG = IPR2
    ifdef C1IF                      ;     18F24K20  18F25K20  18F26K20
CM1IFBIT = C1IF                     ;     18F44K20  18F45K20  18F46K20
    endif
    ifdef CM1IF
CM1IFBIT = CM1IF                    ;     several J PICs
    endif
    ifdef CMP1IF
CMPIFREG = PIR1                     ;     1230/1330 only
CM1IFBIT = CMP1IF
CMPIEREG = PIE1
CMPIPREG = IPR1
    endif

    ifdef CM1IFBIT
       #define CMP1_INT  CMPIFREG, CM1IFBIT     
    endif

    ifdef C2IF                      ;-- Comparator 2
CM2IFBIT = C2IF                     ;     18F24K20  18F25K20  18F26K20
    endif                           ;     18F44K20  18F45K20  18F46K20
    ifdef CM2IF
CM2IFBIT = CM2IF                    ;     several J PICs
    endif    
    ifdef CMP2IF
CM2IFBIT = CMP2IF                   ;     1230/1330 only
    endif    

    ifdef CM2IFBIT
      #define CMP2_INT  CMPIFREG, CM2IFBIT     
    endif

;_____ USB Module ___________________________________________________________

  #define USB_INT       PIR2, USBIF   ;-- USB Interrupt (funnel)
  #define USB_ACTV_INT  UIR,  ACTVIF  ;-- Bus Activity Detect 
  #define USB_ERR_INT   UIR,  UERRIF  ;-- USB Error Condition INT (funnel)
  #define USB_RST_INT   UIR,  URSTIF  ;-- USB Reset 
  #define USB_IDLE_INT  UIR,  IDLEIF  ;-- Idle Detect 
  #define USB_STALL_INT UIR,  STALLIF ;-- A STALL Handshake 
  #define USB_TRN_INT   UIR,  TRNIF   ;-- Transaction Complete 
  #define USB_SOF_INT   UIR,  SOFIF   ;-- START-OF-FRAME Token 

; -- USB Error Flags --
  #define USB_BTO_INT   UEIR, BTOEF   ;-- Bus Turnaround Time-out Error
  #define USB_BTS_INT   UEIR, BTSEF   ;-- Bit Stuff Error
  #define USB_CRC16_INT UEIR, CRC16EF ;-- CRC16 Failure
  #define USB_CRC5_INT  UEIR, CRC5EF  ;-- CRC5 Host Error
  #define USB_DFN8_INT  UEIR, DFN8EF  ;-- Data Field Size Error
  #define USB_PID_INT   UEIR, PIDEF   ;-- PID Check Failure
   
;_____ Ethernet Module ______________________________________________________
  #define ETH_INT       PIR2, ETHIF   ;-- Ethernet Module
  #define ETH_DMA_INT    EIR, DMAIF   ;-- DMA Interrupt
  #define ETH_LINK_INT   EIR, LINKIF  ;-- Link Status Change
  #define ETH_PKT_INT    EIR, PKTIF   ;-- Receive Packet Pending
  #define ETH_RXER_INT   EIR, RXERIF  ;-- Receive Error
  #define ETH_TXER_INT   EIR, TXERIF  ;-- Transmit Error
  #define ETH_TX_INT     EIR, TXIF    ;-- Transmit

;_____ CAN Module ___________________________________________________________
  #define CAN_ERR_INT    PIR3, ERRIF    ;-- CAN bus Error
  #define CAN_IRX_INT    PIR3, IRXIF    ;-- Invalid Received Message
  #define CAN_RXB0_INT   PIR3, RXB0IF   ;-* Receive Buffer 0      Mode 0
  #define CAN_FIFOWM_INT PIR3, FIFOWMIF ;-- FIFO Watermark        Mode 1, 2
  #define CAN_RXB1_INT   PIR3, RXB1IF   ;-* Receive Buffer 1      Mode 0
  #define CAN_RXBn_INT   PIR3, RXBnIF   ;-- Any Receive Buffer    Mode 1, 2
  #define CAN_TXB0_INT   PIR3, TXB0IF   ;-- Transmit Buffer 0
  #define CAN_TXB1_INT   PIR3, TXB1IF   ;-- Transmit Buffer 1
  #define CAN_TXB2_INT   PIR3, TXB2IF   ;-* Transmit Buffer 2     Mode 0
  #define CAN_TXBn_INT   PIR3, TXBnIF   ;-- Any Transmit Buffer   Mode 1, 2
  #define CAN_WAKE_INT   PIR3, WAKIF    ;-- CAN bus Activity Wake-up


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00168	asm

	ASM?

; -- macro --
INT_Source  macro  IFR, IFB, IER, IEB, IPR, IPB
    if (IflagReg == IFR) && (IflagBit == IFB)  
  list  
INT_Flag_Reg = IFR
INT_Flag_Bit = IFB
INT_Enable_Reg = IER
INT_Enable_Bit = IEB
INT_Priority_Reg = IPR
INT_Priority_Bit = IPB
Found = 1
    endif
  nolist  
    endm 


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00186	asm

	ASM?

;____________________________________________________________________________
GetIntInfo  macro  IflagReg, IflagBit
  nolist
INT_Flag_Reg = -1
INT_Flag_Bit = -1
INT_Enable_Reg = -1
INT_Enable_Bit = -1
  ifdef  IPR1
INT_Priority_Reg = -1
INT_Priority_Bit = -1
  endif
Found = 0

  ifdef INT0IF  ;----{ INT0 External Interrupt }----------[INTCON, INT0IF]---
      INT_Source  INTCON,INT0IF, INTCON,INT0IE, -1, -1
  endif
  ifdef INT1IF  ;----{ INT1 External Interrupt }---------[INTCON3, INT1IF]---
      INT_Source  INTCON3,INT1IF, INTCON3,INT1IE, INTCON3,INT1IP
  endif
  ifdef INT2IF  ;----{ INT2 External Interrupt }---------[INTCON3, INT2IF]---
      INT_Source  INTCON3,INT2IF, INTCON3,INT2IE, INTCON3,INT2IP
  endif
  ifdef INT3IF  ;----{ INT3 External Interrupt }---------[INTCON3, INT3IF]---
      INT_Source  INTCON3,INT3IF, INTCON3,INT3IE, INTCON2,INT3IP
  endif
  ifdef RBIF    ;----{ RB Port Change Interrupt }-----------[INTCON, RBIF]---
      INT_Source  INTCON,RBIF, INTCON, RBIE,INTCON2,RBIP
  endif
  ifdef TMR0IF  ;----{ TMR0 Overflow Interrupt }----------[INTCON, TMR0IF]---
      INT_Source  INTCON,TMR0IF, INTCON,TMR0IE, INTCON2,TMR0IP
  endif
  ifdef TMR1IF  ;----{ TMR1 Overflow Interrupt }------------[PIR1, TMR1IF]---
      INT_Source  PIR1,TMR1IF, PIE1,TMR1IE, IPR1,TMR1IP
  endif
  ifdef TMR2IF  ;----{ TMR2 to PR2 Match Interrupt }--------[PIR1, TMR2IF]---
      INT_Source  PIR1,TMR2IF, PIE1,TMR2IE, IPR1,TMR2IP
  endif
  ifdef TMR3IF  ;----{ TMR3 Overflow Interrupt }------------[PIR2, TMR3IF]---
      INT_Source  PIR2,TMR3IF, PIE2,TMR3IE, IPR2,TMR3IP
  endif
  ifdef TMR4IF  ;----{ TMR4 Overflow Interrupt }------------[PIR3, TMR4IF]---
      INT_Source  PIR3,TMR4IF, PIE3,TMR4IE, IPR3,TMR4IP
  endif
  ifndef TX1IF  ;----{ USART Transmit Interrupt }-------------[PIR1, TXIF]---
      ifdef TXIF
          INT_Source  PIR1,TXIF, PIE1,TXIE, IPR1,TXIP
      endif
  endif
  ifdef TX1IF   ;----{ USART1 Transmit Interrupt }-----------[PIR1, TX1IF]---
      INT_Source  PIR1,TX1IF, PIE1,TX1IE, IPR1,TX1IP
  endif
  ifdef TX2IF   ;----{ USART2 Transmit Interrupt }-----------[PIR3, TX2IF]---
      INT_Source  PIR3,TX2IF, PIE3,TX2IE, IPR3,TX2IP
  endif
  ifndef RC1IF  ;----{ USART Receive Interrupt }---------------[PIR1 RCIF]---  
    ifdef RCIF
        INT_Source  PIR1,RCIF, PIE1,RCIE, IPR1,RCIP
    endif
  endif
  ifdef RC1IF   ;----{ USART1 Receive Interrupt }------------[PIR1, RC1IF]---
      INT_Source  PIR1,RC1IF, PIE1,RC1IE, IPR1,RC1IP
  endif
  ifdef RC2IF   ;----{ USART2 Receive Interrupt }------------[PIR3, RC2IF]---
      INT_Source  PIR3,RC2IF, PIE3,RC2IE, IPR3,RC2IP
  endif
  ifdef CMIF    ;----{ Comparator Interrupt }-----------------[PIR2, CMIF]---
      INT_Source  PIR2,CMIF, PIE2,CMIE, IPR2,CMIP
  endif
  ifdef EEIF    ;----{ EEPROM/FLASH Write Operation Interrupt [PIR2, EEIF]---
      INT_Source  PIR2,EEIF, PIE2,EEIE, IPR2,EEIP
  endif
  ifdef BCLIF   ;----{ Bus Collision Interrupt }-------------[PIR2, BCLIF]---
      INT_Source  PIR2,BCLIF, PIE2,BCLIE, IPR2,BCLIP
  endif
  ifdef LVDIF   ;----{ Low Voltage Detect Interrupt }--------[PIR2, LVDIF]---
      INT_Source  PIR2,LVDIF, PIE2,LVDIE, IPR2,LVDIP
  endif
  ifdef HLVDIF  ;----{ High/Low Voltage Detect Interrupt }--[PIR2, HLVDIF]---
      INT_Source  PIR2,HLVDIF, PIE2,HLVDIE, IPR2,HLVDIP
  endif
  ifdef PSPIF   ;----{ Parallel Slave Port Interrupt }-------[PIR1, PSPIF]---
      INT_Source  PIR1,PSPIF, PIE1,PSPIE, IPR1,PSPIP
  endif
  ifdef ADIF    ;----{ A/D Converter Interrupt }--------------[PIR1, ADIF]---
      INT_Source  PIR1,ADIF, PIE1,ADIE, IPR1,ADIP
  endif
  ifdef SSPIF   ;----{ Synchronous Serial Port Interrupt }---[PIR1, SSPIF]---
      INT_Source  PIR1,SSPIF, PIE1,SSPIE, IPR1,SSPIP
  endif
  ifdef CCP1IF  ;----{ CCP1 Interrupt }---------------------[PIR1, CCP1IF]---
      INT_Source  PIR1,CCP1IF, PIE1,CCP1IE, IPR1,CCP1IP
  endif
  ifdef CCP2IF  ;----{ CCP2 Interrupt Flag }----------------[PIR2, CCP2IF]---
      INT_Source  PIR2,CCP2IF, PIE2,CCP2IE, IPR2,CCP2IP
  endif
  ifdef CCP3IF  ;----{ CCP3 Interrupt Flag }----------------[PIR3, CCP3IF]---
      INT_Source  PIR3,CCP3IF, PIE3,CCP3IE, IPR3,CCP3IP
  endif
  ifdef CCP4IF  ;----{ CCP4 Interrupt Flag }----------------[PIR3, CCP4IF]---
      INT_Source  PIR3,CCP4IF, PIE3,CCP4IE, IPR3,CCP4IP
  endif
  ifdef CCP5IF  ;----{ CCP5 Interrupt Flag }----------------[PIR3, CCP5IF]---
      INT_Source  PIR3,CCP5IF, PIE3,CCP5IE, IPR3,CCP5IP
  endif
  ifdef OSCFIF  ;----{ Osc Fail  Interrupt Flag }-----------[PIR2, OSCFIF]---
      INT_Source  PIR2,OSCFIF, PIE2,OSCFIE, IPR2,OSCFIP
  endif


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00296	asm

	ASM?

; -- Added for ver. 3.2 --
  ifdef SPPIF   ;----{ Streaming Parallel Port Read/Write }--[PIR1, SPPIF]---
      INT_Source  PIR1,SPPIF, PIE1,SPPIE, IPR1,SPPIP
  endif
  ifdef BCL1IF  ;----{ Bus 1 Collision }--------------------[PIR2, BCL1IF]---
      INT_Source  BUS1_INT, PIE2,BCL1IE, IPR2,BCL1IP
  endif
  ifdef BCL2IF  ;----{ Bus 2 Collision }--------------------[PIR3, BCL2IF]---
      INT_Source  BUS2_INT, PIE3,BCL2IE, IPR3,BCL2IP
  endif
  ifdef CMP0IF  ;----{ Comparator 0 }-----------------------[PIR1, CMP0IF]---
      INT_Source  CMP0_INT, PIE1,CMP0IE, IPR1,CMP0IP
  endif
  ifdef CM1IFBIT  ;--{ Comparator 1 }-----------------------------[varies]---
      INT_Source  CMP1_INT, CMPIEREG,CM1IFBIT, CMPIPREG,CM1IFBIT
  endif
  ifdef CM2IFBIT  ;--{ Comparator 2 }-----------------------------[varies]---
      INT_Source  CMP2_INT, CMPIEREG,CM2IFBIT, CMPIPREG,CM2IFBIT
  endif
  ifdef ECCP1IF  ;---{ ECCP1 }-----------------------------[PIR2, ECCP1IF]---
      INT_Source  ECCP1_INT, PIE2,ECCP1IE, IPR2,ECCP1IP
  endif
  ifdef LCDIF   ;----{ LCD Driver }--------------------------[PIR3, LCDIF]---
      INT_Source  LCD_INT, PIE3,LCDIE, IPR3,LCDIP
  endif
  ifdef PMPIF   ;----{ Parallel Master Port }----------------[PIR1, PMPIF]---
      INT_Source  PMP_INT, PIE1,PMPIE, IPR1,PMPIP
  endif
  ifdef SSP1IF  ;----{ Synchronous Serial Port 1 }----------[PIR1, SSP1IF]---
      INT_Source  SSP1_INT, PIE1,SSP1IE, IPR1,SSP1IP
  endif
  ifdef SSP2IF  ;----{ Synchronous Serial Port 2 }----------[PIR3, SSP2IF]---
      INT_Source  SSP2_INT, PIE3,SSP2IE, IPR3,SSP2IP
  endif
  ifdef TMR5IF  ;----{ Timer 5 }----------------------------[PIR3, TMR5IF]---
      INT_Source  TMR5_INT, PIE3,TMR5IE, IPR3,TMR5IP
  endif
  ifdef PTIF    ;----{ PWM Time Base }------------------------[PIR3, PTIF]---
      INT_Source  PT_INT, PIE3,PTIE, IPR3,PTIP
  endif
  ifdef IC1IF   ;----{ Motion Feedback IC1}------------------[PIR3, IC1IF]---
      INT_Source  IC1IF_INT, PIE3,IC1IE, IPR3,IC1IP
  endif
  ifdef IC2QEIF ;----{ Motion Feedback IC2QE}--------------[PIR3, IC2QEIF]---
      INT_Source  IC2QEIF_INT, PIE3,IC2QEIE, IPR3,IC2QEIP
  endif
  ifdef IC3DRIF ;----{ Motion Feedback IC3DR}--------------[PIR3, IC3DRIF]---
      INT_Source  IC3DRIF_INT, PIE3,IC3DRIE, IPR3,IC3DRIP
  endif
  
;  ifdef   ;----{  }-------------[, ]---
;      INT_Source  , ,, ,
;  endif


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00352	asm  ; -- USB sources --

	ASM?
  ; -- USB sources --
  ifdef USBIF   ;----{ USB  Interrupt funnel }---------------[PIR2, USBIF]---
      INT_Source  PIR2,USBIF, PIE2,USBIE, IPR2,USBIP
          
                ;----{ Bus Activity Detect }-----------------[UIR, ACTVIF]---
      INT_Source  USB_ACTV_INT, UIE,ACTVIE, _NoPriority
  
                ;----{ USB Reset }---------------------------[UIR, URSTIF]---
      INT_Source  USB_RST_INT, UIE,URSTIE, _NoPriority
  
                ;----{ Idle Detect }-------------------------[UIR, IDLEIF]---
      INT_Source  USB_IDLE_INT, UIE,IDLEIE, _NoPriority
  
                ;----{ A STALL Handshake }------------------[UIR, STALLIF]---
      INT_Source  USB_STALL_INT, UIE,STALLIE, _NoPriority
  
                ;----{ Transaction Complete }-----------------[UIR, TRNIF]---
      INT_Source  USB_TRN_INT, UIE,TRNIE, _NoPriority
  
                ;----{ START-OF-FRAME Token }-----------------[UIR, SOFIF]---
      INT_Source  USB_SOF_INT, UIE,SOFIE, _NoPriority
  
; -- USB Error Flags --
                ;----{ USB Error Condition Int funnel }------[UIR, UERRIF]---
      INT_Source  USB_ERR_INT, UIE,UERRIE, _NoPriority
   
                ;----{ Bus Turnaround Time-out Error }-------[UEIR, BTOEF]---
      INT_Source  USB_BTO_INT, UEIE,BTOEE, _NoPriority
   
                ;----{ Bit Stuff Error }---------------------[UEIR, BTSEF]---
      INT_Source  USB_BTS_INT, UEIE,BTSEE, _NoPriority
   
                ;--{ CRC16 Failure }-----------------------[UEIR, CRC16EF]---
      INT_Source  USB_CRC16_INT, UEIE,CRC16EE, _NoPriority
   
                ;---{ CRC5 Host Error }---------------------[UEIR, CRC5EF]---
      INT_Source  USB_CRC5_INT, UEIE,CRC5EE, _NoPriority
   
                ;---{ Data Field Size Error }---------------[UEIR, DFN8EF]---
      INT_Source  USB_DFN8_INT, UEIE,DFN8EE, _NoPriority
   
                ;----{ PID Check Failure }-------------------[UEIR, PIDEF]---
      INT_Source  USB_PID_INT, UEIE,PIDEE, _NoPriority
  endif


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00398	asm  ; -- Ethernet sources --

	ASM?
  ; -- Ethernet sources --
  ifdef ETHIF  ;----{ Ethernet Module }----------------------[PIR2, ETHIF]---
      INT_Source  ETH_INT, PIE2,ETHIE, IPR2,ETHIP
   
               ;----{ DMA Interrupt }-------------------------[EIR, DMAIF]---
      INT_Source  ETH_DMA_INT, EIE,DMAIE, _NoPriority
   
               ;----{ Link Status Change }-------------------[EIR, LINKIF]---
      INT_Source  ETH_LINK_INT, EIE,LINKIE, _NoPriority
   
               ;----{ Receive Packet Pending }----------------[EIR, PKTIF]---
      INT_Source  ETH_PKT_INT, EIE,PKTIE, _NoPriority
   
               ;----{ Receive Error }------------------------[EIR, RXERIF]---
      INT_Source  ETH_RXER_INT, EIE,RXERIE, _NoPriority
              
               ;----{ Transmit Error }-----------------------[EIR, TXERIF]---
      INT_Source  ETH_TXER_INT, EIE,TXERIE, _NoPriority
   
               ;----{ Transmit }-------------------------------[EIR, TXIF]---
      INT_Source  ETH_TX_INT, EIE,TXIE, _NoPriority
  endif


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00422	asm  ; -- CAN Module --

	ASM?
  ; -- CAN Module --
  ifdef WAKIF  
               ;----{ CAN bus Error }------------------------[PIR3, ERRIF]---
      INT_Source  CAN_ERR_INT, PIE3,ERRIE, IPR3,ERRIP
  
               ;----{ Invalid Received Message }-------------[PIR3, IRXIF]---
      INT_Source  CAN_IRX_INT, PIE3,IRXIE, IPR3,IRXIP
  
               ;----{ Receive Buffer 0 }------Mode 0--------[PIR3, RXB0IF]---
      INT_Source  CAN_RXB0_INT, PIE3,RXB0IE, IPR3,RXB0IP
               ;----{ FIFO Watermark }--------Mode 1, 2---[PIR3, FIFOWMIF]---
      INT_Source  CAN_FIFOWM_INT, PIE3,FIFOWMIE, IPR3,FIFOWMIP
  
               ;----{ Receive Buffer 1 }------Mode 0--------[PIR3, RXB1IF]---
      INT_Source  CAN_RXB1_INT, PIE3,RXB1IE, IPR3,RXB1IP
               ;----{ Any Receive Buffer }----Mode 1, 2-----[PIR3, RXBnIF]---
      INT_Source  CAN_RXBn_INT, PIE3,RXBnIE, IPR3,RXBnIP
  
               ;----{ Transmit Buffer 0 }-------------------[PIR3, TXB0IF]---
      INT_Source  CAN_TXB0_INT, PIE3,TXB0IE, IPR3,TXB0IP
  
               ;----{ Transmit Buffer 1 }-------------------[PIR3, TXB1IF]---
      INT_Source  CAN_TXB1_INT, PIE3,TXB1IE, IPR3,TXB1IP
  
               ;----{ Transmit Buffer 2 }-----Mode 0=-------[PIR3, TXB2IF]---
      INT_Source  CAN_TXB2_INT, PIE3,TXB2IE, IPR3,TXB2IP
               ;----{ Any Transmit Buffer }---Mode 1, 2-----[PIR3, TXBnIF]---
      INT_Source  CAN_TXBn_INT, PIE3,TXBnIE, IPR3,TXBnIP
  
               ;----{ CAN bus Activity Wake-up }-------------[PIR3, WAKIF]---
      INT_Source  CAN_WAKE_INT, PIE3,WAKIE, IPR3,WAKIP
  endif
  
  list
    endm
  list  


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00461	ASM

	ASM?

    ifndef  USE_LOWPRIORITY
INT_ENTRY_L
        retfie
    else
        if (USE_LOWPRIORITY != 1)
INT_ENTRY_L
            retfie
        endif
    endif        


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00474	Asm

	ASM?

asm = 0    ; Assembly language Interrupts
ASM = 0
Asm = 0
pbp = 1    ; Basic language interrupts
PBP = 1
Pbp = 1
YES = 1
yes = 1
Yes = 1
NO = 0
no = 0
No = 0
H equ 'H'  ; High Priority
h equ 'H'
L equ 'L'  ; Low Priority
l equ 'L'

  nolist
  
;_____________________________________________________________________________
SaveFSR  macro  F, Pr
  list
    if (F >= 0) && (F <= 2)
        if (Pr == H) || (Pr == L)
            movff    FSR#v(F)L, fsave#v(F)Pr
            movff    FSR#v(F)H, fsave#v(F)Pr + 1
          nolist
        else
            ERROR "SaveFSR - Invalid Priority"
        endif
    else
        ERROR "SaveFSR - Invalid FSR number"
    endif
    list
  endm


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00513	Asm

	ASM?

RestFSR  macro  F, Pr
  list
    if (F >= 0) && (F <= 2)
        if (Pr == H) || (Pr == L)
            movff    fsave#v(F)Pr , FSR#v(F)L
            movff    fsave#v(F)Pr + 1 , FSR#v(F)H 
          nolist
        else
            ERROR "RestFSR - Invalid Priority"
        endif
    else
        ERROR "RestFSR - Invalid FSR number"
    endif
    list
  endm


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00532	Asm

	ASM?

INT_FINISH_H   macro
  endm


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00538	ASM

	ASM?

INT_CREATE_H  macro
  local OverCREATE
    goto OverCREATE
Priority = H    
INT_ENTRY_H  
    movff   PCLATU, psaveUH
    movff   PCLATH, psaveH
    SaveFSR  0, H
    SaveFSR  1, H
    SaveFSR  2, H
    bsf      _InHPint, 0
List_Start_H
    bcf      _Serviced_H, 0         ; Clear Serviced flag
    clrf  BSR
PREV_BANK = 0
    ifdef INT_LIST_H
        INT_LIST_H                  ; Expand the users list of HP INT handlers
    else
        ifdef INT_LIST
            INT_LIST                ; Expand the 16F Compatible List
        else
            error "INT_CREATE_H - INT_LIST or INT_LIST_H not found"
        endif
    endif
    btfsc    _Serviced_H,0          ; if anything was serviced
    goto     List_Start_H           ; go around, and check again

    ifdef ReEnterHPused             ; was ReEnterPBP-18.bas included
        GetAddress21  INT_EXIT_H, RetAddrH
        L?GOTO   _RestorePBP_H      ; Restore PBP system Vars
    endif

INT_EXIT_H
PREV_BANK = 0
    bcf      _InHPint, 0
    RestFSR  0, H                   ; Restore FSR0, if it was saved?
    RestFSR  1, H                   ; Restore FSR1, if it was saved?
    RestFSR  2, H                   ; Restore FSR2, if it was saved?
    movff    psaveH, PCLATH
    movff    psaveUH, PCLATU
    retfie    FAST                  ; Return from Interrupt
OverCREATE
    bsf   INTCON,GIE, 0             ; Enable High Priority Interrupts
    bsf   INTCON,PEIE, 0            ; Enable Peripheral Interrupts
  endm

;---[Stay compatible with the 14-bit version]---------------------------------
INT_CREATE  macro
    INT_CREATE_H
  endm


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00592	ASM

	ASM?

INT_CREATE_L  macro
  local OverCREATE
    goto OverCREATE
    ifdef USE_LOWPRIORITY
        if (USE_LOWPRIORITY != 1)
       error "'DEFINE USE_LOWPRIORITY 1' required for Low Priority Interrupts"
        endif
    else
       error "'DEFINE USE_LOWPRIORITY 1' required for Low Priority Interrupts"
    endif
Priority = L
INT_ENTRY_L  
    movff   WREG, wsave             ; Wreg
    movff   STATUS, ssave           ; STATUS
    movff   BSR, bsave              ; BSR
    movff   PCLATU, psaveUL
    movff   PCLATH, psaveL
    SaveFSR  0, L                   ; FSR0
    SaveFSR  1, L                   ; FSR1
    SaveFSR  2, L                   ; FSR2
    bcf      _InHPint, 0    
List_Start_L
  clrf  BSR
PREV_BANK = 0
    bcf      _Serviced_L, 0
    ifdef INT_LIST_L
        INT_LIST_L                  ; Expand the users list of HP INT handlers
    else
        error "INT_CREATE_L - INT_LIST_L not defined, can not create"
    endif
    btfsc    _Serviced_L, 0         ; if anything was serviced
    goto     List_Start_L           ; go around, and check again

    ifdef ReEnterLPused             ; was ReEnterPBP-18LP.bas included
        GetAddress21  INT_EXIT_L, RetAddrL
        L?GOTO   _RestorePBP_L      ; Restore PBP system Vars
    endif

INT_EXIT_L
PREV_BANK = 0
    RestFSR  0, L                   ; Restore saved vars
    RestFSR  1, L
    RestFSR  2, L                
    movff   psaveUL, PCLATU
    movff   psaveL, PCLATH
    movff   bsave, BSR              ; BSR
    movff   wsave, WREG             ; WREG
    movff   ssave, STATUS           ; STATUS
    retfie                          ; Return from Interrupt
OverCREATE
    bsf   RCON,IPEN, 0              ; Enable Interrupt Priorities
    bsf   INTCON,GIEL, 0            ; Enable Low Priority Interrupts
  endm


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00648	ASM

	ASM?

;---[Returns the Address of a Label as a Word]--(under 64K)------------------
GetAddress macro Label, Wout
    CHK?RP Wout
    movlw low Label          ; get low byte
    movwf Wout
    movlw High Label         ; get high byte
    movwf Wout + 1
    endm

;---[Returns the Address of a Label as a 3 byte array]---(under/over 64k)-----
GetAddress21 macro Label, Aout
    CHK?RP Aout
    movlw low Label          ; get low byte
    movwf Aout
    movlw high Label         ; get high byte
    movwf Aout + 1
    movlw upper Label        ; get upper byte
    movwf Aout + 2
    endm

;---[find correct bank for a PBP BIT variable]-------------------------------
CHKRP?T  macro reg, bit
        CHK?RP  reg
    endm
    


;---[find Assigned Priority for specified INT Source]------------------------
INT_Count = 0

FindIntPriority  macro IntFlagReg, IntFlagBit
  local LoopCount
    nolist
Pfound = 0
LoopCount = 1
      while LoopCount <= INT_Count
          if  (IntFlagReg == PrList#v(LoopCount)R)
             if (IntFlagBit == PrList#v(LoopCount)B)
  list
Priority =  PrList#v(LoopCount)Priority
Pfound = 1
             endif
          endif
LoopCount += 1
      endw
  endm  


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00697	ASM

	ASM?


;---[Add an Interrupt Source to the user's list of INT Handlers]--------------
INT_Handler  macro  IntFlagReg, IntFlagBit, Label, Type, Reset
  list
    local AfterSave, AfterUser, NoInt
INT_Count += 1
PrList#v(INT_Count)R = IntFlagReg
PrList#v(INT_Count)B = IntFlagBit
PrList#v(INT_Count)Priority = Priority
        GetIntInfo   IntFlagReg, IntFlagBit
        if (Found == YES)
            btfss    INT_Enable_Reg, INT_Enable_Bit, 0  ; if INT is enabled
            goto   NoInt
            btfss    INT_Flag_Reg, INT_Flag_Bit, 0      ; and the Flag set?
            goto     NoInt
            if (Priority == H)
                bsf      _Serviced_H, 0
            else
                bsf      _Serviced_L, 0
            endif
            ifdef NO_CLRWDT
                if  (NO_CLRWDT != 1)
                    CLRWDT
                endif
            else
                CLRWDT
            endif
                
            if (Type == PBP)                         ; If INT handler is PBP
              if (Priority == H)
                ifdef ReEnterHPused
                    GetAddress21  AfterSave, RetAddrH  
                    L?GOTO  _SavePBP_H        ; Save PBP system Vars in HP INT
                else
    error "ReEnterPBP-18 must be INCLUDEd to use High Priority PBP interrupts"
                endif
              else ; Priority = L
                ifdef ReEnterLPused
                    GetAddress21  AfterSave, RetAddrL  
                    L?GOTO  _SavePBP_L        ; Save PBP system Vars in LP INT
                else
   error "ReEnterPBP-18LP must be INCLUDEd to use Low Priority PBP interrupts"
                endif
              endif
            endif
AfterSave
PREV_BANK = 0
            if (Priority == H)
                  GetAddress21  AfterUser, RetAddrH  
            else ; Priority = L
                  GetAddress21  AfterUser, RetAddrL
            endif  
            L?GOTO   Label                        ; goto the users INT handler
                    
AfterUser
PREV_BANK = 0
            if (Reset == YES)                      ; reset flag (if specified)
                bcf      INT_Flag_Reg, INT_Flag_Bit, 0 
            endif
        else
            error Interrupt Source (IntFlagReg,IntFlagBit) not found
        endif
NoInt
    clrf  BSR
PREV_BANK = 0
    endm


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00766	asm

	ASM?

;---[Returns from a "goto" subroutine]--(21-bit RetAddr? must be set first)---
INT_RETURN  macro
  local Ret2LP, Ret2HP
      btfsc   _InHPint, 0
      goto    Ret2HP
Ret2LP
      movff   RetAddrL + 2, PCLATU  ; Load PC buffers with return address
      movff   RetAddrL + 1, PCLATH
      movf    RetAddrL, W, 0
    clrf  BSR                    ; Set to BANK0 before returning
PREV_BANK = 0                    ; Tell PBP about the BANK change
      movwf   PCL, 0             ; Go back to where we were
      
Ret2HP       
      movff   RetAddrH + 2, PCLATU ; Load PC buffers with return address
      movff   RetAddrH + 1, PCLATH
      movf    RetAddrH, W, 0
    clrf  BSR                    ; Set to BANK0 before returning
PREV_BANK = 0                    ; Tell PBP about the BANK change
      movwf   PCL, 0             ; Go back to where we were
    endm    
    
;---[Enable an interrupt source]----------------------------------------------
INT_ENABLE  macro  IntFlagReg, IntFlagBit
      GetIntInfo   IntFlagReg, IntFlagBit
      if (Found == YES)
          FindIntPriority  IntFlagReg, IntFlagBit
          if (Pfound == 1)
              if (INT_Priority_Reg != -1)
                  if (Priority == H)
                      bsf  INT_Priority_Reg, INT_Priority_Bit, 0
                  else
                      if (Priority == L)
                          bcf  INT_Priority_Reg, INT_Priority_Bit, 0
                      else
                          error "INT_ENABLE - Invalid Priority Specified"
                      endif
                  endif
              else
                  if (Priority == L)
                      error "INT0_INT can NOT be assigned to Low Priority"
                  endif
              endif
          else
              error "INT_ENABLE - Priority State Not Found"
          endif
;          bcf     INT_Flag_Reg, INT_Flag_Bit, 0        ; clear the flag first 
          bsf     INT_Enable_Reg, INT_Enable_Bit, 0    ; enable the INT source  
      else
          error  "INT_ENABLE - Interrupt Source not found!"
      endif
    endm    

;---[Disable an interrupt source]---------------------------------------------
INT_DISABLE  macro  IntFlagReg, IntFlagBit
      GetIntInfo   IntFlagReg, IntFlagBit
      if (Found == YES)
          bcf     INT_Enable_Reg, INT_Enable_Bit, 0   ; disable the INT source  
      else
          error "INT_DISABLE - Interrupt Source not found!"
      endif
    endm    

;---[Clear an interrupt Flag]-------------------------------------------------
INT_CLEAR  macro  IntFlagReg, IntFlagBit
      GetIntInfo   IntFlagReg, IntFlagBit
      if (Found == YES)
          bcf     INT_Flag_Reg, INT_Flag_Bit, 0       ; clear the INT flag
      else
          error "INT_CLEAR -  Interrupt Source not found!"
      endif
    endm



	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00843	ASM

	ASM?

Save_TBLPTR = 0

    ifdef SIN_USED
Save_TBLPTR = 1
    endif
    ifdef DTMFOUT_USED
Save_TBLPTR = 1
    endif
    ifdef SERDELAY_USED
Save_TBLPTR = 1
    endif
    ifdef CONVBIT_USED
Save_TBLPTR = 1
    endif
    ifdef ERASECODE_USED
Save_TBLPTR = 1
    endif
    ifdef READCODE_USED
Save_TBLPTR = 1
    endif
    ifdef WRITECODE_USED
Save_TBLPTR = 1
    endif
    ifdef LOOKDOWN?BCLB
Save_TBLPTR = 1
    endif
    ifdef LOOKDOWN?BCLT
Save_TBLPTR = 1
    endif
    ifdef LOOKDOWN?BCLW
Save_TBLPTR = 1
    endif
    ifdef LOOKDOWN?CCLB
Save_TBLPTR = 1
    endif
    ifdef LOOKDOWN?CCLT
Save_TBLPTR = 1
    endif
    ifdef LOOKDOWN?CCLW
Save_TBLPTR = 1
    endif
    ifdef LOOKDOWN?WCLB
Save_TBLPTR = 1
    endif
    ifdef LOOKDOWN?WCLT
Save_TBLPTR = 1
    endif
    ifdef LOOKDOWN?WCLW
Save_TBLPTR = 1
    endif
    ifdef LOOK2_USED 
Save_TBLPTR = 1
    endif
    ifdef LOOKUP?BCLB
Save_TBLPTR = 1
    endif
    ifdef LOOKUP?BCLT
Save_TBLPTR = 1
    endif
    ifdef LOOKUP?BCLW
Save_TBLPTR = 1
    endif
    ifdef LOOKUP?CCLB
Save_TBLPTR = 1
    endif
    ifdef LOOKUP?CCLT
Save_TBLPTR = 1
    endif
    ifdef LOOKUP?CCLW
Save_TBLPTR = 1
    endif
    ifdef LOOKUP?TCLB
Save_TBLPTR = 1
    endif
    ifdef LOOKUP?TCLT
Save_TBLPTR = 1
    endif
    ifdef LOOKUP?TCLW
Save_TBLPTR = 1
    endif
    ifdef LOOKUP?WCLB
Save_TBLPTR = 1
    endif
    ifdef LOOKUP?WCLT
Save_TBLPTR = 1
    endif
    ifdef LOOKUP?WCLW
Save_TBLPTR = 1
    endif


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00935	OVER_DT_INTS_18:

	LABEL?L	_OVER_DT_INTS_18	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DT_INTS-18.BAS	00937	ENABLE DEBUG
	DENABLE?	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00016	INCLUDE "SPWM_INT.bas" 

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\SPWM_INT.BAS	00004	ASM

	ASM?

    ifndef T1CON
        error "This chip doesn't have a Timer1 - Can't use SPWM"
    endif
    ifndef SPWM_FREQ
SPWM_FREQ = 60                            
    endif
    ifndef SPWM_RES
SPWM_RES = 100                            
    else
        if (SPWM_RES > 256)
           error Maximum "SPWM_RES" is 256
        endif
    endif    
FOSC4 = OSC * 1000000 / 4
IntFREQ = SPWM_FREQ * SPWM_RES
TimerCount = FOSC4 / IntFREQ
TimerReload = 65543 - TimerCount

;_____________________________________________________________________________
SPWM_PIN  macro Port, Pin, DutyVar
  local NotIdle, SPWMdone
  
  if (InitializingSPWM == 1)
    MOVE?CT 0, Port, Pin                  ; Make Port.Pin LOW
    ifdef BSR  ; if chip is 16-bit        ; Set TRIS to output
        MOVE?CT 0, (Port + 12h), Pin
    else       ; chip is 14-bit
        MOVE?CT 0, (Port + 80h), Pin    
    endif
  else
    MOVE?BA   DutyVar              ; Copy DutyVar to W reg
    btfss     STATUS, Z            ; if DutyVar = 0
    goto      NotIdle              ; No, Skip Idle
    MOVE?CT   0, Port, Pin         ; YES, then idle Low
    goto      SPWMdone

NotIdle
    MOVE?BA   _DutyCount           ; Copy DutyCount to W reg
    CHK?RP    DutyVar              ; Select proper bank
    subwf     DutyVar, W           ; Subtract DutyVar from DutyCount
    MOVE?TT   STATUS,C, Port,Pin   ; Copy carry bit to Port/Pin

SPWMdone
    endif
  endm


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\SPWM_INT.BAS	00053	@Timer1 = TMR1L                   ; map timer registers to a word variable

	ASM?
Timer1 = TMR1L                   ; map timer registers to a word variable

	ENDASM?

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\SPWM_INT.BAS	00055	TimerReload  CON EXT              ; Get the External Constant

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\SPWM_INT.BAS	00058	ASM

	ASM?

SPWM_INIT  macro  PinList
  local OverSPWMhandler
InitializingSPWM = 1
    PinList                       ; Set SPWM pins to Output LOW
InitializingSPWM = 0
    MOVE?CT  1, T1CON, TMR1ON     ;  1     start timer
    goto OverSPWMhandler
    
SPWMhandler
;end asm
;   T1CON.0 = 0 ; stop timer
;   write 0, TMR1L
;   write 1, TMR1H
;   stop
;asm
StartAdd = $
    ;---Reload Timer1------
    MOVE?CT  0, T1CON, TMR1ON     ;  1     stop timer
    MOVLW    LOW(TimerReload)     ;  1     Add TimerReload to the 
    ADDWF    TMR1L,F              ;  1     value in Timer1
    BTFSC    STATUS,C             ;  1/2
    INCF     TMR1H,F              ;  1
    MOVLW    HIGH(TimerReload)    ;  1
    ADDWF    TMR1H,F              ;  1
    MOVE?CT  1, T1CON, TMR1ON     ;  1     start timer

    PinList                       ;  Execute the PWM for each Pin in the List

    CHK?RP   _DutyCount
    incf     _DutyCount, F
    movlw    low (SPWM_RES)
    subwf    _DutyCount, W
  if (SPWM_RES < 256)  
    btfsc    STATUS, Z
    clrf     _DutyCount
  endif
EndAdd = $     
  INT_RETURN
OverSPWMhandler
  VerifySPWM
  endm


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\SPWM_INT.BAS	00102	ASm

	ASM?

VerifySPWM macro                             ; verify valid SPWM configuration
    ifdef BSR                                ; for 18F's
Entry_Delay = 37                             ;   instructions to Enter INT
Exit_Delay  = 16                             ;   instructions to Exit INT
InstCount = (EndAdd - StartAdd) / 2          ;   instructions used by handler
    else                                     ; for 14-bit core's
Entry_Delay = 32                             ;   instructions to Enter INT
Exit_Delay  = 11                             ;   instructions to Exit INT
InstCount = (EndAdd - StartAdd)              ;   instructions used by handler
    endif
InstCount = InstCount + Entry_Delay + Exit_Delay
BetweenInts = TimerCount - InstCount
  if (BetweenInts < 2)
      error Current SPWM configuration is INVALID - NO time left between interrupts
      messg 1 - Try using a "SPWM_FREQ" lower than SPWM_FREQ Hz
      messg 2 - OR, Try using a "SPWM_RES" lower than SPWM_RES
      messg 3 - OR, Use a faster "OSC" than OSC Mhz
      messg 4 - OR, Use fewer SPWM channels
  endif
  endm


	ENDASM?

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00020	DEFINE  LCD_BITS 4
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00021	DEFINE  LCD_DREG PORTB
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00022	DEFINE  LCD_DBIT 0
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00023	DEFINE  LCD_RSREG PORTB
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00024	DEFINE  LCD_RSBIT 4
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00025	DEFINE  LCD_EREG PORTB
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00026	DEFINE  LCD_EBIT 5
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00027	DEFINE  LCD_LINES 2
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00028	DEFINE  LCD_COMMANDUS 2000
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00029	DEFINE  LCD_DATAUS 40
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00032	DEFINE  SPWM_FREQ  170        
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00033	DEFINE  SPWM_RES   100
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00036	symbol BCKLIGHT = PORTB.6
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00038	SYMBOL TLEFT  = PORTC.4 '* 
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00039	SYMBOL TRIGHT = PORTC.6 '** 
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00040	SYMBOL TDOWN  = PORTC.5 '***
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00041	SYMBOL TUP    = PORTC.7 '****
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00043	SYMBOL ODRZAVANJE = PORTC.0
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00045	SYMBOL SDA  = PORTA.1
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00046	SYMBOL SCL	= PORTA.0

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00049	output bcklight
	OUTPUT?T	_BCKLIGHT

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00050	input  TLEFT
	INPUT?T	_TLEFT

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00051	input  TRIGHT
	INPUT?T	_TRIGHT

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00052	input  TUP
	INPUT?T	_TUP

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00053	input  TDOWN
	INPUT?T	_TDOWN

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00054	input  odrzavanje
	INPUT?T	_ODRZAVANJE

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00058	ADCON1  =   $0F
	MOVE?CB	00Fh, ADCON1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00059	CMCON   =   $7
	MOVE?CB	007h, CMCON

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00060	UCFG.3  =   $1
	MOVE?CT	001h, _UCFG??3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00061	PORTA   =   $00       
	MOVE?CB	000h, PORTA

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00062	PORTB   =   $00
	MOVE?CB	000h, PORTB

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00063	PORTC   =   $00
	MOVE?CB	000h, PORTC

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00064	TRISA   =   %00000011
	MOVE?CB	003h, TRISA
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00067	RTC                 CON $D0
; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00068	SecReg              CON $00

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00144	ASM

	ASM?


SPWM_LIST  macro                   
     SPWM_PIN  LATA, 2, _Kanal1  
     SPWM_PIN  LATA, 3, _Kanal2 
     SPWM_PIN  LATA, 4, _Kanal3 
     SPWM_PIN  LATA, 5, _Kanal4 
  endm
  SPWM_INIT  SPWM_LIST              


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00155	ASM

	ASM?

INT_LIST  macro   
        INT_Handler   TMR1_INT,  SPWMhandler,  ASM,  yes
    endm
    INT_CREATE                    


	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00161	@ INT_ENABLE  TMR1_INT              

	ASM?
 INT_ENABLE  TMR1_INT              

	ENDASM?


; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00163	clear   
	CLEAR?	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00164	starostanjeDesno    = $01
	MOVE?CT	001h, _StaroStanjeDesno

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00165	starostanjelevo     = $01 
	MOVE?CT	001h, _StaroStanjeLevo

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00166	starostanjegore     = $01 
	MOVE?CT	001h, _StaroStanjeGore

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00167	starostanjedole     = $01
	MOVE?CT	001h, _StaroStanjeDole

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00168	meni                = $00
	MOVE?CB	000h, _Meni

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00170	Init:

	LABEL?L	_Init	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00171	    gosub ProveriPrviPut 
	GOSUB?L	_ProveriPrviPut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00172	    gosub EEpromMemorija
	GOSUB?L	_EEpromMemorija

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00173	    Lcdout $fe,1,  "D A N E R  L E D"
	LCDOUT?C	0FEh
	LCDOUT?C	001h
	LCDOUT?C	044h
	LCDOUT?C	020h
	LCDOUT?C	041h
	LCDOUT?C	020h
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	045h
	LCDOUT?C	020h
	LCDOUT?C	052h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	04Ch
	LCDOUT?C	020h
	LCDOUT?C	045h
	LCDOUT?C	020h
	LCDOUT?C	044h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00174	    lcdout $fe,$c0,"CONTROL FW:1.1" 
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	043h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	054h
	LCDOUT?C	052h
	LCDOUT?C	04Fh
	LCDOUT?C	04Ch
	LCDOUT?C	020h
	LCDOUT?C	046h
	LCDOUT?C	057h
	LCDOUT?C	03Ah
	LCDOUT?C	031h
	LCDOUT?C	02Eh
	LCDOUT?C	031h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00175	    BCKLIGHT = 1
	MOVE?CT	001h, _BCKLIGHT

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00176	    kanal1 = 0  
	MOVE?CB	000h, _Kanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00177	    kanal2 = 0
	MOVE?CB	000h, _Kanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00178	    kanal3 = 0
	MOVE?CB	000h, _Kanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00179	    kanal4 = 0
	MOVE?CB	000h, _Kanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00180	    kursor = 1
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00182	    pause 1000
	PAUSE?C	003E8h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00183	    read 3,x
	READADDRESS?C	003h
	READ?B	_x

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00184	    if x = 1 then 
	CMPNE?BCL	_x, 001h, L00001

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00185	        write 84,rkanal1
	WRITEADDRESS?C	054h
	WRITE?B	_RKanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00186	        write 85,rkanal2
	WRITEADDRESS?C	055h
	WRITE?B	_RKanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00187	        write 86,rkanal3
	WRITEADDRESS?C	056h
	WRITE?B	_RKanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00188	        write 87,rkanal4
	WRITEADDRESS?C	057h
	WRITE?B	_RKanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00189	        stkanal1 = rkanal1 * 100
	MUL?BCN	_RKanal1, 064h, _STKanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00190	        stkanal2 = rkanal2 * 100
	MUL?BCN	_RKanal2, 064h, _STKanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00191	        stkanal3 = rkanal3 * 100
	MUL?BCN	_RKanal3, 064h, _STKanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00192	        stkanal4 = rkanal4 * 100
	MUL?BCN	_RKanal4, 064h, _STKanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00193	        goto Main 
	GOTO?L	_Main

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00194	    elseif x = 2 then
	GOTO?L	L00002
	LABEL?L	L00001	
	CMPNE?BCL	_x, 002h, L00003

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00195	       KURSOR = 1
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00196	       read 80,rkanal1
	READADDRESS?C	050h
	READ?B	_RKanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00197	       read 81,rkanal2
	READADDRESS?C	051h
	READ?B	_RKanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00198	       read 82,rkanal3
	READADDRESS?C	052h
	READ?B	_RKanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00199	       read 83,rkanal4
	READADDRESS?C	053h
	READ?B	_RKanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00200	       goto RucnoPodesavanje
	GOTO?L	_RucnoPodesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00201	    endif
	LABEL?L	L00002	
	LABEL?L	L00003	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00203	Main:

	LABEL?L	_Main	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00204	     novostanjedesno = tright
	MOVE?TT	_TRIGHT, _NovoStanjeDesno

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00205	     novostanjelevo  = tleft
	MOVE?TT	_TLEFT, _NovoStanjeLevo

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00206	     novostanjeGore  = tup
	MOVE?TT	_TUP, _NovoStanjeGore

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00207	     novostanjedole  = tdown
	MOVE?TT	_TDOWN, _NovoStanjeDole

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00209	     x = x + 1    
	ADD?BCB	_x, 001h, _x

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00210	     if BezTastera< 3000 then 
	CMPGE?WCL	_BezTastera, 000000BB8h, L00005

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00211	       BezTastera =  BezTastera + 1 
	ADD?WCW	_BezTastera, 001h, _BezTastera

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00212	       bcklight = 1
	MOVE?CT	001h, _BCKLIGHT

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00213	     else
	GOTO?L	L00006
	LABEL?L	L00005	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00214	        bcklight = 0 
	MOVE?CT	000h, _BCKLIGHT

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00215	     endif
	LABEL?L	L00006	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00217	     if novostanjegore < starostanjegore then 
	CMPGE?TTL	_NovoStanjeGore, _StaroStanjeGore, L00007

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00218	         BezTastera = 0
	MOVE?CW	000h, _BezTastera

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00219	         if bcklight = 1 then 
	CMPNE?TCL	_BCKLIGHT, 001h, L00009

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00220	            i = i + 1 
	ADD?BCB	_i, 001h, _i

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00221	            if i >= 1 then 
	CMPLT?BCL	_i, 001h, L00011

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00222	               i = 0 
	MOVE?CB	000h, _i

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00223	               meni = 0
	MOVE?CB	000h, _Meni

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00224	               goto menu 
	GOTO?L	_menu

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00225	            endif
	LABEL?L	L00011	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00226	         endif
	LABEL?L	L00009	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00227	     ENDIF
	LABEL?L	L00007	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00229	     if novostanjedole < starostanjedole then 
	CMPGE?TTL	_NovoStanjeDole, _StaroStanjeDole, L00013

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00230	        BezTastera = 0
	MOVE?CW	000h, _BezTastera

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00231	        if bcklight = 1 then
	CMPNE?TCL	_BCKLIGHT, 001h, L00015

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00232	           i = i + 1 
	ADD?BCB	_i, 001h, _i

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00233	           if i >= 1 then 
	CMPLT?BCL	_i, 001h, L00017

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00234	              i = 0 
	MOVE?CB	000h, _i

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00235	              meni = 68
	MOVE?CB	044h, _Meni

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00236	              goto menu 
	GOTO?L	_menu

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00237	           endif
	LABEL?L	L00017	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00238	        endif
	LABEL?L	L00015	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00239	     ENDIF
	LABEL?L	L00013	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00241	     if novostanjelevo  < starostanjelevo  then  beztastera = 0
	CMPGE?TTL	_NovoStanjeLevo, _StaroStanjeLevo, L00019
	MOVE?CW	000h, _BezTastera
	LABEL?L	L00019	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00242	     if novostanjedesno < starostanjedesno then  beztastera = 0
	CMPGE?TTL	_NovoStanjeDesno, _StaroStanjeDesno, L00021
	MOVE?CW	000h, _BezTastera
	LABEL?L	L00021	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00244	     if odrzavanje = 0 then 
	CMPNE?TCL	_ODRZAVANJE, 000h, L00023

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00245	           StaraV1 = kanal1
	MOVE?BB	_Kanal1, _StaraV1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00246	           StaraV2 = kanal2
	MOVE?BB	_Kanal2, _StaraV2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00247	           StaraV3 = kanal3
	MOVE?BB	_Kanal3, _StaraV3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00248	           StaraV4 = kanal4
	MOVE?BB	_Kanal4, _StaraV4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00249	           read 2,osvetljenjeOdrzavanje
	READADDRESS?C	002h
	READ?B	_osvetljenjeOdrzavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00250	           goto OdrzavanjePrekidac
	GOTO?L	_OdrzavanjePrekidac

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00251	     endif  
	LABEL?L	L00023	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00252	     if x < 90 then  DveTacke =  " " 
	CMPGE?BCL	_x, 05Ah, L00025
	MOVE?CB	020h, _DveTacke
	LABEL?L	L00025	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00253	     if x > 90 and x < 180 then DveTacke = ":"
	CMPGT?BCB	_x, 05Ah, T1
	CMPLT?BCB	_x, 0B4h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00027
	MOVE?CB	03Ah, _DveTacke
	LABEL?L	L00027	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00254	     if x > 180 then x = 0 
	CMPLE?BCL	_x, 0B4h, L00029
	MOVE?CB	000h, _x
	LABEL?L	L00029	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00256	     I2CRead sda,scl,$D0,$00,[RTCSec,RTCMin,RTCHour,RTCDay,RTCDate,RTCMonth,RTCYear]		
	I2CDATA?T	_SDA
	I2CCLOCK?T	_SCL
	I2CWRITE?C	0D0h
	I2CWRITE?C	000h
	I2CREAD?B	_RTCSec
	I2CREAD?B	_RTCMin
	I2CREAD?B	_RTCHour
	I2CREAD?B	_RTCDay
	I2CREAD?B	_RTCDate
	I2CREAD?B	_RTCMonth
	I2CREADS?B	_RTCYear

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00258	     decmin = ((RTCMIN  & %11110000)>>4)  * 10 + (RTCMIN  & %00001111)
	AND?BCN	_RTCMin, 0F0h, T1
	SHIFTR?NCN	T1, 004h, T1
	MUL?NCN	T1, 00Ah, T1
	AND?BCN	_RTCMin, 00Fh, T2
	ADD?NNB	T1, T2, _DECmin

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00259	     decsat = ((RTCHOUR & %11110000)>>4)  * 10 + (RTCHOUR & %00001111)
	AND?BCN	_RTCHour, 0F0h, T1
	SHIFTR?NCN	T1, 004h, T1
	MUL?NCN	T1, 00Ah, T1
	AND?BCN	_RTCHour, 00Fh, T2
	ADD?NNB	T1, T2, _DECsat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00260	     decsec = ((RTCsec  & %11110000)>>4)  * 10 + (RTCsec  & %00001111)
	AND?BCN	_RTCSec, 0F0h, T1
	SHIFTR?NCN	T1, 004h, T1
	MUL?NCN	T1, 00Ah, T1
	AND?BCN	_RTCSec, 00Fh, T2
	ADD?NNB	T1, T2, _DECSec

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00262	     Lcdout $fe, 2,"D A N E R  L E D" 
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	044h
	LCDOUT?C	020h
	LCDOUT?C	041h
	LCDOUT?C	020h
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	045h
	LCDOUT?C	020h
	LCDOUT?C	052h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	04Ch
	LCDOUT?C	020h
	LCDOUT?C	045h
	LCDOUT?C	020h
	LCDOUT?C	044h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00263	     Lcdout $fe, $c0, dec2 decsat ,DveTacke, dec2  decmin,dvetacke, dec2 decsec,"              "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_DECsat
	LCDOUTDEC?	
	LCDOUT?B	_DveTacke
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_DECmin
	LCDOUTDEC?	
	LCDOUT?B	_DveTacke
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_DECSec
	LCDOUTDEC?	
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00264	     gosub Kreni
	GOSUB?L	_Kreni

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00265	     STAROSTANJEgore  = NOVOSTANJEgore
	MOVE?TT	_NovoStanjeGore, _StaroStanjeGore

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00266	     STAROSTANJEdesno = NOVOSTANJEdesno
	MOVE?TT	_NovoStanjeDesno, _StaroStanjeDesno

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00267	     STAROSTANJEdole  = NOVOSTANJEdole
	MOVE?TT	_NovoStanjeDole, _StaroStanjeDole

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00268	     STAROSTANJElevo  = NOVOSTANJElevo
	MOVE?TT	_NovoStanjeLevo, _StaroStanjeLevo

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00269	goto Main
	GOTO?L	_Main

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00271	RucnoPodesavanje:

	LABEL?L	_RucnoPodesavanje	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00273	     novostanjelevo  = tleft
	MOVE?TT	_TLEFT, _NovoStanjeLevo

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00274	     novostanjedesno = tright
	MOVE?TT	_TRIGHT, _NovoStanjeDesno

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00275	     novostanjeGore  = tup
	MOVE?TT	_TUP, _NovoStanjeGore

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00276	     novostanjedole  = tdown  
	MOVE?TT	_TDOWN, _NovoStanjeDole

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00279	     if BezTastera< 3000 then 
	CMPGE?WCL	_BezTastera, 000000BB8h, L00031

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00280	        BezTastera =  BezTastera + 1 
	ADD?WCW	_BezTastera, 001h, _BezTastera

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00281	        bcklight = 1
	MOVE?CT	001h, _BCKLIGHT

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00282	     else
	GOTO?L	L00032
	LABEL?L	L00031	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00283	        bcklight = 0 
	MOVE?CT	000h, _BCKLIGHT

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00284	     endif
	LABEL?L	L00032	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00285	     if odrzavanje = 0 then 
	CMPNE?TCL	_ODRZAVANJE, 000h, L00033

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00286	         StaraV1 = kanal1
	MOVE?BB	_Kanal1, _StaraV1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00287	         StaraV2 = kanal2
	MOVE?BB	_Kanal2, _StaraV2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00288	         StaraV3 = kanal3
	MOVE?BB	_Kanal3, _StaraV3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00289	         StaraV4 = kanal4
	MOVE?BB	_Kanal4, _StaraV4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00290	         read 2,osvetljenjeOdrzavanje
	READADDRESS?C	002h
	READ?B	_osvetljenjeOdrzavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00291	         goto OdrzavanjePrekidac
	GOTO?L	_OdrzavanjePrekidac

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00292	     endif
	LABEL?L	L00033	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00293	     IF KURSOR = 1 THEN  
	CMPNE?BCL	_Kursor, 001h, L00035

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00294	        displej[1] = ">"
	MOVE?CB	03Eh, _Displej + 00001h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00295	        displej[2] = " "
	MOVE?CB	020h, _Displej + 00002h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00296	        displej[3] = " "
	MOVE?CB	020h, _Displej + 00003h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00297	        displej[4] = " " 
	MOVE?CB	020h, _Displej + 00004h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00298	     ENDIF
	LABEL?L	L00035	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00299	     IF KURSOR = 2 THEN
	CMPNE?BCL	_Kursor, 002h, L00037

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00300	        displej[1] = " "
	MOVE?CB	020h, _Displej + 00001h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00301	        displej[2] = ">"
	MOVE?CB	03Eh, _Displej + 00002h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00302	        displej[3] = " "
	MOVE?CB	020h, _Displej + 00003h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00303	        displej[4] = " "
	MOVE?CB	020h, _Displej + 00004h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00304	     endif 
	LABEL?L	L00037	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00305	     if kursor = 3 then
	CMPNE?BCL	_Kursor, 003h, L00039

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00306	        displej[1] = " "
	MOVE?CB	020h, _Displej + 00001h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00307	        displej[2] = " "
	MOVE?CB	020h, _Displej + 00002h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00308	        displej[3] = ">"
	MOVE?CB	03Eh, _Displej + 00003h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00309	        displej[4] = " "
	MOVE?CB	020h, _Displej + 00004h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00310	     endif
	LABEL?L	L00039	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00311	     if kursor = 4 then 
	CMPNE?BCL	_Kursor, 004h, L00041

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00312	        displej[1] = " "
	MOVE?CB	020h, _Displej + 00001h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00313	        displej[2] = " "
	MOVE?CB	020h, _Displej + 00002h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00314	        displej[3] = " "
	MOVE?CB	020h, _Displej + 00003h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00315	        displej[4] = ">"
	MOVE?CB	03Eh, _Displej + 00004h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00316	     endif
	LABEL?L	L00041	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00318	     if novostanjedesno < starostanjedesno then
	CMPGE?TTL	_NovoStanjeDesno, _StaroStanjeDesno, L00043

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00319	       BezTastera = 0
	MOVE?CW	000h, _BezTastera

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00320	       if bcklight = 1 then
	CMPNE?TCL	_BCKLIGHT, 001h, L00045

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00321	           Kursor =  Kursor + 1 
	ADD?BCB	_Kursor, 001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00322	           if kursor > 4 then kursor = 1
	CMPLE?BCL	_Kursor, 004h, L00047
	MOVE?CB	001h, _Kursor
	LABEL?L	L00047	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00323	       endif
	LABEL?L	L00045	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00324	     endif
	LABEL?L	L00043	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00326	     if novostanjelevo < starostanjelevo then
	CMPGE?TTL	_NovoStanjeLevo, _StaroStanjeLevo, L00049

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00327	       BezTastera = 0
	MOVE?CW	000h, _BezTastera

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00328	       if bcklight = 1 then
	CMPNE?TCL	_BCKLIGHT, 001h, L00051

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00329	           i = i + 1 
	ADD?BCB	_i, 001h, _i

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00330	           if i >= 2 then 
	CMPLT?BCL	_i, 002h, L00053

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00331	              i = 0 
	MOVE?CB	000h, _i

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00332	              meni = 0
	MOVE?CB	000h, _Meni

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00333	              goto menu  
	GOTO?L	_menu

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00334	           endif
	LABEL?L	L00053	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00335	       endif
	LABEL?L	L00051	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00336	     endif
	LABEL?L	L00049	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00337	     if tup = 0 then
	CMPNE?TCL	_TUP, 000h, L00055

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00338	        BezTastera = 0
	MOVE?CW	000h, _BezTastera

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00339	         if bcklight = 1 then
	CMPNE?TCL	_BCKLIGHT, 001h, L00057

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00340	            if kursor = 1 then
	CMPNE?BCL	_Kursor, 001h, L00059

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00341	                Rkanal1 = Rkanal1 + 1 
	ADD?BCB	_RKanal1, 001h, _RKanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00342	                if Rkanal1 > 100 then rkanal1 = 100 
	CMPLE?BCL	_RKanal1, 064h, L00061
	MOVE?CB	064h, _RKanal1
	LABEL?L	L00061	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00343	                write 80,rkanal1
	WRITEADDRESS?C	050h
	WRITE?B	_RKanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00344	            endif
	LABEL?L	L00059	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00345	            if kursor = 2 then
	CMPNE?BCL	_Kursor, 002h, L00063

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00346	                Rkanal2 = Rkanal2 + 1
	ADD?BCB	_RKanal2, 001h, _RKanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00347	                if Rkanal2 > 100 then rkanal2 = 100 
	CMPLE?BCL	_RKanal2, 064h, L00065
	MOVE?CB	064h, _RKanal2
	LABEL?L	L00065	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00348	                write 81,rkanal2
	WRITEADDRESS?C	051h
	WRITE?B	_RKanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00349	            endif
	LABEL?L	L00063	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00350	            if kursor = 3 then
	CMPNE?BCL	_Kursor, 003h, L00067

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00351	                Rkanal3 = Rkanal3 + 1 
	ADD?BCB	_RKanal3, 001h, _RKanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00352	                if Rkanal3 > 100 then rkanal3 = 100 
	CMPLE?BCL	_RKanal3, 064h, L00069
	MOVE?CB	064h, _RKanal3
	LABEL?L	L00069	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00353	                write 82,rkanal3
	WRITEADDRESS?C	052h
	WRITE?B	_RKanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00354	            endif
	LABEL?L	L00067	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00355	            if kursor = 4 then
	CMPNE?BCL	_Kursor, 004h, L00071

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00356	                Rkanal4 = Rkanal4 + 1 
	ADD?BCB	_RKanal4, 001h, _RKanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00357	                if Rkanal4 > 100 then rkanal4 = 100 
	CMPLE?BCL	_RKanal4, 064h, L00073
	MOVE?CB	064h, _RKanal4
	LABEL?L	L00073	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00358	                write 83,rkanal4
	WRITEADDRESS?C	053h
	WRITE?B	_RKanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00359	            endif
	LABEL?L	L00071	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00360	            pause 60
	PAUSE?C	03Ch

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00361	         endif
	LABEL?L	L00057	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00362	     endif 
	LABEL?L	L00055	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00364	     if tdown = 0 then
	CMPNE?TCL	_TDOWN, 000h, L00075

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00365	        BezTastera = 0
	MOVE?CW	000h, _BezTastera

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00366	        if bcklight = 1 then
	CMPNE?TCL	_BCKLIGHT, 001h, L00077

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00367	             if kursor = 1 then
	CMPNE?BCL	_Kursor, 001h, L00079

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00368	                Rkanal1 = Rkanal1 - 1
	SUB?BCB	_RKanal1, 001h, _RKanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00369	                if  Rkanal1 < 1 or  Rkanal1 > 100 then  Rkanal1 = 0 
	CMPLT?BCB	_RKanal1, 001h, T1
	CMPGT?BCB	_RKanal1, 064h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00081
	MOVE?CB	000h, _RKanal1
	LABEL?L	L00081	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00370	                write 80,rkanal1
	WRITEADDRESS?C	050h
	WRITE?B	_RKanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00371	             endif
	LABEL?L	L00079	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00373	             if kursor = 2 then
	CMPNE?BCL	_Kursor, 002h, L00083

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00374	                Rkanal2 = Rkanal2 - 1
	SUB?BCB	_RKanal2, 001h, _RKanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00375	                if  Rkanal2 < 1 or  Rkanal2 > 100 then  Rkanal2 = 0 
	CMPLT?BCB	_RKanal2, 001h, T1
	CMPGT?BCB	_RKanal2, 064h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00085
	MOVE?CB	000h, _RKanal2
	LABEL?L	L00085	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00376	                write 81,rkanal2
	WRITEADDRESS?C	051h
	WRITE?B	_RKanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00377	             endif
	LABEL?L	L00083	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00379	             if kursor = 3 then
	CMPNE?BCL	_Kursor, 003h, L00087

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00380	                Rkanal3 = Rkanal3 - 1 
	SUB?BCB	_RKanal3, 001h, _RKanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00381	                if  Rkanal3 < 1 or  Rkanal3 > 100 then  Rkanal3 = 0
	CMPLT?BCB	_RKanal3, 001h, T1
	CMPGT?BCB	_RKanal3, 064h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00089
	MOVE?CB	000h, _RKanal3
	LABEL?L	L00089	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00382	                write 82,rkanal3
	WRITEADDRESS?C	052h
	WRITE?B	_RKanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00383	             endif
	LABEL?L	L00087	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00385	             if kursor = 4 then
	CMPNE?BCL	_Kursor, 004h, L00091

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00386	                Rkanal4 = Rkanal4 - 1 
	SUB?BCB	_RKanal4, 001h, _RKanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00387	                if  Rkanal4 < 1 or  Rkanal4 > 100 then  Rkanal4 = 0
	CMPLT?BCB	_RKanal4, 001h, T1
	CMPGT?BCB	_RKanal4, 064h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00093
	MOVE?CB	000h, _RKanal4
	LABEL?L	L00093	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00388	                write 83,rkanal4
	WRITEADDRESS?C	053h
	WRITE?B	_RKanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00389	             endif
	LABEL?L	L00091	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00390	             pause 60
	PAUSE?C	03Ch

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00391	        endif
	LABEL?L	L00077	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00392	     endif 
	LABEL?L	L00075	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00394	     Lcdout $fe, 2,  displej[1],"#1 ",dec3 rkanal1,"%", displej[2],"2# ",dec3 rkanal2,"% "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?B	_Displej + 00001h
	LCDOUT?C	023h
	LCDOUT?C	031h
	LCDOUT?C	020h
	LCDOUTCOUNT?C	003h
	LCDOUTNUM?B	_RKanal1
	LCDOUTDEC?	
	LCDOUT?C	025h
	LCDOUT?B	_Displej + 00002h
	LCDOUT?C	032h
	LCDOUT?C	023h
	LCDOUT?C	020h
	LCDOUTCOUNT?C	003h
	LCDOUTNUM?B	_RKanal2
	LCDOUTDEC?	
	LCDOUT?C	025h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00395	     Lcdout $fe, $c0,displej[3],"#3 ",dec3 rkanal3,"%", displej[4],"4# ",dec3 rkanal4,"% "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?B	_Displej + 00003h
	LCDOUT?C	023h
	LCDOUT?C	033h
	LCDOUT?C	020h
	LCDOUTCOUNT?C	003h
	LCDOUTNUM?B	_RKanal3
	LCDOUTDEC?	
	LCDOUT?C	025h
	LCDOUT?B	_Displej + 00004h
	LCDOUT?C	034h
	LCDOUT?C	023h
	LCDOUT?C	020h
	LCDOUTCOUNT?C	003h
	LCDOUTNUM?B	_RKanal4
	LCDOUTDEC?	
	LCDOUT?C	025h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00397	     STAROSTANJElevo  = NOVOSTANJElevo 
	MOVE?TT	_NovoStanjeLevo, _StaroStanjeLevo

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00398	     STAROSTANJEgore  = NOVOSTANJEgore
	MOVE?TT	_NovoStanjeGore, _StaroStanjeGore

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00399	     STAROSTANJEdesno = NOVOSTANJEdesno
	MOVE?TT	_NovoStanjeDesno, _StaroStanjeDesno

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00400	     STAROSTANJEdole  = NOVOSTANJEdole
	MOVE?TT	_NovoStanjeDole, _StaroStanjeDole

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00401	 goto RucnoPodesavanje
	GOTO?L	_RucnoPodesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00403	Kreni:

	LABEL?L	_Kreni	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00404	   novostanjertc = decmin
	MOVE?BB	_DECmin, _NovostanjeRTC

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00405	   TMinuta = (decsat * 60) + decmin
	MUL?BCN	_DECsat, 03Ch, T1
	ADD?NBW	T1, _DECmin, _TMinuta

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00406	   if novostanjeRTC <> starostanjeRTC then
	CMPEQ?BBL	_NovostanjeRTC, _StarostanjeRTC, L00095

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00407	      NoviMinut = 1
	MOVE?CB	001h, _NoviMinut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00408	   else
	GOTO?L	L00096
	LABEL?L	L00095	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00409	      NoviMinut = 0
	MOVE?CB	000h, _NoviMinut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00410	   endif
	LABEL?L	L00096	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00412	    satPocetak   = memorija[8] 
	MOVE?BB	_Memorija + 00008h, _SatPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00413	    MinutPocetak = memorija[9]  
	MOVE?BB	_Memorija + 00009h, _MinutPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00414	    osvPocetak   = memorija[10]
	MOVE?BB	_Memorija + 0000Ah, _OsvPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00415	    SatKraj      = memorija[12]  
	MOVE?BB	_Memorija + 0000Ch, _SatKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00416	    MinutKraj    = memorija[13]
	MOVE?BB	_Memorija + 0000Dh, _MinutKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00417	    OsvKraj      = memorija[14]
	MOVE?BB	_Memorija + 0000Eh, _OsvKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00419	    PMinuta = (satpocetak * 60)+ minutpocetak
	MUL?BCN	_SatPocetak, 03Ch, T1
	ADD?NBW	T1, _MinutPocetak, _PMinuta

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00420	    KMinuta = (satkraj * 60) + minutkraj
	MUL?BCN	_SatKraj, 03Ch, T1
	ADD?NBW	T1, _MinutKraj, _KMinuta

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00421	    if Pminuta =< tminuta and KMinuta >= tminuta then 
	CMPLE?WWB	_PMinuta, _TMinuta, T1
	CMPGE?WWB	_KMinuta, _TMinuta, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00097

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00423	       if Pminuta = tminuta and decsec = 0 then   'pocetna vrednost
	CMPEQ?WWB	_PMinuta, _TMinuta, T1
	CMPEQ?BCB	_DECSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00099

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00424	          kanal1 = osvpocetak
	MOVE?BB	_OsvPocetak, _Kanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00425	          stkanal1 = osvpocetak * 100
	MUL?BCN	_OsvPocetak, 064h, _STKanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00426	       else
	GOTO?L	L00100
	LABEL?L	L00099	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00427	          if kminuta = tminuta and decsec = 0 then   'krajnja vrednost 
	CMPEQ?WWB	_KMinuta, _TMinuta, T1
	CMPEQ?BCB	_DECSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00101

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00428	              kanal1 = osvkraj
	MOVE?BB	_OsvKraj, _Kanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00429	          else
	GOTO?L	L00102
	LABEL?L	L00101	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00430	              if noviminut = 1 then 
	CMPNE?BCL	_NoviMinut, 001h, L00103

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00431	                 stkanal1 = stkanal1 + (((osvkraj - osvpocetak)*100/(kminuta - pminuta)))  
	SUB?BBN	_OsvKraj, _OsvPocetak, T1
	MUL?NCN	T1, 064h, T1
	SUB?WWN	_KMinuta, _PMinuta, T2
	DIV?NNN	T1, T2, T2
	ADD?NNN	_STKanal1, T2, _STKanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00432	                 kanal1 = stkanal1 / 100 
	DIV?NCB	_STKanal1, 064h, _Kanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00433	              endif
	LABEL?L	L00103	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00434	          endif
	LABEL?L	L00102	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00435	       endif
	LABEL?L	L00100	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00436	    endif
	LABEL?L	L00097	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00438	    satPocetak   = memorija[16] 
	MOVE?BB	_Memorija + 00010h, _SatPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00439	    MinutPocetak = memorija[17]  
	MOVE?BB	_Memorija + 00011h, _MinutPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00440	    osvPocetak   = memorija[18] 
	MOVE?BB	_Memorija + 00012h, _OsvPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00441	    SatKraj      = memorija[20]  
	MOVE?BB	_Memorija + 00014h, _SatKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00442	    MinutKraj    = memorija[21]  
	MOVE?BB	_Memorija + 00015h, _MinutKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00443	    OsvKraj      = memorija[22]  
	MOVE?BB	_Memorija + 00016h, _OsvKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00445	    PMinuta = (satpocetak * 60)+ minutpocetak
	MUL?BCN	_SatPocetak, 03Ch, T1
	ADD?NBW	T1, _MinutPocetak, _PMinuta

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00446	    KMinuta = (satkraj * 60) + minutkraj
	MUL?BCN	_SatKraj, 03Ch, T1
	ADD?NBW	T1, _MinutKraj, _KMinuta

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00448	    if Pminuta =< tminuta and KMinuta >= tminuta then 
	CMPLE?WWB	_PMinuta, _TMinuta, T1
	CMPGE?WWB	_KMinuta, _TMinuta, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00105

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00449	       if Pminuta = tminuta and decsec = 0 then   'pocetna vrednost
	CMPEQ?WWB	_PMinuta, _TMinuta, T1
	CMPEQ?BCB	_DECSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00107

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00450	          kanal1 = osvpocetak
	MOVE?BB	_OsvPocetak, _Kanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00451	          stkanal1 = osvpocetak * 100
	MUL?BCN	_OsvPocetak, 064h, _STKanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00452	       else
	GOTO?L	L00108
	LABEL?L	L00107	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00453	          if kminuta = tminuta and decsec = 0 then   'krajnja vrednost 
	CMPEQ?WWB	_KMinuta, _TMinuta, T1
	CMPEQ?BCB	_DECSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00109

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00454	              kanal1 = osvkraj
	MOVE?BB	_OsvKraj, _Kanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00455	          else
	GOTO?L	L00110
	LABEL?L	L00109	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00456	              if noviminut = 1 then
	CMPNE?BCL	_NoviMinut, 001h, L00111

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00457	                 stkanal1 = stkanal1 - (((osvpocetak - osvkraj)*100/(kminuta - pminuta)))  
	SUB?BBN	_OsvPocetak, _OsvKraj, T1
	MUL?NCN	T1, 064h, T1
	SUB?WWN	_KMinuta, _PMinuta, T2
	DIV?NNN	T1, T2, T2
	SUB?NNN	_STKanal1, T2, _STKanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00458	                 kanal1 = stkanal1 / 100 
	DIV?NCB	_STKanal1, 064h, _Kanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00459	              endif
	LABEL?L	L00111	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00460	          endif
	LABEL?L	L00110	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00461	       endif
	LABEL?L	L00108	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00462	    endif 
	LABEL?L	L00105	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00464	    satPocetak   = memorija[24] 
	MOVE?BB	_Memorija + 00018h, _SatPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00465	    MinutPocetak = memorija[25]  
	MOVE?BB	_Memorija + 00019h, _MinutPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00466	    osvPocetak   = memorija[26]
	MOVE?BB	_Memorija + 0001Ah, _OsvPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00467	    SatKraj      = memorija[28]  
	MOVE?BB	_Memorija + 0001Ch, _SatKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00468	    MinutKraj    = memorija[29]
	MOVE?BB	_Memorija + 0001Dh, _MinutKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00469	    OsvKraj      = memorija[30]
	MOVE?BB	_Memorija + 0001Eh, _OsvKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00471	    PMinuta = (satpocetak * 60)+ minutpocetak
	MUL?BCN	_SatPocetak, 03Ch, T1
	ADD?NBW	T1, _MinutPocetak, _PMinuta

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00472	    KMinuta = (satkraj * 60) + minutkraj   
	MUL?BCN	_SatKraj, 03Ch, T1
	ADD?NBW	T1, _MinutKraj, _KMinuta

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00473	    if Pminuta =< tminuta and KMinuta >= tminuta then 
	CMPLE?WWB	_PMinuta, _TMinuta, T1
	CMPGE?WWB	_KMinuta, _TMinuta, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00113

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00474	       if Pminuta = tminuta and decsec = 0 then   'pocetna vrednost
	CMPEQ?WWB	_PMinuta, _TMinuta, T1
	CMPEQ?BCB	_DECSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00115

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00475	          kanal2 = osvpocetak
	MOVE?BB	_OsvPocetak, _Kanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00476	          stkanal2 = osvpocetak * 100
	MUL?BCN	_OsvPocetak, 064h, _STKanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00477	       else
	GOTO?L	L00116
	LABEL?L	L00115	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00478	          if kminuta = tminuta and decsec = 0 then   'krajnja vrednost 
	CMPEQ?WWB	_KMinuta, _TMinuta, T1
	CMPEQ?BCB	_DECSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00117

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00479	              kanal2 = osvkraj
	MOVE?BB	_OsvKraj, _Kanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00480	          else
	GOTO?L	L00118
	LABEL?L	L00117	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00481	              if noviminut = 1 then 
	CMPNE?BCL	_NoviMinut, 001h, L00119

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00482	                 stkanal2 = stkanal2 + (((osvkraj - osvpocetak)*100/(kminuta - pminuta)))  
	SUB?BBN	_OsvKraj, _OsvPocetak, T1
	MUL?NCN	T1, 064h, T1
	SUB?WWN	_KMinuta, _PMinuta, T2
	DIV?NNN	T1, T2, T2
	ADD?NNN	_STKanal2, T2, _STKanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00483	                 kanal2 = stkanal2 / 100 
	DIV?NCB	_STKanal2, 064h, _Kanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00484	              endif
	LABEL?L	L00119	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00485	          endif
	LABEL?L	L00118	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00486	       endif
	LABEL?L	L00116	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00487	    endif  
	LABEL?L	L00113	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00489	    satPocetak   = memorija[32] 
	MOVE?BB	_Memorija + 00020h, _SatPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00490	    MinutPocetak = memorija[33]  
	MOVE?BB	_Memorija + 00021h, _MinutPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00491	    osvPocetak   = memorija[34] 
	MOVE?BB	_Memorija + 00022h, _OsvPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00492	    SatKraj      = memorija[36]  
	MOVE?BB	_Memorija + 00024h, _SatKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00493	    MinutKraj    = memorija[37]  
	MOVE?BB	_Memorija + 00025h, _MinutKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00494	    OsvKraj      = memorija[38]  
	MOVE?BB	_Memorija + 00026h, _OsvKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00496	    PMinuta = (satpocetak * 60)+ minutpocetak
	MUL?BCN	_SatPocetak, 03Ch, T1
	ADD?NBW	T1, _MinutPocetak, _PMinuta

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00497	    KMinuta = (satkraj * 60) + minutkraj
	MUL?BCN	_SatKraj, 03Ch, T1
	ADD?NBW	T1, _MinutKraj, _KMinuta

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00499	    if Pminuta =< tminuta and KMinuta >= tminuta then 
	CMPLE?WWB	_PMinuta, _TMinuta, T1
	CMPGE?WWB	_KMinuta, _TMinuta, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00121

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00500	       if Pminuta = tminuta and decsec = 0 then   'pocetna vrednost
	CMPEQ?WWB	_PMinuta, _TMinuta, T1
	CMPEQ?BCB	_DECSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00123

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00501	          kanal2 = osvpocetak
	MOVE?BB	_OsvPocetak, _Kanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00502	          stkanal2 = osvpocetak * 100
	MUL?BCN	_OsvPocetak, 064h, _STKanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00503	       else
	GOTO?L	L00124
	LABEL?L	L00123	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00504	          if kminuta = tminuta and decsec = 0 then   'krajnja vrednost 
	CMPEQ?WWB	_KMinuta, _TMinuta, T1
	CMPEQ?BCB	_DECSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00125

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00505	              kanal2 = osvkraj
	MOVE?BB	_OsvKraj, _Kanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00506	          else
	GOTO?L	L00126
	LABEL?L	L00125	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00507	              if noviminut = 1 then
	CMPNE?BCL	_NoviMinut, 001h, L00127

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00508	                 stkanal2 = stkanal2 - (((osvpocetak - osvkraj)*100/(kminuta - pminuta)))  
	SUB?BBN	_OsvPocetak, _OsvKraj, T1
	MUL?NCN	T1, 064h, T1
	SUB?WWN	_KMinuta, _PMinuta, T2
	DIV?NNN	T1, T2, T2
	SUB?NNN	_STKanal2, T2, _STKanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00509	                 kanal2 = stkanal2 / 100 
	DIV?NCB	_STKanal2, 064h, _Kanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00510	              endif
	LABEL?L	L00127	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00511	          endif
	LABEL?L	L00126	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00512	       endif
	LABEL?L	L00124	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00513	    endif 
	LABEL?L	L00121	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00516	    satPocetak   = memorija[40] 
	MOVE?BB	_Memorija + 00028h, _SatPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00517	    MinutPocetak = memorija[41]  
	MOVE?BB	_Memorija + 00029h, _MinutPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00518	    osvPocetak   = memorija[42]
	MOVE?BB	_Memorija + 0002Ah, _OsvPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00519	    SatKraj      = memorija[44]  
	MOVE?BB	_Memorija + 0002Ch, _SatKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00520	    MinutKraj    = memorija[45]
	MOVE?BB	_Memorija + 0002Dh, _MinutKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00521	    OsvKraj      = memorija[46]
	MOVE?BB	_Memorija + 0002Eh, _OsvKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00523	    PMinuta = (satpocetak * 60)+ minutpocetak
	MUL?BCN	_SatPocetak, 03Ch, T1
	ADD?NBW	T1, _MinutPocetak, _PMinuta

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00524	    KMinuta = (satkraj * 60) + minutkraj
	MUL?BCN	_SatKraj, 03Ch, T1
	ADD?NBW	T1, _MinutKraj, _KMinuta

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00526	    if Pminuta =< tminuta and KMinuta >= tminuta then 
	CMPLE?WWB	_PMinuta, _TMinuta, T1
	CMPGE?WWB	_KMinuta, _TMinuta, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00129

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00527	       if Pminuta = tminuta and decsec = 0 then   'pocetna vrednost
	CMPEQ?WWB	_PMinuta, _TMinuta, T1
	CMPEQ?BCB	_DECSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00131

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00528	          kanal3 = osvpocetak
	MOVE?BB	_OsvPocetak, _Kanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00529	          stkanal3 = osvpocetak * 100
	MUL?BCN	_OsvPocetak, 064h, _STKanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00530	       else
	GOTO?L	L00132
	LABEL?L	L00131	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00531	          if kminuta = tminuta and decsec = 0 then   'krajnja vrednost 
	CMPEQ?WWB	_KMinuta, _TMinuta, T1
	CMPEQ?BCB	_DECSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00133

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00532	              kanal3 = osvkraj
	MOVE?BB	_OsvKraj, _Kanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00533	          else
	GOTO?L	L00134
	LABEL?L	L00133	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00534	              if noviminut = 1 then 
	CMPNE?BCL	_NoviMinut, 001h, L00135

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00535	                 stkanal3 = stkanal3 + (((osvkraj - osvpocetak)*100/(kminuta - pminuta)))  
	SUB?BBN	_OsvKraj, _OsvPocetak, T1
	MUL?NCN	T1, 064h, T1
	SUB?WWN	_KMinuta, _PMinuta, T2
	DIV?NNN	T1, T2, T2
	ADD?NNN	_STKanal3, T2, _STKanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00536	                 kanal3 = stkanal3 / 100 
	DIV?NCB	_STKanal3, 064h, _Kanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00537	              endif
	LABEL?L	L00135	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00538	          endif
	LABEL?L	L00134	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00539	       endif
	LABEL?L	L00132	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00540	    endif 
	LABEL?L	L00129	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00542	    satPocetak   = memorija[48] 
	MOVE?BB	_Memorija + 00030h, _SatPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00543	    MinutPocetak = memorija[49]  
	MOVE?BB	_Memorija + 00031h, _MinutPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00544	    osvPocetak   = memorija[50] 
	MOVE?BB	_Memorija + 00032h, _OsvPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00545	    SatKraj      = memorija[52]  
	MOVE?BB	_Memorija + 00034h, _SatKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00546	    MinutKraj    = memorija[53]  
	MOVE?BB	_Memorija + 00035h, _MinutKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00547	    OsvKraj      = memorija[54]  
	MOVE?BB	_Memorija + 00036h, _OsvKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00549	    PMinuta = (satpocetak * 60)+ minutpocetak
	MUL?BCN	_SatPocetak, 03Ch, T1
	ADD?NBW	T1, _MinutPocetak, _PMinuta

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00550	    KMinuta = (satkraj * 60) + minutkraj
	MUL?BCN	_SatKraj, 03Ch, T1
	ADD?NBW	T1, _MinutKraj, _KMinuta

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00552	    if Pminuta =< tminuta and KMinuta >= tminuta then 
	CMPLE?WWB	_PMinuta, _TMinuta, T1
	CMPGE?WWB	_KMinuta, _TMinuta, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00137

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00553	       if Pminuta = tminuta and decsec = 0 then   'pocetna vrednost
	CMPEQ?WWB	_PMinuta, _TMinuta, T1
	CMPEQ?BCB	_DECSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00139

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00554	          kanal3 = osvpocetak
	MOVE?BB	_OsvPocetak, _Kanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00555	          stkanal3 = osvpocetak * 100
	MUL?BCN	_OsvPocetak, 064h, _STKanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00556	       else
	GOTO?L	L00140
	LABEL?L	L00139	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00557	          if kminuta = tminuta and decsec = 0 then   'krajnja vrednost 
	CMPEQ?WWB	_KMinuta, _TMinuta, T1
	CMPEQ?BCB	_DECSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00141

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00558	              kanal3 = osvkraj
	MOVE?BB	_OsvKraj, _Kanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00559	          else
	GOTO?L	L00142
	LABEL?L	L00141	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00560	              if noviminut = 1 then
	CMPNE?BCL	_NoviMinut, 001h, L00143

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00561	                 stkanal3 = stkanal3 - (((osvpocetak - osvkraj)*100/(kminuta - pminuta)))  
	SUB?BBN	_OsvPocetak, _OsvKraj, T1
	MUL?NCN	T1, 064h, T1
	SUB?WWN	_KMinuta, _PMinuta, T2
	DIV?NNN	T1, T2, T2
	SUB?NNN	_STKanal3, T2, _STKanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00562	                 kanal3 = stkanal3 / 100 
	DIV?NCB	_STKanal3, 064h, _Kanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00563	              endif
	LABEL?L	L00143	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00564	          endif
	LABEL?L	L00142	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00565	       endif
	LABEL?L	L00140	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00566	    endif 
	LABEL?L	L00137	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00568	    satPocetak   = memorija[56] 
	MOVE?BB	_Memorija + 00038h, _SatPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00569	    MinutPocetak = memorija[57]  
	MOVE?BB	_Memorija + 00039h, _MinutPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00570	    osvPocetak   = memorija[58]
	MOVE?BB	_Memorija + 0003Ah, _OsvPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00571	    SatKraj      = memorija[60]  
	MOVE?BB	_Memorija + 0003Ch, _SatKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00572	    MinutKraj    = memorija[61]
	MOVE?BB	_Memorija + 0003Dh, _MinutKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00573	    OsvKraj      = memorija[62]
	MOVE?BB	_Memorija + 0003Eh, _OsvKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00575	    PMinuta = (satpocetak * 60) + minutpocetak
	MUL?BCN	_SatPocetak, 03Ch, T1
	ADD?NBW	T1, _MinutPocetak, _PMinuta

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00576	    KMinuta = (satkraj    * 60) + minutkraj
	MUL?BCN	_SatKraj, 03Ch, T1
	ADD?NBW	T1, _MinutKraj, _KMinuta

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00578	    if Pminuta =< tminuta and KMinuta >= tminuta then 
	CMPLE?WWB	_PMinuta, _TMinuta, T1
	CMPGE?WWB	_KMinuta, _TMinuta, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00145

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00579	       if Pminuta = tminuta and decsec = 0 then   'pocetna vrednost
	CMPEQ?WWB	_PMinuta, _TMinuta, T1
	CMPEQ?BCB	_DECSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00147

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00580	          kanal4 = osvpocetak
	MOVE?BB	_OsvPocetak, _Kanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00581	          stkanal4 = osvpocetak * 100
	MUL?BCN	_OsvPocetak, 064h, _STKanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00582	       else
	GOTO?L	L00148
	LABEL?L	L00147	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00583	          if kminuta = tminuta and decsec = 0 then   'krajnja vrednost 
	CMPEQ?WWB	_KMinuta, _TMinuta, T1
	CMPEQ?BCB	_DECSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00149

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00584	              kanal4 = osvkraj
	MOVE?BB	_OsvKraj, _Kanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00585	          else
	GOTO?L	L00150
	LABEL?L	L00149	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00586	              if noviminut = 1 then 
	CMPNE?BCL	_NoviMinut, 001h, L00151

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00587	                 stkanal4 = stkanal4 + (((osvkraj - osvpocetak)*100/(kminuta - pminuta)))  
	SUB?BBN	_OsvKraj, _OsvPocetak, T1
	MUL?NCN	T1, 064h, T1
	SUB?WWN	_KMinuta, _PMinuta, T2
	DIV?NNN	T1, T2, T2
	ADD?NNN	_STKanal4, T2, _STKanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00588	                 kanal4 = stkanal4 / 100 
	DIV?NCB	_STKanal4, 064h, _Kanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00589	              endif
	LABEL?L	L00151	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00590	          endif
	LABEL?L	L00150	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00591	       endif
	LABEL?L	L00148	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00592	    endif 
	LABEL?L	L00145	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00594	    satPocetak   = memorija[64] 
	MOVE?BB	_Memorija + 00040h, _SatPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00595	    MinutPocetak = memorija[65]  
	MOVE?BB	_Memorija + 00041h, _MinutPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00596	    osvPocetak   = memorija[66] 
	MOVE?BB	_Memorija + 00042h, _OsvPocetak

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00597	    SatKraj      = memorija[68]  
	MOVE?BB	_Memorija + 00044h, _SatKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00598	    MinutKraj    = memorija[69]  
	MOVE?BB	_Memorija + 00045h, _MinutKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00599	    OsvKraj      = memorija[70]  
	MOVE?BB	_Memorija + 00046h, _OsvKraj

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00600	    PMinuta = (satpocetak * 60) + minutpocetak
	MUL?BCN	_SatPocetak, 03Ch, T1
	ADD?NBW	T1, _MinutPocetak, _PMinuta

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00601	    KMinuta = (satkraj    * 60) + minutkraj
	MUL?BCN	_SatKraj, 03Ch, T1
	ADD?NBW	T1, _MinutKraj, _KMinuta

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00603	if Pminuta =< tminuta and KMinuta >= tminuta then 
	CMPLE?WWB	_PMinuta, _TMinuta, T1
	CMPGE?WWB	_KMinuta, _TMinuta, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00153

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00604	       if Pminuta = tminuta and decsec = 0 then   'pocetna vrednost
	CMPEQ?WWB	_PMinuta, _TMinuta, T1
	CMPEQ?BCB	_DECSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00155

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00605	          kanal4 = osvpocetak
	MOVE?BB	_OsvPocetak, _Kanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00606	          stkanal4 = osvpocetak * 100
	MUL?BCN	_OsvPocetak, 064h, _STKanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00607	       else
	GOTO?L	L00156
	LABEL?L	L00155	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00608	          if kminuta = tminuta and decsec = 0 then   'krajnja vrednost 
	CMPEQ?WWB	_KMinuta, _TMinuta, T1
	CMPEQ?BCB	_DECSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00157

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00609	              kanal4 = osvkraj
	MOVE?BB	_OsvKraj, _Kanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00610	          else
	GOTO?L	L00158
	LABEL?L	L00157	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00611	              if noviminut = 1 then
	CMPNE?BCL	_NoviMinut, 001h, L00159

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00612	                 stkanal4 = stkanal4 - (((osvpocetak - osvkraj)*100/(kminuta - pminuta)))  
	SUB?BBN	_OsvPocetak, _OsvKraj, T1
	MUL?NCN	T1, 064h, T1
	SUB?WWN	_KMinuta, _PMinuta, T2
	DIV?NNN	T1, T2, T2
	SUB?NNN	_STKanal4, T2, _STKanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00613	                 kanal4 = stkanal4 / 100 
	DIV?NCB	_STKanal4, 064h, _Kanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00614	              endif
	LABEL?L	L00159	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00615	          endif
	LABEL?L	L00158	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00616	       endif
	LABEL?L	L00156	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00617	    endif 
	LABEL?L	L00153	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00618	 write 84,kanal1
	WRITEADDRESS?C	054h
	WRITE?B	_Kanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00619	 write 85,kanal2
	WRITEADDRESS?C	055h
	WRITE?B	_Kanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00620	 write 86,kanal3
	WRITEADDRESS?C	056h
	WRITE?B	_Kanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00621	 write 87,kanal4
	WRITEADDRESS?C	057h
	WRITE?B	_Kanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00623	    starostanjeRTC = novostanjeRTC 
	MOVE?BB	_NovostanjeRTC, _StarostanjeRTC

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00624	return
	RETURN?	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00626	Menu:

	LABEL?L	_menu	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00627	    novostanjedesno  = tright
	MOVE?TT	_TRIGHT, _NovoStanjeDesno

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00628	    novostanjelevo   = tleft
	MOVE?TT	_TLEFT, _NovoStanjeLevo

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00629	    novostanjeGore   = tup
	MOVE?TT	_TUP, _NovoStanjeGore

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00630	    novostanjedole   = tdown
	MOVE?TT	_TDOWN, _NovoStanjeDole

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00631	    if BezTastera< 3000 then 
	CMPGE?WCL	_BezTastera, 000000BB8h, L00161

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00632	       BezTastera =  BezTastera + 1 
	ADD?WCW	_BezTastera, 001h, _BezTastera

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00633	       bcklight = 1
	MOVE?CT	001h, _BCKLIGHT

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00634	    else
	GOTO?L	L00162
	LABEL?L	L00161	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00635	        bcklight = 0 
	MOVE?CT	000h, _BCKLIGHT

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00636	    endif
	LABEL?L	L00162	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00637	    if odrzavanje = 0 then 
	CMPNE?TCL	_ODRZAVANJE, 000h, L00163

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00638	       StaraV1 = kanal1
	MOVE?BB	_Kanal1, _StaraV1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00639	       StaraV2 = kanal2
	MOVE?BB	_Kanal2, _StaraV2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00640	       StaraV3 = kanal3
	MOVE?BB	_Kanal3, _StaraV3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00641	       StaraV4 = kanal4
	MOVE?BB	_Kanal4, _StaraV4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00642	       read 2,osvetljenjeOdrzavanje
	READADDRESS?C	002h
	READ?B	_osvetljenjeOdrzavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00643	       goto OdrzavanjePrekidac
	GOTO?L	_OdrzavanjePrekidac

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00644	    endif
	LABEL?L	L00163	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00645	    if novostanjegore < starostanjegore then 
	CMPGE?TTL	_NovoStanjeGore, _StaroStanjeGore, L00165

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00646	       BezTastera = 0
	MOVE?CW	000h, _BezTastera

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00647	       if bcklight = 1 then
	CMPNE?TCL	_BCKLIGHT, 001h, L00167

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00648	          i = i + 1 
	ADD?BCB	_i, 001h, _i

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00649	          if i > 1 then  
	CMPLE?BCL	_i, 001h, L00169

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00650	             meni = meni + 4
	ADD?BCB	_Meni, 004h, _Meni

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00651	             if meni > 72 then meni = 0
	CMPLE?BCL	_Meni, 048h, L00171
	MOVE?CB	000h, _Meni
	LABEL?L	L00171	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00652	          endif
	LABEL?L	L00169	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00653	       endif
	LABEL?L	L00167	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00654	    ENDIF
	LABEL?L	L00165	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00656	    if novostanjedole < starostanjedole then 
	CMPGE?TTL	_NovoStanjeDole, _StaroStanjeDole, L00173

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00657	       BezTastera = 0
	MOVE?CW	000h, _BezTastera

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00658	       if bcklight = 1 then
	CMPNE?TCL	_BCKLIGHT, 001h, L00175

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00659	          i = i + 1 
	ADD?BCB	_i, 001h, _i

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00660	          if i > 1 then
	CMPLE?BCL	_i, 001h, L00177

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00661	               if meni = 0 then 
	CMPNE?BCL	_Meni, 000h, L00179

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00662	                  meni = 72
	MOVE?CB	048h, _Meni

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00663	               else
	GOTO?L	L00180
	LABEL?L	L00179	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00664	                   meni = meni - 4
	SUB?BCB	_Meni, 004h, _Meni

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00665	               endif
	LABEL?L	L00180	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00666	          endif
	LABEL?L	L00177	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00667	       endif
	LABEL?L	L00175	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00668	    ENDIF
	LABEL?L	L00173	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00670	    if novostanjelevo < starostanjelevo then
	CMPGE?TTL	_NovoStanjeLevo, _StaroStanjeLevo, L00181

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00671	       beztastera = 0
	MOVE?CW	000h, _BezTastera

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00672	       if bcklight = 1 then
	CMPNE?TCL	_BCKLIGHT, 001h, L00183

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00673	          i = i + 1
	ADD?BCB	_i, 001h, _i

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00674	          if i > 1 then
	CMPLE?BCL	_i, 001h, L00185

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00675	             meni = 72
	MOVE?CB	048h, _Meni

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00676	             i = 0
	MOVE?CB	000h, _i

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00677	          endif
	LABEL?L	L00185	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00678	       endif
	LABEL?L	L00183	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00679	    ENDIF
	LABEL?L	L00181	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00681	Select case meni

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00682	    case 0
	CMPNE?BCL	_Meni, 000h, L00189

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00683	    Lcdout $fe, 2,">RTC time       "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	03Eh
	LCDOUT?C	052h
	LCDOUT?C	054h
	LCDOUT?C	043h
	LCDOUT?C	020h
	LCDOUT?C	074h
	LCDOUT?C	069h
	LCDOUT?C	06Dh
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00684	    Lcdout $fe,$c0," Mode             "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
	LCDOUT?C	04Dh
	LCDOUT?C	06Fh
	LCDOUT?C	064h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00686	    case 4
	GOTO?L	L00188
	LABEL?L	L00189	
	CMPNE?BCL	_Meni, 004h, L00190

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00687	    Lcdout $fe, 2," RTC time        "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
	LCDOUT?C	052h
	LCDOUT?C	054h
	LCDOUT?C	043h
	LCDOUT?C	020h
	LCDOUT?C	074h
	LCDOUT?C	069h
	LCDOUT?C	06Dh
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00688	    Lcdout $fe,$c0,">Mode             "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	03Eh
	LCDOUT?C	04Dh
	LCDOUT?C	06Fh
	LCDOUT?C	064h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00690	    case 8 
	GOTO?L	L00188
	LABEL?L	L00190	
	CMPNE?BCL	_Meni, 008h, L00191

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00691	    Lcdout $fe, 2,">#1 Sunrise ON      "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	03Eh
	LCDOUT?C	023h
	LCDOUT?C	031h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00692	    lcdout $fe,$c0," #1 Sunrise OFF      "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	031h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00694	    case 12 
	GOTO?L	L00188
	LABEL?L	L00191	
	CMPNE?BCL	_Meni, 00Ch, L00192

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00695	    Lcdout $fe, 2," #1 Sunrise ON     "           
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	031h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00696	    lcdout $fe,$c0,">#1 Sunrise OFF     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	03Eh
	LCDOUT?C	023h
	LCDOUT?C	031h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00698	    case 16
	GOTO?L	L00188
	LABEL?L	L00192	
	CMPNE?BCL	_Meni, 010h, L00193

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00699	    Lcdout $fe, 2,">#1 Sunset ON     "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	03Eh
	LCDOUT?C	023h
	LCDOUT?C	031h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00700	    lcdout $fe,$c0," #1 Sunset OFF     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	031h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00702	    case 20
	GOTO?L	L00188
	LABEL?L	L00193	
	CMPNE?BCL	_Meni, 014h, L00194

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00703	    Lcdout $fe, 2," #1 Sunset ON     "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	031h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00704	    lcdout $fe,$c0,">#1 Sunset OFF     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	03Eh
	LCDOUT?C	023h
	LCDOUT?C	031h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00706	    case 24 
	GOTO?L	L00188
	LABEL?L	L00194	
	CMPNE?BCL	_Meni, 018h, L00195

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00707	    Lcdout $fe, 2,">#2 Sunrise ON     "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	03Eh
	LCDOUT?C	023h
	LCDOUT?C	032h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00708	    lcdout $fe,$c0," #2 Sunrise OFF     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	032h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00710	    case 28 
	GOTO?L	L00188
	LABEL?L	L00195	
	CMPNE?BCL	_Meni, 01Ch, L00196

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00711	    Lcdout $fe, 2," #2 Sunrise ON     "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	032h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00712	    lcdout $fe,$c0,">#2 Sunrise OFF     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	03Eh
	LCDOUT?C	023h
	LCDOUT?C	032h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00714	    case 32
	GOTO?L	L00188
	LABEL?L	L00196	
	CMPNE?BCL	_Meni, 020h, L00197

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00715	    Lcdout $fe, 2,">#2 Sunset ON     "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	03Eh
	LCDOUT?C	023h
	LCDOUT?C	032h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00716	    lcdout $fe,$c0," #2 Sunset OFF     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	032h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00718	    case 36
	GOTO?L	L00188
	LABEL?L	L00197	
	CMPNE?BCL	_Meni, 024h, L00198

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00719	    Lcdout $fe, 2," #2 Sunset ON     "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	032h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00720	    lcdout $fe,$c0,">#2 Sunset OFF     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	03Eh
	LCDOUT?C	023h
	LCDOUT?C	032h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00722	    case 40 
	GOTO?L	L00188
	LABEL?L	L00198	
	CMPNE?BCL	_Meni, 028h, L00199

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00723	    Lcdout $fe, 2,">#3 Sunrise ON     "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	03Eh
	LCDOUT?C	023h
	LCDOUT?C	033h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00724	    lcdout $fe,$c0," #3 Sunrise OFF     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	033h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00726	    case 44 
	GOTO?L	L00188
	LABEL?L	L00199	
	CMPNE?BCL	_Meni, 02Ch, L00200

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00727	    Lcdout $fe, 2," #3 Sunrise ON     "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	033h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00728	    lcdout $fe,$c0,">#3 Sunrise OFF     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	03Eh
	LCDOUT?C	023h
	LCDOUT?C	033h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00730	    case 48
	GOTO?L	L00188
	LABEL?L	L00200	
	CMPNE?BCL	_Meni, 030h, L00201

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00731	    Lcdout $fe, 2,">#3 Sunset ON     "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	03Eh
	LCDOUT?C	023h
	LCDOUT?C	033h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00732	    lcdout $fe,$c0," #3 Sunset OFF     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	033h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00734	    case 52 
	GOTO?L	L00188
	LABEL?L	L00201	
	CMPNE?BCL	_Meni, 034h, L00202

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00735	    Lcdout $fe, 2," #3 Sunset ON     "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	033h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00736	    lcdout $fe,$c0,">#3 Sunset OFF     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	03Eh
	LCDOUT?C	023h
	LCDOUT?C	033h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00738	    case 56 
	GOTO?L	L00188
	LABEL?L	L00202	
	CMPNE?BCL	_Meni, 038h, L00203

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00739	    Lcdout $fe, 2,">#4 Sunrise ON     "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	03Eh
	LCDOUT?C	023h
	LCDOUT?C	034h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00740	    lcdout $fe,$c0," #4 Sunrise OFF     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	034h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00742	    case 60 
	GOTO?L	L00188
	LABEL?L	L00203	
	CMPNE?BCL	_Meni, 03Ch, L00204

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00743	    Lcdout $fe, 2," #4 Sunrise ON     "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	034h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00744	    lcdout $fe,$c0,">#4 Sunrise OFF     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	03Eh
	LCDOUT?C	023h
	LCDOUT?C	034h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00746	    case 64
	GOTO?L	L00188
	LABEL?L	L00204	
	CMPNE?BCL	_Meni, 040h, L00205

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00747	    Lcdout $fe, 2,">#4 Sunset ON     "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	03Eh
	LCDOUT?C	023h
	LCDOUT?C	034h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00748	    lcdout $fe,$c0," #4 Sunset OFF     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	034h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00750	    case 68 
	GOTO?L	L00188
	LABEL?L	L00205	
	CMPNE?BCL	_Meni, 044h, L00206

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00751	    Lcdout $fe, 2," #4 Sunset ON     "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	034h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00752	    lcdout $fe,$c0,">#4 Sunset OFF     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	03Eh
	LCDOUT?C	023h
	LCDOUT?C	034h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00754	    case 72
	GOTO?L	L00188
	LABEL?L	L00206	
	CMPNE?BCL	_Meni, 048h, L00207

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00755	        read 3,x
	READADDRESS?C	003h
	READ?B	_x

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00756	        if x = 1 then goto main 
	CMPNE?BCL	_x, 001h, L00208
	GOTO?L	_Main
	LABEL?L	L00208	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00757	        if x = 2 then
	CMPNE?BCL	_x, 002h, L00210

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00758	           i = 0
	MOVE?CB	000h, _i

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00759	           KURSOR = 1
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00760	           read 80,rkanal1
	READADDRESS?C	050h
	READ?B	_RKanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00761	           read 81,rkanal2
	READADDRESS?C	051h
	READ?B	_RKanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00762	           read 82,rkanal3
	READADDRESS?C	052h
	READ?B	_RKanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00763	           read 83,rkanal4
	READADDRESS?C	053h
	READ?B	_RKanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00764	           goto RucnoPodesavanje
	GOTO?L	_RucnoPodesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00765	        endif
	LABEL?L	L00210	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00766	end select
	LABEL?L	L00207	
	LABEL?L	L00188	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00767	if novostanjedesno < starostanjedesno then
	CMPGE?TTL	_NovoStanjeDesno, _StaroStanjeDesno, L00212

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00768	          x =  0 
	MOVE?CB	000h, _x

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00769	          BezTastera = 0
	MOVE?CW	000h, _BezTastera

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00770	          StaraV1 = kanal1
	MOVE?BB	_Kanal1, _StaraV1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00771	          StaraV2 = kanal2
	MOVE?BB	_Kanal2, _StaraV2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00772	          StaraV3 = kanal3
	MOVE?BB	_Kanal3, _StaraV3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00773	          StaraV4 = kanal4
	MOVE?BB	_Kanal4, _StaraV4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00774	          if bcklight = 1 then 
	CMPNE?TCL	_BCKLIGHT, 001h, L00214

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00775	            kursor = 0
	MOVE?CB	000h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00776	            if meni = 0 then
	CMPNE?BCL	_Meni, 000h, L00216

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00777	               T1CON.0=0
	MOVE?CT	000h, _T1CON??0

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00778	               I2CRead SDA,SCL,$D0,$00,[RTCSec,RTCMin,RTCHour,RTCDay,RTCDate,RTCMonth,RTCYear]		
	I2CDATA?T	_SDA
	I2CCLOCK?T	_SCL
	I2CWRITE?C	0D0h
	I2CWRITE?C	000h
	I2CREAD?B	_RTCSec
	I2CREAD?B	_RTCMin
	I2CREAD?B	_RTCHour
	I2CREAD?B	_RTCDay
	I2CREAD?B	_RTCDate
	I2CREAD?B	_RTCMonth
	I2CREADS?B	_RTCYear

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00779	               T1CON.0=1
	MOVE?CT	001h, _T1CON??0

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00780	               decmin = ((RTCMIN  & %11110000)>>4)  * 10 + (RTCMIN  & %00001111)
	AND?BCN	_RTCMin, 0F0h, T1
	SHIFTR?NCN	T1, 004h, T1
	MUL?NCN	T1, 00Ah, T1
	AND?BCN	_RTCMin, 00Fh, T2
	ADD?NNB	T1, T2, _DECmin

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00781	               decsat = ((RTCHOUR & %11110000)>>4)  * 10 + (RTCHOUR & %00001111)
	AND?BCN	_RTCHour, 0F0h, T1
	SHIFTR?NCN	T1, 004h, T1
	MUL?NCN	T1, 00Ah, T1
	AND?BCN	_RTCHour, 00Fh, T2
	ADD?NNB	T1, T2, _DECsat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00782	               decsec = ((RTCsec  & %11110000)>>4)  * 10 + (RTCsec  & %00001111) 
	AND?BCN	_RTCSec, 0F0h, T1
	SHIFTR?NCN	T1, 004h, T1
	MUL?NCN	T1, 00Ah, T1
	AND?BCN	_RTCSec, 00Fh, T2
	ADD?NNB	T1, T2, _DECSec

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00783	               goto RTCvreme
	GOTO?L	_RTCvreme

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00784	            endif
	LABEL?L	L00216	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00786	            if meni = 4 then
	CMPNE?BCL	_Meni, 004h, L00218

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00787	               read 3,x 
	READADDRESS?C	003h
	READ?B	_x

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00788	               goto RezimRada
	GOTO?L	_RezimRada

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00789	            endif
	LABEL?L	L00218	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00790	            if meni = 8 then
	CMPNE?BCL	_Meni, 008h, L00220

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00791	               Kanal = 1
	MOVE?CB	001h, _Kanal

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00792	               svitanje = 1
	MOVE?CB	001h, _Svitanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00793	               pali = 1
	MOVE?CB	001h, _Pali

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00794	               kursor = 1
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00795	               kanal1 = 0
	MOVE?CB	000h, _Kanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00796	               read meni,sat
	READADDRESS?B	_Meni
	READ?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00797	               read meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	READADDRESS?N	T1
	READ?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00798	               read meni + 2,osvetljenje       
	ADD?BCN	_Meni, 002h, T1
	READADDRESS?N	T1
	READ?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00799	               goto podesavanje
	GOTO?L	_podesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00800	            endif
	LABEL?L	L00220	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00801	            if meni = 12 then
	CMPNE?BCL	_Meni, 00Ch, L00222

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00802	               Kanal = 1
	MOVE?CB	001h, _Kanal

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00803	               svitanje = 1
	MOVE?CB	001h, _Svitanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00804	               pali = 0  
	MOVE?CB	000h, _Pali

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00805	               kursor = 1
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00806	               read meni,sat
	READADDRESS?B	_Meni
	READ?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00807	               read meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	READADDRESS?N	T1
	READ?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00808	               read meni + 2,osvetljenje         
	ADD?BCN	_Meni, 002h, T1
	READADDRESS?N	T1
	READ?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00809	               goto podesavanje
	GOTO?L	_podesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00810	            endif 
	LABEL?L	L00222	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00811	            if meni = 16 then
	CMPNE?BCL	_Meni, 010h, L00224

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00812	               Kanal = 1
	MOVE?CB	001h, _Kanal

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00813	               svitanje = 0
	MOVE?CB	000h, _Svitanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00814	               pali = 1 
	MOVE?CB	001h, _Pali

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00815	               kursor = 1 
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00816	               read meni,sat
	READADDRESS?B	_Meni
	READ?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00817	               read meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	READADDRESS?N	T1
	READ?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00818	               read meni + 2,osvetljenje         
	ADD?BCN	_Meni, 002h, T1
	READADDRESS?N	T1
	READ?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00819	               goto podesavanje
	GOTO?L	_podesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00820	            endif 
	LABEL?L	L00224	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00821	            if meni = 20 then
	CMPNE?BCL	_Meni, 014h, L00226

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00822	               Kanal = 1 
	MOVE?CB	001h, _Kanal

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00823	               svitanje = 0
	MOVE?CB	000h, _Svitanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00824	               pali = 0 
	MOVE?CB	000h, _Pali

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00825	               kursor = 1
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00826	               read meni,sat
	READADDRESS?B	_Meni
	READ?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00827	               read meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	READADDRESS?N	T1
	READ?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00828	               read meni + 2,osvetljenje       
	ADD?BCN	_Meni, 002h, T1
	READADDRESS?N	T1
	READ?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00829	               goto podesavanje
	GOTO?L	_podesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00830	            endif
	LABEL?L	L00226	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00831	            if meni = 24 then
	CMPNE?BCL	_Meni, 018h, L00228

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00832	               Kanal = 2
	MOVE?CB	002h, _Kanal

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00833	               svitanje = 1
	MOVE?CB	001h, _Svitanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00834	               pali = 1 
	MOVE?CB	001h, _Pali

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00835	               kursor = 1
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00836	               read meni,sat
	READADDRESS?B	_Meni
	READ?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00837	               read meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	READADDRESS?N	T1
	READ?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00838	               read meni + 2,osvetljenje         
	ADD?BCN	_Meni, 002h, T1
	READADDRESS?N	T1
	READ?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00839	               goto podesavanje
	GOTO?L	_podesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00840	            endif
	LABEL?L	L00228	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00841	            if meni = 28 then
	CMPNE?BCL	_Meni, 01Ch, L00230

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00842	               Kanal = 2
	MOVE?CB	002h, _Kanal

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00843	               svitanje = 1
	MOVE?CB	001h, _Svitanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00844	               pali = 0 
	MOVE?CB	000h, _Pali

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00845	               kursor = 1
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00846	               read meni,sat
	READADDRESS?B	_Meni
	READ?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00847	               read meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	READADDRESS?N	T1
	READ?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00848	               read meni + 2,osvetljenje          
	ADD?BCN	_Meni, 002h, T1
	READADDRESS?N	T1
	READ?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00849	               goto podesavanje
	GOTO?L	_podesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00850	            endif 
	LABEL?L	L00230	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00851	            if meni = 32 then
	CMPNE?BCL	_Meni, 020h, L00232

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00852	               Kanal = 2
	MOVE?CB	002h, _Kanal

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00853	               svitanje = 0
	MOVE?CB	000h, _Svitanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00854	               pali = 1  
	MOVE?CB	001h, _Pali

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00855	               kursor = 1 
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00856	               read meni,sat
	READADDRESS?B	_Meni
	READ?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00857	               read meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	READADDRESS?N	T1
	READ?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00858	               read meni + 2,osvetljenje       
	ADD?BCN	_Meni, 002h, T1
	READADDRESS?N	T1
	READ?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00859	               goto podesavanje
	GOTO?L	_podesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00860	            endif 
	LABEL?L	L00232	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00861	            if meni = 36 then
	CMPNE?BCL	_Meni, 024h, L00234

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00862	               Kanal = 2 
	MOVE?CB	002h, _Kanal

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00863	               svitanje = 0
	MOVE?CB	000h, _Svitanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00864	               pali = 0 
	MOVE?CB	000h, _Pali

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00865	               kursor = 1 
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00866	               read meni,sat
	READADDRESS?B	_Meni
	READ?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00867	               read meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	READADDRESS?N	T1
	READ?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00868	               read meni + 2,osvetljenje     
	ADD?BCN	_Meni, 002h, T1
	READADDRESS?N	T1
	READ?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00869	               goto podesavanje
	GOTO?L	_podesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00870	            endif
	LABEL?L	L00234	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00871	            if meni = 40 then
	CMPNE?BCL	_Meni, 028h, L00236

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00872	               Kanal = 3
	MOVE?CB	003h, _Kanal

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00873	               svitanje = 1
	MOVE?CB	001h, _Svitanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00874	               pali = 1 
	MOVE?CB	001h, _Pali

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00875	               kursor = 1 
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00876	               read meni,sat
	READADDRESS?B	_Meni
	READ?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00877	               read meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	READADDRESS?N	T1
	READ?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00878	               read meni + 2,osvetljenje        
	ADD?BCN	_Meni, 002h, T1
	READADDRESS?N	T1
	READ?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00879	               goto podesavanje
	GOTO?L	_podesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00880	            endif
	LABEL?L	L00236	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00881	            if meni = 44 then
	CMPNE?BCL	_Meni, 02Ch, L00238

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00882	               Kanal = 3
	MOVE?CB	003h, _Kanal

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00883	               svitanje = 1
	MOVE?CB	001h, _Svitanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00884	               pali = 0  
	MOVE?CB	000h, _Pali

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00885	               kursor = 1 
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00886	               read meni,sat
	READADDRESS?B	_Meni
	READ?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00887	               read meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	READADDRESS?N	T1
	READ?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00888	               read meni + 2,osvetljenje        
	ADD?BCN	_Meni, 002h, T1
	READADDRESS?N	T1
	READ?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00889	               goto podesavanje
	GOTO?L	_podesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00890	            endif 
	LABEL?L	L00238	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00891	            if meni = 48 then
	CMPNE?BCL	_Meni, 030h, L00240

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00892	               Kanal = 3
	MOVE?CB	003h, _Kanal

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00893	               svitanje = 0
	MOVE?CB	000h, _Svitanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00894	               pali = 1
	MOVE?CB	001h, _Pali

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00895	               kursor = 1 
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00896	               read meni,sat
	READADDRESS?B	_Meni
	READ?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00897	               read meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	READADDRESS?N	T1
	READ?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00898	               read meni + 2,osvetljenje          
	ADD?BCN	_Meni, 002h, T1
	READADDRESS?N	T1
	READ?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00899	               goto podesavanje
	GOTO?L	_podesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00900	            endif 
	LABEL?L	L00240	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00901	            if meni = 52 then
	CMPNE?BCL	_Meni, 034h, L00242

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00902	               Kanal = 3 
	MOVE?CB	003h, _Kanal

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00903	               svitanje = 0
	MOVE?CB	000h, _Svitanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00904	               pali = 0  
	MOVE?CB	000h, _Pali

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00905	               kursor = 1
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00906	               read meni,sat
	READADDRESS?B	_Meni
	READ?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00907	               read meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	READADDRESS?N	T1
	READ?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00908	               read meni + 2,osvetljenje       
	ADD?BCN	_Meni, 002h, T1
	READADDRESS?N	T1
	READ?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00909	               goto podesavanje
	GOTO?L	_podesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00910	            endif
	LABEL?L	L00242	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00911	            if meni = 56 then
	CMPNE?BCL	_Meni, 038h, L00244

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00912	               Kanal = 4
	MOVE?CB	004h, _Kanal

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00913	               svitanje = 1
	MOVE?CB	001h, _Svitanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00914	               pali = 1
	MOVE?CB	001h, _Pali

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00915	               kursor = 1 
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00916	               read meni,sat
	READADDRESS?B	_Meni
	READ?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00917	               read meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	READADDRESS?N	T1
	READ?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00918	               read meni + 2,osvetljenje         
	ADD?BCN	_Meni, 002h, T1
	READADDRESS?N	T1
	READ?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00919	               goto podesavanje
	GOTO?L	_podesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00920	            endif
	LABEL?L	L00244	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00921	            if meni = 60 then
	CMPNE?BCL	_Meni, 03Ch, L00246

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00922	               Kanal = 4
	MOVE?CB	004h, _Kanal

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00923	               svitanje = 1
	MOVE?CB	001h, _Svitanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00924	               pali = 0 
	MOVE?CB	000h, _Pali

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00925	               kursor = 1 
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00926	               read meni,sat
	READADDRESS?B	_Meni
	READ?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00927	               read meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	READADDRESS?N	T1
	READ?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00928	               read meni + 2,osvetljenje        
	ADD?BCN	_Meni, 002h, T1
	READADDRESS?N	T1
	READ?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00929	               goto podesavanje
	GOTO?L	_podesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00930	            endif 
	LABEL?L	L00246	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00931	            if meni = 64 then
	CMPNE?BCL	_Meni, 040h, L00248

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00932	               Kanal = 4
	MOVE?CB	004h, _Kanal

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00933	               svitanje = 0
	MOVE?CB	000h, _Svitanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00934	               pali = 1
	MOVE?CB	001h, _Pali

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00935	               kursor = 1
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00936	               read meni,sat
	READADDRESS?B	_Meni
	READ?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00937	               read meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	READADDRESS?N	T1
	READ?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00938	               read meni + 2,osvetljenje           
	ADD?BCN	_Meni, 002h, T1
	READADDRESS?N	T1
	READ?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00939	               goto podesavanje
	GOTO?L	_podesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00940	            endif 
	LABEL?L	L00248	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00941	            if meni = 68 then
	CMPNE?BCL	_Meni, 044h, L00250

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00942	               Kanal = 4 
	MOVE?CB	004h, _Kanal

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00943	               svitanje = 0
	MOVE?CB	000h, _Svitanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00944	               pali = 0 
	MOVE?CB	000h, _Pali

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00945	               kursor = 1
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00946	               read meni,sat
	READADDRESS?B	_Meni
	READ?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00947	               read meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	READADDRESS?N	T1
	READ?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00948	               read meni + 2,osvetljenje       
	ADD?BCN	_Meni, 002h, T1
	READADDRESS?N	T1
	READ?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00949	               goto podesavanje
	GOTO?L	_podesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00950	            endif
	LABEL?L	L00250	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00951	         endif
	LABEL?L	L00214	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00952	ENDIF
	LABEL?L	L00212	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00953	    gosub Kreni
	GOSUB?L	_Kreni

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00954	    STAROSTANJEgore=NOVOSTANJEgore
	MOVE?TT	_NovoStanjeGore, _StaroStanjeGore

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00955	    STAROSTANJEdesno=NOVOSTANJEdesno
	MOVE?TT	_NovoStanjeDesno, _StaroStanjeDesno

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00956	    STAROSTANJEdole=NOVOSTANJEdole
	MOVE?TT	_NovoStanjeDole, _StaroStanjeDole

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00957	    STAROSTANJElevo=NOVOSTANJElevo
	MOVE?TT	_NovoStanjeLevo, _StaroStanjeLevo

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00958	goto menu
	GOTO?L	_menu

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00959	RezimRada:

	LABEL?L	_RezimRada	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00960	     bcklight = 1
	MOVE?CT	001h, _BCKLIGHT

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00961	     novostanjelevo = tleft
	MOVE?TT	_TLEFT, _NovoStanjeLevo

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00962	     novostanjeGore = tup
	MOVE?TT	_TUP, _NovoStanjeGore

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00963	     novostanjedole = tdown
	MOVE?TT	_TDOWN, _NovoStanjeDole

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00965	     if novostanjelevo < starostanjelevo then
	CMPGE?TTL	_NovoStanjeLevo, _StaroStanjeLevo, L00252

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00966	        i = 0
	MOVE?CB	000h, _i

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00967	        goto menu
	GOTO?L	_menu

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00968	     ENDIF 
	LABEL?L	L00252	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00969	     if novostanjegore < starostanjegore then
	CMPGE?TTL	_NovoStanjeGore, _StaroStanjeGore, L00254

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00970	       if x = 1 then 
	CMPNE?BCL	_x, 001h, L00256

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00971	          x = 2 
	MOVE?CB	002h, _x

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00972	       else
	GOTO?L	L00257
	LABEL?L	L00256	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00973	          x = 1
	MOVE?CB	001h, _x

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00974	       endif
	LABEL?L	L00257	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00975	       write 3,x
	WRITEADDRESS?C	003h
	WRITE?B	_x

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00976	    endif
	LABEL?L	L00254	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00977	    if novostanjedole < starostanjedole then
	CMPGE?TTL	_NovoStanjeDole, _StaroStanjeDole, L00258

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00978	       if x = 1 then 
	CMPNE?BCL	_x, 001h, L00260

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00979	          x = 2 
	MOVE?CB	002h, _x

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00980	       else
	GOTO?L	L00261
	LABEL?L	L00260	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00981	          x = 1
	MOVE?CB	001h, _x

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00982	       endif
	LABEL?L	L00261	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00983	       write 3,x
	WRITEADDRESS?C	003h
	WRITE?B	_x

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00984	    endif   
	LABEL?L	L00258	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00986	     Lcdout $fe, 2,"Mode          "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	04Dh
	LCDOUT?C	06Fh
	LCDOUT?C	064h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00987	     if x = 1 then lcdout $fe,$c0,">Automatic        "
	CMPNE?BCL	_x, 001h, L00262
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	03Eh
	LCDOUT?C	041h
	LCDOUT?C	075h
	LCDOUT?C	074h
	LCDOUT?C	06Fh
	LCDOUT?C	06Dh
	LCDOUT?C	061h
	LCDOUT?C	074h
	LCDOUT?C	069h
	LCDOUT?C	063h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LABEL?L	L00262	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00988	     if x = 2 then lcdout $fe,$c0,">Manual             " 
	CMPNE?BCL	_x, 002h, L00264
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	03Eh
	LCDOUT?C	04Dh
	LCDOUT?C	061h
	LCDOUT?C	06Eh
	LCDOUT?C	075h
	LCDOUT?C	061h
	LCDOUT?C	06Ch
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LABEL?L	L00264	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00990	     STAROSTANJElevo = NOVOSTANJElevo 
	MOVE?TT	_NovoStanjeLevo, _StaroStanjeLevo

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00991	     STAROSTANJEgore = NOVOSTANJEgore
	MOVE?TT	_NovoStanjeGore, _StaroStanjeGore

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00992	     STAROSTANJEdole = NOVOSTANJEdole
	MOVE?TT	_NovoStanjeDole, _StaroStanjeDole

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00993	goto RezimRada
	GOTO?L	_RezimRada

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00995	Podesavanje:

	LABEL?L	_podesavanje	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00996	     bcklight = 1
	MOVE?CT	001h, _BCKLIGHT

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00997	     novostanjelevo = tleft
	MOVE?TT	_TLEFT, _NovoStanjeLevo

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00998	     novostanjedesno = tright
	MOVE?TT	_TRIGHT, _NovoStanjeDesno

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	00999	     novostanjeGore= tup
	MOVE?TT	_TUP, _NovoStanjeGore

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01000	     novostanjedole= tdown
	MOVE?TT	_TDOWN, _NovoStanjeDole

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01001	     if novostanjedesno < starostanjedesno then
	CMPGE?TTL	_NovoStanjeDesno, _StaroStanjeDesno, L00266

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01002	        x = x + 1 
	ADD?BCB	_x, 001h, _x

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01003	        if x > 1 then 
	CMPLE?BCL	_x, 001h, L00268

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01004	           x = 0 
	MOVE?CB	000h, _x

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01005	           Kursor =  Kursor + 1 
	ADD?BCB	_Kursor, 001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01006	           if kursor > 3 then kursor = 1
	CMPLE?BCL	_Kursor, 003h, L00270
	MOVE?CB	001h, _Kursor
	LABEL?L	L00270	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01007	        endif
	LABEL?L	L00268	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01008	     endif
	LABEL?L	L00266	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01010	     IF KURSOR = 1 THEN
	CMPNE?BCL	_Kursor, 001h, L00272

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01011	        displej[1] = ">"
	MOVE?CB	03Eh, _Displej + 00001h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01012	        displej[2] = " "
	MOVE?CB	020h, _Displej + 00002h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01013	        displej[3] = " "
	MOVE?CB	020h, _Displej + 00003h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01014	     endif 
	LABEL?L	L00272	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01015	     if kursor = 2 then
	CMPNE?BCL	_Kursor, 002h, L00274

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01016	        displej[1] = " "
	MOVE?CB	020h, _Displej + 00001h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01017	        displej[2] = ">"
	MOVE?CB	03Eh, _Displej + 00002h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01018	        displej[3] = " "
	MOVE?CB	020h, _Displej + 00003h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01019	     endif
	LABEL?L	L00274	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01020	     if kursor = 3 then 
	CMPNE?BCL	_Kursor, 003h, L00276

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01021	        displej[1] = " "
	MOVE?CB	020h, _Displej + 00001h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01022	        displej[2] = " "
	MOVE?CB	020h, _Displej + 00002h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01023	        displej[3] = ">"
	MOVE?CB	03Eh, _Displej + 00003h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01024	     endif
	LABEL?L	L00276	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01025	     if tup = 0 then
	CMPNE?TCL	_TUP, 000h, L00278

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01026	         if kursor = 1 then
	CMPNE?BCL	_Kursor, 001h, L00280

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01027	            saT = sat + 1
	ADD?BCB	_sat, 001h, _sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01028	            if sat > 23 then sat = 23
	CMPLE?BCL	_sat, 017h, L00282
	MOVE?CB	017h, _sat
	LABEL?L	L00282	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01029	            write meni,sat
	WRITEADDRESS?B	_Meni
	WRITE?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01030	         endif
	LABEL?L	L00280	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01032	         if kursor = 2 then
	CMPNE?BCL	_Kursor, 002h, L00284

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01033	            minut = minut + 1
	ADD?BCB	_minut, 001h, _minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01034	            if minut > 59 then minut = 59
	CMPLE?BCL	_minut, 03Bh, L00286
	MOVE?CB	03Bh, _minut
	LABEL?L	L00286	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01035	            write meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	WRITEADDRESS?N	T1
	WRITE?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01036	         endif
	LABEL?L	L00284	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01038	         if kursor = 3 then
	CMPNE?BCL	_Kursor, 003h, L00288

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01039	            osvetljenje = osvetljenje + 1 
	ADD?BCB	_osvetljenje, 001h, _osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01040	            if osvetljenje > 100 then osvetljenje = 100
	CMPLE?BCL	_osvetljenje, 064h, L00290
	MOVE?CB	064h, _osvetljenje
	LABEL?L	L00290	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01041	            write meni + 2,osvetljenje
	ADD?BCN	_Meni, 002h, T1
	WRITEADDRESS?N	T1
	WRITE?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01042	         endif
	LABEL?L	L00288	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01043	        pause 60
	PAUSE?C	03Ch

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01044	     endif 
	LABEL?L	L00278	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01046	     if tdown = 0 then
	CMPNE?TCL	_TDOWN, 000h, L00292

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01047	         if kursor = 1 then
	CMPNE?BCL	_Kursor, 001h, L00294

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01048	            saT = sat - 1
	SUB?BCB	_sat, 001h, _sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01049	            if sat < 1 or sat > 23 then sat = 0
	CMPLT?BCB	_sat, 001h, T1
	CMPGT?BCB	_sat, 017h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00296
	MOVE?CB	000h, _sat
	LABEL?L	L00296	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01050	            write meni,sat
	WRITEADDRESS?B	_Meni
	WRITE?B	_sat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01051	         endif
	LABEL?L	L00294	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01053	         if kursor = 2 then
	CMPNE?BCL	_Kursor, 002h, L00298

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01054	            minut = minut - 1
	SUB?BCB	_minut, 001h, _minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01055	            if minut < 1 or minut > 59 then minut = 0
	CMPLT?BCB	_minut, 001h, T1
	CMPGT?BCB	_minut, 03Bh, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00300
	MOVE?CB	000h, _minut
	LABEL?L	L00300	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01056	            write meni + 1,minut
	ADD?BCN	_Meni, 001h, T1
	WRITEADDRESS?N	T1
	WRITE?B	_minut

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01057	         endif
	LABEL?L	L00298	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01059	         if kursor = 3 then
	CMPNE?BCL	_Kursor, 003h, L00302

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01060	            osvetljenje = osvetljenje - 1 
	SUB?BCB	_osvetljenje, 001h, _osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01061	            if osvetljenje < 1 or osvetljenje > 100 then osvetljenje = 0
	CMPLT?BCB	_osvetljenje, 001h, T1
	CMPGT?BCB	_osvetljenje, 064h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00304
	MOVE?CB	000h, _osvetljenje
	LABEL?L	L00304	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01062	            write meni + 2,osvetljenje
	ADD?BCN	_Meni, 002h, T1
	WRITEADDRESS?N	T1
	WRITE?B	_osvetljenje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01063	         endif
	LABEL?L	L00302	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01064	         pause 60
	PAUSE?C	03Ch

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01065	     endif
	LABEL?L	L00292	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01066	     if sat   > 23 then sat   = 0
	CMPLE?BCL	_sat, 017h, L00306
	MOVE?CB	000h, _sat
	LABEL?L	L00306	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01067	     if minut > 59 then minut = 0
	CMPLE?BCL	_minut, 03Bh, L00308
	MOVE?CB	000h, _minut
	LABEL?L	L00308	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01069	     if osvetljenje > 100 then osvetljenje = 0
	CMPLE?BCL	_osvetljenje, 064h, L00310
	MOVE?CB	000h, _osvetljenje
	LABEL?L	L00310	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01070	     if svitanje = 1 then 
	CMPNE?BCL	_Svitanje, 001h, L00312

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01071	        IF pali = 1 then
	CMPNE?BCL	_Pali, 001h, L00314

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01072	          Lcdout $fe, 2,"Sunrise #",#kanal,"  ON          "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUTD?B	_Kanal
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01073	        else
	GOTO?L	L00315
	LABEL?L	L00314	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01074	           Lcdout $fe, 2,"Sunrise #",#kanal,"  OFF        "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUTD?B	_Kanal
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01075	        endif
	LABEL?L	L00315	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01076	     else
	GOTO?L	L00313
	LABEL?L	L00312	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01077	        IF pali = 1 then
	CMPNE?BCL	_Pali, 001h, L00316

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01078	          Lcdout $fe, 2,"Sunset #",#kanal,"  ON          " 
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUTD?B	_Kanal
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01079	        else
	GOTO?L	L00317
	LABEL?L	L00316	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01080	           Lcdout $fe, 2,"Sunset #",#kanal,"  OFF        "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUTD?B	_Kanal
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01081	        endif 
	LABEL?L	L00317	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01082	     endif
	LABEL?L	L00313	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01083	     if kursor = 3 then 
	CMPNE?BCL	_Kursor, 003h, L00318

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01084	        if kanal = 1 then
	CMPNE?BCL	_Kanal, 001h, L00320

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01085	           kanal1 = osvetljenje 
	MOVE?BB	_osvetljenje, _Kanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01086	           kanal2 = 0
	MOVE?CB	000h, _Kanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01087	           kanal3 = 0 
	MOVE?CB	000h, _Kanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01088	           kanal4 = 0
	MOVE?CB	000h, _Kanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01089	        endif
	LABEL?L	L00320	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01090	        if kanal = 2 then
	CMPNE?BCL	_Kanal, 002h, L00322

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01091	           kanal1 = 0 
	MOVE?CB	000h, _Kanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01092	           kanal2 = osvetljenje
	MOVE?BB	_osvetljenje, _Kanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01093	           kanal3 = 0 
	MOVE?CB	000h, _Kanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01094	           kanal4 = 0
	MOVE?CB	000h, _Kanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01095	        endif
	LABEL?L	L00322	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01096	        if kanal = 3 then
	CMPNE?BCL	_Kanal, 003h, L00324

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01097	           kanal1 = 0
	MOVE?CB	000h, _Kanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01098	           kanal2 = 0
	MOVE?CB	000h, _Kanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01099	           kanal3 = osvetljenje
	MOVE?BB	_osvetljenje, _Kanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01100	           kanal4 = 0
	MOVE?CB	000h, _Kanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01101	        endif
	LABEL?L	L00324	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01102	        if kanal = 4 then
	CMPNE?BCL	_Kanal, 004h, L00326

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01103	           kanal1 = 0
	MOVE?CB	000h, _Kanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01104	           kanal2 = 0 
	MOVE?CB	000h, _Kanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01105	           kanal3 = 0 
	MOVE?CB	000h, _Kanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01106	           kanal4 = osvetljenje
	MOVE?BB	_osvetljenje, _Kanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01107	        endif
	LABEL?L	L00326	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01108	     endif
	LABEL?L	L00318	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01109	     lcdout $fe,$c0,displej[1],dec2 sat,":",displej[2],dec2 minut," * ",displej[3],dec3 osvetljenje,"%             "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?B	_Displej + 00001h
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_sat
	LCDOUTDEC?	
	LCDOUT?C	03Ah
	LCDOUT?B	_Displej + 00002h
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_minut
	LCDOUTDEC?	
	LCDOUT?C	020h
	LCDOUT?C	02Ah
	LCDOUT?C	020h
	LCDOUT?B	_Displej + 00003h
	LCDOUTCOUNT?C	003h
	LCDOUTNUM?B	_osvetljenje
	LCDOUTDEC?	
	LCDOUT?C	025h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01110	     if novostanjelevo < starostanjelevo then
	CMPGE?TTL	_NovoStanjeLevo, _StaroStanjeLevo, L00328

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01111	            gosub EEpromMemorija
	GOSUB?L	_EEpromMemorija

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01112	            kanal1 =  StaraV1 
	MOVE?BB	_StaraV1, _Kanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01113	            kanal2 =  StaraV2 
	MOVE?BB	_StaraV2, _Kanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01114	            kanal3 =  StaraV3 
	MOVE?BB	_StaraV3, _Kanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01115	            kanal4 =  StaraV4 
	MOVE?BB	_StaraV4, _Kanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01116	            i = 0
	MOVE?CB	000h, _i

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01117	            goto menu
	GOTO?L	_menu

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01118	     ENDIF
	LABEL?L	L00328	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01119	     STAROSTANJElevo  = NOVOSTANJElevo 
	MOVE?TT	_NovoStanjeLevo, _StaroStanjeLevo

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01120	     STAROSTANJEgore  = NOVOSTANJEgore
	MOVE?TT	_NovoStanjeGore, _StaroStanjeGore

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01121	     STAROSTANJEdesno = NOVOSTANJEdesno
	MOVE?TT	_NovoStanjeDesno, _StaroStanjeDesno

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01122	     STAROSTANJEdole  = NOVOSTANJEdole
	MOVE?TT	_NovoStanjeDole, _StaroStanjeDole

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01123	goto podesavanje
	GOTO?L	_podesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01125	RTCvreme:

	LABEL?L	_RTCvreme	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01126	     bcklight = 1
	MOVE?CT	001h, _BCKLIGHT

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01127	     novostanjelevo  = tleft
	MOVE?TT	_TLEFT, _NovoStanjeLevo

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01128	     novostanjedesno = tright
	MOVE?TT	_TRIGHT, _NovoStanjeDesno

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01129	     novostanjeGore  = tup
	MOVE?TT	_TUP, _NovoStanjeGore

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01130	     novostanjedole  = tdown
	MOVE?TT	_TDOWN, _NovoStanjeDole

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01131	     if novostanjedesno < starostanjedesno then
	CMPGE?TTL	_NovoStanjeDesno, _StaroStanjeDesno, L00330

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01132	        Kursor =  Kursor + 1 
	ADD?BCB	_Kursor, 001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01133	        if kursor > 2 then kursor = 1
	CMPLE?BCL	_Kursor, 002h, L00332
	MOVE?CB	001h, _Kursor
	LABEL?L	L00332	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01134	     endif
	LABEL?L	L00330	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01135	     if novostanjelevo < starostanjelevo then
	CMPGE?TTL	_NovoStanjeLevo, _StaroStanjeLevo, L00334

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01136	            RTCMIN  = (decmin dig 1) * 16 + decmin dig 0
	DIG?BCN	_DECmin, 001h, T1
	MUL?NCN	T1, 010h, T1
	DIG?BCN	_DECmin, 000h, T2
	ADD?NNB	T1, T2, _RTCMin

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01137	            RTCHOUR = (decsat dig 1) * 16 + decsat dig 0
	DIG?BCN	_DECsat, 001h, T1
	MUL?NCN	T1, 010h, T1
	DIG?BCN	_DECsat, 000h, T2
	ADD?NNB	T1, T2, _RTCHour

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01138	            RTCSEC = 0
	MOVE?CB	000h, _RTCSec

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01139	            T1CON.0=0
	MOVE?CT	000h, _T1CON??0

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01140	            I2CWRITE SDA, SCL, RTC, SecReg, [RTCSEC,RTCMIN,RTCHOUR,RTCday,RTCdate,RTCmonTH,RTCYEAR]
	I2CDATA?T	_SDA
	I2CCLOCK?T	_SCL
	I2CWRITE?C	_RTC
	I2CWRITE?C	_SecReg
	I2CWRITE?B	_RTCSec
	I2CWRITE?B	_RTCMin
	I2CWRITE?B	_RTCHour
	I2CWRITE?B	_RTCDay
	I2CWRITE?B	_RTCDate
	I2CWRITE?B	_RTCMonth
	I2CWRITES?B	_RTCYear

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01141	            T1CON.0=1
	MOVE?CT	001h, _T1CON??0

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01142	            pause 20
	PAUSE?C	014h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01143	        i = 0
	MOVE?CB	000h, _i

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01144	        goto menu
	GOTO?L	_menu

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01145	     ENDIF 
	LABEL?L	L00334	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01146	     select case kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01147	        case 1 
	CMPNE?BCL	_Kursor, 001h, L00338

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01148	             displej[1] = ">"
	MOVE?CB	03Eh, _Displej + 00001h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01149	             displej[2] = " "
	MOVE?CB	020h, _Displej + 00002h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01151	        case 2 
	GOTO?L	L00337
	LABEL?L	L00338	
	CMPNE?BCL	_Kursor, 002h, L00339

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01152	             displej[1] = " "
	MOVE?CB	020h, _Displej + 00001h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01153	             displej[2] = ">"
	MOVE?CB	03Eh, _Displej + 00002h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01154	     end select
	LABEL?L	L00339	
	LABEL?L	L00337	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01156	     if tup = 0 then
	CMPNE?TCL	_TUP, 000h, L00340

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01157	         if kursor = 1 then
	CMPNE?BCL	_Kursor, 001h, L00342

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01158	            decsaT = decsat + 1
	ADD?BCB	_DECsat, 001h, _DECsat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01159	            if decsat > 23 then decsat = 23
	CMPLE?BCL	_DECsat, 017h, L00344
	MOVE?CB	017h, _DECsat
	LABEL?L	L00344	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01160	            pause 60
	PAUSE?C	03Ch

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01161	         endif
	LABEL?L	L00342	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01163	         if kursor = 2 then
	CMPNE?BCL	_Kursor, 002h, L00346

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01164	            decmin = decmin + 1
	ADD?BCB	_DECmin, 001h, _DECmin

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01165	            if decmin > 59 then decmin = 59
	CMPLE?BCL	_DECmin, 03Bh, L00348
	MOVE?CB	03Bh, _DECmin
	LABEL?L	L00348	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01166	            pause 60
	PAUSE?C	03Ch

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01167	         endif
	LABEL?L	L00346	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01169	     endif
	LABEL?L	L00340	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01171	     if tdown = 0 then
	CMPNE?TCL	_TDOWN, 000h, L00350

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01172	         if kursor = 1 then
	CMPNE?BCL	_Kursor, 001h, L00352

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01173	            decsaT = decsat - 1
	SUB?BCB	_DECsat, 001h, _DECsat

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01174	            if decsat < 1 or decsat > 23 then decsat = 0
	CMPLT?BCB	_DECsat, 001h, T1
	CMPGT?BCB	_DECsat, 017h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00354
	MOVE?CB	000h, _DECsat
	LABEL?L	L00354	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01175	            pause 60
	PAUSE?C	03Ch

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01176	         endif
	LABEL?L	L00352	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01178	         if kursor = 2 then
	CMPNE?BCL	_Kursor, 002h, L00356

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01179	            decmin = decmin - 1
	SUB?BCB	_DECmin, 001h, _DECmin

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01180	            if decmin < 1 or decmin > 59 then decmin = 0   
	CMPLT?BCB	_DECmin, 001h, T1
	CMPGT?BCB	_DECmin, 03Bh, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00358
	MOVE?CB	000h, _DECmin
	LABEL?L	L00358	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01181	            pause 60
	PAUSE?C	03Ch

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01182	         endif
	LABEL?L	L00356	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01183	     endif
	LABEL?L	L00350	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01184	     Lcdout $fe, 2,"RTC  Time            "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	052h
	LCDOUT?C	054h
	LCDOUT?C	043h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	054h
	LCDOUT?C	069h
	LCDOUT?C	06Dh
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01185	     lcdout $fe,$c0,displej[1],dec2 decsat,":",displej[2],dec2 decmin,"              "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?B	_Displej + 00001h
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_DECsat
	LCDOUTDEC?	
	LCDOUT?C	03Ah
	LCDOUT?B	_Displej + 00002h
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_DECmin
	LCDOUTDEC?	
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01187	     STAROSTANJElevo  = NOVOSTANJElevo 
	MOVE?TT	_NovoStanjeLevo, _StaroStanjeLevo

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01188	     STAROSTANJEgore  = NOVOSTANJEgore
	MOVE?TT	_NovoStanjeGore, _StaroStanjeGore

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01189	     STAROSTANJEdesno = NOVOSTANJEdesno
	MOVE?TT	_NovoStanjeDesno, _StaroStanjeDesno

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01190	     STAROSTANJEdole  = NOVOSTANJEdole
	MOVE?TT	_NovoStanjeDole, _StaroStanjeDole

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01191	 goto RTCVreme
	GOTO?L	_RTCvreme

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01193	OdrzavanjePrekidac:

	LABEL?L	_OdrzavanjePrekidac	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01194	     bcklight = 1
	MOVE?CT	001h, _BCKLIGHT

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01195	     novostanjeGore= tup
	MOVE?TT	_TUP, _NovoStanjeGore

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01196	     novostanjedole=tdown
	MOVE?TT	_TDOWN, _NovoStanjeDole

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01197	     if odrzavanje <> 0 then 
	CMPEQ?TCL	_ODRZAVANJE, 000h, L00360

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01198	             kanal1 =  StaraV1 
	MOVE?BB	_StaraV1, _Kanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01199	             kanal2 =  StaraV2 
	MOVE?BB	_StaraV2, _Kanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01200	             kanal3 =  StaraV3 
	MOVE?BB	_StaraV3, _Kanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01201	             kanal4 =  StaraV4
	MOVE?BB	_StaraV4, _Kanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01202	             read 3,x
	READADDRESS?C	003h
	READ?B	_x

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01203	             if x = 1 then goto main 
	CMPNE?BCL	_x, 001h, L00362
	GOTO?L	_Main
	LABEL?L	L00362	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01204	             if x = 2 then 
	CMPNE?BCL	_x, 002h, L00364

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01205	                kursor = 1 
	MOVE?CB	001h, _Kursor

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01206	                goto rucnopodesavanje
	GOTO?L	_RucnoPodesavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01207	             endif
	LABEL?L	L00364	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01208	     endif 
	LABEL?L	L00360	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01209	     if tup = 0 then
	CMPNE?TCL	_TUP, 000h, L00366

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01210	          osvetljenjeOdrzavanje = osvetljenjeOdrzavanje + 1 
	ADD?BCB	_osvetljenjeOdrzavanje, 001h, _osvetljenjeOdrzavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01211	          if osvetljenjeOdrzavanje > 100 then osvetljenjeOdrzavanje = 100
	CMPLE?BCL	_osvetljenjeOdrzavanje, 064h, L00368
	MOVE?CB	064h, _osvetljenjeOdrzavanje
	LABEL?L	L00368	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01212	          write 2,osvetljenjeOdrzavanje 
	WRITEADDRESS?C	002h
	WRITE?B	_osvetljenjeOdrzavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01213	          PAUSE 60
	PAUSE?C	03Ch

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01214	     endif  
	LABEL?L	L00366	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01215	     if tdown = 0 then
	CMPNE?TCL	_TDOWN, 000h, L00370

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01216	          osvetljenjeOdrzavanje = osvetljenjeOdrzavanje - 1 
	SUB?BCB	_osvetljenjeOdrzavanje, 001h, _osvetljenjeOdrzavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01217	          if osvetljenjeOdrzavanje < 1 or osvetljenjeOdrzavanje > 100 then osvetljenjeOdrzavanje = 0
	CMPLT?BCB	_osvetljenjeOdrzavanje, 001h, T1
	CMPGT?BCB	_osvetljenjeOdrzavanje, 064h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00372
	MOVE?CB	000h, _osvetljenjeOdrzavanje
	LABEL?L	L00372	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01218	          write 2,osvetljenjeOdrzavanje 
	WRITEADDRESS?C	002h
	WRITE?B	_osvetljenjeOdrzavanje

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01219	          PAUSE 60 
	PAUSE?C	03Ch

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01220	     endif
	LABEL?L	L00370	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01222	     Lcdout $fe, 2,"Maintenance %          "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	04Dh
	LCDOUT?C	061h
	LCDOUT?C	069h
	LCDOUT?C	06Eh
	LCDOUT?C	074h
	LCDOUT?C	065h
	LCDOUT?C	06Eh
	LCDOUT?C	061h
	LCDOUT?C	06Eh
	LCDOUT?C	063h
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	025h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01223	     lcdout $fe,$c0,"Channel #1 - ",dec3 osvetljenjeOdrzavanje," %           "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	043h
	LCDOUT?C	068h
	LCDOUT?C	061h
	LCDOUT?C	06Eh
	LCDOUT?C	06Eh
	LCDOUT?C	065h
	LCDOUT?C	06Ch
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	031h
	LCDOUT?C	020h
	LCDOUT?C	02Dh
	LCDOUT?C	020h
	LCDOUTCOUNT?C	003h
	LCDOUTNUM?B	_osvetljenjeOdrzavanje
	LCDOUTDEC?	
	LCDOUT?C	020h
	LCDOUT?C	025h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01224	     kanal1 = osvetljenjeOdrzavanje
	MOVE?BB	_osvetljenjeOdrzavanje, _Kanal1

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01225	     kanal2 = 0
	MOVE?CB	000h, _Kanal2

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01226	     kanal3 = 0
	MOVE?CB	000h, _Kanal3

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01227	     kanal4 = 0
	MOVE?CB	000h, _Kanal4

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01229	     STAROSTANJEgore=NOVOSTANJEgore
	MOVE?TT	_NovoStanjeGore, _StaroStanjeGore

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01230	     sTAROSTANJEdole=NOVOSTANJEdole
	MOVE?TT	_NovoStanjeDole, _StaroStanjeDole

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01231	goto odrzavanjePrekidac
	GOTO?L	_OdrzavanjePrekidac

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01233	 EEpromMemorija:

	LABEL?L	_EEpromMemorija	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01234	    for i = 8 to 72 step 1
	MOVE?CB	008h, _i
	LABEL?L	L00374	
	CMPGT?BCL	_i, 048h, L00375

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01235	        memorija[i] = 0
	AIN?CBB	000h, _Memorija, _i

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01236	        read i,memorija[i]   
	READADDRESS?B	_i
	READ?N	T1
	AIN?BBB	T1, _Memorija, _i

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01237	    next i
	NEXT?BCL	_i, 001h, L00374
	LABEL?L	L00375	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01238	return
	RETURN?	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01240	ProveriPrviPut:

	LABEL?L	_ProveriPrviPut	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01241	    read 1,eprom
	READADDRESS?C	001h
	READ?B	_EPROM

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01242	    if eprom = 255 then 
	CMPNE?BCL	_EPROM, 0FFh, L00376

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01243	       for i = 8 to 72 step 4
	MOVE?CB	008h, _i
	LABEL?L	L00378	
	CMPGT?BCL	_i, 048h, L00379

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01244	            write i,0
	WRITEADDRESS?B	_i
	WRITE?C	000h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01245	            write i + 1,0
	ADD?BCN	_i, 001h, T1
	WRITEADDRESS?N	T1
	WRITE?C	000h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01246	            write i + 2,0
	ADD?BCN	_i, 002h, T1
	WRITEADDRESS?N	T1
	WRITE?C	000h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01247	            write i + 3,0
	ADD?BCN	_i, 003h, T1
	WRITEADDRESS?N	T1
	WRITE?C	000h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01248	       next i 
	NEXT?BCL	_i, 004h, L00378
	LABEL?L	L00379	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01249	       write 1,1  'umemorisi prvo pokretanje
	WRITEADDRESS?C	001h
	WRITE?C	001h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01250	       write 2,10 'odrzavanje
	WRITEADDRESS?C	002h
	WRITE?C	00Ah

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01251	       write 3,1  'automatsko ili rucno
	WRITEADDRESS?C	003h
	WRITE?C	001h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01252	       write 80,0
	WRITEADDRESS?C	050h
	WRITE?C	000h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01253	       write 81,0
	WRITEADDRESS?C	051h
	WRITE?C	000h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01254	       write 82,0
	WRITEADDRESS?C	052h
	WRITE?C	000h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01255	       write 83,0
	WRITEADDRESS?C	053h
	WRITE?C	000h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01257	       write 84,0
	WRITEADDRESS?C	054h
	WRITE?C	000h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01258	       write 85,0
	WRITEADDRESS?C	055h
	WRITE?C	000h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01259	       write 86,0
	WRITEADDRESS?C	056h
	WRITE?C	000h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01260	       write 87,0
	WRITEADDRESS?C	057h
	WRITE?C	000h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01262	       T1CON.0=0
	MOVE?CT	000h, _T1CON??0

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01263	       I2CWrite sda, scl, $D0, $00, [$00, $00, $00, $00, $00, $00, $00, $0]
	I2CDATA?T	_SDA
	I2CCLOCK?T	_SCL
	I2CWRITE?C	0D0h
	I2CWRITE?C	000h
	I2CWRITE?C	000h
	I2CWRITE?C	000h
	I2CWRITE?C	000h
	I2CWRITE?C	000h
	I2CWRITE?C	000h
	I2CWRITE?C	000h
	I2CWRITE?C	000h
	I2CWRITES?C	000h

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01264	       T1CON.0=1
	MOVE?CT	001h, _T1CON??0

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01265	    endif
	LABEL?L	L00376	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01266	return
	RETURN?	

; D:\DAMIR\04 - PROJEKTI\DANER LED CONTROL\V2\PBP\DANER V2.PBP	01268	end
	END?	

	END
