
; PICBASIC PRO(TM) Compiler 3.0.4.1, (c) 1998, 2011 microEngineering Labs, Inc. All Rights Reserved. 
MPASMWIN_USED			EQU	1

#define	PBP_PATH	C:\PBP3\




        NOLIST
    ifdef PM_USED
        LIST
        "Error: PM does not support this device.  Use MPASM."
        NOLIST
    else
        LIST
        LIST p = 18F25K22, r = dec, w = -311, w = -230, f = inhx32
        INCLUDE "P18F25K22.INC"	; MPASM  Header
        NOLIST
    endif
        LIST
EEPROM_START	EQU	0F00000h
BLOCK_SIZE	EQU	64


PBP_HARDWAREDEF  macro
  endm


    CONFIG FOSC = INTIO67
    CONFIG PLLCFG = OFF
    CONFIG PRICLKEN = ON
    CONFIG FCMEN = OFF
    CONFIG IESO = OFF
    CONFIG PWRTEN = ON
    CONFIG BOREN = OFF
    CONFIG BORV = 285
    CONFIG WDTEN = OFF
    CONFIG WDTPS = 32768
    CONFIG CCP2MX = PORTB3
    CONFIG PBADEN = OFF
    CONFIG CCP3MX = PORTC6
    CONFIG HFOFST = ON
    CONFIG T3CMX = PORTC0
    CONFIG P2BMX = PORTC0
    CONFIG MCLRE = INTMCLR
    CONFIG STVREN = ON
    CONFIG LVP = OFF
    CONFIG XINST = OFF
    CONFIG CP0 = ON
    CONFIG CP1 = ON
    CONFIG CP2 = ON
    CONFIG CP3 = ON
    CONFIG CPB = ON
    CONFIG CPD = OFF
    CONFIG WRT0 = OFF
    CONFIG WRT1 = OFF
    CONFIG WRT2 = OFF
    CONFIG WRT3 = OFF
    CONFIG WRTC = OFF
    CONFIG WRTB = OFF
    CONFIG WRTD = OFF
    CONFIG EBTR0 = OFF
    CONFIG EBTR1 = OFF
    CONFIG EBTR2 = OFF
    CONFIG EBTR3 = OFF
    CONFIG EBTRB = OFF



; Define statements.
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00299	DEFINE  ADC_CHANNELS  1
#define		ADC_CHANNELS		  1
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00001	define OSC 16
#define		OSC		 16
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00045	DEFINE LCD_DREG PORTA 
#define		LCD_DREG		 PORTA 
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00046	DEFINE LCD_DBIT 0
#define		LCD_DBIT		 0
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00047	DEFINE LCD_RSREG PORTA
#define		LCD_RSREG		 PORTA
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00048	DEFINE LCD_RSBIT 5 
#define		LCD_RSBIT		 5 
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00049	DEFINE LCD_EREG PORTA
#define		LCD_EREG		 PORTA
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00050	DEFINE LCD_EBIT 4 
#define		LCD_EBIT		 4 
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00051	DEFINE LCD_BITS 4 
#define		LCD_BITS		 4 
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00052	DEFINE LCD_LINE 2
#define		LCD_LINE		 2
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00053	DEFINE I2C_SCLOUT 1
#define		I2C_SCLOUT		 1
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00054	DEFINE I2C_SLOW 1
#define		I2C_SLOW		 1
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00055	define NO_CLRWDT 1
#define		NO_CLRWDT		 1
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00077	DEFINE I2C_SDA PORTC,0 
#define		I2C_SDA		 PORTC,0 
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00078	DEFINE I2C_SCL PORTC,1 
#define		I2C_SCL		 PORTC,1 

RAM_START       		EQU	00000h
RAM_END         		EQU	005FFh
RAM_BANKS       		EQU	00006h
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
; C:\PBP3\PBPPI18L.RAM     	00023	A0000C	RM1     VAR     BYTE BANKA SYSTEM       ' Pin 1 Mask
RM1             		EQU	RAM_START + 00Ch
; C:\PBP3\PBPPI18L.RAM     	00026	A0000D	RM2     VAR     BYTE BANKA SYSTEM       ' Pin 2 Mask
RM2             		EQU	RAM_START + 00Dh
; C:\PBP3\PBPPI18L.RAM     	00021	A0000E	RR1     VAR     BYTE BANKA SYSTEM       ' Pin 1 Register
RR1             		EQU	RAM_START + 00Eh
; C:\PBP3\PBPPI18L.RAM     	00024	A0000F	RR2     VAR     BYTE BANKA SYSTEM       ' Pin 2 Register
RR2             		EQU	RAM_START + 00Fh
; C:\PBP3\PBPPI18L.RAM     	00022	A00010	RS1     VAR     BYTE BANKA SYSTEM       ' Pin 1 Bank
RS1             		EQU	RAM_START + 010h
; C:\PBP3\PBPPI18L.RAM     	00025	A00011	RS2     VAR     BYTE BANKA SYSTEM       ' Pin 2 Bank
RS2             		EQU	RAM_START + 011h
; C:\PBP3\PBPPI18L.RAM     	00012	A00012	R0      VAR     LONG BANKA SYSTEM       ' System Register
R0              		EQU	RAM_START + 012h
; C:\PBP3\PBPPI18L.RAM     	00013	A00016	R1      VAR     LONG BANKA SYSTEM       ' System Register
R1              		EQU	RAM_START + 016h
; C:\PBP3\PBPPI18L.RAM     	00014	A0001A	R2      VAR     LONG BANKA SYSTEM       ' System Register
R2              		EQU	RAM_START + 01Ah
; C:\PBP3\PBPPI18L.RAM     	00015	A0001E	R3      VAR     LONG BANKA SYSTEM       ' System Register
R3              		EQU	RAM_START + 01Eh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00001	A00022	define OSC 16
T1              		EQU	RAM_START + 022h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00001	A00026	define OSC 16
T2              		EQU	RAM_START + 026h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00001	A0002A	define OSC 16
T3              		EQU	RAM_START + 02Ah
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00001	A0002E	define OSC 16
T4              		EQU	RAM_START + 02Eh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00001	A00032	define OSC 16
T5              		EQU	RAM_START + 032h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00001	A00036	define OSC 16
T6              		EQU	RAM_START + 036h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00001	A0003A	define OSC 16
PB01            		EQU	RAM_START + 03Ah
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00001	A0003B	define OSC 16
PB02            		EQU	RAM_START + 03Bh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00001	A0003C	define OSC 16
PB03            		EQU	RAM_START + 03Ch
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00001	A0003D	define OSC 16
PB04            		EQU	RAM_START + 03Dh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00202	A0003E	Akvarijum               var word
_Akvarijum       		EQU	RAM_START + 03Eh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00174	A00040	BezTastera              var word
_BezTastera      		EQU	RAM_START + 040h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00167	A00042	Frequency               var word
_Frequency       		EQU	RAM_START + 042h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00047	A00044	HP_Duty      VAR WORD
_HP_Duty         		EQU	RAM_START + 044h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00046	A00046	HP_Freq      VAR Word
_HP_Freq         		EQU	RAM_START + 046h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00048	A00048	HP_Temp      VAR WORD
_HP_Temp         		EQU	RAM_START + 048h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00179	A0004A	KMinuta                 var word
_KMinuta         		EQU	RAM_START + 04Ah
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00178	A0004C	PMinuta                 var word
_PMinuta         		EQU	RAM_START + 04Ch
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00188	A0004E	PocetnaAdresaJezika     var word
_PocetnaAdresaJezika		EQU	RAM_START + 04Eh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00209	A00050	smece var word
_smece           		EQU	RAM_START + 050h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00201	A00052	Soba                    var word
_Soba            		EQU	RAM_START + 052h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00151	A00054	Temp                    VAR word
_Temp            		EQU	RAM_START + 054h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00155	A00056	TMinuta                 var word
_TMinuta         		EQU	RAM_START + 056h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00193	A00058	BrojKanala              var byte
_BrojKanala      		EQU	RAM_START + 058h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00148	A00059	Days                    var byte
_Days            		EQU	RAM_START + 059h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00132	A0005A	DecMin                  var byte
_DecMin          		EQU	RAM_START + 05Ah
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00139	A0005B	DecMin1                 var byte 
_DecMin1         		EQU	RAM_START + 05Bh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00204	A0005C	DecMinutGlavni          var byte
_DecMinutGlavni  		EQU	RAM_START + 05Ch
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00206	A0005D	DecMinutPrikaz          var byte
_DecMinutPrikaz  		EQU	RAM_START + 05Dh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00131	A0005E	DecSat                  var byte
_DecSat          		EQU	RAM_START + 05Eh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00138	A0005F	DecSat1                 var byte 
_DecSat1         		EQU	RAM_START + 05Fh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00203	A00060	DecSatglavni            var byte
_DecSatglavni    		EQU	RAM_START + 060h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00205	A00061	DecSatPrikaz            var byte
_DecSatPrikaz    		EQU	RAM_START + 061h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00140	A00062	DecSec                  var byte
_DecSec          		EQU	RAM_START + 062h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00208	A00063	DesniBrojac             var byte
_DesniBrojac     		EQU	RAM_START + 063h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00176	A00064	DveTacke                var byte
_DveTacke        		EQU	RAM_START + 064h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00146	A00065	Hours                   var byte
_Hours           		EQU	RAM_START + 065h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00045	A00066	HP_Channel   VAR BYTE
_HP_Channel      		EQU	RAM_START + 066h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00039	A00067	HP_Flags  VAR BYTE
_HP_Flags        		EQU	RAM_START + 067h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00049	A00068	HP_PR2       VAR BYTE
_HP_PR2          		EQU	RAM_START + 068h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00129	A00069	i                       var byte 
_i               		EQU	RAM_START + 069h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00190	A0006A	JacinaOsvetljenja1      var byte
_JacinaOsvetljenja1		EQU	RAM_START + 06Ah
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00191	A0006B	JacinaOsvetljenja2      var byte
_JacinaOsvetljenja2		EQU	RAM_START + 06Bh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00169	A0006C	Kanal1                  var byte
_Kanal1          		EQU	RAM_START + 06Ch
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00170	A0006D	Kanal2                  var byte
_Kanal2          		EQU	RAM_START + 06Dh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00171	A0006E	Kanal3                  var byte
_Kanal3          		EQU	RAM_START + 06Eh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00172	A0006F	Kanal4                  var byte
_Kanal4          		EQU	RAM_START + 06Fh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00207	A00070	LeviBrojac              var byte
_LeviBrojac      		EQU	RAM_START + 070h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00212	A00071	MinDisplej var byte
_MinDisplej      		EQU	RAM_START + 071h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00145	A00072	Minutes                 var byte
_Minutes         		EQU	RAM_START + 072h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00183	A00073	MinutKraj               var byte
_MinutKraj       		EQU	RAM_START + 073h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00182	A00074	MinutPocetak            var byte
_MinutPocetak    		EQU	RAM_START + 074h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00150	A00075	Month                   var byte
_Month           		EQU	RAM_START + 075h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00181	A00076	NoviMinut               var byte
_NoviMinut       		EQU	RAM_START + 076h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00152	A00077	NovostanjeRTC           var byte
_NovostanjeRTC   		EQU	RAM_START + 077h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00147	A00078	OdrzavanjeJacina        var byte 
_OdrzavanjeJacina		EQU	RAM_START + 078h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00130	A00079	OsvetljenjeOdrzavanje   var byte
_OsvetljenjeOdrzavanje		EQU	RAM_START + 079h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00187	A0007A	OsvKraj                 var byte
_OsvKraj         		EQU	RAM_START + 07Ah
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00186	A0007B	OsvPocetak              var byte
_OsvPocetak      		EQU	RAM_START + 07Bh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00137	A0007C	Pozicija                var byte 
_Pozicija        		EQU	RAM_START + 07Ch
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00210	A0007D	pregrejan var byte
_pregrejan       		EQU	RAM_START + 07Dh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00133	A0007E	Rkanal1                 var byte  
_Rkanal1         		EQU	RAM_START + 07Eh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00134	A0007F	Rkanal2                 var byte  
_Rkanal2         		EQU	RAM_START + 07Fh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00135	A00080	Rkanal3                 var byte  
_Rkanal3         		EQU	RAM_START + 080h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00136	A00081	Rkanal4                 var byte 
_Rkanal4         		EQU	RAM_START + 081h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00192	A00082	RtcCnt                  var byte
_RtcCnt          		EQU	RAM_START + 082h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00213	A00083	SatDisplej var byte
_SatDisplej      		EQU	RAM_START + 083h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00185	A00084	SatKraj                 var byte
_SatKraj         		EQU	RAM_START + 084h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00184	A00085	SatPocetak              var byte
_SatPocetak      		EQU	RAM_START + 085h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00144	A00086	Seconds                 var byte
_Seconds         		EQU	RAM_START + 086h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00153	A00087	StarostanjeRTC          var byte
_StarostanjeRTC  		EQU	RAM_START + 087h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00143	A00088	Temperatura             var byte
_Temperatura     		EQU	RAM_START + 088h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00197	A00089	TemperaturaIzEproma     var byte
_TemperaturaIzEproma		EQU	RAM_START + 089h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00128	A0008A	x                       var byte
_x               		EQU	RAM_START + 08Ah
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00189	A0008B	y                       var byte
_y               		EQU	RAM_START + 08Bh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00149	A0008C	Years                   var byte
_Years           		EQU	RAM_START + 08Ch
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00157	A0008D	DutyCycleL1             var long
_DutyCycleL1     		EQU	RAM_START + 08Dh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00158	A00091	DutyCycleL2             var long
_DutyCycleL2     		EQU	RAM_START + 091h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00159	A00095	DutyCycleL3             var long
_DutyCycleL3     		EQU	RAM_START + 095h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00160	A00099	DutyCycleL4             var long
_DutyCycleL4     		EQU	RAM_START + 099h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00162	A0009D	STKanal1                var long
_STKanal1        		EQU	RAM_START + 09Dh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00163	A000A1	STKanal2                var long
_STKanal2        		EQU	RAM_START + 0A1h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00164	A000A5	STKanal3                var long
_STKanal3        		EQU	RAM_START + 0A5h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00165	A000A9	STKanal4                var long
_STKanal4        		EQU	RAM_START + 0A9h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00106	A000AD	displej                 var byte[7]
_displej         		EQU	RAM_START + 0ADh
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00108	A000B4	RelejiMemorija          var byte[17] 
_RelejiMemorija  		EQU	RAM_START + 0B4h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00107	A000C5	Memorija                var byte[75]
_Memorija        		EQU	RAM_START + 0C5h
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00300	PORTL   VAR     PORTB
_PORTL           		EQU	 PORTB
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00301	PORTH   VAR     PORTC
_PORTH           		EQU	 PORTC
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00302	TRISL   VAR     TRISB
_TRISL           		EQU	 TRISB
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00303	TRISH   VAR     TRISC
_TRISH           		EQU	 TRISC
_HP_Temp??LOWBYTE		EQU	_HP_Temp
_HP_Temp??HIGHBYTE		EQU	_HP_Temp + 001h
#define _PS0             	_HP_Flags??0
#define _PS1             	_HP_Flags??1
#define _PSchanged       	_HP_Flags??2
#define _PR2changed      	_HP_Flags??3
#define _TMR2IF          	_PIR1??1
#define _TMR2ON          	_T2CON??2
#define _T2CKPS1         	_T2CON??1
#define _T2CKPS0         	_T2CON??0
#define _BCKLIGHT        	_PORTC??3
#define _SDA             	_PORTC??0
#define _SCL             	_PORTC??1
#define _RELEJ1          	_PORTC??4
#define _RELEJ2          	_PORTC??5
#define _TASTLEVO        	_PORTB??1
#define _TASTDESNO       	_PORTB??4
#define _TASTGORE        	_PORTB??6
#define _TASTDOLE        	_PORTB??5
#define _TASTOK          	_PORTB??2
#define _TASTODRZAVANJE  	_PORTB??7
#define _NovoStanjeDesno 	 PB01, 007h
#define _NovoStanjeLevo  	 PB02, 002h
#define _NovoStanjeGore  	 PB02, 001h
#define _NovoStanjeDole  	 PB02, 000h
#define _NovoStanjeOk    	 PB02, 004h
#define _NovoStanjeOdrzavanje	 PB02, 003h
#define _StaroStanjeDesno	 PB03, 003h
#define _StaroStanjeLevo 	 PB03, 006h
#define _StaroStanjeGore 	 PB03, 005h
#define _StaroStanjeDole 	 PB03, 004h
#define _StaroStanjeOk   	 PB04, 000h
#define _StaroStanjeOdrzavanje	 PB03, 007h
#define _gore            	 PB01, 004h
#define _dole            	 PB01, 003h
#define _levo            	 PB01, 005h
#define _desno           	 PB01, 002h
#define _ok              	 PB02, 007h
#define _odrzavanje      	 PB02, 005h
#define _Pritisnuto      	 PB03, 001h
#define _Room            	 PB03, 002h
#define _ModDanNoc       	 PB01, 006h
#define _Prekidac        	 PB03, 000h
#define _VremePrekidaca  	 PB04, 001h
#define _AutomatskoAkvarijum	 PB01, 000h
#define _AutomatskoSoba  	 PB01, 001h
#define _OdrzavanjeMod   	 PB02, 006h
#define _HP_Flags??0     	_HP_Flags, 000h
#define _HP_Flags??1     	_HP_Flags, 001h
#define _HP_Flags??2     	_HP_Flags, 002h
#define _HP_Flags??3     	_HP_Flags, 003h
#define _PIR1??1         	 PIR1, 001h
#define _T2CON??2        	 T2CON, 002h
#define _T2CON??1        	 T2CON, 001h
#define _T2CON??0        	 T2CON, 000h
#define _INTCON2??7      	 INTCON2, 007h
#define _PORTC??3        	 PORTC, 003h
#define _PORTC??0        	 PORTC, 000h
#define _PORTC??1        	 PORTC, 001h
#define _PORTC??4        	 PORTC, 004h
#define _PORTC??5        	 PORTC, 005h
#define _PORTB??1        	 PORTB, 001h
#define _PORTB??4        	 PORTB, 004h
#define _PORTB??6        	 PORTB, 006h
#define _PORTB??5        	 PORTB, 005h
#define _PORTB??2        	 PORTB, 002h
#define _PORTB??7        	 PORTB, 007h
#define _PORTC??6        	 PORTC, 006h
#define _PORTB??3        	 PORTB, 003h
#define _PORTB??0        	 PORTB, 000h
#define _PORTC??2        	 PORTC, 002h
#define _Minutes??7      	_Minutes, 007h
#define _Hours??7        	_Hours, 007h
#define _Hours??6        	_Hours, 006h

; Constants.
_RTC             		EQU	000A2h
_EPROM           		EQU	000A0h
_LM75            		EQU	00090h
_Strelica        		EQU	0007Eh
_PauzaZaTaster   		EQU	0006Eh

; EEPROM data.


	INCLUDE	"DANER_V3_FULL.MAC"
	INCLUDE	"C:\PBP3\PBPPI18L.LIB"


; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00036	#HEADER
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00062	#CONFIG
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00282	BANKA   $0000, $005F
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00283	BANK0   $0060, $00FF
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00284	BANK1   $0100, $01FF
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00285	BANK2   $0200, $02FF
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00286	BANK3   $0300, $03FF
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00287	BANK4   $0400, $04FF
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00288	BANK5   $0500, $05FF
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00290	'EEPROM  $F00000, $F000FF
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00292	    #MSG "LONG Variables enabled (PBPL used)"
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00293	    LIBRARY  "PBPPI18L"

; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00294	    INCLUDE "PBPPI18L.RAM"
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00297	    INCLUDE "PBPPIC18.RAM"
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00298	#ENDIF
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00299	DEFINE  ADC_CHANNELS  1
; C:\PBP3\DEVICES\PIC18F25K22.PBPINC	00307	#HEADER
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00001	define OSC 16

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00002	include "HPWM10_L.pbp"

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00018	disable debug
	DDISABLE?	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00019	asm

	ASM?

  if (R1 - R0 == 4)
  else
    messg You must use PBPL
  endif
;____[ HPWM10: 10-bit HPWM, with up to 5 CCP modules ]_____________

HPWM10 macro  Channel, Duty, Freq   
    if Channel < 6 
      MOVE?CB Channel, _HP_Channel      ; Save channel from constant
    else
      MOVE?BB Channel, _HP_Channel      ; Save channel from BYTE var
    endif
    MOVE?WW   Duty, _HP_Duty
    MOVE?WW   Freq, _HP_Freq
    L?CALL    _HPWM10_UPD    
    endm


	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00056	goto OverHPWM10
	GOTO?L	_OverHPWM10

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00058	@PS = ((OSC * 1000000) / 1024)

	ASM?
PS = ((OSC * 1000000) / 1024)

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00059	@PR2const = ((OSC * 1000000) / 4)  ;<----Added

	ASM?
PR2const = ((OSC * 1000000) / 4)  ;<----Added

	ENDASM?

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00060	PS  CON EXT                        ;<----Added
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00061	PR2const CON EXT

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00062	HPWM10_UPD:  

	LABEL?L	_HPWM10_UPD	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00064	    HP_Temp = PS / HP_Freq
	DIV?CWW	PS, _HP_Freq, _HP_Temp

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00065	    if HP_Temp >= 4 then
	CMPLT?WCL	_HP_Temp, 004h, L00003

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00066	        if PS1 <> 1 then PSchanged = 1 
	CMPEQ?TCL	_PS1, 001h, L00005
	MOVE?CT	001h, _PSchanged
	LABEL?L	L00005	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00067	        PS1 = 1
	MOVE?CT	001h, _PS1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00068	        if PS0 <> 1 then PSchanged = 1 
	CMPEQ?TCL	_PS0, 001h, L00007
	MOVE?CT	001h, _PSchanged
	LABEL?L	L00007	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00069	        PS0 = 1
	MOVE?CT	001h, _PS0

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00070	    else
	GOTO?L	L00004
	LABEL?L	L00003	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00071	        if PS1 <> 0 then PSchanged = 1 
	CMPEQ?TCL	_PS1, 000h, L00009
	MOVE?CT	001h, _PSchanged
	LABEL?L	L00009	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00072	        PS1 = 0
	MOVE?CT	000h, _PS1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00073	        if HP_Temp > 0 then 
	CMPLE?WCL	_HP_Temp, 000h, L00011

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00074	            if PS0 <> 1 then PSchanged = 1 
	CMPEQ?TCL	_PS0, 001h, L00013
	MOVE?CT	001h, _PSchanged
	LABEL?L	L00013	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00075	            PS0 = 1
	MOVE?CT	001h, _PS0

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00076	        else
	GOTO?L	L00012
	LABEL?L	L00011	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00077	            if PS0 <> 0 then PSchanged = 1 
	CMPEQ?TCL	_PS0, 000h, L00015
	MOVE?CT	001h, _PSchanged
	LABEL?L	L00015	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00078	            PS0 = 0
	MOVE?CT	000h, _PS0

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00079	        endif
	LABEL?L	L00012	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00080	    endif
	LABEL?L	L00004	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00091	    HP_Temp =PR2const/HP_Freq 'DIV32 HP_Freq <-------------------changed
	DIV?CWW	PR2const, _HP_Freq, _HP_Temp

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00093	    if PS0 then HP_Temp = HP_Temp >> 2  ; /4
	CMPF?TL	_PS0, L00017
	SHIFTR?WCW	_HP_Temp, 002h, _HP_Temp
	LABEL?L	L00017	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00094	    if PS1 then HP_Temp = HP_Temp >> 2  ; /4 or 16
	CMPF?TL	_PS1, L00019
	SHIFTR?WCW	_HP_Temp, 002h, _HP_Temp
	LABEL?L	L00019	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00095	    HP_PR2 = HP_Temp.lowbyte - 1
	SUB?BCB	_HP_Temp??LOWBYTE, 001h, _HP_PR2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00096	    if HP_PR2 <> PR2 then PR2changed = 1
	CMPEQ?BBL	_HP_PR2, PR2, L00021
	MOVE?CT	001h, _PR2changed
	LABEL?L	L00021	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00100	    HP_Temp = (HP_PR2 + 1) << 2  ; * 4
	ADD?BCN	_HP_PR2, 001h, T1
	SHIFTL?NCW	T1, 002h, _HP_Temp

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00102	    HP_Duty = (HP_Temp * HP_Duty)/1023 'DIV32 1023 <-------------------changed
	MUL?WWN	_HP_Temp, _HP_Duty, T1
	DIV?NCW	T1, 0000003FFh, _HP_Duty

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00107	    Branch HP_Channel,[GetCH_Done,GetCH_1,GetCH_2,GetCH_3,GetCH_4,GetCH_5]
	BRANCH?BCL	_HP_Channel, 006h, L00001
	BRGOTO?L	_GetCH_Done
	BRGOTO?L	_GetCH_1
	BRGOTO?L	_GetCH_2
	BRGOTO?L	_GetCH_3
	BRGOTO?L	_GetCH_4
	BRGOTO?L	_GetCH_5

	LABEL?L	L00001	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00109	    GetCH_1:

	LABEL?L	_GetCH_1	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00110	            ASM

	ASM?

              ifdef CCP1CON  
                MOVE?BB  CCPR1L,  _HP_Temp
                MOVE?BB  CCP1CON, _HP_Temp + 1
                goto _GetCH_Done        
              endif  
            

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00117	    GetCH_2:  

	LABEL?L	_GetCH_2	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00118	            ASM

	ASM?

              ifdef CCP2CON  
                MOVE?BB  CCPR2L,  _HP_Temp
                MOVE?BB  CCP2CON, _HP_Temp + 1
                goto _GetCH_Done        
              endif  
            

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00125	    GetCH_3:  

	LABEL?L	_GetCH_3	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00126	            ASM

	ASM?

              ifdef CCP3CON  
                MOVE?BB  CCPR3L,  _HP_Temp
                MOVE?BB  CCP3CON, _HP_Temp + 1
                goto _GetCH_Done        
              endif  
            

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00133	    GetCH_4:  

	LABEL?L	_GetCH_4	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00134	            ASM

	ASM?

              ifdef CCP4CON  
                MOVE?BB  CCPR4L,  _HP_Temp
                MOVE?BB  CCP4CON, _HP_Temp + 1
                goto _GetCH_Done        
              endif  
            

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00141	    GetCH_5:  

	LABEL?L	_GetCH_5	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00142	            ASM

	ASM?

              ifdef CCP5CON  
                MOVE?BB  CCPR5L,  _HP_Temp
                MOVE?BB  CCP5CON, _HP_Temp + 1
              endif  
            

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00149	GetCH_Done:

	LABEL?L	_GetCH_Done	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00150	    HP_Temp = (HP_Temp.lowbyte << 2) | ((HP_Temp.highbyte & %110000) >> 4)
	SHIFTL?BCN	_HP_Temp??LOWBYTE, 002h, T1
	AND?BCN	_HP_Temp??HIGHBYTE, 030h, T2
	SHIFTR?NCN	T2, 004h, T2
	OR?NNW	T1, T2, _HP_Temp

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00154	    if HP_Temp <> HP_Duty then  ; if DutyCycle Changed
	CMPEQ?WWL	_HP_Temp, _HP_Duty, L00023

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00156	        HP_Temp = HP_Duty >> 2        ; for CCP?CON
	SHIFTR?WCW	_HP_Duty, 002h, _HP_Temp

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00157	        HP_Temp.highbyte =  %1100 | ((HP_Duty & %11) << 4)
	AND?WCN	_HP_Duty, 003h, T1
	SHIFTL?NCN	T1, 004h, T1
	OR?CNB	00Ch, T1, _HP_Temp??HIGHBYTE

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00159	        if TMR2ON then              ; if Timer2 is running
	CMPF?TL	_TMR2ON, L00025

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00160	            TMR2IF = 0
	MOVE?CT	000h, _TMR2IF

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00161	            while TMR2IF = 0 : wend ; let current cycle complete
	LABEL?L	L00027	
	CMPNE?TCL	_TMR2IF, 000h, L00028

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00161	            while TMR2IF = 0 : wend ; let current cycle complete
	GOTO?L	L00027
	LABEL?L	L00028	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00162	        endif
	LABEL?L	L00025	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00164	        Branch HP_Channel,[PutCH_Done,PutCH_1,PutCH_2,PutCH_3,PutCH_4,PutCH_5]
	BRANCH?BCL	_HP_Channel, 006h, L00002
	BRGOTO?L	_PutCH_Done
	BRGOTO?L	_PutCH_1
	BRGOTO?L	_PutCH_2
	BRGOTO?L	_PutCH_3
	BRGOTO?L	_PutCH_4
	BRGOTO?L	_PutCH_5

	LABEL?L	L00002	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00166	        PutCH_1:  

	LABEL?L	_PutCH_1	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00167	                ASM

	ASM?

                  ifdef CCP1CON
                    MOVE?BB  _HP_Temp, CCPR1L
                    MOVE?BB  _HP_Temp + 1, CCP1CON
                    goto _PutCH_Done
                  endif  
                

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00174	        PutCH_2:  

	LABEL?L	_PutCH_2	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00175	                ASM

	ASM?

                  ifdef CCP2CON  
                    MOVE?BB  _HP_Temp, CCPR2L
                    MOVE?BB  _HP_Temp + 1, CCP2CON
                    goto _PutCH_Done
                  endif  
                

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00182	        PutCH_3:  

	LABEL?L	_PutCH_3	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00183	                ASM

	ASM?

                  ifdef CCP3CON  
                    MOVE?BB  _HP_Temp, CCPR3L
                    MOVE?BB  _HP_Temp + 1, CCP3CON
                    goto _PutCH_Done
                  endif  
                

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00190	        PutCH_4:  

	LABEL?L	_PutCH_4	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00191	                ASM

	ASM?

                  ifdef CCP4CON  
                    MOVE?BB  _HP_Temp, CCPR4L
                    MOVE?BB  _HP_Temp + 1, CCP4CON
                    goto _PutCH_Done
                  endif  
                

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00198	        PutCH_5:

	LABEL?L	_PutCH_5	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00199	                ASM

	ASM?

                  ifdef CCP5CON  
                    MOVE?BB  _HP_Temp, CCPR5L
                    MOVE?BB  _HP_Temp + 1, CCP5CON
                  endif  
                

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00206	    PutCH_Done:

	LABEL?L	_PutCH_Done	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00207	    endif
	LABEL?L	L00023	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00211	    if TMR2ON = 0 then TMR2ON = 1     ; Start Timer2 if not already running
	CMPNE?TCL	_TMR2ON, 000h, L00029
	MOVE?CT	001h, _TMR2ON
	LABEL?L	L00029	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00213	    if (PSchanged | PR2changed) = 1 then
	OR?TTN	_PSchanged, _PR2changed, T1
	CMPNE?NCL	T1, 001h, L00031

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00214	        TMR2IF = 0
	MOVE?CT	000h, _TMR2IF

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00215	        while !TMR2IF : wend          ; let current cycle complete
	LABEL?L	L00033	
	CMPT?TL	_TMR2IF, L00034

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00215	        while !TMR2IF : wend          ; let current cycle complete
	GOTO?L	L00033
	LABEL?L	L00034	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00217	        if PSchanged then             ; set T2 prescaler
	CMPF?TL	_PSchanged, L00035

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00218	            PSchanged = 0
	MOVE?CT	000h, _PSchanged

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00219	            T2CKPS0 = PS0
	MOVE?TT	_PS0, _T2CKPS0

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00220	            T2CKPS1 = PS1
	MOVE?TT	_PS1, _T2CKPS1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00221	        endif
	LABEL?L	L00035	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00223	        if PR2changed then            ; set PR2
	CMPF?TL	_PR2changed, L00037

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00224	            PR2changed = 0
	MOVE?CT	000h, _PR2changed

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00225	            PR2 = HP_PR2
	MOVE?BB	_HP_PR2, PR2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00226	        endif
	LABEL?L	L00037	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00227	    endif
	LABEL?L	L00031	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00228	return
	RETURN?	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\HPWM10_L.PBP	00231	OverHPWM10:

	LABEL?L	_OverHPWM10	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00003	include "ALLDIGITAL.pbp"

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\ALLDIGITAL.PBP	00026	ASM

	ASM?

;----[Module defaults]------------------------------------------------------
  #define SHOWDIGITALDEFAULT  0
  ifdef SHOWDIGITAL
SHOWDIGITALRESULT = SHOWDIGITAL
  else
SHOWDIGITALRESULT = SHOWDIGITALDEFAULT
  endif
  ifndef PM_USED
    ifndef ADLISTALL
      nolist
    endif
  endif
  
;----[Attempt to Set a registers Value]-------------------------------------
SetReg  macro RegOut, Cin
    ifndef PM_USED
      ifndef ADLISTALL
        nolist
      endif
    endif
    
    ifdef RegOut
      ifndef PM_USED
        list
      endif
      MOVE?CB  Cin, RegOut
      if (SHOWDIGITALRESULT == 1)
        ifndef PM_USED
          messg RegOut = Cin
        endif
      endif
      ifndef PM_USED
        ifndef ADLISTALL
          nolist
        endif
      endif
    endif
  endm
  
;----[A/D Converter registers]----------------------------------------------
  ifdef ANSEL0
    SetReg  ANSEL0, 0             ; if chip has ANSEL0, clear it
  endif
  ifdef ANSEL1
    SetReg  ANSEL1, 0             ; if chip has ANSEL1, clear it
  endif
  ifdef ANSELA
    SetReg  ANSELA, 0             ; if chip has ANSELA, clear it
  endif
  ifdef ANSELB
    SetReg  ANSELB, 0             ; if chip has ANSELB, clear it
  endif
  ifdef ANSELD
    SetReg  ANSELD, 0             ; if chip has ANSELD, clear it
  endif
  ifdef ANSELE
    SetReg  ANSELE, 0             ; if chip has ANSELE, clear it
  endif
  ifdef ANSEL
    SetReg  ANSEL, 0              ; if chip has ANSEL(s), clear them
    SetReg  ANSELH, 0
  else
    ifdef ANCON0                  ; Several chips use ANCON0 and ANCON1
      ifdef ADSHR                 ; if chip has shared address SFR's
        MOVE?CT  1, WDTCON,ADSHR  ;   switch to shared address registers
        if (SHOWDIGITALRESULT == 1)
          messg WDTCON.4 = 1
        endif
      endif
      SetReg  ANCON0, 0xFF        ;  
      ifdef PCFG15                ;  0 = Analog with these bits
        SetReg  ANCON1, 0xFF
      else
        SetReg  ANCON1, 0x7F
      endif
      ifdef ADSHR                 ; if chip has shared address SFR's
        MOVE?CT  0, WDTCON,ADSHR  ;   put SFR's back to legacy
        if (SHOWDIGITALRESULT == 1)
          messg WDTCON.4 = 0
        endif
      endif
    else
      ifdef PCFG6                 ; Some chips use ADCON1 bits
        SetReg ADCON1, 0x7F       ;  the same way as ANCONx
      else
        ifdef PCFG3               ; if there's a PCFG3 bit
          ifdef ADCON2            ;  but has no ADCON2, more than 8 A/D's
            SetReg  ADCON1, 0x0F  ;   A/D cfg from table = 0x0F
          else
            SetReg  ADCON1, 7     ; most common, 8 or less A/D ch's
          endif
        else
          ifdef PCFG2
            ifdef ADCON1
              SetReg  ADCON1, 7   ; A/D is 8-bit, 5 ch's
            endif
          else
            ifdef ADCON0          ; if there's an ADCON0
              ifndef ADCON1       ;  but not an ADCON1
                SetReg  ADCON0, 0 ;  A/D is 8-bit and ADS bits are in ADCON0
              endif
            endif
          endif
        endif
      endif
    endif
  endif
    
;----[Comparators]----------------------------------------------------------
  ifdef CMCON
    ifdef CMEN0       ; 18F1230/1330
      ;SetReg CMCON, 0 ; default already
    else
      ifdef CM2
        SetReg CMCON, 7
      else
        ifdef PM_USED
          "ALLDIGITAL: CMCON value not found"
        else
          messg "ALLDIGITAL: CMCON value not found"
        endif
      endif
    endif
  endif
  
  ifdef CMCON0
    ifdef NOT_COUTEN            
      SetReg CMCON0, 0xF1       ; 10F204/206
    else
      ifdef CMCH
        SetReg CMCON0, 0        ; 12F609/615
      else
        ifdef CM2
          SetReg CMCON0, 7      ; 12F635/683, 
        else                    ; 16F636/639/684/688/913/914/916/917/946
          ifndef PM_USED
            "ALLDIGITAL: CMCON0 value not found"
          else  
            messg "ALLDIGITAL: CMCON0 value not found"
          endif  
        endif
      endif
    endif
  endif
  
  ifdef CM1CON0                 ; individual comparator controls
    ifdef NOT_C1OUTEN           
      SetReg CM1CON0, 0xF1      ; 12F510/506, 16F526
    else
;      SetReg CM1CON0, 0         ; these already default to 0 --------------
    endif                       ; 16F882/883/884/886/887/ 610/616
  endif                         ; 16F631/677/685/687/689/690
                                
                                ; these aren't 0, but default to OFF
                                ; 18F24J11/25J11/26J11/44J11/45J11/46J11
                                ; 18F24J50/25J50/26J50/44J50/45J50/46J50
  ifdef CM2CON0
    ifdef NOT_C2OUTEN            
      SetReg CM2CON0, 0xF1
    else
;      SetReg CM2CON0, 0         ; already default to 0 -----see above-
    endif
  endif
    
; CM1CON                      ; these aren't 0, but default to OFF
                              ; 18F65J50/66J50/66J55/67J50/66J11/66J16/67J11
; CM2CON                      ; 18F85J50/86J50/86J55/87J50/86J11/86J16/87J11

  ifndef PM_USED
    list
  endif


	ENDASM?

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00004	#CONFIG
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00045	DEFINE LCD_DREG PORTA 
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00046	DEFINE LCD_DBIT 0
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00047	DEFINE LCD_RSREG PORTA
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00048	DEFINE LCD_RSBIT 5 
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00049	DEFINE LCD_EREG PORTA
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00050	DEFINE LCD_EBIT 4 
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00051	DEFINE LCD_BITS 4 
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00052	DEFINE LCD_LINE 2
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00053	DEFINE I2C_SCLOUT 1
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00054	DEFINE I2C_SLOW 1
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00055	define NO_CLRWDT 1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00057	INTCON2.7 = 0 
	MOVE?CT	000h, _INTCON2??7

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00058	WPUB = %11110111
	MOVE?CB	0F7h, WPUB

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00060	OSCCON  = %1111000
	MOVE?CB	078h, OSCCON

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00061	OSCCON2 = %0000000
	MOVE?CB	000h, OSCCON2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00062	OSCTUNE = %10000000
	MOVE?CB	080h, OSCTUNE

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00063	T1CON = %00000111
	MOVE?CB	007h, T1CON

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00064	ANSELA = 0
	MOVE?CB	000h, ANSELA

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00065	ANSELB = 0
	MOVE?CB	000h, ANSELB

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00066	ANSELC = %000010
	MOVE?CB	002h, ANSELC

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00067	TRISC = 255
	MOVE?CB	0FFh, TRISC

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00068	TRISA = 255
	MOVE?CB	0FFh, TRISA

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00069	TRISB = 255
	MOVE?CB	0FFh, TRISB

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00070	pause 200
	PAUSE?C	0C8h
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00071	RTC             con %10100010
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00072	EPROM           CON %10100000
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00073	LM75            CON %10010000
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00074	Strelica        con 126 
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00075	PauzaZaTaster   con 110
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00077	DEFINE I2C_SDA PORTC,0 
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00078	DEFINE I2C_SCL PORTC,1 
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00080	symbol BCKLIGHT        = PORTC.3
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00081	symbol SDA             = PORTC.0
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00082	symbol SCL             = PORTC.1
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00083	symbol RELEJ1          = PORTC.4
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00084	symbol RELEJ2          = PORTC.5
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00085	symbol TASTLEVO        = PORTB.1
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00086	symbol TASTDESNO       = PORTB.4
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00087	symbol TASTGORE        = PORTB.6
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00088	symbol TASTDOLE        = PORTB.5
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00089	symbol TASTOK          = PORTB.2
; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00090	symbol TASTODRZAVANJE  = PORTB.7

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00092	low bcklight
	LOW?T	_BCKLIGHT

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00093	low relej1
	LOW?T	_RELEJ1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00094	low relej2
	LOW?T	_RELEJ2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00095	LOW portc.6
	LOW?T	_PORTC??6

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00096	low portb.3
	LOW?T	_PORTB??3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00097	low portb.0
	LOW?T	_PORTB??0

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00098	low portc.2
	LOW?T	_PORTC??2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00099	input tastlevo
	INPUT?T	_TASTLEVO

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00100	input tastdesno
	INPUT?T	_TASTDESNO

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00101	input tastdole
	INPUT?T	_TASTDOLE

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00102	input tastgore
	INPUT?T	_TASTGORE

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00103	input tastodrzavanje
	INPUT?T	_TASTODRZAVANJE

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00104	input tastok
	INPUT?T	_TASTOK

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00214	CLEAR
	CLEAR?	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00215	Frequency               = 30000
	MOVE?CW	000007530h, _Frequency

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00216	starostanjeDesno        = 1
	MOVE?CT	001h, _StaroStanjeDesno

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00217	starostanjelevo         = 1 
	MOVE?CT	001h, _StaroStanjeLevo

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00218	starostanjegore         = 1 
	MOVE?CT	001h, _StaroStanjeGore

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00219	starostanjedole         = 1 
	MOVE?CT	001h, _StaroStanjeDole

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00220	StaroStanjeOk           = 1         
	MOVE?CT	001h, _StaroStanjeOk

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00221	StaroStanjeOdrzavanje   = 1 
	MOVE?CT	001h, _StaroStanjeOdrzavanje

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00223	odrzavanjemod = 0 
	MOVE?CT	000h, _OdrzavanjeMod

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00224	BCKLIGHT = 1
	MOVE?CT	001h, _BCKLIGHT

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00225	I2Cwrite  sda, scl, lm75,1,[%00000000]   
	I2CDATA?T	_SDA
	I2CCLOCK?T	_SCL
	I2CWRITE?C	_LM75
	I2CWRITE?C	001h
	I2CWRITES?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00226	pause 20 
	PAUSE?C	014h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00228	read 1,x
	READADDRESS?C	001h
	READ?B	_x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00229	if x <> 1 then gosub PrviPut
	CMPEQ?BCL	_x, 001h, L00039
	GOSUB?L	_PrviPut
	LABEL?L	L00039	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00231	Splash:

	LABEL?L	_Splash	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00232	    gosub ocitajtastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00233	    if odrzavanje = 1 then 
	CMPNE?TCL	_odrzavanje, 001h, L00041

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00234	        read 7,TemperaturaIzEproma
	READADDRESS?C	007h
	READ?B	_TemperaturaIzEproma

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00235	        gosub podesavanjetemperature
	GOSUB?L	_podesavanjetemperature

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00236	    endif 
	LABEL?L	L00041	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00239	    gosub EEpromMemorija
	GOSUB?L	_EEpromMemorija

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00240	    read 5,AutomatskoAkvarijum
	READADDRESS?C	005h
	READ?T	_AutomatskoAkvarijum

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00241	    read 6,Automatskosoba
	READADDRESS?C	006h
	READ?T	_AutomatskoSoba

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00242	    if automatskoakvarijum = 0 then
	CMPNE?TCL	_AutomatskoAkvarijum, 000h, L00043

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00243	        read 100,kanal1
	READADDRESS?C	064h
	READ?B	_Kanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00244	        read 101,kanal2  
	READADDRESS?C	065h
	READ?B	_Kanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00245	        read 102,kanal3 
	READADDRESS?C	066h
	READ?B	_Kanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00246	        DutyCycleL1 =(Kanal1 * 1024)/100
	MUL?BCN	_Kanal1, 000000400h, T1
	DIV?NCN	T1, 064h, _DutyCycleL1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00247	        DutyCycleL2 =(Kanal2 * 1024)/100
	MUL?BCN	_Kanal2, 000000400h, T1
	DIV?NCN	T1, 064h, _DutyCycleL2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00248	        DutyCycleL3 =(Kanal3 * 1024)/100
	MUL?BCN	_Kanal3, 000000400h, T1
	DIV?NCN	T1, 064h, _DutyCycleL3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00249	        @ HPWM10 4, _DutyCycleL1, _Frequency

	ASM?
 HPWM10 4, _DutyCycleL1, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00250	        @ HPWM10 3, _DutyCycleL2, _Frequency

	ASM?
 HPWM10 3, _DutyCycleL2, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00251	        @ HPWM10 1, _DutyCycleL3, _Frequency

	ASM?
 HPWM10 1, _DutyCycleL3, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00252	    endif
	LABEL?L	L00043	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00253	    if automatskosoba = 0 then 
	CMPNE?TCL	_AutomatskoSoba, 000h, L00045

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00254	        read 103,kanal4
	READADDRESS?C	067h
	READ?B	_Kanal4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00255	        DutyCycleL4 =(Kanal4 * 1024)/100
	MUL?BCN	_Kanal4, 000000400h, T1
	DIV?NCN	T1, 064h, _DutyCycleL4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00256	        @ HPWM10 2, _DutyCycleL4, _Frequency

	ASM?
 HPWM10 2, _DutyCycleL4, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00257	    endif
	LABEL?L	L00045	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00258	    Lcdout $fe,2, "AQUA-LED            "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	041h
	LCDOUT?C	051h
	LCDOUT?C	055h
	LCDOUT?C	041h
	LCDOUT?C	02Dh
	LCDOUT?C	04Ch
	LCDOUT?C	045h
	LCDOUT?C	044h
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00259	    Lcdout $fe,$C0, "DANER FW:3.0       "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	044h
	LCDOUT?C	041h
	LCDOUT?C	04Eh
	LCDOUT?C	045h
	LCDOUT?C	052h
	LCDOUT?C	020h
	LCDOUT?C	046h
	LCDOUT?C	057h
	LCDOUT?C	03Ah
	LCDOUT?C	033h
	LCDOUT?C	02Eh
	LCDOUT?C	030h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00260	    pause 1500
	PAUSE?C	005DCh

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00261	    x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00262	goto main
	GOTO?L	_main

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00264	Main:

	LABEL?L	_main	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00265	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00266	gosub uzmivreme
	GOSUB?L	_uzmivreme

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00267	gosub IzmeriTemperaturu
	GOSUB?L	_IzmeriTemperaturu

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00268	if ok = 1 then
	CMPNE?TCL	_ok, 001h, L00047

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00269	    if bcklight = 1 then 
	CMPNE?TCL	_BCKLIGHT, 001h, L00049

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00270	        x = 1 
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00271	        goto GlavniMeni
	GOTO?L	_GlavniMeni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00272	    else
	GOTO?L	L00050
	LABEL?L	L00049	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00273	        bcklight = 1 
	MOVE?CT	001h, _BCKLIGHT

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00274	    endif 
	LABEL?L	L00050	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00275	endif
	LABEL?L	L00047	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00276	if gore = 1  then 
	CMPNE?TCL	_gore, 001h, L00051

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00277	    if bcklight = 1 then 
	CMPNE?TCL	_BCKLIGHT, 001h, L00053

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00278	            if AutomatskoAkvarijum  = 1 then 
	CMPNE?TCL	_AutomatskoAkvarijum, 001h, L00055

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00279	                    AutomatskoAkvarijum  = 0
	MOVE?CT	000h, _AutomatskoAkvarijum

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00280	                    write 5,0
	WRITEADDRESS?C	005h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00281	            else
	GOTO?L	L00056
	LABEL?L	L00055	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00282	                    AutomatskoAkvarijum  = 1
	MOVE?CT	001h, _AutomatskoAkvarijum

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00283	                    kanal1 = stkanal1 / 100 
	DIV?NCB	_STKanal1, 064h, _Kanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00284	                    kanal2 = stkanal2 / 100 
	DIV?NCB	_STKanal2, 064h, _Kanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00285	                    kanal3 = stkanal3 / 100 
	DIV?NCB	_STKanal3, 064h, _Kanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00286	                    write 5,1
	WRITEADDRESS?C	005h
	WRITE?C	001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00287	            endif 
	LABEL?L	L00056	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00288	    else
	GOTO?L	L00054
	LABEL?L	L00053	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00289	            bcklight = 1
	MOVE?CT	001h, _BCKLIGHT

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00290	    endif
	LABEL?L	L00054	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00291	endif
	LABEL?L	L00051	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00293	if dole = 1 then 
	CMPNE?TCL	_dole, 001h, L00057

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00294	     if bcklight = 1 then 
	CMPNE?TCL	_BCKLIGHT, 001h, L00059

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00295	            if AutomatskoSoba = 1 then 
	CMPNE?TCL	_AutomatskoSoba, 001h, L00061

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00296	                    AutomatskoSoba = 0
	MOVE?CT	000h, _AutomatskoSoba

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00297	                    write 6,0
	WRITEADDRESS?C	006h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00298	            else
	GOTO?L	L00062
	LABEL?L	L00061	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00299	                    AutomatskoSoba = 1
	MOVE?CT	001h, _AutomatskoSoba

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00300	                    write 6,1
	WRITEADDRESS?C	006h
	WRITE?C	001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00301	                    kanal4 = stkanal4 / 100 
	DIV?NCB	_STKanal4, 064h, _Kanal4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00302	            endif
	LABEL?L	L00062	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00303	     else
	GOTO?L	L00060
	LABEL?L	L00059	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00304	         bcklight = 1 
	MOVE?CT	001h, _BCKLIGHT

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00305	     endif
	LABEL?L	L00060	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00306	endif  
	LABEL?L	L00057	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00308	if odrzavanje = 1 then
	CMPNE?TCL	_odrzavanje, 001h, L00063

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00309	     if bcklight = 1 then  
	CMPNE?TCL	_BCKLIGHT, 001h, L00065

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00310	            OdrzavanjeMod  = 1 
	MOVE?CT	001h, _OdrzavanjeMod

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00311	            read 2,OdrzavanjeJacina 
	READADDRESS?C	002h
	READ?B	_OdrzavanjeJacina

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00312	            goto mododrzavanje
	GOTO?L	_mododrzavanje

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00313	     else
	GOTO?L	L00066
	LABEL?L	L00065	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00314	        bcklight = 1 
	MOVE?CT	001h, _BCKLIGHT

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00315	     endif 
	LABEL?L	L00066	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00316	endif
	LABEL?L	L00063	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00318	if  tastlevo = 0 then
	CMPNE?TCL	_TASTLEVO, 000h, L00067

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00319	    levibrojac = levibrojac + 1
	ADD?BCB	_LeviBrojac, 001h, _LeviBrojac

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00320	    if levibrojac > 50 then  
	CMPLE?BCL	_LeviBrojac, 032h, L00069

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00321	        toggle relej1
	TOGGLE?T	_RELEJ1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00322	        levibrojac = 0 
	MOVE?CB	000h, _LeviBrojac

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00323	    endif
	LABEL?L	L00069	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00324	else
	GOTO?L	L00068
	LABEL?L	L00067	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00325	    levibrojac = 0 
	MOVE?CB	000h, _LeviBrojac

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00326	endif
	LABEL?L	L00068	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00328	if  tastdesno = 0 then
	CMPNE?TCL	_TASTDESNO, 000h, L00071

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00329	    desnibrojac = desnibrojac + 1
	ADD?BCB	_DesniBrojac, 001h, _DesniBrojac

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00330	    if desnibrojac > 50 then  
	CMPLE?BCL	_DesniBrojac, 032h, L00073

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00331	        toggle relej2
	TOGGLE?T	_RELEJ2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00332	        desnibrojac = 0
	MOVE?CB	000h, _DesniBrojac

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00333	    endif
	LABEL?L	L00073	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00334	else
	GOTO?L	L00072
	LABEL?L	L00071	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00335	    desnibrojac = 0
	MOVE?CB	000h, _DesniBrojac

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00336	endif       
	LABEL?L	L00072	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00338	if automatskoakvarijum = 0 then 
	CMPNE?TCL	_AutomatskoAkvarijum, 000h, L00075

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00339	    read 100,kanal1
	READADDRESS?C	064h
	READ?B	_Kanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00340	    read 101,kanal2  
	READADDRESS?C	065h
	READ?B	_Kanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00341	    read 102,kanal3 
	READADDRESS?C	066h
	READ?B	_Kanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00342	    DutyCycleL1 =(Kanal1 * 1024)/100
	MUL?BCN	_Kanal1, 000000400h, T1
	DIV?NCN	T1, 064h, _DutyCycleL1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00343	    DutyCycleL2 =(Kanal2 * 1024)/100
	MUL?BCN	_Kanal2, 000000400h, T1
	DIV?NCN	T1, 064h, _DutyCycleL2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00344	    DutyCycleL3 =(Kanal3 * 1024)/100
	MUL?BCN	_Kanal3, 000000400h, T1
	DIV?NCN	T1, 064h, _DutyCycleL3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00345	    @ HPWM10 4, _DutyCycleL1, _Frequency

	ASM?
 HPWM10 4, _DutyCycleL1, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00346	    @ HPWM10 3, _DutyCycleL2, _Frequency

	ASM?
 HPWM10 3, _DutyCycleL2, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00347	    @ HPWM10 1, _DutyCycleL3, _Frequency

	ASM?
 HPWM10 1, _DutyCycleL3, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00348	    akvarijum = "M"
	MOVE?CW	04Dh, _Akvarijum

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00349	else
	GOTO?L	L00076
	LABEL?L	L00075	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00350	    akvarijum = "A"
	MOVE?CW	041h, _Akvarijum

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00351	endif
	LABEL?L	L00076	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00353	if automatskosoba = 0 then   'onda je manuelno
	CMPNE?TCL	_AutomatskoSoba, 000h, L00077

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00354	    read 103,kanal4
	READADDRESS?C	067h
	READ?B	_Kanal4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00355	    DutyCycleL4 =(Kanal4 * 1024)/100
	MUL?BCN	_Kanal4, 000000400h, T1
	DIV?NCN	T1, 064h, _DutyCycleL4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00356	    @ HPWM10 2, _DutyCycleL4, _Frequency

	ASM?
 HPWM10 2, _DutyCycleL4, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00357	     soba = "M" 
	MOVE?CW	04Dh, _Soba

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00358	else
	GOTO?L	L00078
	LABEL?L	L00077	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00359	     soba = "A" 
	MOVE?CW	041h, _Soba

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00360	endif 
	LABEL?L	L00078	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00362	x = x + 1 
	ADD?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00363	if x < 90 then  DveTacke =  " " 
	CMPGE?BCL	_x, 05Ah, L00079
	MOVE?CB	020h, _DveTacke
	LABEL?L	L00079	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00364	if x > 90 and x < 170 then DveTacke = ":"
	CMPGT?BCB	_x, 05Ah, T1
	CMPLT?BCB	_x, 0AAh, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00081
	MOVE?CB	03Ah, _DveTacke
	LABEL?L	L00081	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00365	if x > 170 then x = 0 
	CMPLE?BCL	_x, 0AAh, L00083
	MOVE?CB	000h, _x
	LABEL?L	L00083	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00367	    if gore = 1 or dole = 1 or levo = 1 or desno = 1  or ok = 1 or odrzavanje = 1 then 
	CMPEQ?TCB	_gore, 001h, T1
	CMPEQ?TCB	_dole, 001h, T2
	LOR?BBN	T1, T2, T2
	CMPEQ?TCB	_levo, 001h, T3
	LOR?NBN	T2, T3, T3
	CMPEQ?TCB	_desno, 001h, T4
	LOR?NBN	T3, T4, T4
	CMPEQ?TCB	_ok, 001h, T5
	LOR?NBN	T4, T5, T5
	CMPEQ?TCB	_odrzavanje, 001h, T6
	LOR?NBN	T5, T6, T6
	CMPF?NL	T6, L00085

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00368	        BezTastera  =  0
	MOVE?CW	000h, _BezTastera

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00369	    endif 
	LABEL?L	L00085	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00371	    if BezTastera < 1700 then 
	CMPGE?WCL	_BezTastera, 0000006A4h, L00087

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00372	        BezTastera =  BezTastera + 1 
	ADD?WCW	_BezTastera, 001h, _BezTastera

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00373	        bcklight = 1
	MOVE?CT	001h, _BCKLIGHT

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00374	     else
	GOTO?L	L00088
	LABEL?L	L00087	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00375	        bcklight = 0 
	MOVE?CT	000h, _BCKLIGHT

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00376	    endif  
	LABEL?L	L00088	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00379	 if decsec > 10 and  decsec < 15 or decsec > 25 and decsec < 30 or decsec > 45 and decsec < 50 then  
	CMPGT?BCB	_DecSec, 00Ah, T1
	CMPLT?BCB	_DecSec, 00Fh, T2
	LAND?BBN	T1, T2, T2
	CMPGT?BCB	_DecSec, 019h, T3
	LOR?NBN	T2, T3, T3
	CMPLT?BCB	_DecSec, 01Eh, T4
	LAND?NBN	T3, T4, T4
	CMPGT?BCB	_DecSec, 02Dh, T5
	LOR?NBN	T4, T5, T5
	CMPLT?BCB	_DecSec, 032h, T6
	LAND?NBN	T5, T6, T6
	CMPF?NL	T6, L00089

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00380	        Lcdout $fe, 2 ,"AQUA-LED  ","T=",dec2 temperatura,223,"C"
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	041h
	LCDOUT?C	051h
	LCDOUT?C	055h
	LCDOUT?C	041h
	LCDOUT?C	02Dh
	LCDOUT?C	04Ch
	LCDOUT?C	045h
	LCDOUT?C	044h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	054h
	LCDOUT?C	03Dh
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_Temperatura
	LCDOUTDEC?	
	LCDOUT?C	0DFh
	LCDOUT?C	043h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00381	 else
	GOTO?L	L00090
	LABEL?L	L00089	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00382	        Lcdout $fe, 2 ,"AQUA-LED   ",dec2 decsatglavni,DveTacke,dec2 decminutglavni
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	041h
	LCDOUT?C	051h
	LCDOUT?C	055h
	LCDOUT?C	041h
	LCDOUT?C	02Dh
	LCDOUT?C	04Ch
	LCDOUT?C	045h
	LCDOUT?C	044h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_DecSatglavni
	LCDOUTDEC?	
	LCDOUT?B	_DveTacke
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_DecMinutGlavni
	LCDOUTDEC?	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00383	 endif
	LABEL?L	L00090	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00385	        if relej1 = 1 and relej2 = 1 then lcdout $fe,$c0,"1=ON 2=ON    ",akvarijum, "#",soba
	CMPEQ?TCB	_RELEJ1, 001h, T1
	CMPEQ?TCB	_RELEJ2, 001h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00091
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	031h
	LCDOUT?C	03Dh
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	032h
	LCDOUT?C	03Dh
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?W	_Akvarijum
	LCDOUT?C	023h
	LCDOUT?W	_Soba
	LABEL?L	L00091	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00386	        if relej1 = 1 and relej2 = 0 then lcdout $fe,$c0,"1=ON 2=OFF   ",akvarijum, "#",soba
	CMPEQ?TCB	_RELEJ1, 001h, T1
	CMPEQ?TCB	_RELEJ2, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00093
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	031h
	LCDOUT?C	03Dh
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	032h
	LCDOUT?C	03Dh
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?W	_Akvarijum
	LCDOUT?C	023h
	LCDOUT?W	_Soba
	LABEL?L	L00093	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00387	        if relej1 = 0 and relej2 = 0 then lcdout $fe,$c0,"1=OFF 2=OFF  ",akvarijum, "#",soba
	CMPEQ?TCB	_RELEJ1, 000h, T1
	CMPEQ?TCB	_RELEJ2, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00095
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	031h
	LCDOUT?C	03Dh
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	032h
	LCDOUT?C	03Dh
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?W	_Akvarijum
	LCDOUT?C	023h
	LCDOUT?W	_Soba
	LABEL?L	L00095	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00388	        if relej1 = 0 and relej2 = 1 then lcdout $fe,$c0,"1=OFF 2=ON   ",akvarijum, "#",soba
	CMPEQ?TCB	_RELEJ1, 000h, T1
	CMPEQ?TCB	_RELEJ2, 001h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00097
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	031h
	LCDOUT?C	03Dh
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	020h
	LCDOUT?C	032h
	LCDOUT?C	03Dh
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?W	_Akvarijum
	LCDOUT?C	023h
	LCDOUT?W	_Soba
	LABEL?L	L00097	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00391	gosub kreni
	GOSUB?L	_kreni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00392	goto Main 
	GOTO?L	_main

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00394	Kreni:

	LABEL?L	_kreni	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00396	    novostanjertc = DecMinutGlavni
	MOVE?BB	_DecMinutGlavni, _NovostanjeRTC

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00397	    TMinuta = (DecSatglavni * 60) + DecMinutGlavni
	MUL?BCN	_DecSatglavni, 03Ch, T1
	ADD?NBW	T1, _DecMinutGlavni, _TMinuta

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00398	    if novostanjeRTC <> starostanjeRTC then
	CMPEQ?BBL	_NovostanjeRTC, _StarostanjeRTC, L00099

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00399	          NoviMinut = 1
	MOVE?CB	001h, _NoviMinut

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00400	    else
	GOTO?L	L00100
	LABEL?L	L00099	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00401	          NoviMinut = 0
	MOVE?CB	000h, _NoviMinut

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00402	    endif
	LABEL?L	L00100	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00404	    satPocetak   = memorija[8] 
	MOVE?BB	_Memorija + 00008h, _SatPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00405	    MinutPocetak = memorija[9]  
	MOVE?BB	_Memorija + 00009h, _MinutPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00406	    osvPocetak   = memorija[10]
	MOVE?BB	_Memorija + 0000Ah, _OsvPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00407	    SatKraj      = memorija[12]  
	MOVE?BB	_Memorija + 0000Ch, _SatKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00408	    MinutKraj    = memorija[13]
	MOVE?BB	_Memorija + 0000Dh, _MinutKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00409	    OsvKraj      = memorija[14]
	MOVE?BB	_Memorija + 0000Eh, _OsvKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00411	    PMinuta = (satpocetak * 60)+ minutpocetak
	MUL?BCN	_SatPocetak, 03Ch, T1
	ADD?NBW	T1, _MinutPocetak, _PMinuta

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00412	    KMinuta = (satkraj * 60) + minutkraj
	MUL?BCN	_SatKraj, 03Ch, T1
	ADD?NBW	T1, _MinutKraj, _KMinuta

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00413	    if Pminuta =< tminuta and KMinuta >= tminuta then 
	CMPLE?WWB	_PMinuta, _TMinuta, T1
	CMPGE?WWB	_KMinuta, _TMinuta, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00101

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00415	       if Pminuta = tminuta and decsec = 0 then  
	CMPEQ?WWB	_PMinuta, _TMinuta, T1
	CMPEQ?BCB	_DecSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00103

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00416	          kanal1 = osvpocetak
	MOVE?BB	_OsvPocetak, _Kanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00417	          stkanal1 = osvpocetak * 100
	MUL?BCN	_OsvPocetak, 064h, _STKanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00418	       else
	GOTO?L	L00104
	LABEL?L	L00103	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00419	          if kminuta = tminuta and decsec = 0 then    
	CMPEQ?WWB	_KMinuta, _TMinuta, T1
	CMPEQ?BCB	_DecSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00105

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00420	              kanal1 = osvkraj
	MOVE?BB	_OsvKraj, _Kanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00421	          else
	GOTO?L	L00106
	LABEL?L	L00105	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00422	              if noviminut = 1 then 
	CMPNE?BCL	_NoviMinut, 001h, L00107

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00423	                 stkanal1 = stkanal1 + (((osvkraj - osvpocetak)*100/(kminuta - pminuta)))  
	SUB?BBN	_OsvKraj, _OsvPocetak, T1
	MUL?NCN	T1, 064h, T1
	SUB?WWN	_KMinuta, _PMinuta, T2
	DIV?NNN	T1, T2, T2
	ADD?NNN	_STKanal1, T2, _STKanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00424	                 kanal1 = stkanal1 / 100 
	DIV?NCB	_STKanal1, 064h, _Kanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00425	              endif
	LABEL?L	L00107	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00426	          endif
	LABEL?L	L00106	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00427	       endif
	LABEL?L	L00104	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00428	    endif
	LABEL?L	L00101	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00430	    satPocetak   = memorija[16] 
	MOVE?BB	_Memorija + 00010h, _SatPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00431	    MinutPocetak = memorija[17]  
	MOVE?BB	_Memorija + 00011h, _MinutPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00432	    osvPocetak   = memorija[18] 
	MOVE?BB	_Memorija + 00012h, _OsvPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00433	    SatKraj      = memorija[20]  
	MOVE?BB	_Memorija + 00014h, _SatKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00434	    MinutKraj    = memorija[21]  
	MOVE?BB	_Memorija + 00015h, _MinutKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00435	    OsvKraj      = memorija[22]  
	MOVE?BB	_Memorija + 00016h, _OsvKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00437	    PMinuta = (satpocetak * 60)+ minutpocetak
	MUL?BCN	_SatPocetak, 03Ch, T1
	ADD?NBW	T1, _MinutPocetak, _PMinuta

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00438	    KMinuta = (satkraj * 60) + minutkraj
	MUL?BCN	_SatKraj, 03Ch, T1
	ADD?NBW	T1, _MinutKraj, _KMinuta

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00440	    if Pminuta =< tminuta and KMinuta >= tminuta then 
	CMPLE?WWB	_PMinuta, _TMinuta, T1
	CMPGE?WWB	_KMinuta, _TMinuta, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00109

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00441	       if Pminuta = tminuta and decsec = 0 then 
	CMPEQ?WWB	_PMinuta, _TMinuta, T1
	CMPEQ?BCB	_DecSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00111

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00442	          kanal1 = osvpocetak
	MOVE?BB	_OsvPocetak, _Kanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00443	          stkanal1 = osvpocetak * 100
	MUL?BCN	_OsvPocetak, 064h, _STKanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00444	       else
	GOTO?L	L00112
	LABEL?L	L00111	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00445	          if kminuta = tminuta and decsec = 0 then   
	CMPEQ?WWB	_KMinuta, _TMinuta, T1
	CMPEQ?BCB	_DecSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00113

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00446	              kanal1 = osvkraj
	MOVE?BB	_OsvKraj, _Kanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00447	          else
	GOTO?L	L00114
	LABEL?L	L00113	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00448	              if noviminut = 1 then
	CMPNE?BCL	_NoviMinut, 001h, L00115

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00449	                 stkanal1 = stkanal1 - (((osvpocetak - osvkraj)*100/(kminuta - pminuta)))  
	SUB?BBN	_OsvPocetak, _OsvKraj, T1
	MUL?NCN	T1, 064h, T1
	SUB?WWN	_KMinuta, _PMinuta, T2
	DIV?NNN	T1, T2, T2
	SUB?NNN	_STKanal1, T2, _STKanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00450	                 kanal1 = stkanal1 / 100 
	DIV?NCB	_STKanal1, 064h, _Kanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00451	              endif
	LABEL?L	L00115	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00452	          endif
	LABEL?L	L00114	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00453	       endif
	LABEL?L	L00112	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00454	    endif 
	LABEL?L	L00109	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00456	    satPocetak   = memorija[24] 
	MOVE?BB	_Memorija + 00018h, _SatPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00457	    MinutPocetak = memorija[25]  
	MOVE?BB	_Memorija + 00019h, _MinutPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00458	    osvPocetak   = memorija[26]
	MOVE?BB	_Memorija + 0001Ah, _OsvPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00459	    SatKraj      = memorija[28]  
	MOVE?BB	_Memorija + 0001Ch, _SatKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00460	    MinutKraj    = memorija[29]
	MOVE?BB	_Memorija + 0001Dh, _MinutKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00461	    OsvKraj      = memorija[30]
	MOVE?BB	_Memorija + 0001Eh, _OsvKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00463	    PMinuta = (satpocetak * 60)+ minutpocetak
	MUL?BCN	_SatPocetak, 03Ch, T1
	ADD?NBW	T1, _MinutPocetak, _PMinuta

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00464	    KMinuta = (satkraj * 60) + minutkraj   
	MUL?BCN	_SatKraj, 03Ch, T1
	ADD?NBW	T1, _MinutKraj, _KMinuta

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00465	    if Pminuta =< tminuta and KMinuta >= tminuta then 
	CMPLE?WWB	_PMinuta, _TMinuta, T1
	CMPGE?WWB	_KMinuta, _TMinuta, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00117

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00466	       if Pminuta = tminuta and decsec = 0 then  
	CMPEQ?WWB	_PMinuta, _TMinuta, T1
	CMPEQ?BCB	_DecSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00119

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00467	          kanal2 = osvpocetak
	MOVE?BB	_OsvPocetak, _Kanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00468	          stkanal2 = osvpocetak * 100
	MUL?BCN	_OsvPocetak, 064h, _STKanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00469	       else
	GOTO?L	L00120
	LABEL?L	L00119	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00470	          if kminuta = tminuta and decsec = 0 then   
	CMPEQ?WWB	_KMinuta, _TMinuta, T1
	CMPEQ?BCB	_DecSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00121

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00471	              kanal2 = osvkraj
	MOVE?BB	_OsvKraj, _Kanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00472	          else
	GOTO?L	L00122
	LABEL?L	L00121	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00473	              if noviminut = 1 then 
	CMPNE?BCL	_NoviMinut, 001h, L00123

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00474	                 stkanal2 = stkanal2 + (((osvkraj - osvpocetak)*100/(kminuta - pminuta)))  
	SUB?BBN	_OsvKraj, _OsvPocetak, T1
	MUL?NCN	T1, 064h, T1
	SUB?WWN	_KMinuta, _PMinuta, T2
	DIV?NNN	T1, T2, T2
	ADD?NNN	_STKanal2, T2, _STKanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00475	                 kanal2 = stkanal2 / 100 
	DIV?NCB	_STKanal2, 064h, _Kanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00476	              endif
	LABEL?L	L00123	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00477	          endif
	LABEL?L	L00122	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00478	       endif
	LABEL?L	L00120	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00479	    endif  
	LABEL?L	L00117	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00481	    satPocetak   = memorija[32] 
	MOVE?BB	_Memorija + 00020h, _SatPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00482	    MinutPocetak = memorija[33]  
	MOVE?BB	_Memorija + 00021h, _MinutPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00483	    osvPocetak   = memorija[34] 
	MOVE?BB	_Memorija + 00022h, _OsvPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00484	    SatKraj      = memorija[36]  
	MOVE?BB	_Memorija + 00024h, _SatKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00485	    MinutKraj    = memorija[37]  
	MOVE?BB	_Memorija + 00025h, _MinutKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00486	    OsvKraj      = memorija[38]  
	MOVE?BB	_Memorija + 00026h, _OsvKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00488	    PMinuta = (satpocetak * 60)+ minutpocetak
	MUL?BCN	_SatPocetak, 03Ch, T1
	ADD?NBW	T1, _MinutPocetak, _PMinuta

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00489	    KMinuta = (satkraj * 60) + minutkraj
	MUL?BCN	_SatKraj, 03Ch, T1
	ADD?NBW	T1, _MinutKraj, _KMinuta

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00491	    if Pminuta =< tminuta and KMinuta >= tminuta then 
	CMPLE?WWB	_PMinuta, _TMinuta, T1
	CMPGE?WWB	_KMinuta, _TMinuta, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00125

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00492	       if Pminuta = tminuta and decsec = 0 then   
	CMPEQ?WWB	_PMinuta, _TMinuta, T1
	CMPEQ?BCB	_DecSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00127

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00493	          kanal2 = osvpocetak
	MOVE?BB	_OsvPocetak, _Kanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00494	          stkanal2 = osvpocetak * 100
	MUL?BCN	_OsvPocetak, 064h, _STKanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00495	       else
	GOTO?L	L00128
	LABEL?L	L00127	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00496	          if kminuta = tminuta and decsec = 0 then   
	CMPEQ?WWB	_KMinuta, _TMinuta, T1
	CMPEQ?BCB	_DecSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00129

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00497	              kanal2 = osvkraj
	MOVE?BB	_OsvKraj, _Kanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00498	          else
	GOTO?L	L00130
	LABEL?L	L00129	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00499	              if noviminut = 1 then
	CMPNE?BCL	_NoviMinut, 001h, L00131

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00500	                 stkanal2 = stkanal2 - (((osvpocetak - osvkraj)*100/(kminuta - pminuta)))  
	SUB?BBN	_OsvPocetak, _OsvKraj, T1
	MUL?NCN	T1, 064h, T1
	SUB?WWN	_KMinuta, _PMinuta, T2
	DIV?NNN	T1, T2, T2
	SUB?NNN	_STKanal2, T2, _STKanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00501	                 kanal2 = stkanal2 / 100 
	DIV?NCB	_STKanal2, 064h, _Kanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00502	              endif
	LABEL?L	L00131	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00503	          endif
	LABEL?L	L00130	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00504	       endif
	LABEL?L	L00128	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00505	    endif
	LABEL?L	L00125	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00508	    satPocetak   = memorija[40] 
	MOVE?BB	_Memorija + 00028h, _SatPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00509	    MinutPocetak = memorija[41]  
	MOVE?BB	_Memorija + 00029h, _MinutPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00510	    osvPocetak   = memorija[42]
	MOVE?BB	_Memorija + 0002Ah, _OsvPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00511	    SatKraj      = memorija[44]  
	MOVE?BB	_Memorija + 0002Ch, _SatKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00512	    MinutKraj    = memorija[45]
	MOVE?BB	_Memorija + 0002Dh, _MinutKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00513	    OsvKraj      = memorija[46]
	MOVE?BB	_Memorija + 0002Eh, _OsvKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00515	    PMinuta = (satpocetak * 60)+ minutpocetak
	MUL?BCN	_SatPocetak, 03Ch, T1
	ADD?NBW	T1, _MinutPocetak, _PMinuta

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00516	    KMinuta = (satkraj * 60) + minutkraj
	MUL?BCN	_SatKraj, 03Ch, T1
	ADD?NBW	T1, _MinutKraj, _KMinuta

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00518	    if Pminuta =< tminuta and KMinuta >= tminuta then 
	CMPLE?WWB	_PMinuta, _TMinuta, T1
	CMPGE?WWB	_KMinuta, _TMinuta, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00133

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00519	       if Pminuta = tminuta and decsec = 0 then   
	CMPEQ?WWB	_PMinuta, _TMinuta, T1
	CMPEQ?BCB	_DecSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00135

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00520	          kanal3 = osvpocetak
	MOVE?BB	_OsvPocetak, _Kanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00521	          stkanal3 = osvpocetak * 100
	MUL?BCN	_OsvPocetak, 064h, _STKanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00522	       else
	GOTO?L	L00136
	LABEL?L	L00135	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00523	          if kminuta = tminuta and decsec = 0 then   
	CMPEQ?WWB	_KMinuta, _TMinuta, T1
	CMPEQ?BCB	_DecSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00137

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00524	              kanal3 = osvkraj
	MOVE?BB	_OsvKraj, _Kanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00525	          else
	GOTO?L	L00138
	LABEL?L	L00137	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00526	              if noviminut = 1 then 
	CMPNE?BCL	_NoviMinut, 001h, L00139

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00527	                 stkanal3 = stkanal3 + (((osvkraj - osvpocetak)*100/(kminuta - pminuta)))  
	SUB?BBN	_OsvKraj, _OsvPocetak, T1
	MUL?NCN	T1, 064h, T1
	SUB?WWN	_KMinuta, _PMinuta, T2
	DIV?NNN	T1, T2, T2
	ADD?NNN	_STKanal3, T2, _STKanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00528	                 kanal3 = stkanal3 / 100 
	DIV?NCB	_STKanal3, 064h, _Kanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00529	              endif
	LABEL?L	L00139	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00530	          endif
	LABEL?L	L00138	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00531	       endif
	LABEL?L	L00136	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00532	    endif 
	LABEL?L	L00133	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00534	    satPocetak   = memorija[48] 
	MOVE?BB	_Memorija + 00030h, _SatPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00535	    MinutPocetak = memorija[49]  
	MOVE?BB	_Memorija + 00031h, _MinutPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00536	    osvPocetak   = memorija[50] 
	MOVE?BB	_Memorija + 00032h, _OsvPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00537	    SatKraj      = memorija[52]  
	MOVE?BB	_Memorija + 00034h, _SatKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00538	    MinutKraj    = memorija[53]  
	MOVE?BB	_Memorija + 00035h, _MinutKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00539	    OsvKraj      = memorija[54]  
	MOVE?BB	_Memorija + 00036h, _OsvKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00541	    PMinuta = (satpocetak * 60)+ minutpocetak
	MUL?BCN	_SatPocetak, 03Ch, T1
	ADD?NBW	T1, _MinutPocetak, _PMinuta

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00542	    KMinuta = (satkraj * 60) + minutkraj
	MUL?BCN	_SatKraj, 03Ch, T1
	ADD?NBW	T1, _MinutKraj, _KMinuta

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00544	    if Pminuta =< tminuta and KMinuta >= tminuta then 
	CMPLE?WWB	_PMinuta, _TMinuta, T1
	CMPGE?WWB	_KMinuta, _TMinuta, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00141

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00545	       if Pminuta = tminuta and decsec = 0 then   
	CMPEQ?WWB	_PMinuta, _TMinuta, T1
	CMPEQ?BCB	_DecSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00143

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00546	          kanal3 = osvpocetak
	MOVE?BB	_OsvPocetak, _Kanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00547	          stkanal3 = osvpocetak * 100
	MUL?BCN	_OsvPocetak, 064h, _STKanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00548	       else
	GOTO?L	L00144
	LABEL?L	L00143	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00549	          if kminuta = tminuta and decsec = 0 then  
	CMPEQ?WWB	_KMinuta, _TMinuta, T1
	CMPEQ?BCB	_DecSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00145

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00550	              kanal3 = osvkraj
	MOVE?BB	_OsvKraj, _Kanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00551	          else
	GOTO?L	L00146
	LABEL?L	L00145	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00552	              if noviminut = 1 then
	CMPNE?BCL	_NoviMinut, 001h, L00147

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00553	                 stkanal3 = stkanal3 - (((osvpocetak - osvkraj)*100/(kminuta - pminuta)))  
	SUB?BBN	_OsvPocetak, _OsvKraj, T1
	MUL?NCN	T1, 064h, T1
	SUB?WWN	_KMinuta, _PMinuta, T2
	DIV?NNN	T1, T2, T2
	SUB?NNN	_STKanal3, T2, _STKanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00554	                 kanal3 = stkanal3 / 100 
	DIV?NCB	_STKanal3, 064h, _Kanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00555	              endif
	LABEL?L	L00147	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00556	          endif
	LABEL?L	L00146	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00557	       endif
	LABEL?L	L00144	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00558	    endif 
	LABEL?L	L00141	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00562	    satPocetak   = memorija[56] 
	MOVE?BB	_Memorija + 00038h, _SatPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00563	    MinutPocetak = memorija[57]  
	MOVE?BB	_Memorija + 00039h, _MinutPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00564	    osvPocetak   = memorija[58]
	MOVE?BB	_Memorija + 0003Ah, _OsvPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00565	    SatKraj      = memorija[60]  
	MOVE?BB	_Memorija + 0003Ch, _SatKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00566	    MinutKraj    = memorija[61]
	MOVE?BB	_Memorija + 0003Dh, _MinutKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00567	    OsvKraj      = memorija[62]
	MOVE?BB	_Memorija + 0003Eh, _OsvKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00569	    PMinuta = (satpocetak * 60) + minutpocetak
	MUL?BCN	_SatPocetak, 03Ch, T1
	ADD?NBW	T1, _MinutPocetak, _PMinuta

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00570	    KMinuta = (satkraj    * 60) + minutkraj
	MUL?BCN	_SatKraj, 03Ch, T1
	ADD?NBW	T1, _MinutKraj, _KMinuta

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00572	    if Pminuta =< tminuta and KMinuta >= tminuta then 
	CMPLE?WWB	_PMinuta, _TMinuta, T1
	CMPGE?WWB	_KMinuta, _TMinuta, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00149

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00573	       if Pminuta = tminuta and decsec = 0 then 
	CMPEQ?WWB	_PMinuta, _TMinuta, T1
	CMPEQ?BCB	_DecSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00151

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00574	          kanal4 = osvpocetak
	MOVE?BB	_OsvPocetak, _Kanal4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00575	          stkanal4 = osvpocetak * 100
	MUL?BCN	_OsvPocetak, 064h, _STKanal4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00576	       else
	GOTO?L	L00152
	LABEL?L	L00151	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00577	          if kminuta = tminuta and decsec = 0 then  
	CMPEQ?WWB	_KMinuta, _TMinuta, T1
	CMPEQ?BCB	_DecSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00153

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00578	              kanal4 = osvkraj
	MOVE?BB	_OsvKraj, _Kanal4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00579	          else
	GOTO?L	L00154
	LABEL?L	L00153	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00580	              if noviminut = 1 then 
	CMPNE?BCL	_NoviMinut, 001h, L00155

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00581	                 stkanal4 = stkanal4 + (((osvkraj - osvpocetak)*100/(kminuta - pminuta)))  
	SUB?BBN	_OsvKraj, _OsvPocetak, T1
	MUL?NCN	T1, 064h, T1
	SUB?WWN	_KMinuta, _PMinuta, T2
	DIV?NNN	T1, T2, T2
	ADD?NNN	_STKanal4, T2, _STKanal4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00582	                 kanal4 = stkanal4 / 100 
	DIV?NCB	_STKanal4, 064h, _Kanal4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00583	              endif
	LABEL?L	L00155	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00584	          endif
	LABEL?L	L00154	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00585	       endif
	LABEL?L	L00152	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00586	    endif 
	LABEL?L	L00149	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00588	    satPocetak   = memorija[64] 
	MOVE?BB	_Memorija + 00040h, _SatPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00589	    MinutPocetak = memorija[65]  
	MOVE?BB	_Memorija + 00041h, _MinutPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00590	    osvPocetak   = memorija[66] 
	MOVE?BB	_Memorija + 00042h, _OsvPocetak

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00591	    SatKraj      = memorija[68]  
	MOVE?BB	_Memorija + 00044h, _SatKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00592	    MinutKraj    = memorija[69]  
	MOVE?BB	_Memorija + 00045h, _MinutKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00593	    OsvKraj      = memorija[70]  
	MOVE?BB	_Memorija + 00046h, _OsvKraj

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00595	    PMinuta = (satpocetak * 60) + minutpocetak
	MUL?BCN	_SatPocetak, 03Ch, T1
	ADD?NBW	T1, _MinutPocetak, _PMinuta

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00596	    KMinuta = (satkraj    * 60) + minutkraj
	MUL?BCN	_SatKraj, 03Ch, T1
	ADD?NBW	T1, _MinutKraj, _KMinuta

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00598	if Pminuta =< tminuta and KMinuta >= tminuta then 
	CMPLE?WWB	_PMinuta, _TMinuta, T1
	CMPGE?WWB	_KMinuta, _TMinuta, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00157

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00599	       if Pminuta = tminuta and decsec = 0 then   
	CMPEQ?WWB	_PMinuta, _TMinuta, T1
	CMPEQ?BCB	_DecSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00159

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00600	          kanal4 = osvpocetak
	MOVE?BB	_OsvPocetak, _Kanal4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00601	          stkanal4 = osvpocetak * 100
	MUL?BCN	_OsvPocetak, 064h, _STKanal4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00602	       else
	GOTO?L	L00160
	LABEL?L	L00159	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00603	          if kminuta = tminuta and decsec = 0 then  
	CMPEQ?WWB	_KMinuta, _TMinuta, T1
	CMPEQ?BCB	_DecSec, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00161

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00604	              kanal4 = osvkraj
	MOVE?BB	_OsvKraj, _Kanal4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00605	          else
	GOTO?L	L00162
	LABEL?L	L00161	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00606	              if noviminut = 1 then
	CMPNE?BCL	_NoviMinut, 001h, L00163

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00607	                 stkanal4 = stkanal4 - (((osvpocetak - osvkraj)*100 /(kminuta - pminuta)))  
	SUB?BBN	_OsvPocetak, _OsvKraj, T1
	MUL?NCN	T1, 064h, T1
	SUB?WWN	_KMinuta, _PMinuta, T2
	DIV?NNN	T1, T2, T2
	SUB?NNN	_STKanal4, T2, _STKanal4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00608	                 kanal4 = stkanal4 / 100 
	DIV?NCB	_STKanal4, 064h, _Kanal4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00609	              endif
	LABEL?L	L00163	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00610	          endif
	LABEL?L	L00162	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00611	       endif
	LABEL?L	L00160	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00612	    endif 
	LABEL?L	L00157	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00614	if  pregrejan = 0 then   
	CMPNE?BCL	_pregrejan, 000h, L00165

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00615	        if odrzavanjemod = 0 then     
	CMPNE?TCL	_OdrzavanjeMod, 000h, L00167

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00616	            if AutomatskoAkvarijum  = 1 then 
	CMPNE?TCL	_AutomatskoAkvarijum, 001h, L00169

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00617	                 DutyCycleL1 =(kanal1 * 1024)/100
	MUL?BCN	_Kanal1, 000000400h, T1
	DIV?NCN	T1, 064h, _DutyCycleL1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00618	                 DutyCycleL2 =(kanal2 * 1024)/100
	MUL?BCN	_Kanal2, 000000400h, T1
	DIV?NCN	T1, 064h, _DutyCycleL2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00619	                 DutyCycleL3 =(kanal3 * 1024)/100
	MUL?BCN	_Kanal3, 000000400h, T1
	DIV?NCN	T1, 064h, _DutyCycleL3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00620	                 @ HPWM10 4, _DutyCycleL1, _Frequency

	ASM?
 HPWM10 4, _DutyCycleL1, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00621	                 @ HPWM10 3, _DutyCycleL2, _Frequency

	ASM?
 HPWM10 3, _DutyCycleL2, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00622	                 @ HPWM10 1, _DutyCycleL3, _Frequency 

	ASM?
 HPWM10 1, _DutyCycleL3, _Frequency 

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00623	            endif
	LABEL?L	L00169	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00624	            if AutomatskoSoba  = 1 then 
	CMPNE?TCL	_AutomatskoSoba, 001h, L00171

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00625	                DutyCycleL4 =(Kanal4 * 1024)/100
	MUL?BCN	_Kanal4, 000000400h, T1
	DIV?NCN	T1, 064h, _DutyCycleL4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00626	                @ HPWM10 2, _DutyCycleL4, _Frequency

	ASM?
 HPWM10 2, _DutyCycleL4, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00627	            endif
	LABEL?L	L00171	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00629	        endif
	LABEL?L	L00167	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00632	endif
	LABEL?L	L00165	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00633	if RelejiMemorija[1]  = DecSatglavni and RelejiMemorija[2] = DecMinutGlavni and decsec = 0 then 
	CMPEQ?BBB	_RelejiMemorija + 00001h, _DecSatglavni, T1
	CMPEQ?BBB	_RelejiMemorija + 00002h, _DecMinutGlavni, T2
	LAND?BBN	T1, T2, T2
	CMPEQ?BCB	_DecSec, 000h, T3
	LAND?NBN	T2, T3, T3
	CMPF?NL	T3, L00173

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00634	        relej1 = 1 
	MOVE?CT	001h, _RELEJ1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00635	endif
	LABEL?L	L00173	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00636	if RelejiMemorija[3]  = DecSatglavni and RelejiMemorija[4] = DecMinutGlavni and decsec = 0 then 
	CMPEQ?BBB	_RelejiMemorija + 00003h, _DecSatglavni, T1
	CMPEQ?BBB	_RelejiMemorija + 00004h, _DecMinutGlavni, T2
	LAND?BBN	T1, T2, T2
	CMPEQ?BCB	_DecSec, 000h, T3
	LAND?NBN	T2, T3, T3
	CMPF?NL	T3, L00175

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00637	        relej1 = 0 
	MOVE?CT	000h, _RELEJ1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00638	endif
	LABEL?L	L00175	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00641	if RelejiMemorija[5]  = DecSatglavni and RelejiMemorija[6] = DecMinutGlavni and decsec = 0 then 
	CMPEQ?BBB	_RelejiMemorija + 00005h, _DecSatglavni, T1
	CMPEQ?BBB	_RelejiMemorija + 00006h, _DecMinutGlavni, T2
	LAND?BBN	T1, T2, T2
	CMPEQ?BCB	_DecSec, 000h, T3
	LAND?NBN	T2, T3, T3
	CMPF?NL	T3, L00177

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00642	        relej1 = 1 
	MOVE?CT	001h, _RELEJ1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00643	endif
	LABEL?L	L00177	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00644	if RelejiMemorija[7]  = DecSatglavni and RelejiMemorija[8] = DecMinutGlavni and decsec = 0 then 
	CMPEQ?BBB	_RelejiMemorija + 00007h, _DecSatglavni, T1
	CMPEQ?BBB	_RelejiMemorija + 00008h, _DecMinutGlavni, T2
	LAND?BBN	T1, T2, T2
	CMPEQ?BCB	_DecSec, 000h, T3
	LAND?NBN	T2, T3, T3
	CMPF?NL	T3, L00179

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00645	        relej1 = 0 
	MOVE?CT	000h, _RELEJ1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00646	endif
	LABEL?L	L00179	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00649	if RelejiMemorija[9]  = DecSatglavni and RelejiMemorija[10] = DecMinutGlavni and decsec = 0 then 
	CMPEQ?BBB	_RelejiMemorija + 00009h, _DecSatglavni, T1
	CMPEQ?BBB	_RelejiMemorija + 0000Ah, _DecMinutGlavni, T2
	LAND?BBN	T1, T2, T2
	CMPEQ?BCB	_DecSec, 000h, T3
	LAND?NBN	T2, T3, T3
	CMPF?NL	T3, L00181

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00650	        relej2 = 1 
	MOVE?CT	001h, _RELEJ2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00651	endif
	LABEL?L	L00181	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00652	if RelejiMemorija[11]  = DecSatglavni and RelejiMemorija[12] = DecMinutGlavni and decsec = 0 then 
	CMPEQ?BBB	_RelejiMemorija + 0000Bh, _DecSatglavni, T1
	CMPEQ?BBB	_RelejiMemorija + 0000Ch, _DecMinutGlavni, T2
	LAND?BBN	T1, T2, T2
	CMPEQ?BCB	_DecSec, 000h, T3
	LAND?NBN	T2, T3, T3
	CMPF?NL	T3, L00183

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00653	        relej2 = 0 
	MOVE?CT	000h, _RELEJ2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00654	endif
	LABEL?L	L00183	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00657	if RelejiMemorija[13]  = DecSatglavni and RelejiMemorija[14] = DecMinutGlavni and decsec = 0 then 
	CMPEQ?BBB	_RelejiMemorija + 0000Dh, _DecSatglavni, T1
	CMPEQ?BBB	_RelejiMemorija + 0000Eh, _DecMinutGlavni, T2
	LAND?BBN	T1, T2, T2
	CMPEQ?BCB	_DecSec, 000h, T3
	LAND?NBN	T2, T3, T3
	CMPF?NL	T3, L00185

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00658	        relej2 = 1 
	MOVE?CT	001h, _RELEJ2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00659	endif
	LABEL?L	L00185	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00660	if RelejiMemorija[15]  = DecSatglavni and RelejiMemorija[16] = DecMinutGlavni and decsec = 0 then 
	CMPEQ?BBB	_RelejiMemorija + 0000Fh, _DecSatglavni, T1
	CMPEQ?BBB	_RelejiMemorija + 00010h, _DecMinutGlavni, T2
	LAND?BBN	T1, T2, T2
	CMPEQ?BCB	_DecSec, 000h, T3
	LAND?NBN	T2, T3, T3
	CMPF?NL	T3, L00187

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00661	        relej2 = 0 
	MOVE?CT	000h, _RELEJ2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00662	endif
	LABEL?L	L00187	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00664	    starostanjeRTC = novostanjeRTC 
	MOVE?BB	_NovostanjeRTC, _StarostanjeRTC

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00668	return
	RETURN?	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00670	GlavniMeni:

	LABEL?L	_GlavniMeni	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00671	bcklight = 1  
	MOVE?CT	001h, _BCKLIGHT

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00672	gosub kreni
	GOSUB?L	_kreni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00674	  if dole = 1 then 
	CMPNE?TCL	_dole, 001h, L00189

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00675	     if x = 3 then 
	CMPNE?BCL	_x, 003h, L00191

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00676	        x = 3
	MOVE?CB	003h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00677	     else
	GOTO?L	L00192
	LABEL?L	L00191	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00678	        x = x + 1      
	ADD?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00679	     endif
	LABEL?L	L00192	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00680	 endif
	LABEL?L	L00189	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00681	 if gore = 1 then 
	CMPNE?TCL	_gore, 001h, L00193

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00682	     if x = 1 then 
	CMPNE?BCL	_x, 001h, L00195

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00683	        x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00684	     else
	GOTO?L	L00196
	LABEL?L	L00195	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00685	        x = x - 1 
	SUB?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00686	     endif
	LABEL?L	L00196	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00687	 endif
	LABEL?L	L00193	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00688	 if levo = 1 then
	CMPNE?TCL	_levo, 001h, L00197

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00689	        beztastera = 0  
	MOVE?CW	000h, _BezTastera

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00690	        x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00691	        goto MAIN
	GOTO?L	_main

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00692	 endif
	LABEL?L	L00197	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00693	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00695	Select case x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00696	    case 1                     
	CMPNE?BCL	_x, 001h, L00201

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00697	        Lcdout $fe, 2,Strelica ,"LED-light         "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	_Strelica
	LCDOUT?C	04Ch
	LCDOUT?C	045h
	LCDOUT?C	044h
	LCDOUT?C	02Dh
	LCDOUT?C	06Ch
	LCDOUT?C	069h
	LCDOUT?C	067h
	LCDOUT?C	068h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00698	        lcdout $fe,$c0," Internal switch           "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
	LCDOUT?C	049h
	LCDOUT?C	06Eh
	LCDOUT?C	074h
	LCDOUT?C	065h
	LCDOUT?C	072h
	LCDOUT?C	06Eh
	LCDOUT?C	061h
	LCDOUT?C	06Ch
	LCDOUT?C	020h
	LCDOUT?C	073h
	LCDOUT?C	077h
	LCDOUT?C	069h
	LCDOUT?C	074h
	LCDOUT?C	063h
	LCDOUT?C	068h
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00699	        if ok = 1 then
	CMPNE?TCL	_ok, 001h, L00202

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00700	             x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00701	             goto LedPodMeni
	GOTO?L	_LedPodMeni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00702	        endif
	LABEL?L	L00202	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00703	    case 2            
	GOTO?L	L00200
	LABEL?L	L00201	
	CMPNE?BCL	_x, 002h, L00204

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00704	        Lcdout $fe, 2," LED-light        "           
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
	LCDOUT?C	04Ch
	LCDOUT?C	045h
	LCDOUT?C	044h
	LCDOUT?C	02Dh
	LCDOUT?C	06Ch
	LCDOUT?C	069h
	LCDOUT?C	067h
	LCDOUT?C	068h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00705	        lcdout $fe,$c0,Strelica ,"Internal switch   "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	_Strelica
	LCDOUT?C	049h
	LCDOUT?C	06Eh
	LCDOUT?C	074h
	LCDOUT?C	065h
	LCDOUT?C	072h
	LCDOUT?C	06Eh
	LCDOUT?C	061h
	LCDOUT?C	06Ch
	LCDOUT?C	020h
	LCDOUT?C	073h
	LCDOUT?C	077h
	LCDOUT?C	069h
	LCDOUT?C	074h
	LCDOUT?C	063h
	LCDOUT?C	068h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00706	        if ok = 1 then
	CMPNE?TCL	_ok, 001h, L00205

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00707	             X = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00708	             goto UnutrasnjiPrekidaci
	GOTO?L	_UnutrasnjiPrekidaci

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00709	        endif
	LABEL?L	L00205	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00710	    case 3                    
	GOTO?L	L00200
	LABEL?L	L00204	
	CMPNE?BCL	_x, 003h, L00207

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00711	        lcdout $fe,2,strelica,"Tools             "        
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	_Strelica
	LCDOUT?C	054h
	LCDOUT?C	06Fh
	LCDOUT?C	06Fh
	LCDOUT?C	06Ch
	LCDOUT?C	073h
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00712	        lcdout $fe,$c0,"                  "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
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
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00713	        if ok = 1 then      
	CMPNE?TCL	_ok, 001h, L00208

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00714	            x = 1 
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00715	            goto Alat
	GOTO?L	_Alat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00716	        endif       
	LABEL?L	L00208	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00717	end select
	LABEL?L	L00207	
	LABEL?L	L00200	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00718	goto glavniMeni
	GOTO?L	_GlavniMeni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00720	LedPodMeni:

	LABEL?L	_LedPodMeni	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00721	gosub kreni
	GOSUB?L	_kreni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00722	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00724	     if dole = 1 then 
	CMPNE?TCL	_dole, 001h, L00210

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00725	         if x = 2 then 
	CMPNE?BCL	_x, 002h, L00212

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00726	            x = 2
	MOVE?CB	002h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00727	         else
	GOTO?L	L00213
	LABEL?L	L00212	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00728	            x = x + 1 
	ADD?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00729	         endif
	LABEL?L	L00213	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00730	     endif
	LABEL?L	L00210	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00731	     if gore = 1 then 
	CMPNE?TCL	_gore, 001h, L00214

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00732	         if x = 1 then 
	CMPNE?BCL	_x, 001h, L00216

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00733	            x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00734	         else
	GOTO?L	L00217
	LABEL?L	L00216	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00735	            x = x - 1 
	SUB?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00736	         endif
	LABEL?L	L00217	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00737	     endif
	LABEL?L	L00214	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00738	     if levo = 1 then 
	CMPNE?TCL	_levo, 001h, L00218

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00739	        x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00740	         goto GlavniMeni
	GOTO?L	_GlavniMeni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00741	     endif
	LABEL?L	L00218	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00742	    Select case x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00743	        case 1                                    
	CMPNE?BCL	_x, 001h, L00222

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00744	            Lcdout $fe, 2,Strelica ,"Automatic          "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	_Strelica
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
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00745	            lcdout $fe,$c0," Manual              "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
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
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00746	            if ok = 1 then 
	CMPNE?TCL	_ok, 001h, L00223

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00747	                 x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00748	                 goto LedAutomaticIzbor
	GOTO?L	_LedAutomaticIzbor

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00749	            endif
	LABEL?L	L00223	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00750	        case 2          
	GOTO?L	L00221
	LABEL?L	L00222	
	CMPNE?BCL	_x, 002h, L00225

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00751	            Lcdout $fe, 2," Automatic           "           
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
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
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00752	            lcdout $fe,$c0,Strelica ,"Manual             "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	_Strelica
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00753	            if ok = 1 then        
	CMPNE?TCL	_ok, 001h, L00226

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00754	                x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00755	                 goto LedManual
	GOTO?L	_LedManual

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00756	            endif
	LABEL?L	L00226	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00757	     end select
	LABEL?L	L00225	
	LABEL?L	L00221	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00758	goto LedPodMeni
	GOTO?L	_LedPodMeni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00760	LedAutomaticIzbor:

	LABEL?L	_LedAutomaticIzbor	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00761	gosub kreni
	GOSUB?L	_kreni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00762	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00763	     if dole = 1 then 
	CMPNE?TCL	_dole, 001h, L00228

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00764	         if x = 2 then 
	CMPNE?BCL	_x, 002h, L00230

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00765	            x = 2
	MOVE?CB	002h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00766	         else
	GOTO?L	L00231
	LABEL?L	L00230	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00767	            x = x + 1 
	ADD?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00768	         endif
	LABEL?L	L00231	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00769	     endif
	LABEL?L	L00228	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00770	     if gore = 1 then 
	CMPNE?TCL	_gore, 001h, L00232

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00771	         if x = 1 then 
	CMPNE?BCL	_x, 001h, L00234

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00772	            x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00773	         else
	GOTO?L	L00235
	LABEL?L	L00234	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00774	            x = x - 1 
	SUB?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00775	         endif
	LABEL?L	L00235	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00776	     endif
	LABEL?L	L00232	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00777	     if levo = 1 then 
	CMPNE?TCL	_levo, 001h, L00236

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00778	        x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00779	         goto LedPodMeni
	GOTO?L	_LedPodMeni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00780	     endif
	LABEL?L	L00236	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00782	    Select case x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00783	        case 1                    
	CMPNE?BCL	_x, 001h, L00240

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00784	            Lcdout $fe, 2,Strelica ,"Aquarium light      "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	_Strelica
	LCDOUT?C	041h
	LCDOUT?C	071h
	LCDOUT?C	075h
	LCDOUT?C	061h
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	075h
	LCDOUT?C	06Dh
	LCDOUT?C	020h
	LCDOUT?C	06Ch
	LCDOUT?C	069h
	LCDOUT?C	067h
	LCDOUT?C	068h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00785	            lcdout $fe,$c0," Room light             "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
	LCDOUT?C	052h
	LCDOUT?C	06Fh
	LCDOUT?C	06Fh
	LCDOUT?C	06Dh
	LCDOUT?C	020h
	LCDOUT?C	06Ch
	LCDOUT?C	069h
	LCDOUT?C	067h
	LCDOUT?C	068h
	LCDOUT?C	074h
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00786	            if ok = 1 then
	CMPNE?TCL	_ok, 001h, L00241

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00787	                x = 1 
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00788	                goto AquariumLight
	GOTO?L	_AquariumLight

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00789	            endif
	LABEL?L	L00241	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00790	        case 2             
	GOTO?L	L00239
	LABEL?L	L00240	
	CMPNE?BCL	_x, 002h, L00243

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00791	            Lcdout $fe, 2," Aquarium light     "           
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
	LCDOUT?C	041h
	LCDOUT?C	071h
	LCDOUT?C	075h
	LCDOUT?C	061h
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	075h
	LCDOUT?C	06Dh
	LCDOUT?C	020h
	LCDOUT?C	06Ch
	LCDOUT?C	069h
	LCDOUT?C	067h
	LCDOUT?C	068h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00792	            lcdout $fe,$c0,Strelica ,"Room light          "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	_Strelica
	LCDOUT?C	052h
	LCDOUT?C	06Fh
	LCDOUT?C	06Fh
	LCDOUT?C	06Dh
	LCDOUT?C	020h
	LCDOUT?C	06Ch
	LCDOUT?C	069h
	LCDOUT?C	067h
	LCDOUT?C	068h
	LCDOUT?C	074h
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00793	            if ok = 1 then        
	CMPNE?TCL	_ok, 001h, L00244

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00794	                x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00795	                BrojKanala = 4
	MOVE?CB	004h, _BrojKanala

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00796	                goto AquariumLightOdabir
	GOTO?L	_AquariumLightOdabir

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00797	            endif    
	LABEL?L	L00244	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00798	    end select
	LABEL?L	L00243	
	LABEL?L	L00239	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00799	goto LedAutomaticIzbor
	GOTO?L	_LedAutomaticIzbor

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00801	AquariumLight:

	LABEL?L	_AquariumLight	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00802	gosub kreni
	GOSUB?L	_kreni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00803	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00804	     if dole = 1 then 
	CMPNE?TCL	_dole, 001h, L00246

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00805	         if x = 3 then 
	CMPNE?BCL	_x, 003h, L00248

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00806	            x = 3
	MOVE?CB	003h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00807	         else
	GOTO?L	L00249
	LABEL?L	L00248	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00808	            x = x + 1 
	ADD?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00809	         endif
	LABEL?L	L00249	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00810	     endif
	LABEL?L	L00246	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00811	     if gore = 1 then 
	CMPNE?TCL	_gore, 001h, L00250

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00812	         if x = 1 then 
	CMPNE?BCL	_x, 001h, L00252

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00813	            x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00814	         else
	GOTO?L	L00253
	LABEL?L	L00252	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00815	            x = x - 1 
	SUB?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00816	         endif
	LABEL?L	L00253	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00817	     endif
	LABEL?L	L00250	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00818	     if levo = 1 then 
	CMPNE?TCL	_levo, 001h, L00254

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00819	        x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00820	         goto LedAutomaticIzbor
	GOTO?L	_LedAutomaticIzbor

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00821	     endif
	LABEL?L	L00254	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00823	    Select case x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00824	    case 1                  
	CMPNE?BCL	_x, 001h, L00258

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00825	        Lcdout $fe, 2,strelica,"Channel #1        "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	_Strelica
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
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00826	        lcdout $fe,$c0," Channel #2        "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
	LCDOUT?C	043h
	LCDOUT?C	068h
	LCDOUT?C	061h
	LCDOUT?C	06Eh
	LCDOUT?C	06Eh
	LCDOUT?C	065h
	LCDOUT?C	06Ch
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	032h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00827	        if ok = 1 then 
	CMPNE?TCL	_ok, 001h, L00259

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00828	            BrojKanala = 1
	MOVE?CB	001h, _BrojKanala

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00829	            x = 1 
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00830	            goto AquariumLightOdabir
	GOTO?L	_AquariumLightOdabir

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00831	        endif
	LABEL?L	L00259	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00832	    case 2          
	GOTO?L	L00257
	LABEL?L	L00258	
	CMPNE?BCL	_x, 002h, L00261

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00833	        Lcdout $fe, 2," Channel #1       "           
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
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
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00834	        lcdout $fe,$c0,strelica,"Channel #2         "      
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	_Strelica
	LCDOUT?C	043h
	LCDOUT?C	068h
	LCDOUT?C	061h
	LCDOUT?C	06Eh
	LCDOUT?C	06Eh
	LCDOUT?C	065h
	LCDOUT?C	06Ch
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	032h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00835	        if ok = 1 then 
	CMPNE?TCL	_ok, 001h, L00262

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00836	            BrojKanala = 2
	MOVE?CB	002h, _BrojKanala

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00837	            x = 1 
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00838	            goto AquariumLightOdabir
	GOTO?L	_AquariumLightOdabir

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00839	        endif
	LABEL?L	L00262	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00840	    case 3                    
	GOTO?L	L00257
	LABEL?L	L00261	
	CMPNE?BCL	_x, 003h, L00264

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00841	        Lcdout $fe, 2,strelica,"Channel #3         "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	_Strelica
	LCDOUT?C	043h
	LCDOUT?C	068h
	LCDOUT?C	061h
	LCDOUT?C	06Eh
	LCDOUT?C	06Eh
	LCDOUT?C	065h
	LCDOUT?C	06Ch
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	033h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00842	        lcdout $fe,$c0,"                 "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
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
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00843	        if ok = 1 then
	CMPNE?TCL	_ok, 001h, L00265

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00844	            BrojKanala = 3 
	MOVE?CB	003h, _BrojKanala

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00845	            x = 1 
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00846	            goto AquariumLightOdabir
	GOTO?L	_AquariumLightOdabir

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00847	        endif
	LABEL?L	L00265	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00848	   end select 
	LABEL?L	L00264	
	LABEL?L	L00257	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00849	goto AquariumLight
	GOTO?L	_AquariumLight

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00851	AquariumLightOdabir:

	LABEL?L	_AquariumLightOdabir	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00852	gosub kreni
	GOSUB?L	_kreni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00853	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00855	     if dole = 1 then 
	CMPNE?TCL	_dole, 001h, L00267

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00856	         if x = 2 then 
	CMPNE?BCL	_x, 002h, L00269

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00857	            x = 2
	MOVE?CB	002h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00858	         else
	GOTO?L	L00270
	LABEL?L	L00269	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00859	            x = x + 1 
	ADD?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00860	         endif
	LABEL?L	L00270	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00861	     endif
	LABEL?L	L00267	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00863	     if gore = 1 then 
	CMPNE?TCL	_gore, 001h, L00271

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00864	         if x = 1 then 
	CMPNE?BCL	_x, 001h, L00273

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00865	            x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00866	         else
	GOTO?L	L00274
	LABEL?L	L00273	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00867	            x = x - 1 
	SUB?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00868	         endif
	LABEL?L	L00274	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00869	     endif
	LABEL?L	L00271	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00871	     if levo = 1 then 
	CMPNE?TCL	_levo, 001h, L00275

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00872	        if brojkanala = 4 then
	CMPNE?BCL	_BrojKanala, 004h, L00277

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00873	            x = 2 
	MOVE?CB	002h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00874	            goto LedAutomaticIzbor
	GOTO?L	_LedAutomaticIzbor

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00875	        else
	GOTO?L	L00278
	LABEL?L	L00277	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00876	            x = brojkanala
	MOVE?BB	_BrojKanala, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00877	            goto AquariumLight
	GOTO?L	_AquariumLight

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00878	        endif
	LABEL?L	L00278	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00879	     endif
	LABEL?L	L00275	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00880	Select case x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00881	        case 1                                  
	CMPNE?BCL	_x, 001h, L00281

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00882	            Lcdout $fe, 2,strelica,"Sunrise          "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	_Strelica
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00883	            lcdout $fe,$c0," Sunset           "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00884	        case 2       
	GOTO?L	L00280
	LABEL?L	L00281	
	CMPNE?BCL	_x, 002h, L00282

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00885	            Lcdout $fe, 2," Sunrise                  "           
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	073h
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
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00886	            lcdout $fe,$c0,strelica,"Sunset             "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	_Strelica
	LCDOUT?C	053h
	LCDOUT?C	075h
	LCDOUT?C	06Eh
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00887	end select                                             
	LABEL?L	L00282	
	LABEL?L	L00280	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00888	if ok = 1 then
	CMPNE?TCL	_ok, 001h, L00283

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00889	if x = 1 then ModDanNoc = 0 
	CMPNE?BCL	_x, 001h, L00285
	MOVE?CT	000h, _ModDanNoc
	LABEL?L	L00285	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00890	if x = 2 then ModDanNoc = 1  
	CMPNE?BCL	_x, 002h, L00287
	MOVE?CT	001h, _ModDanNoc
	LABEL?L	L00287	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00891	   if brojkanala = 1 and moddannoc  = 0 then 
	CMPEQ?BCB	_BrojKanala, 001h, T1
	CMPEQ?TCB	_ModDanNoc, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00289

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00892	        read 8,decsat
	READADDRESS?C	008h
	READ?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00893	        read 9,decmin
	READADDRESS?C	009h
	READ?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00894	        read 10,JacinaOsvetljenja1
	READADDRESS?C	00Ah
	READ?B	_JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00896	        read 12,decsat1
	READADDRESS?C	00Ch
	READ?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00897	        read 13,decmin1
	READADDRESS?C	00Dh
	READ?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00898	        read 14,JacinaOsvetljenja2
	READADDRESS?C	00Eh
	READ?B	_JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00899	    endif
	LABEL?L	L00289	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00900	    if brojkanala = 1 and moddannoc  = 1 then
	CMPEQ?BCB	_BrojKanala, 001h, T1
	CMPEQ?TCB	_ModDanNoc, 001h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00291

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00901	        read 16,decsat
	READADDRESS?C	010h
	READ?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00902	        read 17,decmin
	READADDRESS?C	011h
	READ?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00903	        read 18,JacinaOsvetljenja1
	READADDRESS?C	012h
	READ?B	_JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00905	        read 20,decsat1
	READADDRESS?C	014h
	READ?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00906	        read 21,decmin1
	READADDRESS?C	015h
	READ?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00907	        read 22,JacinaOsvetljenja2
	READADDRESS?C	016h
	READ?B	_JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00908	    endif 
	LABEL?L	L00291	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00909	    if brojkanala = 2 and moddannoc  = 0 then 
	CMPEQ?BCB	_BrojKanala, 002h, T1
	CMPEQ?TCB	_ModDanNoc, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00293

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00910	        read 24,decsat
	READADDRESS?C	018h
	READ?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00911	        read 25,decmin
	READADDRESS?C	019h
	READ?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00912	        read 26,JacinaOsvetljenja1
	READADDRESS?C	01Ah
	READ?B	_JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00914	        read 28,decsat1
	READADDRESS?C	01Ch
	READ?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00915	        read 29,decmin1
	READADDRESS?C	01Dh
	READ?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00916	        read 30,JacinaOsvetljenja2
	READADDRESS?C	01Eh
	READ?B	_JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00917	    endif 
	LABEL?L	L00293	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00918	    if brojkanala = 2 and moddannoc  = 1 then 
	CMPEQ?BCB	_BrojKanala, 002h, T1
	CMPEQ?TCB	_ModDanNoc, 001h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00295

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00919	        read 32,decsat
	READADDRESS?C	020h
	READ?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00920	        read 33,decmin
	READADDRESS?C	021h
	READ?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00921	        read 34,JacinaOsvetljenja1
	READADDRESS?C	022h
	READ?B	_JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00923	        read 36,decsat1
	READADDRESS?C	024h
	READ?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00924	        read 37,decmin1
	READADDRESS?C	025h
	READ?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00925	        read 38,JacinaOsvetljenja2
	READADDRESS?C	026h
	READ?B	_JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00926	    endif
	LABEL?L	L00295	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00927	    if brojkanala = 3 and moddannoc  = 0 then  
	CMPEQ?BCB	_BrojKanala, 003h, T1
	CMPEQ?TCB	_ModDanNoc, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00297

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00928	        read 40,decsat
	READADDRESS?C	028h
	READ?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00929	        read 41,decmin
	READADDRESS?C	029h
	READ?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00930	        read 42,JacinaOsvetljenja1
	READADDRESS?C	02Ah
	READ?B	_JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00932	        read 44,decsat1
	READADDRESS?C	02Ch
	READ?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00933	        read 45,decmin1
	READADDRESS?C	02Dh
	READ?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00934	        read 46,JacinaOsvetljenja2
	READADDRESS?C	02Eh
	READ?B	_JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00935	    endif
	LABEL?L	L00297	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00936	    if brojkanala = 3 and moddannoc  = 1 then 
	CMPEQ?BCB	_BrojKanala, 003h, T1
	CMPEQ?TCB	_ModDanNoc, 001h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00299

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00937	        read 48,decsat
	READADDRESS?C	030h
	READ?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00938	        read 49,decmin
	READADDRESS?C	031h
	READ?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00939	        read 50,JacinaOsvetljenja1
	READADDRESS?C	032h
	READ?B	_JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00941	        read 52,decsat1
	READADDRESS?C	034h
	READ?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00942	        read 53,decmin1
	READADDRESS?C	035h
	READ?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00943	        read 54,JacinaOsvetljenja2
	READADDRESS?C	036h
	READ?B	_JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00944	    endif 
	LABEL?L	L00299	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00945	    if brojkanala = 4 and moddannoc  = 0 then 
	CMPEQ?BCB	_BrojKanala, 004h, T1
	CMPEQ?TCB	_ModDanNoc, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00301

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00946	        read 56,decsat
	READADDRESS?C	038h
	READ?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00947	        read 57,decmin
	READADDRESS?C	039h
	READ?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00948	        read 58,JacinaOsvetljenja1
	READADDRESS?C	03Ah
	READ?B	_JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00950	        read 60,decsat1
	READADDRESS?C	03Ch
	READ?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00951	        read 61,decmin1
	READADDRESS?C	03Dh
	READ?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00952	        read 62,JacinaOsvetljenja2
	READADDRESS?C	03Eh
	READ?B	_JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00953	    endif 
	LABEL?L	L00301	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00954	    if brojkanala = 4 and moddannoc  = 1 then 
	CMPEQ?BCB	_BrojKanala, 004h, T1
	CMPEQ?TCB	_ModDanNoc, 001h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00303

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00955	        read 64,decsat
	READADDRESS?C	040h
	READ?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00956	        read 65,decmin
	READADDRESS?C	041h
	READ?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00957	        read 66,JacinaOsvetljenja1
	READADDRESS?C	042h
	READ?B	_JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00959	        read 68,decsat1
	READADDRESS?C	044h
	READ?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00960	        read 69,decmin1
	READADDRESS?C	045h
	READ?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00961	        read 70,JacinaOsvetljenja2
	READADDRESS?C	046h
	READ?B	_JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00962	    endif 
	LABEL?L	L00303	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00964	    x = 1 
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00965	    goto AquariumLightAutomatic 
	GOTO?L	_AquariumLightAutomatic

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00966	endif
	LABEL?L	L00283	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00967	goto AquariumLightOdabir
	GOTO?L	_AquariumLightOdabir

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00969	AquariumLightAutomatic:

	LABEL?L	_AquariumLightAutomatic	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00970	gosub kreni
	GOSUB?L	_kreni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00971	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00972	    if desno = 1 then 
	CMPNE?TCL	_desno, 001h, L00305

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00973	         if x = 6  then 
	CMPNE?BCL	_x, 006h, L00307

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00974	            x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00975	         else
	GOTO?L	L00308
	LABEL?L	L00307	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00976	            x = x + 1 
	ADD?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00977	         endif
	LABEL?L	L00308	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00978	    endif
	LABEL?L	L00305	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00979	    IF x = 1 THEN  
	CMPNE?BCL	_x, 001h, L00309

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00980	        displej[1] = strelica
	MOVE?CB	_Strelica, _displej + 00001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00981	        displej[2] = " "
	MOVE?CB	020h, _displej + 00002h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00982	        displej[3] = " "
	MOVE?CB	020h, _displej + 00003h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00983	        displej[4] = " "
	MOVE?CB	020h, _displej + 00004h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00984	        displej[5] = " "
	MOVE?CB	020h, _displej + 00005h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00985	        displej[6] = " "
	MOVE?CB	020h, _displej + 00006h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00986	     ENDIF
	LABEL?L	L00309	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00987	     IF x = 2 THEN
	CMPNE?BCL	_x, 002h, L00311

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00988	        displej[1] = " "
	MOVE?CB	020h, _displej + 00001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00989	        displej[2] = strelica
	MOVE?CB	_Strelica, _displej + 00002h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00990	        displej[3] = " "
	MOVE?CB	020h, _displej + 00003h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00991	        displej[4] = " "
	MOVE?CB	020h, _displej + 00004h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00992	        displej[5] = " "
	MOVE?CB	020h, _displej + 00005h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00993	        displej[6] = " "
	MOVE?CB	020h, _displej + 00006h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00994	     endif 
	LABEL?L	L00311	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00995	     if x = 3 then
	CMPNE?BCL	_x, 003h, L00313

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00996	        displej[1] = " "
	MOVE?CB	020h, _displej + 00001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00997	        displej[2] = " "
	MOVE?CB	020h, _displej + 00002h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00998	        displej[3] = strelica
	MOVE?CB	_Strelica, _displej + 00003h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	00999	        displej[4] = " "
	MOVE?CB	020h, _displej + 00004h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01000	        displej[5] = " "
	MOVE?CB	020h, _displej + 00005h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01001	        displej[6] = " "
	MOVE?CB	020h, _displej + 00006h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01002	     endif
	LABEL?L	L00313	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01003	     if x = 4 then
	CMPNE?BCL	_x, 004h, L00315

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01004	        displej[1] = " "
	MOVE?CB	020h, _displej + 00001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01005	        displej[2] = " "
	MOVE?CB	020h, _displej + 00002h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01006	        displej[3] = " "
	MOVE?CB	020h, _displej + 00003h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01007	        displej[4] = strelica
	MOVE?CB	_Strelica, _displej + 00004h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01008	        displej[5] = " "
	MOVE?CB	020h, _displej + 00005h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01009	        displej[6] = " "
	MOVE?CB	020h, _displej + 00006h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01010	     endif
	LABEL?L	L00315	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01011	     if x = 5 then
	CMPNE?BCL	_x, 005h, L00317

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01012	        displej[1] = " "
	MOVE?CB	020h, _displej + 00001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01013	        displej[2] = " "
	MOVE?CB	020h, _displej + 00002h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01014	        displej[3] = " "
	MOVE?CB	020h, _displej + 00003h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01015	        displej[4] = " " 
	MOVE?CB	020h, _displej + 00004h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01016	        displej[5] = strelica
	MOVE?CB	_Strelica, _displej + 00005h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01017	        displej[6] = " "
	MOVE?CB	020h, _displej + 00006h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01019	     endif
	LABEL?L	L00317	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01020	     if x = 6 then
	CMPNE?BCL	_x, 006h, L00319

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01021	        displej[1] = " "
	MOVE?CB	020h, _displej + 00001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01022	        displej[2] = " "
	MOVE?CB	020h, _displej + 00002h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01023	        displej[3] = " "
	MOVE?CB	020h, _displej + 00003h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01024	        displej[4] = " " 
	MOVE?CB	020h, _displej + 00004h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01025	        displej[5] = " "
	MOVE?CB	020h, _displej + 00005h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01026	        displej[6] = strelica
	MOVE?CB	_Strelica, _displej + 00006h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01027	     endif
	LABEL?L	L00319	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01029	     if levo = 1 then
	CMPNE?TCL	_levo, 001h, L00321

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01030	        if ModDanNoc = 0 then 
	CMPNE?TCL	_ModDanNoc, 000h, L00323

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01031	            x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01032	        else
	GOTO?L	L00324
	LABEL?L	L00323	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01033	            x = 2 
	MOVE?CB	002h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01034	        endif 
	LABEL?L	L00324	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01035	        goto AquariumLightOdabir
	GOTO?L	_AquariumLightOdabir

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01036	     endif
	LABEL?L	L00321	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01039	      if tastgore = 0 then
	CMPNE?TCL	_TASTGORE, 000h, L00325

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01040	         if x = 1 then
	CMPNE?BCL	_x, 001h, L00327

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01041	            decsaT = decsat + 1
	ADD?BCB	_DecSat, 001h, _DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01042	            if decsat < 1 or decsat > 23 then decsat = 23
	CMPLT?BCB	_DecSat, 001h, T1
	CMPGT?BCB	_DecSat, 017h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00329
	MOVE?CB	017h, _DecSat
	LABEL?L	L00329	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01043	            pause PauzaZaTaster
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01044	         endif
	LABEL?L	L00327	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01046	         if x = 2 then
	CMPNE?BCL	_x, 002h, L00331

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01047	            decmin = decmin + 1
	ADD?BCB	_DecMin, 001h, _DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01048	            if decmin < 1 or decmin > 59 then decmin = 59   
	CMPLT?BCB	_DecMin, 001h, T1
	CMPGT?BCB	_DecMin, 03Bh, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00333
	MOVE?CB	03Bh, _DecMin
	LABEL?L	L00333	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01049	            pause PauzaZaTaster
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01050	         endif
	LABEL?L	L00331	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01052	         if x = 3 then
	CMPNE?BCL	_x, 003h, L00335

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01053	            JacinaOsvetljenja1 = JacinaOsvetljenja1 + 1
	ADD?BCB	_JacinaOsvetljenja1, 001h, _JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01054	            if JacinaOsvetljenja1 < 1 or JacinaOsvetljenja1 > 100 then JacinaOsvetljenja1 = 100   
	CMPLT?BCB	_JacinaOsvetljenja1, 001h, T1
	CMPGT?BCB	_JacinaOsvetljenja1, 064h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00337
	MOVE?CB	064h, _JacinaOsvetljenja1
	LABEL?L	L00337	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01055	            pause PauzaZaTaster
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01056	         endif
	LABEL?L	L00335	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01058	         if x = 4 then
	CMPNE?BCL	_x, 004h, L00339

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01059	            decsaT1 = decsat1 + 1
	ADD?BCB	_DecSat1, 001h, _DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01060	            if decsat1 < 1 or decsat1 > 23 then decsat1 = 23
	CMPLT?BCB	_DecSat1, 001h, T1
	CMPGT?BCB	_DecSat1, 017h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00341
	MOVE?CB	017h, _DecSat1
	LABEL?L	L00341	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01061	            pause PauzaZaTaster
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01062	         endif
	LABEL?L	L00339	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01064	         if x = 5 then
	CMPNE?BCL	_x, 005h, L00343

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01065	            decmin1 = decmin1 + 1
	ADD?BCB	_DecMin1, 001h, _DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01066	            if decmin1 < 1 or decmin1 > 59 then decmin1 = 59   
	CMPLT?BCB	_DecMin1, 001h, T1
	CMPGT?BCB	_DecMin1, 03Bh, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00345
	MOVE?CB	03Bh, _DecMin1
	LABEL?L	L00345	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01067	            pause PauzaZaTaster
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01068	         endif
	LABEL?L	L00343	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01070	         if x = 6 then
	CMPNE?BCL	_x, 006h, L00347

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01071	            JacinaOsvetljenja2 = JacinaOsvetljenja2 + 1
	ADD?BCB	_JacinaOsvetljenja2, 001h, _JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01072	            if JacinaOsvetljenja2 < 1 or JacinaOsvetljenja2 > 100 then JacinaOsvetljenja2 = 100  
	CMPLT?BCB	_JacinaOsvetljenja2, 001h, T1
	CMPGT?BCB	_JacinaOsvetljenja2, 064h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00349
	MOVE?CB	064h, _JacinaOsvetljenja2
	LABEL?L	L00349	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01073	            pause PauzaZaTaster
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01074	         endif
	LABEL?L	L00347	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01075	     endif
	LABEL?L	L00325	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01077	     if tastdole = 0 then
	CMPNE?TCL	_TASTDOLE, 000h, L00351

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01078	         if x = 1 then
	CMPNE?BCL	_x, 001h, L00353

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01079	            decsaT = decsat - 1
	SUB?BCB	_DecSat, 001h, _DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01080	            if decsat < 1 or decsat > 23 then decsat = 0
	CMPLT?BCB	_DecSat, 001h, T1
	CMPGT?BCB	_DecSat, 017h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00355
	MOVE?CB	000h, _DecSat
	LABEL?L	L00355	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01081	            pause PauzaZaTaster
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01082	         endif
	LABEL?L	L00353	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01084	         if x = 2 then
	CMPNE?BCL	_x, 002h, L00357

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01085	            decmin = decmin - 1
	SUB?BCB	_DecMin, 001h, _DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01086	            if decmin < 1 or decmin > 59 then decmin = 0   
	CMPLT?BCB	_DecMin, 001h, T1
	CMPGT?BCB	_DecMin, 03Bh, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00359
	MOVE?CB	000h, _DecMin
	LABEL?L	L00359	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01087	            pause PauzaZaTaster
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01088	         endif
	LABEL?L	L00357	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01090	         if x = 3 then
	CMPNE?BCL	_x, 003h, L00361

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01091	            JacinaOsvetljenja1 = JacinaOsvetljenja1 - 1
	SUB?BCB	_JacinaOsvetljenja1, 001h, _JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01092	            if JacinaOsvetljenja1 < 1 or JacinaOsvetljenja1 > 100 then JacinaOsvetljenja1 = 0   
	CMPLT?BCB	_JacinaOsvetljenja1, 001h, T1
	CMPGT?BCB	_JacinaOsvetljenja1, 064h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00363
	MOVE?CB	000h, _JacinaOsvetljenja1
	LABEL?L	L00363	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01093	            pause PauzaZaTaster
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01094	         endif
	LABEL?L	L00361	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01096	         if x = 4 then
	CMPNE?BCL	_x, 004h, L00365

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01097	            decsaT1 = decsat1 - 1
	SUB?BCB	_DecSat1, 001h, _DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01098	            if decsat1 < 1 or decsat1 > 23 then decsat1 = 0
	CMPLT?BCB	_DecSat1, 001h, T1
	CMPGT?BCB	_DecSat1, 017h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00367
	MOVE?CB	000h, _DecSat1
	LABEL?L	L00367	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01099	            pause PauzaZaTaster
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01100	         endif
	LABEL?L	L00365	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01102	         if x = 5 then
	CMPNE?BCL	_x, 005h, L00369

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01103	            decmin1 = decmin1 - 1
	SUB?BCB	_DecMin1, 001h, _DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01104	            if decmin1 < 1 or decmin1 > 59 then decmin1 = 0   
	CMPLT?BCB	_DecMin1, 001h, T1
	CMPGT?BCB	_DecMin1, 03Bh, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00371
	MOVE?CB	000h, _DecMin1
	LABEL?L	L00371	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01105	            pause PauzaZaTaster
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01106	         endif
	LABEL?L	L00369	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01108	         if x = 6 then
	CMPNE?BCL	_x, 006h, L00373

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01109	            JacinaOsvetljenja2 = JacinaOsvetljenja2 - 1
	SUB?BCB	_JacinaOsvetljenja2, 001h, _JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01110	            if JacinaOsvetljenja2 < 1 or JacinaOsvetljenja2 > 100 then JacinaOsvetljenja2 = 0  
	CMPLT?BCB	_JacinaOsvetljenja2, 001h, T1
	CMPGT?BCB	_JacinaOsvetljenja2, 064h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00375
	MOVE?CB	000h, _JacinaOsvetljenja2
	LABEL?L	L00375	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01111	            pause PauzaZaTaster
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01112	         endif
	LABEL?L	L00373	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01113	     endif
	LABEL?L	L00351	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01115	if ok = 1 then  
	CMPNE?TCL	_ok, 001h, L00377

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01116	    if brojkanala = 1 and moddannoc  = 0 then 
	CMPEQ?BCB	_BrojKanala, 001h, T1
	CMPEQ?TCB	_ModDanNoc, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00379

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01117	        write 8,decsat
	WRITEADDRESS?C	008h
	WRITE?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01118	        write 9,decmin
	WRITEADDRESS?C	009h
	WRITE?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01119	        write 10,JacinaOsvetljenja1
	WRITEADDRESS?C	00Ah
	WRITE?B	_JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01121	        write 12,decsat1
	WRITEADDRESS?C	00Ch
	WRITE?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01122	        write 13,decmin1
	WRITEADDRESS?C	00Dh
	WRITE?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01123	        write 14,JacinaOsvetljenja2
	WRITEADDRESS?C	00Eh
	WRITE?B	_JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01124	    endif
	LABEL?L	L00379	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01125	    if brojkanala = 1 and moddannoc  = 1 then
	CMPEQ?BCB	_BrojKanala, 001h, T1
	CMPEQ?TCB	_ModDanNoc, 001h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00381

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01126	        write 16,decsat
	WRITEADDRESS?C	010h
	WRITE?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01127	        write 17,decmin
	WRITEADDRESS?C	011h
	WRITE?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01128	        write 18,JacinaOsvetljenja1
	WRITEADDRESS?C	012h
	WRITE?B	_JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01130	        write 20,decsat1
	WRITEADDRESS?C	014h
	WRITE?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01131	        write 21,decmin1
	WRITEADDRESS?C	015h
	WRITE?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01132	        write 22,JacinaOsvetljenja2
	WRITEADDRESS?C	016h
	WRITE?B	_JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01133	    endif 
	LABEL?L	L00381	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01135	    if brojkanala = 2 and moddannoc  = 0 then 
	CMPEQ?BCB	_BrojKanala, 002h, T1
	CMPEQ?TCB	_ModDanNoc, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00383

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01136	        write 24,decsat
	WRITEADDRESS?C	018h
	WRITE?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01137	        write 25,decmin
	WRITEADDRESS?C	019h
	WRITE?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01138	        write 26,JacinaOsvetljenja1
	WRITEADDRESS?C	01Ah
	WRITE?B	_JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01140	        write 28,decsat1
	WRITEADDRESS?C	01Ch
	WRITE?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01141	        write 29,decmin1
	WRITEADDRESS?C	01Dh
	WRITE?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01142	        write 30,JacinaOsvetljenja2
	WRITEADDRESS?C	01Eh
	WRITE?B	_JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01143	    endif 
	LABEL?L	L00383	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01144	    if brojkanala = 2 and moddannoc  = 1 then 
	CMPEQ?BCB	_BrojKanala, 002h, T1
	CMPEQ?TCB	_ModDanNoc, 001h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00385

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01145	        write 32,decsat
	WRITEADDRESS?C	020h
	WRITE?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01146	        write 33,decmin
	WRITEADDRESS?C	021h
	WRITE?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01147	        write 34,JacinaOsvetljenja1
	WRITEADDRESS?C	022h
	WRITE?B	_JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01149	        write 36,decsat1
	WRITEADDRESS?C	024h
	WRITE?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01150	        write 37,decmin1
	WRITEADDRESS?C	025h
	WRITE?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01151	        write 38,JacinaOsvetljenja2
	WRITEADDRESS?C	026h
	WRITE?B	_JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01152	    endif
	LABEL?L	L00385	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01154	    if brojkanala = 3 and moddannoc  = 0 then  
	CMPEQ?BCB	_BrojKanala, 003h, T1
	CMPEQ?TCB	_ModDanNoc, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00387

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01155	        write 40,decsat
	WRITEADDRESS?C	028h
	WRITE?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01156	        write 41,decmin
	WRITEADDRESS?C	029h
	WRITE?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01157	        write 42,JacinaOsvetljenja1
	WRITEADDRESS?C	02Ah
	WRITE?B	_JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01159	        write 44,decsat1
	WRITEADDRESS?C	02Ch
	WRITE?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01160	        write 45,decmin1
	WRITEADDRESS?C	02Dh
	WRITE?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01161	        write 46,JacinaOsvetljenja2
	WRITEADDRESS?C	02Eh
	WRITE?B	_JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01162	    endif
	LABEL?L	L00387	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01163	    if brojkanala = 3 and moddannoc  = 1 then 
	CMPEQ?BCB	_BrojKanala, 003h, T1
	CMPEQ?TCB	_ModDanNoc, 001h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00389

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01164	        write 48,decsat
	WRITEADDRESS?C	030h
	WRITE?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01165	        write 49,decmin
	WRITEADDRESS?C	031h
	WRITE?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01166	        write 50,JacinaOsvetljenja1
	WRITEADDRESS?C	032h
	WRITE?B	_JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01168	        write 52,decsat1
	WRITEADDRESS?C	034h
	WRITE?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01169	        write 53,decmin1
	WRITEADDRESS?C	035h
	WRITE?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01170	        write 54,JacinaOsvetljenja2
	WRITEADDRESS?C	036h
	WRITE?B	_JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01171	    endif 
	LABEL?L	L00389	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01173	    if brojkanala = 4 and moddannoc  = 0 then 
	CMPEQ?BCB	_BrojKanala, 004h, T1
	CMPEQ?TCB	_ModDanNoc, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00391

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01174	        write 56,decsat
	WRITEADDRESS?C	038h
	WRITE?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01175	        write 57,decmin
	WRITEADDRESS?C	039h
	WRITE?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01176	        write 58,JacinaOsvetljenja1
	WRITEADDRESS?C	03Ah
	WRITE?B	_JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01178	        write 60,decsat1
	WRITEADDRESS?C	03Ch
	WRITE?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01179	        write 61,decmin1
	WRITEADDRESS?C	03Dh
	WRITE?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01180	        write 62,JacinaOsvetljenja2
	WRITEADDRESS?C	03Eh
	WRITE?B	_JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01181	    endif 
	LABEL?L	L00391	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01182	    if brojkanala = 4 and moddannoc  = 1 then 
	CMPEQ?BCB	_BrojKanala, 004h, T1
	CMPEQ?TCB	_ModDanNoc, 001h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00393

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01183	        write 64,decsat
	WRITEADDRESS?C	040h
	WRITE?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01184	        write 65,decmin
	WRITEADDRESS?C	041h
	WRITE?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01185	        write 66,JacinaOsvetljenja1
	WRITEADDRESS?C	042h
	WRITE?B	_JacinaOsvetljenja1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01187	        write 68,decsat1
	WRITEADDRESS?C	044h
	WRITE?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01188	        write 69,decmin1
	WRITEADDRESS?C	045h
	WRITE?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01189	        write 70,JacinaOsvetljenja2
	WRITEADDRESS?C	046h
	WRITE?B	_JacinaOsvetljenja2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01190	    endif 
	LABEL?L	L00393	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01191	     gosub EEpromMemorija
	GOSUB?L	_EEpromMemorija

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01192	     gosub sacuvano
	GOSUB?L	_sacuvano

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01194	     if ModDanNoc = 0 then 
	CMPNE?TCL	_ModDanNoc, 000h, L00395

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01195	            x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01196	     else
	GOTO?L	L00396
	LABEL?L	L00395	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01197	            x = 2 
	MOVE?CB	002h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01198	     endif 
	LABEL?L	L00396	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01199	goto  AquariumLightOdabir
	GOTO?L	_AquariumLightOdabir

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01201	endif    
	LABEL?L	L00377	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01203	    Lcdout $fe, 2, "S:",displej[1],dec2 decsat,":",displej[2],dec2 decmin,"-",displej[3],dec3 JacinaOsvetljenja1,"%    "  
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	053h
	LCDOUT?C	03Ah
	LCDOUT?B	_displej + 00001h
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_DecSat
	LCDOUTDEC?	
	LCDOUT?C	03Ah
	LCDOUT?B	_displej + 00002h
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_DecMin
	LCDOUTDEC?	
	LCDOUT?C	02Dh
	LCDOUT?B	_displej + 00003h
	LCDOUTCOUNT?C	003h
	LCDOUTNUM?B	_JacinaOsvetljenja1
	LCDOUTDEC?	
	LCDOUT?C	025h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01204	    lcdout $fe,$c0,"E:",displej[4],dec2 DecSat1,":",displej[5],dec2 DecMin1,"-",displej[6],dec3 JacinaOsvetljenja2,"%  "  
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	045h
	LCDOUT?C	03Ah
	LCDOUT?B	_displej + 00004h
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_DecSat1
	LCDOUTDEC?	
	LCDOUT?C	03Ah
	LCDOUT?B	_displej + 00005h
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_DecMin1
	LCDOUTDEC?	
	LCDOUT?C	02Dh
	LCDOUT?B	_displej + 00006h
	LCDOUTCOUNT?C	003h
	LCDOUTNUM?B	_JacinaOsvetljenja2
	LCDOUTDEC?	
	LCDOUT?C	025h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01205	 goto AquariumLightAutomatic
	GOTO?L	_AquariumLightAutomatic

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01207	UnutrasnjiPrekidaci:

	LABEL?L	_UnutrasnjiPrekidaci	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01208	gosub kreni
	GOSUB?L	_kreni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01209	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01210	     if dole = 1 then 
	CMPNE?TCL	_dole, 001h, L00397

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01211	         if x = 2 then 
	CMPNE?BCL	_x, 002h, L00399

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01212	            x = 2
	MOVE?CB	002h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01213	         else
	GOTO?L	L00400
	LABEL?L	L00399	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01214	            x = x + 1 
	ADD?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01215	         endif
	LABEL?L	L00400	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01216	     endif
	LABEL?L	L00397	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01217	     if gore = 1 then 
	CMPNE?TCL	_gore, 001h, L00401

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01218	         if x = 1 then 
	CMPNE?BCL	_x, 001h, L00403

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01219	            x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01220	         else
	GOTO?L	L00404
	LABEL?L	L00403	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01221	            x = x - 1 
	SUB?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01222	         endif
	LABEL?L	L00404	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01223	     endif
	LABEL?L	L00401	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01224	     if levo = 1 then 
	CMPNE?TCL	_levo, 001h, L00405

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01225	        x = 2
	MOVE?CB	002h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01226	         goto GlavniMeni
	GOTO?L	_GlavniMeni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01227	     endif
	LABEL?L	L00405	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01228	Select case x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01229	        case 1 
	CMPNE?BCL	_x, 001h, L00409

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01230	            Lcdout $fe, 2,Strelica ,"Switch #1          "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	_Strelica
	LCDOUT?C	053h
	LCDOUT?C	077h
	LCDOUT?C	069h
	LCDOUT?C	074h
	LCDOUT?C	063h
	LCDOUT?C	068h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	031h
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01231	            lcdout $fe,$c0," Switch #2                 "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	077h
	LCDOUT?C	069h
	LCDOUT?C	074h
	LCDOUT?C	063h
	LCDOUT?C	068h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	032h
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
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01232	            if ok = 1 then
	CMPNE?TCL	_ok, 001h, L00410

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01233	                Prekidac = 0
	MOVE?CT	000h, _Prekidac

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01234	                Pozicija = 1
	MOVE?CB	001h, _Pozicija

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01235	                x = 1 
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01236	                goto UnutrasnjiPrekidacOdabir
	GOTO?L	_UnutrasnjiPrekidacOdabir

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01237	            endif
	LABEL?L	L00410	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01238	        case 2 
	GOTO?L	L00408
	LABEL?L	L00409	
	CMPNE?BCL	_x, 002h, L00412

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01239	            Lcdout $fe, 2," Switch #1         "            
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	077h
	LCDOUT?C	069h
	LCDOUT?C	074h
	LCDOUT?C	063h
	LCDOUT?C	068h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	031h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01240	            lcdout $fe,$c0,Strelica ,"Switch #2         "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	_Strelica
	LCDOUT?C	053h
	LCDOUT?C	077h
	LCDOUT?C	069h
	LCDOUT?C	074h
	LCDOUT?C	063h
	LCDOUT?C	068h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	032h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01241	            if ok  = 1 then
	CMPNE?TCL	_ok, 001h, L00413

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01242	                Prekidac = 1 
	MOVE?CT	001h, _Prekidac

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01243	                Pozicija = 2
	MOVE?CB	002h, _Pozicija

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01244	                x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01245	                goto UnutrasnjiPrekidacOdabir
	GOTO?L	_UnutrasnjiPrekidacOdabir

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01246	            endif
	LABEL?L	L00413	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01247	end select
	LABEL?L	L00412	
	LABEL?L	L00408	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01248	goto UnutrasnjiPrekidaci
	GOTO?L	_UnutrasnjiPrekidaci

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01250	UnutrasnjiPrekidacOdabir:

	LABEL?L	_UnutrasnjiPrekidacOdabir	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01251	gosub kreni
	GOSUB?L	_kreni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01252	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01253	     if dole = 1 then 
	CMPNE?TCL	_dole, 001h, L00415

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01254	         if x = 2 then 
	CMPNE?BCL	_x, 002h, L00417

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01255	            x = 2
	MOVE?CB	002h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01256	         else
	GOTO?L	L00418
	LABEL?L	L00417	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01257	            x = x + 1 
	ADD?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01258	         endif
	LABEL?L	L00418	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01259	     endif
	LABEL?L	L00415	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01260	     if gore = 1 then 
	CMPNE?TCL	_gore, 001h, L00419

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01261	         if x = 1 then 
	CMPNE?BCL	_x, 001h, L00421

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01262	            x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01263	         else
	GOTO?L	L00422
	LABEL?L	L00421	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01264	            x = x - 1 
	SUB?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01265	         endif
	LABEL?L	L00422	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01266	     endif
	LABEL?L	L00419	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01267	     if levo = 1 then 
	CMPNE?TCL	_levo, 001h, L00423

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01268	         x = pozicija
	MOVE?BB	_Pozicija, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01269	         goto UnutrasnjiPrekidaci
	GOTO?L	_UnutrasnjiPrekidaci

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01270	     endif
	LABEL?L	L00423	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01272	    Select case x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01273	    case 1 
	CMPNE?BCL	_x, 001h, L00427

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01274	            Lcdout $fe, 2,Strelica ,"Time 1             "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	_Strelica
	LCDOUT?C	054h
	LCDOUT?C	069h
	LCDOUT?C	06Dh
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	031h
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01275	            lcdout $fe,$c0," Time 2                    "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
	LCDOUT?C	054h
	LCDOUT?C	069h
	LCDOUT?C	06Dh
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	032h
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
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01276	    case 2 
	GOTO?L	L00426
	LABEL?L	L00427	
	CMPNE?BCL	_x, 002h, L00428

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01277	            Lcdout $fe, 2," Time 1                  "           
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
	LCDOUT?C	054h
	LCDOUT?C	069h
	LCDOUT?C	06Dh
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	031h
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
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01278	            lcdout $fe,$c0,Strelica ,"Time 2              "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	_Strelica
	LCDOUT?C	054h
	LCDOUT?C	069h
	LCDOUT?C	06Dh
	LCDOUT?C	065h
	LCDOUT?C	020h
	LCDOUT?C	032h
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01279	    end select
	LABEL?L	L00428	
	LABEL?L	L00426	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01281	if ok = 1 then  
	CMPNE?TCL	_ok, 001h, L00429

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01282	    if x = 1 then VremePrekidaca = 0
	CMPNE?BCL	_x, 001h, L00431
	MOVE?CT	000h, _VremePrekidaca
	LABEL?L	L00431	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01283	    if x = 2 then VremePrekidaca = 1
	CMPNE?BCL	_x, 002h, L00433
	MOVE?CT	001h, _VremePrekidaca
	LABEL?L	L00433	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01284	    if Prekidac = 0 and VremePrekidaca  = 0 then 
	CMPEQ?TCB	_Prekidac, 000h, T1
	CMPEQ?TCB	_VremePrekidaca, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00435

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01285	        read 74,decsat
	READADDRESS?C	04Ah
	READ?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01286	        read 75,decmin
	READADDRESS?C	04Bh
	READ?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01288	        read 76,decsat1
	READADDRESS?C	04Ch
	READ?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01289	        read 77,decmin1
	READADDRESS?C	04Dh
	READ?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01290	     endif
	LABEL?L	L00435	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01292	     if Prekidac = 0 and VremePrekidaca  = 1 then 
	CMPEQ?TCB	_Prekidac, 000h, T1
	CMPEQ?TCB	_VremePrekidaca, 001h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00437

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01293	        read 78,decsat
	READADDRESS?C	04Eh
	READ?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01294	        read 79,decmin
	READADDRESS?C	04Fh
	READ?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01296	        read 80,decsat1
	READADDRESS?C	050h
	READ?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01297	        read 81,decmin1
	READADDRESS?C	051h
	READ?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01298	     endif
	LABEL?L	L00437	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01300	     if Prekidac = 1 and VremePrekidaca  = 0 then 
	CMPEQ?TCB	_Prekidac, 001h, T1
	CMPEQ?TCB	_VremePrekidaca, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00439

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01301	        read 82,decsat
	READADDRESS?C	052h
	READ?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01302	        read 83,decmin
	READADDRESS?C	053h
	READ?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01304	        read 84,decsat1
	READADDRESS?C	054h
	READ?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01305	        read 85,decmin1
	READADDRESS?C	055h
	READ?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01306	     endif
	LABEL?L	L00439	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01308	     if Prekidac = 1 and VremePrekidaca  = 1 then 
	CMPEQ?TCB	_Prekidac, 001h, T1
	CMPEQ?TCB	_VremePrekidaca, 001h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00441

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01309	        read 86,decsat
	READADDRESS?C	056h
	READ?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01310	        read 87,decmin
	READADDRESS?C	057h
	READ?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01312	        read 88,decsat1
	READADDRESS?C	058h
	READ?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01313	        read 89,decmin1
	READADDRESS?C	059h
	READ?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01314	     endif
	LABEL?L	L00441	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01316	    if VremePrekidaca = 0 then 
	CMPNE?TCL	_VremePrekidaca, 000h, L00443

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01317	         x = 1 
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01318	    else
	GOTO?L	L00444
	LABEL?L	L00443	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01319	         x = 1 
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01320	    endif
	LABEL?L	L00444	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01321	    goto UnutrasnjiPrekidacPodesavanje
	GOTO?L	_UnutrasnjiPrekidacPodesavanje

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01322	endif
	LABEL?L	L00429	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01323	goto UnutrasnjiPrekidacOdabir
	GOTO?L	_UnutrasnjiPrekidacOdabir

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01325	UnutrasnjiPrekidacPodesavanje:

	LABEL?L	_UnutrasnjiPrekidacPodesavanje	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01326	gosub kreni
	GOSUB?L	_kreni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01327	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01328	    if desno = 1 then 
	CMPNE?TCL	_desno, 001h, L00445

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01329	         if x = 4  then 
	CMPNE?BCL	_x, 004h, L00447

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01330	            x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01331	         else
	GOTO?L	L00448
	LABEL?L	L00447	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01332	            x = x + 1 
	ADD?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01333	         endif
	LABEL?L	L00448	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01334	    endif
	LABEL?L	L00445	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01335	    IF x = 1 THEN  
	CMPNE?BCL	_x, 001h, L00449

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01336	        displej[1] = strelica
	MOVE?CB	_Strelica, _displej + 00001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01337	        displej[2] = " "
	MOVE?CB	020h, _displej + 00002h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01338	        displej[3] = " "
	MOVE?CB	020h, _displej + 00003h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01339	        displej[4] = " "
	MOVE?CB	020h, _displej + 00004h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01340	     ENDIF
	LABEL?L	L00449	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01341	     IF x = 2 THEN
	CMPNE?BCL	_x, 002h, L00451

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01342	        displej[1] = " "
	MOVE?CB	020h, _displej + 00001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01343	        displej[2] = strelica
	MOVE?CB	_Strelica, _displej + 00002h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01344	        displej[3] = " "
	MOVE?CB	020h, _displej + 00003h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01345	        displej[4] = " "
	MOVE?CB	020h, _displej + 00004h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01346	     endif 
	LABEL?L	L00451	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01347	     if x = 3 then
	CMPNE?BCL	_x, 003h, L00453

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01348	        displej[1] = " "
	MOVE?CB	020h, _displej + 00001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01349	        displej[2] = " "
	MOVE?CB	020h, _displej + 00002h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01350	        displej[3] = strelica
	MOVE?CB	_Strelica, _displej + 00003h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01351	        displej[4] = " "
	MOVE?CB	020h, _displej + 00004h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01352	     endif
	LABEL?L	L00453	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01353	     if x = 4 then
	CMPNE?BCL	_x, 004h, L00455

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01354	        displej[1] = " "
	MOVE?CB	020h, _displej + 00001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01355	        displej[2] = " "
	MOVE?CB	020h, _displej + 00002h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01356	        displej[3] = " "
	MOVE?CB	020h, _displej + 00003h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01357	        displej[4] = strelica
	MOVE?CB	_Strelica, _displej + 00004h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01358	     endif
	LABEL?L	L00455	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01359	     if levo = 1 then 
	CMPNE?TCL	_levo, 001h, L00457

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01360	         goto UnutrasnjiPrekidacOdabir
	GOTO?L	_UnutrasnjiPrekidacOdabir

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01361	     endif
	LABEL?L	L00457	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01362	    if tastgore = 0 then
	CMPNE?TCL	_TASTGORE, 000h, L00459

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01363	         if x = 1 then
	CMPNE?BCL	_x, 001h, L00461

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01364	            decsaT = decsat + 1
	ADD?BCB	_DecSat, 001h, _DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01365	            if decsat > 23 then decsat = 23
	CMPLE?BCL	_DecSat, 017h, L00463
	MOVE?CB	017h, _DecSat
	LABEL?L	L00463	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01366	            pause PauzaZaTaster 
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01367	         endif
	LABEL?L	L00461	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01369	         if x = 2 then
	CMPNE?BCL	_x, 002h, L00465

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01370	            decmin = decmin + 1
	ADD?BCB	_DecMin, 001h, _DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01371	            if decmin > 59 then decmin = 59
	CMPLE?BCL	_DecMin, 03Bh, L00467
	MOVE?CB	03Bh, _DecMin
	LABEL?L	L00467	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01372	            pause PauzaZaTaster 
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01373	         endif
	LABEL?L	L00465	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01375	         if x = 3 then
	CMPNE?BCL	_x, 003h, L00469

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01376	            decsaT1 = decsat1 + 1
	ADD?BCB	_DecSat1, 001h, _DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01377	            if decsat1 > 23 then decsat1 = 23
	CMPLE?BCL	_DecSat1, 017h, L00471
	MOVE?CB	017h, _DecSat1
	LABEL?L	L00471	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01378	            pause PauzaZaTaster 
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01379	         endif
	LABEL?L	L00469	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01381	         if x = 4 then
	CMPNE?BCL	_x, 004h, L00473

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01382	            decmin1 = decmin1 + 1
	ADD?BCB	_DecMin1, 001h, _DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01383	            if decmin1 > 59 then decmin1 = 59
	CMPLE?BCL	_DecMin1, 03Bh, L00475
	MOVE?CB	03Bh, _DecMin1
	LABEL?L	L00475	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01384	            pause PauzaZaTaster 
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01385	         endif
	LABEL?L	L00473	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01386	     endif
	LABEL?L	L00459	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01387	     if tastdole = 0 then
	CMPNE?TCL	_TASTDOLE, 000h, L00477

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01388	         if x = 1 then
	CMPNE?BCL	_x, 001h, L00479

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01389	            decsaT = decsat - 1
	SUB?BCB	_DecSat, 001h, _DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01390	            if decsat < 1 or decsat > 23 then decsat = 0
	CMPLT?BCB	_DecSat, 001h, T1
	CMPGT?BCB	_DecSat, 017h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00481
	MOVE?CB	000h, _DecSat
	LABEL?L	L00481	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01391	            pause PauzaZaTaster 
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01392	         endif
	LABEL?L	L00479	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01394	         if x = 2 then
	CMPNE?BCL	_x, 002h, L00483

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01395	            decmin = decmin - 1
	SUB?BCB	_DecMin, 001h, _DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01396	            if decmin < 1 or decmin > 59 then decmin = 0   
	CMPLT?BCB	_DecMin, 001h, T1
	CMPGT?BCB	_DecMin, 03Bh, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00485
	MOVE?CB	000h, _DecMin
	LABEL?L	L00485	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01397	            pause PauzaZaTaster 
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01398	         endif
	LABEL?L	L00483	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01399	         if x = 3 then
	CMPNE?BCL	_x, 003h, L00487

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01400	            decsaT1 = decsat1 - 1
	SUB?BCB	_DecSat1, 001h, _DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01401	            if decsat1 < 1 or decsat1 > 23 then decsat1 = 0
	CMPLT?BCB	_DecSat1, 001h, T1
	CMPGT?BCB	_DecSat1, 017h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00489
	MOVE?CB	000h, _DecSat1
	LABEL?L	L00489	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01402	            pause PauzaZaTaster 
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01403	         endif
	LABEL?L	L00487	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01405	         if x = 4 then
	CMPNE?BCL	_x, 004h, L00491

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01406	            decmin1 = decmin1 - 1
	SUB?BCB	_DecMin1, 001h, _DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01407	            if decmin1 < 1 or decmin1 > 59 then decmin1 = 0   
	CMPLT?BCB	_DecMin1, 001h, T1
	CMPGT?BCB	_DecMin1, 03Bh, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00493
	MOVE?CB	000h, _DecMin1
	LABEL?L	L00493	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01408	            pause PauzaZaTaster 
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01409	         endif
	LABEL?L	L00491	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01410	     endif
	LABEL?L	L00477	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01411	if ok = 1 then 
	CMPNE?TCL	_ok, 001h, L00495

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01412	     if Prekidac = 0 and VremePrekidaca  = 0 then 
	CMPEQ?TCB	_Prekidac, 000h, T1
	CMPEQ?TCB	_VremePrekidaca, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00497

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01413	        write 74,decsat
	WRITEADDRESS?C	04Ah
	WRITE?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01414	        write 75,decmin
	WRITEADDRESS?C	04Bh
	WRITE?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01416	        write 76,decsat1
	WRITEADDRESS?C	04Ch
	WRITE?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01417	        write 77,decmin1
	WRITEADDRESS?C	04Dh
	WRITE?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01418	     endif
	LABEL?L	L00497	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01420	     if Prekidac = 0 and VremePrekidaca  = 1 then 
	CMPEQ?TCB	_Prekidac, 000h, T1
	CMPEQ?TCB	_VremePrekidaca, 001h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00499

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01421	        write 78,decsat
	WRITEADDRESS?C	04Eh
	WRITE?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01422	        write 79,decmin
	WRITEADDRESS?C	04Fh
	WRITE?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01424	        write 80,decsat1
	WRITEADDRESS?C	050h
	WRITE?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01425	        write 81,decmin1
	WRITEADDRESS?C	051h
	WRITE?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01426	     endif
	LABEL?L	L00499	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01428	     if Prekidac = 1 and VremePrekidaca  = 0 then 
	CMPEQ?TCB	_Prekidac, 001h, T1
	CMPEQ?TCB	_VremePrekidaca, 000h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00501

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01429	        write 82,decsat
	WRITEADDRESS?C	052h
	WRITE?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01430	        write 83,decmin
	WRITEADDRESS?C	053h
	WRITE?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01432	        write 84,decsat1
	WRITEADDRESS?C	054h
	WRITE?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01433	        write 85,decmin1
	WRITEADDRESS?C	055h
	WRITE?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01434	     endif
	LABEL?L	L00501	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01436	     if Prekidac = 1 and VremePrekidaca  = 1 then 
	CMPEQ?TCB	_Prekidac, 001h, T1
	CMPEQ?TCB	_VremePrekidaca, 001h, T2
	LAND?BBN	T1, T2, T2
	CMPF?NL	T2, L00503

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01437	        write 86,decsat
	WRITEADDRESS?C	056h
	WRITE?B	_DecSat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01438	        write 87,decmin
	WRITEADDRESS?C	057h
	WRITE?B	_DecMin

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01440	        write 88,decsat1
	WRITEADDRESS?C	058h
	WRITE?B	_DecSat1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01441	        write 89,decmin1
	WRITEADDRESS?C	059h
	WRITE?B	_DecMin1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01442	     endif
	LABEL?L	L00503	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01443	        if VremePrekidaca = 0 then 
	CMPNE?TCL	_VremePrekidaca, 000h, L00505

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01444	            x = 1 
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01445	        else
	GOTO?L	L00506
	LABEL?L	L00505	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01446	            x = 2 
	MOVE?CB	002h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01447	        endif
	LABEL?L	L00506	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01449	        gosub EEpromMemorija
	GOSUB?L	_EEpromMemorija

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01450	        gosub sacuvano
	GOSUB?L	_sacuvano

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01451	        goto UnutrasnjiPrekidacOdabir
	GOTO?L	_UnutrasnjiPrekidacOdabir

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01452	endif
	LABEL?L	L00495	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01454	   Lcdout $fe, 2, "ON: ",displej[1],dec2 decsat,":",displej[2],dec2 decmin,"   "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	04Fh
	LCDOUT?C	04Eh
	LCDOUT?C	03Ah
	LCDOUT?C	020h
	LCDOUT?B	_displej + 00001h
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_DecSat
	LCDOUTDEC?	
	LCDOUT?C	03Ah
	LCDOUT?B	_displej + 00002h
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_DecMin
	LCDOUTDEC?	
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01455	   lcdout $fe,$c0,"OFF:",displej[3],dec2 DecSat1,":",displej[4],dec2 DecMin1,"   "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	04Fh
	LCDOUT?C	046h
	LCDOUT?C	046h
	LCDOUT?C	03Ah
	LCDOUT?B	_displej + 00003h
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_DecSat1
	LCDOUTDEC?	
	LCDOUT?C	03Ah
	LCDOUT?B	_displej + 00004h
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_DecMin1
	LCDOUTDEC?	
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01456	goto UnutrasnjiPrekidacPodesavanje
	GOTO?L	_UnutrasnjiPrekidacPodesavanje

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01458	LedManual:

	LABEL?L	_LedManual	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01459	gosub kreni
	GOSUB?L	_kreni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01460	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01461	     if dole = 1 then 
	CMPNE?TCL	_dole, 001h, L00507

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01462	         if x = 2 then 
	CMPNE?BCL	_x, 002h, L00509

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01463	            x = 2
	MOVE?CB	002h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01464	         else
	GOTO?L	L00510
	LABEL?L	L00509	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01465	            x = x + 1 
	ADD?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01466	         endif
	LABEL?L	L00510	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01467	     endif
	LABEL?L	L00507	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01468	     if gore = 1 then 
	CMPNE?TCL	_gore, 001h, L00511

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01469	         if x = 1 then 
	CMPNE?BCL	_x, 001h, L00513

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01470	            x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01471	         else
	GOTO?L	L00514
	LABEL?L	L00513	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01472	            x = x - 1 
	SUB?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01473	         endif
	LABEL?L	L00514	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01474	     endif
	LABEL?L	L00511	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01475	     if levo = 1 then 
	CMPNE?TCL	_levo, 001h, L00515

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01476	        x = 2
	MOVE?CB	002h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01477	         goto LedPodMeni
	GOTO?L	_LedPodMeni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01478	     endif
	LABEL?L	L00515	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01480	    Select case x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01481	    case 1 
	CMPNE?BCL	_x, 001h, L00519

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01482	        Lcdout $fe, 2,Strelica ,"Aquarium light      "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	_Strelica
	LCDOUT?C	041h
	LCDOUT?C	071h
	LCDOUT?C	075h
	LCDOUT?C	061h
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	075h
	LCDOUT?C	06Dh
	LCDOUT?C	020h
	LCDOUT?C	06Ch
	LCDOUT?C	069h
	LCDOUT?C	067h
	LCDOUT?C	068h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01483	        lcdout $fe,$c0," Room light            "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
	LCDOUT?C	052h
	LCDOUT?C	06Fh
	LCDOUT?C	06Fh
	LCDOUT?C	06Dh
	LCDOUT?C	020h
	LCDOUT?C	06Ch
	LCDOUT?C	069h
	LCDOUT?C	067h
	LCDOUT?C	068h
	LCDOUT?C	074h
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01484	        if ok = 1 then 
	CMPNE?TCL	_ok, 001h, L00520

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01485	            x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01486	            automatskoakvarijum = 0
	MOVE?CT	000h, _AutomatskoAkvarijum

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01487	            read 100,rkanal1
	READADDRESS?C	064h
	READ?B	_Rkanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01488	            read 101,rkanal2
	READADDRESS?C	065h
	READ?B	_Rkanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01489	            read 102,rkanal3
	READADDRESS?C	066h
	READ?B	_Rkanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01490	            goto RucnoSvetlo
	GOTO?L	_RucnoSvetlo

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01491	        endif
	LABEL?L	L00520	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01492	    case 2 
	GOTO?L	L00518
	LABEL?L	L00519	
	CMPNE?BCL	_x, 002h, L00522

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01493	        Lcdout $fe, 2," Aquarium light      "           
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
	LCDOUT?C	041h
	LCDOUT?C	071h
	LCDOUT?C	075h
	LCDOUT?C	061h
	LCDOUT?C	072h
	LCDOUT?C	069h
	LCDOUT?C	075h
	LCDOUT?C	06Dh
	LCDOUT?C	020h
	LCDOUT?C	06Ch
	LCDOUT?C	069h
	LCDOUT?C	067h
	LCDOUT?C	068h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01494	        lcdout $fe,$c0,Strelica ,"Room light      "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	_Strelica
	LCDOUT?C	052h
	LCDOUT?C	06Fh
	LCDOUT?C	06Fh
	LCDOUT?C	06Dh
	LCDOUT?C	020h
	LCDOUT?C	06Ch
	LCDOUT?C	069h
	LCDOUT?C	067h
	LCDOUT?C	068h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01495	        if ok = 1 then 
	CMPNE?TCL	_ok, 001h, L00523

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01496	            automatskosoba = 0
	MOVE?CT	000h, _AutomatskoSoba

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01497	            read 103,x
	READADDRESS?C	067h
	READ?B	_x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01498	            goto RoomLight
	GOTO?L	_RoomLight

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01499	        endif
	LABEL?L	L00523	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01500	    end select
	LABEL?L	L00522	
	LABEL?L	L00518	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01501	goto ledmanual
	GOTO?L	_LedManual

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01503	RucnoSvetlo:

	LABEL?L	_RucnoSvetlo	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01504	gosub kreni
	GOSUB?L	_kreni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01505	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01507	    if desno = 1 then 
	CMPNE?TCL	_desno, 001h, L00525

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01508	         if x = 3 then 
	CMPNE?BCL	_x, 003h, L00527

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01509	            x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01510	         else
	GOTO?L	L00528
	LABEL?L	L00527	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01511	            x = x + 1 
	ADD?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01512	         endif
	LABEL?L	L00528	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01513	    endif
	LABEL?L	L00525	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01515	    if levo = 1 then 
	CMPNE?TCL	_levo, 001h, L00529

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01516	         read 5,automatskoakvarijum 
	READADDRESS?C	005h
	READ?T	_AutomatskoAkvarijum

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01517	         x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01518	         goto LedManual
	GOTO?L	_LedManual

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01519	    endif
	LABEL?L	L00529	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01521	     IF x = 1 THEN  
	CMPNE?BCL	_x, 001h, L00531

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01522	        displej[1] = strelica
	MOVE?CB	_Strelica, _displej + 00001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01523	        displej[2] = " "
	MOVE?CB	020h, _displej + 00002h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01524	        displej[3] = " "
	MOVE?CB	020h, _displej + 00003h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01525	     ENDIF
	LABEL?L	L00531	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01526	     IF x = 2 THEN
	CMPNE?BCL	_x, 002h, L00533

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01527	        displej[1] = " "
	MOVE?CB	020h, _displej + 00001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01528	        displej[2] = strelica
	MOVE?CB	_Strelica, _displej + 00002h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01529	        displej[3] = " "
	MOVE?CB	020h, _displej + 00003h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01530	     endif 
	LABEL?L	L00533	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01531	     if x = 3 then
	CMPNE?BCL	_x, 003h, L00535

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01532	        displej[1] = " "
	MOVE?CB	020h, _displej + 00001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01533	        displej[2] = " "
	MOVE?CB	020h, _displej + 00002h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01534	        displej[3] = strelica
	MOVE?CB	_Strelica, _displej + 00003h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01535	     endif
	LABEL?L	L00535	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01537	     if tastgore = 0 then
	CMPNE?TCL	_TASTGORE, 000h, L00537

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01538	            if x = 1 then
	CMPNE?BCL	_x, 001h, L00539

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01539	                Rkanal1 = Rkanal1 + 1 
	ADD?BCB	_Rkanal1, 001h, _Rkanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01540	                if Rkanal1 > 100 then rkanal1 = 100 
	CMPLE?BCL	_Rkanal1, 064h, L00541
	MOVE?CB	064h, _Rkanal1
	LABEL?L	L00541	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01542	            endif
	LABEL?L	L00539	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01544	            if x = 2 then
	CMPNE?BCL	_x, 002h, L00543

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01545	                Rkanal2 = Rkanal2 + 1
	ADD?BCB	_Rkanal2, 001h, _Rkanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01546	                if Rkanal2 > 100 then rkanal2 = 100 
	CMPLE?BCL	_Rkanal2, 064h, L00545
	MOVE?CB	064h, _Rkanal2
	LABEL?L	L00545	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01548	            endif
	LABEL?L	L00543	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01549	            if x = 3 then
	CMPNE?BCL	_x, 003h, L00547

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01550	                Rkanal3 = Rkanal3 + 1 
	ADD?BCB	_Rkanal3, 001h, _Rkanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01551	                if Rkanal3 > 100 then rkanal3 = 100 
	CMPLE?BCL	_Rkanal3, 064h, L00549
	MOVE?CB	064h, _Rkanal3
	LABEL?L	L00549	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01552	            endif
	LABEL?L	L00547	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01553	            pause PauzaZaTaster 
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01554	     endif 
	LABEL?L	L00537	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01556	     if tastdole = 0 then
	CMPNE?TCL	_TASTDOLE, 000h, L00551

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01557	            if x = 1 then
	CMPNE?BCL	_x, 001h, L00553

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01558	                Rkanal1 = Rkanal1 - 1
	SUB?BCB	_Rkanal1, 001h, _Rkanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01559	                if  Rkanal1 < 1 or  Rkanal1 > 100 then  Rkanal1 = 0 
	CMPLT?BCB	_Rkanal1, 001h, T1
	CMPGT?BCB	_Rkanal1, 064h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00555
	MOVE?CB	000h, _Rkanal1
	LABEL?L	L00555	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01560	            endif
	LABEL?L	L00553	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01562	             if x = 2 then
	CMPNE?BCL	_x, 002h, L00557

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01563	                Rkanal2 = Rkanal2 - 1
	SUB?BCB	_Rkanal2, 001h, _Rkanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01564	                if  Rkanal2 < 1 or  Rkanal2 > 100 then  Rkanal2 = 0 
	CMPLT?BCB	_Rkanal2, 001h, T1
	CMPGT?BCB	_Rkanal2, 064h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00559
	MOVE?CB	000h, _Rkanal2
	LABEL?L	L00559	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01565	             endif
	LABEL?L	L00557	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01567	             if x = 3 then
	CMPNE?BCL	_x, 003h, L00561

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01568	                Rkanal3 = Rkanal3 - 1 
	SUB?BCB	_Rkanal3, 001h, _Rkanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01569	                if  Rkanal3 < 1 or  Rkanal3 > 100 then  Rkanal3 = 0
	CMPLT?BCB	_Rkanal3, 001h, T1
	CMPGT?BCB	_Rkanal3, 064h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00563
	MOVE?CB	000h, _Rkanal3
	LABEL?L	L00563	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01570	             endif
	LABEL?L	L00561	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01571	             pause PauzaZaTaster 
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01572	     endif 
	LABEL?L	L00551	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01576	     Lcdout $fe, 2,  displej[1],"#1 ",dec3 rkanal1,"%", displej[2],"2# ",dec3 rkanal2,"%        "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?B	_displej + 00001h
	LCDOUT?C	023h
	LCDOUT?C	031h
	LCDOUT?C	020h
	LCDOUTCOUNT?C	003h
	LCDOUTNUM?B	_Rkanal1
	LCDOUTDEC?	
	LCDOUT?C	025h
	LCDOUT?B	_displej + 00002h
	LCDOUT?C	032h
	LCDOUT?C	023h
	LCDOUT?C	020h
	LCDOUTCOUNT?C	003h
	LCDOUTNUM?B	_Rkanal2
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01577	     Lcdout $fe, $c0,displej[3],"#3 ",dec3 rkanal3,"%            "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?B	_displej + 00003h
	LCDOUT?C	023h
	LCDOUT?C	033h
	LCDOUT?C	020h
	LCDOUTCOUNT?C	003h
	LCDOUTNUM?B	_Rkanal3
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01578	     if ok = 1 then 
	CMPNE?TCL	_ok, 001h, L00565

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01579	            write 100,rkanal1
	WRITEADDRESS?C	064h
	WRITE?B	_Rkanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01580	            write 101,rkanal2
	WRITEADDRESS?C	065h
	WRITE?B	_Rkanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01581	            write 102,rkanal3
	WRITEADDRESS?C	066h
	WRITE?B	_Rkanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01582	            write 5, 0
	WRITEADDRESS?C	005h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01583	            AutomatskoAkvarijum  = 0 
	MOVE?CT	000h, _AutomatskoAkvarijum

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01584	            gosub Sacuvano
	GOSUB?L	_sacuvano

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01585	            x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01586	            goto LedManual
	GOTO?L	_LedManual

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01587	     endif 
	LABEL?L	L00565	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01588	     DutyCycleL1 =(rkanal1 * 1024)/100
	MUL?BCN	_Rkanal1, 000000400h, T1
	DIV?NCN	T1, 064h, _DutyCycleL1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01589	     DutyCycleL2 =(rkanal2 * 1024)/100
	MUL?BCN	_Rkanal2, 000000400h, T1
	DIV?NCN	T1, 064h, _DutyCycleL2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01590	     DutyCycleL3 =(rkanal3 * 1024)/100
	MUL?BCN	_Rkanal3, 000000400h, T1
	DIV?NCN	T1, 064h, _DutyCycleL3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01592	    @ HPWM10 4, _DutyCycleL1, _Frequency

	ASM?
 HPWM10 4, _DutyCycleL1, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01593	    @ HPWM10 3, _DutyCycleL2, _Frequency

	ASM?
 HPWM10 3, _DutyCycleL2, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01594	    @ HPWM10 1, _DutyCycleL3, _Frequency                        

	ASM?
 HPWM10 1, _DutyCycleL3, _Frequency                        

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01595	Goto  RucnoSvetlo
	GOTO?L	_RucnoSvetlo

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01597	RoomLight:

	LABEL?L	_RoomLight	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01598	gosub kreni
	GOSUB?L	_kreni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01599	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01600	     room = 2
	MOVE?CT	002h, _Room

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01601	     if tastgore  = 0 then
	CMPNE?TCL	_TASTGORE, 000h, L00567

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01602	          x = x + 1 
	ADD?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01603	          if x > 100 then x = 100
	CMPLE?BCL	_x, 064h, L00569
	MOVE?CB	064h, _x
	LABEL?L	L00569	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01604	          PAUSE PauzaZaTaster 
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01605	     endif 
	LABEL?L	L00567	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01607	     if tastdole  = 0 then
	CMPNE?TCL	_TASTDOLE, 000h, L00571

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01608	          x = x - 1 
	SUB?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01609	          if x < 1 or x > 100 then x = 0
	CMPLT?BCB	_x, 001h, T1
	CMPGT?BCB	_x, 064h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00573
	MOVE?CB	000h, _x
	LABEL?L	L00573	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01610	          PAUSE PauzaZaTaster 
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01611	     endif
	LABEL?L	L00571	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01613	     if levo = 1 then 
	CMPNE?TCL	_levo, 001h, L00575

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01614	         read 6,automatskosoba 
	READADDRESS?C	006h
	READ?T	_AutomatskoSoba

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01615	         x = 2
	MOVE?CB	002h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01616	         goto LedManual
	GOTO?L	_LedManual

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01617	     endif
	LABEL?L	L00575	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01619	     IF ok = 1 then 
	CMPNE?TCL	_ok, 001h, L00577

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01620	         write 103,x
	WRITEADDRESS?C	067h
	WRITE?B	_x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01621	         write 6,0
	WRITEADDRESS?C	006h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01622	         AutomatskoSoba = 0 
	MOVE?CT	000h, _AutomatskoSoba

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01623	         gosub Sacuvano
	GOSUB?L	_sacuvano

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01624	         x = 2
	MOVE?CB	002h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01625	         goto LedManual
	GOTO?L	_LedManual

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01626	     endif
	LABEL?L	L00577	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01628	    Lcdout $fe, 2,"Room light #4       "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	052h
	LCDOUT?C	06Fh
	LCDOUT?C	06Fh
	LCDOUT?C	06Dh
	LCDOUT?C	020h
	LCDOUT?C	06Ch
	LCDOUT?C	069h
	LCDOUT?C	067h
	LCDOUT?C	068h
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	023h
	LCDOUT?C	034h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01629	    lcdout $fe,$c0,strelica,dec3 x," %              "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	_Strelica
	LCDOUTCOUNT?C	003h
	LCDOUTNUM?B	_x
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
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01631	    DutyCycleL4 =(x * 1024)/100
	MUL?BCN	_x, 000000400h, T1
	DIV?NCN	T1, 064h, _DutyCycleL4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01632	    @ HPWM10 2, _DutyCycleL4, _Frequency

	ASM?
 HPWM10 2, _DutyCycleL4, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01633	 goto RoomLight 
	GOTO?L	_RoomLight

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01635	Alat:

	LABEL?L	_Alat	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01636	gosub kreni
	GOSUB?L	_kreni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01637	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01639	     if dole = 1 then 
	CMPNE?TCL	_dole, 001h, L00579

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01640	         if x = 2 then 
	CMPNE?BCL	_x, 002h, L00581

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01641	            x = 2
	MOVE?CB	002h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01642	         else
	GOTO?L	L00582
	LABEL?L	L00581	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01643	            x = x + 1 
	ADD?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01644	         endif
	LABEL?L	L00582	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01645	     endif
	LABEL?L	L00579	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01646	     if gore = 1 then 
	CMPNE?TCL	_gore, 001h, L00583

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01647	         if x = 1 then 
	CMPNE?BCL	_x, 001h, L00585

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01648	            x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01649	         else
	GOTO?L	L00586
	LABEL?L	L00585	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01650	            x = x - 1 
	SUB?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01651	         endif
	LABEL?L	L00586	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01652	     endif
	LABEL?L	L00583	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01653	     if levo = 1 then 
	CMPNE?TCL	_levo, 001h, L00587

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01654	        x = 3
	MOVE?CB	003h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01655	         goto GlavniMeni
	GOTO?L	_GlavniMeni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01656	     endif
	LABEL?L	L00587	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01657	    Select case x                 

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01658	        case 1       
	CMPNE?BCL	_x, 001h, L00591

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01659	            Lcdout $fe, 2,strelica,"Time                    "          
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	_Strelica
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
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01660	            lcdout $fe,$c0," Default settings     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	020h
	LCDOUT?C	044h
	LCDOUT?C	065h
	LCDOUT?C	066h
	LCDOUT?C	061h
	LCDOUT?C	075h
	LCDOUT?C	06Ch
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	074h
	LCDOUT?C	069h
	LCDOUT?C	06Eh
	LCDOUT?C	067h
	LCDOUT?C	073h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01661	            if ok  = 1 then 
	CMPNE?TCL	_ok, 001h, L00592

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01662	                x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01663	                goto RTCvreme
	GOTO?L	_RTCvreme

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01664	            endif
	LABEL?L	L00592	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01665	        case 2
	GOTO?L	L00590
	LABEL?L	L00591	
	CMPNE?BCL	_x, 002h, L00594

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01666	            Lcdout $fe, 2," Time                    "  
	LCDOUT?C	0FEh
	LCDOUT?C	002h
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
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01667	            lcdout $fe,$c0,strelica,"Default settings     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	_Strelica
	LCDOUT?C	044h
	LCDOUT?C	065h
	LCDOUT?C	066h
	LCDOUT?C	061h
	LCDOUT?C	075h
	LCDOUT?C	06Ch
	LCDOUT?C	074h
	LCDOUT?C	020h
	LCDOUT?C	073h
	LCDOUT?C	065h
	LCDOUT?C	074h
	LCDOUT?C	074h
	LCDOUT?C	069h
	LCDOUT?C	06Eh
	LCDOUT?C	067h
	LCDOUT?C	073h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01668	            if ok = 1 then 
	CMPNE?TCL	_ok, 001h, L00595

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01669	                x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01670	                goto DefultSet 
	GOTO?L	_DefultSet

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01671	            endif
	LABEL?L	L00595	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01672	    end select 
	LABEL?L	L00594	
	LABEL?L	L00590	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01673	goto Alat
	GOTO?L	_Alat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01675	ModOdrzavanje:

	LABEL?L	_mododrzavanje	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01676	gosub kreni
	GOSUB?L	_kreni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01677	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01679	     if tastgore  = 0 then
	CMPNE?TCL	_TASTGORE, 000h, L00597

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01680	          OdrzavanjeJacina = OdrzavanjeJacina + 1 
	ADD?BCB	_OdrzavanjeJacina, 001h, _OdrzavanjeJacina

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01681	          if OdrzavanjeJacina > 100 then OdrzavanjeJacina = 100
	CMPLE?BCL	_OdrzavanjeJacina, 064h, L00599
	MOVE?CB	064h, _OdrzavanjeJacina
	LABEL?L	L00599	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01682	          PAUSE PauzaZaTaster 
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01683	     endif  
	LABEL?L	L00597	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01684	     if tastdole  = 0 then
	CMPNE?TCL	_TASTDOLE, 000h, L00601

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01685	          OdrzavanjeJacina = OdrzavanjeJacina - 1 
	SUB?BCB	_OdrzavanjeJacina, 001h, _OdrzavanjeJacina

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01686	          if OdrzavanjeJacina < 1 or OdrzavanjeJacina > 100 then OdrzavanjeJacina = 0
	CMPLT?BCB	_OdrzavanjeJacina, 001h, T1
	CMPGT?BCB	_OdrzavanjeJacina, 064h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00603
	MOVE?CB	000h, _OdrzavanjeJacina
	LABEL?L	L00603	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01687	          PAUSE PauzaZaTaster  
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01688	     endif
	LABEL?L	L00601	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01690	     if odrzavanje = 1 then 
	CMPNE?TCL	_odrzavanje, 001h, L00605

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01691	        OdrzavanjeMod = 0
	MOVE?CT	000h, _OdrzavanjeMod

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01692	        write 2,OdrzavanjeJacina 
	WRITEADDRESS?C	002h
	WRITE?B	_OdrzavanjeJacina

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01693	        x = 1 
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01694	             if automatskoakvarijum = 1 then  
	CMPNE?TCL	_AutomatskoAkvarijum, 001h, L00607

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01695	                kanal1 = stkanal1 / 100 
	DIV?NCB	_STKanal1, 064h, _Kanal1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01696	                kanal2 = stkanal2 / 100 
	DIV?NCB	_STKanal2, 064h, _Kanal2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01697	                kanal3 = stkanal3 / 100 
	DIV?NCB	_STKanal3, 064h, _Kanal3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01698	             endif
	LABEL?L	L00607	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01699	             if automatskosoba = 1 then 
	CMPNE?TCL	_AutomatskoSoba, 001h, L00609

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01700	                kanal4 = stkanal4 / 100 
	DIV?NCB	_STKanal4, 064h, _Kanal4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01701	             endif 
	LABEL?L	L00609	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01702	        goto Main
	GOTO?L	_main

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01703	     endif
	LABEL?L	L00605	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01705	    Lcdout $fe, 2,"Work light          "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	057h
	LCDOUT?C	06Fh
	LCDOUT?C	072h
	LCDOUT?C	06Bh
	LCDOUT?C	020h
	LCDOUT?C	06Ch
	LCDOUT?C	069h
	LCDOUT?C	067h
	LCDOUT?C	068h
	LCDOUT?C	074h
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01706	    lcdout $fe,$c0,"Channel #1-",dec3 OdrzavanjeJacina,"%           "
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
	LCDOUT?C	02Dh
	LCDOUTCOUNT?C	003h
	LCDOUTNUM?B	_OdrzavanjeJacina
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01707	    DutyCycleL1 =(OdrzavanjeJacina * 1024)/100
	MUL?BCN	_OdrzavanjeJacina, 000000400h, T1
	DIV?NCN	T1, 064h, _DutyCycleL1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01708	    DutyCycleL2 =(0 * 1024)/100
	MOVE?CN	000h, _DutyCycleL2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01709	    DutyCycleL3 =(0 * 1024)/100
	MOVE?CN	000h, _DutyCycleL3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01710	    @ HPWM10 1, _DutyCycleL3, _Frequency

	ASM?
 HPWM10 1, _DutyCycleL3, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01712	    @ HPWM10 4, _DutyCycleL1, _Frequency

	ASM?
 HPWM10 4, _DutyCycleL1, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01713	    @ HPWM10 3, _DutyCycleL2, _Frequency

	ASM?
 HPWM10 3, _DutyCycleL2, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01714	    if automatskosoba = 1 then  
	CMPNE?TCL	_AutomatskoSoba, 001h, L00611

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01715	        DutyCycleL4 =(0 * 1024)/100
	MOVE?CN	000h, _DutyCycleL4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01716	        @ HPWM10 2, _DutyCycleL4, _Frequency

	ASM?
 HPWM10 2, _DutyCycleL4, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01717	    endif
	LABEL?L	L00611	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01721	goto ModOdrzavanje
	GOTO?L	_mododrzavanje

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01723	DefultSet:

	LABEL?L	_DefultSet	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01724	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01725	gosub kreni
	GOSUB?L	_kreni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01727	if ok = 1 then 
	CMPNE?TCL	_ok, 001h, L00613

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01728	    if x = 0 then 
	CMPNE?BCL	_x, 000h, L00615

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01729	         x = 2 
	MOVE?CB	002h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01730	         goto alat
	GOTO?L	_Alat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01731	    endif
	LABEL?L	L00615	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01733	    if x =  1 then 
	CMPNE?BCL	_x, 001h, L00617

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01734	        gosub prviput
	GOSUB?L	_PrviPut

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01735	    endif
	LABEL?L	L00617	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01736	endif
	LABEL?L	L00613	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01738	if levo = 1 then 
	CMPNE?TCL	_levo, 001h, L00619

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01739	    x = 2 
	MOVE?CB	002h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01740	    goto alat
	GOTO?L	_Alat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01741	endif
	LABEL?L	L00619	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01743	if gore = 1 then 
	CMPNE?TCL	_gore, 001h, L00621

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01744	   if x = 1 then 
	CMPNE?BCL	_x, 001h, L00623

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01745	        x = 0 
	MOVE?CB	000h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01746	   else
	GOTO?L	L00624
	LABEL?L	L00623	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01747	        x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01748	   endif
	LABEL?L	L00624	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01749	endif
	LABEL?L	L00621	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01751	if dole = 1 then 
	CMPNE?TCL	_dole, 001h, L00625

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01752	   if x = 1 then 
	CMPNE?BCL	_x, 001h, L00627

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01753	        x = 0            
	MOVE?CB	000h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01754	   else
	GOTO?L	L00628
	LABEL?L	L00627	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01755	        x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01756	   endif
	LABEL?L	L00628	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01757	endif
	LABEL?L	L00625	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01758	   Lcdout $fe, 2,"Confirm changes             " 
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	043h
	LCDOUT?C	06Fh
	LCDOUT?C	06Eh
	LCDOUT?C	066h
	LCDOUT?C	069h
	LCDOUT?C	072h
	LCDOUT?C	06Dh
	LCDOUT?C	020h
	LCDOUT?C	063h
	LCDOUT?C	068h
	LCDOUT?C	061h
	LCDOUT?C	06Eh
	LCDOUT?C	067h
	LCDOUT?C	065h
	LCDOUT?C	073h
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01759	   if x = 0 then lcdout $fe,$c0,strelica,"NO                 "
	CMPNE?BCL	_x, 000h, L00629
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	_Strelica
	LCDOUT?C	04Eh
	LCDOUT?C	04Fh
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
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LABEL?L	L00629	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01760	   if x = 1 then lcdout $fe,$c0,strelica,"YES                 "
	CMPNE?BCL	_x, 001h, L00631
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	_Strelica
	LCDOUT?C	059h
	LCDOUT?C	045h
	LCDOUT?C	053h
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
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LABEL?L	L00631	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01762	goto DefultSet
	GOTO?L	_DefultSet

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01764	PodesavanjeTemperature:

	LABEL?L	_podesavanjetemperature	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01766	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01768	if ok = 1 then 
	CMPNE?TCL	_ok, 001h, L00633

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01769	    write 7,TemperaturaIzEproma
	WRITEADDRESS?C	007h
	WRITE?B	_TemperaturaIzEproma

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01770	    gosub splash
	GOSUB?L	_Splash

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01771	endif
	LABEL?L	L00633	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01772	if tastgore = 0 then 
	CMPNE?TCL	_TASTGORE, 000h, L00635

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01773	    TemperaturaIzEproma = TemperaturaIzEproma + 1 
	ADD?BCB	_TemperaturaIzEproma, 001h, _TemperaturaIzEproma

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01774	    pause pauzazataster
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01775	endif
	LABEL?L	L00635	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01777	if tastdole = 0 then 
	CMPNE?TCL	_TASTDOLE, 000h, L00637

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01778	    TemperaturaIzEproma = TemperaturaIzEproma - 1 
	SUB?BCB	_TemperaturaIzEproma, 001h, _TemperaturaIzEproma

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01779	    pause pauzazataster
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01780	endif
	LABEL?L	L00637	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01781	if TemperaturaIzEproma > 100  then TemperaturaIzEproma = 100
	CMPLE?BCL	_TemperaturaIzEproma, 064h, L00639
	MOVE?CB	064h, _TemperaturaIzEproma
	LABEL?L	L00639	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01782	if TemperaturaIzEproma < 10 then TemperaturaIzEproma = 10
	CMPGE?BCL	_TemperaturaIzEproma, 00Ah, L00641
	MOVE?CB	00Ah, _TemperaturaIzEproma
	LABEL?L	L00641	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01784	Lcdout $fe, 2,"Temp  relay            "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	054h
	LCDOUT?C	065h
	LCDOUT?C	06Dh
	LCDOUT?C	070h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	072h
	LCDOUT?C	065h
	LCDOUT?C	06Ch
	LCDOUT?C	061h
	LCDOUT?C	079h
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01785	lcdout $fe,$c0,">",dec3 TemperaturaIzEproma," " ,223,"C","                 "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	03Eh
	LCDOUTCOUNT?C	003h
	LCDOUTNUM?B	_TemperaturaIzEproma
	LCDOUTDEC?	
	LCDOUT?C	020h
	LCDOUT?C	0DFh
	LCDOUT?C	043h
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
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01786	goto PodesavanjeTemperature
	GOTO?L	_podesavanjetemperature

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01789	Sacuvano:

	LABEL?L	_sacuvano	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01790	        Lcdout $fe, 2," CHANGES  SAVED        "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
	LCDOUT?C	043h
	LCDOUT?C	048h
	LCDOUT?C	041h
	LCDOUT?C	04Eh
	LCDOUT?C	047h
	LCDOUT?C	045h
	LCDOUT?C	053h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	053h
	LCDOUT?C	041h
	LCDOUT?C	056h
	LCDOUT?C	045h
	LCDOUT?C	044h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01791	        lcdout $fe,$c0, "                     "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
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
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01792	        pause 1500
	PAUSE?C	005DCh

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01793	return
	RETURN?	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01795	PrviPut:

	LABEL?L	_PrviPut	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01796	      write 1,0
	WRITEADDRESS?C	001h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01797	      write 2,10
	WRITEADDRESS?C	002h
	WRITE?C	00Ah

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01798	      write 3,2
	WRITEADDRESS?C	003h
	WRITE?C	002h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01799	      write 4,1
	WRITEADDRESS?C	004h
	WRITE?C	001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01801	      write 5,1
	WRITEADDRESS?C	005h
	WRITE?C	001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01802	      write 6,1
	WRITEADDRESS?C	006h
	WRITE?C	001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01803	      write 7,75
	WRITEADDRESS?C	007h
	WRITE?C	04Bh

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01805	      write 8,6
	WRITEADDRESS?C	008h
	WRITE?C	006h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01806	      write 9,0
	WRITEADDRESS?C	009h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01807	      write 10,0
	WRITEADDRESS?C	00Ah
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01810	      write 12,8
	WRITEADDRESS?C	00Ch
	WRITE?C	008h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01811	      write 13,0
	WRITEADDRESS?C	00Dh
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01812	      write 14,100
	WRITEADDRESS?C	00Eh
	WRITE?C	064h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01814	      write 16,18    
	WRITEADDRESS?C	010h
	WRITE?C	012h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01815	      write 17,0
	WRITEADDRESS?C	011h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01816	      write 18,100
	WRITEADDRESS?C	012h
	WRITE?C	064h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01818	      write 20,20
	WRITEADDRESS?C	014h
	WRITE?C	014h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01819	      write 21,0
	WRITEADDRESS?C	015h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01820	      write 22,0
	WRITEADDRESS?C	016h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01822	      write 24,6
	WRITEADDRESS?C	018h
	WRITE?C	006h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01823	      write 25,0
	WRITEADDRESS?C	019h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01824	      write 26,0
	WRITEADDRESS?C	01Ah
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01826	      write 28,8
	WRITEADDRESS?C	01Ch
	WRITE?C	008h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01827	      write 29,0
	WRITEADDRESS?C	01Dh
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01828	      write 30,100
	WRITEADDRESS?C	01Eh
	WRITE?C	064h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01830	      write 32,18    
	WRITEADDRESS?C	020h
	WRITE?C	012h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01831	      write 33,0
	WRITEADDRESS?C	021h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01832	      write 34,100
	WRITEADDRESS?C	022h
	WRITE?C	064h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01834	      write 36,20
	WRITEADDRESS?C	024h
	WRITE?C	014h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01835	      write 37,0
	WRITEADDRESS?C	025h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01836	      write 38,0
	WRITEADDRESS?C	026h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01838	      write 40,6
	WRITEADDRESS?C	028h
	WRITE?C	006h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01839	      write 41,0
	WRITEADDRESS?C	029h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01840	      write 42,0
	WRITEADDRESS?C	02Ah
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01842	      write 44,8
	WRITEADDRESS?C	02Ch
	WRITE?C	008h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01843	      write 45,0
	WRITEADDRESS?C	02Dh
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01844	      write 46,100
	WRITEADDRESS?C	02Eh
	WRITE?C	064h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01846	      write 48,18    
	WRITEADDRESS?C	030h
	WRITE?C	012h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01847	      write 49,0
	WRITEADDRESS?C	031h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01848	      write 50,100
	WRITEADDRESS?C	032h
	WRITE?C	064h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01850	      write 52,20
	WRITEADDRESS?C	034h
	WRITE?C	014h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01851	      write 53,0
	WRITEADDRESS?C	035h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01852	      write 54,0
	WRITEADDRESS?C	036h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01854	      write 56,6
	WRITEADDRESS?C	038h
	WRITE?C	006h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01855	      write 57,0
	WRITEADDRESS?C	039h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01856	      write 58,0
	WRITEADDRESS?C	03Ah
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01858	      write 60,8
	WRITEADDRESS?C	03Ch
	WRITE?C	008h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01859	      write 61,0
	WRITEADDRESS?C	03Dh
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01860	      write 62,100
	WRITEADDRESS?C	03Eh
	WRITE?C	064h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01862	      write 64,18    
	WRITEADDRESS?C	040h
	WRITE?C	012h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01863	      write 65,0
	WRITEADDRESS?C	041h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01864	      write 66,100
	WRITEADDRESS?C	042h
	WRITE?C	064h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01866	      write 68,20
	WRITEADDRESS?C	044h
	WRITE?C	014h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01867	      write 69,0
	WRITEADDRESS?C	045h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01868	      write 70,0
	WRITEADDRESS?C	046h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01874	      write 74,0
	WRITEADDRESS?C	04Ah
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01875	      write 75,0
	WRITEADDRESS?C	04Bh
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01878	      write 76,0
	WRITEADDRESS?C	04Ch
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01879	      write 77,0
	WRITEADDRESS?C	04Dh
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01881	      write 78,0
	WRITEADDRESS?C	04Eh
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01882	      write 79,0
	WRITEADDRESS?C	04Fh
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01884	      write 80,0
	WRITEADDRESS?C	050h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01885	      write 81,0
	WRITEADDRESS?C	051h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01887	      write 82,0
	WRITEADDRESS?C	052h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01888	      write 83,0
	WRITEADDRESS?C	053h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01890	      write 84,0
	WRITEADDRESS?C	054h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01891	      write 85,0
	WRITEADDRESS?C	055h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01893	      write 86,0
	WRITEADDRESS?C	056h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01894	      write 87,0
	WRITEADDRESS?C	057h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01896	      write 88,0
	WRITEADDRESS?C	058h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01897	      write 89,0
	WRITEADDRESS?C	059h
	WRITE?C	000h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01899	      write 100,10
	WRITEADDRESS?C	064h
	WRITE?C	00Ah

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01900	      write 101,10
	WRITEADDRESS?C	065h
	WRITE?C	00Ah

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01901	      write 102,10
	WRITEADDRESS?C	066h
	WRITE?C	00Ah

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01902	      write 103,10
	WRITEADDRESS?C	067h
	WRITE?C	00Ah

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01904	      relej1 = 0 
	MOVE?CT	000h, _RELEJ1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01905	      relej2 = 0 
	MOVE?CT	000h, _RELEJ2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01906	      DutyCycleL1 = 0
	MOVE?CN	000h, _DutyCycleL1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01907	      DutyCycleL2 = 0
	MOVE?CN	000h, _DutyCycleL2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01908	      DutyCycleL3 = 0
	MOVE?CN	000h, _DutyCycleL3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01909	      DutyCycleL4 = 0
	MOVE?CN	000h, _DutyCycleL4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01910	      @ HPWM10 2, _DutyCycleL4, _Frequency

	ASM?
 HPWM10 2, _DutyCycleL4, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01911	      @ HPWM10 4, _DutyCycleL1, _Frequency

	ASM?
 HPWM10 4, _DutyCycleL1, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01912	      @ HPWM10 3, _DutyCycleL2, _Frequency

	ASM?
 HPWM10 3, _DutyCycleL2, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01913	      @ HPWM10 1, _DutyCycleL3, _Frequency

	ASM?
 HPWM10 1, _DutyCycleL3, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01914	      x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01915	      goto RTCvreme
	GOTO?L	_RTCvreme

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01916	return
	RETURN?	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01918	UzmiVreme:

	LABEL?L	_uzmivreme	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01922	    i2cread sda,scl,rtc,$02,[seconds,minutes,hours]
	I2CDATA?T	_SDA
	I2CCLOCK?T	_SCL
	I2CWRITE?C	_RTC
	I2CWRITE?C	002h
	I2CREAD?B	_Seconds
	I2CREAD?B	_Minutes
	I2CREADS?B	_Hours

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01923	    minutes.7=0:hours.7=0:hours.6=0
	MOVE?CT	000h, _Minutes??7

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01923	    minutes.7=0:hours.7=0:hours.6=0
	MOVE?CT	000h, _Hours??7

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01923	    minutes.7=0:hours.7=0:hours.6=0
	MOVE?CT	000h, _Hours??6

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01924	    DecMinutGlavni = ((minutes  & %11110000)>>4)  * 10 + (minutes & %00001111)
	AND?BCN	_Minutes, 0F0h, T1
	SHIFTR?NCN	T1, 004h, T1
	MUL?NCN	T1, 00Ah, T1
	AND?BCN	_Minutes, 00Fh, T2
	ADD?NNB	T1, T2, _DecMinutGlavni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01925	    decsatglavni = ((Hours & %11110000)>>4)  * 10 + (hours & %00001111)
	AND?BCN	_Hours, 0F0h, T1
	SHIFTR?NCN	T1, 004h, T1
	MUL?NCN	T1, 00Ah, T1
	AND?BCN	_Hours, 00Fh, T2
	ADD?NNB	T1, T2, _DecSatglavni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01926	    decsec = ((Seconds  & %11110000)>>4)  * 10 + (seconds  & %00001111)
	AND?BCN	_Seconds, 0F0h, T1
	SHIFTR?NCN	T1, 004h, T1
	MUL?NCN	T1, 00Ah, T1
	AND?BCN	_Seconds, 00Fh, T2
	ADD?NNB	T1, T2, _DecSec

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01927	    pause 10
	PAUSE?C	00Ah

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01929	return
	RETURN?	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01931	RTCvreme:

	LABEL?L	_RTCvreme	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01932	gosub OcitajTastere
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01933	 read 1,temp
	READADDRESS?C	001h
	READ?W	_Temp

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01934	 if temp = 1 then gosub kreni
	CMPNE?WCL	_Temp, 001h, L00643
	GOSUB?L	_kreni
	LABEL?L	L00643	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01936	     if desno = 1 then
	CMPNE?TCL	_desno, 001h, L00645

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01937	        x =  x + 1 
	ADD?BCB	_x, 001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01938	        if x > 2 then x = 1
	CMPLE?BCL	_x, 002h, L00647
	MOVE?CB	001h, _x
	LABEL?L	L00647	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01939	     endif
	LABEL?L	L00645	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01940	     if levo = 1 then 
	CMPNE?TCL	_levo, 001h, L00649

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01941	        read 1,temp
	READADDRESS?C	001h
	READ?W	_Temp

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01942	        if temp = 1 then 
	CMPNE?WCL	_Temp, 001h, L00651

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01943	            x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01944	            goto Alat
	GOTO?L	_Alat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01945	        endif
	LABEL?L	L00651	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01946	     endif
	LABEL?L	L00649	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01947	     select case x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01948	        case 1 
	CMPNE?BCL	_x, 001h, L00655

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01949	             displej[1] = strelica
	MOVE?CB	_Strelica, _displej + 00001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01950	             displej[2] = " "
	MOVE?CB	020h, _displej + 00002h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01951	        case 2 
	GOTO?L	L00654
	LABEL?L	L00655	
	CMPNE?BCL	_x, 002h, L00656

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01952	             displej[1] = " "
	MOVE?CB	020h, _displej + 00001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01953	             displej[2] = strelica
	MOVE?CB	_Strelica, _displej + 00002h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01954	     end select
	LABEL?L	L00656	
	LABEL?L	L00654	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01955	     if tastgore= 0 then
	CMPNE?TCL	_TASTGORE, 000h, L00657

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01956	         if x = 1 then
	CMPNE?BCL	_x, 001h, L00659

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01957	            decsaTglavni = decsaTglavni + 1
	ADD?BCB	_DecSatglavni, 001h, _DecSatglavni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01958	            if decsaTglavni > 23 then decsaTglavni = 23
	CMPLE?BCL	_DecSatglavni, 017h, L00661
	MOVE?CB	017h, _DecSatglavni
	LABEL?L	L00661	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01959	            pause PauzaZaTaster 
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01960	         endif
	LABEL?L	L00659	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01962	         if x = 2 then
	CMPNE?BCL	_x, 002h, L00663

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01963	            decminutglavni = decminutglavni + 1
	ADD?BCB	_DecMinutGlavni, 001h, _DecMinutGlavni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01964	            if decminutglavni > 59 then decminutglavni = 59
	CMPLE?BCL	_DecMinutGlavni, 03Bh, L00665
	MOVE?CB	03Bh, _DecMinutGlavni
	LABEL?L	L00665	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01965	            pause PauzaZaTaster 
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01966	         endif
	LABEL?L	L00663	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01967	     endif
	LABEL?L	L00657	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01968	     if tastdole = 0 then
	CMPNE?TCL	_TASTDOLE, 000h, L00667

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01969	         if x = 1 then
	CMPNE?BCL	_x, 001h, L00669

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01970	           decsaTglavni = decsaTglavni - 1
	SUB?BCB	_DecSatglavni, 001h, _DecSatglavni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01971	            if decsaTglavni < 1 or decsaTglavni > 23 then decsaTglavni = 0
	CMPLT?BCB	_DecSatglavni, 001h, T1
	CMPGT?BCB	_DecSatglavni, 017h, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00671
	MOVE?CB	000h, _DecSatglavni
	LABEL?L	L00671	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01972	            pause PauzaZaTaster 
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01973	         endif
	LABEL?L	L00669	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01975	         if x = 2 then
	CMPNE?BCL	_x, 002h, L00673

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01976	            decminutglavni = decminutglavni - 1
	SUB?BCB	_DecMinutGlavni, 001h, _DecMinutGlavni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01977	            if decminutglavni < 1 or decminutglavni > 59 then decminutglavni = 0   
	CMPLT?BCB	_DecMinutGlavni, 001h, T1
	CMPGT?BCB	_DecMinutGlavni, 03Bh, T2
	LOR?BBN	T1, T2, T2
	CMPF?NL	T2, L00675
	MOVE?CB	000h, _DecMinutGlavni
	LABEL?L	L00675	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01978	            pause PauzaZaTaster 
	PAUSE?C	_PauzaZaTaster

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01979	         endif
	LABEL?L	L00673	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01980	     endif
	LABEL?L	L00667	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01981	     if ok = 1 then
	CMPNE?TCL	_ok, 001h, L00677

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01982	         Minutes  = (decminutglavni dig 1) * 16 + decminutglavni dig 0
	DIG?BCN	_DecMinutGlavni, 001h, T1
	MUL?NCN	T1, 010h, T1
	DIG?BCN	_DecMinutGlavni, 000h, T2
	ADD?NNB	T1, T2, _Minutes

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01983	         Hours = (decsaTglavni dig 1) * 16 + decsaTglavni dig 0
	DIG?BCN	_DecSatglavni, 001h, T1
	MUL?NCN	T1, 010h, T1
	DIG?BCN	_DecSatglavni, 000h, T2
	ADD?NNB	T1, T2, _Hours

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01984	         Seconds = 0
	MOVE?CB	000h, _Seconds

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01985	         I2cwrite sda,scl,rtc,$02,[seconds,minutes,hours,$01,$02,$01,$01]
	I2CDATA?T	_SDA
	I2CCLOCK?T	_SCL
	I2CWRITE?C	_RTC
	I2CWRITE?C	002h
	I2CWRITE?B	_Seconds
	I2CWRITE?B	_Minutes
	I2CWRITE?B	_Hours
	I2CWRITE?C	001h
	I2CWRITE?C	002h
	I2CWRITE?C	001h
	I2CWRITES?C	001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01986	         pause 10
	PAUSE?C	00Ah

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01987	         x = 1
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01988	         gosub Sacuvano
	GOSUB?L	_sacuvano

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01989	         read 1,x
	READADDRESS?C	001h
	READ?B	_x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01990	         if x <> 1 then 
	CMPEQ?BCL	_x, 001h, L00679

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01991	            x = 1 
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01992	            write 1,1
	WRITEADDRESS?C	001h
	WRITE?C	001h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01993	            goto splash
	GOTO?L	_Splash

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01994	         else
	GOTO?L	L00680
	LABEL?L	L00679	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01995	            x = 1 
	MOVE?CB	001h, _x

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01996	            goto Alat
	GOTO?L	_Alat

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01997	         endif
	LABEL?L	L00680	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	01998	     ENDIF 
	LABEL?L	L00677	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02000	   Lcdout $fe, 2,"System time            "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	053h
	LCDOUT?C	079h
	LCDOUT?C	073h
	LCDOUT?C	074h
	LCDOUT?C	065h
	LCDOUT?C	06Dh
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
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02001	   lcdout $fe,$c0,displej[1],dec2 decsaTglavni,":",displej[2],dec2 decminutglavni,"              "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?B	_displej + 00001h
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_DecSatglavni
	LCDOUTDEC?	
	LCDOUT?C	03Ah
	LCDOUT?B	_displej + 00002h
	LCDOUTCOUNT?C	002h
	LCDOUTNUM?B	_DecMinutGlavni
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

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02002	goto RTCVreme
	GOTO?L	_RTCvreme

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02004	IzmeriTemperaturu:

	LABEL?L	_IzmeriTemperaturu	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02005	i2cread sda, scl, lm75,0,[temperatura]
	I2CDATA?T	_SDA
	I2CCLOCK?T	_SCL
	I2CWRITE?C	_LM75
	I2CWRITE?C	000h
	I2CREADS?B	_Temperatura

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02006	pause 20
	PAUSE?C	014h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02007	i2cread sda, scl, lm75,0,[smece]
	I2CDATA?T	_SDA
	I2CCLOCK?T	_SCL
	I2CWRITE?C	_LM75
	I2CWRITE?C	000h
	I2CREADS?W	_smece

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02009	read 7,TemperaturaIzEproma
	READADDRESS?C	007h
	READ?B	_TemperaturaIzEproma

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02010	if temperatura > TemperaturaIzEproma then 
	CMPLE?BBL	_Temperatura, _TemperaturaIzEproma, L00681

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02012	    DutyCycleL1 = 0
	MOVE?CN	000h, _DutyCycleL1

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02013	    DutyCycleL2 = 0
	MOVE?CN	000h, _DutyCycleL2

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02014	    DutyCycleL3 = 0
	MOVE?CN	000h, _DutyCycleL3

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02015	    DutyCycleL4 = 0
	MOVE?CN	000h, _DutyCycleL4

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02017	    @ HPWM10 2, _DutyCycleL4, _Frequency

	ASM?
 HPWM10 2, _DutyCycleL4, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02018	    @ HPWM10 4, _DutyCycleL1, _Frequency

	ASM?
 HPWM10 4, _DutyCycleL1, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02019	    @ HPWM10 3, _DutyCycleL2, _Frequency

	ASM?
 HPWM10 3, _DutyCycleL2, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02020	    @ HPWM10 1, _DutyCycleL3, _Frequency

	ASM?
 HPWM10 1, _DutyCycleL3, _Frequency

	ENDASM?


; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02021	    Pregrejan = 1
	MOVE?CB	001h, _pregrejan

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02022	    gosub PregrejanEkran
	GOSUB?L	_PregrejanEkran

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02023	else
	GOTO?L	L00682
	LABEL?L	L00681	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02024	    pregrejan = 0
	MOVE?CB	000h, _pregrejan

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02025	endif
	LABEL?L	L00682	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02026	return
	RETURN?	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02028	PregrejanEkran:

	LABEL?L	_PregrejanEkran	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02029	bcklight = 1
	MOVE?CT	001h, _BCKLIGHT

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02030	    gosub kreni
	GOSUB?L	_kreni

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02031	    gosub ocitajtastere  
	GOSUB?L	_ocitajtastere

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02032	    Lcdout $fe, 2,"  !OVERHEATED!  "
	LCDOUT?C	0FEh
	LCDOUT?C	002h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	021h
	LCDOUT?C	04Fh
	LCDOUT?C	056h
	LCDOUT?C	045h
	LCDOUT?C	052h
	LCDOUT?C	048h
	LCDOUT?C	045h
	LCDOUT?C	041h
	LCDOUT?C	054h
	LCDOUT?C	045h
	LCDOUT?C	044h
	LCDOUT?C	021h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02033	    lcdout $fe,$c0, ">PRESS OK        "
	LCDOUT?C	0FEh
	LCDOUT?C	0C0h
	LCDOUT?C	03Eh
	LCDOUT?C	050h
	LCDOUT?C	052h
	LCDOUT?C	045h
	LCDOUT?C	053h
	LCDOUT?C	053h
	LCDOUT?C	020h
	LCDOUT?C	04Fh
	LCDOUT?C	04Bh
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h
	LCDOUT?C	020h

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02034	    if ok = 1 then goto main 
	CMPNE?TCL	_ok, 001h, L00683
	GOTO?L	_main
	LABEL?L	L00683	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02035	goto PregrejanEkran
	GOTO?L	_PregrejanEkran

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02037	EEpromMemorija:

	LABEL?L	_EEpromMemorija	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02038	    for i = 8 to 70 step 1
	MOVE?CB	008h, _i
	LABEL?L	L00685	
	CMPGT?BCL	_i, 046h, L00686

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02039	        memorija[i] = 0
	AIN?CBB	000h, _Memorija, _i

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02040	        read i,memorija[i]   
	READADDRESS?B	_i
	READ?N	T1
	AIN?BBB	T1, _Memorija, _i

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02041	    next i
	NEXT?BCL	_i, 001h, L00685
	LABEL?L	L00686	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02043	    for i = 1 to 16 step 1 
	MOVE?CB	001h, _i
	LABEL?L	L00687	
	CMPGT?BCL	_i, 010h, L00688

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02044	        RelejiMemorija[i] = 0
	AIN?CBB	000h, _RelejiMemorija, _i

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02045	        read 73 + i,RelejiMemorija[i]
	ADD?CBN	049h, _i, T1
	READADDRESS?N	T1
	READ?N	T1
	AIN?BBB	T1, _RelejiMemorija, _i

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02046	    next i         
	NEXT?BCL	_i, 001h, L00687
	LABEL?L	L00688	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02047	return
	RETURN?	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02049	OcitajTastere:

	LABEL?L	_ocitajtastere	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02050	    NovoStanjeDesno         =  tastdesno
	MOVE?TT	_TASTDESNO, _NovoStanjeDesno

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02051	    NovoStanjeLevo          =  tastlevo
	MOVE?TT	_TASTLEVO, _NovoStanjeLevo

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02052	    NovoStanjeGore          =  tastgore
	MOVE?TT	_TASTGORE, _NovoStanjeGore

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02053	    NovoStanjeDole          =  tastdole
	MOVE?TT	_TASTDOLE, _NovoStanjeDole

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02054	    NovoStanjeOk            =  tastok
	MOVE?TT	_TASTOK, _NovoStanjeOk

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02055	    NovoStanjeOdrzavanje    =  tastodrzavanje
	MOVE?TT	_TASTODRZAVANJE, _NovoStanjeOdrzavanje

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02057	    if novostanjegore < starostanjegore then 
	CMPGE?TTL	_NovoStanjeGore, _StaroStanjeGore, L00689

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02058	        gore = 1 
	MOVE?CT	001h, _gore

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02059	    else
	GOTO?L	L00690
	LABEL?L	L00689	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02060	        gore = 0 
	MOVE?CT	000h, _gore

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02061	    ENDIF
	LABEL?L	L00690	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02063	    if novostanjedole < starostanjedole then 
	CMPGE?TTL	_NovoStanjeDole, _StaroStanjeDole, L00691

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02064	        dole = 1
	MOVE?CT	001h, _dole

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02065	    else 
	GOTO?L	L00692
	LABEL?L	L00691	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02066	        dole = 0
	MOVE?CT	000h, _dole

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02067	    ENDIF
	LABEL?L	L00692	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02069	    if novostanjelevo < starostanjelevo then 
	CMPGE?TTL	_NovoStanjeLevo, _StaroStanjeLevo, L00693

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02070	        levo = 1
	MOVE?CT	001h, _levo

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02071	    else
	GOTO?L	L00694
	LABEL?L	L00693	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02072	        levo = 0
	MOVE?CT	000h, _levo

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02073	    ENDIF
	LABEL?L	L00694	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02075	    if novostanjedesno < starostanjedesno then 
	CMPGE?TTL	_NovoStanjeDesno, _StaroStanjeDesno, L00695

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02076	        desno = 1
	MOVE?CT	001h, _desno

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02077	    else
	GOTO?L	L00696
	LABEL?L	L00695	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02078	        desno = 0
	MOVE?CT	000h, _desno

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02079	    ENDIF
	LABEL?L	L00696	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02081	    if novostanjeOK < starostanjeok then 
	CMPGE?TTL	_NovoStanjeOk, _StaroStanjeOk, L00697

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02082	        ok = 1
	MOVE?CT	001h, _ok

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02083	    else
	GOTO?L	L00698
	LABEL?L	L00697	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02084	        ok = 0
	MOVE?CT	000h, _ok

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02085	    ENDIF
	LABEL?L	L00698	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02087	    if novostanjeodrzavanje < starostanjeodrzavanje then 
	CMPGE?TTL	_NovoStanjeOdrzavanje, _StaroStanjeOdrzavanje, L00699

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02088	        odrzavanje = 1
	MOVE?CT	001h, _odrzavanje

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02089	    else
	GOTO?L	L00700
	LABEL?L	L00699	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02090	        odrzavanje = 0
	MOVE?CT	000h, _odrzavanje

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02091	    ENDIF
	LABEL?L	L00700	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02093	    StaroStanjeDesno        = NovoStanjeDesno        
	MOVE?TT	_NovoStanjeDesno, _StaroStanjeDesno

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02094	    StaroStanjeLevo         = NovoStanjeLevo      
	MOVE?TT	_NovoStanjeLevo, _StaroStanjeLevo

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02095	    StaroStanjeGore         = NovoStanjeGore          
	MOVE?TT	_NovoStanjeGore, _StaroStanjeGore

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02096	    StaroStanjeDole         = NovoStanjeDole         
	MOVE?TT	_NovoStanjeDole, _StaroStanjeDole

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02097	    StaroStanjeOk           = NovoStanjeOk           
	MOVE?TT	_NovoStanjeOk, _StaroStanjeOk

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02098	    StaroStanjeOdrzavanje   = NovoStanjeOdrzavanje    
	MOVE?TT	_NovoStanjeOdrzavanje, _StaroStanjeOdrzavanje

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02099	return
	RETURN?	

; C:\USERS\ADMIN\DESKTOP\PROGRAM_D_FULL\DANER_V3_FULL.PBP	02100	end 
	END?	

	END
