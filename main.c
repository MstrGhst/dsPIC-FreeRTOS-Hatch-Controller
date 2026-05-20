#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "BlockQ.h"
#include "crflash.h"
#include "blocktim.h"
#include "integer.h"
#include "comtest2.h"
#include "partest.h"
//#include "lcd.h"
#include "timertest.h"
#include "new_lcd.h"
#include "new_serial.h"
#include "libq.h"
//#include "serial.h"
#include "p33Fxxxx.h"
#include "ds18b20.h"

#define mainBLOCK_Q_PRIORITY ( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY ( tskIDLE_PRIORITY + 3 )
#define mainCOM_TEST_PRIORITY ( 2 )
#define mainCHECK_TAKS_STACK_SIZE ( configMINIMAL_STACK_SIZE * 2 )
#define mainCHECK_TASK_PERIOD ( ( portTickType ) 3000 / portTICK_RATE_MS )
#define mainNUM_FLASH_COROUTINES ( 5 )

//#define mainCOM_TEST_BAUD_RATE ( 19200 )
#define mainCOM_TEST_BAUD_RATE ( 9600 )
#define comBUFFER_LEN ( 200 )
#define comNO_BLOCK ( ( portTickType ) 0 )
#define comRX_BLOCK_TIME ( ( portTickType ) 0xffff )
#define mainCOM_TEST_LED ( 6 )
#define mainTEST_INTERRUPT_FREQUENCY ( 20000 )
#define mainEXPECTED_CLOCKS_BETWEEN_INTERRUPTS ( configCPU_CLOCK_HZ / mainTEST_INTERRUPT_FREQUENCY )
#define mainNS_PER_CLOCK ( ( unsigned short ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )
#define mainMAX_STRING_LENGTH ( 20 )

// pozitiile servomotorului (min, centru, max)
#define PWM_MIN    625  
#define PWM_CENTER 937  
#define PWM_MAX    1250  

_FOSCSEL(FNOSC_FRC);
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF);
_FWDT(FWDTEN_OFF);

static void prvSetupHardware( void );
static xQueueHandle xUART1_Queue;

// prototipurile functiilor
void initPLL(void);
void initTemp(void);
void initPWM(void);
void initAdc1(void);
void initTmr3(void);
char* floatToString(float toConvert);

// declarare de variabile globale
volatile int flag=1; // flag pentru starea aplicatiei (1=pornit, 0=oprit)
float t; 	// variabila pt temperatura
volatile unsigned int adcRawValue = 0; //variabila pt convertorul adc
volatile float currentVoltage = 0.0;
volatile int isAutoMode = 1;
volatile char lastCommand = '-'; 

// functie pt transformarea din float in strig pt functia de temperatura
char* floatToString(float toConvert) {
    static char sir[10];
    sprintf(sir, "%.1f C ", (double)toConvert);
    return sir;
}

// functie care pregateste modulul ADC pt a putea fi folosit
void initAdc1(void)
{
    AD1CON1bits.ADON = 0;  
    AD1CON1bits.AD12B = 1;
    AD1CON1bits.SSRC = 2;
    AD1CON1bits.ASAM = 1;
    AD1CON2bits.CSCNA = 1;
    AD1CON3bits.ADRC = 0;
    AD1CON3bits.ADCS = 63;
    AD1CSSL = 0x0020;
    AD1PCFGL = 0xFFFF;      
    AD1PCFGLbits.PCFG5 = 0;
    _AD1IF = 0;
    _AD1IE = 1;
    AD1CON1bits.ADON = 1;  
}

void initTmr3(void)
{
    TMR3 = 0;
    PR3 = 50000;
    T3CONbits.TON = 1;
}

void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void)
{
    adcRawValue = ADC1BUF0;
    _AD1IF = 0;
}

// functie care pregateste servomotorul pt a putea fi folosit
void initPWM(void)
{
    /* PWM1H3 (ex. RB10): Motor Control PWM modul 1, generator 3.
     * NU folosi OC1RS aici ? duty se scrie in P1DC3. */
    P1FLTACON = 0;
    P1OVDCON  = 0x3F00;

    P1TCON = 0;
    P1TMR  = 0;
    P1TCONbits.PTOPS  = 0;
    P1TCONbits.PTMOD  = 0;
    P1TCONbits.PTCKPS = 3;

    P1TPER = 12500;
    P1DC3  = PWM_CENTER;

    PWM1CON1 = 0;
    PWM1CON1bits.PMOD3 = 1;
    PWM1CON1bits.PEN3H = 1;
    PWM1CON1bits.PEN3L = 0;

    PWM1CON2 = 0;
    PWM1CON2bits.IUE = 1;

    _TRISB10 = 0;
    _LATB10  = 0;

    P1TCONbits.PTEN = 1;
}

void initPLL(void) {
    PLLFBD = 41;
    CLKDIVbits.PLLPOST = 0;
    CLKDIVbits.PLLPRE = 0;
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(0x01);
    while (OSCCONbits.COSC != 0b001);
    while (OSCCONbits.LOCK != 1);
}

void Task_Buton(void *params) // Task-ul care se ocupa de butonul S2
{
    _TRISB7=1; // setam pinul RB7 ca intrare
    for(;;)
    {
        if(_RB7==0) // daca s-a apasat butonul
        {
            vTaskDelay(20/portTICK_RATE_MS); // pauza pentru debounce
          
            if (_RB7==0) // verificam iar daca e inca apasat
            {
                // schimbam starea aplicatiei
                if (flag==0)
				{
                    flag=1;
				}
				else
				{
                    flag=0;
				}
               
                while(_RB7==0) // bucla care asteapta ca butonul sa nu mai fie apasat
				{
                    vTaskDelay(10/portTICK_RATE_MS);
				}
            }
        }
       
        vTaskDelay(10/portTICK_RATE_MS);
    }
}

void Task_Led(void *params) // Task-ul care aprinde continuu sau intermitent led-ul
{
    _TRISB11=0;// setam pinul RB11 ca iesire

    for(;;)
    {
        if(flag==1) // daca aplicatia e pornita
        {
            _RB11=0; // led-ul sta aprins
            vTaskDelay(50/portTICK_RATE_MS);
        }
        else // daca aplicatia e oprita
        {
            _RB11=!(_RB11); // led-ul isi schimba starea (intermitent)
            vTaskDelay(500/portTICK_RATE_MS);
        }
    }
}

void Task_Temperatura(void *params)
{
 	CNPU1 = 0x0040; // activeaza rezisten?a interna de pull-up
 	output_float();
 	ONE_WIRE_PIN = 1; // seteaza starea logica ini?iala a pinului la 1 (HIGH)

	for (;;)
	{
		if(flag==1) // citim temperatura doar daca e pornita aplicatia
		{
			t=ds1820_read();
    		vTaskSuspendAll();
			LCD_Goto(1,6);
			LCD_printf(floatToString(t));
			xTaskResumeAll();
		}
		vTaskDelay(500/portTICK_RATE_MS);
	}
}

void Task_Tensiune(void *params)
{
    char sirTensiune[15]; //vector de caractere (un buffer) pentru a stoca valoarea tensiunii transformata din numar în text

    for (;;)
    {
        if(flag==1) //daca aplicatia este pornita
        {
            currentVoltage = ((float)adcRawValue / 4095.0) * 3.3; // conversia matematica a semnalului
            sprintf(sirTensiune, "%.2f V", (double)currentVoltage);
           
			//afisare pe LCD
            vTaskSuspendAll();
            LCD_Goto(3, 6);
            LCD_printf(sirTensiune);
            xTaskResumeAll();
        }
        vTaskDelay(300 / portTICK_RATE_MS);
    }
}

void Task_Servomotor(void *params)
{
    unsigned int pwmDutyCycle = PWM_CENTER;//var locala care va stoca factorul de umplere (implicit corespunde poz de mijloc)
    for (;;)
    {
        if(flag) //daca aplicatia e pornita 
        {
            if (isAutoMode == 1)
            {
                if (t <= 20.0) pwmDutyCycle = PWM_MIN; //trapa se afla pe pozitia minima daca temp <= 20 grade
                else if (t >= 30.0) pwmDutyCycle = PWM_MAX;
                else 
				{ 	//calculam poz intermediara a trapei
                    pwmDutyCycle = PWM_MIN + (unsigned int)(((t - 20.0) * (PWM_MAX - PWM_MIN)) / 10.0);
                }
            }
            else //daca ne aflam in modul manual
            {
                if (currentVoltage <= 1.0) pwmDutyCycle = PWM_MIN; //trapa se afla pe pozitia minima daca volt <= 1V
                else if (currentVoltage >= 3.0) pwmDutyCycle = PWM_MAX;
                else 
				{ 	//calculam poz intermediara a trapei
                    pwmDutyCycle = PWM_MIN + (unsigned int)(((currentVoltage - 1.0) * (PWM_MAX - PWM_MIN)) / 2.0);
                }
            }
            P1DC3 = pwmDutyCycle; //comanda "fizica" de actionare a trapei
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

void TaskRefreshSerial(void *params)
{
    signed char cByteRxed; char cmdDisplay[16]; char outStr[64]; //var locale 
	
	//meniul initial de comenzi pt utilizator care va aparea pe interfata seriala
    vSerialPutString(NULL, (signed char *)"1. Interogare Temperatura\r\n", comNO_BLOCK);
    vSerialPutString(NULL, (signed char *)"2. Comutare Mod (Auto/Manual)\r\n", comNO_BLOCK);
    vSerialPutString(NULL, (signed char *)"3. Interogare mod lucru\r\n", comNO_BLOCK);

    for (;;)
    {
        if (flag) 
        {
            if(xSerialGetChar(NULL, &cByteRxed, portMAX_DELAY)) //daca am primit o comanda de la utilizator
            {	// salvam si afisam pe LCD ultima comanda a utilizatorului
                lastCommand = cByteRxed;
                vTaskSuspendAll();
                LCD_Goto(4, 1);
                sprintf(cmdDisplay, "Ultm cmd: %c", lastCommand);
                LCD_printf(cmdDisplay);
                xTaskResumeAll();
                vSerialPutString(NULL, (signed char *)"\r\n", comNO_BLOCK);

                if(cByteRxed == '1') 
                {   //daca utilizatorul a apasat 1 trimite temp curenta catre interfata seriala
                    sprintf(outStr, "TEMP: %.1f C\r\n\n", (double)t);
                    vSerialPutString(NULL, (signed char *)outStr, comNO_BLOCK);
                }
                else if(cByteRxed == '2')
                {	//daca utilizatorul a apasat 2 comuta modul de lucru si afiseaza pe LCD si pe interfata seriala
                    isAutoMode = !isAutoMode;
                    _RB12 = isAutoMode; //pune pinul RB12 în starea HIGH (1) când modul este auto (LED aprins) si în stare LOW (0) când modul este manual (LED stins)
                    vTaskSuspendAll();
                    LCD_Goto(2, 1);
                    if(isAutoMode) LCD_printf("Mod lucru:AUTO  ");
                    else LCD_printf("Mod lucru:MANUAL");
                    xTaskResumeAll();
                    sprintf(outStr, "MOD SCHIMBAT: %s\r\n\n", isAutoMode ? "AUTO" : "MANUAL");
                    vSerialPutString(NULL, (signed char *)outStr, comNO_BLOCK);
                }
                else if(cByteRxed == '3')
                {	//daca utilizatorul a apasat 3 trimite temp curenta catre interfata seriala 
                    sprintf(outStr, "Mod lucru: %s\r\n\n", isAutoMode ? "AUTO" : "MANUAL");
                    vSerialPutString(NULL, (signed char *)outStr, comNO_BLOCK);
                }
                else
                {	//daca utilizatorul a apasat orice altceva afiseaza un mesaj de avertizare
                    vSerialPutString(NULL, (signed char *)"COMANDA INVALIDA\r\n\n", comNO_BLOCK);
                }

				//afiseaza din nou meniul principal indiferent care care a fost ultima comanda procesata
                vSerialPutString(NULL, (signed char *)"1. Interogare Temperatura\r\n", comNO_BLOCK);
    			vSerialPutString(NULL, (signed char *)"2. Comutare Mod (Auto/Manual)\r\n", comNO_BLOCK);
    			vSerialPutString(NULL, (signed char *)"3. Interogare mod lucru\r\n", comNO_BLOCK);
            }
        }
        else vTaskDelay(200 / portTICK_RATE_MS); 
} 
}

int main( void )
{
prvSetupHardware();

//Crearea task-urilor pt ficare functie
xTaskCreate(Task_Buton,"Buton",256,NULL,tskIDLE_PRIORITY+4,NULL);
xTaskCreate(Task_Led,"LED",256,NULL,tskIDLE_PRIORITY+4,NULL);
xTaskCreate(Task_Temperatura, "Temp", 256, NULL, tskIDLE_PRIORITY + 3, NULL);
xTaskCreate(Task_Tensiune,"Tens",256, NULL, tskIDLE_PRIORITY + 3, NULL);
xTaskCreate(Task_Servomotor,"Servomotor",  256, NULL, tskIDLE_PRIORITY + 3, NULL);
xTaskCreate(TaskRefreshSerial,"Serial", 512, NULL, tskIDLE_PRIORITY + 2, NULL);

/* Finally start the scheduler. */
vTaskStartScheduler();

return 0;
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{

ADPCFG = 0xFFFF; //make ADC pins all digital - adaugat

	_TRISB12 = 0;
	_TRISB11 = 0;
	_RB12 = 1;

vParTestInitialise();
initTmr3();
initAdc1();
initPWM();    
initPLL();

LCD_init();
LCD_line(1);
LCD_printf("Temp:");
LCD_line(2);
LCD_printf("Mod lucru:AUTO");
LCD_line(3);
LCD_printf("Tens:");
LCD_line(4);
LCD_printf("Ultm cmd:");

// Initializare interfata UART1
xSerialPortInitMinimal( mainCOM_TEST_BAUD_RATE, comBUFFER_LEN );
//AD1PCFGL=0xFFFF; // seteaz? pinii portului ADC1 ca fiind digitali
//AD1PCFGLbits.PCFG5 = 0; // seteaz? pinul AN4(RB2) ca intrare analogic?

}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
/* Schedule the co-routines from within the idle task hook. */
vCoRoutineSchedule();
}
/*-----------------------------------------------------------*/