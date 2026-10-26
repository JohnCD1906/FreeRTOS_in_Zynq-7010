/**
 * @file  main.c
 * @brief FreeRTOS en Zynq-7000 (Zybo Z7-10): dos tareas -> LED (MIO7) + UART0.
 * @details
 *   Tarea LED : parpadea el LED on-board LD4 (MIO7) cada 500 ms.
 *   Tarea UART: imprime un mensaje por UART0 cada 1000 ms (polling).
 *   El tick de FreeRTOS lo genera el private timer del Cortex-A9 (SCU),
 *   ya integrado por el BSP freertos10_xilinx: aqui NO se toca el GIC.
 * @date  2026-10-25
 */

/* ---------------- Includes ---------------- */
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

#include "semphr.h" //asociar semaphore a interrupcion de UART


#include "xparameters.h"
#include "xgpiops.h"
#include "xuartps.h"
#include "xil_printf.h"

/* ---------------- Constantes ---------------- */
#define LED_PIN         7u                          /* MIO7 -> LD4 (PS)      */
#define GPIO_BASEADDR   XPAR_XGPIOPS_0_BASEADDR      /* 0xE000A000            */
#define UART_BASEADDR   XPAR_XUARTPS_0_BASEADDR  
#define UART_INT_ID     59u     /* UART0 = IRQ 59 (UART1 = 82). O XPAR_XUARTPS_0_INTR */    /* UART0 = 0xE0000000    */
#define UART_BAUDRATE   115200u

#define LED_PERIOD_MS   500u
#define UART_PERIOD_MS  1000u

#define LED_TASK_PRIO   (tskIDLE_PRIORITY + 1)
#define UART_TASK_PRIO  (tskIDLE_PRIORITY + 2)
#define LED_TASK_STACK  configMINIMAL_STACK_SIZE
#define UART_TASK_STACK (configMINIMAL_STACK_SIZE + 128)

/* ---------------- Instancias de los drivers ---------------- */
static XGpioPs Gpio;
static XUartPs Uart;


/* Semaforo que la ISR de UART le da a la tarea */
//static SemaphoreHandle_t xUartSem = NULL;


/* ---------------- Prototipos ---------------- */
static int  hw_init(void);




static void vLedTask(void *pvParameters);
static void vUartTask(void *pvParameters);

/* ======================================================================= */
int main(void)
{
    if (hw_init() != XST_SUCCESS) {
        xil_printf("Error inicializando el hardware\r\n");
        for (;;);
    }

    /* Crea el semaforo binario ANTES de habilitar la interrupcion */
    /*xUartSem = xSemaphoreCreateBinary();
    if (xUartSem == NULL) {
        xil_printf("Sin heap para el semaforo\r\n");
        for (;;);
    }  */  

    xil_printf("Arrancando FreeRTOS...\r\n");

    xTaskCreate(vLedTask,  "LED",  LED_TASK_STACK,  NULL, LED_TASK_PRIO,  NULL);
    xTaskCreate(vUartTask, "UART", UART_TASK_STACK, NULL, UART_TASK_PRIO, NULL);

    vTaskStartScheduler();   /* no regresa si hay heap suficiente */

    for (;;);                /* solo se llega aqui si fallo el scheduler */
    return 0;
}

/* ---------------- Inicializacion de HW (GPIO + UART0) ---------------- */
static int hw_init(void)
{
    XGpioPs_Config *gpioCfg;
    XUartPs_Config *uartCfg;

    /* GPIO */
    gpioCfg = XGpioPs_LookupConfig(GPIO_BASEADDR);
    if (gpioCfg == NULL) return XST_FAILURE;
    if (XGpioPs_CfgInitialize(&Gpio, gpioCfg, gpioCfg->BaseAddr) != XST_SUCCESS)
        return XST_FAILURE;

    /* MIO7 como salida (MIO7 es solo-salida en el Zynq, ideal para LED) */
    XGpioPs_SetDirectionPin(&Gpio, LED_PIN, 1);
    XGpioPs_SetOutputEnablePin(&Gpio, LED_PIN, 1);
    XGpioPs_WritePin(&Gpio, LED_PIN, 0);

    /* UART0 */
    uartCfg = XUartPs_LookupConfig(UART_BASEADDR);
    if (uartCfg == NULL) return XST_FAILURE;
    if (XUartPs_CfgInitialize(&Uart, uartCfg, uartCfg->BaseAddress) != XST_SUCCESS)
        return XST_FAILURE;

    XUartPs_SetBaudRate(&Uart, UART_BAUDRATE);
    return XST_SUCCESS;
}



/* ---------------- ISR callback (lo llama el driver desde la GIC) ---------------- */
/*static void Uart_IsrCallback(void *CallBackRef, u32 Event, u32 EventData)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    (void)CallBackRef;
    (void)EventData;

    //lee fifo, limpia isr, procesa en tarea

    //Señaliza a la tarea que hubo actividad de UART 
    xSemaphoreGiveFromISR(xUartSem, &xHigherPriorityTaskWoken);

    //Cambio de contexto si despertó una tarea de mayor prioridad 
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}*/

/* ---------------- Tarea 1: LED en MIO7 ---------------- */
static void vLedTask(void *pvParameters)
{
    (void)pvParameters;
    uint32_t estado = 0;
    for (;;) {
        estado = !estado;
        XGpioPs_WritePin(&Gpio, LED_PIN, estado);
        vTaskDelay(pdMS_TO_TICKS(LED_PERIOD_MS));
    }
}


// ---------------- Tarea 2: mensaje por UART0 (polling) ---------------- 
static void vUartTask(void *pvParameters)
{
    (void)pvParameters;
    const char *msg = "FreeRTOS vivo en Zynq-7000 :)\r\n";
    TickType_t  xLastWake = xTaskGetTickCount();

    for (;;) {
        XUartPs_Send(&Uart, (u8 *)msg, strlen(msg));
        while (XUartPs_IsSending(&Uart)) { }      //espera a vaciar el FIFO Tx 
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(UART_PERIOD_MS));
    }
}

/* ---------------- Tarea 2: espera la interrupcion de UART ---------------- */
/*static void vUartTask(void *pvParameters)
{
    (void)pvParameters;


    if (Uart_IrqInit() != XST_SUCCESS) {
        xil_printf("Error configurando la interrupcion de UART\r\n");
        vTaskDelete(NULL);
    }

    for (;;) {
        // Bloquea (sin gastar CPU) hasta que la ISR de el semaforo 
        if (xSemaphoreTake(xUartSem, portMAX_DELAY) == pdTRUE) {
            //procesa bytes de la isr
        }
    }
}*/