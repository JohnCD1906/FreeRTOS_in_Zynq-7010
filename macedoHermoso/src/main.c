/**
 * @file  main.c
 * @brief FreeRTOS + Zynq: LED (MIO7) y eco por UART1 con interrupcion + semaforos.
 * @date  2026-10-26
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "xparameters.h"
#include "xgpiops.h"
#include "xuartps.h"
#include "xscugic.h"
#include "xil_printf.h"
#include <string.h>

#include "xinterrupt_wrap.h"

/* ---------------- Constantes ---------------- */
#define LED_PIN         7u
#define GPIO_BASEADDR   XPAR_XGPIOPS_0_BASEADDR
#define UART_BASEADDR   XPAR_XUARTPS_0_BASEADDR   /* UART0 = 0xE0000000 */
#define UART_INT_ID     59u                       /* UART0 = IRQ 59 */
#define UART_BAUDRATE   115200u
#define RX_BUFFER_SIZE  64u

#define LED_PERIOD_MS   500u
#define LED_TASK_PRIO   (tskIDLE_PRIORITY + 1)
#define UART_TASK_PRIO  (tskIDLE_PRIORITY + 2)
#define LED_TASK_STACK  configMINIMAL_STACK_SIZE
#define UART_TASK_STACK (configMINIMAL_STACK_SIZE + 256)

/* ---------------- Globales ---------------- */
static XGpioPs Gpio;
static XUartPs Uart;
//extern XScuGic xInterruptController;      /* lo inicializa el port de FreeRTOS */

static SemaphoreHandle_t xRxSem = NULL;   /* ISR avisa: llegaron datos   */
static SemaphoreHandle_t xTxSem = NULL;   /* ISR avisa: termino el envio */

static u8 RxBuffer[RX_BUFFER_SIZE];
static u8 EchoBuffer[RX_BUFFER_SIZE];
static volatile u32 RxCount = 0;          /* bytes recibidos (lo pone la ISR) */

/* ---------------- Prototipos ---------------- */
static int  hw_init(void);
static int  Uart_IrqInit(void);
static void UartHandler(void *CallBackRef, u32 Event, unsigned int EventData);
static void vLedTask(void *pv);
static void vUartTask(void *pv);

/* ======================================================================= */
int main(void)
{
    if (hw_init() != XST_SUCCESS) {
        xil_printf("Error init HW\r\n");
        for (;;);
    }

    xRxSem = xSemaphoreCreateBinary();
    xTxSem = xSemaphoreCreateBinary();
    if (xRxSem == NULL || xTxSem == NULL) {
        xil_printf("Sin heap para semaforos\r\n");
        for (;;);
    }

    xTaskCreate(vLedTask,  "LED",  LED_TASK_STACK,  NULL, LED_TASK_PRIO,  NULL);
    xTaskCreate(vUartTask, "UART", UART_TASK_STACK, NULL, UART_TASK_PRIO, NULL);

    vTaskStartScheduler();
    for (;;);
    return 0;
}

/* ---------------- Init basico: GPIO + UART ---------------- */
static int hw_init(void)
{
    XGpioPs_Config *gpioCfg;
    XUartPs_Config *uartCfg;

    /* GPIO / LED */
    gpioCfg = XGpioPs_LookupConfig(GPIO_BASEADDR);
    if (gpioCfg == NULL) return XST_FAILURE;
    if (XGpioPs_CfgInitialize(&Gpio, gpioCfg, gpioCfg->BaseAddr) != XST_SUCCESS)
        return XST_FAILURE;
    XGpioPs_SetDirectionPin(&Gpio, LED_PIN, 1);
    XGpioPs_SetOutputEnablePin(&Gpio, LED_PIN, 1);
    XGpioPs_WritePin(&Gpio, LED_PIN, 0);

    /* UART1 */
    uartCfg = XUartPs_LookupConfig(UART_BASEADDR);
    if (uartCfg == NULL) return XST_FAILURE;
    if (XUartPs_CfgInitialize(&Uart, uartCfg, uartCfg->BaseAddress) != XST_SUCCESS)
        return XST_FAILURE;

    XUartPs_SetOperMode(&Uart, XUARTPS_OPER_MODE_NORMAL);
    if (XUartPs_SetBaudRate(&Uart, UART_BAUDRATE) != XST_SUCCESS)
        return XST_FAILURE;
    XUartPs_SetFifoThreshold(&Uart, 1u);   /* interrumpe con 1 byte en el FIFO RX */
    XUartPs_SetRecvTimeout(&Uart, 8u);     /* entrega bloques parciales por timeout */

    return XST_SUCCESS;
}

static int Uart_IrqInit(void)
{
    int Status;

    /* Callback del driver cuando ocurre un evento de UART */
    XUartPs_SetHandler(&Uart, (XUartPs_Handler)UartHandler, &Uart);

    /* Eventos que disparan interrupcion */
    XUartPs_SetInterruptMask(&Uart,
        XUARTPS_IXR_RXOVR | XUARTPS_IXR_RXFULL | XUARTPS_IXR_TOUT | XUARTPS_IXR_TXEMPTY);

    /* API moderna del BSP (xinterrupt_wrap): conecta la ISR a la GIC,
       le pone prioridad segura y la habilita. Compatible con FreeRTOS. */
    Status = XSetupInterruptSystem(&Uart,
                 (void *)XUartPs_InterruptHandler,
                 Uart.Config.IntrId,
                 Uart.Config.IntrParent,
                 0xA0);                 /* prioridad >= 18<<3, segura para FromISR */
    if (Status != XST_SUCCESS) return XST_FAILURE;

    return XST_SUCCESS;
}

static void UartHandler(void *CallBackRef, u32 Event, unsigned int EventData)
{
    BaseType_t xWoken = pdFALSE;
    (void)CallBackRef;

    switch (Event) {
        case XUARTPS_EVENT_SENT_DATA:
            xSemaphoreGiveFromISR(xTxSem, &xWoken);      /* TX termino */
            break;
        case XUARTPS_EVENT_RECV_DATA:
        case XUARTPS_EVENT_RECV_TOUT:
            RxCount = EventData;                          /* bytes recibidos */
            xSemaphoreGiveFromISR(xRxSem, &xWoken);      /* avisa a la tarea */
            break;
        case XUARTPS_EVENT_RECV_ERROR:
        case XUARTPS_EVENT_RECV_ORERR:
            break;                                        /* error RX (opcional) */
        default:
            break;
    }
    portYIELD_FROM_ISR(xWoken);
}

/* ---------------- Tarea LED ---------------- */
static void vLedTask(void *pv)
{
    (void)pv;
    uint32_t estado = 0;
    for (;;) {
        estado = !estado;
        XGpioPs_WritePin(&Gpio, LED_PIN, estado);
        vTaskDelay(pdMS_TO_TICKS(LED_PERIOD_MS));
    }
}

static void vUartTask(void *pv)
{
    (void)pv;
    static const char *bienvenida =
        "\r\nUART0 PS - modo interrupcion (FreeRTOS)\r\n"
        "Escribe algo para recibir el eco.\r\n";

    if (Uart_IrqInit() != XST_SUCCESS) { xil_printf("err irq\r\n"); vTaskDelete(NULL); }

    XUartPs_Send(&Uart, (u8 *)bienvenida, strlen(bienvenida));
    xSemaphoreTake(xTxSem, portMAX_DELAY);               /* ahora SI se da */

    XUartPs_Recv(&Uart, RxBuffer, RX_BUFFER_SIZE);       /* arma recepcion */

    for (;;) {
        if (xSemaphoreTake(xRxSem, portMAX_DELAY) == pdTRUE) {
            u32 n = RxCount;
            if (n > RX_BUFFER_SIZE) n = RX_BUFFER_SIZE;

            memcpy(EchoBuffer, RxBuffer, n);
            XUartPs_Send(&Uart, EchoBuffer, n);
            xSemaphoreTake(xTxSem, portMAX_DELAY);       /* espera fin de TX */

            XUartPs_Recv(&Uart, RxBuffer, RX_BUFFER_SIZE);  /* rearma */
        }
    }
}