#ifndef TASK_CAMERA_H
#define TASK_CAMERA_H

/*Includes*/
#include "stm32f4xx.h"
#include "freeRTOS.h"
#include "semphr.h"
#include "timers.h"
#include "board.h"


/*FreeRTOS related Declarations*/
#define CAMERA_TASK_STACK_SIZE		 		1024    //(configMINIMAL_STACK_SIZE)
#define CAMERA_TASK_PRIORITY				( tskIDLE_PRIORITY + 1UL )

/*Application specific Macros*/

//#define OK                                                                 0
//#define FAILED                                                          ( ! ON )

/* Camera states*/
#define DCMI_IDEAL                                                             0
#define DCMI_DMA_ENABLE                                                        1
#define DCMI_WAIT_FOR_DMA_READY                                                2
#define DCMI_START_CAPTURE_IMAGE                                               3
#define DCMI_WAIT_FOR_IMAGE_CAPTURED                                           4
#define DCMI_START_IMG_COMPRESS                                                5
#define DCMI_GET_LINE_NO                                                       6
#define DCMI_PREPARE_ONE_LINE_DATA                                             7
#define DCMI_SCAN_ONE_LINE_DATA                                                8
#define DCMI_IMG_COMPRESSION                                                   9
#define DCMI_SAVE_COMPRESSED_FILE                                             10
#define WAIT_IMAGE_UPLOAD_ACK                                                 11



#define DCMI_IMAGE_1                                                           0
#define DCMI_IMAGE_2                                                           1
#define DCMI_IMAGE_3                                                           2
#define DCMI_IMAGE_4                                                           3
#define DCMI_IMAGE_5                                                           4
#define DCMI_MAX_IMAGES                                                        5


/* Captured image realted macros */
#define IMAGE_WIDTH                                                          176
#define IMAGE_HEIGHT                                                         144
//2 Bytes/Pixel
#define PIXEL_WIDTH                                                            2

#define IMAGE_PIXELS                                (IMAGE_WIDTH * IMAGE_HEIGHT)

#define BYTES_PER_PIXEL                                                        2
#define IMAGE_BYTES                             (IMAGE_PIXELS * BYTES_PER_PIXEL)

#define DCMI_PERIPHERAL_SIZE_BYTES                                             4
#define DMA_BUFFER_SIZE_DCMI        ((uint16_t)(IMAGE_BYTES / DCMI_PERIPHERAL_SIZE_BYTES))
#define __LINEBUFFSIZE__                                                     528

/* Camera Compression JPEG library related macros */
#define JPEG_IMAGE_WIDTH                                             IMAGE_WIDTH
#define JPEG_IMAGE_HIGHT                                            IMAGE_HEIGHT
#define JPEG_IMAGE_COMPONENT                                                   3
#define IMAGE_QUALITY                                                         50

#define DCMI_DR_ADDRESS                                               0x50050028

#define IMAGE_FILE_HEADER_PADDING                                              7
#define IMAGE_FILE_HEADER_SIZE                                                 9
#define COMPRESSED_IMG_MAX_SIZE                                            10224
#define COMPRESSED_IMAGE_BUF_SIZE     (COMPRESSED_IMG_MAX_SIZE + IMAGE_FILE_HEADER_SIZE + IMAGE_FILE_HEADER_PADDING)


/* Camera realted macros */
#define OV7692_MIDH                                                         0x1C
#define OV7692_MIDL                                                         0x1D
#define OV7692_PIDH                                                         0x0A
#define OV7692_PIDL                                                         0x0B
#define OV7692_REG12                                                        0x12
#define CAMERA_OV7692_SLAVE_ADDRESS                              ((uint8_t)0x78)



#define DOVDD_ON_PIN                GPIO_Pin_8
#define DOVDD_ON_PORT               GPIOD
#define DOVDD_ON_CLK                RCC_AHB1Periph_GPIOD
#define DOVDD_ON_SRC                GPIO_PinSource8

#define DCMI_PWDN_PIN               GPIO_Pin_3
#define DCMI_PWDN_GPIO_PORT         GPIOA
#define DCMI_PWDN_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define DCMI_PWDN_SOURCE            GPIO_PinSource3

#define CAMERA_POWER_ON_PIN         GPIO_Pin_7
#define CAMERA_POWER_ON_PORT        GPIOG
#define CAMERA_POWER_ON_CLK         RCC_AHB1Periph_GPIOG
#define CAMERA_POWER_ON_SRC         GPIO_PinSource7

#define CAMERA_PWR_ON()             GPIO_ResetBits(DCMI_PWDN_GPIO_PORT, DCMI_PWDN_PIN)
#define CAMERA_PWR_OFF()            GPIO_SetBits(DCMI_PWDN_GPIO_PORT, DCMI_PWDN_PIN)

#define CAMERA_SUPPLY_ON()          GPIO_SetBits(CAMERA_POWER_ON_PORT, CAMERA_POWER_ON_PIN)
#define CAMERA_SUPPLY_OFF()         GPIO_ResetBits(CAMERA_POWER_ON_PORT, CAMERA_POWER_ON_PIN)

#define IsCameraBusy()             (DcmiState)

#define DOVDD_SUPPLY_ON()          GPIO_SetBits(DOVDD_ON_PORT, DOVDD_ON_PIN)
#define DOVDD_SUPPLY_OFF()         GPIO_ResetBits(DOVDD_ON_PORT, DOVDD_ON_PIN)

/*================== PUT IT INTO board.c ====================================*/
#define DCMI_AF                     GPIO_AF_DCMI

/* Enable HSE Output on PC9 - MCO2 */
#define XTAL_OUT_PIN                GPIO_Pin_9
#define XTAL_OUT_GPIO_PORT          GPIOC
#define XTAL_OUT_GPIO_CLK           RCC_AHB1Periph_GPIOC
#define XTAL_OUT_GPIO_PIN_SOURCE    GPIO_PinSource9

#define DCMI_CLK                    RCC_AHB2Periph_DCMI

#define DCMI_D0_PIN                 GPIO_Pin_9
#define DCMI_D0_GPIO_PORT           GPIOA
#define DCMI_D0_GPIO_CLK            RCC_AHB1Periph_GPIOA
#define DCMI_D0_SOURCE              GPIO_PinSource9

#define DCMI_D1_PIN                 GPIO_Pin_10
#define DCMI_D1_GPIO_PORT           GPIOA
#define DCMI_D1_GPIO_CLK            RCC_AHB1Periph_GPIOA
#define DCMI_D1_SOURCE              GPIO_PinSource10

#define DCMI_D2_PIN                 GPIO_Pin_0
#define DCMI_D2_GPIO_PORT           GPIOE
#define DCMI_D2_GPIO_CLK            RCC_AHB1Periph_GPIOE
#define DCMI_D2_SOURCE              GPIO_PinSource0

#define DCMI_D3_PIN                 GPIO_Pin_1
#define DCMI_D3_GPIO_PORT           GPIOE
#define DCMI_D3_GPIO_CLK            RCC_AHB1Periph_GPIOE
#define DCMI_D3_SOURCE              GPIO_PinSource1

#define DCMI_D4_PIN                 GPIO_Pin_11
#define DCMI_D4_GPIO_PORT           GPIOC
#define DCMI_D4_GPIO_CLK            RCC_AHB1Periph_GPIOC
#define DCMI_D4_SOURCE              GPIO_PinSource11


#define DCMI_D5_PIN                 GPIO_Pin_6
#define DCMI_D5_GPIO_PORT           GPIOB
#define DCMI_D5_GPIO_CLK            RCC_AHB1Periph_GPIOB
#define DCMI_D5_SOURCE              GPIO_PinSource6

#define DCMI_D6_PIN                 GPIO_Pin_5
#define DCMI_D6_GPIO_PORT           GPIOE
#define DCMI_D6_GPIO_CLK            RCC_AHB1Periph_GPIOE
#define DCMI_D6_SOURCE              GPIO_PinSource5

#define DCMI_D7_PIN                 GPIO_Pin_6
#define DCMI_D7_GPIO_PORT           GPIOE
#define DCMI_D7_GPIO_CLK            RCC_AHB1Periph_GPIOE
#define DCMI_D7_SOURCE              GPIO_PinSource6


#define DCMI_HSYNC_PIN              GPIO_Pin_4
#define DCMI_HSYNC_GPIO_PORT        GPIOA
#define DCMI_HSYNC_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define DCMI_HSYNC_SOURCE           GPIO_PinSource4

#define DCMI_VSYNC_PIN              GPIO_Pin_7
#define DCMI_VSYNC_GPIO_PORT        GPIOB
#define DCMI_VSYNC_GPIO_CLK         RCC_AHB1Periph_GPIOB
#define DCMI_VSYNC_SOURCE           GPIO_PinSource7

#define DCMI_PCLK_PIN               GPIO_Pin_6
#define DCMI_PCLK_GPIO_PORT         GPIOA
#define DCMI_PCLK_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define DCMI_PCLK_SOURCE            GPIO_PinSource6

#define DCMI_XVCLK_PIN              GPIO_Pin_9
#define DCMI_XVCLK_GPIO_PORT        GPIOC
#define DCMI_XVCLK_GPIO_CLK         RCC_AHB1Periph_GPIOC
#define DCMI_XVCLK_SOURCE           GPIO_PinSource9
#define DCMI_XVCLK_AF               GPIO_AF_MCO

#define DCMI_PWDN_PIN               GPIO_Pin_3
#define DCMI_PWDN_GPIO_PORT         GPIOA
#define DCMI_PWDN_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define DCMI_PWDN_SOURCE            GPIO_PinSource3


/* Size defines, defined for readability in code below */


/*Enums, sturtcures, unions*/


/*Gloabal veriables*/
extern uint8_t image[IMAGE_BYTES];

//extern volatile uint8_t image_captured;

volatile struct Camera_Flags
{
  unsigned int image_captured  : 1;
  unsigned int StartCaptureCommandRCV : 1;
};
extern volatile struct Camera_Flags Camera_Flags_t;

typedef struct
{
  uint8_t Manufacturer_ID1;
  uint8_t Manufacturer_ID2;
  uint8_t PIDH;
  uint8_t PIDL;
}OV7692_IDTypeDef;

typedef struct ImageFile
{
    uint16_t year;
    uint8_t  month;
    uint8_t  date;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint16_t file_size;
}ImageFile_t;


extern uint16_t image_v_size ;
extern uint8_t dstImage[COMPRESSED_IMAGE_BUF_SIZE];
extern char ImageName[22];

/*FreeRTOS realted declration*/
extern SemaphoreHandle_t xSemaphore_StartCamera_Capture;


/* Camera main task */
void Camera_task_Handler(void *pvParameters);
uint8_t Camera_TimerChanged( uint8_t );
//void vDcmiCameraTimer( TimerHandle_t pxTimer );

/*User app. Funcion declaration*/
void Camera_Init( void );
static void Camera_supply_on();
static void Init_JPEG_Lib();
static void CameraJpeg_Init(void);
static void Camera_QCIF_ImageQualityConfig( void );
static void Camera_OV7692_Reset(void);
static void DcmiProcessEvents(void);
static void StartCaptureImage(uint8_t);
static void OV7692_ReadID(OV7692_IDTypeDef*);
static uint8_t Iscamera_configured();
/*DCMI pin muxing for camera*/
static void Camera_OV7692_pinmux_Init( void );
static void Camera_OV7692_Config( void );

#endif /* TASK_CAMERA_H*/


