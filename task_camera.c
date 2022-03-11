#include "task_camera.h"
#include "board.h"
#include "stm32f4xx.h"
#include <string.h>
#include "dcmi.h"
#include "dma2.h"
#include "mem_utils.h"
#include "dbg_uart.h"
#include "stm32f4xx_dcmi.h"
#include "i2c2.h"

//TO GET IMAGE NAME WITH TIME STAMP
#include "RTC.h"

/*For I2C Lock*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* FOR JPEG LIB */
#include "cdjpeg.h"
#include <stdint.h>
#include <stdbool.h>

/*
 *NOTE : i was getting section code placment fail error from linker so i modified lineker
 * script and changed RAM End area to : 0x2003FFFF from 0x2002FFFF
 */


/* Private macro -------------------------------------------------------------*/
#define BMP_PIXEL16_TO_R(pixel)       ((pixel & 0x1F) << 3)
#define BMP_PIXEL16_TO_G(pixel)       (((pixel >> 5) & 0x3F) << 2)
#define BMP_PIXEL16_TO_B(pixel)       (((pixel >> 11) & 0x1F) << 3)
#define MAX_IMAGE_SIZE                (176*144*3)      /* Size of RGB24 image  */
#define IMAGE_COLUMN_SIZE             144
#define IMAGE_LINE_SIZE               176
#define IMAGE_LINE_BYTES              (IMAGE_LINE_SIZE * 2)




/* Freertos related veriables */

 /* bus mutex */
//as multibuese are connected with I2C2//
extern xSemaphoreHandle i2c2_lock;
SemaphoreHandle_t xSemaphore_StartCamera_Capture = NULL;
TimerHandle_t xTimer_Camera;

/* Globle veriables */
unsigned long image_size = 0;
uint8_t image[IMAGE_BYTES];
static struct jpeg_compress_struct cinfo; /* This struct contains the JPEG compression parameters */
struct jpeg_error_mgr jerr; /* This struct represents a JPEG error handler */
JSAMPROW row_pointer; /* Pointer to a single row */
uint8_t *dst;

uint8_t DcmiState = DCMI_IDEAL;
//uint8_t camera_timer_expired = 0;
static uint8_t Image_Number = DCMI_IMAGE_1;
OV7692_IDTypeDef OV7692_Camera_ID;
//10KB
ImageFile_t fileImg;
uint8_t *Ptr;
uint8_t linebuf[__LINEBUFFSIZE__];
uint8_t ImgLineNo = 0;
static uint16_t RGB565_ByteNo = 0,RGB888_ByteNo = 0;
uint16_t PixelValue;


uint16_t image_v_size;
char ImageName[22];
extern volatile struct Camera_Flags Camera_Flags_t = {0};
uint8_t dstImage[COMPRESSED_IMAGE_BUF_SIZE];


/* QCIF 176x144 */
const unsigned char Camera_OV7692_QCIF[][2]=
{
 { 0x12, 0x80 },// reset the system
 { 0x0e, 0x08 },//sleep mode enable
 { 0x69, 0x52 },//horizontal winodw start point controll(LSB)
 { 0x1e, 0xb3 },//AGE & AEC realted
 { 0x48, 0x42 },
 { 0xff, 0x01 },//MIPI bank selcted
 { 0xb5, 0x30 },//Power down MIPI
 { 0xff, 0x00 },///MIPI bank NOT selcted
 { 0x16, 0x07 },//enable slow PCKL for YUV size less then QVGA
 { 0x62, 0x10 },//In defulat state
 { 0x12, 0x40 },//Skip mode enable
 { 0x12, 0x46 },//RGB and RGB565 - 16 bit
 { 0x18, 0xa4 },//defult state Horiz -sensor size 2*[msb(0xa4),lsb0x16's 6th bit[0]]
 { 0x19, 0x06 },//vertical winodw start point controll(LSB)
 { 0x1a, 0x7b },//vertical sensor size =2*[0x7b]
 { 0x22, 0x10 },//reserved
 { 0x37, 0x04 },//PCLK divider selection - No change here
 { 0x3e, 0x20 },//get 1/2 or 1/4 of PCLK [depnds upon REG5E] - here 5E 's 4th bit set so may be 1/4
 { 0x64, 0x21 },//PCLK is half ofS CLK
 { 0x5e, 0x10 },
 { 0x69, 0x02 }, //bypass BLC
 { 0x80, 0xff },//gamma, lensC, AWB_Gain all onn
 { 0x81, 0x3f },
 { 0xcc, 0x01 },//High 2 bits of horizantal o/p size
 { 0xcd, 0x40 },//low 8 bit of hozinaontal o/p size
 { 0xce, 0x00 },//9th bit of vertical o/p size
 { 0xcf, 0xf0 },//low 8 bit of vertical o/p size
 { 0xcc, 0x00 },
 { 0xcd, 0xb0 },
 { 0xce, 0x00 },
 { 0xcf, 0x90 },
 { 0x82, 0x03 },//out_sel = yuv422
 { 0xc8, 0x02 },//High 2 bits of horizantal i/p size
 { 0xc9, 0x80 },//low 8 bit of hozinaontal i/p size
 { 0xca, 0x00 },//9th bit of vertical i/p size
 { 0xcb, 0xf0 },//low 8 bit of vertical i/p size
 { 0xd0, 0x28 },// boundry offset win_hoff winv_off
 { 0x0e, 0x00 },//Normal mode from sleeep mode
 { 0x70, 0x00 },//defulat state - thrshould timer count 1sec , band set deflt 60 hz, timer no division
 { 0x71, 0x34 },//band counter enable // [0:1:0:0] bits[3:2:1:0]
 { 0x74, 0x28 },//threshould for low sum value
 { 0x75, 0x98 },//threshould for hogh sum value
 { 0x76, 0x00 },//low threshluld for light meter MSB
 { 0x77, 0x64 },///low threshluld for light meter LSB
 { 0x78, 0x01 },//High threshluld for light meter MSB
 { 0x79, 0xc2 },////high threshluld for light meter LSB
 { 0x7a, 0x4e },//clock periad for sample number MSB - defult
 { 0x7b, 0x1f },//clock periad for sample number LSB - defult
 { 0x7c, 0x00 },// indeirect regisgter add
 { 0x11, 0x01 },//intrnal clock = i/p clock / bit[5:0]+1 =0:0:0:0:1 = 8MHz if 24 Mhz is i/p
 { 0x20, 0x00 }, //defult - related to banding filter -refer data sheet if needed
 { 0x21, 0x57 },
 { 0x50, 0x4d },//50 Hz bending AEC
 { 0x51, 0x40 },//60 Hz bending AEC
 { 0x4c, 0x7d },
 { 0x0e, 0x00 },//Normal mode
 { 0x80, 0x7f },
 { 0x85, 0x00 },//defult
 { 0x86, 0x00 },//defult - lc_redius
 { 0x87, 0x00 },//defult
 { 0x88, 0x00 },//defult
 { 0x89, 0x2a },//lc_rgain
 { 0x8a, 0x22 },//lc_ggain
 { 0x8b, 0x20 },///lc_bgain
 { 0xbb, 0xab },//m1
 { 0xbc, 0x84 },//m2
 { 0xbd, 0x27 },//m3
 { 0xbe, 0x0e },//m4
 { 0xbf, 0xb8 },//m5
 { 0xc0, 0xc5 },//m6
 { 0xc1, 0x1e },// defult
 { 0xb7, 0x05 },//Offset
 { 0xb8, 0x09 },//BASE1
 { 0xb9, 0x00 },//BASE2
 { 0xba, 0x18 },//gain 4x is limted to 16
 { 0x5a, 0x1f },//Slop of UV curve
 { 0x5b, 0x9f },//UV adjustment gain refer data sheet
 { 0x5c, 0x69 },
 { 0x5d, 0x42 },
 { 0x24, 0x78 },//AEG and AEC stable operating -uper limit
 { 0x25, 0x68 },//AEG and AEC stable operating -lower limit
 { 0x26, 0xb3 },//Fast mode operating aec aeg
 { 0xa3, 0x0b },//YST0
 { 0xa4, 0x15 },
 { 0xa5, 0x29 },
 { 0xa6, 0x4a },
 { 0xa7, 0x58 },
 { 0xa8, 0x65 },
 { 0xa9, 0x70 },
 { 0xaa, 0x7b },
 { 0xab, 0x85 },
 { 0xac, 0x8e },
 { 0xad, 0xa0 },
 { 0xae, 0xb0 },
 { 0xaf, 0xcb },
 { 0xb0, 0xe1 },
 { 0xb1, 0xf1 },
 { 0xb2, 0x14 },//YST15
 { 0x8e, 0x92 },//AWP simple, stable range, local limit
 { 0x96, 0xff },//value top limit
 { 0x97, 0x00 },//value bot limit
 { 0x14, 0x3b },//Maximux age limit from 16x to 2x
 { 0x0e, 0x00 },
 { 0x30, 0x04 },//deflut
 { 0x31, 0x87 } //R_PLL_1 systme clk diver/2,divide VCo output , PLL loop divder
// { 0x61, 0x60 }//test bar genrate and 8-bit mode 1 //added by RUTVIJ
};


/*============================== put it into board.c =======================*/
/*   Turn on proper power sequance of camera as per data sheet
 *    mentioed as a waveform
 */
#if 0
void Camera_supply_on()
{
  // First and formost we need to on the DOVDD suplly but
  // its been done in main for more informaton refer to
  // enable_supply_3V_ldo() in hardware setup which uses PD8 pin

  //configure PG7 for LDO pulse high //
  // 2V8_EN pin for camera //
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(CAMERA_POWER_ON_CLK, ENABLE);

  GPIO_InitStructure.GPIO_Pin = CAMERA_POWER_ON_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(CAMERA_POWER_ON_PORT, &GPIO_InitStructure);
  //2V8_ENLDO ENABLE CONFIG//

  // Follow the turn on sequance as per data sheet
  // kepping time maintain
  // 3v supply basicaly
  DOVDD_SUPPLY_OFF();

  //vTaskDelay(5);
  Delay_us(5);

  // Turn on the DOVDD //
  DOVDD_SUPPLY_ON();

  // max 10 ms delay //
  //vTaskDelay(2);
  Delay_us(2);

  // LDO enableing for Camera 2v8_en //
  CAMERA_SUPPLY_ON();

  // 1ms delay //
  Delay_us(1);
}
#endif


static void Camera_supply_on()
{
  //CAMERA DOVDD PIN CONFIGURE //
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(DOVDD_ON_CLK, ENABLE);

  GPIO_InitStructure.GPIO_Pin = DOVDD_ON_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(DOVDD_ON_PORT, &GPIO_InitStructure);
//  GPIO_ReadOutputDataBit(DOVDD_ON_PORT, DOVDD_ON_PIN);
  //CAMERA DOVDD PIN CONFIGURE //


  //configure PG7 for LDO pulse high //
  //GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(CAMERA_POWER_ON_CLK, ENABLE);

  GPIO_InitStructure.GPIO_Pin = CAMERA_POWER_ON_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(CAMERA_POWER_ON_PORT, &GPIO_InitStructure);

  //LDO ENABLE CONFIG//

  // Turning off the DOVDD //
  DOVDD_SUPPLY_OFF();
  //vTaskDelay(5);
  Delay_us(5);

  // Turn on the DOVDD //
  DOVDD_SUPPLY_ON();

  //vTaskDelay(2); //max 10 ms delay
  Delay_us(2);

  //LDO enableing...//
  CAMERA_SUPPLY_ON();

  //vTaskDelay(1); //1ms delay
   Delay_us(1);
}



/**
  * @brief  Initializes the hardware resources (I2C and GPIO) used to configure
  *         the OV7692 camera.
  * @param  None
  * @retval None
  */
void Camera_OV7692_pinmux_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(DCMI_PWDN_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd(XTAL_OUT_GPIO_CLK, ENABLE);

  GPIO_StructInit(&GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

  GPIO_InitStructure.GPIO_Pin = DCMI_PWDN_PIN;
  GPIO_Init(DCMI_PWDN_GPIO_PORT, &GPIO_InitStructure);

  CAMERA_PWR_ON();
  //CAMERA_PWR_OFF();

  //vTaskDelay(100);
  Delay_us(100);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

  GPIO_InitStructure.GPIO_Pin = XTAL_OUT_PIN;
  GPIO_Init(XTAL_OUT_GPIO_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(XTAL_OUT_GPIO_PORT, XTAL_OUT_GPIO_PIN_SOURCE, GPIO_AF_MCO);
  RCC_MCO2Config(RCC_MCO2Source_HSE, RCC_MCO2Div_1);

  /*** Configures the DCMI GPIOs to interface with the OV9655 camera module ***/
  /* Enable DCMI GPIOs clocks */
  RCC_AHB1PeriphClockCmd(DCMI_D0_GPIO_CLK | DCMI_D1_GPIO_CLK | DCMI_D2_GPIO_CLK | DCMI_D3_GPIO_CLK |
                         DCMI_D4_GPIO_CLK | DCMI_D5_GPIO_CLK | DCMI_D6_GPIO_CLK | DCMI_D7_GPIO_CLK |
                         DCMI_HSYNC_GPIO_CLK | DCMI_VSYNC_GPIO_CLK | DCMI_PCLK_GPIO_CLK , ENABLE);

  /* Connect DCMI pins to AF13 */
  GPIO_PinAFConfig(DCMI_D0_GPIO_PORT, DCMI_D0_SOURCE, DCMI_AF);
  GPIO_PinAFConfig(DCMI_D1_GPIO_PORT, DCMI_D1_SOURCE, DCMI_AF);
  GPIO_PinAFConfig(DCMI_D2_GPIO_PORT, DCMI_D2_SOURCE, DCMI_AF);
  GPIO_PinAFConfig(DCMI_D3_GPIO_PORT, DCMI_D3_SOURCE, DCMI_AF);
  GPIO_PinAFConfig(DCMI_D4_GPIO_PORT, DCMI_D4_SOURCE, DCMI_AF);
  GPIO_PinAFConfig(DCMI_D5_GPIO_PORT, DCMI_D5_SOURCE, DCMI_AF);
  GPIO_PinAFConfig(DCMI_D6_GPIO_PORT, DCMI_D6_SOURCE, DCMI_AF);
  GPIO_PinAFConfig(DCMI_D7_GPIO_PORT, DCMI_D7_SOURCE, DCMI_AF);

  GPIO_PinAFConfig(DCMI_HSYNC_GPIO_PORT, DCMI_HSYNC_SOURCE, DCMI_AF);
  GPIO_PinAFConfig(DCMI_VSYNC_GPIO_PORT, DCMI_VSYNC_SOURCE, DCMI_AF);
  GPIO_PinAFConfig(DCMI_PCLK_GPIO_PORT, DCMI_PCLK_SOURCE, DCMI_AF);

  GPIO_StructInit(&GPIO_InitStructure);

  /* DCMI GPIO configuration */
  /* D0,D1, HSYNC, PIXCLK */
  GPIO_InitStructure.GPIO_Pin = DCMI_D0_PIN | DCMI_D1_PIN | DCMI_HSYNC_PIN | DCMI_PCLK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(DCMI_D0_GPIO_PORT, &GPIO_InitStructure);

  /* D2, D3 ,D6, D7*/
  GPIO_InitStructure.GPIO_Pin = DCMI_D2_PIN | DCMI_D3_PIN | DCMI_D6_PIN | DCMI_D7_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(DCMI_D2_GPIO_PORT, &GPIO_InitStructure);

  /* D4 */
  GPIO_InitStructure.GPIO_Pin = DCMI_D4_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(DCMI_D4_GPIO_PORT, &GPIO_InitStructure);

    /* D5, VSYNC */
  GPIO_InitStructure.GPIO_Pin = DCMI_D5_PIN | DCMI_VSYNC_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(DCMI_VSYNC_GPIO_PORT, &GPIO_InitStructure);

}


/* Configure camera DCMI / DMA2 and DMA2 IRQ */
static void Camera_OV7692_Config( void )
{
  /* Camera Registers read/write i2c */
  // camerea I2C is I2C 2, which has already been
  // initaed in afe4404 for more info see AFE4404.c
  /* confiog DCMI from dcmi.c */
  Camera_OV7692_Init_DCMI();
  /* confiog DMA2 from dma2.c */
//  Camera_OV7692_DMA2_Config(); // changing sequnace from diag firmware
//  /* DMA2 inturrupt configuration from dma2.c*/
//  Camera_DMA2_IRQ_Config();
  /* Config camera to capture QCIF 176x144 Images */
  Camera_QCIF_ImageQualityConfig();
  /*Enable DMA2 stream*/
  DMA_Cmd(DMA2_Stream1, ENABLE);
  /* Enable DCMI interface */
  DCMI_Cmd(ENABLE);
  /* minior delay */
  //vTaskDelay(100);
  Delay_us(100);
  /*Check weather device is configured or not */
  if(!Iscamera_configured())
  {
      #ifdef ENABLE_DBG
            {
              uint8_t *print = "camera I2C failed - camera config error\r\n";
              uart_send(print,strlen((const char *)print));
            }
      #endif

  }
  else
  {
     #ifdef ENABLE_DBG
            {
              uint8_t *print = "camera configured sucessfully\r\n";
              uart_send(print,strlen((const char *)print));
            }
      #endif
  }
  /* Init Compression formte JPEG */
  Init_JPEG_Lib();

}


static void Camera_QCIF_ImageQualityConfig( void )
{
    uint32_t i;

    Camera_OV7692_Reset();
    Delay_us(200);

    /* Initialize OV9655 */
    // take mutex loack as we are about to wirte into I2C bus //
    xSemaphoreTake(i2c2_lock,(TickType_t)portMAX_DELAY );

    for(i=0; i<(sizeof(Camera_OV7692_QCIF)/2); i++)
    {
        I2C2_WriteDeviceRegister(CAMERA_OV7692_SLAVE_ADDRESS, Camera_OV7692_QCIF[i][0], Camera_OV7692_QCIF[i][1]);
        Delay_us(2);
    }

    // we have completed transaction of I2C bus so relsease the lock //
    xSemaphoreGive(i2c2_lock);
}

/* Will do jpeg compresion config */
static void Init_JPEG_Lib()
{
    /* Allocate Memory require for JPEG */
    mem_pool_init();
    CameraJpeg_Init();
}

/*  check weather device gets configured */
static uint8_t Iscamera_configured()
{
     // by defult fail //
    uint8_t status_t = 0;

    // take mutex loack as we are about to wirte into I2C bus //
    xSemaphoreTake(i2c2_lock,(TickType_t)portMAX_DELAY );

    OV7692_ReadID(&OV7692_Camera_ID);

    // we have completed transaction of I2C bus so relsease the lock //
    xSemaphoreGive(i2c2_lock);

    if( (OV7692_Camera_ID.PIDH  == 0x76) && (OV7692_Camera_ID.PIDL  == 0x92)  )
    {
        /* Camera Found */
      status_t = 1;
    }

    return status_t;
}
/**
  * @brief  Initialize Software JPEG Engine.
  * @param  None
  * @retval None
  */
static void CameraJpeg_Init(void)
{
   /* Step 1: allocate and initialize JPEG compression object */
   /* Set up the error handler */
   cinfo.err = jpeg_std_error(&jerr);

   /* Initialize the JPEG compression object */
   jpeg_create_compress(&cinfo);

   image_size = COMPRESSED_IMG_MAX_SIZE;
   dst = &dstImage[IMAGE_FILE_HEADER_SIZE + IMAGE_FILE_HEADER_PADDING];
   jpeg_mem_dest(&cinfo, &dst, &image_size);

   /* Step 3: set parameters for compression */
   cinfo.image_width = JPEG_IMAGE_WIDTH;
   cinfo.image_height = JPEG_IMAGE_HIGHT;
   cinfo.input_components = JPEG_IMAGE_COMPONENT;
   cinfo.in_color_space = JCS_RGB;

   /* Set default compression parameters */
   jpeg_set_defaults(&cinfo);
   jpeg_set_quality(&cinfo, IMAGE_QUALITY, TRUE);
}




/**
  * @brief  Resets the OV7692 camera.
  * @param  None
  * @retval None
  */
static void Camera_OV7692_Reset(void)
{
   (xSemaphoreTake( i2c2_lock, ( TickType_t ) portMAX_DELAY ));
    // write i2c ocntorl regiser of camera to reset the camera //
    I2C2_WriteDeviceRegister(CAMERA_OV7692_SLAVE_ADDRESS, OV7692_REG12, 0x80);
     // we have completed transaction of I2C bus so relsease the lock //
    xSemaphoreGive(i2c2_lock);
}

/**
  * @brief  Reads the OV7692 Manufacturer identifier.
  * @param  OV7692ID: pointer to the OV7692 Manufacturer identifier.
  * @retval None
  */
static void OV7692_ReadID(OV7692_IDTypeDef* OV7692ID)
{

    OV7692ID->Manufacturer_ID1 = I2C2_ReadDeviceRegister(CAMERA_OV7692_SLAVE_ADDRESS, OV7692_MIDH);
    OV7692ID->Manufacturer_ID2 = I2C2_ReadDeviceRegister(CAMERA_OV7692_SLAVE_ADDRESS, OV7692_MIDL);
    OV7692ID->PIDH = I2C2_ReadDeviceRegister(CAMERA_OV7692_SLAVE_ADDRESS, OV7692_PIDH);
    OV7692ID->PIDL = I2C2_ReadDeviceRegister(CAMERA_OV7692_SLAVE_ADDRESS, OV7692_PIDL);
}


void Camera_Init( void )
{
    /* Turn on power seqaunce for camera  */
    Camera_supply_on();

    /*DCMI pin muxing for camera*/
    Camera_OV7692_pinmux_Init();

    /*Camera config I2C2,DMA2 and DCMI */
    Camera_OV7692_Config();

}

uint8_t CameraHandlerCreateTask(void)
{

    uint8_t status = SUCCESS;
    TaskHandle_t xHandle_camera = NULL;

    BaseType_t xReturned = xTaskCreate( Camera_task_Handler, "Camera_Task",CAMERA_TASK_STACK_SIZE, NULL,
                                            CAMERA_TASK_PRIORITY,&xHandle_camera);
    /* task creation faild */
    if( xReturned != pdPASS )
    {
        /* The task was created.  Use the task's handle to delete the task. */
        vTaskDelete( xHandle_camera);
        status = ERROR;
    }



    /*creating semaphore to capture image*/
    xSemaphore_StartCamera_Capture = xSemaphoreCreateBinary();

    if(xSemaphore_StartCamera_Capture == NULL)
    {
      //TODO: Insufficient FreeRTOS HEAP available for the semaphore
      status = ERROR;
    }

    return status ;
}


void Camera_task_Handler(void *pvParameters)
{

#if 0
    xcx
    Camera_Flags_t.StartCaptureCommandRCV = 1;
    for( ; ; )
    {

        // TODO :: this task should not be in continues waitng reather
        // use freeRTOS api to block this task untill the signal is been recived
        if(!IsCameraBusy() && (Camera_Flags_t.StartCaptureCommandRCV) )
        {
          Camera_Flags_t.StartCaptureCommandRCV = 0;
          StartCaptureImage(1);
        }

        DcmiProcessEvents();
    }
#endif

    for( ; ; )
    {
      //do wait on emergany to happen and block till the time
      // semaphore will be relsed as soon as emergancy occurese from task_app.c
      if( xSemaphoreTake( xSemaphore_StartCamera_Capture,(TickType_t)portMAX_DELAY ) == pdTRUE )
      {

#ifdef ENABLE_DBG
        char *print = "Camera capture command recived\r\n";
        uart_send(print,strlen (print));
#endif
        for(uint8_t numberOfCapture = 1; numberOfCapture <=3; numberOfCapture ++ )
        {
            //send capture command //
            StartCaptureImage(numberOfCapture);

            do
            {
                //proceed caputing command
                DcmiProcessEvents();

            //wait untill it finishees image capturing process
            //taking one by one images to being captured
            }while( IsCameraBusy() );
        }
      }
    }

}

/**
  * @brief  Init Image Capture Resources.
  * @param  image_no - image number to capture
  * @retval None
  */
static void StartCaptureImage(uint8_t image_no)
{
    if(image_no < DCMI_MAX_IMAGES)
    {
        DcmiState = DCMI_DMA_ENABLE;
        Image_Number = image_no;
    }
}



 /**
  * @brief  Dcmi Process Events , Events can be capture image
  *         ,Store Image, Compress Image.
  * @param  none
  * @retval Non
  */
static void DcmiProcessEvents (void)
{
    /* Return If Flash Operation is ongoing Or Captres are being Sent because dstImage will be in use*/
//    if(FLASH_BUSY() || SendCaptures)
//      return;

    switch(DcmiState)
    {
        case DCMI_IDEAL:
        break;

        case DCMI_DMA_ENABLE:
            DMA_Cmd(DMA2_Stream1, ENABLE);
            DCMI_Cmd(ENABLE);
            // Sleep Fsor 100 ms
            vTaskDelay(200);
            DcmiState = DCMI_WAIT_FOR_DMA_READY;
        break;

        case DCMI_WAIT_FOR_DMA_READY:
          {
            DcmiState = DCMI_START_CAPTURE_IMAGE;
          }
        break;

        case DCMI_START_CAPTURE_IMAGE:
          {
            DCMI_CaptureCmd(ENABLE);
            DcmiState = DCMI_WAIT_FOR_IMAGE_CAPTURED;
          }
        break;

        case DCMI_WAIT_FOR_IMAGE_CAPTURED:
            if(Camera_Flags_t.image_captured)
            {
                Camera_Flags_t.image_captured = 0;
                Ptr = image;

                RTC_TimeTypeDef  RTC_Time;
                RTC_DateTypeDef RTC_DateStruct ;
                // DEFUALT NAMING IF rtc IS NOT CONFIGURED //
                fileImg.date = 1;
                fileImg.month = 1;
                fileImg.year =  (uint16_t) (2000);
                fileImg.hour = 1;
                fileImg.min = 1;
                fileImg.sec = 1;

                //GOT actual file name bt RTC STAMP //
                if(Is_RTC_Configured)
                {
                    RTC_DateTypeDef RTC_DateStruct;
                    RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);
                    RTC_GetTime(RTC_Format_BIN, &RTC_Time);


                    fileImg.date = RTC_DateStruct.RTC_Date;
                    fileImg.month = RTC_DateStruct.RTC_Month;
                    fileImg.year =  (uint16_t) (RTC_DateStruct.RTC_Year + 2000);
                    fileImg.hour = RTC_Time.RTC_Hours;
                    fileImg.min = RTC_Time.RTC_Minutes;
                    fileImg.sec = RTC_Time.RTC_Seconds;
                }

                image_size = COMPRESSED_IMG_MAX_SIZE;
                sprintf(ImageName,"IMG%02d%02d%02d_%02d%02d%04d.jpg",fileImg.hour,fileImg.min,fileImg.sec,fileImg.date,fileImg.month,fileImg.year);
                dst = &dstImage[IMAGE_FILE_HEADER_SIZE + IMAGE_FILE_HEADER_PADDING];
                jpeg_mem_dest(&cinfo, &dst, &image_size);
                DcmiState = DCMI_START_IMG_COMPRESS;

              }
            break;

        case DCMI_START_IMG_COMPRESS:
            jpeg_start_compress(&cinfo, TRUE);
            row_pointer = (JSAMPROW)linebuf;
            ImgLineNo = 0;
            DcmiState = DCMI_GET_LINE_NO;
        break;

        case DCMI_GET_LINE_NO:
            if(cinfo.next_scanline < cinfo.image_height)
            {
                  DcmiState = DCMI_PREPARE_ONE_LINE_DATA;
            }
            else
            {
                  DcmiState = DCMI_IMG_COMPRESSION;
            }
        break;

        case DCMI_PREPARE_ONE_LINE_DATA:
             RGB565_ByteNo = 0;
             RGB888_ByteNo = 0;
             for (RGB565_ByteNo = 0; RGB565_ByteNo < IMAGE_LINE_BYTES; RGB565_ByteNo+=2 )
             {
                  PixelValue = image[(ImgLineNo * IMAGE_LINE_BYTES) + RGB565_ByteNo + 1] & 0xFF;
                  PixelValue = ((image[(ImgLineNo * IMAGE_LINE_BYTES) + RGB565_ByteNo] & 0xFF) << 8) | PixelValue;

                  linebuf[RGB888_ByteNo++] = BMP_PIXEL16_TO_B(PixelValue);
                  linebuf[RGB888_ByteNo++] = BMP_PIXEL16_TO_G(PixelValue);
                  linebuf[RGB888_ByteNo++] = BMP_PIXEL16_TO_R(PixelValue);
             }
             DcmiState = DCMI_SCAN_ONE_LINE_DATA;
        break;

        case DCMI_SCAN_ONE_LINE_DATA:
            jpeg_write_scanlines(&cinfo, &row_pointer, 1);
            ImgLineNo++;
            DcmiState = DCMI_GET_LINE_NO;
        break;

        case DCMI_IMG_COMPRESSION:
            jpeg_finish_compress(&cinfo);
            DcmiState = DCMI_SAVE_COMPRESSED_FILE;
        break;

        case DCMI_SAVE_COMPRESSED_FILE:
            image_v_size= fileImg.file_size = (uint16_t) image_size;
            memcpy(&dstImage[0], &fileImg, sizeof(fileImg));

          #ifdef ENABLE_DBG
              {
                uint8_t *print = "========image capture started ====\r\n";
                uart_send(print,strlen((const char *)print));
              }
           #endif
        char temp[25] = {0};
        static uint8_t c = 0;
        for(int t = 16 ; t < (fileImg.file_size + IMAGE_FILE_HEADER_PADDING + IMAGE_FILE_HEADER_SIZE) ; t++)
        {
            //print 0x0  to 0xf//
            sprintf(temp,"%02x \0",dstImage[t]);
            #ifdef ENABLE_DBG
              {
                uart_send((uint8_t *)temp,strlen((const char *)temp));
              }
           #endif
            memset(temp,0,sizeof(temp));
            c++;

            if(c == 0x10)
            {
              c =0;
              #ifdef ENABLE_DBG
              {
                uint8_t *print = "\r\n";
                uart_send(print,strlen((const char *)print));
              }
           #endif
            }
           vTaskDelay(5);

        }

        #ifdef ENABLE_DBG
              {
                uint8_t *print = "\r\n========image has been captured ====\r\n";
                uart_send(print,strlen((const char *)print));
              }
        #endif


        #ifdef ENABLE_DBG
          {
            uint8_t *print = "\r\n\r\n";
            uart_send(print,strlen((const char *)print));

          }
        #endif
        //FLASH_PUT_FOR_WRITE(ImageMap[Image_Number], &dstImage[0], (fileImg.file_size + IMAGE_FILE_HEADER_PADDING + IMAGE_FILE_HEADER_SIZE) );

        /* now image has been captured so tell GSM to send it over FTP */
        uint8_t UploadCMD_to_gsm = FTP_UPLOAD_IMAGE ;

#ifdef ENABLE_DBG
        char *print1 = "Informing GSM about capture\r\n";
        uart_send(print1,strlen (print1));
#endif

        xQueueSend( xGSMHandleQueue, ( void * ) &UploadCMD_to_gsm, ( TickType_t ) 10 );

        DcmiState = WAIT_IMAGE_UPLOAD_ACK;
        break;

    case  WAIT_IMAGE_UPLOAD_ACK:
#ifdef ENABLE_DBG
        char *print3 = "Waitng for image upload ACK from GSM...\r\n";
        uart_send(print3,strlen (print3));
#endif
        /* now we have sent FTP upload request to GSM so wait/block
          untill it does the process of FTP image sending sending */
        if(xSemaphoreTake( xImageSent, portMAX_DELAY) == pdTRUE );
        {
#ifdef ENABLE_DBG
        char *print2 = "Recived ACK from GSM about FTP upload\r\n";
        uart_send(print2,strlen (print2));
#endif
            //we have been given ACK about image uploadation over FTP
            //so now proceed further
            DcmiState = DCMI_IDEAL;
        }
        break;
    }
}
