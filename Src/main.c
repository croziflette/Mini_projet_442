/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma2d.h"
#include "i2c.h"
#include "ltdc.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_camera.h"
#include "stm32746g_discovery_ts.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  MODE_TRAIN = 0,
  MODE_RECO
} AppMode_t;

typedef enum
{
  BTN_NONE = 0,
  BTN_P1_ADD,
  BTN_P2_ADD,
  BTN_TEST,
  BTN_RESET
} TouchButtonId_t;

typedef struct
{
  uint16_t x;
  uint16_t y;
  uint16_t w;
  uint16_t h;
  TouchButtonId_t id;
  const char *label;
} TouchButton_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_WIDTH               480U
#define LCD_HEIGHT              272U
#define SCREEN_PIXELS           (LCD_WIDTH * LCD_HEIGHT)

#define CAMERA_FRAME_BUFFER     0xC0260000U
#define CAMERA_RESOLUTION       RESOLUTION_R480x272

#define LAYER0_ADDRESS          ((uint32_t)LCD_FB_START_ADDRESS)
#define LAYER1_ADDRESS          (LAYER0_ADDRESS + SCREEN_PIXELS * 4U)

#define FEATURE_W               24U
#define FEATURE_H               24U
#define FEATURE_SIZE            (FEATURE_W * FEATURE_H)
#define ROI_SIZE                250U
#define N_TRAIN                 6U

#define BTN_Y                   224U
#define BTN_H                   40U
#define BTN_W                   108U

#define SOBEL_THRESHOLD_DEFAULT 75U
#define SOBEL_MIN_TH            20U
#define SOBEL_MAX_TH            400U

/* Couleurs ARGB8888 */
#define COL_BG_TRANSPARENT      0x00000000U
#define COL_WHITE               0xFFFFFFFFU
#define COL_RED                 0xFFFF4040U
#define COL_GREEN               0xFF30C060U
#define COL_BLUE                0xFF3A7BFFU
#define COL_ORANGE              0xFFFFA020U
#define COL_GREY                0xFF707070U
#define COL_YELLOW              0xFFFFFF00U
#define COL_CYAN                0xFF00FFFFU
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ABS(x) ((x) < 0 ? -(x) : (x))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint8_t camera_frame_ready = 0U;
static uint8_t camera_initialized = 0U;

static AppMode_t app_mode = MODE_TRAIN;

static uint32_t sum_p1[FEATURE_SIZE];
static uint32_t sum_p2[FEATURE_SIZE];

static uint8_t proto_p1[FEATURE_SIZE];
static uint8_t proto_p2[FEATURE_SIZE];
static uint8_t feat_cur[FEATURE_SIZE];

static uint8_t count_p1 = 0U;
static uint8_t count_p2 = 0U;

static uint8_t last_result = 0U;      /* 0 = aucun, 1 = P1, 2 = P2 */
static uint32_t last_d1 = 0U;
static uint32_t last_d2 = 0U;

static char status_msg[64] = "Touchez P1+ ou P2+";
static char result_msg[64] = "";

/* Sobel */
static uint32_t sobel_threshold = SOBEL_THRESHOLD_DEFAULT;
static uint8_t gray_line_0[LCD_WIDTH];
static uint8_t gray_line_1[LCD_WIDTH];
static uint8_t gray_line_2[LCD_WIDTH];

static const TouchButton_t btn_p1   = {  8U, BTN_Y, BTN_W, BTN_H, BTN_P1_ADD, "P1 +"  };
static const TouchButton_t btn_p2   = {126U, BTN_Y, BTN_W, BTN_H, BTN_P2_ADD, "P2 +"  };
static const TouchButton_t btn_test = {244U, BTN_Y, BTN_W, BTN_H, BTN_TEST,   "TEST"  };
static const TouchButton_t btn_rst  = {362U, BTN_Y, BTN_W, BTN_H, BTN_RESET,  "RESET" };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t RGB565_ToGray(uint16_t c);
static void LCD_LL_ConvertLineToARGB8888(void *pSrc, void *pDst);
static void CameraFrameToLayer0(void);
static uint8_t CaptureOneFrozenFrame(void);

static void ExtractFeatureFromFrame(const uint16_t *frame, uint8_t *feat);
static void NormalizeFeature(uint8_t *feat);

static void ResetTraining(void);
static void AddSampleToPerson(uint8_t person, const uint8_t *feat);
static void FinalizeTraining(void);
static uint32_t DistanceL1(const uint8_t *a, const uint8_t *b);
static uint8_t ClassifyFeature(const uint8_t *feat);

static void DrawButton(const TouchButton_t *btn, uint32_t color, uint8_t enabled);
static void DrawROIFrame(void);
static void RedrawUI(void);
static TouchButtonId_t GetTouchedButton(uint16_t x, uint16_t y);

static void DoAddSample(uint8_t person);
static void DoTest(void);

/* Sobel */
static void Frame565_GetGrayRow(const uint16_t *frame, uint32_t row, uint8_t *dst);
static void ApplySobelOverlayToLayer0(const uint16_t *frame, uint32_t threshold, uint32_t edgeColor);

void BSP_CAMERA_FrameEventCallback(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t RGB565_ToGray(uint16_t c)
{
  uint32_t r = (c >> 11) & 0x1FU;
  uint32_t g = (c >> 5)  & 0x3FU;
  uint32_t b =  c        & 0x1FU;

  r = (r * 255U) / 31U;
  g = (g * 255U) / 63U;
  b = (b * 255U) / 31U;

  return (uint8_t)((77U * r + 150U * g + 29U * b) >> 8);
}

static void LCD_LL_ConvertLineToARGB8888(void *pSrc, void *pDst)
{
  hdma2d.Instance = DMA2D;

  hdma2d.Init.Mode         = DMA2D_M2M_PFC;
  hdma2d.Init.ColorMode    = DMA2D_ARGB8888;
  hdma2d.Init.OutputOffset = 0;

  hdma2d.LayerCfg[1].AlphaMode      = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha     = 0xFF;
  hdma2d.LayerCfg[1].InputColorMode = CM_RGB565;
  hdma2d.LayerCfg[1].InputOffset    = 0;

  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DMA2D_Start(&hdma2d,
                      (uint32_t)pSrc,
                      (uint32_t)pDst,
                      LCD_WIDTH,
                      1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DMA2D_PollForTransfer(&hdma2d, 100) != HAL_OK)
  {
    Error_Handler();
  }
}

static void CameraFrameToLayer0(void)
{
  uint32_t y;
  uint32_t src = CAMERA_FRAME_BUFFER;
  uint32_t dst = LAYER0_ADDRESS;

  for (y = 0U; y < LCD_HEIGHT; y++)
  {
    LCD_LL_ConvertLineToARGB8888((void *)src, (void *)dst);
    src += LCD_WIDTH * 2U;
    dst += LCD_WIDTH * 4U;
  }
}

static uint8_t CaptureOneFrozenFrame(void)
{
  uint32_t t0;

  if (camera_initialized != 0U)
  {
    BSP_CAMERA_DeInit();
    HAL_Delay(20);
    camera_initialized = 0U;
  }

  if (BSP_CAMERA_Init(CAMERA_RESOLUTION) != CAMERA_OK)
  {
    return 0U;
  }

  camera_initialized = 1U;
  HAL_Delay(50);

  camera_frame_ready = 0U;
  BSP_CAMERA_SnapshotStart((uint8_t *)CAMERA_FRAME_BUFFER);

  t0 = HAL_GetTick();
  while (camera_frame_ready == 0U)
  {
    if ((HAL_GetTick() - t0) > 1000U)
    {
      return 0U;
    }
    HAL_Delay(1);
  }

  CameraFrameToLayer0();
  return 1U;
}

static void ExtractFeatureFromFrame(const uint16_t *frame, uint8_t *feat)
{
  uint32_t x, y;
  uint32_t x0 = (LCD_WIDTH  - ROI_SIZE) / 2U;
  uint32_t y0 = (LCD_HEIGHT - ROI_SIZE) / 2U;

  for (y = 0U; y < FEATURE_H; y++)
  {
    for (x = 0U; x < FEATURE_W; x++)
    {
      uint32_t sx = x0 + (x * ROI_SIZE) / FEATURE_W;
      uint32_t sy = y0 + (y * ROI_SIZE) / FEATURE_H;

      if (sx >= LCD_WIDTH)
      {
        sx = LCD_WIDTH - 1U;
      }
      if (sy >= LCD_HEIGHT)
      {
        sy = LCD_HEIGHT - 1U;
      }

      feat[y * FEATURE_W + x] = RGB565_ToGray(frame[sy * LCD_WIDTH + sx]);
    }
  }
}

static void NormalizeFeature(uint8_t *feat)
{
  uint32_t i;
  uint8_t minv = 255U;
  uint8_t maxv = 0U;

  for (i = 0U; i < FEATURE_SIZE; i++)
  {
    if (feat[i] < minv) minv = feat[i];
    if (feat[i] > maxv) maxv = feat[i];
  }

  if (maxv == minv)
  {
    for (i = 0U; i < FEATURE_SIZE; i++)
    {
      feat[i] = 128U;
    }
    return;
  }

  for (i = 0U; i < FEATURE_SIZE; i++)
  {
    feat[i] = (uint8_t)((255U * (feat[i] - minv)) / (maxv - minv));
  }
}

static void ResetTraining(void)
{
  uint32_t i;

  for (i = 0U; i < FEATURE_SIZE; i++)
  {
    sum_p1[i] = 0U;
    sum_p2[i] = 0U;
    proto_p1[i] = 0U;
    proto_p2[i] = 0U;
    feat_cur[i] = 0U;
  }

  count_p1 = 0U;
  count_p2 = 0U;
  last_result = 0U;
  last_d1 = 0U;
  last_d2 = 0U;
  app_mode = MODE_TRAIN;

  snprintf(status_msg, sizeof(status_msg), "Touchez P1+ ou P2+");
  result_msg[0] = '\0';
}

static void AddSampleToPerson(uint8_t person, const uint8_t *feat)
{
  uint32_t i;

  if (person == 1U)
  {
    if (count_p1 >= N_TRAIN)
    {
      return;
    }

    for (i = 0U; i < FEATURE_SIZE; i++)
    {
      sum_p1[i] += feat[i];
    }
    count_p1++;
  }
  else
  {
    if (count_p2 >= N_TRAIN)
    {
      return;
    }

    for (i = 0U; i < FEATURE_SIZE; i++)
    {
      sum_p2[i] += feat[i];
    }
    count_p2++;
  }
}

static void FinalizeTraining(void)
{
  uint32_t i;

  if ((count_p1 == 0U) || (count_p2 == 0U))
  {
    return;
  }

  for (i = 0U; i < FEATURE_SIZE; i++)
  {
    proto_p1[i] = (uint8_t)(sum_p1[i] / count_p1);
    proto_p2[i] = (uint8_t)(sum_p2[i] / count_p2);
  }

  app_mode = MODE_RECO;
  snprintf(status_msg, sizeof(status_msg), "Apprentissage termine");
}

static uint32_t DistanceL1(const uint8_t *a, const uint8_t *b)
{
  uint32_t i;
  uint32_t d = 0U;

  for (i = 0U; i < FEATURE_SIZE; i++)
  {
    d += (uint32_t)ABS((int)a[i] - (int)b[i]);
  }

  return d;
}

static uint8_t ClassifyFeature(const uint8_t *feat)
{
  last_d1 = DistanceL1(feat, proto_p1);
  last_d2 = DistanceL1(feat, proto_p2);

  return (last_d1 <= last_d2) ? 1U : 2U;
}

static void DrawButton(const TouchButton_t *btn, uint32_t color, uint8_t enabled)
{
  uint32_t fill = enabled ? color : COL_GREY;

  BSP_LCD_SelectLayer(1);
  BSP_LCD_SetTextColor(fill);
  BSP_LCD_FillRect(btn->x, btn->y, btn->w, btn->h);

  BSP_LCD_SetTextColor(COL_WHITE);
  BSP_LCD_DrawRect(btn->x, btn->y, btn->w, btn->h);

  BSP_LCD_SetBackColor(fill);
  BSP_LCD_SetTextColor(COL_WHITE);
  BSP_LCD_DisplayStringAt(btn->x + 18U, btn->y + 12U, (uint8_t *)btn->label, LEFT_MODE);
}

static void DrawROIFrame(void)
{
  uint32_t x0 = (LCD_WIDTH  - ROI_SIZE) / 2U;
  uint32_t y0 = (LCD_HEIGHT - ROI_SIZE) / 2U;

  BSP_LCD_SelectLayer(1);
  BSP_LCD_SetTextColor(COL_YELLOW);
  BSP_LCD_DrawRect(x0, y0, ROI_SIZE, ROI_SIZE);
}

static void RedrawUI(void)
{
  char line0[80];
  char line1[80];
  char line2[80];
  char line3[80];

  BSP_LCD_SelectLayer(1);
  BSP_LCD_Clear(COL_BG_TRANSPARENT);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_SetBackColor(COL_BG_TRANSPARENT);

  DrawROIFrame();

  if (app_mode == MODE_TRAIN)
  {
    BSP_LCD_SetTextColor(COL_YELLOW);
    snprintf(line0, sizeof(line0), "TRAIN  P1:%u/%u  P2:%u/%u", count_p1, N_TRAIN, count_p2, N_TRAIN);
    snprintf(line1, sizeof(line1), "Touchez P1+ ou P2+ pour capturer");
    snprintf(line2, sizeof(line2), "%s", status_msg);
    snprintf(line3, sizeof(line3), "Sobel=%lu", (unsigned long)sobel_threshold);

    BSP_LCD_DisplayStringAtLine(1, (uint8_t *)line0);
    BSP_LCD_DisplayStringAtLine(2, (uint8_t *)line1);
    BSP_LCD_DisplayStringAtLine(3, (uint8_t *)line2);
    BSP_LCD_DisplayStringAtLine(4, (uint8_t *)line3);

    DrawButton(&btn_p1, COL_GREEN, 1U);
    DrawButton(&btn_p2, COL_BLUE, 1U);
    DrawButton(&btn_test, COL_ORANGE, 0U);
    DrawButton(&btn_rst, COL_RED, 1U);
  }
  else
  {
    BSP_LCD_SetTextColor(COL_CYAN);
    snprintf(line0, sizeof(line0), "RECO PRETE");
    snprintf(line1, sizeof(line1), "Touchez TEST pour classifier");
    snprintf(line2, sizeof(line2), "%s", status_msg);
    snprintf(line3, sizeof(line3), "Sobel=%lu", (unsigned long)sobel_threshold);

    BSP_LCD_DisplayStringAtLine(1, (uint8_t *)line0);
    BSP_LCD_DisplayStringAtLine(2, (uint8_t *)line1);
    BSP_LCD_DisplayStringAtLine(3, (uint8_t *)line2);
    BSP_LCD_DisplayStringAtLine(4, (uint8_t *)line3);

    if (last_result != 0U)
    {
      if (last_result == 1U)
      {
        BSP_LCD_SetTextColor(COL_GREEN);
      }
      else
      {
        BSP_LCD_SetTextColor(COL_BLUE);
      }

      BSP_LCD_DisplayStringAtLine(5, (uint8_t *)result_msg);

      BSP_LCD_SetTextColor(COL_WHITE);
      snprintf(line0, sizeof(line0), "d1=%lu   d2=%lu",
               (unsigned long)last_d1,
               (unsigned long)last_d2);
      BSP_LCD_DisplayStringAtLine(6, (uint8_t *)line0);
    }

    DrawButton(&btn_p1, COL_GREEN, 0U);
    DrawButton(&btn_p2, COL_BLUE, 0U);
    DrawButton(&btn_test, COL_ORANGE, 1U);
    DrawButton(&btn_rst, COL_RED, 1U);
  }
}

static TouchButtonId_t GetTouchedButton(uint16_t x, uint16_t y)
{
  const TouchButton_t *buttons[4] = { &btn_p1, &btn_p2, &btn_test, &btn_rst };
  uint32_t i;

  for (i = 0U; i < 4U; i++)
  {
    const TouchButton_t *b = buttons[i];
    if ((x >= b->x) && (x < (b->x + b->w)) &&
        (y >= b->y) && (y < (b->y + b->h)))
    {
      return b->id;
    }
  }

  return BTN_NONE;
}

/* ---- Sobel sur frame caméra RGB565 ---- */

static void Frame565_GetGrayRow(const uint16_t *frame, uint32_t row, uint8_t *dst)
{
  uint32_t x;
  const uint16_t *line = &frame[row * LCD_WIDTH];

  for (x = 0U; x < LCD_WIDTH; x++)
  {
    dst[x] = RGB565_ToGray(line[x]);
  }
}

static void ApplySobelOverlayToLayer0(const uint16_t *frame, uint32_t threshold, uint32_t edgeColor)
{
  uint8_t *l0 = gray_line_0;
  uint8_t *l1 = gray_line_1;
  uint8_t *l2 = gray_line_2;
  uint8_t *tmp;
  uint32_t *dst = (uint32_t *)LAYER0_ADDRESS;
  uint32_t x, y;

  if ((LCD_WIDTH < 3U) || (LCD_HEIGHT < 3U))
  {
    return;
  }

  Frame565_GetGrayRow(frame, 0U, l0);
  Frame565_GetGrayRow(frame, 1U, l1);
  Frame565_GetGrayRow(frame, 2U, l2);

  for (y = 1U; y < LCD_HEIGHT - 1U; y++)
  {
    for (x = 1U; x < LCD_WIDTH - 1U; x++)
    {
      int gx =
          -l0[x - 1U] + l0[x + 1U]
          -2 * l1[x - 1U] + 2 * l1[x + 1U]
          -l2[x - 1U] + l2[x + 1U];

      int gy =
          -l0[x - 1U] - 2 * l0[x] - l0[x + 1U]
          +l2[x - 1U] + 2 * l2[x] + l2[x + 1U];

      uint32_t mag = (uint32_t)(ABS(gx) + ABS(gy));

      if (mag > threshold)
      {
        dst[y * LCD_WIDTH + x] = edgeColor;
      }
    }

    if (y < LCD_HEIGHT - 2U)
    {
      tmp = l0;
      l0 = l1;
      l1 = l2;
      l2 = tmp;

      Frame565_GetGrayRow(frame, y + 2U, l2);
    }
  }
}

static void DoAddSample(uint8_t person)
{
  uint32_t edgeColor;

  if ((person == 1U) && (count_p1 >= N_TRAIN))
  {
    snprintf(status_msg, sizeof(status_msg), "P1 deja complete");
    return;
  }

  if ((person == 2U) && (count_p2 >= N_TRAIN))
  {
    snprintf(status_msg, sizeof(status_msg), "P2 deja complete");
    return;
  }

  if (!CaptureOneFrozenFrame())
  {
    snprintf(status_msg, sizeof(status_msg), "Erreur capture camera");
    return;
  }

  /* Sobel couleur selon la personne choisie en train */
  edgeColor = (person == 1U) ? COL_GREEN : COL_BLUE;
  ApplySobelOverlayToLayer0((const uint16_t *)CAMERA_FRAME_BUFFER, sobel_threshold, edgeColor);

  ExtractFeatureFromFrame((const uint16_t *)CAMERA_FRAME_BUFFER, feat_cur);
  NormalizeFeature(feat_cur);
  AddSampleToPerson(person, feat_cur);

  if ((count_p1 >= N_TRAIN) && (count_p2 >= N_TRAIN))
  {
    FinalizeTraining();
  }
  else
  {
    snprintf(status_msg, sizeof(status_msg), "Photo ajoutee a P%u", person);
  }
}

static void DoTest(void)
{
  uint32_t edgeColor;

  if (!CaptureOneFrozenFrame())
  {
    snprintf(status_msg, sizeof(status_msg), "Erreur capture camera");
    return;
  }

  ExtractFeatureFromFrame((const uint16_t *)CAMERA_FRAME_BUFFER, feat_cur);
  NormalizeFeature(feat_cur);

  last_result = ClassifyFeature(feat_cur);

  if (last_result == 1U)
  {
    snprintf(result_msg, sizeof(result_msg), "RESULTAT : PERSONNE 1");
    edgeColor = COL_GREEN;
  }
  else
  {
    snprintf(result_msg, sizeof(result_msg), "RESULTAT : PERSONNE 2");
    edgeColor = COL_BLUE;
  }

  /* Sobel couleur selon la personne détectée en test */
  ApplySobelOverlayToLayer0((const uint16_t *)CAMERA_FRAME_BUFFER, sobel_threshold, edgeColor);

  snprintf(status_msg, sizeof(status_msg), "Test capture");
}

void BSP_CAMERA_FrameEventCallback(void)
{
  camera_frame_ready = 1U;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  TS_StateTypeDef ts;
  uint8_t touch_lock = 0U;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC3_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_UART7_Init();

  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();

  BSP_LCD_LayerDefaultInit(0, LAYER0_ADDRESS);
  BSP_LCD_LayerDefaultInit(1, LAYER1_ADDRESS);
  BSP_LCD_DisplayOn();

  BSP_LCD_SelectLayer(0);
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  BSP_LCD_SelectLayer(1);
  BSP_LCD_Clear(COL_BG_TRANSPARENT);
  BSP_LCD_SetTransparency(1, 255);

  if (BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize()) != TS_OK)
  {
    Error_Handler();
  }

  ResetTraining();
  RedrawUI();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    BSP_TS_GetState(&ts);

    if (ts.touchDetected != 0U)
    {
      if (touch_lock == 0U)
      {
        TouchButtonId_t btn = GetTouchedButton(ts.touchX[0], ts.touchY[0]);
        touch_lock = 1U;

        switch (btn)
        {
          case BTN_P1_ADD:
            if (app_mode == MODE_TRAIN)
            {
              DoAddSample(1U);
            }
            else
            {
              snprintf(status_msg, sizeof(status_msg), "Utilisez RESET");
            }
            RedrawUI();
            HAL_Delay(150);
            break;

          case BTN_P2_ADD:
            if (app_mode == MODE_TRAIN)
            {
              DoAddSample(2U);
            }
            else
            {
              snprintf(status_msg, sizeof(status_msg), "Utilisez TEST");
            }
            RedrawUI();
            HAL_Delay(150);
            break;

          case BTN_TEST:
            if (app_mode == MODE_RECO)
            {
              DoTest();
            }
            else
            {
              snprintf(status_msg, sizeof(status_msg), "Finissez l'apprentissage");
            }
            RedrawUI();
            HAL_Delay(150);
            break;

          case BTN_RESET:
            ResetTraining();
            BSP_LCD_SelectLayer(0);
            BSP_LCD_Clear(LCD_COLOR_BLACK);
            RedrawUI();
            HAL_Delay(150);
            break;

          default:
            HAL_Delay(100);
            break;
        }
      }
    }
    else
    {
      touch_lock = 0U;
    }

    HAL_Delay(10);
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
