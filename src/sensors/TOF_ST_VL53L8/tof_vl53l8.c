#include "tof_vl53l8.h"
#include "VL53L8CX_ULD_driver_1.2.1/VL53L8CX_ULD_API/inc/vl53l8cx_api.h"
#include "VL53L8CX_ULD_driver_1.2.1/VL53L8CX_ULD_API/inc/vl53l8cx_plugin_motion_indicator.h"
#include "VL53L8CX_ULD_driver_1.2.1/VL53L8CX_ULD_API/inc/vl53l8cx_plugin_detection_thresholds.h"
// #include "VL53LMZ_ULD_v2.0.10/VL53LMZ_ULD_API/inc/vl53lmz_api.h"

const struct gpio_dt_spec spi_i2c_n = GPIO_DT_SPEC_GET(DT_ALIAS(spii2cn), gpios);
const struct gpio_dt_spec LPn = GPIO_DT_SPEC_GET(DT_ALIAS(lpn), gpios);
const struct gpio_dt_spec pwren = GPIO_DT_SPEC_GET(DT_ALIAS(pwren), gpios);
const struct i2c_dt_spec vl53l8_1 = I2C_DT_SPEC_GET(VL53L8_1_NODE);
struct spi_cs_control tof_cs = {
    .gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(vl53lcx8)),
    .delay = 0,
};
const struct spi_config spi_tof_cfg = {
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
                 SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_HOLD_ON_CS,
    .frequency = 5000000, // 500000,//1000000,
    .slave = 0,
    .cs = &tof_cs,
};

int WaitForL5Interrupt(VL53L8CX_Configuration *pDev)
{
  // while ( !check_for_gpio_interrupt() );
  return (1);
}

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// I2C_HandleTypeDef hi2c1;

// UART_HandleTypeDef huart2;
// DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
// void SystemClock_Config(void);
// static void MX_GPIO_Init(void);
// static void MX_I2C1_Init(void);
// static void MX_DMA_Init(void);
// static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /*if (GPIO_Pin==INT_C_Pin)
  {
    IntCount++;
  }
    */
}

int check_for_gpio_interrupt(void)
{
  return 0;
}

void Reset_Sensor_via_GPIO(void)
{
#ifdef GPIO_FOR_LPn_PIN
  /* Set 0 to pin LPn = PB0 */
  // HAL_GPIO_WritePin(LPn_GPIO_Port, LPn_Pin, GPIO_PIN_RESET);
  gpio_pin_set_dt(&LPn, GPIO_PIN_RESET);
#endif

#ifdef GPIO_FOR_PWR_EN
  /* Set 0 to pin PWR_EN = PB0 or PA7 */
  gpio_pin_set_dt(&pwren, GPIO_PIN_RESET);
  k_msleep(100);

  /* Set 1 to pin PWR_EN */
  gpio_pin_set_dt(&pwren, GPIO_PIN_SET);
  k_msleep(100);
#endif

#ifdef GPIO_FOR_LPn_PIN
  /* Set 1 to pin LPn */
  // HAL_GPIO_WritePin(LPn_GPIO_Port, LPn_Pin, GPIO_PIN_SET);
  // Set for I2C and RESET for SPI
  gpio_pin_set_dt(&LPn, GPIO_PIN_RESET);
#endif

  // If I2C master does not support "I2C bus clear" feature, need to toggle I2C_SPI_N pin to release I2C bus
  gpio_pin_set_dt(&spi_i2c_n, GPIO_PIN_SET);
  k_msleep(1);
  gpio_pin_set_dt(&spi_i2c_n, GPIO_PIN_RESET);
  k_msleep(1);
  gpio_pin_set_dt(&spi_i2c_n, GPIO_PIN_SET);
  k_msleep(1);

  return;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
void tof_vl5318_thread(void)
{
  /*********************************/
  /*   VL53L8CX ranging variables  */
  /*********************************/

  uint8_t status, loop, isAlive, isReady, i;
  VL53L8CX_Configuration Dev;   /* Sensor configuration */
  VL53L8CX_ResultsData Results; /* Results data from VL53L8CX */
  uint8_t resolution;
  /* USER CODE END 1 */

  gpio_pin_configure_dt(&spi_i2c_n, GPIO_OUTPUT);
  gpio_pin_configure_dt(&LPn, GPIO_OUTPUT);
  gpio_pin_configure_dt(&pwren, GPIO_OUTPUT);
  Reset_Sensor_via_GPIO();

  printf("\n\nVL53LMZ_ULD_TEST\n\n");

  Dev.platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;
  //Dev.platform.i2cSpec = &vl53l8_1;
  Dev.platform.spi_tof_cfg = &spi_tof_cfg;
  Dev.platform.spi_dev = spi_dev;
  Dev.platform.i2c_spi = 1;

  status = vl53l8cx_is_alive(&Dev, &isAlive);
  if (!isAlive || status)
  {
    printf("VL53L8CX not detected at requested address\n");
    return status;
  }
  /* (Mandatory) Init VL53L8CX sensor */
  status = vl53l8cx_init(&Dev);
  if (status)
  {
    printf("VL53L8CX ULD Loading failed status error %d \n", status);
    return status;
  }

  status = vl53l8cx_set_ranging_mode(&Dev, VL53L8CX_RANGING_MODE_AUTONOMOUS); // Set mode continuous
  if (status)
    printf("Error in setting mode\n");

  status = vl53l8cx_start_ranging(&Dev);
  if (status)
    printf("Error in starting ranging\n");

  /* USER CODE BEGIN 3 */
  for (;;)
  {
    status = vl53l8cx_check_data_ready(&Dev, &isReady);
    if (isReady)
    {
      vl53l8cx_get_ranging_data(&Dev, &Results);
			printf("Print data no : %3u\n", Dev.streamcount);
      for (i = 0; i < 16; i++)
      {
        printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
               i,
               Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i],
               Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i]);
      }
      printf("\n");
    }
    else
    {
      k_yield();
      k_msleep(5);
    }
  }
  /* USER CODE END 3 */
}

/* --------------------------------------------------
 * --- This function can be used to test VL53L5CX I2C
 * --------------------------------------------------
 */

uint8_t vl53l5cx_test_i2c(VL53L8CX_Configuration *p_dev)
{
  uint8_t status = 0;
  printf("Starting VL53L5CX I2C test...\n");

  /* To check the I2C RdByte/WrByte function :
   * Inside the function “vl53l5cx_is_alive()”, it will call I2C RdByte/WrByte to
   * read device and verion ID. which can help you to verify the I2C RdByte/WrByte
   * functions at same time.
   */
  uint8_t device_id, revision_id;
  status |= WrByte(&(p_dev->platform), 0x7fff, 0x00);
  status |= RdByte(&(p_dev->platform), 0, &device_id);
  status |= RdByte(&(p_dev->platform), 1, &revision_id);
  status |= WrByte(&(p_dev->platform), 0x7fff, 0x02);

  if (status)
  {
    printf("Error Rd/Wr byte: status %u\n", status);
    return status;
  }

  /* To check the I2C RdMulti/WrMulti function:
   * Below is example codes which can help you vefify the I2C RdMulti/WrMulti
   * function.
   */

  uint8_t Data_write[4] = {0x5A, 0xA5, 0xAA, 0x55};
  uint8_t Data_read[4] = {0, 0, 0, 0};
  uint8_t Data_default[4] = {0, 0, 0, 0};

  status |= RdMulti(&(p_dev->platform), 0x100, Data_default, 4);
  if (status)
  {
    printf("Error RdMulti: status %u\n", status);
    return status;
  }

  printf("Read default value and save it at begging\n");
  printf("Data_default (0x%x)\n", Data_default[0]);
  printf("Data_default (0x%x)\n", Data_default[1]);
  printf("Data_default (0x%x)\n", Data_default[2]);
  printf("Data_default (0x%x)\n", Data_default[3]);

  status |= WrMulti(&(p_dev->platform), 0x100, Data_write, 4);
  if (status)
  {
    printf("Error WrMulti: status %u\n", status);
    return status;
  }
  printf("Writing values 0x5A 0xA5 0xAA 0x55\n");

  status |= RdMulti(&(p_dev->platform), 0x100, Data_read, 4);
  if (status)
  {
    printf("Error RdMulti: status %u\n", status);
    return status;
  }

  printf("Reading:\n");
  printf("Data_read (0x%x)\n", Data_read[0]);
  printf("Data_read (0x%x)\n", Data_read[1]);
  printf("Data_read (0x%x)\n", Data_read[2]);
  printf("Data_read (0x%x)\n", Data_read[3]);

  status |= WrMulti(&(p_dev->platform), 0x100, Data_default, 4);
  printf("Write back default value\n");
  if (status)
  {
    printf("Error WrMulti: status %u\n", status);
    return status;
  }

  status |= RdMulti(&(p_dev->platform), 0x100, Data_default, 4);
  if (status)
  {
    printf("Error RdMulti: status %u\n", status);
    return status;
  }

  printf("Read value again to make sure default value was correct loaded\n");
  printf("Data_default (0x%x)\n", Data_default[0]);
  printf("Data_default (0x%x)\n", Data_default[1]);
  printf("Data_default (0x%x)\n", Data_default[2]);
  printf("Data_default (0x%x)\n", Data_default[3]);

  printf("I2C test done - everything works fine.\n");

  return status;
}
