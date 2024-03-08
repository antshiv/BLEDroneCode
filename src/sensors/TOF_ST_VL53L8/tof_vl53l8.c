#include "tof_vl53l8.h"
// #include "VL53LMZ_ULD_v2.0.10/VL53LMZ_ULD_API/inc/vl53lmz_api.h"

const struct gpio_dt_spec spi_i2c_n = GPIO_DT_SPEC_GET(DT_ALIAS(spii2cn), gpios);
const struct gpio_dt_spec LPn = GPIO_DT_SPEC_GET(DT_ALIAS(lpn), gpios);
const struct gpio_dt_spec pwren = GPIO_DT_SPEC_GET(DT_ALIAS(pwren), gpios);
const struct i2c_dt_spec vl53l8_1 = I2C_DT_SPEC_GET(VL53L8_1_NODE);

int WaitForL5Interrupt(VL53LMZ_Configuration *pDev)
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
  gpio_pin_set_dt(&LPn, GPIO_PIN_SET);
#endif

  // If I2C master does not support "I2C bus clear" feature, need to toggle I2C_SPI_N pin to release I2C bus
  gpio_pin_set_dt(&spi_i2c_n, GPIO_PIN_RESET);
  k_msleep(1);
  gpio_pin_set_dt(&spi_i2c_n, GPIO_PIN_SET);
  k_msleep(1);
  gpio_pin_set_dt(&spi_i2c_n, GPIO_PIN_RESET);
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
  /* USER CODE BEGIN 1 */
  int status;
  /* USER CODE END 1 */

  gpio_pin_configure_dt(&spi_i2c_n, GPIO_OUTPUT);
  gpio_pin_configure_dt(&LPn, GPIO_OUTPUT);
  gpio_pin_configure_dt(&pwren, GPIO_OUTPUT);
  Reset_Sensor_via_GPIO();

  printf("\n\nVL53LMZ_ULD_TEST\n\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  status = 0;
  while (status == 0)
  {
    // status = example1();
    // status = example2();
    // status = example5();
    printk("Running example11\n");
    status = example1(&vl53l8_1);
    // continous_mode(&vl53l8_1);
    printk("Finished example11\n");
    k_yield();
    k_msleep(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  for (;;)
  {
    k_msleep(1000);
  }
  /* USER CODE END 3 */
}

/* --------------------------------------------------
 * --- This function can be used to test VL53L5CX I2C
 * --------------------------------------------------
 */

uint8_t vl53l5cx_test_i2c(VL53LMZ_Configuration *p_dev)
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
