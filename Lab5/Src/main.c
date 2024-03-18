/**
 *
 * Monthon Paul
 * u1274364
 *
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
void _Error_Handler(char* file, int line);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {
    /**
     * Part 1 of the Lab5
     */
    HAL_Init();
    SystemClock_Config();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

    // Initialize LED pins
    GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_7,
                                GPIO_MODE_OUTPUT_PP,
                                GPIO_SPEED_FREQ_LOW,
                                GPIO_NOPULL};
    HAL_GPIO_Init(GPIOC, &initStr);  // Initialize LED pins

    // Setting up PB11 and PB13
    GPIOB->MODER &= ~(GPIO_MODER_MODER11_0 | GPIO_MODER_MODER13_0);
    GPIOB->MODER |= (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER13_1);

    GPIOB->OTYPER |= GPIO_OTYPER_OT_11;
    GPIOB->OTYPER |= GPIO_OTYPER_OT_13;

    GPIOB->AFR[1] |= (1 << 12);
    GPIOB->AFR[1] |= (5 << 20);

    // Setting up PB14
    GPIOB->MODER |= GPIO_MODER_MODER14_0;
    GPIOB->MODER &= ~(GPIO_MODER_MODER14_1);

    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_14);

    GPIOB->ODR |= GPIO_ODR_14;

    // Setting up PC0
    GPIOC->MODER |= GPIO_MODER_MODER0_0;
    GPIOC->MODER &= ~(GPIO_MODER_MODER0_1);

    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_0);

    GPIOC->ODR |= GPIO_ODR_0;

    // Leave PB15 on input mode
    GPIOB->MODER &= ~(GPIO_MODER_MODER15_0 | GPIO_MODER_MODER15_1);

    // Setting up I2C2
    I2C2->TIMINGR |= (1 << 28) | 0x13 | (0xF << 8) | (0x2 << 16) | (0x4 << 20);

    I2C2->CR1 |= I2C_CR1_PE;

    /* Clear the NBYTES and SADD bit fields
     * The NBYTES field begins at bit 16, the SADD at bit 0
     */
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
    /* Set NBYTES = 1 and SADD = 0x69
     * Can use hex or decimal values directly as bitmasks.
     * Remember that for 7-bit addresses, the lowest SADD bit
     * is not used and the mask must be shifted by one.
     */
    I2C2->CR2 |= (1 << 16) | (0x69 << 1);

    // RD_WRN to write
    I2C2->CR2 &= ~(I2C_CR2_RD_WRN);
    // Start
    I2C2->CR2 |= I2C_CR2_START;

    // wait until TXIS or NACKF flags are set
    while (!(I2C2->ISR & I2C_ISR_TXIS))
        ;
    if (I2C2->ISR & I2C_ISR_NACKF)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);  // Error State
    // write who_am_I reg into I2C transmit register
    I2C2->TXDR |= 0x0F;
    while (!(I2C2->ISR & I2C_ISR_TC))
        ; /* loop waiting for TC */
    // Reload the CR2 register
    // setting SADD & NBYTES
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
    I2C2->CR2 |= (1 << 16) | (0x69 << 1);

    // reset RD_WRN to read
    I2C2->CR2 |= I2C_CR2_RD_WRN;
    // reset start bit
    I2C2->CR2 |= I2C_CR2_START;

    // wait until RXNE or NACKF flags are set
    while (!(I2C2->ISR & I2C_ISR_RXNE))
        ;
    if (I2C2->ISR & I2C_ISR_NACKF)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);  // Error State
    while (!(I2C2->ISR & I2C_ISR_TC))
        ; /* loop waiting for TC */

    // Check the contents of the RXDR register to see if it matches the correct value of WHO_AM_I register
    if (I2C2->RXDR == 0xD3)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    // stop
    I2C2->CR2 |= I2C_CR2_STOP;
    // End of Part 1 Checkoff 

    /**
     * Part 2 of the Lab5: GYROSCOPE
     */
    // Following transmit I2C protocol beginning flowchart
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
    I2C2->CR2 |= (1 << 16) | (0x69 << 1);
    // RD_WRN to write
    I2C2->CR2 &= ~(I2C_CR2_RD_WRN);
    // Start
    I2C2->CR2 |= I2C_CR2_START;

    // wait until TXIS or NACKF flags are set
    while (!(I2C2->ISR & I2C_ISR_TXIS))
        ;
    if (I2C2->ISR & I2C_ISR_NACKF)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);  // Error State
    // write CTRL_REG1 into I2C transmit register & set PD to Xen & Yen
    I2C2->TXDR |= (0x20 | 0x0B);
    while (!(I2C2->ISR & I2C_ISR_TC))
        ; /* loop waiting for TC */
    // stop
    I2C2->CR2 |= I2C_CR2_STOP;

    char x_LSB, x_MSB, y_LSB, y_MSB;
    int x_data, y_data;
    while (1) {
        // Reset LEDS
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

        /* x_LSB */
        // Following transmit I2C protocol beginning flowchart
        I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
        I2C2->CR2 |= (1 << 16) | (0x69 << 1);
        // RD_WRN to write
        I2C2->CR2 &= ~(I2C_CR2_RD_WRN);
        // Start
        I2C2->CR2 |= I2C_CR2_START;

        // wait until TXIS or NACKF flags are set
        while (!(I2C2->ISR & I2C_ISR_TXIS))
            ;
        if (I2C2->ISR & I2C_ISR_NACKF)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        // write CTRL_REG1 into I2C transmit register
        I2C2->TXDR |= 0x28;
        while (!(I2C2->ISR & I2C_ISR_TC))
            ; /* loop waiting for TC */

        // Reload the CR2 register
        // setting SADD & NBYTES
        I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
        I2C2->CR2 |= (1 << 16) | (0x69 << 1);
        // reset RD_WRN to read
        I2C2->CR2 |= I2C_CR2_RD_WRN;
        // reset start bit
        I2C2->CR2 |= I2C_CR2_START;

        // wait until RXNE or NACKF flags are set
        while (!(I2C2->ISR & I2C_ISR_RXNE))
            ;
        if (I2C2->ISR & I2C_ISR_NACKF)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        while (!(I2C2->ISR & I2C_ISR_TC))
            ; /* loop waiting for TC */

        x_LSB = I2C2->RXDR & I2C_RXDR_RXDATA;

        /* x_MSB */
        // Following transmit I2C protocol beginning flowchart
        I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
        I2C2->CR2 |= (1 << 16) | (0x69 << 1);
        // RD_WRN to write
        I2C2->CR2 &= ~(I2C_CR2_RD_WRN);
        // Start
        I2C2->CR2 |= I2C_CR2_START;

        // wait until TXIS or NACKF flags are set
        while (!(I2C2->ISR & I2C_ISR_TXIS))
            ;
        if (I2C2->ISR & I2C_ISR_NACKF)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        // write CTRL_REG1 into I2C transmit register
        I2C2->TXDR |= 0x29;
        while (!(I2C2->ISR & I2C_ISR_TC))
            ; /* loop waiting for TC */

        // Reload the CR2 register
        // setting SADD & NBYTES
        I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
        I2C2->CR2 |= (1 << 16) | (0x69 << 1);
        // reset RD_WRN to read
        I2C2->CR2 |= I2C_CR2_RD_WRN;
        // reset start bit
        I2C2->CR2 |= I2C_CR2_START;

        // wait until RXNE or NACKF flags are set
        while (!(I2C2->ISR & I2C_ISR_RXNE))
            ;
        if (I2C2->ISR & I2C_ISR_NACKF)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        while (!(I2C2->ISR & I2C_ISR_TC))
            ; /* loop waiting for TC */

        x_MSB = I2C2->RXDR & I2C_RXDR_RXDATA;
        x_data = x_LSB | (x_MSB << 8);

        /* y_LSB */
        // Following transmit I2C protocol beginning flowchart
        I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
        I2C2->CR2 |= (1 << 16) | (0x69 << 1);
        // RD_WRN to write
        I2C2->CR2 &= ~(I2C_CR2_RD_WRN);
        // Start
        I2C2->CR2 |= I2C_CR2_START;

        // wait until TXIS or NACKF flags are set
        while (!(I2C2->ISR & I2C_ISR_TXIS))
            ;
        if (I2C2->ISR & I2C_ISR_NACKF)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        // write CTRL_REG1 into I2C transmit register
        I2C2->TXDR |= 0x2A;
        while (!(I2C2->ISR & I2C_ISR_TC))
            ; /* loop waiting for TC */

        // Reload the CR2 register
        // setting SADD & NBYTES
        I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
        I2C2->CR2 |= (1 << 16) | (0x69 << 1);
        // reset RD_WRN to read
        I2C2->CR2 |= I2C_CR2_RD_WRN;
        // reset start bit
        I2C2->CR2 |= I2C_CR2_START;

        // wait until RXNE or NACKF flags are set
        while (!(I2C2->ISR & I2C_ISR_RXNE))
            ;
        if (I2C2->ISR & I2C_ISR_NACKF)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        while (!(I2C2->ISR & I2C_ISR_TC))
            ; /* loop waiting for TC */

        y_LSB = I2C2->RXDR & I2C_RXDR_RXDATA;

        /* y_MSB */
        // Following transmit I2C protocol beginning flowchart
        I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
        I2C2->CR2 |= (1 << 16) | (0x69 << 1);
        // RD_WRN to write
        I2C2->CR2 &= ~(I2C_CR2_RD_WRN);
        // Start
        I2C2->CR2 |= I2C_CR2_START;

        // wait until TXIS or NACKF flags are set
        while (!(I2C2->ISR & I2C_ISR_TXIS))
            ;
        if (I2C2->ISR & I2C_ISR_NACKF)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        // write CTRL_REG1 into I2C transmit register
        I2C2->TXDR |= 0x2B;
        while (!(I2C2->ISR & I2C_ISR_TC))
            ; /* loop waiting for TC */

        // Reload the CR2 register
        // setting SADD & NBYTES
        I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
        I2C2->CR2 |= (1 << 16) | (0x69 << 1);
        // reset RD_WRN to read
        I2C2->CR2 |= I2C_CR2_RD_WRN;
        // reset start bit
        I2C2->CR2 |= I2C_CR2_START;

        // wait until RXNE or NACKF flags are set
        while (!(I2C2->ISR & I2C_ISR_RXNE))
            ;
        if (I2C2->ISR & I2C_ISR_NACKF)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        while (!(I2C2->ISR & I2C_ISR_TC))
            ; /* loop waiting for TC */

        y_MSB = I2C2->RXDR & I2C_RXDR_RXDATA;
        y_data = y_LSB | (y_MSB << 8);
        // stop
        I2C2->CR2 |= I2C_CR2_STOP;

        /* turning LEDs on*/
        if (x_data > 10)  // turns on orange LED if X is +
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
        if (x_data < -10)  // turns on green LED if X is -
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
        if (y_data > 10)  // turns on red LED if Y is +
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        if (y_data < -10)  // turns on blue LED if Y is -
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
        // delay for reading
        HAL_Delay(100);
    }
}

/** System Clock Configuration
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time
     */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**Configure the Systick
     */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char* file, int line) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
