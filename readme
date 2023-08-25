# LCD Library

The LCD Library provides a set of functions for controlling an LCD display. It allows you to initialize the display, clear the screen, set the cursor position, print text and numbers, draw bitmaps, lines, rectangles, and 3D cubes.

## Features

- Initialize the LCD display
- Clear the screen
- Set the cursor position
- Print text and numbers
- Draw bitmaps, lines, rectangles, and 3D cube demo

## Installation

1. Clone the repository or download the library files.
2. Include the necessary header files in your project.
3. Link the library files with your project.
4. Make sure to configure the necessary pins and SPI settings for your specific hardware.

## Usage

1. Include the `LCD.hpp` header file in your project and create an instance of the LCD class.
2. Initialize the LCD display using the `init()` function.
3. Use the various functions provided by the library to control the LCD display.

#include "LCD.hpp"

int main(void)
{

    LCD N_LCD(&hspi2, RST_PIN_GPIO_Port, RST_PIN_Pin, DC_PIN_GPIO_Port, DC_PIN_Pin);

    

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SPI2_Init();
    /* USER CODE BEGIN 2 */
    N_LCD.init();

    // Clear the screen
    N_LCD.clear();

    // Set the cursor position
    N_LCD.setCursor(0, 0);

    // Print text
    N_LCD.print("Hello, LCD!");

    // Draw a line
    N_LCD.drawLine(0, 0, 84, 48);

    // Draw a filled rectangle
    N_LCD.drawRectangleFilled(10, 10, 30, 20);

    // Update the LCD display
    N_LCD.buffer_Update();
}

## Contributing

Contributions are welcome! If you find any issues or have suggestions for improvements, please open an issue or submit a pull request on the GitHub repository.


## License

This program is licensed under the GNU Lesser General Public License, version 3.0. You can redistribute and modify this program under the terms of the license. See the [LICENSE](LICENSE) file for more details.

## Author

This program is created by Jalal. You can contact the author at mrhazerberg614@gmail.com. Visit the author's GitHub page at [https://github.com/jalal82/](https://github.com/jalal82/) for more information.
