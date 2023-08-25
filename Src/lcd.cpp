/**
 * @file LCD.hpp
 * Head file of LCD class
 *
 * Copyright (C) Jalal
 * mrhazerberg614@@gmail.com
 * https://github.com/jalal82/
 *
 * This program is a free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "lCD.hpp"
#include <cmath> // Add this line for math functions
#include <cstdio>
#include <cstring>

/**
 * @brief  LCD constructor.
 * @param  hspi: Pointer to the SPI_HandleTypeDef struct.
 * @param  rstPort: Pointer to the GPIO_TypeDef struct for the RST pin.
 * @param  rstPin: RST pin number.
 * @param  dcPort: Pointer to the GPIO_TypeDef struct for the DC pin.
 * @param  dcPin: DC pin number.
 * @retval None
 */
LCD::LCD(SPI_HandleTypeDef* hspi, GPIO_TypeDef* rstPort, uint16_t rstPin, GPIO_TypeDef* dcPort, uint16_t dcPin)
    : hspi(hspi)
    , rstPort(rstPort)
    , rstPin(rstPin)
    , dcPort(dcPort)
    , dcPin(dcPin)
{
}

/**
 * @brief  Initializes the LCD display.
 * @retval None
 */
void LCD::init()
{
    HAL_GPIO_WritePin(rstPort, rstPin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(rstPort, rstPin, GPIO_PIN_SET);

    sendCommand(0x21); // LCD Extended Commands
    sendCommand(0xB8); // Set LCD Vop (Contrast)
    sendCommand(0x04); // Set Temp coefficient
    sendCommand(0x14); // LCD bias mode 1:48
    sendCommand(0x20); // LCD Standard Commands, Horizontal addressing mode
    sendCommand(0x0C); // LCD in normal mode
}

/**
 * @brief  Clears the LCD display by setting all pixels to 0.
 * @retval None
 */
void LCD::clear()
{
    memset(buffer, 0, 504);
    setCursor(0, 0);
    for (int i = 0; i < 504; i++) {
        sendData(0x00);
    }
}

/**
 * @brief  Sets the cursor position on the LCD display.
 * @param  row: Row number.
 * @param  col: Column number.
 *
 */
void LCD::setCursor(uint8_t row, uint8_t col) { sendCommand(0x80 | (row * 84 + col)); }

/**
 * @brief  Prints a string on the LCD display.
 * @param  str: Pointer to the string to be printed.
 * @retval None
 */
void LCD::print(const char* str)
{
    while (*str) {
        for (int i = 0; i <= 5; i++) {
            sendData(ASCII[*str - 0x20][i]);
        }
        sendData(0x00); // Add space between characters
        str++;
    }
}

/**
 * @brief  Prints a number on the LCD display.
 * @param  num: Number to be printed.
 * @retval None
 */
void LCD::print_nbr(uint32_t num)
{
    char buffer[11];
    sprintf(buffer, "%lu", num);
    print(buffer);
}

/**
 * @brief  Draws a bitmap image on the LCD display.
 * @param  bitmap: Pointer to the bitmap image data.
 * @param  width: Width of the bitmap image.
 * @param  height: Height of the bitmap image.
 * @retval None
 */
void LCD::drawBitmap(const uint8_t* bitmap, uint8_t width, uint8_t height)
{
    for (int i = 0; i < width * height / 8; i++) {
        sendData(bitmap[i]);
    }
}

void LCD::sendCommand(uint8_t command)
{
    HAL_GPIO_WritePin(dcPort, dcPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, &command, 1, HAL_MAX_DELAY);
}

void LCD::sendData(uint8_t data)
{
    HAL_GPIO_WritePin(dcPort, dcPin, GPIO_PIN_SET);
    HAL_SPI_Transmit(hspi, &data, 1, HAL_MAX_DELAY);
}

/**
 * @brief  Draws a line on the LCD display.
 * @param  x0: X-coordinate of the starting point.
 * @param  y0: Y-coordinate of the starting point.
 * @param  x1: X-coordinate of the ending point.
 * @param  y1: Y-coordinate of the ending point.
 * @retval None
 */
void LCD::drawLine(int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    // Set pixel at (x0, y0) on the LCD
    setPixel(x0, y0);

    while (x0 != x1 || y0 != y1) {
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }

        // Set pixel at (x0, y0) on the LCD
        setPixel(x0, y0);
    }
}

void LCD::setPixel(int x, int y)
{
    int index = (y / 8) * 84 + x;
    int bit = y % 8;

    buffer[index] |= (1 << bit);

    // // Calculate the page and column of the pixel
    // int page = y / 8;
    // int column = x;

    // // Set the LCD address to the pixel location
    // sendCommand(0x80 | column);
    // sendCommand(0x40 | page);
}

/**
 * @brief Draws a rectangle on the LCD display.
 * @param x: X-coordinate of the top-left corner of the rectangle.
 * @param y: Y-coordinate of the top-left corner of the rectangle.
 * @param width: Width of the rectangle.
 * @param height: Height of the rectangle.
 * @retval None
 */
void LCD::drawRectangle(int x, int y, int width, int height)
{
    // Draw top and bottom horizontal lines
    for (int i = x; i < x + width; i++) {
        setPixel(i, y);
        setPixel(i, y + height - 1);
    }

    // Draw left and right vertical lines
    for (int i = y; i < y + height; i++) {
        setPixel(x, i);
        setPixel(x + width - 1, i);
    }
}

/**
 * @brief Draws a filled rectangle on the LCD display.
 * @param x: X-coordinate of the top-left corner of the rectangle.
 * @param y: Y-coordinate of the top-left corner of the rectangle.
 * @param width: Width of the rectangle.
 * @param height: Height of the rectangle.
 * @retval None
 */
void LCD::drawRectangleFilled(int x, int y, int width, int height)
{
    for (int i = y; i < y + height; i++) {
        for (int j = x; j < x + width; j++) {
            setPixel(j, i);
        }
    }
}

/**
 * @brief Updates a specific portion of the LCD display with new data in buffer.
 * @param x0: X-coordinate of the top-left corner of the portion.
 * @param y0: Y-coordinate of the top-left corner of the portion.
 * @param x1: X-coordinate of the bottom-right corner of the portion.
 * @param y1: Y-coordinate of the bottom-right corner of the portion.
 * @retval None
 */
void LCD::aria_Update(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    for (int i = 0; i < 6; i++) {
        if (i * 8 > y1) {
            break;
        }
        // LCD_goXY(xmin, i);
        sendCommand(LCD_SETYADDR | i);
        sendCommand(LCD_SETXADDR | x0);
        for (int j = x0; j <= x1; j++) {
            sendData(buffer[(i * LCD_WIDTH) + j]);
        }
    }
}

/**
 * @brief Updates the entire buffer on the LCD display.
 * @retval None
 */
void LCD::buffer_Update()
{
    sendCommand(0x80 | LCD_SETXADDR); // Column.
    sendCommand(0x40 | LCD_SETYADDR); // Row.
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < LCD_WIDTH; j++) {
            sendData(buffer[(i * LCD_WIDTH) + j]);
        }
    }
}

void LCD::rotateCube(double angleX, double angleY)
{
    double sinX = sin(angleX);
    double cosX = cos(angleX);
    double sinY = sin(angleY);
    double cosY = cos(angleY);

    for (int i = 0; i < 8; i++) {
        double x = cubeVertices[i][0];
        double y = cubeVertices[i][1];
        double z = cubeVertices[i][2];

        // Rotate around the X-axis
        double rotatedX = x * cosX - z * sinX;
        double rotatedZ = z * cosX + x * sinX;
        z = rotatedZ;

        // Rotate around the Y-axis
        double rotatedY = y * cosY - z * sinY;
        rotatedZ = z * cosY + y * sinY;

        // Apply tolerance for doubleing-point comparisons
        if (fabs(rotatedX) < 1e-6) {
            rotatedX = 0.0;
        }
        if (fabs(rotatedY) < 1e-6) {
            rotatedY = 0.0;
        }
        if (fabs(rotatedZ) < 1e-6) {
            rotatedZ = 0.0;
        }

        cubeVertices[i][0] = rotatedX;
        cubeVertices[i][1] = rotatedY;
        cubeVertices[i][2] = rotatedZ;
    }
}

/**
 * @brief Resets the cube vertices to their initial positions.
 * @retval None
 */
void LCD::reset_cube()
{
    static double tempCubeVertices[8][3] = { { -1, -1, -1 }, { -1, -1, 1 }, { -1, 1, -1 }, { -1, 1, 1 }, { 1, -1, -1 }, { 1, -1, 1 }, { 1, 1, -1 }, { 1, 1, 1 } };
    memcpy(cubeVertices, tempCubeVertices, sizeof(cubeVertices));
}

void LCD::drawCube()
{
    clear();

    for (int i = 0; i < 12; i++) {
        int startVertex = cubeEdges[i][0];
        int endVertex = cubeEdges[i][1];
        int startX = (int)(cubeVertices[startVertex][0] * 15) + LCD_WIDTH / 2;
        int startY = (int)(cubeVertices[startVertex][1] * 15) + LCD_HEIGHT / 2;
        int endX = (int)(cubeVertices[endVertex][0] * 15) + LCD_WIDTH / 2;
        int endY = (int)(cubeVertices[endVertex][1] * 15) + LCD_HEIGHT / 2;

        // Draw line between start and end vertices
        drawLine(startX, startY, endX, endY);
    }
}

/**
 * @brief  Animates a 3D cube on an LCD display.
 * @retval None
 * @example
 *    LCD myLCD;
 *    myLCD.Cube3D();
 *    myLCD.reset_cube();
 *     // printf("fps=%d\n", myLCD.fps);
 */
void LCD::Cube3D()
{
    for (int i = 0; i <= 360 * 1.5; i++) {

        currentMillis = HAL_GetTick(); // Get the current time
        elapsedTime = currentMillis - previousMillis; // C
        fps = 1000 / elapsedTime;

        clear();
        drawCube();
        buffer_Update();
        setCursor(0, 0);
        print_nbr((int)fps);
        rotateCube(PI / 180.0, PI / 360.0);
        HAL_Delay(8);

        previousMillis = currentMillis;
    }
    rotateCube(PI / 4.0, atan(sqrt(2.0)));
}