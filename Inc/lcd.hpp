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

#ifndef _LCDCPP_H
#define _LCDCPP_H

#include "font.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"

#define LCD_SETYADDR 0x40
#define LCD_SETXADDR 0x80
#define LCD_DISPLAY_BLANK 0x08
#define LCD_DISPLAY_NORMAL 0x0C
#define LCD_DISPLAY_ALL_ON 0x09
#define LCD_DISPLAY_INVERTED 0x0D

#define LCD_WIDTH 84
#define LCD_HEIGHT 48
#define LCD_SIZE LCD_WIDTH* LCD_HEIGHT / 8
#define PI 3.14159265359

class LCD {
public:
    LCD(SPI_HandleTypeDef* hspi, GPIO_TypeDef* rstPort, uint16_t rstPin, GPIO_TypeDef* dcPort, uint16_t dcPin);

    void init();
    void clear();
    void setCursor(uint8_t row, uint8_t col);
    void print(const char* str);
    void print_nbr(uint32_t num);
    void drawBitmap(const uint8_t* bitmap, uint8_t width, uint8_t height);
    void drawLine(int x0, int y0, int x1, int y1);

    void aria_Update(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
    void buffer_Update();

    void reset_cube();
    void Cube3D();

    void drawRectangle(int x, int y, int width, int height);
    void drawRectangleFilled(int x, int y, int width, int height);

    int fps = 0;

private:
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* rstPort;
    uint16_t rstPin;
    GPIO_TypeDef* dcPort;
    uint16_t dcPin;

    // HAL ticks
    uint32_t previousMillis = HAL_GetTick(); // Variable to store the previous time
    uint32_t currentMillis = HAL_GetTick(); // Get the current time
    uint32_t elapsedTime = currentMillis - previousMillis; // C

    uint8_t buffer[504] = { 0 }; // Define the buffer for LCD data

    // Cube vertices (x, y, z)
    double cubeVertices[8][3] = { { -1, -1, -1 }, { -1, -1, 1 }, { -1, 1, -1 }, { -1, 1, 1 }, { 1, -1, -1 }, { 1, -1, 1 }, { 1, 1, -1 }, { 1, 1, 1 } };

    // Cube edges (start vertex, end vertex)
    int cubeEdges[12][2] = { { 0, 1 }, { 1, 3 }, { 3, 2 }, { 2, 0 }, { 4, 5 }, { 5, 7 }, { 7, 6 }, { 6, 4 }, { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 } };

    void sendCommand(uint8_t command);

    void rotateCube(double angleX, double angleY);
    void setPixel(int x, int y);
    void sendData(uint8_t data);
    void drawCube();
};

#endif
