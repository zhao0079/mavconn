/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

This file is part of the PIXHAWK project

    PIXHAWK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PIXHAWK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 *  @file
 *  @brief  defines dimention of captured image by camera
 *  @author Benjamin Knecht <bknecht@student.ethz.ch>
 */

/*
 * camera spezifications
 */

#define CAPTURED_FRAME_WIDTH 640 // ((65536 - 4 - 4) / 3) // 752 // 100
#define CAPTURED_FRAME_HEIGHT 480 // 1 // 480 // 64
#define CAPTURED_FRAME_N_COLOR_CHANNELS 1

#define IMG_BYTE_SIZE CAPTURED_FRAME_WIDTH * CAPTURED_FRAME_HEIGHT * CAPTURED_FRAME_N_COLOR_CHANNELS
