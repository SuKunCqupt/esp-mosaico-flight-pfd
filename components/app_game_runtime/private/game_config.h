// SPDX-License-Identifier: Apache-2.0
#pragma once

#define GAME_NAME "flight_pfd"
/* Viewport in pixels. The tick rate is also the render target: rendering
 * faster than the panel (~40 fps full-frame over QSPI) keeps the newest
 * attitude in every presented frame; older pending frames are superseded. */
enum { GAME_WIDTH = 480, GAME_HEIGHT = 480, GAME_TICK_HZ = 60 };
enum { GAME_ACTION_PAUSE = 3, GAME_ACTION_RESET = 4 };
