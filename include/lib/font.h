/************************************************************************
 * lib/font.h
 *
 * Copyright (C) Lisa Milne 2014 <lisa@ltmnet.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *
 ************************************************************************/

/*
 * The font has been found on opengameart.org:
 * http://opengameart.org/content/8x8-ascii-bitmap-font-with-c-source
 */

#ifndef LIB_FONT_H
#define LIB_FONT_H

/*
 * The values in this array are a 8x8 bitmap font for ascii characters
 * As memory is a very precious ressource on a micro-controller all chars
 * before "space" (0x20) have been removed.
 */
#define NB_FONT_TILES 95
extern uint8_t first_font_char;
extern uint64_t font[NB_FONT_TILES];

#endif /* LIB_FONT_H */
