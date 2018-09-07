/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  Cristian Rodriguez <u5419700@anu.edu.au>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef TAPE_H
#define TAPE_H

#include <ostream>
static const uint BLUE   = 1;
static const uint RED    = 2;
static const uint GREEN  = 3;
static const uint YELLOW = 4;


struct Tape
{
    double Cx;
    double Cy;
    double Zw;
    uint Color;
};

std::ostream& operator<<(std::ostream& os, const Tape& tape) {
    return os << "(" << tape.Cx << "," << tape.Cy << "," << tape.Zw  << "," << tape.Color << ")" ;
}

bool sortTapes(Tape x, Tape y)
{
    return (x.Cy < y.Cy);
}
#endif // TAPE_H
