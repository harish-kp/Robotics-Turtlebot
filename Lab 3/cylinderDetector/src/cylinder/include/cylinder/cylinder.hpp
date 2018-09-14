/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  Cristian Rodriguez <email>
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

#ifndef CYLINDER_H
#define CYLINDER_H

#include <opencv2/core/core.hpp>
#include <vector>
#include <map>
#include "tape.hpp"

struct colors
{
    uint first;
    uint second;
    uint third;
    uint fourth;
};

inline bool operator<(colors const& left, colors const& right) {
  if (left.first < right.first) { return true; }
  if (left.first > right.first) { return false; }
  if (left.second < right.second) { return true; }
  if (left.second > right.second) { return false; }
  if (left.third < right.third) { return true; }
  if (left.third > right.third) { return false; }
  return left.fourth < right.fourth;
}

static const colors c1 = {RED,BLUE,GREEN,YELLOW};
static const colors c2 = {YELLOW,BLUE,RED,GREEN};
static const colors c3 = {RED,GREEN,BLUE,YELLOW};
static const colors c4 = {BLUE,RED,YELLOW,GREEN};
static const colors c5 = {RED,YELLOW,BLUE,GREEN};
static const colors c6 = {GREEN,RED,BLUE,YELLOW};
static const colors c7 = {YELLOW,BLUE,GREEN,RED};
static const colors c8 = {BLUE,YELLOW,GREEN,RED};
static const colors c9 = {GREEN,BLUE,YELLOW,RED};

static std::map<colors,int> create_map()
{
    std::map<colors,int> m;
    m[c1] = 1;
    m[c2] = 2;
    m[c3] = 3;
    m[c4] = 4;
    m[c5] = 5;
    m[c6] = 6;
    m[c7] = 7;
    m[c8] = 8;
    m[c9] = 9;
    return m;
}

static const std::map<colors,int> labelMap =  create_map();



class Cylinder
{
private:
    std::vector<Tape>* tapes;
    int Cx;
    int Cy;
    
    double Xw;
    double Zw;    
    float f;
    float invF;
    uint label;
    
public:
    Cylinder();
    ~Cylinder();
    bool addTape(Tape tape_);
    std::vector<Tape> getTape();
    int getCx(){return Cx;}
    int getCy(){return Cy;}
    double getZw(){return Zw;}
    double getXw()
    {    
        this->Xw = (double)((Zw * (Cx-320) * invF));
        return Xw;        
    }
    uint getLabel();
    cv::Point2d getWorld();
};

#endif // CYLINDER_H
