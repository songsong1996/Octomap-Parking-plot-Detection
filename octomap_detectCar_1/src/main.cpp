//
// Created by songsong on 18-9-21.
//
#include "detect_Objects.h"
int main( int argc, char** argv )
{
    double* bottom_height=new double[2]{-0.7,0};
    double* up_height=new double[2]{-0.5,0.7};
    detect_object detect_ob("../data/58.bt",bottom_height,up_height,2);
    detect_ob.detect_wall_Pillar_car();
}