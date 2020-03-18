//
// Created by songsong on 18-9-21.
//

#ifndef OCTOMAP_DETECTOR_CAR_SPACE_NORM_H
#define OCTOMAP_DETECTOR_CAR_SPACE_NORM_H


class car_space_norm
{
public:

    car_space_norm(int type_tp,double width1,double width2,double length_tp,double width_pass_tp)
    {
        type=type_tp;
        width_near_the_space=width1;
        width_near_the_wall=width2;
        length=length_tp;
        width_pass=width_pass_tp;
    }
    car_space_norm(int type)
    {
        if(type==1)
        {
            width_near_the_wall = 2.4;
            width_near_the_space = 2.1;
            length=6.0;
            width_pass=3.8;
        }
        if(type==2)
        {
            width_near_the_wall =5.3;
            width_near_the_space = 5.1;
            length=2.4;
            width_pass=9.0;
        }
        if(type==3)
        {
            width_near_the_wall =5.3;
            width_near_the_space = 5.1;
            length=2.4;
            width_pass=5.5;
        }
    }
    car_space_norm(){}
    ~car_space_norm(){}

    int type;
    //1----parallel  //2------vertical----forward //3-----vertical----backward
    double width_near_the_wall;
    double width_near_the_space;
    double length;
    double width_pass;

};



#endif //OCTOMAP_DETECTOR_CAR_SPACE_NORM_H
