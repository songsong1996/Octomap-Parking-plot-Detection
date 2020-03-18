//
// Created by songsong on 18-9-18.
//

#ifndef OCTOMAP_DETECTOR_DETECTELEM_H
#define OCTOMAP_DETECTOR_DETECTELEM_H



#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <iostream>
#include "line_ransac.h"
#include "octoMap_processing.h"
#include "car_space_norm.h"
using namespace std;
using namespace cv;


void computeCentralPoint(vector<Point>& points,Point2d& central_point)
{
    vector<Point>::iterator itp=points.begin();
    central_point.x=0;central_point.y=0;

    while(itp!=points.end())
    {
        central_point.x+=(double)(*itp).x;
        central_point.y+=(double)(*itp).y;
        itp++;
    }
    central_point.x/=(double)points.size();
    central_point.y/=(double)points.size();
}


class mContour
{
public:
    Vec4i hierarchy;
    vector<Point> point_list;
    Point2d centralPoint;
    int type;  //type=1 wall;type=2 car;type=3 pillar
    int type_id;
    //int object_point_num;
    mContour(){}
    mContour(vector<Point> point_list_tp,Vec4i hierarchy_tp,int type_tp=0)
    {

        point_list=point_list_tp;hierarchy=hierarchy_tp;type=type_tp;
        computeCentralPoint(point_list,centralPoint);
        //object_point_num=0;
    }
    ~mContour(){}
};


class detect_object
{
public:
    detect_object(){}
    ~detect_object(){}
    detect_object(string filename,double* bottom_height,double* up_height,int length)
    {
        map=octoMap_processing(filename);
        resolution=map.getResolution();
        image_list_orig.empty();
        image_list_after_rotate.empty();
        wall_boundingRect.empty();
        pillar_boundingRect.empty();
        car_boundingRect.empty();
//        int length=int(sizeof(*bottom_height)/sizeof(bottom_height[0]));
        for(int i=0;i<length-1;i++)
            for(int j=0;j<length-1;j++)
            {
                if (bottom_height[j]>bottom_height[j+1])
                {
                    int tp=bottom_height[j];
                    bottom_height[j]=bottom_height[j+1];
                    bottom_height[j+1]=tp;
                    int tp2=up_height[j];
                    up_height[j]=up_height[j+1];
                    up_height[j+1]=tp2;
                }
            }
        for(int i=0;i<length;i++)
        {
            Mat tp;
            map.subtract2DMat(tp,bottom_height[i],up_height[i]);
            //cout<<tp<<endl;
            image_list_orig.push_back(tp);
        }
        detect_wall_rotate();
    }


    void detect_wall_rotate()
    {
        wall_angle=0;
        for(int i=0;i<image_list_orig.size();i++)
        {
            //dilate(image_list_orig[i],image_list_orig[i],Mat());
            cv::Mat contours;
            cv::Canny(image_list_orig[i],contours,700,800);
            ransac_line_detection detection(contours,20,100);
            //detection.draw_line();
            wall_angle+=std::atan2(-detection.A,detection.B);
        }
        wall_angle/=(double)image_list_orig.size();
        if(wall_angle>0)rotate_angle=wall_angle-CV_PI/2.0;
        else rotate_angle=wall_angle+CV_PI/2.0;
        image_list_after_rotate.empty();
        vector<Mat>::iterator iti=image_list_orig.begin();
        int i=1;
        while(iti!=image_list_orig.end())
        {
            cv::Mat tp;
            center=Point2f((*iti).cols/2,(*iti).rows/2);
            cv::Mat rot_matrix=cv::getRotationMatrix2D(center,rotate_angle/CV_PI*180.0,1);
            cv::Rect bbox = cv::RotatedRect(center, (*iti).size(), rotate_angle/CV_PI*180.0).boundingRect();
            warpAffine( (*iti), tp, rot_matrix, bbox.size());
            image_list_after_rotate.push_back(tp);
            cv::imwrite( String( "../data/image2/after_rotate_")+std::to_string(i)+String(".jpg"),tp);
            cv::namedWindow("after_rotate",CV_WINDOW_NORMAL);
            cv::imshow("after_rotate",tp);
            cv::waitKey(0);
            iti++;
            i++;
        }
    }





    //检测墙面角度并得到旋转图像，围绕每张图中心点进行旋转
    void detect_wall_Pillar_car()
    {
        vector<vector<mContour>> mcontour_list;
        vector<cv::Mat>::iterator itm=image_list_after_rotate.begin();
        //get all contours
        while(itm!=image_list_after_rotate.end())
        {
            vector<vector<Point>> contours;
            vector<Vec4i> hierarchy;
            findContours((*itm),contours,hierarchy,RETR_LIST,CHAIN_APPROX_NONE);
            vector<mContour> mcontours;
            for(int i=0;i<contours.size();i++)
            {
                Rect bound=boundingRect(contours[i]);
                if(fabs(cv::contourArea(contours[i]))<25);
                else
                {
                    mContour tp = mContour(contours[i], hierarchy[i], 0);
                    mcontours.push_back(tp);
                }
            }
            mcontour_list.push_back(mcontours);
            itm++;
        }
        //detect the wall
        for(int i=0;i<image_list_after_rotate.size();i++)
        {
            cv::Mat contours;
            cv::Canny(image_list_orig[i],contours,700,800);
            ransac_line_detection detection(contours,20,100);
            for(int j=0;j<mcontour_list[i].size();i++)
                if( pointPolygonTest(mcontour_list[i][j].point_list,detection.ptStart,false)>0
                    &   pointPolygonTest(mcontour_list[i][j].point_list,detection.ptEnd,false)>0)
                {
                    mcontour_list[i][j].type=1;
                    mcontour_list[i][j].type_id=0;
                    Rect2d wall_bounding;
                    double min_x,min_y,max_x,max_y;
                    if( detection.ptStart.x>detection.ptEnd.x)
                    {max_x=detection.ptStart.x;min_x=detection.ptEnd.x;}
                    else {max_x=detection.ptEnd.x;min_x=detection.ptStart.x;}
                    if( detection.ptStart.y>detection.ptEnd.y)
                    {max_x=detection.ptStart.y;min_x=detection.ptEnd.y;}
                    else {max_x=detection.ptEnd.y;min_x=detection.ptStart.y;}
                    wall_bounding.tl()=Point2d(min_x,min_y);
                    wall_bounding.br()=Point2d(max_x,max_y);
                    wall_boundingRect.push_back(wall_bounding);
                }
        }
        //detect the pillar //use the upper one to determine which is pillar
        double thres=5;
        find_pillar_match(mcontour_list,thres);
        //detect the car
        int car_id=0;
        for(int j=0;j<mcontour_list[0].size();j++)
        {
            if( mcontour_list[0][j].type_id==0 && mcontour_list[0][j].type!=1 && mcontour_list[0][j].type!=2)//car
            {
                if(contourArea(mcontour_list[0][j].point_list)>(1.5/resolution)*(1.5/resolution))
                {
                    changeMatchID(mcontour_list[0][j].centralPoint,3,car_id,mcontour_list[1]);
                    car_id++;
                    //changeMatchID(mcontour_list[0][j].centralPoint,3,car_id,mcontour_list[2]);
                }
            }
        }
        //
        vector<mContour>::iterator itmm=mcontour_list[0].begin();
        while(itmm!=mcontour_list[0].end())
        {
            if((*itmm).type==2)
                pillar_boundingRect.push_back( boundingRect(Mat((*itmm).point_list)));
            else if ((*itmm).type==3)
                car_boundingRect.push_back(boundingRect(Mat((*itmm).point_list)));
            itmm++;
        }
        rotate_back(wall_boundingRect,wall_boundingRect_after);
        rotate_back(pillar_boundingRect,pillar_boundingRect_after);
        rotate_back(car_boundingRect,car_boundingRect_after);

    }

    //通过检测n组墙面确定障碍物  //目前用于三通道情况
    //detect pillar------step 1-------which one has appears 2 times
    void find_pillar_match(vector<vector<mContour>> &contours_orig,double thres)
    {
        vector<mContour>::iterator itc1=contours_orig[0].begin();
        int num=0;
        int match_id=100+contours_orig[0].size()+contours_orig[1].size()+contours_orig[2].size();
        while(itc1!=contours_orig[0].end())
        {
            vector<mContour>::iterator itc2=contours_orig[1].begin();
            while(itc2!=contours_orig[1].end())
            {
                if(calDist((*itc1).centralPoint,(*itc2).centralPoint)<thres)
                {
                    if((*itc1).type!=1)
                    {
                        (*itc1).type = 4;
                        (*itc2).type = 4;
                        (*itc1).type_id = match_id;
                        (*itc2).type_id = match_id;
                        match_id++;
                    }
//                    Point2d *tp=new Point2d(((*itc1).centralPoint.x+(*itc2).centralPoint.x)*0.5,((*itc1).centralPoint.y+(*itc2).centralPoint.y)*0.5);
//                    (*itc1).centralPoint.x=tp->x;(*itc1).centralPoint.y=tp->y;
//                    (*itc2).centralPoint.x=tp->x;(*itc2).centralPoint.y=tp->y;
                }
                itc2++;
            }
            itc1++;
        }
        //detect pillar------step 2------only if it shows 3 times
        itc1=contours_orig[0].begin();
        int type_pillar=0;
        while(itc1!=contours_orig[0].end())
        {
            vector<mContour>::iterator itc2=contours_orig[2].begin();
            while(itc2!=contours_orig[2].end())
            {
                if(calDist((*itc1).centralPoint,(*itc2).centralPoint)<thres)
                {
                    if((*itc1).type==4)//show 2 times and not the wall
                    {
                        changeMatchID((*itc1).type_id,2,type_pillar,contours_orig[2]);
                        (*itc1).type = 2;
                        (*itc2).type = 2;
                        (*itc1).type_id = type_pillar;
                        (*itc2).type_id = type_pillar;
                        type_pillar++;
                    }
                }
                itc2++;
            }
            itc1++;
        }
    }
    //middle process
    void changeMatchID(int match_id,int type,int type_id,vector<mContour> &contour_list)
    {
        for(int i=0;i<contour_list.size();i++)
        {
            if(contour_list[i].type_id==match_id)
            {
                contour_list[i].type_id = type_id;
                contour_list[i].type = type;
            }
        }
    }

    void changeMatchID(Point2d &central_point,int type,int type_id,vector<mContour> &contour_list)
    {
        for(int i=0;i<contour_list.size();i++)
        {
            if(pointPolygonTest(contour_list[i].point_list,central_point,false)>0)
            {
                contour_list[i].type_id = type_id;
                contour_list[i].type = type;
            }
        }
    }

    void rotate_back(vector<cv::Rect2d> &rect_list,vector<cv::Rect2d> &rect_list_out)
    {
        cv::Point center_point=Point2f(image_list_after_rotate[0].cols/2,image_list_after_rotate[0].rows/2);
        cv::Mat rot_matrix=cv::getRotationMatrix2D(center_point,-rotate_angle,1);
        cv::Rect bbox = cv::RotatedRect(center, image_list_after_rotate[0].size(), rotate_angle).boundingRect();
        double offset_x,offset_y;
        offset_x=(bbox.width-image_list_orig[0].cols)/2.0;
        offset_y=(bbox.height-image_list_orig[0].rows)/2.0;
        rect_list_out.empty();
        for(int i=0;i<rect_list.size();i++)
        {
            vector<cv::Point> contour_tp_in;
            contour_tp_in.push_back(rect_list[i].tl());
            contour_tp_in.push_back(rect_list[i].br());
            vector<cv::Point> contour_tp_out;
            warpAffine(contour_tp_in, contour_tp_out, rot_matrix, bbox.size());
            cv::Rect2d tp;
            tp.tl()=contour_tp_out[0];
            tp.br()=contour_tp_out[1];
            rect_list_out.push_back(tp);
        }
    }


    void compute_car_space()
    {
        vector<cv::Rect2d>::iterator itp=pillar_boundingRect.begin();
        int i=0;
        //compute the nearest objects
        while (itp!=pillar_boundingRect.end()-1)
        {
            double dist_min_p = calDist((*itp),pillar_boundingRect[(i+1)%pillar_boundingRect.size()]);
            int min_id_p = (i+1)%pillar_boundingRect.size();
            for (int j = i+1; j < pillar_boundingRect.size(); j++)
                if (dist_min_p > calDist((*itp), pillar_boundingRect[j]))
                {
                    dist_min_p = calDist((*itp), pillar_boundingRect[j]);
                    min_id_p=j;
                }
            double dist_min_c = calDist((*itp),car_boundingRect[0]);
            int min_id_c = 0;
            for(int j=0;j<car_boundingRect.size();j++)
                if (dist_min_c > calDist((*itp), car_boundingRect[j]))
                {
                    dist_min_c = calDist((*itp), car_boundingRect[j]);
                    min_id_c=j;
                }


            if(dist_min_p<dist_min_c)  //the nearest object is a pillar
            {
                Point2d tp_p1=getCentralPoint((*itp));
                Point2d tp_p2=getCentralPoint(pillar_boundingRect[min_id_p]);
                if(std::abs( getAngle(tp_p1,tp_p2)-CV_PI/2.0)<5.0/180.0*CV_PI)//two pillar are parallel to the wall
                {
                    bool near_the_wall=false;
                    //是否靠近墙
                    if(std::abs(wall_boundingRect[0].tl().x+wall_boundingRect[0].width/2.0-tp_p1.x)<car_norm.width_pass/2.0+car_norm.width_near_the_wall)
                        near_the_wall=true;

                    if (tp_p1.y > tp_p2.y)
                    {
                        double h=(*itp).tl().y-pillar_boundingRect[min_id_p].br().y;
                        int n=floor(h/car_norm.length);
                        for(int i=0;i<n;i++)
                        {
                            cv::Rect2d car_tp;
                            double x;
                            if(near_the_wall)
                            {
                                if (wall_boundingRect[0].tl().x - tp_p1.x > 0)
                                    x = wall_boundingRect[0].tl().x - car_norm.width_near_the_wall / resolution;
                                else
                                    x = wall_boundingRect[0].br().x + car_norm.width_near_the_wall / resolution;
                                car_tp.tl() = Point2d(x, pillar_boundingRect[min_id_p].br().y +
                                                         i * car_norm.length / resolution);
                                car_tp.width=car_norm.width_near_the_wall/resolution;
                            }
                            else
                            {
                                if (wall_boundingRect[0].tl().x - tp_p1.x > 0)
                                    x = wall_boundingRect[0].tl().x - car_norm.width_near_the_wall / resolution-car_norm.width_pass/resolution-car_norm.width_near_the_space/resolution;
                                else
                                    x = wall_boundingRect[0].br().x +car_norm.width_near_the_wall / resolution+car_norm.width_pass/resolution+car_norm.width_near_the_space/resolution;
                                car_tp.tl() = Point2d(x, pillar_boundingRect[min_id_p].br().y +
                                                         i * car_norm.length / resolution);
                                car_tp.width=car_norm.width_near_the_space/resolution;
                            }
                            car_tp.height=car_norm.length/resolution;
                            valid_car_lot.push_back(car_tp);
                        }
                    }
                    else
                    {
                        double h=pillar_boundingRect[min_id_p].br().y-(*itp).tl().y;
                        int n=floor(h/car_norm.length);
                        for(int i=0;i<n;i++)
                        {
                            cv::Rect2d car_tp;
                            double x;
                            if(near_the_wall)
                            {
                                if (wall_boundingRect[0].tl().x - tp_p1.x > 0)
                                    x = wall_boundingRect[0].tl().x - car_norm.width_near_the_wall / resolution;
                                else
                                    x = wall_boundingRect[0].br().x + car_norm.width_near_the_wall / resolution;
                                car_tp.tl() = Point2d(x, (*itp).br().y + i * car_norm.length / resolution);
                                car_tp.width=car_norm.width_near_the_wall/resolution;
                            } else
                            {
                                if (wall_boundingRect[0].tl().x - tp_p1.x > 0)
                                    x = wall_boundingRect[0].tl().x - car_norm.width_near_the_wall / resolution-car_norm.width_pass/resolution-car_norm.width_near_the_space/resolution;
                                else
                                    x = wall_boundingRect[0].br().x +car_norm.width_near_the_wall / resolution+car_norm.width_pass/resolution+car_norm.width_near_the_space/resolution;
                                car_tp.tl() = Point2d(x, (*itp).br().y +i * car_norm.length / resolution);
                                car_tp.width=car_norm.width_near_the_space/resolution;
                            }
                            car_tp.height=car_norm.length/resolution;
                            valid_car_lot.push_back(car_tp);
                        }
                    }

                }

            }
            else  //the nearest object is a car
            {
                Point2d tp_p1=getCentralPoint((*itp));
                Point2d tp_p2=getCentralPoint(car_boundingRect[min_id_c]);
                double angle_tp=std::atan2(tp_p2.y-tp_p1.y,tp_p2.x-tp_p1.x);
                double angle_car=std::atan2(car_norm.length,car_norm.width_near_the_wall);
                if((angle_tp>angle_car & angle_tp<CV_PI-angle_car) || (angle_tp<-angle_car & angle_tp>-CV_PI+angle_car) )//two pillar are parallel to the wall
                {
                    bool near_the_wall=false;
                    //是否靠近墙
                    if(std::abs(wall_boundingRect[0].tl().x+wall_boundingRect[0].width/2.0-tp_p1.x)<car_norm.width_pass/2.0+car_norm.width_near_the_wall)
                        near_the_wall=true;

                    if (tp_p1.y > tp_p2.y)
                    {
                        double h=(*itp).tl().y-pillar_boundingRect[min_id_p].br().y;
                        int n=floor(h/car_norm.length);
                        for(int i=0;i<n;i++)
                        {
                            cv::Rect2d car_tp;
                            double x;
                            if(near_the_wall)
                            {
                                if (wall_boundingRect[0].tl().x - tp_p1.x > 0)
                                    x = wall_boundingRect[0].tl().x - car_norm.width_near_the_wall / resolution;
                                else
                                    x = wall_boundingRect[0].br().x + car_norm.width_near_the_wall / resolution;
                                car_tp.tl() = Point2d(x, pillar_boundingRect[min_id_p].br().y +
                                                         i * car_norm.length / resolution);
                                car_tp.width=car_norm.width_near_the_wall/resolution;
                            }
                            else
                            {
                                if (wall_boundingRect[0].tl().x - tp_p1.x > 0)
                                    x = wall_boundingRect[0].tl().x - car_norm.width_near_the_wall / resolution-car_norm.width_pass/resolution-car_norm.width_near_the_space/resolution;
                                else
                                    x = wall_boundingRect[0].br().x +car_norm.width_near_the_wall / resolution+car_norm.width_pass/resolution+car_norm.width_near_the_space/resolution;
                                car_tp.tl() = Point2d(x, pillar_boundingRect[min_id_p].br().y +
                                                         i * car_norm.length / resolution);
                                car_tp.width=car_norm.width_near_the_space/resolution;
                            }
                            car_tp.height=car_norm.length/resolution;
                            valid_car_lot.push_back(car_tp);
                        }
                    }
                    else
                    {
                        double h=pillar_boundingRect[min_id_p].br().y-(*itp).tl().y;
                        int n=floor(h/car_norm.length);
                        for(int i=0;i<n;i++)
                        {
                            cv::Rect2d car_tp;
                            double x;
                            if(near_the_wall)
                            {
                                if (wall_boundingRect[0].tl().x - tp_p1.x > 0)
                                    x = wall_boundingRect[0].tl().x - car_norm.width_near_the_wall / resolution;
                                else
                                    x = wall_boundingRect[0].br().x + car_norm.width_near_the_wall / resolution;
                                car_tp.tl() = Point2d(x, (*itp).br().y + i * car_norm.length / resolution);
                                car_tp.width=car_norm.width_near_the_wall/resolution;
                            } else
                            {
                                if (wall_boundingRect[0].tl().x - tp_p1.x > 0)
                                    x = wall_boundingRect[0].tl().x - car_norm.width_near_the_wall / resolution-car_norm.width_pass/resolution-car_norm.width_near_the_space/resolution;
                                else
                                    x = wall_boundingRect[0].br().x +car_norm.width_near_the_wall / resolution+car_norm.width_pass/resolution+car_norm.width_near_the_space/resolution;
                                car_tp.tl() = Point2d(x, (*itp).br().y +i * car_norm.length / resolution);
                                car_tp.width=car_norm.width_near_the_space/resolution;
                            }
                            car_tp.height=car_norm.length/resolution;
                            valid_car_lot.push_back(car_tp);
                        }
                    }
                }
            }
            i++;
            itp++;
        }
    }
    //计算两点间的距离
    double calDist(Point2d p1,Point2d p2)
    { return sqrt( pow(p1.x-p2.x,2.0)+pow(p1.y-p2.y,2.0)); }

    double calDist(Rect2d r1,Rect2d r2)
    {
        Point2d p1=Point2d(r1.tl().x+r1.width/2.0,r1.tl().y+r1.width/2.0);
        Point2d p2=Point2d(r2.tl().x+r2.width/2.0,r2.tl().y+r2.width/2.0);
        return calDist(p1,p2);
    }

    Point2d getCentralPoint(Rect2d &r)
    { return Point2d(r.tl().x+r.width/2.0,r.tl().y+r.height/2.0); }

    double getAngle(Point2d &p1,Point2d &p2)
    { return std::atan2((p1.y-p2.y),(p1.x-p2.x)); }
private:

    octoMap_processing map=octoMap_processing();
    double resolution;
    vector<Mat> image_list_orig;
    vector<Mat> image_list_after_rotate;
    vector<Rect2d> car_boundingRect;
    vector<Rect2d> wall_boundingRect;
    vector<Rect2d> pillar_boundingRect;
    vector<Rect2d> car_boundingRect_after;
    vector<Rect2d> wall_boundingRect_after;
    vector<Rect2d> pillar_boundingRect_after;
    vector<Rect2d> valid_car_lot;

    double wall_angle;
    Point2f center;
    double rotate_angle;
    double detect_threshhold;
    car_space_norm car_norm;
    int parking_type;//决定了如何停车

};

#endif //OCTOMAP_DETECTOR_DETECTELEM_H
