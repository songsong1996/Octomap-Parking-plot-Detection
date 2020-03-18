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
#include <boost/timer.hpp> //boost
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
    mContour(vector<Point> point_list_tp,const Vec4i &hierarchy_tp,int type_tp=0)
    {
        point_list=point_list_tp;hierarchy=hierarchy_tp;type=type_tp;
        computeCentralPoint(point_list,centralPoint);
        //object_point_num=0;
    }
    ~mContour(){}
};

class mRotatedRect
{
public:
    mRotatedRect(){}
    ~mRotatedRect(){}
    mRotatedRect(Point2d* pt,double width_tp,double height_tp)
    {
        point_list=pt;width=width_tp;height=height_tp;
        Point2d tp_c(0,0);
        for(int i=0;i<4;i++)
        {
            tp_c.x+=point_list[i].x;
            tp_c.y+=point_list[i].y;
        }
        tp_c.x/=4.0;
        tp_c.y/=4.0;
        centralPoint=tp_c;
    }

    Point2d *point_list=new Point2d[4];
    Point2d centralPoint;
    double width;
    double height;
};


class detect_object
{
public:
    detect_object(){}
    ~detect_object(){}
    detect_object(string &filepath_high,string & filepath_low)
    {
        resolution=0.1;
        image_list_orig.empty();
        image_list_after_rotate.empty();
        wall_boundingRect.empty();
        pillar_boundingRect.empty();
        car_boundingRect.empty();
        Mat tp=imread(filepath_high,-1);
        // cout<<tp.type();
//        imshow("tp",tp);
//        waitKey(0);

        image_list_orig.emplace_back(tp);
        tp=imread(filepath_low,-1);
        image_list_orig.emplace_back(tp);

    }
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
                    double tp=bottom_height[j];
                    bottom_height[j]=bottom_height[j+1];
                    bottom_height[j+1]=tp;
                    double tp2=up_height[j];
                    up_height[j]=up_height[j+1];
                    up_height[j+1]=tp2;
                }
            }
        for(int i=0;i<length;i++)
        {
            Mat tp;
            map.subtract2DMat(tp,bottom_height[i],up_height[i]);
            image_list_orig.emplace_back(tp);
        }
    }


    void detect_wall_rotate()
    {
//        cv::namedWindow("tp",WINDOW_NORMAL);
        wall_angle=0;
        cv::Mat contours;
        cv::Canny(image_list_orig[0], contours, 700, 800);
//        cv::imshow("tp1",contours);
//        cv::waitKey(0);
        ransac_line_detection detection(contours, 10, 100);
        //detection.draw_line();
        wall_angle=std::atan2(-detection.A, detection.B);
        // if(wall_angle<0)wall_angle=wall_angle+CV_PI;
//        for(const Mat &im:image_list_orig)
//        {
//            cv::Mat contours;
//            cv::Canny(im, contours, 700, 800);
//            ransac_line_detection detection(contours, 10, 100);
//            //detection.draw_line();
//            double tp_angle=std::atan2(-detection.A, detection.B);
//            tp_angle=tp_angle<0?tp_angle+CV_PI:tp_angle;
//            wall_angle += tp_angle;
//        }
//        wall_angle/=(double)image_list_orig.size();
        if(wall_angle>0)rotate_angle=wall_angle-CV_PI/2.0;
        else rotate_angle=wall_angle+CV_PI/2.0;

        rotate_angle=0;


        auto iti=image_list_orig.begin();
        int i=1;
        while(iti!=image_list_orig.end())
        {
            cv::Mat tp;
            center=Point2f((float)((*iti).cols/2.0),(float)((*iti).rows/2.0));
            cv::Mat rot_matrix=cv::getRotationMatrix2D(center,rotate_angle/CV_PI*180.0,1);
            cv::Rect bbox = cv::RotatedRect(center, (*iti).size(), (float)(rotate_angle/CV_PI*180.0)).boundingRect();
            warpAffine( (*iti), tp, rot_matrix, bbox.size());
            cv::dilate(tp,tp,Mat());
            cv::erode(tp,tp,Mat());
            for(int i=0;i<tp.rows;i++)
                for(int j=0;j<tp.cols;j++)
                    if(tp.at<uchar>(i,j)>0)
                        tp.at<uchar>(i,j)=255;
            image_list_after_rotate.emplace_back(tp);
//
//            cv::imshow("tp1",tp);
//            cv::waitKey(0);

            iti++;
            i++;
        }

//        cv::imshow("tp",image_list_after_rotate[0]);
//        cv::waitKey(0);
//        cv::imshow("tp",image_list_after_rotate[1]);
//        cv::waitKey(0);
    }

    //检测墙面角度并得到旋转图像，围绕每张图中心点进行旋转
    void detect_wall_Pillar_car()
    {
        vector<Vec4i> lines;
        //cv::HoughLinesP(image_list_after_rotate[1],lines,1,CV_PI/180.0,40,40,3);
        cv::HoughLinesP(image_list_after_rotate[0],lines,1,CV_PI/180.0,35,35,5);

//        cv::imshow("tp",image_list_after_rotate[0]);
//        cv::waitKey(0);
//        cv::imshow("tp",image_list_after_rotate[1]);
//        cv::waitKey(0);

        is_walls_exist=false;
        if(lines.size()>0)is_walls_exist=true;
        vector<Rect2d>wall_bounding_tp;
        for(Vec4i l:lines)
        {
            double angle_lines=std::atan2( l[1]-l[3],l[0]-l[2]);
            Rect2d wall_bounding;
            double min_x,min_y,max_x,max_y;
            min_x=l[0]<l[2]?l[0]:l[2];
            min_y=l[1]<l[3]?l[1]:l[3];
            max_x=l[0]>l[2]?l[0]:l[2];
            max_y=l[1]>l[3]?l[1]:l[3];

            wall_bounding.x=min_x;wall_bounding.y=min_y;
            wall_bounding.width=(max_x-min_x);
            wall_bounding.height=(max_y-min_y);
            wall_bounding_tp.emplace_back(wall_bounding);
        }


//        //draw
//        cv::namedWindow("tp1",WINDOW_NORMAL);
//        Mat tp1;
//        cv::cvtColor(image_list_after_rotate[0], tp1, CV_GRAY2BGR);
//        for(Rect2d wall_tp:wall_bounding_tp)
//        {
//            cv::rectangle(tp1,wall_tp,cv::Scalar(255, 0, 255));
//        }
//        cv::imshow("tp1",tp1);
//        cv::waitKey(0);

        //merge
        merge_rect(wall_bounding_tp,wall_boundingRect);

//        //draw
//        cv::namedWindow("tp",WINDOW_NORMAL);
//        Mat tp;
//        cv::cvtColor(image_list_after_rotate[1], tp, CV_GRAY2BGR);
//        for(Rect2d wall_tp:wall_boundingRect)
//        {
//            cv::rectangle(tp,wall_tp,cv::Scalar(255, 255, 0));
//        }
//        cv::imshow("tp",tp);
//        cv::waitKey(0);





        vector<vector<mContour>> mcontour_list;
        auto itm=image_list_after_rotate.begin();
        //get all contours

        while(itm!=image_list_after_rotate.end())
        {
//            cv::namedWindow("tp",WINDOW_NORMAL);
//            Mat tp1;
//            cv::cvtColor(*itm, tp1, CV_GRAY2BGR);
            vector<vector<Point>> contours_tp;
            vector<vector<Point>> contours;
            vector<Vec4i> hierarchy;
            findContours((*itm),contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);

            vector<mContour> mcontours;
            for(int i=0;i<contours.size();i++)
            {
                Rect bound=boundingRect(contours[i]);


                if  (contourArea(contours[i])>5 || bound.height*bound.height>9)
                {
                    //cv::rectangle(tp1,bound,cv::Scalar(255, 255, 0));
                    mContour tp = mContour(contours[i], hierarchy[i], 0);
                    mcontours.emplace_back(tp);
                }
            }
//        cv::imshow("tp",tp1);
//        cv::waitKey(0);
            mcontour_list.emplace_back(mcontours);
            itm++;
        }



        if(lines.size()>0)
            find_wall_match( mcontour_list );

        //detect the pillar //use the upper one to determine which is pillar
        find_pillar_match(mcontour_list);

        //detect the car
        int car_id=0;
        for(int j=0;j<mcontour_list[1].size();j++)
        {
            if(  mcontour_list[1][j].type!=1 && mcontour_list[1][j].type!=2)//car
            {
                Rect bound=boundingRect(Mat(mcontour_list[1][j].point_list));
                if(contourArea(mcontour_list[1][j].point_list)>(0.5/resolution)*(0.5/resolution))
                {
                    mcontour_list[1][j].type=3;
                    mcontour_list[1][j].type_id=car_id;
                    car_id++;
                }
                if(bound.width>1.2/resolution || bound.height>1.2/resolution)
                    //if( calRectAngle(bound)<50.0/180.0*CV_PI && (bound.width>0.8/resolution || bound.height>0.8/resolution)  )
                {
                    mcontour_list[1][j].type=3;
                    mcontour_list[1][j].type_id=car_id;
                    car_id++;
                }
            }
        }

        auto itmm1=mcontour_list[0].begin();
        while(itmm1!=mcontour_list[0].end())
        {
            if((*itmm1).type==2)
                pillar_boundingRect.emplace_back( boundingRect(Mat((*itmm1).point_list)));
            itmm1++;
        }


        auto itmm=mcontour_list[1].begin();
        while(itmm!=mcontour_list[1].end())
        {
            if ((*itmm).type==3)
            {
                bool has_pillar=false;
                Rect2d re_tp = boundingRect(Mat((*itmm).point_list));  //the border of the car
                //check if the car rect contains any pillars
                for (int i = 0; i < pillar_boundingRect.size(); i++)
                {
                    Point center_tp =Point(pillar_boundingRect[i].x+pillar_boundingRect[i].width/2.0,pillar_boundingRect[i].y+pillar_boundingRect[i].height/2.0);
                    if (center_tp.x <= re_tp.br().x && center_tp.x >= re_tp.tl().x &&
                        center_tp.y <= re_tp.br().y && center_tp.y >= re_tp.tl().y)
                    {
                        has_pillar=true;
                        vector<Rect2d> output_list;
                        divide_rect(re_tp,pillar_boundingRect[i],output_list);

//                        Mat dstImage;
//                        cv::cvtColor(image_list_after_rotate[1],dstImage,COLOR_GRAY2BGR);
//                        cv::rectangle( dstImage, pillar_boundingRect[i], Scalar(255,0,255), 1, CV_AA);
//                        for(Rect r: output_list )
//                        { cv::rectangle( dstImage, r, Scalar(0,0,255), 1, CV_AA); }
//                        imshow("tp",dstImage);
//                        waitKey(0);

                        for(Rect2d ri:output_list)
                            car_boundingRect.emplace_back(ri);
                        break;
                    }
                }
                if(!has_pillar)
                    car_boundingRect.emplace_back(re_tp);
            }
            itmm++;
        }

        combine_rect_list(car_boundingRect,pillar_boundingRect);

        //draw(1);
        rotate_back(wall_boundingRect,wall_boundingRect_rotateback);
        rotate_back(pillar_boundingRect,pillar_boundingRect_rotateback);
        rotate_back(car_boundingRect,car_boundingRect_rotateback);
    }

    void combine_rect_list(vector<Rect2d> &list1,vector<Rect2d> &list2)
    {
        vector<int> delete_list;
        for (Rect2d rc:list1)
        {
            for (int i=0;i<list2.size();i++)
            {
                if(list2[i].x+list2[i].width/2.0<=rc.br().x && list2[i].x+list2[i].width/2.0>=rc.x
                   && list2[i].y+list2[i].height/2.0<=rc.br().y && list2[i].y+list2[i].height/2.0>=rc.y)
                {
                    delete_list.emplace_back(i);
                }
            }
        }
        int n=0;
        while(delete_list.size()!=0)
        {
            list2.erase(list2.begin()+delete_list[0]-n);
            delete_list.erase(delete_list.begin());
            n+=1;
        }
    }

    void divide_rect(Rect2d &r1,Rect2d &r2,vector<Rect2d> &output_list)
    {
        vector<Point> r1_list;
        Rect2List(r1,r1_list);

        if(r2.tl().y>r1.tl().y)
        {
            if(r2.br().y<r1.br().y)
            {
                double d1 = r2.tl().y - r1.tl().y, d2 = r1.br().y - r2.br().y;
                Rect2d tp1(r1.tl().x, r1.tl().y, r1.width, d1);
                Rect2d tp2(r1.tl().x, r2.br().y, r1.width, d2);
                if (d1 < 0.2 / resolution);
                else output_list.emplace_back(tp1);
                if (d2 < 0.2 / resolution);
                else output_list.emplace_back(tp2);
            }
            else
            {
                double d1 = r2.tl().y - r1.tl().y;
                Rect2d tp1(r1.tl().x, r1.tl().y, r1.width, d1);
                if (d1 < 0.2 / resolution);
                else output_list.emplace_back(tp1);
            }
        }
        else
        {
            if(r2.br().y<r1.br().y)
            {
                double d2 = r1.br().y - r2.br().y;
                Rect2d tp2(r1.tl().x, r2.br().y, r1.width, d2);
                if (d2 < 0.2 / resolution);
                else output_list.emplace_back(tp2);
            }
        }
    }

    void find_wall_match(vector<vector<mContour>>& mcontour_list )
    {
        for(int i=0;i<mcontour_list.size();i++)
        {
            for(int j=0;j<mcontour_list[i].size();j++)
            {
                Rect bound=boundingRect(Mat(mcontour_list[i][j].point_list));
                //if(bound.width>1.0/resolution || bound.height>1.0/resolution)
                for(Rect2d wall:wall_boundingRect)
                {
                    Point2d central_point_wall=Point2d(wall.x+wall.width/2.0,wall.y+wall.height/2.0);
                    if( central_point_wall.x<=bound.x+bound.width &&  central_point_wall.x>=bound.x
                        &&  central_point_wall.y<=bound.y+bound.height &&  central_point_wall.y>=bound.y )  //wall is within mcontour
                    {
                        mcontour_list[i][j].type=1;
                        //break;
                    }
                    if(calRectAngle(wall)<60.0/180.0*CV_PI) //HORIZONTAL
                    {
                        if(std::abs(bound.y+bound.height/2.0-central_point_wall.y)<0.5/resolution)
                        {mcontour_list[i][j].type=1;}
                    }
                    else  //VERTICLE
                    {
                        if(std::abs(bound.x+bound.width/2.0-central_point_wall.x)<0.5/resolution)
                        {mcontour_list[i][j].type=1; }
                    }
                }
            }
        }
        //draw(1);
    }
    //通过检测n组墙面确定障碍物  //双通道 检测两张图都存在的
    //detect pillar------step 1-------which one has appears 2 times
    void find_pillar_match(vector<vector<mContour>> &contours_orig)
    {
        auto itc1=contours_orig[0].begin();
        int num=0;
        int pillar_id=0;
        while(itc1!=contours_orig[0].end()) //HIGH
        {
            Rect2d pillar_border = boundingRect(Mat((*itc1).point_list));
            if ((*itc1).type != 1)
            {
                Rect2d pillar_border = boundingRect(Mat((*itc1).point_list));
                double sum =  contourArea((*itc1).point_list);

                double n = 0;
                for (int i = (int) pillar_border.x; i <= (int) pillar_border.x + pillar_border.width; i++) {
                    for (int j = (int) pillar_border.y; j <= (int) pillar_border.y + pillar_border.height; j++) {
                        // cout<<(int)image_list_after_rotate[1].at<uchar>(j,i)<<endl ;
                        if ((int)image_list_after_rotate[1].at<uchar>(j,i) > 0 )
                            n += 1;
                    }
                }
                if (n / sum > 0.8)  //exists in 2 image
                {
                    auto itc2 = contours_orig[1].begin();
                    while (itc2 != contours_orig[1].end())
                    {
                        if((*itc2).type!=1) {
                            Rect r2 = boundingRect(Mat((*itc2).point_list));

                            if( calRectAngle(pillar_border)<20.0/180.0*CV_PI || calRectAngle(pillar_border)>70.0/180.0*CV_PI)
                                (*itc1).type = 1;
                            else
                            {
                                if(pillar_border.width>0.3/resolution && pillar_border.height>0.3/resolution)
                                    (*itc1).type = 2;
                            }
                        }
                        itc2++;
                    }
                }
            }
            itc1++;
        }
    }

    void rotate_back(vector<cv::Rect2d> &rect_list,vector<mRotatedRect> &rect_list_out)
    {
        Mat rotate_mat=Mat(2,2,CV_64FC1);

        rotate_mat.at<double>(0,0)=std::cos(-rotate_angle);
        rotate_mat.at<double>(0,1)=std::sin(-rotate_angle);
        rotate_mat.at<double>(1,0)=-std::sin(-rotate_angle);
        rotate_mat.at<double>(1,1)=std::cos(-rotate_angle);
        Mat offset_mat=Mat(2,1,CV_64FC1);
        offset_mat.at<double>(0,0)=-image_list_after_rotate[0].cols/2.0;
        offset_mat.at<double>(1,0)=-image_list_after_rotate[0].rows/2.0;
        double offset_x,offset_y;
        offset_x=image_list_orig[0].cols/2.0;
        offset_y=image_list_orig[0].rows/2.0;
        offset_x=image_list_after_rotate[0].cols/2.0;
        offset_y=image_list_after_rotate[0].rows/2.0;
        rect_list_out.empty();
        for(cv::Rect2d ri:rect_list)
        {
            Point2d* tp_list=new Point2d[4];
            tp_list[0]=mrotatePoint(ri.tl(),rotate_mat,offset_mat,offset_x,offset_y);
            tp_list[1]=mrotatePoint(Point(ri.x+ri.width,ri.y),rotate_mat,offset_mat,offset_x,offset_y);
            tp_list[2]=mrotatePoint(ri.br(),rotate_mat,offset_mat,offset_x,offset_y);
            tp_list[3]=mrotatePoint(Point(ri.x,ri.y+ri.height),rotate_mat,offset_mat,offset_x,offset_y);
            mRotatedRect tp(tp_list,ri.width,ri.height);
            rect_list_out.emplace_back(tp);
        }
    }

    Point2d mrotatePoint(const Point2d &tp,Mat &rotate_mat,Mat &move_mat,double offset_x=0,double offset_y=0)
    {
        Mat point_orig=Mat(2,1,CV_64FC1);
        point_orig.at<double>(0,0)=tp.x;
        point_orig.at<double>(1,0)=tp.y;
        //   cout<<point_orig<<endl;
        Mat tppp=point_orig+move_mat;
        //   cout<<tppp<<endl;
        Mat output_mat=rotate_mat*tppp;
        // cout<<output_mat<<endl;
        Point2d output;
        output.x=output_mat.at<double>(0,0)+offset_x;
        output.y=output_mat.at<double>(1,0)+offset_y;
        return output;
    }

    void compute_car_space()
    {

        if(wall_boundingRect.size()>0)
        {
            int num_verticle=0;
            Rect2d wall_verticle;
            for (Rect2d wall:wall_boundingRect)
                if (std::abs(calRectAngle(wall)) > 80.0 / 180.0 * CV_PI)//verticle
                {
                    num_verticle++;
                    wall_verticle=wall;
                    if(wall.x>image_list_after_rotate[0].cols*0.25 && wall.x<image_list_after_rotate[0].cols*0.75)
                        range_no_car_y.emplace_back(wall);
                }
            if(num_verticle>0)
            {
                double car_wall_min_dist = std::abs(image_list_after_rotate[0].rows*1.0);
//                int verticle_wall_num=0;
//                for (Rect2d wall:wall_boundingRect)
//                    if (std::abs(calRectAngle(wall)) > 80.0 / 180.0 * CV_PI)
//                        verticle_wall_num++;
                for (Rect2d car:car_boundingRect)
                    for (Rect2d wall:wall_boundingRect)
                        if (std::abs(calRectAngle(wall)) > 80.0 / 180.0 * CV_PI)//verticle
                        {
                            if(num_verticle>1)
                            {
                                if((car.x>image_list_after_rotate[0].cols/2.0 && wall.x>car.x)  ||
                                   (car.x<image_list_after_rotate[0].cols/2.0 && wall.x<car.x))
                                    if (std::abs(wall.x+wall.width/2.0 - car.x-car.width/2.0) <= car_wall_min_dist)
                                    {
                                        car_wall_min_dist = std::abs(wall.x+wall.width/2.0 - car.x-car.width/2.0);
                                    }
                            }
                            else
                            {
                                if((car.x>image_list_after_rotate[0].cols/2.0 && wall.x>car.x)  ||
                                   (car.x<image_list_after_rotate[0].cols/2.0 && wall.x<car.x))
                                {
                                    car_wall_min_dist = std::abs(wall.x+wall.width/2.0 - car.x-car.width/2.0);
                                }
                                else
                                    car_norm = car_space_norm(2);
                            }
                        }

                if (car_wall_min_dist > 1.5 / resolution)
                    car_norm = car_space_norm(2);
                else
                    car_norm = car_space_norm(1);
            }
            else
            {
                car_norm = car_space_norm(2);
//                double pillar_wall_min_dist = std::abs(wall_verticle.x - pillar_boundingRect[0].x);
//                for (Rect2d pillar:pillar_boundingRect)
//                    for (Rect2d wall:wall_boundingRect)
//                        if (std::abs(calRectAngle(wall)) > 80.0 / 180.0 * CV_PI)
//                            if (std::abs(wall.x+wall.width/2.0 -pillar.x-pillar.width/2.0) <= pillar_wall_min_dist)
//                                pillar_wall_min_dist = std::abs(wall.x+wall.width/2.0 -pillar.x-pillar.width/2.0);
//
//                if (pillar_wall_min_dist > 2.5 / resolution)
//                    car_norm = car_space_norm(2);
//                else
//                    car_norm = car_space_norm(1);
            }

        }
        else  car_norm = car_space_norm(2);
        //compute the nearest object
        obstacles_list.clear();
        for(int i=0;i<car_boundingRect.size();i++)
        {
            if(car_boundingRect[i].x+car_boundingRect[i].width/2.0>image_list_after_rotate[0].cols*0.1
               && car_boundingRect[i].x+car_boundingRect[i].width/2.0<image_list_after_rotate[0].cols*0.9)
                obstacles_list.emplace_back(car_boundingRect[i]);
        }

        for(int i=0;i<pillar_boundingRect.size();i++)
        {
            if(pillar_boundingRect[i].x+pillar_boundingRect[i].width/2.0>image_list_after_rotate[0].cols*0.1
               && pillar_boundingRect[i].x+pillar_boundingRect[i].width/2.0<image_list_after_rotate[0].cols*0.9)
                obstacles_list.emplace_back(pillar_boundingRect[i]);
        }


        double dist_max=0,max_x=0,min_x=image_list_after_rotate[0].cols;
        Rect2d min_x_r,max_x_r;
        for(Rect2d r1:obstacles_list)
        {
            if(r1.x+r1.width/2.0>max_x){max_x=r1.x+r1.width/2.0;max_x_r=r1;}
            if(r1.x+r1.width/2.0<min_x){min_x=r1.x+r1.width/2.0;min_x_r=r1;}
            dist_max=max_x_r.x+max_x_r.width/2.0-min_x_r.x-min_x_r.width/2.0;
//            for(Rect2d r2:obstacles_list)
//                if(r1.x!=r2.x && r1.y!=r2.y && r1.width!=r2.width && r1.height!=r2.height)
//                {
//                    if(dist_max<std::abs(r1.x+r1.width/2.0-r2.x-r2.width/2.0))
//                    {
//                        dist_max=std::abs(r1.x+r1.width/2.0-r2.x-r2.width/2.0);
//                    }
//                }
        }

        for(int i=0;i<wall_boundingRect.size();i++)
        {
            if(calRectAngle(wall_boundingRect[i])<CV_PI/3.0)//horizontal
                obstacles_list.emplace_back(wall_boundingRect[i]);
        }

//        //draw
//        Mat dstImage;
//        cv::cvtColor(image_list_after_rotate[1], dstImage, COLOR_GRAY2BGR);
//       // cv::rectangle(dstImage, *it1, cv::Scalar(255, 0, 255));
//        for(int kk=0;kk<wall_boundingRect.size();kk++)
//        {
//            cv::rectangle(dstImage, wall_boundingRect[kk], cv::Scalar(255, 255, 0));
//        }
//        imshow("tp", dstImage);
//        waitKey(0);

        if(dist_max>car_norm.width_pass/resolution+car_norm.width_near_the_wall*0.8/resolution) //can divide
        {
            for(Rect2d ri:obstacles_list)
//                if(ri.x+ri.width/2.0>max_x_r.x-car_norm.width_pass*0.7/resolution && ri.x+ri.width/2.0<max_x_r.x+car_norm.width_near_the_wall*0.7/resolution )
//                    obstacles_list_right.emplace_back(ri);
//                else if (ri.x+ri.width/2.0<max_x_r.x-car_norm.width_pass/resolution-car_norm.width_near_the_space/resolution*0.3 && ri.x+ri.width/2.0>min_x_r.br().x-car_norm.width_near_the_wall*0.7/resolution )
//                    obstacles_list_left.emplace_back(ri);
                if(ri.x+ri.width/2.0>max_x_r.x-car_norm.width_pass*0.5/resolution )
                    obstacles_list_right.emplace_back(ri);
                else if (ri.x+ri.width/2.0<max_x_r.x-car_norm.width_pass*0.5/resolution)
                    obstacles_list_left.emplace_back(ri);
        }
        else  //can not divide
        {
            if(std::abs(image_list_after_rotate[0].cols-max_x)>min_x)
                for(Rect2d ri:obstacles_list)
                    // if(ri.x+ri.width/2.0>min_x_r.x-car_norm.width_near_the_wall*0.6/resolution )
                    obstacles_list_left.emplace_back(ri);
            if(std::abs(image_list_after_rotate[0].cols-max_x)<min_x)
                for(Rect2d ri:obstacles_list)
                    //if(ri.x+ri.width/2.0<max_x_r.x+car_norm.width_near_the_wall*0.6/resolution)
                    obstacles_list_right.emplace_back(ri);
        }

//        // draw
//        Mat dstImage;
//        cv::cvtColor(image_list_after_rotate[1], dstImage, COLOR_GRAY2BGR);
//       // cv::rectangle(dstImage, *it1, cv::Scalar(255, 0, 255));
//        for(int kk=0;kk<obstacles_list_right.size();kk++)
//        {
//            cv::rectangle(dstImage, obstacles_list_right[kk], cv::Scalar(255, 255, 0));
//        }
//        for(int kk=0;kk<obstacles_list_left.size();kk++)
//        {
//            cv::rectangle(dstImage, obstacles_list_left[kk], cv::Scalar(255, 0, 255));
//        }
//        imshow("tp", dstImage);
//        waitKey(0);

        cal_car_lot(obstacles_list_left);
        cal_car_lot(obstacles_list_right);
        rotate_back(valid_car_lot,valid_car_lot_rotateback);
    }



    void cal_car_lot(vector<Rect2d> &ob_list)
    {
        auto it1=ob_list.begin();
        int i1=0;
        while(it1!=ob_list.end())
        {
            Rect2d obs1,obs2;

//            //draw
//            Mat dstImage;
//            cv::cvtColor(image_list_after_rotate[1], dstImage, COLOR_GRAY2BGR);
//            cv::rectangle(dstImage, *it1, cv::Scalar(255, 0, 255));
////            for(int kk=0;kk<ob_list.size();kk++)
////            {
////                cv::rectangle(dstImage, ob_list[kk], cv::Scalar(255, 0, 0));
////            }
//            imshow("tp", dstImage);
//            waitKey(0);

            obs1 = *it1;
            int id1 = 0, id2 = 0, i2 = 0;
            id1 = i1;
            double obsticles_min_dist = image_list_after_rotate[0].rows;
            bool change_dist=false;
            Point2d tp_p1 = getCentralPoint(*it1);
            //find the nearst obstacles
            auto it2 = ob_list.begin();
            while (it2 != ob_list.end())
            {
                if (it1 != it2 && (*it2).y+(*it2).height/2.0>(*it1).y+(*it1).height/2.0)
                {
//                    //double dist_tp=calRectDist(*it1,*it2);
//                    bool can_find;
//                    if(obs1.height>obs2.height)
//                    {
//                    }
//                    else
//                    {
//                    }

//                   double dt=obs1.height>(*it2).height?obs1.height:(*it2).height;
//                    if(std::abs(obs1.y+obs1.height/2.0-(*it2).y-(*it2).height/2.0)>dt/2.0)
                    double tpppp=calRectDist(*it1, *it2);
                    if (calRectDist(*it1, *it2) < obsticles_min_dist)
                    {
                        change_dist=true;
                        obsticles_min_dist = calRectDist(*it1, *it2);
                        obs2 = *it2;
                        id2 = i2;
                    }

                }
                it2++;
                i2++;
            }

//            //draw
//            cv::rectangle(dstImage, obs2, cv::Scalar(255, 255, 0));
//            imshow("tp", dstImage);
//            waitKey(0);


            if (wall_boundingRect.size() > 0)
            {//judge if near the wall
                if(change_dist) {
                    bool near_the_wall = false;
                    double wall_min_dist = image_list_after_rotate[0].cols;
                    Rect2d nearest_wall;
                    int verticle_num=0;
                    for (Rect2d wall:wall_boundingRect) {
                        if (std::abs(calRectAngle(wall)) > 70.0 / 180.0 * CV_PI) {
                            verticle_num++;
                        }
                    }
                    for (Rect2d wall:wall_boundingRect) {
                        if (std::abs(calRectAngle(wall)) > 70.0 / 180.0 * CV_PI)
                        {
                            if(verticle_num>1)
                            {
                                if((wall.x > (*it1).x && (*it1).x>image_list_after_rotate[0].cols/2.0)
                                   || (wall.x < (*it1).x && (*it1).x<image_list_after_rotate[0].cols/2.0))
                                {
                                    if (std::abs(wall.x - (*it1).x) <= wall_min_dist)
                                    {
                                        wall_min_dist = std::abs(wall.x - (*it1).x);
                                        nearest_wall = wall;
                                    }
                                }
                            }
                            else if(verticle_num=1)
                            {
                                if((wall.x > (*it1).x && (*it1).x>image_list_after_rotate[0].cols/2.0)
                                   || (wall.x < (*it1).x && (*it1).x<image_list_after_rotate[0].cols/2.0))
                                {
                                    if (std::abs(wall.x - (*it1).x) <= wall_min_dist)
                                    {
                                        wall_min_dist = std::abs(wall.x - (*it1).x);
                                        nearest_wall = wall;
                                    }
                                }
                                else
                                {
                                    near_the_wall = false;
                                    nearest_wall.width = -100;
                                }
                            }
                            else if(verticle_num=0)
                            {
                                near_the_wall = false;
                                nearest_wall.width = -100;
                            }
                        }

                    }
                    if (wall_min_dist < (car_norm.width_pass / 2.0 + car_norm.width_near_the_wall) / resolution)
                        near_the_wall = true;
                    if (id1 > car_boundingRect.size() + pillar_boundingRect.size() &&
                        id2 > car_boundingRect.size() + pillar_boundingRect.size());
                    else cal_car_rect(obs1, obs2, nearest_wall, near_the_wall);
                }
            }
            else
            {
                if(change_dist) {
                    bool near_the_wall = false;
                    Rect2d wall_tp;
                    wall_tp.width = -100;
                    cal_car_rect(obs1, obs2, wall_tp, near_the_wall);
                }
            }
            it1++;
            i1++;
        }
    }

    void cal_car_rect(Rect2d r1,Rect2d r2,Rect2d wall,bool near_the_wall)
    {
        Point2d tp_p1=getCentralPoint(r1);
        Point2d tp_p2=getCentralPoint(r2);

        if (tp_p1.y > tp_p2.y)
        {
            double h=r1.tl().y-r2.br().y;
            //double h=tp_p1.y-tp_p2.y;
            int n=floor(h/car_norm.length*resolution);
            if(h/car_norm.length*resolution-n>0.9)n=n+1;
            if(wall.width<0)
            {
                for(int i=0;i<n;i++)
                {
                    cv::Rect2d car_tp;
                    car_tp.x = r1.x<r2.x?r1.x:r2.x;
                    car_tp.y =r2.br().y + i * car_norm.length / resolution;
                    car_tp.width = car_norm.width_near_the_space / resolution;
                    car_tp.height = car_norm.length / resolution;
                    valid_car_lot.emplace_back(car_tp);
                }
                return;
            }
            for(int i=0;i<n;i++)
            {
                cv::Rect2d car_tp;
                double x;
                if (near_the_wall) {
                    if (wall.tl().x - tp_p1.x > 0)
                        x = wall.tl().x - car_norm.width_near_the_wall / resolution;
                    else
                        x = wall.br().x;
                    car_tp.width = car_norm.width_near_the_wall / resolution;
                    car_tp.x = x;
                    car_tp.y = r2.br().y + i * car_norm.length / resolution;
                } else {
                    if (wall.tl().x - tp_p1.x > 0)
                        x = wall.tl().x - car_norm.width_near_the_wall / resolution -
                            car_norm.width_pass / resolution - car_norm.width_near_the_space / resolution;
                    else
                        x = wall.br().x + car_norm.width_near_the_wall / resolution +
                            car_norm.width_pass / resolution;
                    car_tp.width = car_norm.width_near_the_space / resolution;
                    car_tp.y = r2.br().y + i * car_norm.length / resolution;
                    car_tp.x = x;
                }
                car_tp.height = car_norm.length / resolution;
                bool is_within=false;
                for(int r=0;r<range_no_car_y.size();r++)
                {
                    if(range_no_car_y[r].x<image_list_after_rotate[0].cols/2.0)//left
                    {
                        if ((ifPointWithinRect(Point(car_tp.x,car_tp.y),
                                               0,range_no_car_y[r].x+range_no_car_y[r].width,
                                               range_no_car_y[r].y,range_no_car_y[r].y+range_no_car_y[r].height)==true)
                            || (ifPointWithinRect(Point(car_tp.x+car_tp.width,car_tp.y),
                                                  0,range_no_car_y[r].x+range_no_car_y[r].width,
                                                  range_no_car_y[r].y,range_no_car_y[r].y+range_no_car_y[r].height)==true)
                            || (ifPointWithinRect(Point(car_tp.x+car_tp.width,car_tp.y+car_tp.height),
                                                  0,range_no_car_y[r].x+range_no_car_y[r].width,
                                                  range_no_car_y[r].y,range_no_car_y[r].y+range_no_car_y[r].height)==true)
                            || (ifPointWithinRect(Point(car_tp.x,car_tp.y+car_tp.height),
                                                  0,range_no_car_y[r].x+range_no_car_y[r].width,
                                                  range_no_car_y[r].y,range_no_car_y[r].y+range_no_car_y[r].height)==true))
                            is_within=true;
                    }
                    else//right
                    {
                        if ((ifPointWithinRect(Point(car_tp.x,car_tp.y),
                                               range_no_car_y[r].x,image_list_after_rotate[0].cols,
                                               range_no_car_y[r].y,range_no_car_y[r].y+range_no_car_y[r].height)==true)
                            || (ifPointWithinRect(Point(car_tp.x+car_tp.width,car_tp.y),
                                                  range_no_car_y[r].x,image_list_after_rotate[0].cols,
                                                  range_no_car_y[r].y,range_no_car_y[r].y+range_no_car_y[r].height)==true)
                            || (ifPointWithinRect(Point(car_tp.x+car_tp.width,car_tp.y+car_tp.height),
                                                  range_no_car_y[r].x,image_list_after_rotate[0].cols,
                                                  range_no_car_y[r].y,range_no_car_y[r].y+range_no_car_y[r].height)==true)
                            || (ifPointWithinRect(Point(car_tp.x,car_tp.y+car_tp.height),
                                                  range_no_car_y[r].x,image_list_after_rotate[0].cols,
                                                  range_no_car_y[r].y,range_no_car_y[r].y+range_no_car_y[r].height)==true))
                            is_within=true;
                    }
                }
                if(is_within==false)
                    valid_car_lot.emplace_back(car_tp);
                //}
            }
        }
        else
        {
            //double h=pillar_boundingRect[min_id_p].br().y-(*itp).tl().y;
            double  h=-r1.br().y+r2.tl().y;
            int n=floor(h/car_norm.length*resolution);
            if(h/car_norm.length*resolution-n>0.9)n=n+1;
            if(wall.width<0)
            {
                for(int i=0;i<n;i++)
                {
                    cv::Rect2d car_tp;
                    car_tp.x = r1.x<r2.x?r1.x:r2.x;
                    car_tp.y=r1.br().y + i * car_norm.length / resolution;
                    car_tp.width = car_norm.width_near_the_space / resolution;
                    car_tp.height = car_norm.length / resolution;
                    valid_car_lot.emplace_back(car_tp);
                }
                return;
            }
            for(int i=0;i<n;i++)
            {
                cv::Rect2d car_tp;
                double x;
                if(near_the_wall)
                {
                    if (wall.tl().x - tp_p1.x > 0)
                        x = wall.tl().x - car_norm.width_near_the_wall / resolution;
                    else
                        x = wall.br().x;
                    // car_tp.tl() = Point2d(x, (*itp).br().y + i * car_norm.length / resolution);
                    car_tp.width=car_norm.width_near_the_wall/resolution;
                    car_tp.x=x;car_tp.y=r1.br().y + i * car_norm.length / resolution;
                } else
                {
                    if (wall.tl().x - tp_p1.x > 0)
                        x = wall.tl().x - car_norm.width_near_the_wall / resolution-car_norm.width_pass/resolution-car_norm.width_near_the_space/resolution;
                    else
                        x = wall.br().x +car_norm.width_near_the_wall / resolution+car_norm.width_pass/resolution;
                    //  car_tp.tl() = Point2d(x, (*itp).br().y +i * car_norm.length / resolution);
                    car_tp.width=car_norm.width_near_the_space/resolution;
                    car_tp.x=x;
                    car_tp.y=r1.br().y +i * car_norm.length / resolution;
                }
                car_tp.height=car_norm.length/resolution;
                bool is_within=false;
                for(int r=0;r<range_no_car_y.size();r++)
                {
                    if(range_no_car_y[r].x<image_list_after_rotate[0].cols/2.0)//left
                    {
                        if ((ifPointWithinRect(Point(car_tp.x,car_tp.y),
                                               0,range_no_car_y[r].x+range_no_car_y[r].width,
                                               range_no_car_y[r].y,range_no_car_y[r].y+range_no_car_y[r].height)==true)
                            || (ifPointWithinRect(Point(car_tp.x+car_tp.width,car_tp.y),
                                                  0,range_no_car_y[r].x+range_no_car_y[r].width,
                                                  range_no_car_y[r].y,range_no_car_y[r].y+range_no_car_y[r].height)==true)
                            || (ifPointWithinRect(Point(car_tp.x+car_tp.width,car_tp.y+car_tp.height),
                                                  0,range_no_car_y[r].x+range_no_car_y[r].width,
                                                  range_no_car_y[r].y,range_no_car_y[r].y+range_no_car_y[r].height)==true)
                            || (ifPointWithinRect(Point(car_tp.x,car_tp.y+car_tp.height),
                                                  0,range_no_car_y[r].x+range_no_car_y[r].width,
                                                  range_no_car_y[r].y,range_no_car_y[r].y+range_no_car_y[r].height)==true))
                            is_within=true;
                    }
                    else//right
                    {
                        if ((ifPointWithinRect(Point(car_tp.x,car_tp.y),
                                               range_no_car_y[r].x,image_list_after_rotate[0].cols,
                                               range_no_car_y[r].y,range_no_car_y[r].y+range_no_car_y[r].height)==true)
                            || (ifPointWithinRect(Point(car_tp.x+car_tp.width,car_tp.y),
                                                  range_no_car_y[r].x,image_list_after_rotate[0].cols,
                                                  range_no_car_y[r].y,range_no_car_y[r].y+range_no_car_y[r].height)==true)
                            || (ifPointWithinRect(Point(car_tp.x+car_tp.width,car_tp.y+car_tp.height),
                                                  range_no_car_y[r].x,image_list_after_rotate[0].cols,
                                                  range_no_car_y[r].y,range_no_car_y[r].y+range_no_car_y[r].height)==true)
                            || (ifPointWithinRect(Point(car_tp.x,car_tp.y+car_tp.height),
                                                  range_no_car_y[r].x,image_list_after_rotate[0].cols,
                                                  range_no_car_y[r].y,range_no_car_y[r].y+range_no_car_y[r].height)==true))
                            is_within=true;
                    }
                }
                if(is_within==false)
                    valid_car_lot.emplace_back(car_tp);
//                valid_car_lot.emplace_back(car_tp);
            }
        }
    }

    bool ifPointWithinRect(Point pt,int min_x,int max_x,int min_y,int max_y)
    {
        bool is_within=false;
        if(pt.x>min_x && pt.x<max_x)
            if(pt.y>min_y && pt.y<max_y)
            {
                is_within=true;
            }
        return is_within;
    }

    double calRectDist(Rect2d &r1,Rect2d &r2)
    {
        return std::abs(r1.y+r1.height/2.0-(r2.y+r2.height/2.0));
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

    double calRectAngle(Rect2d r)
    {return std::atan2(r.height,r.width); }

    void Rect2List(Rect2d & rect,vector<Point> &output)
    {
        output.emplace_back(rect.tl());
        output.emplace_back(Point(rect.tl().x+rect.width,rect.tl().y));
        output.emplace_back(rect.br());
        output.emplace_back(Point(rect.tl().x,rect.tl().y+rect.height));
    }


    void merge_contours(vector<vector<Point>> &contours_input,vector<vector<Point>> & contours_output,int thres=8)
    {
        vector<int> delete_id;

        auto itc1=contours_input.begin();
        while(itc1!=contours_input.end())
        {
            Point2d center_1;
            computeCentralPoint(*itc1,center_1);
            auto itc2=contours_input.begin();
            while(itc2!=contours_input.end())
            {
                if(itc1!=itc2)
                {
                    Point2d center_2;
                    computeCentralPoint(*itc2, center_2);
                    if (calDist(center_1, center_2) <= thres)//merge available
                    {
                        delete_id.emplace_back(itc2 - contours_input.begin());
                        for (int i = 0; i < (*itc2).size(); i++)
                            (*itc1).emplace_back((*itc2)[i]);
                    }
                }
                itc2++;
            }
            bool exist=false;
            for(int i=0;i<delete_id.size();i++)
            {
                if(itc1-contours_input.begin()==delete_id[i])
                    exist=true;
            }
            if(!exist)
                contours_output.emplace_back(*itc1);
            itc1++;
        }
    }

    void merge_rect(vector<Rect2d>& rect_input,vector<Rect2d>& rect_output,double thres=8.0)
    {
        vector<Rect2d> rect_output_tp;
        auto itr1=rect_input.begin();
        while(itr1!=rect_input.end())
        {
            Rect2d rtp1=*itr1;
            Point2d pt1(rtp1.width/2.0+rtp1.x,rtp1.height/2.0+rtp1.y);
            auto itr2=rect_input.begin();
            while(itr2!=rect_input.end())
            {
                if(itr1!=itr2)
                {
                    Point2d pt2((*itr2).width / 2.0 + (*itr2).x, (*itr2).height / 2.0 + (*itr2).y);
                    if (calRectAngle(rtp1) > 60.0 / 180.0 * CV_PI &&
                        calRectAngle(*itr2) > 60.0 / 180.0 * CV_PI)  //VERTICLE
                    {
                        /////////////////
//                        if (std::abs(pt1.x - pt2.x) < thres) {
///////////////////////
                        double min_dist_y=pt1.y>pt2.y?
                                          (*itr1).y-(*itr2).y-(*itr2).height:(*itr2).y-(*itr1).y-(*itr1).height;
                        if (std::abs(pt1.x - pt2.x) < thres  && (std::abs(min_dist_y)<30  ||  std::abs(pt1.y-pt2.y)<30  )   ) {

                            double min_x, min_y, max_x, max_y;
                            min_x = rtp1.x < (*itr2).x ? rtp1.x : (*itr2).x;
                            min_y = rtp1.y < (*itr2).y ? rtp1.y : (*itr2).y;
                            max_x = rtp1.x + rtp1.width > (*itr2).x + (*itr2).width ? rtp1.x + rtp1.width : (*itr2).x +
                                                                                                            (*itr2).width;
                            max_y = rtp1.y + rtp1.height > (*itr2).y + (*itr2).height ? rtp1.y + rtp1.height :
                                    (*itr2).y + (*itr2).height;
                            rtp1.x = min_x;
                            rtp1.y = min_y;
                            rtp1.width = max_x - min_x;
                            rtp1.height = max_y - min_y;
                            pt1 = Point2d(rtp1.width / 2.0 + rtp1.x, rtp1.height / 2.0 + rtp1.y);
                        }
                    } else if (calRectAngle(rtp1) < 60.0 / 180.0 * CV_PI &&
                               calRectAngle(*itr2) < 60.0 / 180.0 * CV_PI) //horizontal
                    {
                        /////////////////////////////////////
//                        if (std::abs(pt1.y - pt2.y) < thres) {
/////////////////////////////////////


                        double min_dist_x=pt1.x>pt2.x?
                                          (*itr1).x-(*itr2).x-(*itr2).width:(*itr2).x-(*itr1).x-(*itr1).width;
                        if (std::abs(pt1.y - pt2.y) < thres && (std::abs(min_dist_x)<50   ||  std::abs(pt1.x-pt2.x)<40) ){

                            double min_x, min_y, max_x, max_y;
                            min_x = rtp1.x < (*itr2).x ? rtp1.x : (*itr2).x;
                            min_y = rtp1.y < (*itr2).y ? rtp1.y : (*itr2).y;
                            max_x = rtp1.x + rtp1.width > (*itr2).x + (*itr2).width ? rtp1.x + rtp1.width : (*itr2).x +
                                                                                                            (*itr2).width;
                            max_y = rtp1.y + rtp1.height > (*itr2).y + (*itr2).height ? rtp1.y + rtp1.height :
                                    (*itr2).y + (*itr2).height;
                            rtp1.x = min_x;
                            rtp1.y = min_y;
                            rtp1.width = max_x - min_x;
                            rtp1.height = max_y - min_y;
                            pt1 = Point2d(rtp1.width / 2.0 + rtp1.x, rtp1.height / 2.0 + rtp1.y);
                        }
                    }
                }
                itr2++;
            }
            rect_output_tp.emplace_back(rtp1);
            itr1++;
        }
        vector<int> id;
        for(int i=0;i<rect_output_tp.size();i++)
        {
            for(int j=i+1;j<rect_output_tp.size();j++)
                if(rect_output_tp[i].x==rect_output_tp[j].x && rect_output_tp[i].y==rect_output_tp[j].y && rect_output_tp[i].width==rect_output_tp[j].width )
                    id.emplace_back(j);
        }
        for(int i=0;i<rect_output_tp.size();i++)
        {
            bool exist=false;
            for(int j=0;j<id.size();j++)
                if(i==id[j])exist=true;
            if(!exist)
                rect_output.emplace_back(rect_output_tp[i]);
        }
        for(int i=0;i<rect_output.size();i++)
        {
            if(calRectAngle(rect_output[i])>60.0/180.0*CV_PI)//VERTICLE
            {
                for(int j=0;j<image_list_after_rotate.size();j++)
                {
                    int thed=3;
                    int left_x=int(rect_output[i].x)-thed>0?int(rect_output[i].x)-thed:0;
                    int right_x=int(rect_output[i].x+rect_output[i].width+thed)<image_list_after_rotate[j].cols?
                                int(rect_output[i].x+rect_output[i].width+thed):image_list_after_rotate[j].cols;
                    int up_y=int(rect_output[i].y)-thed>0?int(rect_output[i].y)-thed:0;
                    int bottom_y=int(rect_output[i].y+rect_output[i].height+thed)<image_list_after_rotate[j].rows?
                                 int(rect_output[i].y+rect_output[i].height+thed):image_list_after_rotate[j].rows;

                    for (int k = left_x; k < right_x; k++) //cols
                        for (int l = up_y; l <bottom_y; l++)//rows
                            image_list_after_rotate[j].at<uchar>(l, k) = 0;

//                    if(rect_output[i].x<image_list_after_rotate[0].cols/2.0)
//                    {
//                        for (int k = 0; k < left_x+8; k++) //cols
//                            for (int l = 0; l < image_list_after_rotate[j].rows; l++)//rows
//                                image_list_after_rotate[j].at<uchar>(l, k) = 0;
//                        for (int k = right_x-8; k < right_x; k++) //cols
//                            for (int l = 0; l < image_list_after_rotate[j].rows; l++)//rows
//                                image_list_after_rotate[j].at<uchar>(l, k) = 0;
//                    }
//                    else
//                    {
//                        for (int k = left_x; k < left_x+8; k++) //cols
//                            for (int l = 0; l < image_list_after_rotate[j].rows; l++)//rows
//                                image_list_after_rotate[j].at<uchar>(l, k) = 0;
//                        for (int k = right_x-8; k < image_list_after_rotate[j].cols; k++) //cols
//                            for (int l = 0; l < image_list_after_rotate[j].rows; l++)//rows
//                                image_list_after_rotate[j].at<uchar>(l, k) = 0;
//                    }
                }
                //rect_output[i].y=image_list_after_rotate[0].rows*0.3;
                if(rect_output[i].width>5.0)
                {
                    rect_output[i].x=rect_output[i].x+(rect_output[i].width-3.0)/2.0;
                    rect_output[i].width=3.0;
                    //rect_output[i].height=image_list_after_rotate[0].rows*0.4;
                }
            }
            else
            {
                for(int j=0;j<image_list_after_rotate.size();j++)
                {
//                    int up_y=int(rect_output[i].y)-8>0?int(rect_output[i].y)-8:0;
//                    int bottom_y=int(rect_output[i].y+rect_output[i].height+8)<image_list_after_rotate[j].rows?
//                                 int(rect_output[i].y+rect_output[i].height+8):image_list_after_rotate[j].rows;
                    int thed=2;
                    int left_x=int(rect_output[i].x)-thed>0?int(rect_output[i].x)-thed:0;
                    int right_x=int(rect_output[i].x+rect_output[i].width+thed)<image_list_after_rotate[j].cols?
                                int(rect_output[i].x+rect_output[i].width+thed):image_list_after_rotate[j].cols;
                    int up_y=int(rect_output[i].y)-thed>0?int(rect_output[i].y)-thed:0;
                    int bottom_y=int(rect_output[i].y+rect_output[i].height+thed)<image_list_after_rotate[j].rows?
                                 int(rect_output[i].y+rect_output[i].height+thed):image_list_after_rotate[j].rows;

                    for (int k = left_x; k < right_x; k++) //cols
                        for (int l = up_y; l <bottom_y; l++)//rows
                            image_list_after_rotate[j].at<uchar>(l, k) = 0;

//                    if(up_y<image_list_after_rotate[j].rows/2.0)
//                    {
//                        for (int k = 0; k < up_y+8; k++) //rows
//                            for (int l = 0; l < image_list_after_rotate[j].cols; l++)//rows
//                                image_list_after_rotate[j].at<uchar>(k, l) = 0;
//                        for (int k = bottom_y-8; k < bottom_y; k++) //rows
//                            for (int l = 0; l < image_list_after_rotate[j].cols; l++)//rows
//                                image_list_after_rotate[j].at<uchar>(k, l) = 0;
//
//                    }
//                    else
//                    {
//                        for (int k = bottom_y; k < image_list_after_rotate[j].rows; k++) //rows
//                            for (int l = 0; l < image_list_after_rotate[j].cols; l++)//rows
//                                image_list_after_rotate[j].at<uchar>(k, l) = 0;
//                        for (int k = up_y; k < up_y+8; k++) //rows
//                            for (int l = 0; l < image_list_after_rotate[j].cols; l++)//rows
//                                image_list_after_rotate[j].at<uchar>(k, l) = 0;
//                    }
                }
                if(rect_output[i].height>6.0)
                {
                    // rect_output[i].y=rect_output[i].y+(rect_output[i].height-3.0)/2.0;
                    rect_output[i].height=3.0;
                }
            }
        }
    }



    void draw(int typet,string name="")
    {
        if(typet==1) {
            for (int i = 0; i < image_list_after_rotate.size(); i++)
            {
                Mat tp;
                cv::cvtColor(image_list_after_rotate[i], tp, CV_GRAY2BGR);
                for (int j = 0; j < car_boundingRect.size(); j++)
                    cv::rectangle(tp, car_boundingRect[j], cv::Scalar(255, 0, 255));
                for (int j = 0; j < wall_boundingRect.size(); j++)
                    cv::rectangle(tp, wall_boundingRect[j], cv::Scalar(255, 255, 0));
                for (int j = 0; j < pillar_boundingRect.size(); j++)
                    cv::rectangle(tp, pillar_boundingRect[j], cv::Scalar(0, 255, 255));
                cv::namedWindow("show", WINDOW_NORMAL);
                cv::imshow("show", tp);
                //cv::imwrite("../find_objects"+std::to_string(i+1)+".jpg", tp);
                cv::waitKey(0);
            }
        } else if(typet==2)
        {
            for (int i = 0; i < image_list_after_rotate.size(); i++)
            {
                Mat tp;
                cv::cvtColor(image_list_after_rotate[i], tp, CV_GRAY2BGR);
                for (int j = 0; j < car_boundingRect.size(); j++)
                    cv::rectangle(tp, car_boundingRect[j], cv::Scalar(255, 0, 255));
                for (int j = 0; j < wall_boundingRect.size(); j++)
                    cv::rectangle(tp, wall_boundingRect[j], cv::Scalar(255, 255, 0));
                for (int j = 0; j < pillar_boundingRect.size(); j++)
                    cv::rectangle(tp, pillar_boundingRect[j], cv::Scalar(0, 255, 255));
                for (int j = 0; j < valid_car_lot.size(); j++)
                    cv::rectangle(tp, valid_car_lot[j], cv::Scalar(0, 0, 255));
                if(name=="")
                    cv::imwrite("../data/image3/"+name,tp);
                else
                {
//                    cv::namedWindow("show", WINDOW_NORMAL);
//                    cv::imshow("show", tp);
                    cv::imwrite("../data/new/4/result-new" + name, tp);
                    // cv::waitKey(0);
                }
            }
        }
        else if (typet==3)
        {
            for (int i = 0; i < image_list_orig.size(); i++)
            {
                Mat tp;
                cv::cvtColor(image_list_orig[i], tp, CV_GRAY2BGR);
                for (int j = 0; j < car_boundingRect.size(); j++)
                    drawRotateRect(tp, car_boundingRect_rotateback[j], cv::Scalar(255, 0, 255));
                for (int j = 0; j < wall_boundingRect.size(); j++)
                    drawRotateRect(tp, wall_boundingRect_rotateback[j], cv::Scalar(255, 255, 0));
                for (int j = 0; j < pillar_boundingRect.size(); j++)
                    drawRotateRect(tp, pillar_boundingRect_rotateback[j], cv::Scalar(0, 255, 255));
                for (int j = 0; j < valid_car_lot.size(); j++)
                    drawRotateRect(tp, valid_car_lot_rotateback[j], cv::Scalar(0, 0, 255));
//                cv::namedWindow("show", WINDOW_NORMAL);
//                cv::imshow("show", tp);
                cv::imwrite("../data/new/5/result-new" + name, tp);
//                cv::waitKey(0);
            }
        }
    }

    void drawRotateRect(Mat& image,mRotatedRect &rotate_rect,const cv::Scalar &color)
    {
        for (int j = 0; j < 4; j++)
        {
            cv::line(image,rotate_rect.point_list[j], rotate_rect.point_list[(j + 1) % 4], color);
        }
    }


private:

    octoMap_processing map=octoMap_processing();
    double resolution;
    vector<Mat> image_list_orig;
    vector<Mat> image_list_after_rotate;

    vector<Rect2d> car_boundingRect;
    vector<Rect2d> wall_boundingRect;
    vector<Rect2d> pillar_boundingRect;
    vector<Rect2d> valid_car_lot;
    vector<Rect2d> range_no_car_y;
    vector<mRotatedRect> car_boundingRect_rotateback;
    vector<mRotatedRect> wall_boundingRect_rotateback;
    vector<mRotatedRect> pillar_boundingRect_rotateback;
    vector<mRotatedRect> valid_car_lot_rotateback;

    vector<Rect2d> obstacles_list;

    vector<Rect2d> obstacles_list_left;
    vector<Rect2d> obstacles_list_right;

    double wall_angle;
    Point2f center;
    double rotate_angle;
    car_space_norm car_norm;

    bool is_walls_exist;

};

#endif //OCTOMAP_DETECTOR_DETECTELEM_H

