//
// Created by songsong on 18-9-12.
//



#include <iostream>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeNode.h>
#include <octomap/OcTreeStamped.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeIterator.hxx>
#include <octomap/ColorOcTree.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <math.h>
#include "detect_Objects.h"
#include "octoMap_processing.h"
using namespace std;
using namespace octomap;
using namespace cv;
#define pi 3.1415926


void drawContourlines(Mat& orig,vector<Point> contour,cv::Scalar color)
{
    //Mat median =cv::med  medianBlur(image_positive,5);
//绘制轮廓图1_single lines
    for(int j=0;j<contour.size()-1;j++)
    {
        Point tp1=contour.at(j);
        Point tp2=contour.at(j+1);
        cv::line( orig, tp1, tp2,  color, 2);
    }
}

void drawContoursLines(Mat& orig,Mat& out, vector<vector<Point>> contours,vector<Vec4i> hierarchy,int type)
{
    if(type==1)
    {
        //Mat median =cv::med  medianBlur(image_positive,5);
//绘制轮廓图1_single lines
        out = Mat::zeros(orig.size(), CV_8UC3);
        orig.copyTo(out);
        for(int i=0;i<contours.size();i++)
        {
            vector<Point>::iterator it=contours[i].begin();
            for(int j=0;j<contours[i].size()-1;j++)
            {
                Point tp1=contours[i].at(j);
                Point tp2=contours[i].at(j+1);
                cv::line( out, tp1, tp2,  cv::Scalar(100), 2);
            }
        }
//        cv::namedWindow("image",CV_WINDOW_NORMAL);
//        imshow("image", out);
//        waitKey(0);
    }
    else
    {
//绘制轮廓图2_use drawCOntours_with boundRect
        orig.copyTo(out);
        for (int i = 0; i < hierarchy.size(); i++)
        {
            Scalar color = Scalar(200);
            //drawContours(out, contours, i, color, CV_FILLED, 8, hierarchy);
            Rect r = boundingRect(Mat(contours[i]));
            rectangle(out, r, Scalar(200), 1);
        }
//        cv::namedWindow("image",CV_WINDOW_NORMAL);
//        imshow("image", out);
//        waitKey(0);
    }
}



void getRoateImage(Mat& image_orig,Mat& image_rotate,double& rotate_angle,double wall_angle)
{
    dilate(image_orig,image_orig,Mat());
    //erode(image_positive,image_positive,Mat());
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    //findContours(image_positive,contours,hierarchy,RETR_LIST,CHAIN_APPROX_SIMPLE);
    findContours(image_orig,contours,hierarchy,RETR_LIST,CHAIN_APPROX_NONE);

 //   Mat median =cv::medianBlur(image_positive,5);
//绘制轮廓图1_single lines
    Mat dstImage = Mat::zeros(image_orig.size(), CV_8U);
    for(int i=0;i<contours.size();i++)
    {
        vector<Point>::iterator it=contours[i].begin();
        for(int j=0;j<contours[i].size()-1;j++)
        {
            Point tp1=contours[i].at(j);
            Point tp2=contours[i].at(j+1);
            cv::line( image_orig, tp1, tp2,  cv::Scalar(100), 2);
        }
    }
    cv::namedWindow("image",CV_WINDOW_NORMAL);
    imshow("image", image_orig);
    waitKey(0);

//绘制轮廓图2_use drawCOntours_with boundRect
//    Mat dstImage = Mat::zeros(image_orig.size(), CV_8UC3);
//    for (int i = 0; i < hierarchy.size(); i++)
//    {
//        Scalar color = Scalar(255, 255, 0);
//        drawContours(dstImage, contours, i, color, CV_FILLED, 8, hierarchy);
//        Rect r = boundingRect(Mat(contours[i]));
//        rectangle(dstImage, r, Scalar(0,100,100), 2);
//    }
//    cv::namedWindow("image",CV_WINDOW_NORMAL);
//    imshow("image", dstImage);
//    waitKey(0);

    //compute the wall azimuth
    //judge the wall contour by using the  number of points
    int max=contours.size(),wall_id=0;
    Rect wall_border;
    for(int i=0;i<contours.size();i++)
    {
        if(contours[i].size()>=max)
        {
            wall_id=i;
            max=contours[i].size();
        }
    }
    wall_border=boundingRect(Mat(contours[wall_id]));
    double angle_tp1=atan(double(wall_border.tl().y-wall_border.br().y)/ double(wall_border.tl().x-wall_border.br().x)) ;
    double angle_tp2= atan(double(wall_border.tl().y-wall_border.br().y)/ double(wall_border.tl().x-wall_border.br().x+2*wall_border.width));
    int num_tp1=0,num_tp2=0;
    for(int i=0;i<contours[wall_id].size();i++)
    {
        double angle1=atan(double(contours[wall_id].at(i).y-wall_border.br().y)/double( contours[wall_id].at(i).x-wall_border.br().x));
        double angle2=atan(double(contours[wall_id].at(i).y-wall_border.br().y)/double( contours[wall_id].at(i).x-wall_border.br().x+wall_border.width));
        if(  abs(angle1-angle_tp1)<=CV_PI/180.0*5.0  ) num_tp1+=1;
        if(  abs(angle2-angle_tp2)<=CV_PI/180.0*5.0  ) num_tp2+=1;
    }
    if(num_tp1>num_tp2)wall_angle=angle_tp1;
    else wall_angle=angle_tp2;//radian

    //rotate the image_rotate_angle is angle by the end
    cv::Point2f center(image_orig.cols/2,image_orig.rows/2);
    cv::Mat rot_matrix;
    if(wall_angle<0)//需要逆时针旋转
    {
        rotate_angle=CV_PI/2.0+wall_angle;
        rotate_angle=rotate_angle/CV_PI*180.0;
        rot_matrix=cv::getRotationMatrix2D(center,rotate_angle,1);
    }
    else
    {
        rotate_angle=wall_angle-CV_PI/2.0;
        rotate_angle=rotate_angle/CV_PI*180.0;
        rot_matrix=cv::getRotationMatrix2D(center,rotate_angle,1);
    }
    cv::Rect bbox = cv::RotatedRect(center, image_orig.size(), rotate_angle).boundingRect();
    warpAffine( image_orig, image_rotate, rot_matrix, bbox.size() );
//    cv::namedWindow("after_rotate",CV_WINDOW_NORMAL);
//    imshow("after_rotate", image_rotate);
    imwrite("../data/afterRotate.jpg",image_rotate);
    // waitKey(0);
}


double calDist(Point p1,Point p2)
{
    return sqrt( pow(p1.x-p2.x,2.0)+pow(p1.y-p2.y,2.0));
}

int main( int argc, char** argv )
{
    int parking_type;  //1:vertical to the wall   //2:horizontal to the wall
    string filename="../data/58.bt";
    Mat image_positive,image_negative;
    subtract2DMat(filename,image_positive,0,0.7);
    subtract2DMat(filename,image_negative,-0.5,0);
    double rsl=0.1;
    // Mat image_positive=cv::imread("../data/image_0-0.7m.jpg");
    //Mat image_negative=cv::imread("../data/image-0.7-0m.jpg",0);
    //cout<<image_positive<<endl;
    Mat image_rotate;
    double rotate_angle,wall_angle;
    getRoateImage(image_positive,image_rotate,rotate_angle,wall_angle);
//    cv::namedWindow("after_rotate",CV_WINDOW_NORMAL);
//    imshow("after_rotate", image_rotate);
//    waitKey(0);

    //find pillar_use image after rotating.
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(image_rotate,contours,hierarchy,RETR_LIST,CHAIN_APPROX_NONE);
    vector<mContour> contour_list;
    for(int i=0;i<contours.size();i++)
    {
        Rect bound=boundingRect(contours[i]);
        if(bound.width<=7 or bound.height<=7);
        else {
            mContour tp = mContour(contours[i], hierarchy[i], 0);
            contour_list.push_back(tp);
        }
    }
    //show
//    Mat dstImage_tp = Mat::zeros(image_positive.size(), CV_8UC3);
//    image_rotate.copyTo(dstImage_tp);
//    Mat dstImage;
//    dstImage_tp.convertTo(dstImage,CV_8UC3);

    Mat dstImage = Mat::zeros(image_positive.size(), CV_8UC1);
    image_rotate.copyTo(dstImage);
    //to show
//    Mat image_tp;
//    drawContoursLines(dstImage,image_tp, contours,hierarchy,2);
//    cv::namedWindow("image",CV_WINDOW_NORMAL);
//    imshow("image", image_tp);
//    waitKey(0);


    //judge the contour car or pillow or wall
    int max_point_num=contour_list[0].point_list.size();
    int wall_id=0;
    for(int i=0;i<contour_list.size();i++)
    {
        if(max_point_num<contour_list[i].point_list.size())
        {
            max_point_num=contour_list[i].point_list.size();
            wall_id=i;
        }
    }
    contour_list[wall_id].type=1;
    //get wall_line
    vector<Point> wall_line;
    Point* tp_wall=new Point(contour_list[wall_id].boundRect.tl().x+contour_list[wall_id].boundRect.width/2.0,contour_list[wall_id].boundRect.tl().y);
    wall_line.push_back(*tp_wall);
    tp_wall=new  Point(contour_list[wall_id].boundRect.tl().x+contour_list[wall_id].boundRect.width/2.0,contour_list[wall_id].boundRect.tl().y);
    wall_line.push_back(*tp_wall);

    vector<mContour>::iterator itc=contour_list.begin();
    while(itc!=contour_list.end())
    {
        if  ((*itc).boundRect.width<20 && (*itc).boundRect.height<20)
            (*itc).type=3;//pillar
        else {
            if ((*itc).type != 1)(*itc).type = 2; //car
        }
        itc++;
    }


    Mat image_tp=Mat(dstImage.size(),CV_8UC3);
    Mat* channel=new Mat[3];
    for(int i=0;i<3;i++)
    dstImage.copyTo(channel[i]);
    merge(channel,3,image_tp);
    //dstImage.copyTo(image_tp);

    for(int i=0;i<contour_list.size();i++)
    {
        if(contour_list[i].type==2)//car
            rectangle(image_tp,contour_list[i].boundRect.tl(),contour_list[i].boundRect.br(),cv::Scalar(255,255,0));
        if(contour_list[i].type==3)//pillar
            rectangle(image_tp,contour_list[i].boundRect.tl(),contour_list[i].boundRect.br(),cv::Scalar(0,255,255));
        if(contour_list[i].type==1)//pillar
            rectangle(image_tp,contour_list[i].boundRect.tl(),contour_list[i].boundRect.br(),cv::Scalar(255,0,255));
    }
    cv::namedWindow("image",CV_WINDOW_NORMAL);
    imshow("image", image_tp);
    waitKey(0);

    //get car size
    //here is one problem as we don't know if there is one pillar or whether the car is more than one    vector<Rect> available_parking_space;
    vector<Rect> available_parking_space;
    vector<Rect> car_list;
    itc=contour_list.begin();
    while(itc!=contour_list.end())
    {
        if((*itc).type==2) //car
        {
            Rect tp;
            tp.tl()=(*itc).boundRect.tl();
            tp.br()= Point(wall_line[0].x,(*itc).boundRect.br().y);
            car_list.push_back(tp);
        }
        itc++;
    }
    double space_width=0,space_height=0;
    for(int i=0;i<car_list.size();i++)
    {
        space_width+=car_list[i].width;
        space_height+=car_list[i].height;
    }
    space_height/=(double)car_list.size();
    space_width/=(double)car_list.size();
    if(space_height>space_width)
        parking_type=2;//horizontal
    else parking_type=1;//vertical

    for(int i=0;i<contour_list.size()-1;i++)
    {
        if(contour_list[i].type==3) {
            // vector<mContour> possible_list;
            mContour possible_contour=mContour();
            vector<double>dist_list;
            double dist=dstImage.rows;
            for (int j = i + 1; j < contour_list.size(); j++) {
                double angle = atan((double) (contour_list[i].centralPoint.y - contour_list[j].centralPoint.y) /
                                    (double) (contour_list[i].centralPoint.x - contour_list[j].centralPoint.x));
                if (abs(angle - CV_PI / 2.0) / CV_PI * 180.0 <= 5.0)  //angle
                {
                    double  dist_tp = calDist(contour_list[i].centralPoint, contour_list[j].centralPoint);
                    if (dist_tp<dist) possible_contour=contour_list[j];
//                        dist_list.push_back(dist);
//                        possible_list.push_back(contour_list[j]);
                }
            }
            dist=calDist(contour_list[i].centralPoint, possible_contour.centralPoint);
            int car_num=0;
            double length;
            if(parking_type==1) {car_num=round(dist/space_height);length=space_height;}
            else {car_num=round(dist/space_width);length=space_width;}
            if(possible_contour.centralPoint.y>contour_list[i].centralPoint.y) {
                if (possible_contour.centralPoint.x < dstImage.cols/2.0) //in the left side
                {
                    for (int k = 0; k < car_num; k++) {
                        Rect tp_space;
                        tp_space.br()= Point( contour_list[i].centralPoint.x,contour_list[i].centralPoint.y-k*length);
                        tp_space.tl()= Point(0,tp_space.br().y-length);
                        available_parking_space.push_back(tp_space);
                    }
                }
                if (possible_contour.centralPoint.x > dstImage.cols/2.0) //in the right side
                {
                    for (int k = 0; k < car_num; k++) {
                        Rect tp_space;
                        tp_space.br()= Point( contour_list[i].centralPoint.x,contour_list[i].centralPoint.y-k*length);
                        tp_space.tl()= Point(0,tp_space.br().y-length);
                        available_parking_space.push_back(tp_space);
                    }
                }
            }
            else
            {
                if (possible_contour.centralPoint.x > dstImage.cols/2.0) //in the right side
                {
                    for (int k = 0; k < car_num; k++) {
                        Rect tp_space;
                        tp_space.br()= Point( contour_list[i].centralPoint.x,contour_list[i].centralPoint.y-k*length);
                        tp_space.tl()= Point(0,tp_space.br().y-length);
                        available_parking_space.push_back(tp_space);
                    }
                }
            }
        }
    }

    Mat image_detect_result;
    image_rotate.copyTo(image_detect_result);
    for(int i=0;i<contour_list.size();i++)
    {
        if (contour_list[i].type==1)
            line(image_detect_result,
                 Point(contour_list[i].boundRect.tl().x+contour_list[i].boundRect.width/2.0,contour_list[i].boundRect.tl().y),
                 Point(contour_list[i].boundRect.tl().x+contour_list[i].boundRect.width/2.0,contour_list[i].boundRect.tl().y),
                 cv::Scalar(255,255,0));
    }

    system("pause");
    return 0;
}


