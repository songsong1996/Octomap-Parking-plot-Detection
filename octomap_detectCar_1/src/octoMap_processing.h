//
// Created by songsong on 18-9-18.
//

#ifndef OCTOMAP_DETECTOR_READFILE_H
#define OCTOMAP_DETECTOR_READFILE_H



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

using namespace std;
using namespace octomap;
using namespace cv;

class octoMap_processing
{
public:
//  OctoMapReader(OcTree tp){tp.;}
    octoMap_processing(string& filepath)
    {

        filename=filepath;
        map=new OcTree(filepath);
        resolution=map->getResolution();
        cout<<"------------basic information----------------"<<endl;
        map->getMetricMax(maxx,maxy,maxz);
        cout<<"maxx="<<maxx<<" maxy="<<maxy<<" maxz="<<maxz<<endl;
        map->getMetricMin(minx,miny,minz);
        cout<<"minx="<<minx<<" miny="<<miny<<" minz="<<minz<<endl;
        cout<<"resolution="<<map->getResolution()<<endl<<"-------------------------"<<endl;
    }
    //subtract image from the OcTree
    void subtract2DMat(Mat& image,double height_bottom,double height_up,string output_filename="../data/image2/subtract.jpg")
    {
        map=new OcTree(filename);
        int rows=(int)((maxz-minz)/resolution);
        int cols=(int)((maxx-minx)/resolution);
        image=cv::Mat(rows,cols,CV_8UC1);
        for(octomap::OcTree::leaf_iterator it = map->begin_leafs(),
                    end=map->end_leafs(); it!= end; ++it)
        {
            int r=(int)((it.getZ()-minz)/resolution);
            int c=(int)((it.getX()-minx)/resolution);
            if(it.getY()<=height_up && it.getY()>=height_bottom)
            {
                if(it->getOccupancy()<0.5)
                    image.at<uchar>(r,c)=0;
                else
                    image.at<uchar>(r,c)=255;
            }
        }
        std::string filename=output_filename;
        cv::imwrite(filename,image);
    }
    octoMap_processing(){}
    ~octoMap_processing()
    {
    delete(map);
    }
    double getResolution()
    {return resolution;}
private:
    string filename;
    OcTree *map=new OcTree(0.1);
    double resolution;
    double subtract_height_end;
    double subtract_height_start;
    double minx;
    double miny;
    double minz;
    double maxx;
    double maxy;
    double maxz;
};




#endif //OCTOMAP_DETECTOR_READFILE_H
