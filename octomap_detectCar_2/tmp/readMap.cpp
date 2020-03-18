//
// Created by songsong on 18-9-11.
//

//
//#include <AbstractOcTree.h>

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

int main( int argc, char** argv )
{
    //string filename="../data/58.bt";
    OcTree map_orig	("../data/58.bt");
    ColorOcTree getXYZ_map(map_orig.getResolution());

    cout<<"------------basic information----------------"<<endl;
    double maxx,maxy,maxz;
    map_orig.getMetricMax(maxx,maxy,maxz);
    cout<<"x="<<maxx<<" y="<<maxy<<" z="<<maxz<<endl;
    double minx,miny,minz;
    map_orig.getMetricMin(minx,miny,minz);
    cout<<"x="<<minx<<" y="<<miny<<" z="<<minz<<endl;
    cout<<"resolution="<<map_orig.getResolution()<<endl<<"-------------------------"<<endl;

//    //translate to color_map
//    int i=0;
//    for(OcTree::leaf_iterator it=map_orig.begin_leafs(),
//                end=map_orig.end_leafs();it!=end;++it)
//    {
//        i+=1;
//        getXYZ_map.updateNode(octomap::point3d(it.getX(),it.getY(),it.getZ()),it->getLogOdds());
//        // cout<<it->getOccupancy()<<endl;
//    }
//    cout<<i<<endl;
//    for(OcTree::leaf_iterator it=map_orig.begin_leafs(),
//                end=map_orig.end_leafs();it!=end;++it)
//    {
//        i+=1;
//        //   getXYZ_map.updateNode(octomap::point3d(it.getX(),it.getY(),it.getZ()),true);
//        double tp=2.0;
////        if(it.getY()>=-tp && it.getY()<=tp && it.getZ()>=-tp && it.getZ()<=tp  )//X axis
////            getXYZ_map.integrateNodeColor(it.getX(),it.getY(),it.getZ(),255,0,0);
////        else if(it.getX()>=-tp && it.getX()<=tp && it.getY()>=-tp && it.getY()<=tp  )//Z axis
////            getXYZ_map.integrateNodeColor(it.getX(),it.getY(),it.getZ(),0,255,0);
////        else if(it.getX()>=-tp && it.getX()<=tp && it.getZ()>=-tp && it.getZ()<=tp  )//Y axis
////            getXYZ_map.integrateNodeColor(it.getX(),it.getY(),it.getZ(),0,0,255);
////        else if (it.getY()>0.6)
////            getXYZ_map.integrateNodeColor(it.getX(),it.getY(),it.getZ(),0,0,255);
//        if (it.getY()<=0.7 && it.getY()>=0.6)
//            getXYZ_map.integrateNodeColor(it.getX(),it.getY(),it.getZ(),0,0,255);
//        // std::cout<<it->getValue()<<std::endl;
//    }
//    cout<<i<<endl;
//    getXYZ_map.updateInnerOccupancy();
//    // 存储octomap, 注意要存成.ot文件而非.bt文件
//    getXYZ_map.write( "../data/Map_with_XYZAxis.ot" );
//    cout<<"done."<<endl;



    //get image_Mat
    double rsl=map_orig.getResolution();
    int rows=(int)((maxz-minz)/rsl);
    int cols=(int)((maxx-minx)/rsl);
//    for(int i=miny/rsl;i<maxy/rsl;i++)
//    {
//        cv::Mat* image=new cv::Mat(rows,cols,CV_64FC1);
//        double depth=(double)(i)*rsl;
//        for(OcTree::leaf_iterator it=map_orig.begin_leafs(),
//                    end=map_orig.end_leafs();it!=end;++it)
//        {
//            int r=(int)((it.getZ()-minz)/rsl);
//            int c=(int)((it.getX()-minx)/rsl);
//            // cout<<it->getOccupancy()<<endl;
//            if(it.getY()<=depth+rsl && it.getY()>=depth)
//            {
//                if(it->getOccupancy()<0.5)
//                    image->at<double>(r,c)=0;
//                else
//                    image->at<double>(r,c)=255;
//            }
//        }
//       std::string filename="../data/image2/image_"+to_string(depth)+ "m.jpg";
//        cv::imwrite(filename,*image);
//    }
//
    cv::Mat* image=new cv::Mat(rows,cols,CV_64FC1);
    for(OcTree::leaf_iterator it=map_orig.begin_leafs(),
                    end=map_orig.end_leafs();it!=end;++it)
        {
            int r=(int)((it.getZ()-minz)/rsl);
            int c=(int)((it.getX()-minx)/rsl);
            // cout<<it->getOccupancy()<<endl;
            if(it.getY()<=0.7 && it.getY()>=0)
            {
                if(it->getOccupancy()<0.5)
                    image->at<double>(r,c)=0;
                else
                    image->at<double>(r,c)=255;
            }
        }
       std::string filename="../data/image2/image_0-0.7m.jpg";
        cv::imwrite(filename,*image);



    image=new cv::Mat(rows,cols,CV_64FC1);
            for(OcTree::leaf_iterator it=map_orig.begin_leafs(),
                    end=map_orig.end_leafs();it!=end;++it)
        {
            int r=(int)((it.getZ()-minz)/rsl);
            int c=(int)((it.getX()-minx)/rsl);
            // cout<<it->getOccupancy()<<endl;
            if(it.getY()<=0 && it.getY()>=-0.7)
            {
                if(it->getOccupancy()<0.5)
                    image->at<double>(r,c)=0;
                else
                    image->at<double>(r,c)=255;
            }
        }
       filename="../data/image2/image-0.7-0m.jpg";
        cv::imwrite(filename,*image);


//    cv::namedWindow("image_0.5",CV_WINDOW_NORMAL);
//    cv::imshow("image_0.5",*image);
//    cv::waitKey(0);

    system("pause");
    return 0;
}



//int main() {
//    std::cout << "Hello, World!" << std::endl;
//    return 0;
//}
