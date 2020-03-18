//
// Created by songsong on 18-9-21.
//
#include "detect_Objects.h"
#include "readFolder.h"
int main( int argc, char** argv )
{
//    cv::namedWindow("tp",WINDOW_NORMAL);
//    string filepath3="../data/3/high/46.png";
//    string filepath4="../data/3/low/46.png";
//    detect_object detect_ob(filepath3,filepath4);
//    detect_ob.detect_wall_rotate();
//    detect_ob.detect_wall_Pillar_car();
//    detect_ob.compute_car_space();
//    detect_ob.draw(2);

    string filepath1="../data/4/high/";
    string filepath2="../data/4/low/";

    vector<string> file_list_high;
    vector<string> file_list_low;
    int tp;
    tp=readFileList(filepath1,file_list_high);
    tp=readFileList(filepath2,file_list_low);

    for(int i=0;i<file_list_high.size();i++)
    {
        boost::timer timer_;
        detect_object detect_ob(file_list_high[i],file_list_low[i]);
        detect_ob.detect_wall_rotate();
        detect_ob.detect_wall_Pillar_car();
        detect_ob.compute_car_space();
        cout<<file_list_high[i]<<endl;
        cout<<file_list_low[i]<<endl;
        cout << "time: " << timer_.elapsed() <<endl;
        const char* tpn=std::strrchr(file_list_high[i].c_str(),'/');
        detect_ob.draw(2,string(tpn));
        //detect_ob.draw(2);
    }


    system("pause");
    return 0;
}