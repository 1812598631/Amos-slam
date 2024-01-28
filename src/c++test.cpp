#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <cstdio>
#include "yolact.h"
#include <opencv2/opencv.hpp>
#include <time.h>//引入头文件
using namespace std;
int main() {
    cout<<"Test yolact c++ interface."<<endl;
    cout<<"Complied at "<<__TIME__<<" "<<__DATE__<<"."<<endl;
    timeval start, end;
    double totaltime;
    ORB_SLAM2::yolact YOLACT("/home/run/桌面/SLAM/ORB-SLAM-yolact/src/python/yolact_interface.py",
                          "/home/run/桌面/SLAM/ORB-SLAM-yolact/src/python/weights/yolact_resnet50_54_800000.pth",
                          20);
    if(!YOLACT.isInitializedResult())
    {
        cout<<"[C++] Something wrong."<<endl;
        cout<<"[C++] "<<YOLACT.getErrorDescriptionString()<<endl;
        return 0;
    }

    cout<<"[C++] OK."<<endl;
    cv::Mat input_img=cv::imread("/home/run/桌面/SLAM/ORB-SLAM-yolact/bin/1341846313.553992.png");
    cv::Mat confidence_img;
    cout<<"[C++] Ready to eval image ..."<<endl;

    gettimeofday(&start,NULL);
    bool res=YOLACT.evalImage(input_img,confidence_img);
    gettimeofday(&end, NULL);
    totaltime =1000*(end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec)/1000;
    cout<<"程序运行时间为："<<totaltime<<"毫秒!"<<endl;
    if(res)
    {
        cout<<"[C++] Eval OK."<<endl;
    }
    else
    {
        cout<<"[C++] Eval Failed."<<endl;
        cout<<"[C++] "<<YOLACT.getErrorDescriptionString()<<endl;
        return 0;
    }

    cv::imshow("c_img",confidence_img);
    cv::waitKey(0);
    return 0;
}