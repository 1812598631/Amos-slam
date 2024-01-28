//
// Created by run on 22-6-15.
//

#ifndef CLUSTER_H
#define CLUSTER_H

//
// Created by run on 22-6-15.
//

#include<iostream>
#include<opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <sys/time.h>

using namespace cv;
using namespace std;

namespace ORB_SLAM2
{
    struct center{
        int x;//column
        int y;//row
        int L;
        int A;
        int B;
        int D;
        int label;
        int id;
    };
    class cluster
    {
    public:
        // 构造函数
        cluster(const cv::Mat &im, const cv::Mat &imD,vector<center> &centers, const int&  nk);
        // 析构函数
        ~cluster();

    public:
        cv::Mat mIm;
        cv::Mat mImD;
        cv::Mat labelMask;//save every pixel's label保存每个像素的标签
        // 执行一张图像的预测
        vector<vector<Point2i>> points;
        void evalImage(const cv::Mat &im, const cv::Mat &imD,vector<center> &centers, const int &nk);

    private:
        std::vector<std::vector<int>> label;
        //vector< vector<T> > dataSet;//the data set
        std::vector< center > dataSet;
        std::vector< center > mmin,mmax;
        int colLen,rowLen;//colLen:the dimension of vector;rowLen:the number of vectors
        int k;
        int len;
        int m;
        std::vector< center > centroids;

        typedef struct Node
        {
            int minIndex; //the index of each node
            double minDist;
            Node(int idx,double dist):minIndex(idx),minDist(dist) {}
        }tNode;
        std::vector<tNode>  clusterAssment;//聚类

    private:
    /**SILC相关函数 */
        int clustering(const cv::Mat &imageLAB,const cv::Mat &DepthImage, cv::Mat &DisMask, cv::Mat &labelMask,std::vector<center> &centers, int len, int m);
        int updateCenter(cv::Mat &imageLAB, cv::Mat &labelMask,cv::Mat const &Depth,std::vector<center> &centers, int len);
        int initilizeCenters(cv::Mat &imageLAB,cv::Mat const &imagedepth, std::vector<center> &centers, int len);
        int fituneCenter(cv::Mat &imageLAB, cv::Mat &sobelGradient, std::vector<center> &centers);
        int SLIC(cv::Mat const &image,cv::Mat const &image_D, cv::Mat &resultLabel, std::vector<center> &centers, int len, int m);
    /**k-means相关函数 */
        void initClusterAssment();
        double distEclud(center &v1 , center &v2);
        void loadDataSet(std::vector<center> &centers);
        void randCent();
        void kmeans();

    };      // class cluster
}       // namespace orb-slam


#endif //CLUSTER_H
