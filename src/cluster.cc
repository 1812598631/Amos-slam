//
// Created by run on 22-6-15.
//
#include <opencv2/imgproc/types_c.h>
#include "cluster.h"


namespace ORB_SLAM2 {
    cluster::cluster(const cv::Mat &im, const cv::Mat &imD,vector<center> &centers,const int &nk)
            :k(nk) {
        int len = 5;//the scale of the superpixel ,len*len
        int m = 10;//a parameter witch adjust the weights of spacial distance and the color space distance
        SLIC(im,imD, labelMask, centers, len, m);

        loadDataSet(centers);
        randCent();
        kmeans();
        Mat kmeans_cluster_Map=Mat::zeros(im.size(), CV_8UC1);

        for (int i = 0; i < k; i++) {
            for (int ck = 0; ck < label[i].size(); ck++) {
                if (label[i].empty()) break;
                //points.push_back(Point2i(centers[label[i][ck]-1].x, centers[label[i][ck]-1].y));
                centers[label[i][ck]-1].id=i;
            }
            if (label[i].empty()) continue;
        }

        for (int i = 0; i < k; i++) {
            //对每个超像素块赋值聚类ID
            vector<Point2i> temPoint;
            points.push_back(temPoint);
            for (int ck = 0; ck < label[i].size(); ck++) {
                if (label[i].empty()) break;
                points[i].push_back(Point2i(centers[label[i][ck] - 1].x, centers[label[i][ck] - 1].y));
                ///同一个聚类的超像素块为：centers[kms.label[i][ck]-1]
                centers[label[i][ck] - 1].id = i;
            }

            if (label[i].empty()) continue;
        };

    }

    cluster::~cluster() {}

    void cluster::evalImage(const cv::Mat &im, const cv::Mat &imD,vector<center> &centers, const int &nk)
    {
        SLIC(im,imD, labelMask, centers, len, m);

        loadDataSet(centers);
        randCent();
        kmeans();
        Mat kmeans_cluster_Map=Mat::zeros(im.size(), CV_8UC1);

        for (int i = 0; i < k; i++) {
            for (int ck = 0; ck < label[i].size(); ck++) {
                if (label[i].empty()) break;
                //points.push_back(Point2i(centers[label[i][ck]-1].x, centers[label[i][ck]-1].y));
                centers[label[i][ck]-1].id=i;
            }
            if (label[i].empty()) continue;
        }

        for (int i = 0; i < k; i++) {
            //对每个超像素块赋值聚类ID
            vector<Point2i> temPoint;
            points.push_back(temPoint);
            for (int ck = 0; ck < label[i].size(); ck++) {
                if (label[i].empty()) break;
                points[i].push_back(Point2i(centers[label[i][ck] - 1].x, centers[label[i][ck] - 1].y));
                ///同一个聚类的超像素块为：centers[kms.label[i][ck]-1]
                centers[label[i][ck] - 1].id = i;
            }

            if (label[i].empty()) continue;
        };
    }
//input parameters:
//imageLAB:    the source image in Lab color space
//DisMask:       it save the shortest distance to the nearest center
//labelMask:   it save every pixel's label
//centers:       clustering center
//len:         the super pixls will be initialize to len*len
//m:           a parameter witch adjust the weights of the spacial and color space distance
//
//output:
    int cluster::clustering(const cv::Mat &imageLAB,const cv::Mat &DepthImage, cv::Mat &DisMask, cv::Mat &labelMask,
                   std::vector<center> &centers, int len, int m)
    {
        if (imageLAB.empty())
        {
            std::cout << "clustering :the input image is empty!\n";
            return -1;
        }

        double *disPtr = NULL;//disMask type: 64FC1
        double *labelPtr = NULL;//labelMask type: 64FC1
        const uchar *imgPtr = NULL;//imageLAB type: 8UC3

        //disc = std::sqrt(pow(L - cL, 2)+pow(A - cA, 2)+pow(B - cB,2))
        //diss = std::sqrt(pow(x-cx,2) + pow(y-cy,2));
        //dis = sqrt(disc^2 + (diss/len)^2 * m^2)
        double dis = 0, disc = 0, diss = 0;
        //cluster center's cx, cy,cL,cA,cB;
        int cx, cy, cL, cA, cB, clabel,cD;
        //imageLAB's  x, y, L,A,B
        int x, y, L, A, B,D;

        //注：这里的图像坐标以左上角为原点，水平向右为x正方向,水平向下为y正方向，与opencv保持一致
        //      从矩阵行列角度看，i表示行，j表示列，即(i,j) = (y,x)
        for (int ck = 0; ck < centers.size(); ++ck)
        {
            cx = centers[ck].x;
            cy = centers[ck].y;
            cL = centers[ck].L;
            cA = centers[ck].A;
            cB = centers[ck].B;
            clabel = centers[ck].label;

            for (int i = cy - len; i < cy + len; i++)
            {
                if (i < 0 | i >= imageLAB.rows) continue;
                //pointer point to the ith row
                imgPtr = imageLAB.ptr<uchar>(i);
                disPtr = DisMask.ptr<double>(i);
                labelPtr = labelMask.ptr<double>(i);
                for (int j = cx - len; j < cx + len; j++)
                {
                    if (j < 0 | j >= imageLAB.cols) continue;
                    L = *(imgPtr + j * 3);
                    A = *(imgPtr + j * 3 + 1);
                    B = *(imgPtr + j * 3 + 2);

                    disc = std::sqrt(pow(L - cL, 2) + pow(A - cA, 2) + pow(B - cB, 2));
                    diss = std::sqrt(pow(j - cx, 2) + pow(i - cy, 2));
                    dis = sqrt(pow(disc, 2) + m * pow(diss, 2));

                    if (dis < *(disPtr + j))
                    {
                        *(disPtr + j) = dis;
                        *(labelPtr + j) = clabel;
                    }//end if
                }//end for
            }
        }//end for (int ck = 0; ck < centers.size(); ++ck)


        return 0;
    }

//input parameters:
//imageLAB:    the source image in Lab color space
//labelMask:    it save every pixel's label
//centers:       clustering center
//len:         the super pixls will be initialize to len*len
//
//output:

    int cluster::updateCenter(cv::Mat &imageLAB, cv::Mat &labelMask,cv::Mat const &Depth,std::vector<center> &centers, int len)
    {
        double *labelPtr = NULL;//labelMask type: 64FC1
        const uchar *imgPtr = NULL;//imageLAB type: 8UC3
        int cx, cy;
        int cD;

        for (int ck = 0; ck < centers.size(); ++ck)
        {
            double sumx = 0, sumy = 0, sumL = 0, sumA = 0, sumB = 0, sumNum = 0,sumD=0;
            cx = centers[ck].x;
            cy = centers[ck].y;
            for (int i = cy - len; i < cy + len; i++)
            {
                if (i < 0 | i >= imageLAB.rows) continue;
                //pointer point to the ith row
                imgPtr = imageLAB.ptr<uchar>(i);
                labelPtr = labelMask.ptr<double>(i);
                for (int j = cx - len; j < cx + len; j++)
                {
                    if (j < 0 | j >= imageLAB.cols) continue;

                    if (*(labelPtr + j) == centers[ck].label)
                    {
                        sumL += *(imgPtr + j * 3);
                        sumA += *(imgPtr + j * 3 + 1);
                        sumB += *(imgPtr + j * 3 + 2);
                        sumx += j;
                        sumy += i;
                        sumNum += 1;
                        cD=static_cast<int>(Depth.at<unsigned short>(i,j));
                        if (cD<0) cD=0;
                        sumD+=cD;
                    }//end if
                }
            }
            //update center
            if (sumNum == 0) sumNum = 0.000000001;
            centers[ck].x = sumx / sumNum;
            centers[ck].y = sumy / sumNum;
            centers[ck].L = sumL / sumNum;
            centers[ck].A = sumA / sumNum;
            centers[ck].B = sumB / sumNum;
            centers[ck].D = sumD / sumNum;


        }//end for

        return 0;
    }


    int cluster::initilizeCenters(cv::Mat &imageLAB,cv::Mat const &imagedepth, std::vector<center> &centers, int len)
    {
        if (imageLAB.empty())
        {
            std::cout << "In itilizeCenters:     image is empty!\n";
            return -1;
        }

        uchar *ptr = NULL;
        center cent;
        int num = 0;
        for (int i = 0; i < imageLAB.rows; i += len)
        {
            cent.y = i + len / 2;
            if (cent.y >= imageLAB.rows) continue;
            ptr = imageLAB.ptr<uchar>(cent.y);
            for (int j = 0; j < imageLAB.cols; j += len)
            {
                cent.x = j + len / 2;
                if ((cent.x >= imageLAB.cols)) continue;
                cent.L = *(ptr + cent.x * 3);
                cent.A = *(ptr + cent.x * 3 + 1);
                cent.B = *(ptr + cent.x * 3 + 2);
                cent.label = ++num;
                cent.D = static_cast<int>(imagedepth.at<unsigned short>(cent.y, cent.x));
                if (cent.D<0) cent.D=0;
                centers.push_back(cent);
            }
        }
        return 0;
    }


//if the center locates in the edges, fitune it's location.
    int cluster::fituneCenter(cv::Mat &imageLAB, cv::Mat &sobelGradient, std::vector<center> &centers)
    {
        if (sobelGradient.empty()) return -1;

        center cent;
        double *sobPtr = sobelGradient.ptr<double>(0);
        uchar *imgPtr = imageLAB.ptr<uchar>(0);
        int w = sobelGradient.cols;
        for (int ck = 0; ck < centers.size(); ck++)
        {
            cent = centers[ck];
            if (cent.x - 1 < 0 || cent.x + 1 >= sobelGradient.cols || cent.y - 1 < 0 || cent.y + 1 >= sobelGradient.rows)
            {
                continue;
            }//end if
            double minGradient = 9999999;
            int tempx = 0, tempy = 0;
            for (int m = -1; m < 2; m++)
            {
                sobPtr = sobelGradient.ptr<double>(cent.y + m);
                for (int n = -1; n < 2; n++)
                {
                    double gradient = pow(*(sobPtr + (cent.x + n) * 3), 2)
                                      + pow(*(sobPtr + (cent.x + n) * 3 + 1), 2)
                                      + pow(*(sobPtr + (cent.x + n) * 3 + 2), 2);
                    if (gradient < minGradient)
                    {
                        minGradient = gradient;
                        tempy = m;//row
                        tempx = n;//column
                    }//end if
                }
            }
            cent.x += tempx;
            cent.y += tempy;
            imgPtr = imageLAB.ptr<uchar>(cent.y);
            centers[ck].x = cent.x;
            centers[ck].y = cent.y;
            centers[ck].L = *(imgPtr + cent.x * 3);
            centers[ck].A = *(imgPtr + cent.x * 3 + 1);
            centers[ck].B = *(imgPtr + cent.x * 3 + 2);

        }//end for
        return 0;
    }


//input parameters:
//image:    the source image in RGB color space
//resultLabel:     it save every pixel's label
//len:         the super pixls will be initialize to len*len
//m:           a parameter witch adjust the weights of diss
//output:

    int cluster::SLIC(cv::Mat const &image,cv::Mat const &image_D, cv::Mat &resultLabel, std::vector<center> &centers, int len, int m)
    {

        int MAXDIS = 999999;
        int height, width;
        height = image.rows;
        width = image.cols;

        //convert color
        cv::Mat imageLAB;
        cv::cvtColor(image, imageLAB, cv::COLOR_BGR2Lab);

        //get sobel gradient map
        cv::Mat sobelImagex, sobelImagey, sobelGradient;
        cv::Sobel(imageLAB, sobelImagex, CV_64F, 0, 1, 3);
        cv::Sobel(imageLAB, sobelImagey, CV_64F, 1, 0, 3);
        cv::addWeighted(sobelImagex, 0.5, sobelImagey, 0.5, 0, sobelGradient);//sobel output image type is CV_64F

        //initiate
        //std::vector<center> centers;
        //disMask save the distance of the pixels to center;
        cv::Mat disMask ;
        //labelMask save the label of the pixels
        cv::Mat labelMask = cv::Mat::zeros(cv::Size(width, height), CV_64FC1);

        //initialize centers,  get centers
        initilizeCenters(imageLAB,image_D,centers, len);
        //if the center locates in the edges, fitune it's location
        fituneCenter(imageLAB, sobelGradient, centers);

        //update cluster 10 times
        for (int time = 0; time < 5; time++)
        {
            //clustering
            disMask = cv::Mat(height, width, CV_64FC1, cv::Scalar(MAXDIS));
            clustering(imageLAB,image_D, disMask, labelMask, centers, len, m);
            //update
            updateCenter(imageLAB, labelMask,image_D, centers, len);
            //fituneCenter(imageLAB, sobelGradient, centers);
        }

        resultLabel = labelMask;

        return 0;
    }
    void cluster::initClusterAssment()//确定vector的数量
    {
        tNode node(-1,-1);
        for(int i=0;i<rowLen;i++)
        {
            clusterAssment.push_back(node);
        }
    }
    void cluster::randCent()//随机种子
    {
        //init centroids
        for(int i=0;i<k;i++)
        {
            center temp=dataSet[rand()%rowLen+1];
            while (temp.D<=0)
                temp=dataSet[rand()%rowLen+1];
            //cout<<"set centroids"<<endl;
            centroids.push_back(temp);
        }
    }
    void cluster::loadDataSet(vector<center> &centers)
    {
        dataSet=centers;
        //init colLen,rowLen
        colLen = 2;
        rowLen = dataSet.size();
    }
    double cluster::distEclud(center &v1 ,center &v2)///输入数据类型为超像素中心点center
    {
        double sum_D = 0,sum_E=0,sum_C=0,sum=0;
        double max_D=20000,max_E=0,max_C;
        max_C= sqrt(255*255+255*255+255*255);
        max_E= sqrt(640*640+480*480);
        sum_D= abs(v2.D-v1.D)/max_D;
        sum_E= sqrt((v1.x - v2.x)*(v1.x - v2.x)+(v1.y - v2.y)*(v1.y - v2.y))/max_E;
        //sum_C= sqrt(pow(v1.L-v2.L,2)+pow(v1.A-v2.A,2)+pow(v1.B-v2.B,2))/max_C;
        sum=1*sum_E+1*sum_D;
        //sum += sqrt((v1.x - v2.x)*(v1.x - v2.x)+(v1.y - v2.y)*(v1.y - v2.y));

        return sum;
    }
    void cluster::kmeans()
    {
        initClusterAssment();
        bool clusterChanged = true;
        //the termination condition can also be the loops less than	some number such as 1000
        while( clusterChanged )
        {
            clusterChanged = false;
            //step one : find the nearest centroid of each point
            //cout<<"find the nearest centroid of each point : "<<endl;
            for(int i=0;i<rowLen;i++)
            {
                int minIndex = -1;
                double minDist = INT_MAX;
                for(int j=0;j<k;j++)
                {
                    double distJI = distEclud( centroids[j],dataSet[i] );
                    if( distJI < minDist )
                    {
                        minDist = distJI;
                        minIndex = j;
                    }
                }
                if( clusterAssment[i].minIndex != minIndex )
                {
                    clusterChanged = true;
                    clusterAssment[i].minIndex = minIndex;
                    clusterAssment[i].minDist = minDist ;
                }
            }

            //step two : update the centroids
            //cout<<"update the centroids:"<<endl;
            for(int cent=0;cent<k;cent++)
            {
                //vector<T> vec(colLen,0);
                center vec;
                int cnt = 0;
                for(int i=0;i<rowLen;i++)
                {
                    if( clusterAssment[i].minIndex == cent )
                    {
                        ++cnt;
                        //sum of two vectors
                        vec.x+= dataSet[i].x;
                        vec.y+= dataSet[i].y;
                        vec.D+= dataSet[i].D;
                        vec.L+= dataSet[i].L;
                        vec.A+= dataSet[i].A;
                        vec.B+= dataSet[i].B;
                    }
                }
                //mean of the vector and update the centroids[cent]

                if( cnt!=0 ){
                    vec.x /= cnt;
                    vec.y /= cnt;
                    vec.D /= cnt;
                    vec.L /= cnt;
                    vec.A /= cnt;
                    vec.B /= cnt;
                }
                centroids[cent]= vec;
            }//for
        }//while
        typename vector<tNode> :: iterator it = clusterAssment.begin();
        int i=0;
        vector<int> instance_label[k];
        while( it!=clusterAssment.end() )
        {
            //cout<<(*it).minIndex<<"\t"<<(*it).minDist<<endl;
            instance_label[(*it).minIndex].push_back(dataSet[i].label);
            it++;
            i++;
        }
        for (int j=0;j<k;j++)
            label.push_back(instance_label[j]);
    }

}