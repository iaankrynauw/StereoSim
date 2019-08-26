#include <sstream>
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/contrib/contrib.hpp>
#include "opencv/cv.h"
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <limits>
#include <sys/time.h>
#include <queue>
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/ocl/ocl.hpp"

using namespace cv;
using namespace std;

struct PointFrameCount{
    Point firstPoint;
    Point center;
    Point prevCenter;
    int FrameCount;   
    float dist;    
    int classification;
    int lives;
    bool detected;
};

int SyncAndCalib();
int videoCVSim();
int videoCVSimImg();
Mat integralImage(Mat &a_image);
Mat SparseDispInterpolation( Mat &a_image, int Threshold, int minxT, int maxxT);
Mat AndBinary( Mat &a_image0, Mat &a_image1);
Mat OrBinary( Mat &a_image0, Mat &a_image1);
Mat diluteBinary(Mat & a_image, int winSize);
Mat erodeBinary(Mat & a_image, int winSize);
Mat NotAndBinary( Mat &a_image0, Mat &a_image1);
Mat CutImage(Mat & a_image, int left, int right, int top, int bot);
Mat Histogram(Mat &a_image, Mat &histOut,bool showHist);
Mat HistogramEq(Mat &a_image, Mat &histogram);
Mat Threshold(Mat &a_image, int min, int max);
void minMaxInt(Mat &a_image, int &_min, int &_max);
Mat Scale(Mat &a_image,int min, int max, bool);
Mat CalculateDisparityMap(Mat &left, Mat &right, Mat &_ThreshMap, int winSize, int minDisp, int maxDisp, int medianSize);
Mat CalculateDisparityMapR2L(Mat &_left, Mat &_right, int winSize, int minDisp, int maxDisp, int medianSize);
Mat CalculateDisparityMapPar(Mat &_left, Mat &_right, Mat &_ThreshMapL, Mat &_ThreshMapR, int winSize, int minDisp, int maxDisp,int medianSize, bool);
Mat AbsoluteDiff(Mat &Image1, Mat &Image2);
Mat Convolution(Mat &img, Mat &kernel,double val);
Mat ConditionalConvolution(Mat &img, Mat &conditionalKernel, Mat &kernel, int alphaThresh, double condVal, double val);
Mat SumAvgImg(Mat &Image1, Mat &Image2 );
Mat LoGKernel(int winSize,double sigma );
void mouseHandler(int event, int x, int y, int flags, void* param );
void minMax(Mat &a_image,double &_min,double &_max );
Mat Scale(Mat &a_image,double max, double min);
Mat Sobel(Mat &a_image);
Mat BinaryMask(Mat &a_image, Mat &mask);
Mat BlobAnalysis(Mat &a_image, Mat &disp, Mat &view, vector<PointFrameCount> &prevDetections);
Mat ParabolaBinaryMask(Mat &a_image);
Mat PasteImageBack(Mat &a_image, Mat &a_image2, int left, int right, int top, int bot);
Mat flipFrame(Mat &a_image);
Mat colourMapFix(Mat &a_image);
Mat RemoveThreshOfNorm(Mat &, Mat &);
Mat outputPrevDet( Mat , vector<PointFrameCount> &);
int testPedestrian();
Mat Laplaciankernel7 = (Mat_<short>(7, 7) << 0, 1, 1, 2, 2, 2, 1, 1, 0,
                                       1, 2, 4, 5, 5, 5, 4, 2, 1,
                                       1, 4, 5, 3, 0, 3, 5, 4, 1,
                                       2, 5, 3, -12, -24, -12, 3, 5, 2,
                                       2, 5, 0, -24, -40, -24, 0, 5, 2,
                                       2, 5, 3, -12, -24, -12, 3, 5, 2,
                                       1, 4, 5, 3, 0, 3, 5, 4, 1,
                                       1, 2, 4, 5, 5, 5, 4, 2, 1,
                                       0, 1, 1, 2, 2, 2, 1, 1, 0 );

Mat Laplaciankernel3 = (Mat_<short>(3, 3) << -1, -2, -1,
                                            -2,  12, -2,
                                            -1, -2, -1);
Mat Gaussiankernel = (Mat_<short>(3, 3) << 1, 2, 1,
                                           2, 4, 2,
                                           1, 2, 1);  

Mat Gaussiankernel5 = (Mat_<short>(5, 5) << 1, 2, 4, 2, 1,
                                           2, 4, 8, 4, 2,
                                           4, 8, 16, 8, 4,
                                           2, 4, 8, 4, 2,
                                           1, 2, 4, 2, 1);  

Mat CondMedianKernel = (Mat_<short>(3, 3) << 1, 1, 1,
                                             1, 1, 1,
                                             1, 1, 1);  
Mat MedianKernel = (Mat_<short>(3, 3) << 1, 1, 1,
                                         1, 0, 1,
                                         1, 1, 1);  
typedef unsigned long long timestamp_t;

static timestamp_t
get_timestamp ()
{
  struct timeval now;
  gettimeofday(&now, NULL);
  return  now.tv_usec + (timestamp_t)now.tv_sec * 1000000;
}

int DistanceMultiplier = 2000;
bool enableTrackbars = false;
bool showLeftRight = false;
bool showDisp = false;
bool showProcessingTime = false;
enum MODE {VIDEO, SEQUENCE, SYNC, PED};
MODE mode = SEQUENCE;
bool useCalibration = false;
int main(int argc, char** argv)
{
    destroyAllWindows();
    namedWindow("Click here and Press a Command", CV_WINDOW_NORMAL);
    resizeWindow("Click here and Press a Command", 800, 300);
    
    cout << "Usage: Press 'a' to start a video or webcam implementation" << endl << "Press 's' to run image sequence application" << endl << "Press 'd' to manually sync video" << endl;
    cout << "Press 'f' to start pedestrian sim" << endl;
    cout << "Settings: Press 'z' to enable trackbars" << endl;
    cout << "Press 'x' to show processing time" << endl;
    cout << "Press 'c' to show build information" << endl;
    cout << "Press 'v' to show Disparity Maps" << endl;
    cout << "Press 'b' to show left right images" << endl;
    cout << "Press 'n' to use calibration files ensure intrinsic.xml and extrinsic.xml is in root directory and image size is set in setting.xml" << endl;
    while(1){        
        int key = waitKey(1);
        switch( key){
            case 'a': mode = VIDEO; cout << "Video" << endl; goto Start; break;
            case 's': mode = SEQUENCE; cout << "Sequence" << endl; goto Start; break;
            case 'd': mode = SYNC; cout << "Sync" << endl; goto Start; break;
            case 'z': enableTrackbars = !enableTrackbars; cout << "enableTrackbars: " << enableTrackbars << endl; break;
            case 'x': showProcessingTime = !showProcessingTime; cout << "showProcessingTime: " << showProcessingTime << endl; break;            
            case 'c': cout << getBuildInformation() << endl; cout << getNumThreads() << endl; break;
            case 'v': showDisp = !showDisp; cout << "showDisp: " << showDisp << endl; break;
            case 'b': showLeftRight = !showLeftRight; cout << "showLeftRight: " << showLeftRight << endl; break;        
            case 'n': useCalibration = !useCalibration; cout << "useCalibration: " << useCalibration << endl; break;        
            case 'q': useCalibration = true; showLeftRight = true; showDisp = true; showProcessingTime = true; enableTrackbars = true; break;
            case 'f': mode = PED; cout << "Pedestrian" << endl; goto Start; break;
        }
    }
    
    Start:
    cout << "Starting" << endl;
    destroyWindow("Click here and Press a Command");
      
    switch( mode){
        case VIDEO: return videoCVSim();break;
        case SEQUENCE: return videoCVSimImg();break;
        case SYNC: return SyncAndCalib();break;
        case PED: return testPedestrian();break;
    }
    
//    

    
}

int SyncAndCalib(){
    bool custom = false;
    restart:
    cout << "Press Enter to grab next frame" << endl << "Press Up arrow to grab only left frame" << endl << "Press Down arrow to grab only right frame" << endl << "Press Left arrow to grab both frames" << endl;
    cout << "Press a for custom video set settings.yml for custom" << endl;
        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
        compression_params.push_back(100);  
        
        int LcutL = 139;
        int LcutR = 213;
        int LcutT = 98;
        int LcutB = 148;           
          
        namedWindow("Trackbars", CV_WINDOW_AUTOSIZE)  ;
        createTrackbar("LcutL", "Trackbars", &LcutL,1000);
        createTrackbar("LcutR", "Trackbars", &LcutR,1000);
        createTrackbar("LcutT", "Trackbars", &LcutT,1000);
        createTrackbar("LcutB", "Trackbars", &LcutB,1000);
        
        // Mat_<double> used here for easy << initialization  
        
        string leftVid, rightVid; 
        int leftFrame, rightFrame;
        if (custom){
            FileStorage fsSettings("settings.yml", FileStorage::READ);
            if( !fsSettings.isOpened()){
                cout << "settings.yml not found exiting" << endl;
                exit(0);
            }
            fsSettings["VideoLDir"] >> leftVid;
            fsSettings["VideoRDir"] >> rightVid;            
            fsSettings["FrameL"] >> leftFrame;
            fsSettings["FrameR"] >> rightFrame;            
            fsSettings.release();
            
            cout << "Custom Video: " << leftVid << " " << rightVid << endl;
            cout << "Frame offset: " << leftFrame << " " << rightFrame << endl;
        }
        

        int imcount = 0;
        VideoCapture capR, capL;
        Mat left, right;
        Restart:
        if(custom){
                capL.open(leftVid); capR.open(rightVid); 
                capL.set(CV_CAP_PROP_POS_FRAMES,capL.get(CV_CAP_PROP_POS_FRAMES)+leftFrame); capR.set(CV_CAP_PROP_POS_FRAMES,capR.get(CV_CAP_PROP_POS_FRAMES)+rightFrame);
        } else {        
            capL.open("calibR3.MP4"); capR.open("calibL3.MP4");
        
            //calib3
            capL.set(CV_CAP_PROP_POS_FRAMES,capL.get(CV_CAP_PROP_POS_FRAMES) + 1687);   
            capR.set(CV_CAP_PROP_POS_FRAMES,capR.get(CV_CAP_PROP_POS_FRAMES) + 1509); 
        }
        
        int changeLR = 0;
        
        if ( changeLR == 0){
            capR >> right;
        }
        if ( changeLR == 1){
            capL >> left;
        }        
        int frameCountL = 0;                           
        int frameCountR = 0;                           
        Restart3:
        while(1){

//            timestamp_t t0 = get_timestamp();    
            if ( changeLR == 0){
                capL >> left;
            }
            if ( changeLR == 1){
                capR >> right;
            }
            if ( changeLR == 2){
                capL >> left;
                capR >> right;
            }
                          
            if(left.empty() || right.empty()) goto Restart3;
            
            if ( changeLR == 0){
                frameCountL++;
   //            cvtColor( left, left, CV_BGR2GRAY);
   //            Mat leftTemp;
   //            undistort(left,leftTemp, cameraMatrix1, distCoeffs1);
   //            remap(leftTemp, left, map11, map12, INTER_LINEAR);
            }

           
            if ( changeLR == 1){
                frameCountR++;
        //        cvtColor( right, right, CV_BGR2GRAY); 
        //        Mat rightTemp;
        //        undistort(right,rightTemp, cameraMatrix2, distCoeffs2);
        //        remap(rightTemp, right, map21, map22, INTER_LINEAR);      
            }  
            
            if ( changeLR == 2){
                frameCountL++;
                frameCountR++;
//               cvtColor( left, left, CV_BGR2GRAY);
//               Mat leftTemp;
//               undistort(left,leftTemp, cameraMatrix1, distCoeffs1);
//               remap(leftTemp, left, map11, map12, INTER_LINEAR);
//               cvtColor( right, right, CV_BGR2GRAY); 
//               Mat rightTemp;
//               undistort(right,rightTemp, cameraMatrix2, distCoeffs2);
//               remap(rightTemp, right, map21, map22, INTER_LINEAR);                 
            }            
                
//            left = flipFrame(left);
//            right = flipFrame(right);
//            
//            left = CutImage(left, LcutL < left.cols ? LcutL : left.cols-1, LcutR < left.cols ? LcutR : left.cols-1, LcutT < left.rows ? LcutT : left.rows-1, LcutB < left.rows ? LcutB : left.rows-1);
//            right = CutImage(right, LcutL < left.cols ? LcutL : left.cols-1, LcutR < left.cols ? LcutR : left.cols-1, LcutT < left.rows ? LcutT : left.rows-1, LcutB < left.rows ? LcutB : left.rows-1);
       
            imshow("left", left);                           
            imshow("right", right);                           
            
//            cvSetMouseCallback("left",mouseHandler,&left);                 
//            if( capL.get(CV_CAP_PROP_FRAME_COUNT) == frameCount){
//                capL.release();
//            }
//            timestamp_t t1 = get_timestamp();
//            double secs0 = (t1 - t0) / 1000000.0L;    
//            int waitTime = 100 - secs0*1000;
//            if (waitTime <= 0) waitTime = 1;
            cout << frameCountL << " " << frameCountR << endl;
            while(1){                    
                int key = waitKey(33);
                if( key == 13) break;
                ostringstream sstream3, sstream4;              
                switch(key){
                    case 'a' : custom = true; goto restart;
                    case 8 :
                        sstream3 << "left" << setfill('0') << setw(2) <<  imcount << ".jpg";
                        imwrite(sstream3.str(),left,compression_params);
                        sstream4 << "right" << setfill('0') << setw(2) <<  imcount << ".jpg";
                        imwrite(sstream4.str(),right,compression_params);
                        imcount++; break;
                    case 2490368  : changeLR = 0; break;
                    case 2621440  : changeLR = 1; break;
                    case 2424832  : changeLR = 2; break;
                    case 'x'  : main(0,0); break;
                    case 'r'  : goto Restart; break;
////                    case 2555904  : goto End; break;
//                    case 32 : goto Restart; break;
                }
            }                
            
  
        }
    return 0;
}

int X_MIN0 = 216;
int X_MIN1 = 0;        
int X_MAX0 = 20;
int X_MAX1 = 75;

//int X_MIN0 = 256;
//int X_MIN1 = 0;        
//int X_MAX0 = 20;
//int X_MAX1 = 75;
enum VID { HIGHWAY, INTERSECTION, RURAL, RURAL2, RURAL3, RURAL4, WEBCAM, CUSTOM};
enum STEREO { SBM, SGBM, SBMGPU};
int videoCVSim(){
  try{

      cout << "Press 'a' for video, 's' for Webcam and 'd' custom video set settings.yml for custom, program defaults to default video driveL/R.mp4" << endl;      
             
//        int SADWinSize = 5;
//        int nDisp = 64;
//        int preFilterCap = 4;
//        int mDisp = 64;
//        int unqRatio = 47;
//        int speckWinSize = 150;
//        int speckRange = 2;
//        int disp12MaxDiff = 10;
//        int fullDp = 0;
//        int p1 = 719;
//        int p2 = 3459;
        
//        int SADWinSize = 22;
//        int nDisp = 70;
//        int preFilterCap = 4;
//        int mDisp = 42;
//        int unqRatio = 40;
//        int speckWinSize = 0;
//        int speckRange = 0;
//        int disp12MaxDiff = 10;
//        int fullDp = 0;
//        int p1 = 0;
//        int p2 = 0;

        int SADWinSize = 22;
        int nDisp = 70;
        int preFilterCap = 0;
        int mDisp = 0;
        int unqRatio = 0;
        int speckWinSize = 0;
        int speckRange = 0;
        int disp12MaxDiff = 0;
        int fullDp = 0;
        int p1 = 1250;
        int p2 = 3080;
            
        
//        int cutL = 279;
//        int cutR = 566;
//        int cutT = 70;
//        int cutB = 101;           
        
        int cutL;
        int cutR;
        int cutT;
        int cutB;    
        
//        int cutL = 155;
//        int cutR = 527;
//        int cutT = 62;
//        int cutB = 411;
        
        if( enableTrackbars){
            namedWindow("Trackbars", CV_WINDOW_FREERATIO);

            createTrackbar("SADWinSize", "Trackbars", &SADWinSize,256);
            createTrackbar("nDisp", "Trackbars", &nDisp,256);
            createTrackbar("preFilterCap", "Trackbars", &preFilterCap,256);
            createTrackbar("mDisp", "Trackbars", &mDisp,256);
            createTrackbar("unqRatio", "Trackbars", &unqRatio,256);
            createTrackbar("speckWinSize", "Trackbars", &speckWinSize,256);
            createTrackbar("speckRange", "Trackbars", &speckRange,256);
            createTrackbar("disp12MaxDiff", "Trackbars", &disp12MaxDiff,256);
            createTrackbar("fullDp", "Trackbars", &fullDp,1);
            createTrackbar("p1", "Trackbars", &p1,5000);
            createTrackbar("p2", "Trackbars", &p2,5000);
            createTrackbar("DistanceMultiplier", "Trackbars", &DistanceMultiplier,5000);

            createTrackbar("X_Min0", "Trackbars", &X_MIN0,256);
            createTrackbar("X_Max0", "Trackbars", &X_MAX0,256);

            createTrackbar("X_Max1", "Trackbars", &X_MAX1,256);
            createTrackbar("X_Min1", "Trackbars", &X_MIN1,256);

            createTrackbar("cutL", "Trackbars", &cutL,1000);
            createTrackbar("cutR", "Trackbars", &cutR,1000);
            createTrackbar("cutT", "Trackbars", &cutT,1000);
            createTrackbar("cutB", "Trackbars", &cutB,1000);
        }
        
        VID video = HIGHWAY;
        STEREO stereoAlg = SGBM;
        bool custom = false;
        bool lr = true;
        bool lrS = false;
        Restart:  
        Rect ROI[2];
        if( useCalibration){
            FileStorage fsSettings("roi.yml", FileStorage::READ);
            if( !fsSettings.isOpened()){
                cout << "roi.yml not found exiting" << endl;
                exit(0);
            }
           
            fsSettings["ROI1"] >> ROI[0];
            fsSettings["ROI2"] >> ROI[1];            
            fsSettings.release();
            
            cutL = 0;
            cutR = 0;
            cutT = 0;
            cutB = 0;
        } else{
            cutL = 558;
            cutR = 209;
            cutT = 326;
            cutB = 147; 
        }
        string leftVid, rightVid; 
        int leftFrame, rightFrame;
        if (custom){
            FileStorage fsSettings("settings.yml", FileStorage::READ);
            if( !fsSettings.isOpened()){
                cout << "settings.yml not found exiting" << endl;
                exit(0);
            }
            fsSettings["VideoLDir"] >> leftVid;
            fsSettings["VideoRDir"] >> rightVid;            
            fsSettings["FrameL"] >> leftFrame;
            fsSettings["FrameR"] >> rightFrame;            
            fsSettings.release();
            
            cout << "Custom Video: " << leftVid << " " << rightVid << endl;
            cout << "Frame offset: " << leftFrame << " " << rightFrame << endl;
        }
        VideoCapture capR, capL;
        switch(video){
            case HIGHWAY  : capR.open("driveL.mp4"); capL.open("driveR.mp4"); break;                                             
            case CUSTOM : capR.open(leftVid); capL.open(rightVid); 
                capL.set(CV_CAP_PROP_POS_FRAMES,capL.get(CV_CAP_PROP_POS_FRAMES)+rightFrame); capR.set(CV_CAP_PROP_POS_FRAMES,capR.get(CV_CAP_PROP_POS_FRAMES)+leftFrame);   break;
            case WEBCAM : capR.open(0); capL.open(1);  break;
        }
        if (!capR.isOpened() || !capL.isOpened()){
            cout << "Videos or Webcam not found returning to highway dataset driveL/R.mp4" << endl;
            if( video == HIGHWAY){
                cout << "Default video not found exiting program" << endl;
                exit(0);
            }
            video = HIGHWAY;
            goto Restart;
        }
        Mat left, right, prevLeft, prevRight;
        
//        // Mat_<double> used here for easy << initialization
//        Mat_<double> cameraMatrix1P(3,3); // 3x3 matrix
//        Mat_<double> distCoeffs1P(5,1);   // 5x1 matrix for five distortion coefficients
//        Mat_<double> cameraMatrix2P(3,3); // 3x3 matrix
//        Mat_<double> distCoeffs2P(5,1);   // 5x1 matrix
        
        
//        cameraMatrix1P << 5.9470580179854539e+002, 0., 6.3950000000000000e+002, 0.,
//            5.9470580179854539e+002, 3.5950000000000000e+002, 0., 0., 1.;    
//        distCoeffs1P << -2.7192233385526449e-001, 1.1866907518122856e-001, 0., 0.,
//            -3.1976495611086120e-002;    
//        cameraMatrix2P << 5.9122607972334174e+002, 0., 6.3950000000000000e+002, 0.,
//            5.9122607972334174e+002, 3.5950000000000000e+002, 0., 0., 1.;    
//        distCoeffs2P << -2.6381849285291881e-001, 1.0662036550461518e-001, 0., 0.,
//            -2.6615029148327581e-002; 
        
        // Mat_<double> used here for easy << initialization
        Mat_<double> cameraMatrix1(3,3); // 3x3 matrix
        Mat_<double> distCoeffs1(5,1);   // 5x1 matrix for five distortion coefficients
        Mat_<double> cameraMatrix2(3,3); // 3x3 matrix
        Mat_<double> distCoeffs2(5,1);   // 5x1 matrix
        
        Mat_<double> R1(3,3);
        Mat_<double> R2(3,3);   
        Mat_<double> P1(3,4);
        Mat_<double> P2(3,4);   
        int w,h;
        
        Mat map11, map12, map21, map22;        
        if( useCalibration){
            
            FileStorage fsIntrinsic("intrinsics.yml", FileStorage::READ);
            if( !fsIntrinsic.isOpened()){
                cout << "intrinsic.yml not found exiting" << endl;
                exit(0);
            }
            fsIntrinsic["M1"] >> cameraMatrix1;
            fsIntrinsic["D1"] >> distCoeffs1;
            fsIntrinsic["M2"] >> cameraMatrix2;
            fsIntrinsic["D2"] >> distCoeffs2;
            
            FileStorage fsExtrinsic("extrinsics.yml", FileStorage::READ);
            if( !fsExtrinsic.isOpened()){
                cout << "extrinsics.yml not found exiting" << endl;
                exit(0);
            }            
            fsExtrinsic["R1"] >> R1;
            fsExtrinsic["R2"] >> R2;
            fsExtrinsic["P1"] >> P1;
            fsExtrinsic["P2"] >> P2;
            
            FileStorage fsSettings("settings.yml", FileStorage::READ);
            if( !fsSettings.isOpened()){
                cout << "settings.yml not found exiting" << endl;
                exit(0);
            }   
            Mat_<double> mImgSize(1,2);
            fsSettings["ImgSize"] >> mImgSize;

            fsIntrinsic.release();
            fsExtrinsic.release(); 
            fsSettings.release();
            
            if( custom || video == WEBCAM){
                w = (int) mImgSize.at<double>(0,0);
                h = (int) mImgSize.at<double>(0,1);
            } else {
                w = 1348;
                h = 374;
            }
            initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, Size( w, h), CV_16SC2, map11, map12);
            initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, Size( w, h), CV_16SC2, map21, map22);            
        }
        int frameCount = 0;
        vector<PointFrameCount> prevDet;
        Mat disp8, colourMap;
        while(1){

            timestamp_t t0 = get_timestamp();     
            if(lr){
                capL >> left;
                capR >> right;            
            } else{
                capL >> right;
                capR >> left;                            
            }
            
            if( left.empty()){
                goto Restart;
            }
            
            cvtColor( left, left, CV_BGR2GRAY);
            cvtColor( right, right, CV_BGR2GRAY);           
            
        if( useCalibration){
            Mat leftTemp = left.clone();
            Mat rightTemp = right.clone();
            
            if ( video != WEBCAM){
                if ( left.cols != w || left.rows != h){
                    cout << "ImgSize in settings.yml is set incorrectly, exiting program" << endl;
                    exit(0);
                }   
            } 
                
            
//            undistort(leftTemp,left, cameraMatrix1P, distCoeffs1P);
//            undistort(rightTemp,right, cameraMatrix2P, distCoeffs2P);
//            
//            leftTemp = left.clone();
//            rightTemp = right.clone();
            
            
            remap(leftTemp, left, map11, map12, INTER_LINEAR);
            remap(rightTemp, right, map21, map22, INTER_LINEAR);  
            
            if(custom || video == WEBCAM){
                
                Rect interesect  = ROI[0] & ROI[1];    
//                cout << interesect.x << " " << interesect.y << " " << interesect.width << " " << interesect.height << endl;
//                cout << left.cols << " " << left.rows << endl;
                left = left(interesect);
                right = right(interesect);

            }
            
        }            
            Mat largeLeft = left.clone(); 
            if( cutL + cutR > left.cols){
                cutL = 0;
                cutR = 0;
            }
            if( cutT + cutB > left.rows){
                cutT = 0;
                cutB = 0;
            }
            
            left = CutImage(left, cutL, cutR, cutT, cutB);
            right = CutImage(right, cutL, cutR, cutT, cutB);
                        
            if (++frameCount % 2 != 0){
                StereoSGBM sgbm;      
                StereoBM sbm;
                if( stereoAlg == SBM){            
                    sbm.init(CV_STEREO_BM_XSOBEL, (int) (nDisp%16 == 0 ? nDisp : (nDisp + (16 - nDisp%16))), SADWinSize%2 == 0 ? SADWinSize++ : SADWinSize);
                    sbm.state->preFilterSize = 5;
                    sbm.state->preFilterCap = 61;
                    sbm.state->minDisparity = (mDisp == 0 ? 1 : -1*mDisp);
                    sbm.state->textureThreshold = p1;
                    sbm.state->uniquenessRatio = unqRatio;
                    sbm.state->speckleWindowSize = speckWinSize;
                    sbm.state->speckleRange = speckRange;
                    sbm.state->disp12MaxDiff = disp12MaxDiff;                    
                } else if( stereoAlg == SGBM){

            

                    sgbm.SADWindowSize = SADWinSize;
                    sgbm.numberOfDisparities = (int) (nDisp%16 == 0 ? nDisp : (nDisp + (16 - nDisp%16)));
                    sgbm.preFilterCap = preFilterCap;
                    sgbm.minDisparity = -1*mDisp;
                    sgbm.uniquenessRatio = unqRatio;
                    sgbm.speckleWindowSize = speckWinSize;
                    sgbm.speckleRange = speckRange;
                    sgbm.disp12MaxDiff = disp12MaxDiff;
                    sgbm.fullDP = fullDp;
                    sgbm.P1 = p1;
                    sgbm.P2 = p2;
                } 
                if(nDisp%8 != 0){
                    nDisp = ((int) nDisp/8)*8;
                }
                if(SADWinSize%2 != 1) SADWinSize++;
                ocl::StereoBM_OCL bm(1,nDisp,SADWinSize);
                
                Mat disp, disp8;
                switch(stereoAlg){
                    case SGBM: 
                        if(lrS) 
                            sgbm(left, right, disp);
                        else
                            sgbm(right, left, disp);
                        normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8UC1);
                        break;
                    case SBM: 
                        if(lrS) 
                            sbm(left, right, disp); 
                        else
                            sbm(right, left, disp); 
                        normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8UC1);
                        break;
                    case SBMGPU:   
                        ocl::oclMat leftOcl(left),rightOcl(right),dispOcl;   
                        if( lrS){
                            bm.operator ()(leftOcl, rightOcl, dispOcl);                            
                        }
                        else{
                            bm.operator ()(rightOcl, leftOcl, dispOcl);
                        }            
                        
                        dispOcl.download(disp8); 
                        normalize(disp8, disp8, 0, 255, CV_MINMAX, CV_8UC1);
                        break;
                }
                Mat dispT = Threshold(disp8, 2, 244);
                dispT = erodeBinary(dispT, 3);
                dispT = erodeBinary(dispT, 3);
                dispT = erodeBinary(dispT, 3);
                dispT = diluteBinary(dispT, 7);
                Mat parMask = ParabolaBinaryMask(dispT);
                dispT = BinaryMask(dispT, parMask);
//                Mat borders = BlobAnalysis( dispT, disp8, left, prevDet);
                Mat borders(left);
                Mat output = PasteImageBack(largeLeft, borders, cutL < left.cols ? cutL : left.cols-1, cutR < left.cols ? cutR : left.cols-1, cutT < left.rows ? cutT : left.rows-1, cutB < left.rows ? cutB : left.rows-1);
                colourMap;
                applyColorMap(disp8, colourMap, COLORMAP_RAINBOW);  
                colourMap = colourMapFix(colourMap);
                imshow("output", output);
//                imshow("parMask", parMask);
                if (showDisp){
                    imshow("disp8", disp8);                    
                    imshow("colourMap", colourMap);                    
                    imshow("dispT", dispT);
                }
                if(showLeftRight){
                    imshow("left", left);  
                    imshow("right", right);                           
                }
                
    //            imshow("borders", borders);                           

    //            imshow("gradL",gradL);
    //            imshow("gradR",gradR);
        } else {
                Mat borders = outputPrevDet( left,prevDet);
                Mat output = PasteImageBack(largeLeft, borders, cutL < left.cols ? cutL : left.cols-1, cutR < left.cols ? cutR : left.cols-1, cutT < left.rows ? cutT : left.rows-1, cutB < left.rows ? cutB : left.rows-1);
                imshow("output", output);
//                imshow("parMask", parMask);
                imshow("disp8", disp8);              
                imshow("colourMap", colourMap);
    //            imshow("dispT", dispT);
                imshow("left", left);                           
                imshow("right", right);                 
        }

     
            frameCount++;
            if( capL.get(CV_CAP_PROP_FRAME_COUNT) == frameCount){
                capL.release();
                capR.release();
                goto Restart;
            }
            timestamp_t t1 = get_timestamp();
            double secs0 = (t1 - t0) / 1000000.0L;  
            if(showProcessingTime)
                cout << secs0 << endl;
            int waitTime = 100 - secs0*1000;
            if (waitTime <= 0) waitTime = 1;
            int key = waitKey( waitTime);
            if( key == 13) break;
            switch(key){
                case 'a'  : custom = false;video = HIGHWAY; goto Restart; break;              
                case 's'  : custom = false;video = WEBCAM; goto Restart; break;
                case 'd'  : custom = true;video = CUSTOM; goto Restart; break;
//                case 'f'  : capL.set(CV_CAP_PROP_POS_FRAMES,capL.get(CV_CAP_PROP_POS_FRAMES)-5); capR.set(CV_CAP_PROP_POS_FRAMES,capR.get(CV_CAP_PROP_POS_FRAMES)-5); break;
                case 32 : goto Restart; break;
                case 'x'  : main(0,0); break;
                case 'i'  : cout << "SBM" << endl; stereoAlg = SBM; break;
                case 'o'  : cout << "SGBM" << endl; stereoAlg = SGBM; break;
                case 'p'  : cout << "SBMGPU" << endl; stereoAlg = SBMGPU; break;
                case 'l'  : cout << "Left Right Switch" << endl;lr = !lr; break;
                case 'k'  : cout << "Left Right Switch Stereo" << endl;lrS = !lrS; break;
                case 'q'  : frameCount = frameCount + 200;capL.set(CV_CAP_PROP_POS_FRAMES,capL.get(CV_CAP_PROP_POS_FRAMES)+200); capR.set(CV_CAP_PROP_POS_FRAMES,capR.get(CV_CAP_PROP_POS_FRAMES)+200); break;
            }
        }
    } catch( Exception & e){
        while(1){
            if( waitKey (10) == 8) break;
        }
    }
    return 0; 
}

int Kmax = 105;
int Kmin = 5;
int X_3 = 100;
int Bc = 16;
int Ac = 71;
int Cc = 95;
int testPedestrian(){
    vector<PointFrameCount> prevDet;
    namedWindow("Trackbars", CV_WINDOW_AUTOSIZE);
    createTrackbar("Kmax", "Trackbars", &Kmax,256);
    createTrackbar("Kmin", "Trackbars", &Kmin,256);    
    createTrackbar("X_3", "Trackbars", &X_3,256);    
    createTrackbar("X_4", "Trackbars", &Bc,256);
    createTrackbar("X_5", "Trackbars", &Ac,256);    
    createTrackbar("X_6", "Trackbars", &Cc,256);    
    while(1){
        Mat disp8 = imread("disp.png", 0);
//        Mat disp8 = imread("disp2.png", 0);
        Mat left = disp8.clone();
        Mat dispT = Threshold(disp8, 2, 244);
        dispT = erodeBinary(dispT, 3);
        dispT = erodeBinary(dispT, 3);
        dispT = erodeBinary(dispT, 3);
        dispT = diluteBinary(dispT, 7);
        Mat parMask = ParabolaBinaryMask(dispT);
        dispT = BinaryMask(dispT, parMask);

        Mat borders = BlobAnalysis( dispT, disp8, left, prevDet);    

        imshow("left", left);
        imshow("borders", borders);
        imshow("dispT", dispT);
        imshow("disp8", disp8);

        waitKey(100); 
//        cvSetMouseCallback("disp8",mouseHandler,&disp8);     
    }
    return 0;
}

int videoCVSimImg(){
  try{

        

      cout << "Press up arrow for Intersection dataset and Down arrow for Highway dataset" << endl;
        int SADWinSize = 5;
        int nDisp = 32;
        int preFilterCap = 40;
        int mDisp = 0;
        int unqRatio = 0;
        int speckWinSize = 0;
        int speckRange = 0;
        int disp12MaxDiff = 0;
        int fullDp = 0;
        int p1 = 0;
        int p2 = 0;
            
        int cutL = 279;
        int cutR = 566;
        int cutT = 70;
        int cutB = 101;           
        
        if( enableTrackbars){
            namedWindow("Trackbars", CV_WINDOW_FREERATIO);
            createTrackbar("SADWinSize", "Trackbars", &SADWinSize,256);
            createTrackbar("nDisp", "Trackbars", &nDisp,256);
            createTrackbar("preFilterCap", "Trackbars", &preFilterCap,256);
            createTrackbar("mDisp", "Trackbars", &mDisp,256);
            createTrackbar("unqRatio", "Trackbars", &unqRatio,256);
            createTrackbar("speckWinSize", "Trackbars", &speckWinSize,256);
            createTrackbar("speckRange", "Trackbars", &speckRange,256);
            createTrackbar("disp12MaxDiff", "Trackbars", &disp12MaxDiff,256);
            createTrackbar("fullDp", "Trackbars", &fullDp,1);
            createTrackbar("p1", "Trackbars", &p1,5000);
            createTrackbar("p2", "Trackbars", &p2,5000);

            createTrackbar("X_Min0", "Trackbars", &X_MIN0,256);
            createTrackbar("X_Max0", "Trackbars", &X_MAX0,256);

            createTrackbar("X_Max1", "Trackbars", &X_MAX1,256);
            createTrackbar("X_Min1", "Trackbars", &X_MIN1,256);

            createTrackbar("cutL", "Trackbars", &cutL,1000);
            createTrackbar("cutR", "Trackbars", &cutR,1000);
            createTrackbar("cutT", "Trackbars", &cutT,1000);
            createTrackbar("cutB", "Trackbars", &cutB,1000);
            createTrackbar("DistanceMultiplier", "Trackbars", &DistanceMultiplier,5000);
        }
        VID video = HIGHWAY;
        Restart:                      
        Mat left, right;
        
        int frameCount = 0;
        vector<PointFrameCount> prevDet;
        Mat disp8, colourMap;
        float totalTime = 0;
        while(1){

            timestamp_t t0 = get_timestamp(); 
            ostringstream sstreamL1, sstreamR1;    
            switch(video){                
               case HIGHWAY  : sstreamL1 << "G:/Saves/Saves/l" << setfill('0') << setw(5) <<  frameCount << ".jpg";
                               sstreamR1 << "G:/Saves/Saves/r" << setfill('0') << setw(5) <<  frameCount << ".jpg"; break;
               case INTERSECTION: sstreamL1 << "G:/Saves/Saves2/l" << setfill('0') << setw(5) <<  frameCount << ".jpg";
                                  sstreamR1 << "G:/Saves/Saves2/r" << setfill('0') << setw(5) <<  frameCount << ".jpg"; break;
            }
                
   

            left = imread(sstreamL1.str());
            right = imread(sstreamR1.str());
            
            if( left.empty()){
                frameCount = 0;
                goto Restart;
            }  
            
            Mat largeLeft = left.clone(); 
            
            cvtColor( left, left, CV_BGR2GRAY);
            cvtColor( right, right, CV_BGR2GRAY);           
            
                       
            
//            left = CutImage(left, cutL < left.cols ? cutL : left.cols-1, cutR < left.cols ? cutR : left.cols-1, cutT < left.rows ? cutT : left.rows-1, cutB < left.rows ? cutB : left.rows-1);
//            right = CutImage(right, cutL < right.cols ? cutL : right.cols-1, cutR < right.cols ? cutR : right.cols-1, cutT < right.rows ? cutT : right.rows-1, cutB < right.rows ? cutB : right.rows-1);                  
                        
//            if (1 == 1 ){//frameCount % 1 == 0){
                StereoBM sbm;
                sbm.init(CV_STEREO_BM_XSOBEL, (int) (nDisp%16 == 0 ? nDisp : (nDisp + (16 - nDisp%16))), SADWinSize);


            
                StereoSGBM sgbm;
                sgbm.SADWindowSize = SADWinSize;
                sgbm.numberOfDisparities = (int) (nDisp%16 == 0 ? nDisp : (nDisp + (16 - nDisp%16)));
                sgbm.preFilterCap = preFilterCap;
                sgbm.minDisparity = -1*mDisp;
                sgbm.uniquenessRatio = unqRatio;
                sgbm.speckleWindowSize = speckWinSize;
                sgbm.speckleRange = speckRange;
                sgbm.disp12MaxDiff = disp12MaxDiff;
                sgbm.fullDP = fullDp;
                sgbm.P1 = p1;
                sgbm.P2 = p2;

                Mat disp;
                disp8;
                sgbm(right, left, disp);
                timestamp_t td = get_timestamp();
    //            sbm(left, right, disp);
                normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8UC1);              
//                Mat dispT = Threshold(disp8, 2, 244);
//                dispT = erodeBinary(dispT, 3);
//                dispT = erodeBinary(dispT, 3);
//                dispT = erodeBinary(dispT, 3);
//                dispT = diluteBinary(dispT, 7);
//                Mat parMask = ParabolaBinaryMask(dispT);
//                dispT = BinaryMask(dispT, parMask);
//                Mat borders = BlobAnalysis( dispT, disp8, left, prevDet);
//                Mat output = PasteImageBack(largeLeft, borders, cutL < left.cols ? cutL : left.cols-1, cutR < left.cols ? cutR : left.cols-1, cutT < left.rows ? cutT : left.rows-1, cutB < left.rows ? cutB : left.rows-1);
                colourMap;
                applyColorMap(disp8, colourMap, COLORMAP_RAINBOW);  
                colourMap = colourMapFix(colourMap);
//                imshow("output", output);
                if(showDisp){
                    imshow("colourMap", colourMap);
//                    imshow("dispT", dispT);
                }
                if (showLeftRight){
                    imshow("left", left);                           
                    imshow("right", right);                           
                }

            cvSetMouseCallback("disp8",mouseHandler,&disp8);     
            frameCount++;

            timestamp_t t1 = get_timestamp();            
            double secs0 = (t1 - t0) / 1000000.0L;  
            double secsd = (td - t0) / 1000000.0L;  
            totalTime = totalTime + secs0;
            ostringstream sstreamTime;
            sstreamTime << (double) secs0 << " " << (double) secsd << " " << (double) secs0 - secsd << " TT: " << (double) totalTime;
            int waitTime = 100 - secs0*1000;
            if (showProcessingTime){
                cout << sstreamTime.str() << endl;
                cout << waitTime << endl;
                cout << "FC: " << frameCount << endl;
            }
            
            if (waitTime <= 0) waitTime = 1;
            int key = waitKey(waitTime);
            if( key == 13) break;
            switch(key){
                case 2490368  : video = HIGHWAY; goto Restart; break;
                case 2621440  : video = INTERSECTION; goto Restart; break;
                case 32 : goto Restart; break;
                case 'x'  : main(0,0); break;
            }
        }
    } catch( Exception & e){
        while(1){
            if( waitKey (10) == 8) break;
        }
    }
    return 0; 
}

Mat erodeBinary( Mat & a_image, int winSize){
    
    Mat kernel = Mat( winSize, winSize, CV_16SC1, 1.0);
    Mat conv = Convolution( a_image, kernel, 1.0);
    return Threshold( conv, 254, 255);    
}

Mat AndBinary( Mat &a_image0, Mat &a_image1){
    
    Mat out = Mat( a_image0.rows, a_image0.cols, CV_8UC1,0.0);
    for(int i = 0; i < a_image0.cols; i++){
        for(int j = 0; j < a_image0.rows; j++){
            if((a_image0.at<uchar>(j, i) == 255)  && (a_image1.at<uchar>(j, i) == 255))
                out.at<uchar>(j, i) = 255;
        }
    }
    return out;
}

Mat OrBinary( Mat &a_image0, Mat &a_image1){
    
    Mat out = Mat( a_image0.rows, a_image0.cols, CV_8UC1,0.0);
    for(int i = 0; i < a_image0.cols; i++){
        for(int j = 0; j < a_image0.rows; j++){
            if((a_image0.at<uchar>(j, i) == 255)  || (a_image1.at<uchar>(j, i) == 255))
                out.at<uchar>(j, i) = 255;
        }
    }
    return out;
}

Mat NotAndBinary( Mat &a_image0, Mat &a_image1){
    
    Mat out = Mat( a_image0.rows, a_image0.cols, CV_8UC1,0.0);
    for(int i = 0; i < a_image0.cols; i++){
        for(int j = 0; j < a_image0.rows; j++){
            if(a_image0.at<uchar>(j, i) == 255 && a_image1.at<uchar>(j, i) != 255)            
                out.at<uchar>(j, i) = 255;            
        }
    }
    return out;
}

Mat diluteBinary( Mat & a_image, int winSize){

    if (winSize%2 == 0) winSize++;
    int hWin = (winSize-1)/2;
    Mat out = Mat( a_image.rows, a_image.cols, CV_8UC1,0.0);
    int accumulator;
    for(int i = hWin; i < a_image.cols -hWin; i++){
        for(int j = hWin; j < a_image.rows -hWin; j++){
            accumulator = 0;
            for(int k = -hWin; k <= hWin; k++){
                for(int l = -hWin; l <= hWin; l++){
                    if(a_image.at<uchar>(l + j, i + k) == 255) accumulator++;                    
                }
            }
            if( accumulator > 0.2*winSize*winSize )
                out.at<uchar>(j,i) = 255;
        }
    }

    return out;    
}

Mat CutImage(Mat & a_image, int left, int right, int top, int bot){
    
    Mat out = Mat( a_image.rows -top -bot, a_image.cols - left - right, a_image.type(), 0.0);
    
    if (a_image.type() == CV_8UC1){
        for (int i = top; i < a_image.rows - bot; i++) {
            for (int j = left; j < a_image.cols - right ; j++) { 
                out.at<uchar>(i - top,j - left) = a_image.at<uchar>(i,j);
            }
        }
    } else if (a_image.type() == CV_8UC3){        
        for (int i = top; i < a_image.rows - bot; i++) {
            for (int j = left; j < a_image.cols - right ; j++) { 
                Vec3b pix = a_image.at<Vec3b>(i,j);
                out.at<Vec3b>(i - top,j - left)[0] = pix[0];
                out.at<Vec3b>(i - top,j - left)[1] = pix[1];
                out.at<Vec3b>(i - top,j - left)[2] = pix[2];
            }
        }
    } else if (a_image.type() == CV_16SC1){
        for (int i = top; i < a_image.rows - bot; i++) {
            for (int j = left; j < a_image.cols - right ; j++) { 
                out.at<short>(i - top,j - left) = a_image.at<short>(i,j);
            }
        }
    }
    
    return out;    
}

Mat AbsoluteDiff(Mat &Image1, Mat &Image2){
    Mat out = Mat(Image1.rows,Image1.cols,Image1.type(), 0.0);
    uchar* Image1Data = Image1.data;
    uchar* Image2Data = Image2.data;
    uchar* outData = out.data;
    for (int i = 0; i < Image1.rows; i++) {
        for (int j = 0; j < Image1.cols; j++) {  
                outData[i*out.step+j] = abs(Image1Data[i*Image1.step+j]-Image2Data[i*Image2.step+j]);            
                
        }
    }
    return out;
}

Mat SumAvgImg(Mat &Image1, Mat &Image2){
    Mat out = Mat(Image1.rows,Image1.cols,Image1.type(), 0.0);
    uchar* Image1Data = Image1.data;
    uchar* Image2Data = Image2.data;
    uchar* outData = out.data;
    for (int i = 0; i < Image1.rows; i++) {
        for (int j = 0; j < Image1.cols; j++) {  
            outData[i*out.step+j] = (Image1Data[i*Image1.step+j]+Image2Data[i*Image2.step+j])/2;
        }
    }
    return out;
}
    
Mat LoGKernel(int winSize,double sigma){
    
    Mat kernel(winSize,winSize,CV_64F, 0.0);    
    double hWin = (winSize-1)/2.0; 
    for (int i = 0; i < kernel.rows ; i++){
      for (int j = 0; j < kernel.cols ; j++){ 
            double x = j - hWin;
            double y = i - hWin;           
            kernel.at<double>(j,i) = (1.0 /(M_PI*pow(sigma,4))) * (1 - (x*x+y*y)/(sigma*sigma))* (pow(2.718281828, - (x*x + y*y) / 2*sigma*sigma));
         }
    }
    
    return kernel;
}

Mat GaussianKernel(int winSize,double sigma){
    
    Mat kernel(winSize,winSize,CV_64F, 0.0);    
    double hWin = (winSize-1)/2.0; 
    for (int i = 0; i < kernel.rows ; i++){
      for (int j = 0; j < kernel.cols ; j++){ 
            double x = j - hWin;
            double y = i - hWin;           
            kernel.at<double>(j,i) = (1.0 /(2*M_PI*sigma*sigma)) * (pow(2.718281828, - (x*x + y*y) / 2*sigma*sigma));
         }
    }
    
    return kernel;
}

Mat CalculateDisparityMap(Mat &_left, Mat &_right, Mat &_ThreshMap,int winSize = 9, int minDisp = 0, int maxDisp = 16,int medianSize = 9){

    //Clone input mats to avoid unintentional alterations
    Mat left = _left.clone();
    Mat right = _right.clone();
    Mat ThreshMap = _ThreshMap.clone();
        
     int numCols = left.cols;
     int numRows = left.rows;
     //Winsize must be an odd number in order to have a single centered pixel.
     if(winSize%2 == 0) winSize++; //If winSize is an even number make it an odd number. 
     Mat disp = Mat( left.rows, left.cols, CV_8UC1, 0.0);
     int hWin = (winSize-1)/2; //Represents the edges protruding from the window excluding the center pixel.

     for (int i = hWin; i < left.rows -hWin; i++) {
         int minScore,Score,bestPos;
            for (int j = hWin; j < left.cols-maxDisp -hWin; j++) {                
                unsigned char* p = const_cast<unsigned char*>(disp.ptr(i,j));
                if (ThreshMap.at<uchar>(i,j) == 255){ //Only check disparity within Threshold map
                    minScore = INT_MAX;
                    for (int d = minDisp; d <= maxDisp; d++) { //Disparity loop
                        Score = 0;
                        for (int l = i; l < i + hWin; l++) { //Window loop
                            for (int m = j; m <= j + hWin ; m++) { //Window loop 
                                if (Score > minScore) goto cut; //Skip calculations if allready invalid.
                                Score += abs(left.at<uchar>(l, m) - right.at<uchar>(l, m + d)); //Correlation - SAD                         
                            }
                        }
                        cut:
                        if (Score < minScore) { //Winner takes all scenario
                            minScore = Score;
                            bestPos = d; //Save best score
                        }
                    }
                    *p = (bestPos - minDisp)*255/(maxDisp - minDisp); //Assign disparity to center pixel
                }
            }    
     }
     //Apply a median filter with a odd kernel
     if (medianSize%2 == 0) medianSize++;
     Mat medianKernel = Mat(medianSize,medianSize,CV_16SC1,1.0);
     disp = Convolution( disp, medianKernel, (double) 1/(medianSize*medianSize)); 
     return disp;
 }

class ParallelDispL2R: public ParallelLoopBody
{   
public:
ParallelDispL2R(Mat &_left, Mat &_right, Mat &_ThreshMapL, Mat &_ThreshMapR , Mat &_disp, int _maxD, int _hWin, int _i) : left(_left), right( _right), ThreshMapL(_ThreshMapL), ThreshMapR(_ThreshMapR), disp( _disp) , maxD( _maxD), hWin(_hWin), i(_i)
{    
}

void operator() (const Range &r) const
{        
        int minScore,Score,bestPos;
        for (int j = r.start; j < r.end; ++j) {
            unsigned char* p = const_cast<unsigned char*>(disp.ptr(i,j));
            minScore = INT_MAX;
            if (ThreshMapL.at<uchar>(i,j) == 255){
                for (int d = 0; d <= maxD; ++d) {
                    Score = 0;
                    if ( ThreshMapR.at<uchar>(i,j+d) == 255){
                        for (int l = i; l < i + hWin; ++l) {
                            for (int m = j; m <= j + hWin ; ++m) {
                                if (Score > minScore) goto cut;
                                Score += abs(left.at<uchar>(l, m) - right.at<uchar>(l, m + d));                          
                            }
                        }
                        cut:
                        if (Score < minScore) {
                            minScore = Score;
                            bestPos = d;
                        }
                    }
                }
                *p = bestPos*255/maxD;
            } else {
                *p = 0;
            }
        }    
        
//        int bestPos;
//        double Ncc, minNcc;
//        for (int j = r.start; j < r.end; ++j) {
//            unsigned char* p = const_cast<unsigned char*>(disp.ptr(i,j));
//            minNcc = DBL_MAX;
//                for (int d = 0; d <= maxD; ++d) {
//                    Ncc = 0.0; 
//                    int NccNum = 0;
//                    int NccDemRW = 0;
//                    int NccDemLW = 0;
//                    for (int l = i; l < i + hWin; ++l) {
//                        for (int m = j; m <= j + hWin ; ++m) {
//                            NccNum += left.at<uchar>(l,m)*right.at<uchar>(l,m+d);
////                            NccDemRW += right.at<uchar>(l,m+d)*right.at<uchar>(l,m+d);
////                            NccDemLW += left.at<uchar>(l,m)*left.at<uchar>(l,m);
//                        }
//                    }
////                    double NccDem = sqrt( NccDemRW*NccDemLW);
//                    double Ncc = (double) NccNum;///NccDem;
//                    if (Ncc < minNcc) {
//                        minNcc = Ncc;
//                        bestPos = d;
//                    }
//                }
//                *p = bestPos*255/maxD;   
//        }
         
}

private:
    int maxD;
    int hWin;
    int i;
    Mat left, right, disp, ThreshMapL, ThreshMapR;
    unsigned char lookupTable[256];
};

class ParallelDispR2L: public ParallelLoopBody
{   
public:
ParallelDispR2L(Mat &_left, Mat &_right, Mat &_ThreshMapL, Mat &_ThreshMapR , Mat &_disp, int _maxD, int _hWin, int _i) : left(_left), right( _right), ThreshMapL(_ThreshMapL), ThreshMapR(_ThreshMapR), disp( _disp) , maxD( _maxD), hWin(_hWin), i(_i)
{    
}

void operator() (const Range &r) const
{        
        int minScore,Score,bestPos;
        for (int j = r.end; j >= r.start; --j) {
            unsigned char* p = const_cast<unsigned char*>(disp.ptr(i,j-maxD));
            minScore = INT_MAX;
            if (ThreshMapL.at<uchar>(i,j) == 255){
                for (int d = 0; d <= maxD; ++d) {
                    Score = 0;
                    if ( ThreshMapR.at<uchar>(i,j+d) == 255){
                        for (int l = i; l < i + hWin; ++l) {
                            for (int m = j; m <= j + hWin ; ++m) {
                                if (Score > minScore) goto cut;
                                Score += abs(right.at<uchar>(l, m) - left.at<uchar>(l, m - d));                          
                            }
                        }
                        cut:
                        if (Score < minScore) {
                            minScore = Score;
                            bestPos = d;
                        }
                    }
                }
                *p = bestPos*255/maxD;
            } else {
                *p = 0;
            }
        }    
                
}

private:
    int maxD;
    int hWin;
    int i;
    Mat left, right, disp, ThreshMapL, ThreshMapR;
    unsigned char lookupTable[256];
};

Mat CalculateDisparityMapPar(Mat &_left, Mat &_right, Mat& _ThreshMapL, Mat& _ThreshMapR , int winSize = 9, int minDisp = 0, int maxDisp = 16,int medianSize = 9, bool l2r = true){

    Mat left = _left.clone();
    Mat right = _right.clone();
    Mat ThreshMapL = _ThreshMapL.clone();
    Mat ThreshMapR = _ThreshMapR.clone();
    if(winSize%2 == 0) winSize++;
    Mat disp = Mat( left.rows, left.cols, CV_8UC1, 0.0);
    int hWin = (winSize-1)/2;    
    if (l2r == true){
        for (int i = hWin; i < left.rows -hWin; i++) {
               parallel_for_(Range(hWin,left.cols-maxDisp),ParallelDispL2R( left, right, ThreshMapL, ThreshMapR, disp, maxDisp, hWin,i));
        }     
    } else {
        for (int i = hWin; i < left.rows -hWin; i++) {
               parallel_for_(Range(hWin + maxDisp,left.cols -hWin -1),ParallelDispR2L( left, right, ThreshMapL, ThreshMapR, disp, maxDisp, hWin,i));
        }
    }    
     if (medianSize%2 == 0) medianSize++;
     Mat medianKernel = Mat(medianSize,medianSize,CV_16SC1,1.0);
     disp = Convolution( disp, medianKernel, (double) 1/(medianSize*medianSize));
     return disp;
 }

Mat CalculateDisparityMapR2L(Mat &_left, Mat &_right, int winSize = 9, int minDisp = 0, int maxDisp = 16,int medianSize = 9){

    Mat left = _left.clone();
    Mat right = _right.clone();
    
     int numCols = left.cols;
     int numRows = left.rows;
     if(winSize%2 == 0) winSize++;
 //    if(winSize%2-1 != 0) throw "Winsize must be odd for center pixel.";//Allows for a single center pixel.
 //    if(minDisp>maxDisp) throw "minDisp must be smaller than maxDisp.";
     Mat disp = Mat( left.rows, left.cols, CV_8UC1, 0.0);
     int hWin = (winSize-1)/2; //Represents the edges protruding from the window excluding the center pixel.

     int minDispVar = (minDisp <= 0) ? -minDisp : 0;
     for (int i = hWin; i < left.rows -hWin; i++) {
         int minScore,Score,bestPos;
         for (int j = left.cols -hWin -1; j >=  hWin + maxDisp + minDispVar; j--) {
             unsigned char* p = const_cast<unsigned char*>(disp.ptr(i,j));
             minScore = INT_MAX;
             for (int d = minDisp; d <= maxDisp; d++) {
                 Score = 0;
                 for (int l = i; l < i + hWin; l++) {
                     for (int m = j; m <= j + hWin ; m++) {
                         if (Score > minScore) goto cut;
                         Score += abs(right.at<uchar>(l, m) - left.at<uchar>(l, m - d));                          
                     }
                 }
                 cut:
                 if (Score < minScore) {
                     minScore = Score;
                     bestPos = d;
                 }
             }
             *p = (bestPos - minDisp)*255/(maxDisp - minDisp);
         }    
     }
     if (medianSize%2 == 0) medianSize++;
     Mat medianKernel = Mat(medianSize,medianSize,CV_16SC1,1.0);
     disp = Convolution( disp, medianKernel, (double) 1/(medianSize*medianSize));
     return disp;

 }


class ParallelConvolution: public ParallelLoopBody
{   
public:
ParallelConvolution(Mat &_img, Mat &_kernel, Mat &_out, int _hWin, double _val, int _i) : img(_img), kernel( _kernel), out( _out), hWin(_hWin), val(_val), i(_i)
{    
}

void operator() (const Range &r) const
{        
    double accumulator;    
    for (int j = r.start; j < r.end; ++j) {
        short* p = const_cast<short*>(out.ptr<short>(i,j));
        accumulator = 0.0;
        for(int k = -hWin; k <= hWin; k++){
            for(int l = -hWin; l <= hWin; l++){
                accumulator += (img.at<uchar>(i+l,j+k)*kernel.at<short>(l+hWin,k+hWin));
            }
        }
        *p = accumulator*val;         
    }
       
}

private:
    double val;
    int i, hWin;
    Mat img, kernel, out;
};


Mat Convolution(Mat &img, Mat &kernel,double val = 1.0){
    int winSize = kernel.cols;
    if (winSize%2 == 0) throw "Require odd kernel";
    int hWin = (winSize-1)/2;
    Mat out = Mat( img.rows, img.cols, CV_16SC1,0.0);
  
    for(int i = hWin; i < img.rows -hWin; i++){
         parallel_for_(Range(hWin,img.cols -hWin),ParallelConvolution( img, kernel, out, hWin, val, i));
    }    
    
    short min = SHRT_MAX;
    short max = SHRT_MIN;
    
    for(int i = hWin; i < out.cols -hWin; i++){
        for(int j = hWin; j < out.rows -hWin; j++){ 
            short pixelvalue = out.at<short>(j,i);
                if( pixelvalue < min)
                    min = pixelvalue;
                if (pixelvalue > max)
                    max = pixelvalue;            
        }  
    }   
    
    Mat out8U = Mat( out.rows, out.cols, CV_8UC1, 0.0);
    if( min != max){        
        for(int i = hWin; i < out.cols -hWin; i++){
            for(int j = hWin; j < out.rows -hWin; j++){ 
                out8U.at<uchar>(j,i) = (out.at<short>(j,i) -min)*255/(max-min);
    //            cout << (int) out8U.at<uchar>(j,i) << " " << out.at<short>(j,i) << endl;
            }   
        }
    }
    return out8U;
    
}

Mat ConditionalConvolution(Mat &img, Mat &conditionalKernel, Mat &kernel, int alphaThresh, double condVal = 1.0, double val = 1.0){
    int winSize = kernel.cols;
    if (winSize%2 == 0) throw "Require odd kernel";
    int hWin = (winSize-1)/2;
    Mat out = Mat( img.rows, img.cols, CV_16SC1,0.0);
    short conAccumulator;
    short accumulator;
    for(int i = hWin; i < img.cols -hWin; i++){
        for(int j = hWin; j < img.rows -hWin; j++){
            conAccumulator = 0.0;
            for(int k = -hWin; k <= hWin; k++){
                for(int l = -hWin; l <= hWin; l++){
                    conAccumulator += (img.at<uchar>(j+l,i+k)*conditionalKernel.at<short>(l+hWin,k+hWin));
//                    cout << accumulator << endl;
                }
            }
            cout << alphaThresh << " " << (conAccumulator*condVal) << " " << (int) img.at<uchar>(i,j) << endl;
            if( alphaThresh < (conAccumulator*condVal) - img.at<uchar>(i,j)){            
                for(int k = -hWin; k <= hWin; k++){
                    for(int l = -hWin; l <= hWin; l++){
                        accumulator += (img.at<uchar>(j+l,i+k)*kernel.at<short>(l+hWin,k+hWin));
    //                    cout << accumulator << endl;
                    }
                }
                out.at<short>(j,i) = (accumulator - img.at<uchar>(i,j))*val;
            } else
                out.at<short>(j,i) = img.at<uchar>(i,j);
//            cout << out.at<short>(j,i) << endl;
        }
    }
    
    short min = SHRT_MAX;
    short max = SHRT_MIN;
    
    for(int i = hWin; i < out.cols -hWin; i++){
        for(int j = hWin; j < out.rows -hWin; j++){ 
            short pixelvalue = out.at<short>(j,i);
                if( pixelvalue < min)
                    min = pixelvalue;
                if (pixelvalue > max)
                    max = pixelvalue;            
        }  
    }   
    
    Mat out8U = Mat( out.rows, out.cols, CV_8UC1, 0.0);
    if( min != max){        
        for(int i = hWin; i < out.cols -hWin; i++){
            for(int j = hWin; j < out.rows -hWin; j++){ 
                out8U.at<uchar>(j,i) = (out.at<short>(j,i) -min)*255/(max-min);
    //            cout << (int) out8U.at<uchar>(j,i) << " " << out.at<short>(j,i) << endl;
            }   
        }
    }
    return out8U;
    
}

Mat Scale(Mat &a_image,int min, int max, bool custom = false){
    
    Mat out = Mat(a_image.rows, a_image.cols, CV_8UC1);
    uchar* a_imageData = a_image.data;
    uchar* outData = out.data;    
    if( !custom ){
        if( a_image.type() == CV_8UC1){
            for (int i = 0; i < a_image.rows; i++) {
                for (int j = 0; j < a_image.cols; j++) {                 
                    outData[i*a_image.step + j] = (a_imageData[i*a_image.step + j] - min)*255/(max-min);
                }
            }  
        } else if ( a_image.type() == CV_16SC1){

            for (int i = 0; i < a_image.rows; i++) {
                for (int j = 0; j < a_image.cols; j++) { 
                    outData[i*a_image.step + j] = (a_image.at<short>(i,j) - min)*255/(max-min);                
                }
            }  
        }
    } else {
        if( a_image.type() == CV_8UC1){
            for (int i = 0; i < a_image.rows; i++) {
                for (int j = 0; j < a_image.cols; j++) {   
                    int temp = (a_imageData[i*a_image.step + j] - min)*255/(max-min);
                    if (temp > 255) 
                        temp = 255;
                    else if (temp < 0) 
                        temp = 0;
                    
                    outData[i*a_image.step + j] = temp;
                }
            }  
        } else if ( a_image.type() == CV_16SC1){

            for (int i = 0; i < a_image.rows; i++) {
                for (int j = 0; j < a_image.cols; j++) { 
                    int temp = (a_image.at<short>(i,j) - min)*255/(max-min);
                    if (temp > 255) 
                        temp = 255;
                    else if (temp < 0) 
                        temp = 0;
                    outData[i*a_image.step + j] = temp;                
                }
            }  
        }
    }
    return out;
}

void minMaxInt(Mat &a_image, int &_min, int &_max){
    
    int min = INT_MAX;
    int max = INT_MIN;
    
    if( a_image.type() == CV_8UC1){
        
        uchar* a_imageData = a_image.data;
        uchar pixelvalue;

        for (int i = 0; i < a_image.rows; i++) {
            for (int j = 0; j < a_image.cols; j++) { 
                pixelvalue = a_imageData[a_image.step*i+j];
                if(pixelvalue < min)
                    min = pixelvalue;
                if (pixelvalue > max)
                    max = pixelvalue;
            }  
        }
        
    } else if ( a_image.type() == CV_16SC1){
                
        short pixelvalue;

        for (int i = 0; i < a_image.cols; i++) {
            for (int j = 0; j < a_image.rows; j++) { 
                pixelvalue = a_image.at<short>(j,i);
                if(pixelvalue < min)
                    min = pixelvalue;
                if (pixelvalue > max)
                    max = pixelvalue;
            }  
        }
    }
    _min = min;
    _max = max;
}

Mat Threshold(Mat &a_image, int min, int max){
    Mat out = Mat(a_image.rows,a_image.cols,a_image.type());
    uchar* imgData = a_image.data;
    uchar* outData = out.data;
    for (int i = 0; i < a_image.rows; i++) {
        for (int j = 0; j < a_image.cols; j++) {  
            uchar temp = imgData[i*a_image.step+j];
            if( temp <= max & temp >= min)
                outData[i*out.step+j] = 255;
            else 
                outData[i*out.step+j] = 0;
        }
    }        
    
    return out;
}

Mat Histogram(Mat &a_image, Mat &histOut,bool showHist){

    if ( a_image.type() == CV_8UC1){
        Mat greyTable = Mat(1,256,CV_16SC1,0.0);
        uchar* a_imageData = a_image.data;    
        for (int i = 0; i < a_image.rows; i++) {
            for (int j = 0; j < a_image.cols; j++) { 
                greyTable.at<short>(0,a_imageData[i*a_image.step + j])++;
            }
        }

        histOut = greyTable.clone();
        Mat histogram = Mat(256,256,CV_8UC1,0.0);
        if (showHist){
            int min, max;
            minMaxInt(greyTable, min, max);
            Mat greyHist8U = Scale(greyTable, min, max);

            //Visualize    

            uchar* histogramData = histogram.data;
            for (int i = 0; i < histogram.rows; i++) {
                for (int j = histogram.cols - 1; j >  histogram.cols - greyHist8U.at<uchar>(0,i); j--) { 
                    histogramData[j*histogram.step + i] = i;
                }
            }        
        } 

        return histogram;
    } else if ( a_image.type() == CV_8UC3){

        Mat rgbTable = Mat(3,256,CV_16SC1,0.0); 
        for (int i = 0; i < a_image.rows; i++) {
            for (int j = 0; j < a_image.cols; j++) { 
                Vec3b p = a_image.at<Vec3b>(i,j);
                rgbTable.at<short>(0,p[0])++;
                rgbTable.at<short>(1,p[1])++;
                rgbTable.at<short>(2,p[2])++;
            }
        }

        histOut = rgbTable.clone();        
        Mat histogram = Mat(256,255*3,CV_8UC3,0.0);
        if (showHist){
            int min, max;
            Mat bTable = rgbTable.row(0); 
            Mat gTable = rgbTable.row(1); 
            Mat rTable = rgbTable.row(2); 

            minMaxInt(bTable, min, max); 
            Mat bHist8U = Scale(bTable, min, max);
            minMaxInt(gTable, min, max);
            Mat gHist8U = Scale(gTable, min, max);
            minMaxInt(rTable, min, max);
            Mat rHist8U = Scale(rTable, min, max);

            //Visualize    
            for (int i = 0; i < 256; i++) {
                for (int j = histogram.cols - 1; j >  histogram.cols - bHist8U.at<uchar>(0,i); j--) {
                    histogram.at<Vec3b>(j,i)[0] = i;
                }
                for (int j = histogram.cols - 1; j >  histogram.cols - gHist8U.at<uchar>(0,i); j--) {
                    histogram.at<Vec3b>(j,i+255-1)[1] = i;
                }
                for (int j = histogram.cols - 1; j >  histogram.cols - rHist8U.at<uchar>(0,i); j--) {
                    histogram.at<Vec3b>(j,i+255*2-1)[2] = i;
                }
            }

        }
        return histogram;        
    }

}

Mat HistogramEq(Mat &a_image, Mat &histogram){
    int pixTotal = a_image.rows*a_image.cols;
    int L = 256;
    
    if ( a_image.type() == CV_8UC1){        
        uchar transform[256] = {0};
        double temp = 0.0;
        for( int i = 0; i < 256; i++){  
            //Cumulative Sum
            temp += histogram.at<short>(0,i);
            transform[i] = floor(255*temp/pixTotal);
        }
        Mat out = Mat(a_image.rows,a_image.cols,CV_8UC1,0.0);
                
        
        
        for (int i = 0; i < a_image.rows; i++) {
            for (int j = 0; j < a_image.cols; j++) { 
                out.at<uchar>(i,j) = transform[a_image.at<uchar>(i,j)];
            }
        }
        return out;
    } else if ( a_image.type() == CV_8UC3){
        uchar transform[3][255] = {0};

        double tempB = 0.0;
        double tempG = 0.0;
        double tempR = 0.0;
        for( int i = 0; i < 256; i++){  
            //Cumulative Sum
            tempB += histogram.at<short>(0,i);
            tempG += histogram.at<short>(1,i);
            tempR += histogram.at<short>(2,i);
            transform[0][i] = floor(255*tempB/pixTotal);
            transform[1][i] = floor(255*tempG/pixTotal);
            transform[2][i] = floor(255*tempR/pixTotal);
        }
        Mat out = Mat(a_image.rows,a_image.cols,CV_8UC3,0.0);
                        
        for (int i = 0; i < a_image.rows; i++) {
            for (int j = 0; j < a_image.cols; j++) { 
                Vec3b pixIn = a_image.at<Vec3b>(i,j);
                out.at<Vec3b>(i,j)[0] = transform[0][pixIn[0]];
                out.at<Vec3b>(i,j)[1] = transform[1][pixIn[1]];
                out.at<Vec3b>(i,j)[2] = transform[2][pixIn[2]];
            }
        }   
        
        return out;
    }
    
}

Mat Sobel(Mat &a_image){
    
            /// Generate grad_x and grad_y
            Mat xL, yL, axL, ayL;           

            int scale = 1;
            int delta = 0;
            /// Gradient X
            //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
            Sobel( a_image, xL, CV_16S, 1, 0, 3, scale, delta, BORDER_DEFAULT );
            convertScaleAbs( xL, axL );

            /// Gradient Y
            //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
            Sobel( a_image, yL, CV_16S, 0, 1, 3, scale, delta, BORDER_DEFAULT );
            convertScaleAbs( yL, ayL );

            /// Total Gradient (approximate)
            Mat gradL;
            addWeighted( axL, 0.5, ayL, 0.5, 0, gradL );   
            Mat out;
            gradL.convertTo(out, CV_8UC1);
            
            return out;
    
}

Mat SparseDispInterpolation( Mat &a_image, int Threshold, int minxT, int maxxT){
    
    Mat out = Mat(a_image.rows, a_image.cols, CV_8UC1, 0.0);
    for(int i = 0; i < a_image.rows; i++){
        for(int j = 0; j < a_image.cols; j++){
            int minx = 0;
            while( a_image.at<uchar>(i, j - minx) == 0 && j - minx > 0 && minx < minxT) minx++;
            int maxx = 0;
            while( a_image.at<uchar>(i, j + maxx) == 0 && j + maxx < a_image.cols && maxx < maxxT) maxx++;
            
            int miny = 0;
            while( a_image.at<uchar>(i - miny, j ) == 0 && i - miny > 0 && miny < minxT) miny++;
            int maxy = 0;
            while( a_image.at<uchar>(i + maxy, j) == 0 && i + maxy < a_image.rows && maxy < maxxT) maxy++;            
            if( maxy == maxxT -1 || minx == minxT -1 )
                out.at<uchar>(i,j) = 0;
            else if( abs(a_image.at<uchar>(i,j-minx) - a_image.at<uchar>(i,j+maxx)) < Threshold && abs(a_image.at<uchar>(i-miny,j) - a_image.at<uchar>(i+maxy,j)) < Threshold)
                out.at<uchar>(i,j) = (a_image.at<uchar>(i,j-minx) + a_image.at<uchar>(i,j+maxx) + a_image.at<uchar>(i-miny,j) + a_image.at<uchar>(i+maxy,j))/4;
            else
                out.at<uchar>(i,j) = 0;
            
        }
    }
    
    return out;
}

struct ObjPoint{
    int x;
    int y;
    int hWin;
};

Mat integralImage(Mat &a_image) {
    Mat out = Mat(a_image.rows+1, a_image.cols+1, CV_16SC1, 0.0);
        
    for(int i = 1; i < out.rows; i++){
        for(int j = 1; j < out.cols; j++){
            out.at<short>(i,j) = a_image.at<uchar>(i-1,j-1) + out.at<short>(i-1,j) + out.at<short>(i,j-1) - out.at<short>(i-1,j-1);  
        }
    }
//    out = CutImage(out,1,1,1,1);
    return out;
}

Mat BinaryMask(Mat &a_image, Mat &mask){
    
    Mat out = Mat(a_image.rows, a_image.cols, CV_8UC1, 0.0);
    out = a_image.clone();
        
    for(int i = 1; i < out.rows; i++){
        for(int j = 1; j < out.cols; j++){
            if( mask.at<uchar>(i,j) == 0){
                out.at<uchar>(i,j) = 0;
            }
        }
    }
    return out;
}

Mat ParabolaBinaryMask(Mat &a_image){
    
    Mat out = Mat(a_image.rows, a_image.cols, CV_8UC1, 0.0);    
    int r = a_image.rows;
    int c = a_image.cols;
    for(int x = 0; x < a_image.cols; x++){
        int y = (int) (-( (double) 2.5*r/(c*c))*(x-c*2/3)*(x-c*2/3) + (r*9)/10);   
        if( y > 0 && y < a_image.rows){
            while( y != 0){
                out.at<uchar>(r - y, x) = 255;
                y--;
            }  
        }
    }
    return out;
}

struct curPoints{
    float dist;
    Point center;
    int classification;
};
struct labelDataStruct{
    int xdiff, ydiff, pixCount, avgDisp; 
    Point center;
};
Mat BlobAnalysis(Mat &a_image, Mat &disp, Mat &view, vector<PointFrameCount> &prevDetections){
    //Blob detection classification and tracking
//    Mat out = Mat(a_image.rows, a_image.cols, CV_8UC1, 0.0);
    Mat labeled = Mat(a_image.rows, a_image.cols, CV_8UC1, 255.0);
    int curLabel = 1;
    queue<Point> pointQueue;
    
    vector<labelDataStruct> labelData;// maxx - minx, maxy - miny
    for(int i = 1; i < labeled.rows-1; i++){
        for(int j = 1; j < labeled.cols-1; j++){
            if( a_image.at<uchar>(i,j) == 255){
                if( labeled.at<uchar>(i,j) == 255){
                    labeled.at<uchar>(i,j) = curLabel;
                    pointQueue.push(Point(i,j));
                    int maxx = INT_MIN; int maxy = INT_MIN;
                    int minx = INT_MAX; int miny = INT_MAX;
                    int pixCount = 0;
                    int avgDisp = 0;
                    while( !pointQueue.empty()){
                        Point temp = pointQueue.front();
                        pixCount++;
//                        cout << "(" << avgDisp << "*(" << pixCount << "-1) + " << (int) disp.at<uchar>(i,j) << ")/" << pixCount << endl;
                        avgDisp = (avgDisp*(pixCount -1) + disp.at<uchar>(i,j))/pixCount;                        
//                        cout << "= " << avgDisp << endl;
                        if( temp.x > maxx) maxx = temp.x;
                        if( temp.y > maxy) maxy = temp.y;
                        if( temp.x < minx) minx = temp.x;
                        if( temp.y < miny) miny = temp.y;
                        pointQueue.pop();
                        for(int x = -1; x <= 1; x++){
                            for(int y = -1; y <= 1; y++){ 
                                if( !(x == 0 && y == 0)){
//                                    cout << "Disp at "<<  (int) disp.at<uchar>(temp.x +x, temp.y +y)  << endl;
                                    if( temp.x + x > 0 && temp.x +x < labeled.rows && temp.y + y > 0 && temp.y +y < labeled.cols ){
                                        if( a_image.at<uchar>(temp.x +x, temp.y +y) == 255 && labeled.at<uchar>(temp.x +x, temp.y +y) == 255 && disp.at<uchar>(temp.x +x, temp.y +y) < avgDisp + X_MIN0 && disp.at<uchar>(temp.x +x, temp.y +y) > avgDisp - X_MIN0){                                        
                                            labeled.at<uchar>(temp.x +x, temp.y +y) = curLabel;                                        
                                            pointQueue.push(Point(temp.x +x, temp.y +y));
                                        }
                                    }
                                }
                            }
                        }                        
                    }
                    labelDataStruct temp;
                    temp.xdiff = maxx - minx;
                    temp.ydiff = maxy - miny;
                    temp.pixCount = pixCount;
                    temp.center = Point(miny + (maxy - miny)/2, minx + (maxx - minx)/2);
                    temp.avgDisp = avgDisp;
                    labelData.push_back(temp);
                    curLabel++;
                }                
            }
        }
    }   
         
    vector<curPoints> curPVec;
    while(!labelData.empty()){   
        labelDataStruct temp = labelData.back();
        int w = temp.xdiff;
        int h = temp.ydiff;
        int pixCount = temp.pixCount;
        Point center = temp.center;        
        labelData.pop_back();        
        int dispAt = disp.at<uchar>(center.y, center.x);
//        int dispAt = temp.avgDisp;
        if( h != 0 && w != 0){        
//            cout << labelData.size() << " "  << (double) w/h << " " << (double) X_1/100 << " " << center.x << "," << center.y << endl;
            if( w*h < (double) Kmax*(255 -dispAt) && w*h > Kmin*(255 -dispAt) && (float) 0.5787*DistanceMultiplier/(255 -dispAt) < (float) X_3){
//            if( w*h < (double) 100*(255 -dispAt) && w*h > 4*(255 -dispAt) && (float) 6.4619921749223158e+002*1/(256-dispAt) < 60.0){
                if ((double) w/h > 0.2){
                    curPoints temp;
                    temp.dist = (float) 0.5787*DistanceMultiplier/(255 -dispAt);
//                    temp.dist = (float) 6.4619921749223158e+002*10/(256-dispAt);
                    temp.center = center;                    
                    if ((double) w/h > (double) Bc/10)//0.5)
                        temp.classification = 0; 
                    else if( (double) w/h < (double) Ac/10)//1)
                        temp.classification = 1;     
                    else if( (double) w/h <= (double) Cc/10)//1.5)
                        temp.classification = 2;                 
                    curPVec.push_back(temp);
                }
            }
        }
    }
    
//    cout<< "curPVec: ";
//    for(vector<curPoints>::iterator it = curPVec.begin(); it != curPVec.end(); ++it) {    
//       cout << "(" << (*it).center.x << "," << (*it).center.y << ") ";
//    }
//    cout << endl;     
        
    vector<PointFrameCount> newPrev;    
    for(vector<curPoints>::iterator it = curPVec.begin(); it != curPVec.end(); ++it) {    
        bool wasPrev = false;        
        for(vector<PointFrameCount>::iterator it2 = prevDetections.begin(); it2 != prevDetections.end(); ++it2) { 
            if( !(*it2).detected){
                if ( sqrt(((*it).center.x-(*it2).center.x)*((*it).center.x-(*it2).center.x) + ((*it).center.y-(*it2).center.y)*((*it).center.y-(*it2).center.y)) < X_MAX0){//0.5*(255 -disp.at<uchar>((*it).center.x, (*it).center.y))
                    PointFrameCount temp;
                    temp.FrameCount = (*it2).FrameCount + 1;
                    temp.lives = (*it2).lives;
                    if( temp.FrameCount%6 == 0 && temp.FrameCount != 0) temp.lives++;
                    temp.center = (*it).center;
                    temp.dist = (*it).dist;
                    temp.prevCenter = (*it2).center;
                    temp.firstPoint = (*it2).firstPoint;
                    temp.classification = (*it2).classification;
                    temp.detected = false;
                    (*it2).detected = true;
                    newPrev.push_back(temp);
                    wasPrev = true;
                    break;
                }
            }
        }
        if (!wasPrev){
                PointFrameCount temp;
                temp.FrameCount = 0;
                temp.lives = 0;
                temp.detected = false;
                temp.firstPoint = (*it).center;
                temp.center = (*it).center;
                temp.dist = (*it).dist;
                temp.classification = (*it).classification;
                newPrev.push_back(temp);
        }
    }    
    curPVec.~vector();
    
    for(vector<PointFrameCount>::iterator it2 = prevDetections.begin(); it2 != prevDetections.end(); ++it2) {     
        if( (*it2).detected == false && (*it2).lives > 0){            
            (*it2).lives--; 
            (*it2).detected = false; 
            newPrev.push_back( (*it2));
        }        
    }
    
    Mat rgbView = view.clone();
    cvtColor(rgbView, rgbView, CV_GRAY2BGR);    
    for(vector<PointFrameCount>::iterator it = newPrev.begin(); it != newPrev.end(); ++it) {  
//        Scalar colour = Scalar(0,200,225);
//        Scalar textColour = Scalar(0,255,225);                
//        if( (*it).FrameCount > 10) {colour = Scalar(200,0,0); textColour = Scalar(255,0,0);} 
//        if( (*it).FrameCount > 20) {colour = Scalar(0,200,0); textColour = Scalar(0,255,0);}
//        if( (*it).FrameCount > 30) {colour = Scalar(0,0,200); textColour = Scalar(0,0,255);}

        Mat colourMap = (Mat_<uchar>(1, 1) << (uchar) (*it).dist*255/50);          
        applyColorMap(colourMap, colourMap, COLORMAP_RAINBOW);            
        Vec3b c = colourMap.at<Vec3b>(0,0);
        Scalar colour = Scalar(c[0], c[1], c[2]);
        if( (*it).FrameCount > 4){
            char str[200];
            sprintf(str,"%.2f m", (*it).dist); //0.5787 = b f =        
            char str2[200];
            sprintf(str2,"%d", (*it).lives); //0.5787 = b f =   
            if( (*it).center.y + 40 < rgbView.rows)
                putText(rgbView , str, Point((*it).center.x, (*it).center.y + 40), FONT_HERSHEY_PLAIN, 1.0, colour, 1.8);
            if( (*it).center.y + 60 < rgbView.rows && (*it).center.x + 20 < rgbView.cols)
                putText(rgbView , str2, Point((*it).center.x + 20, (*it).center.y + 60), FONT_HERSHEY_PLAIN, 1.0, colour, 1.8);
            if( (*it).center.y + 60 < rgbView.rows){
                if( (*it).classification == 0) sprintf(str2,"P");
                else if( (*it).classification == 1) sprintf(str2,"V");
                else if( (*it).classification == 2) sprintf(str2,"U");
                putText(rgbView , str2, Point((*it).center.x, (*it).center.y + 60), FONT_HERSHEY_PLAIN, 1.0, colour, 1.8);                
            }
            circle(rgbView, (*it).center, 20, colour);  
            if( ((*it).center.x + (*it).center.x -(*it).prevCenter.x > 0) && ((*it).center.y + (*it).center.y - (*it).prevCenter.y > 0))
                line(rgbView, (*it).firstPoint, Point((*it).center.x + (*it).center.x -(*it).prevCenter.x, (*it).center.y + (*it).center.y - (*it).prevCenter.y), colour, 2.0);                  
        }
        
    }
    
//    cout<< "newPrev: ";
//    for(vector<PointFrameCount>::iterator it = newPrev.begin(); it != newPrev.end(); ++it) {    
//       cout << "(" << (*it).FrameCount << ")(" << (*it).center.x << "," << (*it).center.y << ") ";
//    }
//    cout << endl; 
    prevDetections.swap(newPrev);
    return rgbView;
}

Mat outputPrevDet( Mat src, vector<PointFrameCount> &prevD){
    
    Mat rgbView = src.clone();
    cvtColor(rgbView, rgbView, CV_GRAY2BGR);    
    for(vector<PointFrameCount>::iterator it = prevD.begin(); it != prevD.end(); ++it) {  
//        Scalar colour = Scalar(0,200,225);
//        Scalar textColour = Scalar(0,255,225);                
//        if( (*it).FrameCount > 10) {colour = Scalar(200,0,0); textColour = Scalar(255,0,0);} 
//        if( (*it).FrameCount > 20) {colour = Scalar(0,200,0); textColour = Scalar(0,255,0);}
//        if( (*it).FrameCount > 30) {colour = Scalar(0,0,200); textColour = Scalar(0,0,255);}

        Mat colourMap = (Mat_<uchar>(1, 1) << (uchar) (*it).dist*255/50);          
        applyColorMap(colourMap, colourMap, COLORMAP_RAINBOW);            
        Vec3b c = colourMap.at<Vec3b>(0,0);
        Scalar colour = Scalar(c[0], c[1], c[2]);
        if( (*it).FrameCount > 4){
            char str[200];
            sprintf(str,"%.2f m", (*it).dist); //0.5787 = b f =        
            char str2[200];
            sprintf(str2,"%d", (*it).lives); //0.5787 = b f =   
            if( (*it).center.y + 40 < rgbView.rows)
                putText(rgbView , str, Point((*it).center.x, (*it).center.y + 40), FONT_HERSHEY_PLAIN, 1.0, colour, 1.8);
            if( (*it).center.y + 60 < rgbView.rows && (*it).center.x + 20 < rgbView.cols)
                putText(rgbView , str2, Point((*it).center.x + 20, (*it).center.y + 60), FONT_HERSHEY_PLAIN, 1.0, colour, 1.8);
            if( (*it).center.y + 60 < rgbView.rows){
//                if( (*it).classification == 0) sprintf(str2,"P");
//                else if( (*it).classification == 1) sprintf(str2,"V");
//                else if( (*it).classification == 2) sprintf(str2,"U");
//                putText(rgbView , str2, Point((*it).center.x, (*it).center.y + 60), FONT_HERSHEY_PLAIN, 1.0, colour, 1.8);                
            }
            circle(rgbView, (*it).center, 20, colour);  
//            if( ((*it).center.x + (*it).center.x -(*it).prevCenter.x > 0) && ((*it).center.y + (*it).center.y - (*it).prevCenter.y > 0))
//                line(rgbView, (*it).firstPoint, Point((*it).center.x + (*it).center.x -(*it).prevCenter.x, (*it).center.y + (*it).center.y - (*it).prevCenter.y), colour, 2.0);                  
        }
        
    }
    return rgbView;
}

Mat PasteImageBack(Mat &a_image, Mat &a_image2, int left, int right, int top, int bot){
    
    Mat out = a_image.clone();
    
    if (a_image2.type() == CV_8UC1){
        for (int i = 0; i < a_image2.rows; i++) {
            for (int j = 0; j < a_image2.cols ; j++) { 
                out.at<uchar>(i + top,j + left) = a_image2.at<uchar>(i,j);
            }
        }
    } else if (a_image2.type() == CV_8UC3){        
        for (int i = 0; i < a_image2.rows; i++) {
            for (int j = 0; j < a_image2.cols ; j++) { 
                Vec3b pix = a_image2.at<Vec3b>(i,j);
                out.at<Vec3b>(i + top,j + left)[0] = pix[0];
                out.at<Vec3b>(i + top,j + left)[1] = pix[1];
                out.at<Vec3b>(i + top,j + left)[2] = pix[2];                                
            }
        }
    } else if (a_image2.type() == CV_16SC1){

    }
    
    return out;    
}

Mat colourMapFix(Mat &a_image){
    Mat out = a_image.clone();    
    for (int i = 0; i < a_image.rows; i++) {
        for (int j = 0; j < a_image.cols ; j++) { 
            Vec3b pix = a_image.at<Vec3b>(i,j);            
            if ( pix[0] == 0 && pix[1] == 0 && pix[2] == 255){                
                out.at<Vec3b>(i,j)[0] = 0;
                out.at<Vec3b>(i,j)[1] = 0;
                out.at<Vec3b>(i,j)[2] = 0;
            }
        }
    }
    return out;
}
Mat flipFrame(Mat &a_image){
    Mat out = a_image.clone();
    for (int i = 0; i < a_image.rows; i++) {
        for (int j = 0; j < a_image.cols ; j++) { 
            out.at<uchar>(i,j) = a_image.at<uchar>(a_image.rows - i - 1, a_image.cols - j - 1);
        }
    }
    return out;
}

Mat RemoveThreshOfNorm(Mat &a_image, Mat &thresh){
    Mat out = a_image.clone();    
    for (int i = 0; i < a_image.rows; i++) {
        for (int j = 0; j < a_image.cols ; j++) {             
            if ( thresh.at<uchar>(i,j) == 0)
                out.at<uchar>(i,j) = 0;
            else
                out.at<uchar>(i,j) = a_image.at<uchar>(i,j);
        }
    }
    return out;
}

void mouseHandler(int event, int x, int y, int flags, void* param){
    Mat* p = (Mat*) param;
    switch(event){
        ////cout << "Left button down with CTRL pressed" << endl;
        break;
    case CV_EVENT_LBUTTONUP:
        cout << (int) p->at<uchar>(Point(x,y)) << endl;        
//        cout << (int) p->at<Vec3b>(Point(x,y))[0] << " " << (int) p->at<Vec3b>(Point(x,y))[1] << " " << (int) p->at<Vec3b>(Point(x,y))[2] << endl;        
//        cout << (int) x << "," << (int) y << endl;        
        ////cout << "Left button up" << endl;
        break;
    }
}
