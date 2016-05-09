//
// Created by age2pierre on 03/05/16.
//

#ifndef ORBOX_APPLICATION_H
#define ORBOX_APPLICATION_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "Parameters.h"
#include "BOWKmajorityTrainer.h"

using namespace cv;
using namespace cv::ml;

class Application {
private :
    String pathToConfigFile;
    VideoCapture videoCapture;
    Parameters parameters;
    Mat map1, map2;
    Ptr<SVM> svm;
    Ptr<Feature2D> feature2d;
    Ptr<BOWImgDescriptorExtractor> extractor;

    void calculateCameraMatrices();

    void preProcessing(Mat &input, Mat &output);

    void segmentation(Mat &inputOn, Mat &inputOff, vector<Mat> &output);

public:
    Application(String pathToConfigFile);

    void init();

    void run(String path, int number);

    void close();

    void training();

    void testing();

};


#endif //ORBOX_APPLICATION_H
