//
// Created by age2pierre on 03/05/16.
//

#include <ml.h>
#include "Application.h"

Application::Application(String pathToConfigFile) {
    this->pathToConfigFile = pathToConfigFile;
}

void Application::init() {
    cout << "\tReading parameters from configuration file ..." << endl;
    FileStorage configFile(this->pathToConfigFile, FileStorage::READ);
    if (!configFile.isOpened()) {
        cout << "ERROR : Could not open the configuration file: \"" << pathToConfigFile << "\"" << endl;
        return;
    }
    else {
        configFile["Parameters"] >> parameters;
        configFile.release();

        FileStorage fs(parameters.getPathToCameraConfig(), FileStorage::READ);
        if (!fs.isOpened()) {
            cout << "WARNING : Could not open the camera configuration file: \"" <<
            parameters.getPathToCameraConfig() << "\"" <<
            endl;
            calculateCameraMatrices();
        }
        else {
            fs["map_1"] >> map1;
            fs["map_2"] >> map2;
            fs.release();
            cout << "\tOK" << endl;
        }
    }

    this->videoCapture.open(parameters.getCameraDevice());
    if (!this->videoCapture.set(CV_CAP_PROP_AUTOFOCUS, 0))
        cout << "ERROR : Failed to set autofocus OFF" << endl;
    if (!this->videoCapture.set(CV_CAP_PROP_FRAME_WIDTH, 1920))
        cout << "ERROR : Failed to set frame width to 1920" << endl;
    if (!this->videoCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080))
        cout << "ERROR : Failed to set frame height to 1080" << endl;

}

void Application::run(String path, int number) {
    /*
     * Step 1 : Get input pictures from camera
     *          One picture w/ light ON and one w/ light OFF
     */
    Mat tempOn, tempOff;

    if (!videoCapture.isOpened()) {
        cout << "ERROR : camera flux is not open!";
        return;
    }
    else if (parameters.isDebugMode()) {
        namedWindow("DEBUG");
        Mat frame;
        while (1) {
            videoCapture >> frame;
            imshow("DEBUG", frame);
            if (waitKey(30) >= 0) {
                videoCapture >> tempOff;
                break;
            }
        }
        while (1) {
            videoCapture >> frame;
            imshow("DEBUG", frame);
            if (waitKey(30) >= 0) {
                videoCapture >> tempOn;
                break;
            }
        }
        destroyAllWindows();
    }
    else {
        videoCapture >> tempOff;
        //TODO turn on light with Rpi and delay;
        videoCapture >> tempOn;
        //TODO turn off light with Rpi;
    }

    /*
     * Step 2 : Pre-processing both pictures
     *          Cropped and undistorted
     */
    Mat picLightOn, picLightOff;
    this->preProcessing(tempOn, picLightOn);
    this->preProcessing(tempOff, picLightOff);

    /*
     * Step 3 : Segmentation - finds objects put on top of the box
     */
    vector<Mat> segmentResult;
    this->segmentation(picLightOn, picLightOff, segmentResult);

    if (parameters.isDebugMode()) {
        int j(0);
        for (Mat obj : segmentResult)
            imwrite(format((path + "SEG_%02d_FROM_%03d.png").c_str(), j++, number), obj);
    }
}

void Application::close() {
    FileStorage configFile(this->pathToConfigFile, FileStorage::WRITE);
    if (!configFile.isOpened())
        cout << "ERROR : Could not open the configuration file: \"" << pathToConfigFile << "\" while trying to save." <<
        endl;
    else {
        cout << "\tSaving parameters to the configuration file..." << endl;
        configFile << parameters;
        configFile.release();
        cout << "\tOK" << endl;
    }
}

void Application::calculateCameraMatrices() {
    cout << "\tCalculating Map1 and Map2 from Camera Matrix and Distortion Coefficients ..." << endl;
    Mat map1, map2, newCamMat;
    fisheye::estimateNewCameraMatrixForUndistortRectify(parameters.getCameraMatrix(),
                                                        parameters.getDistortionCoefficients(),
                                                        cvSize(1080, 1080),
                                                        Matx33d::eye(),
                                                        newCamMat,
                                                        1);
    fisheye::initUndistortRectifyMap(parameters.getCameraMatrix(),
                                     parameters.getDistortionCoefficients(),
                                     Matx33d::eye(),
                                     newCamMat,
                                     cvSize(1080, 1080),
                                     CV_16SC2,
                                     map1,
                                     map2);
    FileStorage fs(parameters.getPathToCameraConfig(), FileStorage::WRITE);
    fs << "map_1" << map1;
    fs << "map_2" << map2;
    fs.release();
    cout << "\tOK" << endl;
}

void Application::preProcessing(Mat &input, Mat &output) {
    output = input(Rect(460, 0, 1080, 1080));
    input = output.clone();
    remap(input, output, map1, map2, INTER_LANCZOS4);
}

void Application::segmentation(Mat &inputOn, Mat &inputOff, vector<Mat> &output) {
    Mat diff, blurred, bin, hsv;
    vector<Mat> hsv_split;

    // subtract the two images, only objects lit by the LEDs will appear
    absdiff(inputOff, inputOn, diff);

    // once converted to HSV, only the value channel will be used
    // as it contained information on how much light a pixel get
    cvtColor(diff, hsv, CV_BGR2HSV);
    split(hsv, hsv_split);

    // blurring is used to get better result for thresholding
    switch (parameters.getBlurType()) {
        case Parameters::BlurType::BLUR :
            blur(hsv_split[2], blurred, Size(15, 15));
            break;
        case Parameters::BlurType::GAUSSIAN :
            GaussianBlur(hsv_split[2], blurred, Size(15, 15), 0, 0);
            break;
        case Parameters::BlurType::MEDIAN :
            medianBlur(hsv_split[2], blurred, 15);
            break;
        case Parameters::BlurType::NONE :
        default:
            blurred = hsv_split[2];
            break;
    }

    // thresholding applied to get a binary map
    switch (parameters.getThresholdType()) {
        case Parameters::ThresholdType::ADAPT_GAUSSIAN :
            adaptiveThreshold(blurred, bin, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 20, 2);
            break;
        case Parameters::ThresholdType::ADAPT_MEAN :
            adaptiveThreshold(blurred, bin, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 20, 2);
            break;
        case Parameters::ThresholdType::OTSU :
            threshold(blurred, bin, 0, 255, THRESH_OTSU);
            break;
        case Parameters::ThresholdType::TRIANGLE :
            threshold(blurred, bin, 0, 255, THRESH_TRIANGLE);
            break;
        default:
        case Parameters::ThresholdType::CONSTANT :
            threshold(blurred, bin, 60, 255, THRESH_BINARY);
            break;
    }

    // erosion and & dilation applied to clean the binary map
    erode(bin, bin, getStructuringElement(MORPH_RECT, Size(parameters.getErosion(), parameters.getErosion())));
    dilate(bin, bin, getStructuringElement(MORPH_RECT, Size(parameters.getDilatation(), parameters.getDilatation())));

    //find the bounding rectangle of each white spot
    vector<vector<Point>> contours;
    vector<Rect> boundRect;
    findContours(bin, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    for (vector<Point> contour : contours)
        boundRect.push_back(boundingRect(Mat(contour)));

    for (Rect rect : boundRect) {
        Mat currentShape;
        currentShape = inputOn(rect);
        output.push_back(currentShape);
    }
}

void Application::training() {
    Ptr<DescriptorMatcher> matcher;
    Ptr<BOWTrainer> trainer;

    cout << "Training ..." << endl;

    switch (parameters.getFeatureType()) {
        case Parameters::FeatureType::AKAZE :
            feature2d = cv::AKAZE::create();
            break;
        case Parameters::FeatureType::BRISK :
            feature2d = cv::BRISK::create();
            break;
        case Parameters::FeatureType::ORB :
        default:
            feature2d = cv::ORB::create();
            break;
    }

    if (parameters.isFlannBased())
        matcher = makePtr<FlannBasedMatcher>();
    else
        matcher = makePtr<BFMatcher>(NORM_HAMMING);

    trainer = makePtr<BOWKmajorityTrainer>(parameters.getClusterFiles().size());
    extractor = makePtr<BOWImgDescriptorExtractor>(feature2d, matcher);

    /*
     * Train BOW using Features2D features
     */
    cout << "... training BOW ..." << endl;

    for (string pathToClusterFile : parameters.getClusterFiles()) {
        FileStorage fileStorage(pathToClusterFile, FileStorage::READ);
        if (fileStorage.isOpened()) {
            FileNode fileNode = fileStorage["pics_path_training_bow"];
            if (fileNode.type() == FileNode::SEQ) {
                FileNodeIterator iterator1 = fileNode.begin();
                FileNodeIterator iterator2 = fileNode.end();
                for (; iterator1 != iterator2; iterator1++) {
                    Mat temp, im, features;
                    vector<KeyPoint> keyPoints;
                    if (parameters.isEqualizeHist()) {
                        temp = imread(*iterator1, IMREAD_GRAYSCALE);
                        equalizeHist(temp, im);
                    }
                    else
                        im = imread(*iterator1, IMREAD_GRAYSCALE);
                    feature2d->detectAndCompute(im, Mat(), keyPoints, features);
                    if (!features.empty())
                        trainer->add(features);
                }
            }
            else {
                cout << "ERROR : the file \"" << pathToClusterFile <<
                "\" does not contain a sequence <pics_path_training_bow>." <<
                endl;
            }
            fileStorage.release();
        }
        else
            cout << "ERROR : Could not open the file : " << pathToClusterFile << endl;
    }
    Mat vocab = trainer->cluster();
    extractor->setVocabulary(vocab);
    cout << "\tBOW Vocab size : " << vocab.size() << endl;
    cout << "\tFeautures used : " << parameters.getFeatureTypeString() << endl;
    cout << "\tEqualize histogram : " << (parameters.isEqualizeHist()?"Yes":"No") << endl;

    /*
     *  Gather data and then train SVM
     */
    cout << "... training SVM ..." << endl;

    Mat svmTrainData;
    Mat svmTrainLabels;
    for (string pathToClusterFile : parameters.getClusterFiles()) {
        FileStorage fileStorage(pathToClusterFile, FileStorage::READ);
        if (fileStorage.isOpened()) {
            int label;
            fileStorage["label"] >> label;
            FileNode fileNode = fileStorage["pics_path_training_svm"];
            if (fileNode.type() == FileNode::SEQ) {
                FileNodeIterator iterator1 = fileNode.begin();
                FileNodeIterator iterator2 = fileNode.end();
                for (; iterator1 != iterator2; iterator1++) {
                    Mat im, temp, bowFeatures;
                    vector<KeyPoint> keyPoints;
                    if (parameters.isEqualizeHist()) {
                        temp = imread(*iterator1, IMREAD_GRAYSCALE);
                        equalizeHist(temp, im);
                    }
                    else
                        im = imread(*iterator1, IMREAD_GRAYSCALE);
                    feature2d->detect(im, keyPoints);
                    extractor->compute2(im, keyPoints, bowFeatures);
                    if (!bowFeatures.empty()) {
                        svmTrainData.push_back(bowFeatures);
                        svmTrainLabels.push_back(label);
                    }
                }
            }
            else {
                cout << "ERROR : the file \"" << pathToClusterFile <<
                "\" does not contain a sequence <pics_path_training_svm>." <<
                endl;
            }
            fileStorage.release();
        }
        else
            cout << "ERROR : Could not open the file : " << pathToClusterFile << endl;
    }
    cout << "\tSVM data size : " << svmTrainData.size() << endl;
    cout << "\tSVM labels size : " << svmTrainLabels.size() << endl;

    svm = SVM::create();
    svm->setKernel(SVM::RBF);
    if (parameters.isUseNuClassification())
        svm->setType(SVM::NU_SVC);
    else
        svm->setType(SVM::C_SVC);
    svm->setC(parameters.getSvmParamC());
    svm->setNu(parameters.getSvmParamNu());
    svm->setGamma(parameters.getSvmParamGamma());

    Ptr<TrainData> trainData = TrainData::create(svmTrainData, ROW_SAMPLE, svmTrainLabels);

    bool res;
    if (parameters.isSvmAutoParam())
        res = svm->trainAuto(trainData);
    else
        res = svm->train(trainData);

    if (!res)
        cout << "ERROR : SVM training failed !";
    else {
        cout << "\tSVM type :" <<
        ((svm->getType() == SVM::NU_SVC) ? "Nu-Support Vector Classification" : "C-Support Vector Classification") <<
        endl;
        cout << "\tSVM C param : " << svm->getC() << endl;
        cout << "\tSVM Nu param : " << svm->getC() << endl;
        cout << "\tSVM Gamma param : " << svm->getGamma() << endl;
        cout << "OK\n" << endl;
    }


}

void Application::testing() {
    for (string pathToClusterFile : parameters.getClusterFiles()) {
        FileStorage fileStorage(pathToClusterFile, FileStorage::READ);
        if (fileStorage.isOpened()) {
            int label;
            fileStorage["label"] >> label;
            FileNode fileNode = fileStorage["pics_path_testing"];
            if (fileNode.type() == FileNode::SEQ) {
                FileNodeIterator iterator1 = fileNode.begin();
                FileNodeIterator iterator2 = fileNode.end();
                cout << "Prediction for " << label << " :" << endl;
                int total(0), good(0), foundFeatures(0);
                for (; iterator1 != iterator2; iterator1++) {
                    Mat im, temp, bowFeatures;
                    vector<KeyPoint> keyPoints;
                    if (parameters.isEqualizeHist()) {
                        temp = imread(*iterator1, IMREAD_GRAYSCALE);
                        equalizeHist(temp, im);
                    }
                    else
                        im = imread(*iterator1, IMREAD_GRAYSCALE);
                    feature2d->detect(im, keyPoints);
                    extractor->compute2(im, keyPoints, bowFeatures);
                    total++;
                    if (!bowFeatures.empty()) {
                        foundFeatures++;
                        if (svm->predict(bowFeatures) == label)
                            good++;
                    }
                }

                cout << format("\tTotal %3d\n", total) <<
                format("\tFfeat %3d ( %3.1f %%)\n", foundFeatures, 100 * ((double) foundFeatures) / total) <<
                format("\tGood  %3d ( %3.1f %%)", good, 100 * ((double) good) / foundFeatures) << endl;
            }
            else {
                cout << "ERROR : the file \"" << pathToClusterFile <<
                "\" does not contain a sequence <pics_path_training>." <<
                endl;
            }
            fileStorage.release();
        }
        else
            cout << "ERROR : Could not open the file : " << pathToClusterFile << endl;
    }
}





