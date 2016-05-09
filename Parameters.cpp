//
// Created by age2pierre on 04/05/16.
//

#include "Parameters.h"

void Parameters::write(FileStorage &fs) const {
    fs << "{" <<
    "camera_device" << this->cameraDevice <<
    "path_to_camera_config" << this->pathToCameraConfig <<
    "debug_mode" << this->debugMode <<
    "camera_matrix" << this->cameraMatrix <<
    "distortion_coefficients" << this->distortionCoefficients <<
    "blur_type" << this->blurTypeString <<
    "threshold_type" << this->thresholdTypeString <<
    "feature_type" << this->featureTypeString <<
    "flann_based" << this->flannBased <<
    "erosion" << this->erosion <<
    "dilatation" << this->dilatation <<
    "svm_use_nu" << this->useNuClassification <<
    "svm_param_c" << this->svmParamC <<
    "svm_param_nu" << this->svmParamNu <<
    "svm_param_gamma" << this->svmParamGamma <<
    "nb_clusters" << this->nbClusters <<
    "svm_auto_param" << this->svmAutoParam <<
    "equalize_pics" << this->equalizeHist;

    fs << "cluster_files" << "[";
    for (string str : this->clusterFiles)
        fs << str;
    fs << "]";

    fs << "}";
}

void Parameters::read(const FileNode &node) {
    node["camera_device"] >> this->cameraDevice;
    node["path_to_camera_config"] >> this->pathToCameraConfig;
    node["debug_mode"] >> this->debugMode;
    node["camera_matrix"] >> this->cameraMatrix;
    node["distortion_coefficients"] >> this->distortionCoefficients;
    node["blur_type"] >> this->blurTypeString;
    node["threshold"] >> this->thresholdTypeString;
    node["feature_type"] >> this->featureTypeString;
    node["flann_based"] >> this->flannBased;
    node["erosion"] >> this->erosion;
    node["dilatation"] >> this->dilatation;
    node["svm_param_c"] >> this->svmParamC;
    node["svm_param_gamma"] >> this->svmParamGamma;
    node["svm_param_nu"] >> this->svmParamNu;
    node["svm_use_nu"] >> this->useNuClassification;
    node["nb_clusters"] >> this->nbClusters;
    node["svm_auto_param"] >> this->svmAutoParam;
    node["equalize_pics"] >> this->equalizeHist;

    if (node["cluster_files"].type() == FileNode::SEQ) {
        FileNodeIterator iterator1 = node["cluster_files"].begin();
        FileNodeIterator iterator2 = node["cluster_files"].end();
        for (; iterator1 != iterator2; ++iterator1)
            this->clusterFiles.push_back(*iterator1);
    }
    else
        cout << "ERROR : Could not read the sequence <cluster_files>" << endl;

    blurType = BlurType::NONE;
    if (blurTypeString.compare("BLUR") == 0) blurType = BlurType::BLUR;
    else if (blurTypeString.compare("MEDIAN") == 0) blurType = BlurType::MEDIAN;
    else if (blurTypeString.compare("GAUSSIAN") == 0) blurType = BlurType::GAUSSIAN;
    thresholdType = TRIANGLE;
    if (thresholdTypeString.compare("OTSU") == 0) thresholdType = ThresholdType::OTSU;
    else if (thresholdTypeString.compare("ADAPT_GAUSSIAN") == 0) thresholdType = ThresholdType::ADAPT_GAUSSIAN;
    else if (thresholdTypeString.compare("ADAPT_MEAN") == 0) thresholdType = ThresholdType::ADAPT_MEAN;
    else if (thresholdTypeString.compare("CONSTANT") == 0) thresholdType = ThresholdType::CONSTANT;
    featureType = FeatureType::ORB;
    if (featureTypeString.compare("BRISK") == 0) featureType = FeatureType::BRISK;
    else if (featureTypeString.compare("AKAZE") == 0) featureType = FeatureType::AKAZE;

}



