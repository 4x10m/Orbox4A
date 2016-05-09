//
// Created by age2pierre on 04/05/16.
//

#ifndef ORBOX_PARAMETERS_H
#define ORBOX_PARAMETERS_H

#include <iostream>
#include <opencv/cv.hpp>

using namespace cv;
using namespace std;

class Parameters {
public :
    Parameters() : debugMode(1), cameraDevice(1),
                   pathToCameraConfig("/home/age2pierre/OrboxWorkspace/Map12.xml") { }

    enum BlurType {NONE, BLUR, MEDIAN, GAUSSIAN};
    enum ThresholdType {ADAPT_MEAN, ADAPT_GAUSSIAN, OTSU, TRIANGLE, CONSTANT};
    enum FeatureType {ORB, BRISK, AKAZE};

    void write(FileStorage &fs) const;

    void read(const FileNode &node);

/* ==== Getters ==== */
    BlurType getBlurType() const {
        return blurType;
    }

    ThresholdType getThresholdType() const {
        return thresholdType;
    }

    const Mat &getCameraMatrix() const {
        return cameraMatrix;
    }

    const Mat &getDistortionCoefficients() const {
        return distortionCoefficients;
    }

    const string &getPathToCameraConfig() const {
        return pathToCameraConfig;
    }

    int getCameraDevice() const {
        return cameraDevice;
    }

    bool isDebugMode() const {
        return debugMode;
    }

    FeatureType getFeatureType() const {
        return featureType;
    }

    bool isFlannBased() const {
        return flannBased;
    }

    const vector<string> &getClusterFiles() const {
        return clusterFiles;
    }

    int getErosion() const {
        return erosion;
    }

    int getDilatation() const {
        return dilatation;
    }

    bool isUseNuClassification() const {
        return useNuClassification;
    }

    double getSvmParamC() const {
        return svmParamC;
    }

    double getSvmParamNu() const {
        return svmParamNu;
    }

    double getSvmParamGamma() const {
        return svmParamGamma;
    }

    bool isSvmAutoParam() const {
        return svmAutoParam;
    }

    const string &getFeatureTypeString() const {
        return featureTypeString;
    }

    bool isEqualizeHist() const {
        return equalizeHist;
    }

private:
    string pathToCameraConfig;
    int cameraDevice;
    bool debugMode;
    int erosion;
    int dilatation;
    Mat cameraMatrix;
    Mat distortionCoefficients;
    BlurType blurType;
    string blurTypeString;
    ThresholdType thresholdType;
    string thresholdTypeString;
    FeatureType featureType;
    string featureTypeString;
    bool flannBased;
    vector<string> clusterFiles;
    bool useNuClassification;
    bool svmAutoParam;
    double svmParamC;
    double svmParamNu;
    double svmParamGamma;
    bool equalizeHist;

};


static inline void read(const FileNode &node, Parameters &x, const Parameters &default_value = Parameters()) {
    if (node.empty())
        x = default_value;
    else
        x.read(node);
}

static inline void write(FileStorage &fs, const String &, const Parameters &s) {
    s.write(fs);
}


#endif //ORBOX_PARAMETERS_H
