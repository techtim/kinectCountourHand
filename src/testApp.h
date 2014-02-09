#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"
#include "ofxGui.h"

#define UDP_POINTS_ID 1
#define UDP_MAX_POINT 20
#define UDP_MAX_PACKET 1024
#define HOST "localhost"
#define PORT 1337

#define OFFSET_FROM_TOP_POINT 10 // offset from coutour highest point to get raw depth from kinect

class Glow : public ofxCv::RectFollower {
protected:
	ofColor color;
	ofVec2f cur, smooth;
    ofVec3f pos;
	float startedDying;
	ofPolyline all;
public:
	Glow()
		:startedDying(0) {
	}
	void setup(const cv::Rect& track);
	void update(const cv::Rect& track);
	void kill();
	void draw();
    void getPos() { return cur; };
};

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	
    void exit();
    
    ofPoint getHighestContourPoint(int i);
    
//	ofVideoPlayer movie;
	ofxCv::ContourFinder contourFinder;
    ofxCv::ObjectFinder objFinder;
	ofxCv::RectTrackerFollower<Glow> tracker;
    
    ofxKinect kinect;
    cv::Mat grayImageMat;
    cv::Mat rgbImageMat;

    ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    
    ofxPanel gui;
    
    ofParameter<int> MinAreaRadius;
    ofParameter<int> MaxAreaRadius;
    ofParameter<int> MinArea;
    ofParameter<int> DetectionHeight;
    ofParameter<int> Treshold;
    ofParameter<int>kinectAngle;
//    ofParameter<ofColor> color;
//    ofParameter<ofVec2f> center;
    ofParameter<int> circleResolution;
    
    // --- OUTPUT ---
    ofxOscSender oscSender;
    bool udpAvailable;
    ofVec3f ** triGrabPoints;
};
