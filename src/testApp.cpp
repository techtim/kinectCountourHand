#include "testApp.h"

using namespace ofxCv;
using namespace cv;

const float dyingTime = 1;

void testApp::setup() {
	ofSetVerticalSync(true);
	ofBackground(0);
	ofSetFrameRate(30);
    
    kinect.setRegistration(true);
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
//	kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();

    kinect.setLed(ofxKinect::LED_OFF);
    gui.setup("panel");
    gui.add(MinAreaRadius.set( "minAreaRad", 20,1, 200 ));
    gui.add(MaxAreaRadius.set( "maxAreaRad", 100,10, 200 ));
//    gui.add(MinArea.set( "minArea", 1,1, 200 ));
    gui.add(DetectionHeight.set( "detection height", 300,10, 480 ));
    gui.add(Treshold.set( "Treshold", 150,1, 250 ));
    gui.add(kinectAngle.set("ANGLE", -3, -30, 30));
    gui.setPosition(ofPoint(600, 10));
    
    gui.loadFromFile("gui_conf.xml");
    
    contourFinder.setMinAreaRadius(MinAreaRadius);
	contourFinder.setMaxAreaRadius(MaxAreaRadius);

	contourFinder.setThreshold(Treshold); // min Z value

//    contourFinder.setTargetColor(TRACK_COLOR_HSV); //TRACK_COLOR_RGB, TRACK_COLOR_HSV, TRACK_COLOR_H, TRACK_COLOR_HS};
//        contourFinder.set
	// wait for half a frame before forgetting something
//	tracker.setPersistence(15);
	// an object can move up to 50 pixels per frame
	tracker.setMaximumDistance(100);
    
    // --- OUTPUT ---
    oscSender.setup(HOST, PORT);
}

void testApp::update() {

    kinect.setCameraTiltAngle(kinectAngle);
    kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
        // load grayscale depth image from the kinect source

        grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		grayImageMat = Mat(DetectionHeight, kinect.width, CV_8UC1, kinect.getDepthPixels(), 0);

        blur(grayImageMat, 15);
        contourFinder.setMinAreaRadius(MinAreaRadius);
        contourFinder.setMaxAreaRadius(MaxAreaRadius);
        contourFinder.setThreshold(Treshold); // min Z value
        
		contourFinder.findContours(grayImageMat);

		tracker.track(contourFinder.getBoundingRects());
//        vector<Glow>& followers = tracker.getFollowers();

        if (contourFinder.size() > 0) {

            ofVec3f * pos = new ofVec3f [contourFinder.size()];
            ofVec3f * vel = new ofVec3f [contourFinder.size()];
            unsigned int * labels = new unsigned int [contourFinder.size()];
            for(int i = 0; i < contourFinder.size(); i++) {
                ofRectangle rect = toOf(contourFinder.getBoundingRect(i));
                ofVec2f center = getHighestContourPoint(i);
//                center.interpolate(center, .5);
                labels[i] = contourFinder.getLabel(i);
                vel[i] = toOf(contourFinder.getVelocity(i));
                
                center.y = DetectionHeight - center.y;
                
                pos[i] = ofVec3f(
                    center.x,
                    center.y,
                    kinect.getRawDepthPixels()[static_cast <int> ((center.y+OFFSET_FROM_TOP_POINT)*kinect.width+center.x)]
//                    grayImage.getPixels()[int(center.y*kinect.width+center.x)]
                    //rect.x+rect.width/2,
                    // rect.y-5,
                    //grayImage.getPixels()[int((rect.y-5)*kinect.width+rect.x+rect.width/2)]
                );

                ofxOscMessage m;
                m.setAddress("/kinect/point");
                m.addIntArg(i);
                m.addIntArg(pos[i].x);
                m.addIntArg(pos[i].y);
                m.addIntArg(pos[i].z);
                m.addIntArg(vel[i].x);
                m.addIntArg(vel[i].y);
                oscSender.sendMessage(m);
//                printf("%i: %f x %f x %f \n", i, pos[i].x, pos[i].y, pos[i].z);

            }
            
//            sendUdp(contourFinder.size(), pos, vel, labels);
            
            delete [] pos;
            delete [] vel;
            delete [] labels;
        } else {
            ofxOscMessage m;
            m.setAddress("/kinect/clear");
            m.addIntArg(1);
            oscSender.sendMessage(m);
        }
        
//        for(int i = 0; i < followers.size(); i++) {
//            followers[i].getPos();
//        }
    }
}

ofPoint testApp::getHighestContourPoint(int i) {
    vector<cv::Point> points = contourFinder.getContour(i);
    ofPoint highest = toOf(points[0]);
    for (int i=1; i<points.size(); i++) {
        if (points[i].y < highest.y) {
            highest = toOf(points[i]);
            
        }
    }
//    printf("%f x %f \n", highest.x, highest.y);
//    printf("\n --------- \n");
    return highest;
}

void testApp::draw() {
	ofSetColor(255);

    kinect.drawDepth(0, 0, kinect.width, kinect.height);

//    drawMat(grayImageMat, 0,0,kinect.width,kinect.height);
	contourFinder.draw();

	vector<Glow>& followers = tracker.getFollowers();
    
	for(int i = 0; i < followers.size(); i++) {
		followers[i].draw();
	}
    
    for(int i = 0; i < contourFinder.size(); i++) {
        ofPoint center = toOf(contourFinder.getCenter(i));
        ofPushMatrix();
        ofTranslate(center.x, center.y);
        int label = contourFinder.getLabel(i);
        ofVec2f velocity = toOf(contourFinder.getVelocity(i));
        ofVec3f obj = ofVec3f(center.x, center.y, grayImage.getPixels()[int(center.y*kinect.width+center.x)]);
//        ofVec3f obj = toOf(contourFinder.getCenter(i));
        ofDrawBitmapString(ofToString(obj), 0,0);
        ofScale(5, 5);
        ofLine(0, 0, velocity.x, velocity.y);
        ofPopMatrix();
    }

    for(int i = 0; i < contourFinder.size(); i++) {
        ofRectangle rect = toOf(contourFinder.getBoundingRect(i));
        ofVec2f center = getHighestContourPoint(i);
        center.y += 5;
        ofSetColor(255, 155, 0);
        ofDrawSphere(center, 3);
    }

    
    gui.draw();

    ofDrawBitmapString(ofToString(ofGetFrameRate()), 20, ofGetHeight()-30);
    ofDrawBitmapString("SIZE"+ofToString(contourFinder.size()), 100, ofGetHeight()-30);
}

void testApp::keyPressed(int key){
    switch (key){
        case 's':
            gui.saveToFile("gui_conf.xml");
            break;
        case 'l':
            gui.loadFromFile("gui_conf.xml");
            break;
        default:
            break;
    }
}

void testApp::exit() {
    kinect.close();
}

void Glow::setup(const cv::Rect& track) {
	color.setHsb(ofRandom(0, 255), 255, 255);
	cur = toOf(track).getCenter();
	smooth = cur;
}

void Glow::update(const cv::Rect& track) {
	cur = toOf(track).getCenter();
	smooth.interpolate(cur, .5);
	all.addVertex(smooth);
}

void Glow::kill() {
	float curTime = ofGetElapsedTimef();
	if(startedDying == 0) {
		startedDying = curTime;
	} else if(curTime - startedDying > dyingTime) {
		dead = true;
	}
}

void Glow::draw() {
	ofPushStyle();
	float size = 26;
	ofSetColor(255);
	if(startedDying) {
		ofSetColor(ofColor::red);
		size = ofMap(ofGetElapsedTimef() - startedDying, 0, dyingTime, size, 0, true);
	}
	ofNoFill();
	ofCircle(cur, size);
	ofSetColor(color);
	all.draw();
	ofSetColor(255);
	ofDrawBitmapString(ofToString(label), cur);
	ofPopStyle();
}
