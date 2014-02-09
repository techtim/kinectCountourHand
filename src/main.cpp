#include "testApp.h"
#include "ofAppGLFWWindow.h"

int main() {
//	ofSetupOpenGL(320, 240, OF_WINDOW);
//	ofRunApp(new testApp());
    ofPtr<ofAppGLFWWindow> win = ofPtr<ofAppGLFWWindow>(new ofAppGLFWWindow());
    win->setStencilBits(8);
	// set width, height, mode (OF_WINDOW or OF_FULLSCREEN)
	ofSetupOpenGL(win, 800, 600, OF_WINDOW);
	ofRunApp(new testApp()); // start the app
}
