#include "ofApp.h"
//--------------------------------------------------------------
void ofApp::setup() {
    
    
    oculusRift.baseCamera = &easyCam;
    oculusRift.setup();
    easyCam.begin();
    easyCam.end();
    
    
    //ofSetLogLevel(OF_LOG_VERBOSE);
    ofSetVerticalSync( true );
    
    // enable depth->video image calibration
    kinect.setRegistration(false);// set true for alignment of rgb and depth
    //kinect.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    kinect.init(false, false); // disable video image (faster fps)
    kinect.open();		// opens first available kinect
    
    
    //debugWindow.setup("debug Window", 0, 0, 1280/2, 960/2, false);
    
    //setting up the kinect CV images
    nearThreshold = 230;
    farThreshold = 10;
    minArea = 1000;
    maxArea = 70000;
    threshold = 15;
    persistence = 15;
    maxDistance = 32;
    kLeftThreshold = -1000;
    kRightThreshold = 1000;
    kTopThreshold = 1000;
    kBottomThreshold = -1000;
    kFrontThreshold = 0;
    kBackThreshold = 6000;
    colorMapping =100;
    
    
    kinectDepthImage.allocate(kinect.width, kinect.height);
    kinectThresholdedImage.allocate(kinect.width, kinect.height);
    
    
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
#ifdef USE_TWO_KINECTS
    kinect2.init();
    kinect2.open();
#endif
    
    
    ofSetFrameRate(60);
    
    // zero the tilt on startup
    angle = 0;
    kinect.setCameraTiltAngle(angle);
    
    // start from the front
    bDrawPointCloud = true;
    kinect.update();
    mesh.setMode(OF_PRIMITIVE_POINTS);
    trackingMesh.setMode(OF_PRIMITIVE_POINTS);
    tempMesh.setMode(OF_PRIMITIVE_POINTS);
    
    
    for ( int i = 0 ; i< w ; i++)
    {
        for ( int j = 0 ; j< h ; j++)
        {
            ofColor c = kinect.getColorAt(i, j);
            mesh.addColor(c);
            mesh.addVertex(kinect.getWorldCoordinateAt(i, j)*10);
        }
    }
    int noOfVertices = mesh.getNumVertices();
    cout<< "number of vertices are:"<< noOfVertices;
    
    
    /*
     
     //meshCopy.clear();
     //meshCopy.setMode(OF_PRIMITIVE_POINTS);
     meshCopy.enableIndices();
     meshCopy.setMode(OF_PRIMITIVE_LINES);
     
     
     for ( int i = 0 ; i< mesh.getNumVertices(); i+=step)
     
     {
     ofColor c;
     c.set(0, 100, 255);
     meshCopy.addColor(c);
     meshCopy.addVertex( mesh.getVertex(i));
     
     }
     
     cout<<"meshcopy vetices are: "<< meshCopy.getNumVertices();
     
     
     int height = h;
     int width = w;
     
     for (int y = 0; y<height-1; y++){
     for (int x=0; x<width-1; x++){
     mesh.addIndex(x+y*width);               // 0
     mesh.addIndex((x+1)+y*width);           // 1
     mesh.addIndex(x+(y+1)*width);           // 10
     
     mesh.addIndex((x+1)+y*width);           // 1
     mesh.addIndex((x+1)+(y+1)*width);       // 11
     mesh.addIndex(x+(y+1)*width);           // 10
     }
     }
     */
    
    // setup gui
    gui1 = new ofxUISuperCanvas("PANEL 1: OpenCV");
    gui1->setHeight(800);
    gui1->setName("parameters");
    gui1->addLabel("kinect");
    gui1->addSpacer();
    gui1->addSlider("nearThresh", 0, 255, &nearThreshold);
    gui1->addSlider("farThresh", 0, 255, &farThreshold);
    gui1->addLabel("contours");
    gui1->addSpacer();
    gui1->addSlider("minArea", 0, 5000, &minArea);
    gui1->addSlider("maxArea", 5000, 500000, &maxArea);
    gui1->addSlider("threshold", 1, 100, &threshold);
    gui1->addSlider("persistence", 1, 100, &persistence);
    gui1->addSlider("maxDistance", 1, 100, &maxDistance);
    gui1->addSlider("front Threshold", 0, 6000, &kFrontThreshold);
    gui1->addSlider("back Threshold", 0, 10000, &kBackThreshold);
    gui1->addSlider("top Threshold", 0, 1000, &kTopThreshold);
    gui1->addSlider("bottom Threshold", -1000, 0, &kBottomThreshold);
    gui1->addSlider("left Threshold", 0, 1000, &kLeftThreshold);
    gui1->addSlider("right Threshold", -1000, 0, &kRightThreshold);
    gui1->addSlider("colorMapping", 10, 2000, &colorMapping);
    
    gui1->addToggle("isTrackingOn", false);
    
    gui1->setWidgetColor(OFX_UI_WIDGET_COLOR_FILL, ofColor(0,128,192,200));
    gui1->setWidgetColor(OFX_UI_WIDGET_COLOR_FILL_HIGHLIGHT, ofColor(0,128,128, 200));
    
    
    
    
    
}

//--------------------------------------------------------------
void ofApp::update() {
    
    easyCam.setPosition(xPos, yPos, zPos);
    ofBackground(100, 100, 100);
    
    mesh.clearVertices();
    mesh.clearColors();
    
    kinect.update();
    //trackingMesh.append(mesh);
    for ( int i = 0 ; i< w ; i++)
    {
        for ( int j = 0 ; j< h ; j++)
        {
            bool isItIn = false;
            isItIn = checkPointWithinLimits( kinect.getWorldCoordinateAt(i, j) );
            if( isItIn == true)
            {
                ofColor c ;
                c.setHsb(kinect.getWorldCoordinateAt(i, j).z /colorMapping, 255, 255 );
                
                //mesh.setColor(j*w+i ,c);
                //mesh.setVertex( j* w + i  , kinect.getWorldCoordinateAt(i, j) );
                
                mesh.addColor(c);
                mesh.addVertex(kinect.getWorldCoordinateAt(i, j) );
                if( writingToFile)
                    addPointsToFile(kinect.getWorldCoordinateAt(i, j));
                //cout<<kinect.getWorldCoordinateAt(i, j) <<endl;
            }
            
            //if( isTrackingOn && isItIn)
            if(record && isItIn)
            {
                tempMesh.addVertex(kinect.getWorldCoordinateAt(i, j));
                
            }
            
        }
        
    }
    
    if ( isTrackingOn)
    {
        trackingMesh.append(mesh);
        tempMesh.clearVertices();
    }
    
    
    
    record = false;
    kinectDepthImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
    kinectThresholdedImage = kinectDepthImage;
    
    kinectThresholdedImage.threshold(nearThreshold,true);
    kinectThresholdedImage.threshold(farThreshold,true);
    contourFinder.setMinArea(minArea);
    contourFinder.setMaxArea(maxArea);
    contourFinder.setThreshold(threshold);
    contourFinder.getTracker().setPersistence(persistence);
    contourFinder.getTracker().setMaximumDistance(maxDistance);
    
    // determine found contours
    contourFinder.findContours(kinectThresholdedImage);
    
    
    //--------oculus stuff
    if(oculusRift.isSetup()){
        ofRectangle viewport = oculusRift.getOculusViewport();
        
        /*
        for(int i = 0; i < demos.size(); i++){
            // mouse selection
            float mouseDist = oculusRift.distanceFromMouse(demos[i].floatPos);
            demos[i].bMouseOver = (mouseDist < 50);
            
            // gaze selection
            ofVec3f screenPos = oculusRift.worldToScreen(demos[i].floatPos, true);
            float gazeDist = ofDist(screenPos.x, screenPos.y,
                                    viewport.getCenter().x, viewport.getCenter().y);
            demos[i].bGazeOver = (gazeDist < 25);
        }
         
         */
        
    }
    //------ end of oculus stuff
    
    
    
#ifdef USE_TWO_KINECTS
    kinect2.update();
#endif
}




//--------------------------------------------------------------
void ofApp::draw() {
    /*
    kinectDepthImage.draw(640,480);
    kinectThresholdedImage.draw(640,0);
    ofPushMatrix();
    ofTranslate(640,0);
    contourFinder.draw();
    ofPopMatrix();
    
    ofDrawBitmapString( ofToString(record) , 20, 400);
    debugWindow.begin();
    ofBackground(0, 0, 0);
    glPointSize(1);
    //glLineWidth(1);
    easyCam.begin();
    ofPushMatrix();
    ofScale(1, -1, -1);
    //ofTranslate( 0 ,0 ,0);
    
    //ofTranslate(ofGetWidth()/2, ofGetHeight()/2);
    ofEnableDepthTest();
    mesh.draw();
    //kinect.draw(0, 0, 640, 480);
    //kinect.drawDepth(0, 0, 640, 480);
    
    
    ofDisableDepthTest();
    ofPopMatrix();
    easyCam.end();
    
    debugWindow.end();
    
    */
    
    
    
    //-----oculus stufff
     if(oculusRift.isSetup()){
     
     if(showOverlay){
     
     oculusRift.beginOverlay(-230, 320,240);
     ofRectangle overlayRect = oculusRift.getOverlayRectangle();
     
     ofPushStyle();
     ofEnableAlphaBlending();
     ofFill();
     ofSetColor(255, 40, 10, 200);
     
     ofRect(overlayRect);
     
     ofSetColor(255,255);
     ofFill();
     ofDrawBitmapString("ofxOculusRift by\nAndreas Muller\nJames George\nJason Walters\nElie Zananiri\nFPS:"+ofToString(ofGetFrameRate())+"\nPredictive Tracking " + (oculusRift.getUsePredictiveOrientation() ? "YES" : "NO"), 40, 40);
     
     ofSetColor(0, 255, 0);
     ofNoFill();
     ofCircle(overlayRect.getCenter(), 20);
     
     ofPopStyle();
     oculusRift.endOverlay();
     }
     
     ofSetColor(255);
     glEnable(GL_DEPTH_TEST);
     
     
     oculusRift.beginLeftEye();
     drawScene();
     oculusRift.endLeftEye();
     
     oculusRift.beginRightEye();
     drawScene();
     oculusRift.endRightEye();
     
     oculusRift.draw();
     
     glDisable(GL_DEPTH_TEST);
     }
        /*
     else{
     easyCam.begin();
     drawScene();
     easyCam.end();
     }

     */
     
     //---- end of oculus stuff
    
    
    
    
}

//--------------------------------------------------------------
void ofApp::drawScene()
{
    ofPushStyle();
    //billboard and draw the mouse
   //if(oculusRift.isSetup())
    {
        /*
        ofPushMatrix();
        oculusRift.multBillboardMatrix();
        ofSetColor(255, 0, 0);
        ofCircle(0,0,.5);
        ofPopMatrix();
        */
        
        
        ofScale(1, -1, -1);
        //ofTranslate( 0 ,0 ,0);
        
        //ofTranslate(ofGetWidth()/2, ofGetHeight()/2);
        ofEnableDepthTest();
        mesh.draw();
        //kinect.draw(0, 0, 640, 480);
        //kinect.drawDepth(0, 0, 640, 480);
        
        
        ofDisableDepthTest();
        ofPopMatrix();

        
    }
    
    ofPopStyle();
    
}

//--------------------------------------------------------------


//--------------------------------------------------------------
void ofApp::exit() {
    kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
    
#ifdef USE_TWO_KINECTS
    kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
    
    
    if( key == 'i')
        yPos+=5;
    if( key == 'j')
        xPos-=5;
    if ( key == 'k')
        yPos-=5;
    if(key == 'l')
        xPos+=5;
    
    if(key == 'q')
        zPos+=5;
    if(key == 'a')
        zPos-=5;

    switch (key) {
            
     
        case'm':
            recordMesh();
            break;
        case 'r':
            record =!record;
            break;
            
        case'p':
            bDrawPointCloud = !bDrawPointCloud;
            break;
            
        case 'o':
            kinect.setCameraTiltAngle(angle); // go back to prev tilt
            kinect.open();
            break;
            
        case 'c':
            kinect.setCameraTiltAngle(0); // zero the tilt
            kinect.close();
            break;
            
        case '1':
            kinect.setLed(ofxKinect::LED_GREEN);
            break;
            
        case '2':
            kinect.setLed(ofxKinect::LED_YELLOW);
            break;
            
        case '3':
            kinect.setLed(ofxKinect::LED_RED);
            break;
            
        case '4':
            kinect.setLed(ofxKinect::LED_BLINK_GREEN);
            break;
            
        case '5':
            kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
            break;
            
        case '0':
            kinect.setLed(ofxKinect::LED_OFF);
            break;
            
        case OF_KEY_UP:
            angle++;
            if(angle>30) angle=30;
            kinect.setCameraTiltAngle(angle);
            break;
            
        case OF_KEY_DOWN:
            angle--;
            if(angle<-30) angle=-30;
            kinect.setCameraTiltAngle(angle);
            break;
    }
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{}






bool ofApp::checkPointWithinLimits(ofVec3f point)
{
    bool itsIn = false;
    //if( point.x > kLeftThreshold && point.x < kRightThreshold && point.y < kTopThreshold && point.y > kBottomThreshold && point.z > kFrontThreshold && point.z < kBackThreshold )
    if( point.z > kFrontThreshold && point.z < kBackThreshold )
    {
        itsIn = true;
    }
    
    return itsIn;
}


/*
 
 void saveSceneConfig(std::string name, Scene *s){
 ofFile file(name + ".txt", ofFile::WriteOnly);
 file << s->x << " " << s->y << " ";
 file << s->scale << " " << s->scalex << " " << s->scaley << " ";
 file << s->rotate << " ";
 file << s->r << " " << s->g << " " << s->b << " " << s->brightness;
 };
 
 */


/*
 
 
 
 void ofApp::createNewPointFile()
 {
 string name = ofGetTimestampString();
 ofFile file(name + ".txt", ofFile::WriteOnly);
 writingToFile = true;
 }
 
 
 void ofApp::addPointsToFile(ofVec3f point)
 {
 string name = ofGetTimestampString();
 ofFile file(name + ".txt", ofFile::WriteOnly);
 file << "v(" << point.x << "," << point.y << "," << point.z << ")" <<endl;
 
 }
 
 
 void ofApp::stopFileWrite()
 {
 writingToFile = false;
 }
 
 
 
 
 void createNewPointFile();
 void addPointsToFile();
 void stopFileWrite();
 bool writingToFile = false;
 */






void ofApp::createNewPointFile()
{
    string name = ofGetTimestampString();
    ofFile file(name + ".txt", ofFile::WriteOnly);
    writingToFile = false;
}


void ofApp::addPointsToFile(ofVec3f point)
{
    string name = ofGetTimestampString();
    ofFile file(name + ".txt", ofFile::WriteOnly);
    file << "v(" << point.x << "," << point.y << "," << point.z << ")" <<endl;
    
}


void ofApp::stopFileWrite()
{
    writingToFile = false;
}





void ofApp::recordMesh()
{
    trackingMesh.save( "generativeMesh/mesh.ply");
}








