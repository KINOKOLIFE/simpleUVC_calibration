#include "ofApp.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
bool capture_frame = false;
int num_capture = 0;
const int chess_rows = 7;
const int chess_cols = 10;
const double chess_size = 22.5;
std::vector<std::vector<cv::Point2f>> image_points;
cv::Mat intrinsic, distortion;
//--------------------------------------------------------------
void ofApp::setup(){
    camWidth = 1280;  // try to grab at this size.
    camHeight = 720;
    ofCropped.allocate(camWidth, camHeight, OF_IMAGE_COLOR);
    vector<ofVideoDevice> devices = vidGrabber.listDevices();
    for(size_t i = 0; i < devices.size(); i++){
        if(devices[i].bAvailable){
            ofLogNotice() << devices[i].id << ": " << devices[i].deviceName;
        }else{
            ofLogNotice() << devices[i].id << ": " << devices[i].deviceName << " - unavailable ";
        }
    }
    vidGrabber.setDeviceID(0);
    vidGrabber.initGrabber(camWidth, camHeight);
    ofSetVerticalSync(true);
    //---im_gui
    gui.setup();
    ImGui::SetNextWindowPos(ImVec2(0, 100), ImGuiCond_Once);
    intrinsic = cv::Mat::eye(3,3, CV_64F);
}

//--------------------------------------------------------------
void ofApp::update(){
    vidGrabber.update();
    if(vidGrabber.isFrameNew()){
        ofPixels & pixels = vidGrabber.getPixels();
        cv::Mat img, gray;
        img = cv::Mat(camHeight, camWidth, CV_8UC3, pixels.getData());
        if( capture_frame ){
            std::vector<cv::Point2f> corners;
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
            bool found = cv::findChessboardCorners(gray, cv::Size(chess_cols, chess_rows), corners);
            
            if(found &&  (corners.size() == chess_rows * chess_cols) ){
                cv::find4QuadCornerSubpix(gray, corners, cv::Size(3, 3));
                cv::drawChessboardCorners(img, cv::Size(chess_cols, chess_rows), corners, found);
                image_points.push_back(corners);
                num_capture ++;
                capture_frame = false;
            }
        }
        
        ofCropped.setFromPixels(img.ptr(), camWidth, camHeight, OF_IMAGE_COLOR, false);
    }
    if( num_capture == 20){
        num_capture = 0;
        calc();
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofCropped.draw(0, 0);
    draw_gui();
}

//--------------------------------------------------------------
void ofApp::calc(){
    std::cout << "calculating the point coordinates in the object space..." << std::endl;
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<cv::Point3f> points;
    for(int i = 0; i< chess_rows; i++) {
        for(int j = 0; j< chess_cols; j++) {
            points.push_back(cv::Point3f( i * chess_size, j * chess_size, 0.0f));
        }
    }
    object_points.assign(image_points.size(), points);
    std::cout << "calibrating..." << std::endl;
    cv::Mat rvecs, tvecs;
    std::cout << "STEP 2: intrinsic & distortion estimation" << std::endl;
    cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 1024, 1e-9);
    int flags = 0;
    cv::calibrateCamera(object_points, image_points, cv::Size(camWidth, camHeight), intrinsic, distortion, rvecs, tvecs, flags, criteria);
    std::cout << "--- intrinsic ---\n" << intrinsic << std::endl;
    std::cout << "--- distortion ---\n" << distortion << std::endl;
    cv::FileStorage fs("usv_camera.xml", cv::FileStorage::WRITE);
    fs << "intrinsic" << intrinsic;
    fs << "distortion" << distortion;
    fs.release();
}

//--------------------------------------------------------------
void ofApp::draw_gui(){
    gui.begin();
    ImGui::Begin("control"
    , &show_app_about // bool 表示するか否か。ウインドウ右肩のxボタン
    , ImGuiWindowFlags_AlwaysAutoResize // 位置決めのflag
    );//https://qiita.com/ousttrue/items/ae7c8d5715adffc5b1fa
    {
        if(ImGui::IsWindowHovered()){
        }else{
        }
        ImGui::SetNextTreeNodeOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNode("calibration raw image")) {
            if (ImGui::Button(" capture ")) {
                if(!capture_frame){
                    capture_frame = true;
                }
            }
            ImGui::Text("count: %i", num_capture);
            ImGui::TreePop();
        }
        ImGui::End();
    }gui.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
