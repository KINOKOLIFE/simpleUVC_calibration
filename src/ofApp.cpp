#include "ofApp.h"

bool capture_frame = false;
bool use_guess = false;
double fxp, fyp;
int num_capture = 0;
const int chess_rows = 6;
const int chess_cols = 9;
const double chess_size = 22.5;
std::vector<std::vector<cv::Point2f>> image_points;
cv::Mat intrinsic, distortion;
void ofApp::shot(bool &b){
    capture_frame =  true;
}
ofRectangle area = ofRectangle(0,360,1280,360);
vector<cv::Vec3d> rvecsarray, tvecsarray;



//--------------------------------------------------------------
void ofApp::setup(){
    camWidth = 1280;  // try to grab at this size.
    camHeight = 720;
    intrinsic = cv::Mat::eye(3,3, CV_64F);
    cv::Mat intrinsic_ =  (cv::Mat_<double>(3, 3) <<
                  1150.0, 0.0, camWidth/2,
                  0.0, 1150.0, camHeight/2,
                  0.0, 0.0, 1.0);
    cv::Mat dist = cv::Mat::zeros(1, 4, CV_64F);
    //http://www.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/tech0118.html
    intrinsic_.copyTo(intrinsic);
    std::cout << "--- intrinsic ---\n" << intrinsic << std::endl;
    ofCropped.allocate(camWidth, camHeight, OF_IMAGE_COLOR);
    vector<ofVideoDevice> devices = vidGrabber.listDevices();
    for(size_t i = 0; i < devices.size(); i++){
        if(devices[i].bAvailable){
            ofLogNotice() << devices[i].id << ": " << devices[i].deviceName;
        }else{
            ofLogNotice() << devices[i].id << ": " << devices[i].deviceName << " - unavailable ";
        }
    }
    vidGrabber.setDeviceID(1);
    vidGrabber.initGrabber(camWidth, camHeight);
    ofSetVerticalSync(true);
    //---im_gui
    gui.setup();
    ImGui::SetNextWindowPos(ImVec2(0, 100), ImGuiCond_Once);
    
    //---HID
    esp32_hid_camera = new esp32_HID_camera();
    if(esp32_hid_camera->setup(0xE502, 0xBBAB)){
        esp32_hid_camera->startThread();
    }
    ofAddListener(esp32_hid_camera->shot, this, &ofApp::capture);
    //--
    perspective.allocate(1280,360);
    imagePlane.allocate(640,360);
    cam.setControlArea(area);
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
    drawTerrios(perspective, cam);
    for(int i = 0; i < tvecsarray.size(); i++){
        Eigen::Matrix4d M;
        M = VecsToMat(rvecsarray[i], tvecsarray[i]);
        draw_marker(M, 20.0, ofColor(100,200,200), ofColor(100,100,100), perspective, cam);
    }
    imagePlane.begin();
    ofDisableDepthTest();
    ofClear(0,0);
    for(auto points: image_points){
        for(auto point:points){
            ofNoFill();
            ofSetColor(0,255,0);
            ofDrawCircle(point.x / 2.0, point.y / 2.0, 4);
        }
    }
    imagePlane.end();
    cam.enableMouseInput();
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofDisableDepthTest();
    ofCropped.draw(0, 0, 640 ,360);
    imagePlane.draw(0,0);
    perspective.draw(area.x, area.y);
    draw_gui();
}

//--------------------------------------------------------------
void ofApp::calc(){
    rvecsarray.clear();
    tvecsarray.clear();
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
    std::cout << "--- intrinsic_guess ---\n" << intrinsic << std::endl;
    std::cout << "STEP 2: intrinsic & distortion estimation" << std::endl;
    cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 1024, 1e-9);
    int flags = 0;
    if(use_guess){
        flags = cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_PRINCIPAL_POINT ;
    }
    double rms = cv::calibrateCamera(object_points, image_points, cv::Size(camWidth, camHeight), intrinsic, distortion, rvecs, tvecs, flags, criteria);
    std::cout << "--- intrinsic ---\n" << intrinsic << std::endl;
    std::cout << "--- distortion ---\n" << distortion << std::endl;
    cv::FileStorage fs("usv_camera.xml", cv::FileStorage::WRITE);
    fs << "intrinsic" << intrinsic;
    fs << "distortion" << distortion;
    fs.release();

    for(int i = 0; i < 20; i++){
        cv::Mat r = rvecs(cv::Rect(0,i,1,1));
        cv::Vec3d rv =  cv::Vec3d(r.at<double>(0), r.at<double>(1), r.at<double>(2));
        rvecsarray.push_back(rv);
        cv::Mat t = tvecs(cv::Rect(0,i,1,1));
        cv::Vec3d tv =  cv::Vec3d(t.at<double>(0), t.at<double>(1), t.at<double>(2));
        tvecsarray.push_back(tv);
    }
    std::cout<<rms<<std::endl;
    image_points.clear();
}
//--------------------------------------------------------------
void ofApp::draw_gui(){
    gui.begin();{
        ImGui::Begin("control");{
            if(ImGui::IsWindowHovered()){
                cam.disableMouseInput();
            }else{
                
            }
            if(use_guess){
                if (ImGui::InputDouble("fx", &fxp)) {
                    intrinsic.at<double>(0,0) = fxp;
                    intrinsic.at<double>(1,1) = fxp;
                }
                if (ImGui::Button(" nouse_guess ")) {
                        use_guess = !use_guess;
                    }
            }else{
                if (ImGui::Button(" use_guess ")) {
                    use_guess = !use_guess;
                }
            }
            if (ImGui::Button(" capture ")) {
                if(!capture_frame){
                    capture_frame = true;
                }
            }
            ImGui::Text("count: %i", num_capture);
        }ImGui::End();
    }gui.end();
}

//--------------------------------------------------------------
/*
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
 */
//--------------------------------------------------------------
void ofApp::capture(bool &e){
    if(!capture_frame){
        capture_frame = true;
    }
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

/*
 [824.6285415708186, 0, 584.4611373694785;
  0, 823.0042987890754, 383.8118324468167;
  0, 0, 1]
 --- distortion ---
 [-0.2177301785371422, 0.2683719228722065, 0.01738746417836339, 0.002531695754987587, -0.2036533583848873]
 1.56022
--- intrinsic ---
[830.7179517948707, 0, 579.8348119568883;
 0, 824.3631074076923, 357.0593930400046;
 0, 0, 1]
--- distortion ---
[-0.2346837009532462, 0.3796089895349467, 0.01362375534885017, -0.002320921207977041, -0.3672634486294185]
*/
