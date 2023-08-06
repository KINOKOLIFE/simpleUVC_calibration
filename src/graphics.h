#pragma once
#include "ofApp.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include "graphics.h"

//-------------

static Eigen::Matrix4d rotx(double degree)
{
    Eigen::Matrix4d R = Eigen::Matrix4d::Identity(4,4);
    Eigen::Matrix3d AxisAngle;
    Eigen::Vector3d axis;
    axis<<1,0,0;  //x軸を指定
    AxisAngle = Eigen::AngleAxisd( degree / 180.0 * M_PI, axis);
    R.block(0,0,3,3) = AxisAngle;
    return R;
}
static Eigen::Matrix4d roty(double degree)
{
    Eigen::Matrix4d R = Eigen::Matrix4d::Identity(4,4);
    Eigen::Matrix3d AxisAngle;
    Eigen::Vector3d axis;
    axis<<0,1,0;  //y軸を指定
    AxisAngle = Eigen::AngleAxisd( degree / 180.0 * M_PI, axis);
    R.block(0,0,3,3) = AxisAngle;
    return R;
}
static Eigen::Matrix4d rotz(double degree)
{
    Eigen::Matrix4d R = Eigen::Matrix4d::Identity(4,4);
    Eigen::Matrix3d AxisAngle;
    Eigen::Vector3d axis;
    axis<<0,0,1;  //z軸を指定
    AxisAngle = Eigen::AngleAxisd( degree / 180.0 * M_PI, axis);
    R.block(0,0,3,3) = AxisAngle;
    return R;
}

//--------------------


static Eigen::Matrix4d VecsToMat(cv::Vec3d rvec, cv::Vec3d tvec)
{
    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
    cv::Mat R;

    // Reallocate from rvec to H
    cv::Rodrigues(rvec, R);

    for (unsigned int m = 0; m < 3; m++)
    {
        for (unsigned int n = 0; n < 3; n++)
        {
            H(m, n) = R.at<double>(m, n);

        }
    }

    // Reallocate from tvec to H
    for (unsigned int m = 0; m < 3; m++)
    {
        H(m, 3) = tvec(m);
    }

    return H;
}
static void MatToVecs(cv::Vec3d &rvec, cv::Vec3d &tvec, Eigen::Matrix4d H)
{
    cv::Vec3d temp1, temp2;
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);

    
    for (unsigned int m = 0; m < 3; m++)
    {
        for (unsigned int n = 0; n < 3; n++)
        {
            R.at<double>(m, n) = H(m, n);
        }
    }

    
    // Reallocate from H to rvec
    cv::Rodrigues(R, temp1);

    

    // Reallocate from tvec to H
    for (unsigned int m = 0; m < 3; m++)
    {
        temp2(m) = H(m, 3);
    }

    // Return tvec and rvec
    rvec = temp1;
    tvec = temp2;
}


static glm::mat4x4 eigen_glm(Eigen::Matrix4d &mat){
    ofMatrix4x4 dest;
    float array[16];
    for(int j=0; j<4 ; j++){
        for(int k=0; k<4; k++){
            array[j*4 + k] = (float)mat(j,k);
        }
    }
    dest.set(array);
    ofMatrix4x4 src = dest.getTransposedOf(dest);
    return glm::mat4x4(src);
};
static Eigen::Matrix4d glm_Eigen(glm::mat4x4  &mat){
    Eigen::Matrix4d dest;
    for(int j=0; j<4 ; j++){
        for(int k=0; k<4; k++){
            dest(j,k) = mat[j][k];
        }
    }
    
    return dest.transpose();
};

//------------------
static void fake_draw(ofFbo &fbo,ofEasyCam &cam){
    fbo.begin();
    ofBackground(50);
        cam.begin();
        cam.end();
    fbo.end();
};
static void drawTerrios(ofFbo &fbo,ofEasyCam &cam){
    ofEnableAlphaBlending();
    ofEnableBlendMode(OF_BLENDMODE_ALPHA);
    ofEnableDepthTest();
    
    
    fbo.begin();
    
        ofClear(0);
    //ofBackground(50);
    ofBackgroundGradient(ofColor(64), ofColor(0));
        cam.begin();
            ofDrawGrid(10,10,true,false,false,true);
            ofDrawAxis(10);
           
        cam.end();
    fbo.end();
};
static void drawTerrios(ofFbo &fbo,ofCamera &cam){
    ofEnableAlphaBlending();
    ofEnableBlendMode(OF_BLENDMODE_ALPHA);
    ofEnableDepthTest();

    fbo.begin();
        ofClear(0);
        cam.begin();
            ofDrawGrid(10,10,true,false,false,true);
            ofDrawAxis(10);
        cam.end();
    fbo.end();
};
static void draw_axis(ofFbo &fbo, ofEasyCam &cam, Eigen::Matrix4d &mat){
    fbo.begin();{
        cam.begin();{
            ofPushMatrix();{
                ofMultMatrix(eigen_glm(mat));
                ofDrawAxis(100);
            }ofPopMatrix();
        }cam.end();
    }fbo.end();
};
static void draw_axis(ofFbo &fbo, ofCamera &cam, Eigen::Matrix4d &mat){
    fbo.begin();{
        cam.begin();{
            ofPushMatrix();{
                ofMultMatrix(eigen_glm(mat));
                ofDrawAxis(100);
            }ofPopMatrix();
        }cam.end();
    }fbo.end();
};

static void draw_marker(Eigen::Matrix4d &mat,float length, ofColor stroke, ofColor fill,ofFbo &fbo,ofEasyCam &cam){
    fbo.begin();
         cam.begin();
            ofEnableDepthTest();
    
             ofPushMatrix();
             {
                 ofMultMatrix(eigen_glm(mat));
                 ofSetColor(fill);
                 ofNoFill();
                 ofDrawRectangle(-length / 2.0, -length / 2.0, length ,length);
                 ofSetColor(255,0,0);
                ofFill();
                 //ofDrawCircle(-length / 2.0, -length / 2.0, 0.0 ,10);
             }
             ofPopMatrix();
            ofDisableDepthTest();
         cam.end();
     fbo.end();
}
static void draw_marker(Eigen::Matrix4d &mat,float length, ofColor stroke, ofColor fill,ofFbo &fbo,ofEasyCam &cam,bool cv_to_gl){
    Eigen::Matrix4d mm = Eigen::Matrix4d::Identity(4,4);
    if(cv_to_gl){
        mm = mat * rotx(180);
    }else{
        mm << mat;
    }
    fbo.begin();
         cam.begin();
            ofEnableDepthTest();
             ofPushMatrix();
             {
                 ofPushStyle();{
                     ofMultMatrix(eigen_glm(mm));
                     ofSetColor(fill);
                     ofFill();
                     //ofSetLineWidth(4);
                     ofDrawRectangle(-length / 2.0, -length / 2.0, length ,length);
                     ofSetColor(stroke);
                     ofNoFill();
                     //ofSetLineWidth(0);
                     ofDrawRectangle(-length / 2.0 , -length / 2.0  , length , length);
                     ofDrawAxis(length /2.0);
                 }ofPopStyle();
             }
             ofPopMatrix();
            ofDisableDepthTest();
         cam.end();
     fbo.end();
}
static void draw_board(Eigen::Matrix4d &mat, int gridnum, float marker_length, ofColor stroke, ofColor fill, ofFbo &fbo,ofEasyCam &cam){
    float board_length = ( gridnum - 1 ) * ( marker_length + 2.0 ) + marker_length / 2.0;
    float board_center = board_length / 2.0;
    fbo.begin();
         cam.begin();
            ofEnableDepthTest();
    
             ofPushMatrix();
             {
                 ofMultMatrix(eigen_glm(mat));
                 ofPushMatrix();
                 {
                     ofTranslate(board_center + marker_length / 2.0 , board_center + marker_length / 2.0);
                     ofDrawAxis(marker_length / 2.0);
                 }
                 ofPopMatrix();
                 ofDrawAxis(marker_length / 2.0);
                 ofSetColor(fill);
                 ofNoFill();
                 ofDrawRectangle(marker_length /2.0 , marker_length /2.0 , board_length ,board_length);
                 ofSetColor(255,0,0);
                 ofFill();
                 //ofDrawCircle(-length / 2.0, -length / 2.0, 0.0 ,10);
             }
             ofPopMatrix();
            ofDisableDepthTest();
         cam.end();
     fbo.end();
}

//---local

static void draw_marker_center(ofFbo &fbo, ofEasyCam &cam, std::vector<cv::Vec3d> &points, ofColor c, float rad, bool fiiiinsde){
    fbo.begin();{
        cam.begin();{
            ofPushStyle();{
                ofSetColor(c);
                if(fiiiinsde){
                    ofFill();
                }else{
                    ofNoFill();
                }
                for( auto p : points){
                    ofDrawSphere(p(0), p(1), p(2), rad);
                }
            }ofPopStyle();
        }cam.end();
    }fbo.end();
}

static ofMesh rectMesh(float x, float y, float w, float h, bool normalized)
{
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);

    mesh.addVertex(ofVec3f(x, y));
    mesh.addTexCoord(ofVec2f(0, 0));

    mesh.addVertex(ofVec3f(x + w, y));
    mesh.addTexCoord(ofVec2f(normalized ? 1 : w, 0));

    mesh.addVertex(ofVec3f(x, y + h));
    mesh.addTexCoord(ofVec2f(0, normalized ? 1 : h));

    mesh.addVertex(ofVec3f(x + w, y + h));
    mesh.addTexCoord(ofVec2f(normalized ? 1 : w, normalized ? 1 : h));

    return mesh;
}
//-----------------------
static void setNormals( ofMesh &mesh ){
//The number of the vertices
    int nV = mesh.getNumVertices();
//The number of the triangles
    int nT = mesh.getNumIndices() / 3;
    vector<ofPoint> norm( nV ); //Array for the normals
    vector<glm::vec3> norm1(nV);
//Scan all the triangles. For each triangle add its
//normal to norm's vectors of triangle's vertices
    for (int t=0; t<nT; t++) {
        //Get indices of the triangle t
        int i1 = mesh.getIndex( 3 * t );
        int i2 = mesh.getIndex( 3 * t + 1 );
        int i3 = mesh.getIndex( 3 * t + 2 );
        //Get vertices of the triangle
        const ofPoint &v1 = mesh.getVertex( i1 );
        const ofPoint &v2 = mesh.getVertex( i2 );
        const ofPoint &v3 = mesh.getVertex( i3 );
        //Compute the triangle's normal
        ofPoint dir = ( (v2 - v1).crossed( v3 - v1 ) ).normalized();
        //Accumulate it to norm array for i1, i2, i3
        norm1[ i1 ] += dir;
        norm1[ i2 ] += dir;
        norm1[ i3 ] += dir;
    }

    //Normalize the normal's length
    for (int i=0; i<nV; i++) {
        norm1[i] = glm::normalize(norm1[i]);
    }
    //Set the normals to mesh
    mesh.clearNormals();
    //mesh.addNormals( norm );
    mesh.addNormals(norm1);
}
