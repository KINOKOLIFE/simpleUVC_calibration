#pragma once
#include "ofApp.h"
#include <hidapi/hidapi.h>
class hid : public ofThread {
public:
    ofEvent<bool> dragging_button1;
    ofEvent<bool> up_button1;
    ofEvent<bool> up_button2;
    ofEvent<bool> up_button3;
    
    bool button_0_state = false;
    bool button_1_state = false;
    bool button_2_state = false;
    bool button_3_state = false;
    hid_device *handle;
    unsigned char buf[4];
    bool setup(int vid, int pid){
        if(!isThreadRunning()){
            int res = hid_init();
           //handle = hid_open(0xE502, 0xBBAB, NULL);
           // handle = hid_open(0xE502, 0xABCD, NULL);
            handle = hid_open(vid, pid,  NULL);
            //wchar_t s[] = L"98-CD-AC-D3-56-76";
            if(handle == NULL){
                ofLog(OF_LOG_NOTICE) << "fail to connect BLE ";
                return false;
            }else{
                ofLog(OF_LOG_NOTICE) << "connect BLE ";
                wchar_t wstr[64];
                int res = hid_get_serial_number_string(handle, wstr, 64);
                wprintf(L"Serial Number String: (%d) %s\n", wstr[0], wstr);
                return true;
            }
        }
    }
    void threadedFunction() {
        while(isThreadRunning()) {
            int res = hid_read(handle, buf, 4);
            //button1
            
            if ((int)buf[1] & (1<<0)){
                if(!button_1_state){
                    //ofNotifyEvent(dragging_button1, button_1_state);
                }
                button_1_state = true;
                
            }else{
                if(button_1_state){
                    //ofNotifyEvent(up_button1, button_1_state);
                }
                button_1_state = false;
            }
            //buntton2
            if ((int)buf[1] & (1<<1)){
                if(!button_2_state){
                }
                button_2_state = true;
                
            }else{
                if(button_2_state){
                    ofNotifyEvent(up_button2, button_2_state);
                }
                button_2_state = false;
            }
            //button3
            if ((int)buf[1] & (1<<2)){
                if(!button_3_state){
                }
                button_3_state = true;
                
            }else{
                if(button_3_state){
                    ofNotifyEvent(up_button3, button_3_state);
                }
                button_3_state = false;
            }
            //printf("buf[%d]: %d\n", 1, buf[1]);
        }
    }

    void disconnect(){
        if(isThreadRunning()){
            this->stopThread();
            hid_close(handle);
            int res = hid_exit();
            ofLog(OF_LOG_NOTICE) << "stop HID";
        }
    }
};

class esp32_HID_us : public hid{
public:
    ofEvent<bool> capture_stack;
    void threadedFunction() {
        while(isThreadRunning()) {
            int res = hid_read(handle, buf, 4);
            //button1
       
            if ( ((int)buf[1] & (1<<0))          ){
                ofLog(OF_LOG_NOTICE) << "capture";
            }
            if ( !((int)buf[1] & (1<<1)) ){
                ofLog(OF_LOG_NOTICE) << "t265 capture start";
            }
       
        }
    }
};
//----------------------------------
class esp32_HID_camera : public hid{
public:
    ofEvent<bool> shot;
    bool switch1 = false;
    void threadedFunction() {
        while(isThreadRunning()) {
            int res = hid_read(handle, buf, 4);
            //button1
            //cout  << (int)buf[1] << endl;
                //printf("buf[%d]: %d\n", 1, buf[1]);
            if((int)buf[1] & (1<<0)){
                if( !switch1 ){
                    switch1 = true;
                    bool e = false;
                    ofNotifyEvent(shot, e);
                }
               
            }else
                switch1 = false;
            }
        }
};
//-------------------------
class HidEventDiscpatcher{
public:
    HidEventDiscpatcher(){
        
    }
    void eventCatch(bool & b){
        
    }
};
