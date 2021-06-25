//
// Created by qingyun on 1/21/21.
//
#include <iostream>
#include <fstream>
#include "JsonHelper.h"
JsonHelper::JsonHelper() {

}


void JsonHelper::readSingleJson(string filename){
    ifstream ifs(filename);
    reader.parse(ifs, root);
}


void JsonHelper::parseJsonForMiris(vector<DetectorState> &vector){
    int frameSize = root.size();
    for(int i=0;i<frameSize;i++){
        int objectSize = root[i].size();
        for(int j=0;j<objectSize;j++) {
            Json::Value currObj = root[i][j];
            DetectorState currState;
            int track_id = currObj["track_id"].asInt();
            currState.frameIndex = currObj["frame_idx"].asInt();
            currState.left = currObj["left"].asInt();
            currState.top = currObj["top"].asInt();
            currState.right = currObj["right"].asInt();
            currState.bottom = currObj["bottom"].asInt();
            currState.track_id = track_id;
            vector.push_back(currState);
        }
    }
}