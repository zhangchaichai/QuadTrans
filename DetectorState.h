//
// Created by qingyun on 1/22/21.
//

#ifndef TRANSVIDEO_DETECTORSTATE_H
#define TRANSVIDEO_DETECTORSTATE_H

#include <string>
using namespace std;
class DetectorState {
public:
    int frameIndex;
    int left;
    int top;
    int right;
    int bottom;
    int track_id;
    string toString();
};


#endif //TRANSVIDEO_DETECTORSTATE_H
