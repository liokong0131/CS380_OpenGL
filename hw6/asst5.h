#pragma once
//asst5
#include <vector>
#include <list>
#include "rigtform.h"
#include "scenegraph.h"
#include <fstream>

using namespace std;

struct AnimationState {
  bool playing = false;
  std::vector<RigTForm> curFrame;
  std::vector<std::shared_ptr<SgRbtNode>> nodePtrVec;
  std::list<std::vector<RigTForm>> keyFrameList;
  int curFrameIdx = -1;
  char filename[15] = "animation.txt";
  int numRbtNodes;
  int msBetweenKeyFrames = 2000; // 2 seconds between keyframes
  int animateFramesPerSecond = 60; // frames to render per second during animation playback

  bool hasKeyFrame() const {
    return curFrameIdx >= 0;
  }

  bool isPlaying() const {
    return playing;
  }

  void setFrameIdx(int num) {
    assert(0 <= num && num < keyFrameList.size());
    curFrameIdx = num;
    auto iter = next(keyFrameList.begin(), curFrameIdx);
    curFrame = *iter;
  }

  void setNumRbtNodes() {
    numRbtNodes = static_cast<int>(nodePtrVec.size());
  }

  void printCurrentFrameIdx(void) const {
    if(!hasKeyFrame()){
      printf("No frames defined\n");
    } else {
      printf("Now at frame [%d]\n", curFrameIdx);
    }
  }

  void loadCurrentFrame() {
    if(!hasKeyFrame()){
      printf("No key frame defined\n");
      return;
    }

    for (int i = 0; i < nodePtrVec.size(); i++) {
      nodePtrVec[i]->setRbt(curFrame[i]);
    }
  }

  void updateCurrentFrame() {
    curFrame.clear();
    for (auto& node : nodePtrVec) {
      curFrame.push_back(node->getRbt());
    }
  }

  void moveCurrentFrame(int offset) {
    int nextIdx = curFrameIdx + offset;
    if(nextIdx < 0 || nextIdx >= static_cast<int>(keyFrameList.size()))
      return;
  
    curFrameIdx = nextIdx;
    auto iter = next(keyFrameList.begin(), curFrameIdx);
    curFrame = *iter;
  
    loadCurrentFrame();
  
    if(offset > 0)
      printf("Stepped forward to frame [%d]\n", curFrameIdx);
    else
      printf("Stepped backward to frame [%d]\n", curFrameIdx);
  }
  
  void deleteCurrentFrame() {
    if(!hasKeyFrame()){
      printf("Frame list is now EMPTY\n");
      return;
    }

    auto iter = next(keyFrameList.begin(), curFrameIdx);
    keyFrameList.erase(iter);
  
    printf("Deleting current frame [%d]\n", curFrameIdx);
  
    curFrameIdx = max(0, curFrameIdx - 1);
    curFrameIdx = min(static_cast<int>(keyFrameList.size()) - 1, curFrameIdx);

    if (!hasKeyFrame()) {
      curFrame.clear();
    } else {
      curFrame = *next(keyFrameList.begin(), curFrameIdx);
      loadCurrentFrame();
    }
  
    printCurrentFrameIdx();
  }
  
  void createKeyFrame() {
    // curFrame set
    updateCurrentFrame();
  
    // curFrame add to keyFrameList
    curFrameIdx++;
    auto iter = (!hasKeyFrame()) ? keyFrameList.begin() : next(keyFrameList.begin(), curFrameIdx);
    keyFrameList.insert(iter, curFrame);

    printf("Create new frame [%d]\n", curFrameIdx);
  }
  
  void updateKeyFrame() {
    if (!hasKeyFrame()) {
      createKeyFrame();
    }
    else {
      updateCurrentFrame();
  
      auto iter = next(keyFrameList.begin(), curFrameIdx);
      *iter = curFrame;
    }
  
    printf("Copying scene graph to current frame [%d]\n", curFrameIdx);
  }

  void readKeyFrameList(void){
    ifstream readFile(filename);

    if (!readFile) {
      printf("Failed to open file: %s\n", filename);
      return;
    }

    int numFrames, numRbtNodes_;
    readFile >> numFrames >> numRbtNodes_;

    if(numRbtNodes_ != numRbtNodes){
      printf("Number of Rbt per frame in animation.txt does not match number of SgRbtNodes in the current scene graph.0 frames read.\n");
      return;
    }

    keyFrameList.clear();
    curFrame.clear();
    curFrameIdx = (numFrames == 0) ? -1 : 0;

    for (int i = 0; i < numFrames; i++) {
      vector<RigTForm> frame;
      for (int j = 0; j < numRbtNodes; j++) {
        Cvec3 t;
        Quat r;
        readFile >> t[0] >> t[1] >> t[2];
        readFile >> r[0] >> r[1] >> r[2] >> r[3];
        frame.push_back(RigTForm(t, r));
      }
      keyFrameList.push_back(frame);
    }
    readFile.close();
    
    if(hasKeyFrame())
      curFrame = *keyFrameList.begin();

    loadCurrentFrame();

    printf("Reading animation from %s\n", filename);
    printf("%d frames read.\n", numFrames);
  }

  void writeKeyFrameList(void) {
    ofstream writeFile(filename);

    writeFile << keyFrameList.size() << " " << numRbtNodes << "\n";

    for (const auto& frame : keyFrameList) {
      for (const auto& rbt : frame) {
        const Cvec3& t = rbt.getTranslation();
        const Quat& r = rbt.getRotation();
        writeFile << t[0] << " " << t[1] << " " << t[2] << " ";
        writeFile << r[0] << " " << r[1] << " " << r[2] << " " << r[3] << "\n";
      }
    }

    writeFile.close();

    printf("Writing animation to %s\n", filename);
  }

  void updateAnimationSpeed(int offset) {
    msBetweenKeyFrames = min(10000, max(100, msBetweenKeyFrames - offset));
  }
};

inline Cvec3 lerp(Cvec3 c0, Cvec3 c1, float alpha) {
  return c0 * (1 - alpha) + c1 * alpha;
}

inline Quat slerp(Quat q0, Quat q1, float alpha) {
  return pow(cn(q1  * inv(q0)), alpha) * q0;
}

// Given t in the range [0, n], perform interpolation and draw the scene
 // for the particular t. Returns true if we are at the end of the animation
 // sequence, or false otherwise.
