////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

#include <vector>
#include <list>
#include <string>
#include <memory>
#include <stdexcept>

#include "GL/glew.h"
#ifdef __APPLE__
#   include <GLUT/glut.h>
#else
#   include "GL/glut.h"
#endif

#include "cvec.h"
#include "matrix4.h"
#include "geometrymaker.h"
#include "ppm.h"
#include "glsupport.h"

// asst3
#include "rigtform.h"
#include <cmath>
#include "arcball.h"
#include "quat.h"
#include <algorithm>

//asst4
#include "asstcommon.h"
#include "scenegraph.h"
#include "drawer.h"
#include "picker.h"

//asst5
#include "sgutils.h"
#include "asst5.h"

//asst6
#include "geometry.h"
#include "asst6.h"

//asst7
#include "asst7.h"

//asst8
#include "mesh.h"
#include "asst8.h"

#define ESC 27
#define SPACE 32

using namespace std;      // for string, vector, iostream, and other standard C++ stuff

// G L O B A L S ///////////////////////////////////////////////////

// --------- IMPORTANT --------------------------------------------------------
// Before you start working on this assignment, set the following variable
// properly to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or
// OpenGL 3.x+ with GLSL 1.3.
//
// Set g_Gl2Compatible = true to use GLSL 1.0 and g_Gl2Compatible = false to
// use GLSL 1.3. Make sure that your machine supports the version of GLSL you
// are using. In particular, on Mac OS X currently there is no way of using
// OpenGL 3.x with GLSL 1.3 when GLUT is used.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get
// loaded
// ----------------------------------------------------------------------------

//static const bool g_Gl2Compatible = false;
const bool g_Gl2Compatible = false;

static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane
static const float g_groundY = -2.0;      // y coordinate of the ground
static const float g_groundSize = 10.0;   // half the ground length

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false;    // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
static int g_activeShader = 0;
// ============================================================================
// Define your own global variables here if needed.
// ============================================================================
// static int g_newVariable = 0;
static int editMode = 0; // 0 for sky, 1 for cube 1, 2 for cube 2
static int viewMode = 0;
static int skyFrameMode = 0;  // 0 for  world-sky frame, 1 for sky-sky frame
static double g_arcballScale = 1.0;
double g_arcballScreenRadius = 1.0 ;

//asst4
static bool g_picking = false;

//asst5
static bool g_arcballVisible = true;
static AnimationState g_animationState;

//asst8
static AnimationState g_cubeAS;
static bool g_smoothShading = false;
vector<double> g_vertexPhase;
static int g_time = 0;
static int subdivision_level = 0;
static float bubble_speed = 0.01;
// ============================================================================

static shared_ptr<Material> g_redDiffuseMat,
g_blueDiffuseMat,
g_bumpFloorMat,
g_arcballMat,
g_pickingMat,
g_lightMat,
g_cubeMeshMat;

shared_ptr<Material> g_overridingMaterial;

// --------- Geometry

// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_sphere;
static shared_ptr<SimpleGeometryPN> g_newcube;
typedef SgGeometryShapeNode MyShapeNode;

static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node, g_light1Node, g_light2Node, g_cubeNode;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode;

//asst8
static shared_ptr<Mesh> g_animatedMesh;
static vector<shared_ptr<Mesh>> g_cubeMesh_vec;
// --------- Scene
// 
//static RigTForm g_skyRbt = RigTForm(Cvec3(0.0, 0.25, 4.0));
//static RigTForm g_centerRbt = RigTForm(Cvec3(0.0, 0.0, 0.0));
//static RigTForm g_objectRbt[2] = {
//    RigTForm(Cvec3(-1, 0, 0)),  // 1st cube
//    RigTForm(Cvec3(1, 0, 0))   // 2nd cube
//};  // 1st cube
//static Cvec3f g_objectColors[2] = {
//    Cvec3f(1, 0, 0),       // 1st cube color
//    Cvec3f(0, 1, 0)	       // 2nd cube color
//};
static Cvec3f g_arcBallColor = Cvec3f((float) 153/255, (float) 255/255, (float) 153/255);
///////////////// END OF G L O B A L S //////////////////////////////////////////////////


static void initGround() {
  int ibLen, vbLen;
  getPlaneVbIbLen(vbLen, ibLen);

  // Temporary storage for cube Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makePlane(g_groundSize*2, vtx.begin(), idx.begin());
  g_ground.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initCubes() {
  int ibLen, vbLen;
  getCubeVbIbLen(vbLen, ibLen);

  // Temporary storage for cube Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeCube(1, vtx.begin(), idx.begin());
  g_cube.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initSpheres() {
  int ibLen, vbLen;
  getSphereVbIbLen(20, 10, vbLen, ibLen);

  // Temporary storage for sphere Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);
  makeSphere(1, 20, 10, vtx.begin(), idx.begin());
  g_sphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}

static RigTForm getEyeRbt() {
  switch (viewMode) {
  case 0: return getPathAccumRbt(g_world, g_skyNode);
  case 1: return getPathAccumRbt(g_world, g_robot1Node);
  case 2: return getPathAccumRbt(g_world, g_robot2Node);
  default: break;
  }
}

static RigTForm getCenterRbt() {
  if (g_currentPickedRbtNode == shared_ptr<SgRbtNode>())
    switch (viewMode) {
    case 0: return getPathAccumRbt(g_world, g_world);
    case 1: return getPathAccumRbt(g_world, g_robot1Node);
    case 2: return getPathAccumRbt(g_world, g_robot2Node);
    default: break;
    }
  else
    return getPathAccumRbt(g_world, g_currentPickedRbtNode);
}

static void updateArcballScale() {
  const RigTForm eyeRbt = getEyeRbt();
  const RigTForm centerRbt = getCenterRbt();

  const RigTForm invEyeRbt = inv(eyeRbt);
  const RigTForm MVM_r = invEyeRbt * centerRbt;
  g_arcballScale = getScreenToEyeScale(MVM_r.getTranslation()[2], g_frustFovY, max(g_windowHeight, 1));
}

// takes a projection matrix and send to the the shaders
inline void sendProjectionMatrix(Uniforms& uniforms, const Matrix4& projMatrix) {
  uniforms.put("uProjMatrix", projMatrix);
}

// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY() {
  if (g_windowWidth >= g_windowHeight)
    g_frustFovY = g_frustMinFov;
  else {
    const double RAD_PER_DEG = 0.5 * CS175_PI/180;
    g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth, cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
  }
}

static Matrix4 makeProjectionMatrix() {
  return Matrix4::makeProjection(
           g_frustFovY, g_windowWidth / static_cast <double> (g_windowHeight),
           g_frustNear, g_frustFar);
}

static void drawArcBall(Uniforms uniforms) {
  //const Matrix4 projmat = makeProjectionMatrix();
  //sendProjectionMatrix(uniforms, projmat);

  const RigTForm eyeRbt = getEyeRbt();
  const RigTForm centerRbt = getCenterRbt();

  const RigTForm invEyeRbt = inv(eyeRbt);

  if(!g_mouseMClickButton && !(g_mouseLClickButton && g_mouseRClickButton))
    updateArcballScale();

  if(!g_arcballVisible)
    return;
  const double arcballRadius = g_arcballScale * g_arcballScreenRadius;
  const Matrix4 MVM = rigTFormToMatrix(invEyeRbt * centerRbt) * Matrix4::makeScale(Cvec3(1, 1, 1) * arcballRadius);
  const Matrix4 NMVM = normalMatrix(MVM);
  // Use uniforms as opposed to curSS
  sendModelViewNormalMatrix(uniforms, MVM, normalMatrix(MVM));

  // No more glPolygonMode calls

  g_arcballMat->draw(*g_sphere, uniforms);

  // No more glPolygonMode calls
}

static shared_ptr<SgRbtNode> getCurrentViewNode() {
  switch (viewMode) {
  case 0: return g_skyNode;
  case 1: return g_robot1Node;
  case 2: return g_robot2Node;
  default: break;
  }
}

// drawStuff just takes in a picking flag, and no curSS
static void drawStuff(bool picking) {
  // Declare an empty uniforms
  Uniforms uniforms;
  // build & send proj. matrix to vshader
  const Matrix4 projmat = makeProjectionMatrix();
  // as opposed to the current vtx shader
  sendProjectionMatrix(uniforms, projmat);
  
  const RigTForm eyeRbt = getEyeRbt();
  const RigTForm invEyeRbt = inv(eyeRbt);

  Cvec3 light1 = getPathAccumRbt(g_world, g_light1Node).getTranslation();
  Cvec3 light2 = getPathAccumRbt(g_world, g_light2Node).getTranslation();
  const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(light1, 1)); // g_light1 position in eye coordinates
  const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(light2, 1)); // g_light2 position in eye coordinates
  // send the eye space coordinates of lights to uniforms
  uniforms.put("uLight", eyeLight1);
  uniforms.put("uLight2", eyeLight2);

  if (!picking) {
    Drawer drawer(invEyeRbt, uniforms);
    g_world->accept(drawer);

    if (g_currentPickedRbtNode != shared_ptr<SgRbtNode>() || (viewMode == 0 && skyFrameMode == 0))
      drawArcBall(uniforms);
  }
  else {
    Picker picker(invEyeRbt, uniforms);

    // set overiding material to our picking material
    g_overridingMaterial = g_pickingMat;

    g_world->accept(picker);
    // unset the overriding material
    g_overridingMaterial.reset();

    glFlush();
    g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
    if (g_currentPickedRbtNode == g_groundNode)
      g_currentPickedRbtNode = shared_ptr<SgRbtNode>();
    if (g_currentPickedRbtNode == getCurrentViewNode()) 
      g_currentPickedRbtNode = shared_ptr<SgRbtNode>();

    printf("Picking mode is off\n");
    g_picking = false;
  }
}

static void display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (g_smoothShading) {
    update_avgNormal(g_animatedMesh);
  }
  update_mesh_to_geometry(g_animatedMesh, g_newcube, g_smoothShading);

  drawStuff(false);
 
  glutSwapBuffers();    // show the back buffer (where we rendered stuff)                             

  checkGlErrors();
}

static void reshape(const int w, const int h) {
  g_windowWidth = w;
  g_windowHeight = h;
  g_arcballScreenRadius = max(0.25 * min(g_windowWidth, g_windowHeight), 0.1);
  glViewport(0, 0, w, h);
  cerr << "Size of window is now " << w << "x" << h << endl;
  updateFrustFovY();
  updateArcballScale();
  glutPostRedisplay();
}

static double clamp(double value, double min_value, double max_value) {
    if      (value < min_value) return min_value;
    else if (value > max_value) return max_value;
    else                        return value;
}

static RigTForm calculate_Rotation(Cvec2 p1, Cvec2 p2) {
  if (p1[0] == p2[0] && p1[1] == p2[1])
    return RigTForm();

  const RigTForm eyeRbt = getEyeRbt();
  const RigTForm centerRbt = getCenterRbt();

  const RigTForm invEyeRbt = inv(eyeRbt);
  const RigTForm MVM_r = invEyeRbt * centerRbt;

  Quat rotatQ;
  const double radius_square = pow(g_arcballScreenRadius, 2);
  Cvec3 o = Cvec3(getScreenSpaceCoord(MVM_r.getTranslation() , makeProjectionMatrix(), g_frustNear, g_frustFovY, g_windowWidth, g_windowHeight), 0);
  const Cvec3 s1 = Cvec3(p1, sqrt(max(radius_square - pow(p1[0] - o[0], 2) - pow(p1[1] - o[1], 2), 0.0)));
  const Cvec3 s2 = Cvec3(p2, sqrt(max(radius_square - pow(p2[0] - o[0], 2) - pow(p2[1] - o[1], 2), 0.0)));
  Cvec3 v1 = s1 - o;
  Cvec3 v2 = s2 - o;

  v1 = normalize(v1);
  v2 = normalize(v2);

  rotatQ = Quat(clamp(dot(v1, v2), -1.0, 1.0), cross(v1, v2));
  return RigTForm(rotatQ);
}



static void motion(const int x, const int y) {
  const double dx = x - g_mouseClickX;
  const double dy = g_windowHeight - y - 1 - g_mouseClickY;
  
  RigTForm rotatR;
  const RigTForm eyeRbt = getEyeRbt();
  
  if (g_currentPickedRbtNode != shared_ptr<SgRbtNode>() || (viewMode == 0 && skyFrameMode == 0))
    rotatR = calculate_Rotation(Cvec2(g_mouseClickX, g_mouseClickY), Cvec2(x, g_windowHeight - y - 1));
  else
    rotatR = RigTForm(Quat::makeXRotation(-dy) * Quat::makeYRotation(dx));

  const RigTForm transR = RigTForm(Cvec3(dx, dy, 0) * g_arcballScale);
  const RigTForm scaleR = RigTForm(Cvec3(0, 0, dy) * 0.01);
  RigTForm newFrame;
  shared_ptr<SgRbtNode> curObjNode;

  if(g_currentPickedRbtNode == shared_ptr<SgRbtNode>()){    // camera == picked Object
    curObjNode = getCurrentViewNode();

    if (curObjNode == g_skyNode && skyFrameMode == 0)
        newFrame = makeMixedFrame_r(getPathAccumRbt(g_world, g_world), getPathAccumRbt(g_world, g_skyNode));
    else 
        newFrame = getPathAccumRbt(g_world, curObjNode);

    if (g_mouseLClickButton && !g_mouseRClickButton) { // left button down?
      curObjNode ->setRbt(doMtoOwrtA_r(inv(rotatR), newFrame, getPathAccumRbt(g_world, curObjNode)));
    }
    else if (g_mouseRClickButton && !g_mouseLClickButton) { // right button down?
      curObjNode ->setRbt(getPathAccumRbt(g_world, curObjNode) * inv(transR));
    }
    else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) {  // middle or (left and right) button down?
      curObjNode ->setRbt(getPathAccumRbt(g_world, curObjNode) * scaleR);
    }
  }else {
    shared_ptr<SgRbtNode> curViewNode = getCurrentViewNode();
      
    curObjNode = g_currentPickedRbtNode;

    newFrame = makeMixedFrame_r(getPathAccumRbt(g_world, curObjNode), eyeRbt);
    if (g_mouseLClickButton && !g_mouseRClickButton) { // left button down?
      curObjNode->setRbt(inv(getPathAccumRbt(g_world, curObjNode, 1)) * doMtoOwrtA_r(rotatR, newFrame, getPathAccumRbt(g_world, curObjNode)));
    }
    else if (g_mouseRClickButton && !g_mouseLClickButton) { // right button down?
      curObjNode->setRbt(inv(getPathAccumRbt(g_world, curObjNode, 1)) * doMtoOwrtA_r(transR, newFrame, getPathAccumRbt(g_world, curObjNode)));
    }
    else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) {  // middle or (left and right) button down?
      curObjNode->setRbt(inv(getPathAccumRbt(g_world, curObjNode, 1)) * doMtoOwrtA_r(inv(scaleR), newFrame, getPathAccumRbt(g_world, curObjNode)));
    }
  }

  if (g_mouseClickDown) {
    //g_objectRbt[0] *= m; // Simply right-multiply is WRONG
    //g_objectRbt[1] *= m;
    //updateArcballScale();
    glutPostRedisplay(); // we always redraw if we changed the scene
  }

  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;
}


static void pick() {
  // We need to set the clear color to black, for pick rendering.
  // so let's save the clear color
  GLdouble clearColor[4];
  glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

  glClearColor(0, 0, 0, 0);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // No more glUseProgram
  drawStuff(true); // no more curSS

  // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
  // to see result of the pick rendering pass
  // glutSwapBuffers();

  //Now set back the clear color
  glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

  checkGlErrors();
}

static void mouse(const int button, const int state, const int x, const int y) {
  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;  // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

  g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
  g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
  g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

  g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
  g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
  g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

  g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;
  
  if (g_mouseLClickButton && g_picking)
    pick();

  glutPostRedisplay();
}


bool interpolateAndDisplay(float t) {
  vector<RigTForm> curFrame;
  const int curFrameIdx = floor(t);
  if (curFrameIdx >= g_animationState.keyFrameList.size() - 2)
    return true;

  const float alpha = t - curFrameIdx;
  
  auto iter = next(g_animationState.keyFrameList.begin(), curFrameIdx);
  const vector<RigTForm>& frame0 = *next(g_animationState.keyFrameList.begin(), curFrameIdx-1);
  const vector<RigTForm>& frame1 = *iter;
  const vector<RigTForm>& frame2 = *next(iter);
  const vector<RigTForm>& frame3 = *next(iter, 2);

  for (int i = 0; i < g_animationState.numRbtNodes-1; i++) {
    RigTForm interpolateRbt = RigTForm(cvecCRS(frame0[i].getTranslation(), frame1[i].getTranslation(), frame2[i].getTranslation(), frame3[i].getTranslation(), alpha),
                                    quatCRS(frame0[i].getRotation(), frame1[i].getRotation(), frame2[i].getRotation(), frame3[i].getRotation(), alpha));
    curFrame.push_back(interpolateRbt);
  }

  for (int i = 0; i < g_animationState.nodePtrVec.size()-1; i++) {
    g_animationState.nodePtrVec[i]->setRbt(curFrame[i]);
  }
  
  glutPostRedisplay();

  return false;
}
// Interpret "ms" as milliseconds into the animation

static void animateTimerCallback(int ms) {
  float t = (float)ms / (float) g_animationState.msBetweenKeyFrames;
  bool endReached = interpolateAndDisplay(t);
  if (!endReached && g_animationState.isPlaying())
    glutTimerFunc(1000 / g_animationState.animateFramesPerSecond,
                  animateTimerCallback,
                  ms + 1000 / g_animationState.animateFramesPerSecond);
  else {
    g_animationState.setFrameIdx(g_animationState.keyFrameList.size()-2);
    g_animationState.loadCurrentFrame();
    g_animationState.printCurrentFrameIdx();
    g_animationState.playing = false;
    glutPostRedisplay();
    printf("Finished playing animation\n");
  }
}

void interpolateAndDisplay_cube(float t) {
  *g_animatedMesh = *g_cubeMesh_vec[0];
  for (int i = 0; i < 8; i++) {
    Cvec3 v0_pos = g_cubeMesh_vec[0]->getVertex(i).getPosition();
    double scale = 1.0 + 0.7 * sin((t + 1) * (i%7 + 2));
    g_animatedMesh->getVertex(i).setPosition(v0_pos * scale);
  }
  for (int i = 0; i < subdivision_level; i++) {
    update_subdivision(g_animatedMesh);
  }
  glutPostRedisplay();
}
// Interpret "ms" as milliseconds into the animation

static void animateTimerCallback_cube(int ms) {
  if(ms < 0) ms = 0;
  g_time = ms;
  float t = ms / 1000.0f;
  interpolateAndDisplay_cube(t);
  if (g_cubeAS.isPlaying())
    glutTimerFunc(1000 / g_cubeAS.animateFramesPerSecond,
                  animateTimerCallback_cube,
                  1000 * (t + bubble_speed));
  else {
    //g_cubeAS.setFrameIdx(g_cubeAS.keyFrameList.size()-2);
    //g_cubeAS.loadCurrentFrame();
    //g_cubeAS.printCurrentFrameIdx();
    g_cubeAS.playing = false;
    g_time = ms;
    glutPostRedisplay();
    printf("Bubbling stopped\n");
  }
}

static void keyboard(const unsigned char key, const int x, const int y) {
  switch (key) {
  case ESC:
    exit(0);                                  // ESC
  case 'h':
    cout << " ============== H E L P ==============\n\n"
    << "h\t\thelp menu\n"
    << "s\t\tsave screenshot\n"
    << "p\t\tUse mouse to pick a part to edit\n"
    << "v\t\tCycle view\n"
    << "drag left mouse to rotate\n"
    << "a\t\tToggle display arcball\n"
    << "w\t\tWrite animation to animation.txt\n"
    << "i\t\tRead animation from animation.txt\n"
    << "<space>\t\tCopy frame to scene\n"
    << "u\t\tCopy sceneto frame\n"
    << "n\t\tCreate new frame after current frame and copy scene to it\n"
    << "d\t\tDelete frame\n"
    << ">\t\tGo to next frame\n"
    << "<\t\tGo to prev. frame\n"
    << "y\t\tPlay/Stop animation\n" << endl
    << "9\t\tDecrease subdivision levels\n" << endl
    << "0\t\tIncrease subdivision levels\n" << endl
    << "b\t\tToggle subdivison surface bubbling\n" << endl
    << "f\t\tToggle faceted shading of subdivision surface\n" << endl
    << "7\t\tDecrease subdivision surface bubbling speed\n" << endl
    << "8\t\tIncrease subdivision surface bubbling speed\n" << endl;
    break;
  case 's':
    glFlush();
    writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
    break;

  case 'm':
    skyFrameMode ^= 1;
    if (skyFrameMode == 0) printf("Editing sky eye w.r.t. world-sky frame\n\n");
    else                   printf("Editing sky eye w.r.t. sky-sky frame\n\n");
    break;
  case 'p':
    g_picking = !g_picking;

    if (!g_picking) printf("Picking mode is off\n");
    else            printf("Picking mode is on\n");

    break;
  case 'v':
    viewMode++;
	viewMode %= 3;
    switch (viewMode) {
    case 0:
      printf("Active eye is Sky\n");
      break;
    case 1:
      printf("Active eye is Robot1\n");
      break;
    case 2:
      printf("Active eye is Robot2\n");
      break;
    }
    break;
  
  //asst5
  case 'a':
    g_arcballVisible = !g_arcballVisible;
    break;
  case SPACE:
    if (g_animationState.isPlaying()){
      printf("Cannot operate when playing animation\n");
      break;
    }
    g_animationState.loadCurrentFrame();
    if(g_animationState.hasKeyFrame())
      printf("Loading current key frame [%d] to scene graph\n", g_animationState.curFrameIdx);
    break;
  case 'u':
    if (g_animationState.isPlaying()){
      printf("Cannot operate when playing animation\n");
      break;
    }
    g_animationState.updateKeyFrame ();
    break;
  case 'n':
    if (g_animationState.isPlaying()){
      printf("Cannot operate when playing animation\n");
      break;
    }
    g_animationState.createKeyFrame ();
    break;
  case '>':
    if (g_animationState.isPlaying()){
      printf("Cannot operate when playing animation\n");
      break;
    }
    g_animationState.moveCurrentFrame(1);
    break;
  case '<':
    if (g_animationState.isPlaying()){
      printf("Cannot operate when playing animation\n");
      break;
    }
    g_animationState.moveCurrentFrame(-1);
    break;
  case 'd':
    if (g_animationState.isPlaying()){
      printf("Cannot operate when playing animation\n");
      break;
    }
    g_animationState.deleteCurrentFrame();
    break;
  case 'i':
    if (g_animationState.isPlaying()){
      printf("Cannot operate when playing animation\n");
      break;
    }
    g_animationState.readKeyFrameList();
    break;
  case 'w':
    g_animationState.writeKeyFrameList();
    break;
  case 'y':
    if(g_animationState.keyFrameList.size() < 4){
      printf("Cannot play animation with less than 4 keyframes.\n");
      break;
    }

    g_animationState.playing = !g_animationState.playing;
    if (g_animationState.isPlaying()){
      animateTimerCallback(g_animationState.msBetweenKeyFrames);
      printf("Playing animation...\n");
    } else{
      printf("Stopping animation...\n");
    }
    break;
  case '+':
    g_animationState.updateAnimationSpeed(100);
    printf("%d ms between keyframes.\n", g_animationState.msBetweenKeyFrames);
    break;
  case '-':
    g_animationState.updateAnimationSpeed(-100);
    printf("%d ms between keyframes.\n", g_animationState.msBetweenKeyFrames);
    break;
  //asst8
  case '9':
    subdivision_level = max(0, subdivision_level-1);
    printf("Subdivision levels = %d\n", subdivision_level);
    interpolateAndDisplay_cube(g_time / 1000.0f);
    break;
  case '0':
    subdivision_level = min(7, subdivision_level + 1);
    printf("Subdivision levels = %d\n", subdivision_level);
    interpolateAndDisplay_cube(g_time / 1000.0f);
    break;
  case 'b':
    g_cubeAS.playing = !g_cubeAS.playing;
    if (g_cubeAS.isPlaying()){
      animateTimerCallback_cube(g_time);
      printf("Starts bubbling...\n");
    } else{
      printf("Stops bubbling...\n");
    }
    break;
  case 'f':
    g_smoothShading ^= 1;
    if (g_smoothShading) printf("smoothShading\n");
    else                 printf("flatShading\n");
    break;
  case '7':
    bubble_speed /= 2;
    printf("bubbling speed = %g\n", bubble_speed);
    break;
  case '8':
    bubble_speed *= 2;
    printf("bubbling speed = %g\n", bubble_speed);
    break;
  default:
    break;
  }
  glutPostRedisplay();
}

static void initGlutState(int argc, char * argv[]) {
  glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);  //  RGBA pixel channels and double buffering
  glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
  glutCreateWindow("Assignment 8");                       // title the window

  glutDisplayFunc(display);                               // display rendering callback
  glutReshapeFunc(reshape);                               // window reshape callback
  glutMotionFunc(motion);                                 // mouse movement callback
  glutMouseFunc(mouse);                                   // mouse click callback
  glutKeyboardFunc(keyboard);
}

static void initGLState() {
  glClearColor(128./255., 200./255., 255./255., 0.);
  glClearDepth(0.);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_GREATER);
  glReadBuffer(GL_BACK);
  if (!g_Gl2Compatible)
    glEnable(GL_FRAMEBUFFER_SRGB);
}

static void initMaterials() {
  // Create some prototype materials
  Material diffuse("./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader");
  Material solid("./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader");

  // copy diffuse prototype and set red color
  g_redDiffuseMat.reset(new Material(diffuse));
  g_redDiffuseMat->getUniforms().put("uColor", Cvec3f(1, 0, 0));

  // copy diffuse prototype and set blue color
  g_blueDiffuseMat.reset(new Material(diffuse));
  g_blueDiffuseMat->getUniforms().put("uColor", Cvec3f(0, 0, 1));

  // normal mapping material
  g_bumpFloorMat.reset(new Material("./shaders/normal-gl3.vshader", "./shaders/normal-gl3.fshader"));
  g_bumpFloorMat->getUniforms().put("uTexColor", shared_ptr<ImageTexture>(new ImageTexture("Fieldstone.ppm", true)));
  g_bumpFloorMat->getUniforms().put("uTexNormal", shared_ptr<ImageTexture>(new ImageTexture("FieldstoneNormal.ppm", false)));

  // copy solid prototype, and set to wireframed rendering
  g_arcballMat.reset(new Material(solid));
  g_arcballMat->getUniforms().put("uColor", Cvec3f(0.27f, 0.82f, 0.35f));
  g_arcballMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_LINE);

  // copy solid prototype, and set to color white
  g_lightMat.reset(new Material(solid));
  g_lightMat->getUniforms().put("uColor", Cvec3f(1, 1, 1));

  // pick shader
  g_pickingMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"));

  //asst8
  g_cubeMeshMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/specular-gl3.fshader"));
  g_cubeMeshMat->getUniforms().put("uColor", Cvec3f(1, 1, 0));
};

static void initGeometry() {
  initGround();
  initCubes();
  initSpheres();

  //asst8
  g_cubeMesh_vec.resize(1);
  initCubeMesh(g_cubeMesh_vec[0], g_newcube);
  update_avgNormal(g_cubeMesh_vec[0]);
  if (!g_animatedMesh)
    g_animatedMesh.reset(new Mesh());
  *g_animatedMesh = *g_cubeMesh_vec[0];
  update_avgNormal(g_animatedMesh);
  update_mesh_to_geometry(g_animatedMesh, g_newcube, g_smoothShading);
}

static void constructRobot(shared_ptr<SgTransformNode> base, shared_ptr<Material> material) {

  const float ARM_LEN = 0.7,
               ARM_THICK = 0.25,
              LEG_LEN = 1,
              LEG_THICK = 0.25,
               TORSO_LEN = 1.5,
               TORSO_THICK = 0.25,
               TORSO_WIDTH = 1,
               HEAD_RADIUS = 0.35;

  const int NUM_JOINTS = 10,
            NUM_SHAPES = 10;

  struct JointDesc {
    int parent;
    float x, y, z;
  };

  JointDesc jointDesc[NUM_JOINTS] = {
    {-1}, // torso
    {0, 0, TORSO_LEN / 2, 0}, // neck
    {0,  TORSO_WIDTH/2, TORSO_LEN/2, 0},     // upper right arm
    {2,  ARM_LEN, 0, 0},                     // lower right arm
    {0,  -TORSO_WIDTH / 2, TORSO_LEN / 2, 0}, // upper left arm
    {4,  -ARM_LEN, 0, 0},                     // lower left arm
    {0,  TORSO_WIDTH / 3, -TORSO_LEN / 2, 0}, // upper right leg
    {6,  0, -LEG_LEN, 0},                     // lower right leg
    {0,  -TORSO_WIDTH / 3, -TORSO_LEN / 2, 0}, // upper left leg
    {8,  0, -LEG_LEN, 0},                     // lower left leg
  };

  struct ShapeDesc {
    int parentJointId;
    float x, y, z, sx, sy, sz;
    shared_ptr<Geometry> geometry;
  };

  ShapeDesc shapeDesc[NUM_SHAPES] = {
    { 0,       0,       0, 0, TORSO_WIDTH, TORSO_LEN, TORSO_THICK, g_cube}, // torso
    { 1,      0,TORSO_LEN / 3, 0, HEAD_RADIUS, HEAD_RADIUS, HEAD_RADIUS, g_sphere}, // head
    { 2, ARM_LEN / 2,   0, 0, ARM_LEN/2, ARM_THICK/2, ARM_THICK/2, g_sphere}, // upper right arm
    { 3, ARM_LEN / 2,   0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower right arm
    { 4, -ARM_LEN / 2,   0, 0, ARM_LEN/2, ARM_THICK/2, ARM_THICK/2, g_sphere}, // upper left arm
    { 5, -ARM_LEN / 2,   0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower left arm
    { 6, 0,  -LEG_LEN / 2, 0, LEG_THICK/2, LEG_LEN/2, LEG_THICK/2, g_sphere}, // upper right leg
    { 7, 0,  -LEG_LEN / 2, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // lower right leg
    { 8, 0,  -LEG_LEN / 2, 0, LEG_THICK/2, LEG_LEN/2, LEG_THICK/2, g_sphere}, // upper right leg
    { 9, 0,  -LEG_LEN / 2, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // lower right leg
  };

  shared_ptr<SgTransformNode> jointNodes[NUM_JOINTS];

  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (jointDesc[i].parent == -1)
      jointNodes[i] = base;
    else {
      jointNodes[i].reset(new SgRbtNode(RigTForm(Cvec3(jointDesc[i].x, jointDesc[i].y, jointDesc[i].z))));
      jointNodes[jointDesc[i].parent]->addChild(jointNodes[i]);
    }
  }
  for (int i = 0; i < NUM_SHAPES; ++i) {
    shared_ptr<SgGeometryShapeNode> shape(
      new MyShapeNode(shapeDesc[i].geometry,
                      material, // USE MATERIAL as opposed to color
                      Cvec3(shapeDesc[i].x, shapeDesc[i].y, shapeDesc[i].z),
                      Cvec3(0, 0, 0),
                      Cvec3(shapeDesc[i].sx, shapeDesc[i].sy, shapeDesc[i].sz)));
    jointNodes[shapeDesc[i].parentJointId]->addChild(shape);
  }
}

static void initScene() {
  g_world.reset(new SgRootNode());

  g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 4.0))));

  g_groundNode.reset(new SgRbtNode());
  g_groundNode->addChild(shared_ptr<MyShapeNode>(
      new MyShapeNode(g_ground, g_bumpFloorMat, Cvec3(0, g_groundY, 0))));

  g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-5, 1, 0))));
  g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(5, 1, 0))));

  constructRobot(g_robot1Node, g_redDiffuseMat); // a Red robot
  constructRobot(g_robot2Node, g_blueDiffuseMat); // a Blue robot
  
  g_world->addChild(g_skyNode);
  g_world->addChild(g_groundNode);
  g_world->addChild(g_robot1Node);
  g_world->addChild(g_robot2Node);

  //asst6
  g_light1Node.reset(new SgRbtNode(RigTForm(Cvec3(-4.0, 1, -4.0))));
  createLight(g_light1Node, g_lightMat, g_sphere);
  g_light2Node.reset(new SgRbtNode(RigTForm(Cvec3(4.0, 3, 4.0))));
  createLight(g_light2Node, g_lightMat, g_sphere);

  g_world->addChild(g_light1Node);
  g_world->addChild(g_light2Node);

  //asst8
  g_cubeNode.reset(new SgRbtNode());
  createNewCube(g_cubeNode, g_cubeMeshMat, g_newcube);
  g_world->addChild(g_cubeNode);

  dumpSgRbtNodes(g_cubeNode, g_cubeAS.nodePtrVec);
  g_animationState.setNumRbtNodes();
  g_cubeAS.playing = true;
  //initVertexPhase(g_vertexPhase, g_animatedMesh);
  animateTimerCallback_cube(0);
  //asst5
  dumpSgRbtNodes(g_world, g_animationState.nodePtrVec);
  g_animationState.setNumRbtNodes();
}

int main(int argc, char * argv[]) {
  try {
    initGlutState(argc,argv);

    glewInit(); // load the OpenGL extensions

    cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.3") << endl;
    if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
    else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");
    
    initGLState();
    initMaterials();
    initGeometry();  //asst3
    initScene();     //asst4

    glutMainLoop();
    return 0;
  }
  catch (const runtime_error& e) {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}