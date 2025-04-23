////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

#include <vector>
#include <string>
#include <memory>
#include <stdexcept>

#include <GL/glew.h>
#ifdef __APPLE__
#   include <GLUT/glut.h>
#else
#   include <GL/glut.h>
#endif

#include "cvec.h"
#include "matrix4.h"
#include "geometrymaker.h"
#include "ppm.h"
#include "glsupport.h"

// asst3
#include "rigtform.h"
#include "cmath"
#include "arcball.h"
#include "quat.h"
#include <algorithm>

//asst4
#include "asstcommon.h"
#include "scenegraph.h"
#include "drawer.h"
#include "picker.h"

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
static int pickMode = 0;
// ============================================================================

static const int PICKING_SHADER = 2; // index of the picking shader is g_shaerFiles
static const int g_numShaders = 3; // 3 shaders instead of 2
static const char* const g_shaderFiles[g_numShaders][2] = {
  {"./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader"},
  {"./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader"},
  {"./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"}
};
static const char * const g_shaderFilesGl2[g_numShaders][2] = {
  {"./shaders/basic-gl2.vshader", "./shaders/diffuse-gl2.fshader"},
  {"./shaders/basic-gl2.vshader", "./shaders/solid-gl2.fshader"},
  {"./shaders/basic-gl2.vshader", "./shaders/pick-gl2.fshader"}
};

static vector<shared_ptr<ShaderState> > g_shaderStates; // our global shader states

// --------- Geometry

// Macro used to obtain relative offset of a field within a struct
#define FIELD_OFFSET(StructType, field) &(((StructType *)0)->field)

// A vertex with floating point position and normal
struct VertexPN {
  Cvec3f p, n;

  VertexPN() {}
  VertexPN(float x, float y, float z,
           float nx, float ny, float nz)
    : p(x,y,z), n(nx, ny, nz)
  {}

  // Define copy constructor and assignment operator from GenericVertex so we can
  // use make* functions from geometrymaker.h
  VertexPN(const GenericVertex& v) {
    *this = v;
  }

  VertexPN& operator = (const GenericVertex& v) {
    p = v.pos;
    n = v.normal;
    return *this;
  }
};

struct Geometry {
  GlBufferObject vbo, ibo;
  int vboLen, iboLen;

  Geometry(VertexPN *vtx, unsigned short *idx, int vboLen, int iboLen) {
    this->vboLen = vboLen;
    this->iboLen = iboLen;

    // Now create the VBO and IBO
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPN) * vboLen, vtx, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * iboLen, idx, GL_STATIC_DRAW);
  }

  void draw(const ShaderState& curSS) {
    // Enable the attributes used by our shader
    safe_glEnableVertexAttribArray(curSS.h_aPosition);
    safe_glEnableVertexAttribArray(curSS.h_aNormal);

    // bind vbo
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    safe_glVertexAttribPointer(curSS.h_aPosition, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, p));
    safe_glVertexAttribPointer(curSS.h_aNormal, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, n));

    // bind ibo
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

    // draw!
    glDrawElements(GL_TRIANGLES, iboLen, GL_UNSIGNED_SHORT, 0);

    // Disable the attributes used by our shader
    safe_glDisableVertexAttribArray(curSS.h_aPosition);
    safe_glDisableVertexAttribArray(curSS.h_aNormal);
  }
};

// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_sphere;
typedef SgGeometryShapeNode<Geometry> MyShapeNode;

static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode; // used later when you do picking

// --------- Scene

static const Cvec3 g_light1(2.0, 3.0, 14.0), g_light2(-2, -3.0, -5.0);  // define two lights positions in world space
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
  // A x-z plane at y = g_groundY of dimension [-g_groundSize, g_groundSize]^2
  VertexPN vtx[4] = {
    VertexPN(-g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
    VertexPN(-g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
    VertexPN( g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
    VertexPN( g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
  };
  unsigned short idx[] = {0, 1, 2, 0, 2, 3};
  g_ground.reset(new Geometry(&vtx[0], &idx[0], 4, 6));
}

static void initCubes() {
  int ibLen, vbLen;
  getCubeVbIbLen(vbLen, ibLen);

  // Temporary storage for cube geometry
  vector<VertexPN> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeCube(1, vtx.begin(), idx.begin());
  g_cube.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
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

static void initSpheres() {
  int ibLen, vbLen;
  getSphereVbIbLen(20, 10, vbLen, ibLen);

  // Temporary storage for sphere geometry
  vector<VertexPN> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeSphere(1, 20, 10, vtx.begin(), idx.begin());
  g_sphere.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
}

// takes a projection matrix and send to the the shaders
static void sendProjectionMatrix(const ShaderState& curSS, const Matrix4& projMatrix) {
  GLfloat glmatrix[16];
  projMatrix.writeToColumnMajorMatrix(glmatrix); // send projection matrix
  safe_glUniformMatrix4fv(curSS.h_uProjMatrix, glmatrix);
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

static void drawArcBall() {
  const ShaderState& curSS = *g_shaderStates[g_activeShader];
  const Matrix4 projmat = makeProjectionMatrix();
  sendProjectionMatrix(curSS, projmat);

  const RigTForm eyeRbt = getEyeRbt();
  const RigTForm centerRbt = getCenterRbt();

  const RigTForm invEyeRbt = inv(eyeRbt);

  if(!g_mouseMClickButton && !(g_mouseLClickButton && g_mouseRClickButton))
    updateArcballScale();
  const double arcballRadius = g_arcballScale * g_arcballScreenRadius;
  const Matrix4 MVM = rigTFormToMatrix(invEyeRbt * centerRbt) * Matrix4::makeScale(Cvec3(1, 1, 1) * arcballRadius);
  const Matrix4 NMVM = normalMatrix(MVM);
  sendModelViewNormalMatrix(curSS, MVM, NMVM);
  safe_glUniform3f(curSS.h_uColor, g_arcBallColor[0], g_arcBallColor[1], g_arcBallColor[2]);

  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  g_sphere->draw(curSS);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

static shared_ptr<SgRbtNode> getCurrentViewNode() {
  switch (viewMode) {
  case 0: return g_skyNode;
  case 1: return g_robot1Node;
  case 2: return g_robot2Node;
  default: break;
  }
}

static void drawStuff(const ShaderState& curSS, bool picking) {
  // short hand for current shader state
  // const ShaderState& curSS = *g_shaderStates[g_activeShader];

  // build & send proj. matrix to vshader
  const Matrix4 projmat = makeProjectionMatrix();
  sendProjectionMatrix(curSS, projmat);
  
  const RigTForm eyeRbt = getEyeRbt();
  const RigTForm invEyeRbt = inv(eyeRbt);

  const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(g_light1, 1)); // g_light1 position in eye coordinates
  const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(g_light2, 1)); // g_light2 position in eye coordinates
  safe_glUniform3f(curSS.h_uLight, eyeLight1[0], eyeLight1[1], eyeLight1[2]);
  safe_glUniform3f(curSS.h_uLight2, eyeLight2[0], eyeLight2[1], eyeLight2[2]);

  if (!picking) {
      Drawer drawer(invEyeRbt, curSS);
      g_world->accept(drawer);

      if (g_currentPickedRbtNode != shared_ptr<SgRbtNode>() || (viewMode == 0 && skyFrameMode == 0))
        drawArcBall();
  }
  else {
      Picker picker(invEyeRbt, curSS);
      g_world->accept(picker);
      glFlush();
      g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
      if (g_currentPickedRbtNode == g_groundNode)
        g_currentPickedRbtNode = shared_ptr<SgRbtNode>();
      if (g_currentPickedRbtNode == getCurrentViewNode()) 
        g_currentPickedRbtNode = shared_ptr<SgRbtNode>();

      printf("Picking mode is off\n");
      pickMode = 0;
  }
}

static void display() {
  if(pickMode == 2){
    glClearColor(0, 0, 0, 0);
    glUseProgram(g_shaderStates[PICKING_SHADER]->program);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawStuff(*g_shaderStates[PICKING_SHADER], true);
  }

  glClearColor(128./255., 200./255., 255./255., 0.);
  glUseProgram(g_shaderStates[g_activeShader]->program);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  drawStuff(*g_shaderStates[g_activeShader], false);
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
  
  if (g_mouseLClickButton && pickMode == 1)
    pickMode = 2;

  glutPostRedisplay();
}


static void keyboard(const unsigned char key, const int x, const int y) {
  switch (key) {
  case 27:
    exit(0);                                  // ESC
  case 'h':
    cout << " ============== H E L P ==============\n\n"
    << "h\t\thelp menu\n"
    << "s\t\tsave screenshot\n"
    << "f\t\tToggle flat shading on/off.\n"
    << "p\t\tUse mouse to pick a part to edit\n"
    << "v\t\tCycle view\n"
    << "drag left mouse to rotate\n" << endl;
    break;
  case 's':
    glFlush();
    writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
    break;
  case 'f':
    g_activeShader ^= 1;
    break;
  case 'm':
    skyFrameMode ^= 1;
    if (skyFrameMode == 0) printf("Editing sky eye w.r.t. world-sky frame\n\n");
    else                   printf("Editing sky eye w.r.t. sky-sky frame\n\n");
    break;
  case 'p':
    if(pickMode != 0) pickMode = 0;
    else              pickMode = 1;

    if (pickMode == 0) printf("Picking mode is off\n");
    else               printf("Picking mode is on\n");

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
  }
  glutPostRedisplay();
}

static void initGlutState(int argc, char * argv[]) {
  glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);  //  RGBA pixel channels and double buffering
  glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
  glutCreateWindow("Assignment 4");                       // title the window

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

static void initShaders() {
  g_shaderStates.resize(g_numShaders);
  for (int i = 0; i < g_numShaders; ++i) {
    if (g_Gl2Compatible)
      g_shaderStates[i].reset(new ShaderState(g_shaderFilesGl2[i][0], g_shaderFilesGl2[i][1]));
    else
      g_shaderStates[i].reset(new ShaderState(g_shaderFiles[i][0], g_shaderFiles[i][1]));
  }
}

static void initGeometry() {
  initGround();
  initCubes();
  initSpheres();
}

static void constructRobot(shared_ptr<SgTransformNode> base, const Cvec3& color) {

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
    shared_ptr<MyShapeNode> shape(
      new MyShapeNode(shapeDesc[i].geometry,
                      color,
                      Cvec3(shapeDesc[i].x, shapeDesc[i].y, shapeDesc[i].z),
                      Cvec3(0, 0, 0),
                      Cvec3(shapeDesc[i].sx, shapeDesc[i].sy, shapeDesc[i].sz)));
    jointNodes[shapeDesc[i].parentJointId]->addChild(shape);
  }

  printf("children : %d\n", (*base).getNumChildren());
}

static void initScene() {
  g_world.reset(new SgRootNode());

  g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 4.0))));

  g_groundNode.reset(new SgRbtNode());
  g_groundNode->addChild(shared_ptr<MyShapeNode>(
                           new MyShapeNode(g_ground, Cvec3(0.1, 0.95, 0.1))));

  g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
  g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

  constructRobot(g_robot1Node, Cvec3(1, 0, 0)); // a Red robot
  constructRobot(g_robot2Node, Cvec3(0, 0, 1)); // a Blue robot

  g_world->addChild(g_skyNode);
  g_world->addChild(g_groundNode);
  g_world->addChild(g_robot1Node);
  g_world->addChild(g_robot2Node);
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
    initShaders();
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
