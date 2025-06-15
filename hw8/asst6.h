#pragma once

#include "scenegraph.h"
#include "cvec.h"

typedef SgGeometryShapeNode MyShapeNode;

void createLight(shared_ptr<SgTransformNode> lightNode, shared_ptr<Material> material,
                 shared_ptr<Geometry> geometry) {

  const float RADIUS = 0.4;
  lightNode->addChild(shared_ptr<MyShapeNode>(
          new MyShapeNode(geometry,
                          material,
                          Cvec3(0, 0, 0),
                          Cvec3(0, 0, 0),
                          Cvec3(RADIUS, RADIUS, RADIUS))));
}