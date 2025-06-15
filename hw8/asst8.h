#pragma once

#include "mesh.h"
#include "scenegraph.h"
#include "cvec.h"
#include <random>

typedef SgGeometryShapeNode MyShapeNode;

using namespace std;

void update_mesh_to_geometry(shared_ptr<Mesh>& g_cubeMesh, shared_ptr<SimpleGeometryPN>& g_newcube, bool g_smoothShading) {
  vector<VertexPN> vtxList;
  for (int i = 0; i < g_cubeMesh->getNumFaces(); i++) {
  	const Mesh::Face face = g_cubeMesh->getFace(i);
  	const Cvec3 faceNormal = face.getNormal();
  
  	vector<Cvec3> vec(4);
    vector<Cvec3> normal(4);
  	for (int j = 0; j < 4; j++) {
  		vec[j] = face.getVertex(j).getPosition();
        normal[j] = face.getVertex(j).getNormal();
  	}

    if(g_smoothShading){
      vtxList.push_back(VertexPN(vec[0], normal[0]));
  	  vtxList.push_back(VertexPN(vec[1], normal[1]));
  	  vtxList.push_back(VertexPN(vec[2], normal[2]));
  
  	  vtxList.push_back(VertexPN(vec[0], normal[0]));
  	  vtxList.push_back(VertexPN(vec[2], normal[2]));
  	  vtxList.push_back(VertexPN(vec[3], normal[3]));
    }
    else {
      vtxList.push_back(VertexPN(vec[0], faceNormal));
  	  vtxList.push_back(VertexPN(vec[1], faceNormal));
  	  vtxList.push_back(VertexPN(vec[2], faceNormal));
  
  	  vtxList.push_back(VertexPN(vec[0], faceNormal));
  	  vtxList.push_back(VertexPN(vec[2], faceNormal));
  	  vtxList.push_back(VertexPN(vec[3], faceNormal));
    }
  }

  g_newcube->upload(&vtxList[0], vtxList.size());
}

void initCubeMesh(shared_ptr<Mesh>& g_cubeMesh, shared_ptr<SimpleGeometryPN>& g_newcube) {
  g_cubeMesh.reset(new Mesh());
  g_cubeMesh->load("cube.mesh");
  g_newcube.reset(new SimpleGeometryPN());
}

void createNewCube(shared_ptr<SgTransformNode> cubeNode, shared_ptr<Material> material,
  shared_ptr<Geometry> geometry) {
  
  const float size = 1;
  cubeNode->addChild(shared_ptr<MyShapeNode>(
  	new MyShapeNode(geometry,
  		material,
  		Cvec3(0, 0, 0),
  		Cvec3(0, 0, 0),
  		Cvec3(size, size, size))));
}

void update_avgNormal(shared_ptr<Mesh>& g_cubeMesh) {
  for (int i = 0; i < g_cubeMesh->getNumVertices(); ++i) {
    const Mesh::Vertex v = g_cubeMesh->getVertex(i);

    Cvec3 avg_normal;
    Mesh::VertexIterator it(v.getIterator()), it0(it);
    int cnt = 0;
    do
    {
      Mesh::Face face = it.getFace();
      avg_normal += face.getNormal();
      cnt++;
    } while (++it != it0);
    avg_normal /= cnt;
    v.setNormal(normalize(avg_normal));
  }
}

void update_subdivision(shared_ptr<Mesh>& g_cubeMesh) {
  int nv = g_cubeMesh->getNumVertices();

  for (int i = 0; i < g_cubeMesh->getNumFaces(); i++) {
    Mesh::Face f = g_cubeMesh->getFace(i);
    Cvec3 NewFaceVertexPos;
    for (int j = 0; j < f.getNumVertices(); j++) {
      NewFaceVertexPos += f.getVertex(j).getPosition();
    }
    NewFaceVertexPos /= f.getNumVertices();
    g_cubeMesh->setNewFaceVertex(f, NewFaceVertexPos);
  }

  for (int i = 0; i < g_cubeMesh->getNumEdges(); i++) {
    Mesh::Edge e = g_cubeMesh->getEdge(i);
    Mesh::Vertex v0 = e.getVertex(0);
    Mesh::Vertex v1 = e.getVertex(1);
    Mesh::Face f0 = e.getFace(0);
    Mesh::Face f1 = e.getFace(1);

    Cvec3 NewEdgeVertexPos = v0.getPosition() + v1.getPosition()
                            + g_cubeMesh->getNewFaceVertex(f0) + g_cubeMesh->getNewFaceVertex(f1);
    NewEdgeVertexPos /= 4;
    g_cubeMesh->setNewEdgeVertex(e, NewEdgeVertexPos);
  }

  for (int i = 0; i < nv; i++) {
    Mesh::Vertex v = g_cubeMesh->getVertex(i);
    Cvec3 vertex_neighbor_sum;
    Cvec3 face_neighbor_sum;
    Mesh::VertexIterator it(v.getIterator()), it0(it);
    int cnt = 0;
    do
    {
      Mesh::Face face = it.getFace();
      vertex_neighbor_sum += it.getVertex().getPosition();
      face_neighbor_sum += g_cubeMesh->getNewFaceVertex(face);
      cnt++;
    } while (++it != it0);
    g_cubeMesh->setNewVertexVertex(v, v.getPosition() * (1.0 - 2.0 / cnt) + vertex_neighbor_sum / (cnt * cnt * 1.0f) + face_neighbor_sum / (cnt * cnt * 1.0f));
  }
  g_cubeMesh->subdivide();
}

//void initVertexPhase(vector<double>& g_vertexPhase, const shared_ptr<Mesh>& g_cubeMesh) {
//  random_device random_seed;
//  mt19937 generator(random_seed());
//  uniform_real_distribution<> dist(1.0, 2 * CS175_PI);
//
//  for (int i = 0; i < g_cubeMesh->getNumVertices(); i++) {
//    g_vertexPhase.push_back(dist(generator));
//  }
//}