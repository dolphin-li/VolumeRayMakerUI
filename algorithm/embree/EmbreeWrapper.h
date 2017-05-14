#pragma once

#include <memory>
#include "ldpMat\ldp_basic_vec.h"
class ObjMesh;

struct EmbreeDataHolder;
class EmbreeWrapper
{
public:
	struct Ray
	{
		ldp::Float3 org;
		ldp::Float3 dir;
		float tnear = 0.f;
		float tfar = FLT_MAX;
	};
	struct Result
	{
		ldp::Float3 Ng; // unnormalized geometry normal
		float u = 0.f;
		float v = 0.f;
		float tfar = FLT_MAX;
		int tri_id = -1;
	};
public:
	EmbreeWrapper();
	~EmbreeWrapper();
	void create(int nVerts, const ldp::Float3* verts, int nTriangles, const ldp::Int3* tris);
	void release();
	Result intersect(Ray ray);
private:
	std::shared_ptr<EmbreeDataHolder> m_holder;
};