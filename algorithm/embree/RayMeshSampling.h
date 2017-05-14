#pragma once

#include "ldpMat\ldp_basic_vec.h"
class RayMeshSampling
{
public:
	RayMeshSampling();
	~RayMeshSampling();

	// boundingBox: min and max
	void init(const ldp::Float3 boundingBox[2], int sampleWidth = 400, int sampleHeight = 400);

	void sample(int nVerts, const ldp::Float3* verts, int nTri, const ldp::Int3* tris,
		std::vector<ldp::Float3>& sampleVerts, std::vector<ldp::Float3>& sampleNormals);
protected:
	struct VirtualCamera
	{
		ldp::Float3 center;
		ldp::Float3 dir;
		ldp::Float3 ray_step_x;
		ldp::Float3 ray_step_y;
		int width = 0;
		int height = 0;
		int wb = 0;
		int we = 0;
		int hb = 0;
		int he = 0;
	};
	std::vector<VirtualCamera> m_vcams;
};