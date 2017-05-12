#include "EmbreeWrapper.h"

#include "embree\include\embree2\rtcore.h"
#include "embree\include\embree2\rtcore_ray.h"

#include "Renderable\ObjMesh.h"

#include <xmmintrin.h>
#include <pmmintrin.h>

struct EmbreeDataHolder
{
	RTCDevice device = nullptr;
	RTCScene scene = nullptr;
};
#define EMBREE_CHECK() {\
int status = rtcDeviceGetError(m_holder->device);\
if((status)!=RTC_NO_ERROR){\
 printf("[%s][%d]: embree_error=%d\n", __FILE__, __LINE__, status);\
assert(0); \
throw std::exception();\
}\
}

EmbreeWrapper::EmbreeWrapper()
{
	m_holder.reset(new EmbreeDataHolder);
	_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
	_MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
}

EmbreeWrapper::~EmbreeWrapper()
{
	release();
}

void EmbreeWrapper::create(const ObjMesh& mesh)
{
	release();
	m_holder->device = rtcNewDevice(NULL);
	EMBREE_CHECK();
	m_holder->scene = rtcDeviceNewScene(m_holder->device, RTC_SCENE_STATIC, RTC_INTERSECT1);
	EMBREE_CHECK();

	std::vector<ldp::Int3> tris;
	for (const auto& f : mesh.face_list)
		for (int k = 0; k < f.vertex_count - 2; k++)
			tris.push_back(ldp::Int3(f.vertex_index[0], f.vertex_index[k + 1], f.vertex_index[k + 2]));
	auto geomID = rtcNewTriangleMesh(m_holder->scene, RTC_GEOMETRY_STATIC,
		(int)tris.size(), (int)mesh.vertex_list.size());
	EMBREE_CHECK();

	struct Vertex { float x, y, z, a; };
	struct Triangle { int v0, v1, v2; };
	Vertex* vPtr = (Vertex*)rtcMapBuffer(m_holder->scene, geomID, RTC_VERTEX_BUFFER);
	for (int i = 0; i < (int)mesh.vertex_list.size(); i++)
	{
		vPtr[i].x = mesh.vertex_list[i][0];
		vPtr[i].y = mesh.vertex_list[i][1];
		vPtr[i].z = mesh.vertex_list[i][2];
		vPtr[i].a = 0.f;
	}
	rtcUnmapBuffer(m_holder->scene, geomID, RTC_VERTEX_BUFFER);
	EMBREE_CHECK();
	Triangle* triPtr = (Triangle*)rtcMapBuffer(m_holder->scene, geomID, RTC_INDEX_BUFFER);
	for (int i = 0; i < (int)tris.size(); i++)
	{
		triPtr[i].v0 = tris[i][0];
		triPtr[i].v1 = tris[i][1];
		triPtr[i].v2 = tris[i][2];
	}
	rtcUnmapBuffer(m_holder->scene, geomID, RTC_INDEX_BUFFER);
	EMBREE_CHECK();
	rtcCommit(m_holder->scene);
	EMBREE_CHECK();
}

void EmbreeWrapper::release()
{
	if(m_holder->scene)
		rtcDeleteScene(m_holder->scene);
	EMBREE_CHECK();
	if(m_holder->device)
		rtcDeleteDevice(m_holder->device);
	m_holder.reset(new EmbreeDataHolder);
}

EmbreeWrapper::Result EmbreeWrapper::intersect(Ray ray)
{
	RTCRay emb_ray;
	memset(&emb_ray, 0, sizeof(emb_ray));
	emb_ray.geomID = RTC_INVALID_GEOMETRY_ID;
	emb_ray.org[0] = ray.org[0];
	emb_ray.org[1] = ray.org[1];
	emb_ray.org[2] = ray.org[2];
	emb_ray.dir[0] = ray.dir[0];
	emb_ray.dir[1] = ray.dir[1];
	emb_ray.dir[2] = ray.dir[2];
	emb_ray.tnear = 0.0f;
	emb_ray.tfar = FLT_MAX;
	emb_ray.geomID = RTC_INVALID_GEOMETRY_ID;
	emb_ray.primID = RTC_INVALID_GEOMETRY_ID;
	emb_ray.mask = -1;
	emb_ray.time = 0;
	rtcIntersect(m_holder->scene, emb_ray);
	EMBREE_CHECK();
	Result result;

	if (emb_ray.geomID != RTC_INVALID_GEOMETRY_ID)
	{
		result.Ng[0] = emb_ray.Ng[0];
		result.Ng[1] = emb_ray.Ng[1];
		result.Ng[2] = emb_ray.Ng[2];
		result.tri_id = emb_ray.primID;
		result.tfar = emb_ray.tfar;
		result.u = emb_ray.u;
		result.v = emb_ray.v;
	}

	return result;
}