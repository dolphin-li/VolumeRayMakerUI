#include "RayMeshSampling.h"
#include "EmbreeWrapper.h"

RayMeshSampling::RayMeshSampling()
{
}

RayMeshSampling::~RayMeshSampling()
{
}

void RayMeshSampling::init(const ldp::Float3 box[2], int sampleWidth, int sampleHeight)
{
	const ldp::Float3 boxCenter = (box[0] + box[1]) * 0.5f;
	const ldp::Float3 boxExt = box[1] - box[0];

	m_vcams.clear();

	// eight corner points
	for (int i = 0; i < 8; i++)
	{
		const int ix = i & 0x01;
		const int iy = ((i & 0x02) >> 1);
		const int iz = ((i & 0x04) >> 2);
		VirtualCamera cam;
		cam.center = ldp::Float3(box[ix][0], box[iy][1], box[iz][2]);
		m_vcams.push_back(cam);
	} // end for i, 8 corner points

	  // six face points
	for (int i = 0; i < 6; i++)
	{
		continue;
		const float dir = float((i % 2) * 2 - 1);
		const int axis = i / 2;
		VirtualCamera cam;
		cam.center = boxCenter;
		cam.center[axis] += boxExt[axis] * 0.5f * dir;
		m_vcams.push_back(cam);
	} // end for i, six face points

	  // generate ray configerations
	for (size_t iCam = 0; iCam < m_vcams.size(); iCam++)
	{
		auto& cam = m_vcams[iCam];
		cam.dir = (boxCenter - cam.center).normalize();
		cam.ray_step_x = ldp::Float3(1, 0, 0).cross(cam.dir);
		if (ldp::Float3(0, 1, 0).cross(cam.dir).length() > cam.ray_step_x.length())
			cam.ray_step_x = ldp::Float3(0, 1, 0).cross(cam.dir);
		if (ldp::Float3(0, 0, 1).cross(cam.dir).length() > cam.ray_step_x.length())
			cam.ray_step_x = ldp::Float3(0, 0, 1).cross(cam.dir);
		cam.ray_step_x.normalizeLocal();
		cam.ray_step_y = cam.dir.cross(cam.ray_step_x);
		if (cam.ray_step_y.length() == 0.f)
		{
			printf("error: invalid virtual camera generation!");
			throw std::exception();
		}
		cam.ray_step_y.normalizeLocal();

		cam.width = sampleWidth;
		cam.height = sampleHeight;
		if (iCam >= 8)
		{
			cam.width = std::lroundf(cam.width * sqrt(3));
			cam.height = std::lroundf(cam.height * sqrt(3));
		}
		cam.wb = -cam.width / 2;
		cam.we = -cam.wb;
		cam.hb = -cam.height / 2;
		cam.he = -cam.hb;
		cam.ray_step_x /= (float)(cam.width);
		cam.ray_step_y /= (float)(cam.height);
		if (iCam >= 8)
		{
			cam.ray_step_x *= sqrt(3);
			cam.ray_step_y *= sqrt(3);
		}
	} // end for cam
}

void RayMeshSampling::sample(int nVerts, const ldp::Float3* verts, int nTris, const ldp::Int3* tris,
	std::vector<ldp::Float3>& sampleVerts, std::vector<ldp::Float3>& sampleNormals)
{
	EmbreeWrapper wrapper;
	wrapper.create(nVerts, verts, nTris, tris);

	// perform ray tracing
	sampleVerts.clear();
	sampleNormals.clear();
	std::vector<EmbreeWrapper::Ray> rays;
	std::vector<EmbreeWrapper::Result> results;
	for (int iCam = 0; iCam < (int)m_vcams.size(); iCam++)
	{
		const auto& cam = m_vcams[iCam];
		for (int y = cam.hb; y < cam.he; y++)
		{
			const int thread_id = 0;
			const ldp::Float3 ray_pos_y = cam.center + cam.ray_step_y * y;
			for (int x = cam.wb; x < cam.we; x++)
			{
				EmbreeWrapper::Ray ray;
				ray.org = ray_pos_y + cam.ray_step_x*x;
				ray.dir = cam.dir;
				auto result = wrapper.intersect(ray);
				if (result.tri_id >= 0 && result.Ng.length())
				{
					if (result.Ng.dot(0.f - ray.dir) < 0.f)
						result.Ng = 0.f - result.Ng;
					result.Ng.normalizeLocal();
					sampleNormals.push_back(result.Ng);
					sampleVerts.push_back(ray.org + result.tfar * ray.dir);
				}
			} // end for x
		} // end for y
	} // end for iCam
}