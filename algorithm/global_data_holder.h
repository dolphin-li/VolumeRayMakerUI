#pragma once
#include "util.h"
#include "ObjMesh.h"
#include "VolumeData.h"
#include "global_param.h"
class GlobalDataHolder
{
public:
	void init();
	void batch_config(const char* filename);
	void generateViews();
	void applyView(const ObjMesh& A, ObjMesh& B, int viewId)const;
	static void volume2mesh(ObjMesh& mesh, const mpu::VolumeData& volume);
	static void mesh2volume(mpu::VolumeData& volume, const ObjMesh& mesh, ObjMesh& pointClound);
	void showParams();
	void loadHdf5(const char* filename);
	void loadHdf5bool(const char* filename);
	void fetchHdf5Index(int id);
protected:
	void batch_mesh2vol();
public:
	ObjMesh m_mesh_noRot;
	ObjMesh m_mesh;
	mpu::VolumeData m_volume, m_volumeNormal[3];
	ObjMesh m_pointCloud;
	std::vector<ldp::Float2> m_view_az_els;
	TimeStamp m_timeStamp;

	std::vector<float> m_hdf5volume;
	std::vector<float> m_hdf5label;
	std::vector<int> m_hdf5dims;

public:
	// for mesh2vol==================================
	std::string m_mesh2vol_inputDir;
	std::string m_mesh2vol_meshExt;
	std::string m_mesh2vol_outputDir;
};
extern GlobalDataHolder g_dataholder;