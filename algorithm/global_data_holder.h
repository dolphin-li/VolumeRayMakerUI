#pragma once
#include "util.h"
#include "ObjMesh.h"
#include "VolumeData.h"
#include "global_param.h"
class GlobalDataHolder
{
	enum ConfigMethod
	{
		CONFIG_BATCH_MESH2VOL = 0,
		CONFIG_BATCH_CONVERT_HDF5_FROM_0_1_TO_1_n1 = 1,
		CONFIG_BATCH_CONVERT_HDF5_FROM_FLOAT_TO_BIT1 = 2,
	}m_configMethod;
public:
	void init();

	void batch_config(const char* filename);

	void generateViews();

	static void volume2mesh(ObjMesh& mesh, const mpu::VolumeData& volume);
	static void mesh2volume(mpu::VolumeData& volume, const ObjMesh& mesh);
	static void binarizeVolume(mpu::VolumeData& volume);

	void applyView(const ObjMesh& A, ObjMesh& B, int viewId);
	void showParams();

	void loadHdf5(const char* filename);
	void loadHdf5bool(const char* filename);
	void fetchHdf5Index(int id);
protected:
	void batch_mesh2vol();

	// previously, hdf5 is saved as occupied voxel format, 
	// where occupied = 1 while empty = 0
	// now we convert to occupied = -1 and empty = 1
	void batch_convertHdf5_from_0_1_to_1_n1();

	// when using binary volume (occupied volume)
	// using 1 bit per voxel is enough
	// for the label, we still use float
	void batch_convertHdf5_from_float_to_bit1();
public:
	ObjMesh m_mesh_noRot;
	ObjMesh m_mesh;
	mpu::VolumeData m_volume;
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

	// for convert from 0_1 to 1_n1===============================
	std::string m_convertHdf5_from_0_1_to_1_n1_dir;

	// for convert from float to bit1===============================
	std::string m_convertHdf5_from_float_to_bit1_indir;
	std::string m_convertHdf5_from_float_to_bit1_outdir;
};
extern GlobalDataHolder g_dataholder;