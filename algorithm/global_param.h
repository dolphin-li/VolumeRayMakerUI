#pragma once

namespace global_param
{
	const static int MESH_2_VOLUME_PADDING = 2;
	const static int VIRTUAL_RAY_NUM_W = 400;
	const static int VIRTUAL_RAY_NUM_H = 400;

	// volume resolution
	extern int volume_res[3];

	// this num of meshes will be written into a single hdf5file
	extern int hdf5numMeshes;

	// for view generation
	extern int random_view_num; // <= 0 means uniform sampling
	extern int view_id;	// not a valid param, just for UI usage
	extern float azimuth_min;
	extern float azimuth_max;
	extern float azimuth_step;	// only used if random_view_num <= 0
	extern float elevation_min;
	extern float elevation_max;
	extern float elevation_step;	// only used if random_view_num <= 0

	// mesh2vol related
	extern int mesh2vol_export_bool_occupy; // each occupied voxel is set to 1 and others set to 0
	extern int mesh2vol_export_float_normal; // each occupied voxel is set to 1 and others set to 0
}