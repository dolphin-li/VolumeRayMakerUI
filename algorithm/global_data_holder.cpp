#include "global_data_holder.h"
#include "OpenVDBWrapper.h"
#include <fstream>
#include "hdf5.h"
#include "hdf5_hl.h"
//#include "RigidEstimation.h"
GlobalDataHolder g_dataholder;

static void convert_float_to_bool(int n, const float* A, std::vector<char>& B)
{
	if (n % 8)
		throw std::exception("convert_float_to_bool: data size must be times of 8!");
	B.resize(n / 8);
	for (size_t i = 0; i < B.size(); i++)
	{
		int apos = i * 8;
		char data = 0;
		for (int k = 0; k < 8; k++, apos++)
		{
			bool inner = A[apos] <= 0;
			data |= (inner << k);
		}
		B[i] = data;
	}
}

static void convert_float_to_bool(const std::vector<float>& A, std::vector<char>& B)
{
	convert_float_to_bool(A.size(), A.data(), B);
}

static void convert_bool_to_float(std::vector<float>& A, const std::vector<char>& B)
{
	A.resize(B.size() * 8);
	for (size_t i = 0; i < B.size(); i++)
	{
		int apos = i * 8;
		char data = B[i];
		for (int k = 0; k < 8; k++, apos++)
		{
			bool inner = (data >> k) & 0x1;
			if (inner)
				A[apos] = -0.1f;
			else
				A[apos] = 0.1f;
		}
	}
}

void GlobalDataHolder::init()
{
	//rigid_estimation::debug_estimation();
	global_param::volume_res[0] = 30;
	global_param::volume_res[1] = 30;
	global_param::volume_res[2] = 30;
	global_param::hdf5numMeshes = 1;
	global_param::random_view_num = -1;
	global_param::view_id = 0;
	global_param::azimuth_min = 0;
	global_param::azimuth_max = 360;
	global_param::azimuth_step = 30;
	global_param::elevation_min = 0;
	global_param::elevation_max = 40;
	global_param::elevation_step = 45;
	global_param::mesh2vol_isoValue = 0.6f;
	global_param::mesh2vol_occupy_1 = true;
	m_hdf5dims.resize(5, 0);
	m_volume.resize(global_param::volume_res[0], global_param::volume_res[1], global_param::volume_res[2]);
}

void GlobalDataHolder::generateViews()
{
	m_view_az_els.clear();

	// uniform sampling
	if (global_param::random_view_num <= 0)
	{
		m_view_az_els.push_back(ldp::Float2(0, 0));
		for (float ele = global_param::elevation_min;
			ele < global_param::elevation_max - 1e-3f;
			ele += global_param::elevation_step)
		{
			for (float az = global_param::azimuth_min;
				az < global_param::azimuth_max - 1e-3f;
				az += global_param::azimuth_step)
			{
				if (abs(ele) < 1e-3 && abs(az) < 1e-3)
					continue;
				m_view_az_els.push_back(ldp::Float2(az, ele));
			}
		}
		printf("%d views generated\n", m_view_az_els.size());
	}
	// random sampling
	else
	{
		m_view_az_els.push_back(ldp::Float2(0, 0));
		for (int i = 0; i < global_param::random_view_num - 1; i++)
		{
			float ele = float(rand()) / float(RAND_MAX) *
				(global_param::elevation_max - global_param::elevation_min)
				+ global_param::elevation_min;
			float az = float(rand()) / float(RAND_MAX) *
				(global_param::azimuth_max - global_param::azimuth_min)
				+ global_param::azimuth_min;
			m_view_az_els.push_back(ldp::Float2(az, ele));
		}
		printf("%d random views generated\n", m_view_az_els.size());
	}
}

void GlobalDataHolder::applyView(const ObjMesh& A, ObjMesh& B, int viewId)
{
	ldp::Float2 az_ele = m_view_az_els.at(viewId);
	Camera cam;
	cam.setModelViewMatrixFromEuler(-90, az_ele[1], az_ele[0], 1);
	ldp::Mat3f R = cam.getModelViewMatrix().getRotationPart();
	ldp::Float3 c = (A.boundingBox[0] + A.boundingBox[1]) * 0.5f;
	B = A;
	for (size_t i = 0; i < B.vertex_list.size(); i++)
	{
		ldp::Float3& v = B.vertex_list[i];
		v = R * (v - c) + c;
	}
	B.updateNormals();
	B.updateBoundingBox();
}

void GlobalDataHolder::volume2mesh(ObjMesh& mesh, const mpu::VolumeData& volume)
{
	FloatGrid::Ptr grid;
	OpenVDBWrapper::fromDenseVolume(grid, volume);
	OpenVDBWrapper::toObjMesh(grid, mesh, global_param::VOLUME_MESH_ADAPTIVE_THRE);
}

void GlobalDataHolder::mesh2volume(mpu::VolumeData& volume, const ObjMesh& mesh)
{
	ldp::Int3 res(global_param::volume_res[0], global_param::volume_res[1], global_param::volume_res[2]);
	ldp::Int3 res_p = res - 2 * global_param::MESH_2_VOLUME_PADDING;
	kdtree::AABB box(mesh.boundingBox[0], mesh.boundingBox[1]);
	ldp::Float3 rg = box.getExtents();
	float vsz = std::max(rg[0] / res_p[0], std::max(rg[1] / res_p[1], rg[2] / res_p[2]));
	volume.resize(res, vsz);
	for (int k = 0; k < 3; k++)
	{
		box.min[k] -= (vsz - rg[k] / res[k]) * res[k] * 0.5f;
		box.max[k] += (vsz - rg[k] / res[k]) * res[k] * 0.5f;
	}
	volume.setBound(box);

	FloatGrid::Ptr grid;
	OpenVDBWrapper::fromObjMesh(grid, mesh, volume.getVoxelSize(), global_param::mesh2vol_isoValue);
	OpenVDBWrapper::toDenseVolume(grid, volume);
}

void GlobalDataHolder::binarizeVolume(mpu::VolumeData& volume)
{
	ldp::Int3 res = volume.getResolution();
	int n = res[0] * res[1] * res[2];
	for (int i = 0; i < n; i++)
	{
		if (volume.data()[i] <= 0)
			volume.data()[i] = -0.5;
		else
			volume.data()[i] = 0.5;
	}
}

void GlobalDataHolder::showParams()
{
	m_timeStamp.Stamp("VOLUME_BAND_HALF_WIDTH: %f", global_param::VOLUME_BAND_HALF_WIDTH);
	m_timeStamp.Stamp("VOLUME_MESH_ADAPTIVE_THRE: %f", global_param::VOLUME_MESH_ADAPTIVE_THRE);
	m_timeStamp.Stamp("MESH_2_VOLUME_PADDING: %d", global_param::MESH_2_VOLUME_PADDING);
	m_timeStamp.Stamp("volume_res: %d %d %d", global_param::volume_res[0], global_param::volume_res[1],
		global_param::volume_res[2]);
	m_timeStamp.Stamp("hdf5numMeshes: %d", global_param::hdf5numMeshes);
	m_timeStamp.Stamp("mesh2vol_isoValue: %f", global_param::mesh2vol_isoValue);
	m_timeStamp.Stamp("mesh2vol_occupy_1: %d", global_param::mesh2vol_occupy_1);
	m_timeStamp.Stamp("random_view_num: %d", global_param::random_view_num);
	m_timeStamp.Stamp("azimuth: [%f:%f:%f]", global_param::azimuth_min, 
		global_param::azimuth_step, global_param::azimuth_max);
	m_timeStamp.Stamp("elevation: [%f:%f:%f]\n", global_param::elevation_min,
		global_param::elevation_step, global_param::elevation_max);
}

void GlobalDataHolder::batch_config(const char* filename)
{
	m_timeStamp.Reset();
	m_timeStamp.Prefix("global data holder");
	m_timeStamp.logToFile((std::string(filename) + ".log.txt").c_str());

	// parse config file
	std::ifstream stm(filename);
	if (stm.fail())
		throw std::exception((std::string("error openning: ") + filename).c_str());
	std::string lineBuffer;
	while (!stm.eof())
	{
		std::getline(stm, lineBuffer);
		if (lineBuffer[0] == '#')
			continue;

		std::string lineLabel = getLineLabel(lineBuffer);
		if (lineLabel == "config_method")
			m_configMethod = (ConfigMethod)atoi(lineBuffer.c_str());
		// params
		else if (lineLabel == "param_volRes")
			sscanf(lineBuffer.c_str(), "%d %d %d", &global_param::volume_res[0],
			&global_param::volume_res[1], &global_param::volume_res[2]);
		else if (lineLabel == "hdf5numMeshes")
			global_param::hdf5numMeshes = atof(lineBuffer.c_str());
		else if (lineLabel == "rand_view_num")
			global_param::random_view_num = atoi(lineBuffer.c_str());
		else if (lineLabel == "azimuth_min")
			global_param::azimuth_min = atof(lineBuffer.c_str());
		else if (lineLabel == "azimuth_max")
			global_param::azimuth_max = atof(lineBuffer.c_str());
		else if (lineLabel == "azimuth_step")
			global_param::azimuth_step = atof(lineBuffer.c_str());
		else if (lineLabel == "elevation_min")
			global_param::elevation_min = atof(lineBuffer.c_str());
		else if (lineLabel == "elevation_max")
			global_param::elevation_max = atof(lineBuffer.c_str());
		else if (lineLabel == "elevation_step")
			global_param::elevation_step = atof(lineBuffer.c_str());
		// mesh2vol related
		else if (lineLabel == "mesh2vol_isoValue")
			global_param::mesh2vol_isoValue = atof(lineBuffer.c_str());
		else if (lineLabel == "mesh2vol_occupy_1")
			global_param::mesh2vol_occupy_1 = atoi(lineBuffer.c_str());
		else if (lineLabel == "mesh2vol_inputDir")
			m_mesh2vol_inputDir = lineBuffer;
		else if (lineLabel == "mesh2vol_outputDir")
			m_mesh2vol_outputDir = lineBuffer;
		else if (lineLabel == "mesh2vol_meshExt")
			m_mesh2vol_meshExt = lineBuffer;
		else if (lineLabel == "convertHdf5_from_0_1_to_1_n1_dir")
			m_convertHdf5_from_0_1_to_1_n1_dir = lineBuffer;
		else if (lineLabel == "convertHdf5_from_float_to_bit1_indir")
			m_convertHdf5_from_float_to_bit1_indir = lineBuffer;
		else if (lineLabel == "convertHdf5_from_float_to_bit1_outdir")
			m_convertHdf5_from_float_to_bit1_outdir = lineBuffer;
	}
	stm.close();
	m_timeStamp.Stamp("config file parsed: ");
	showParams();
	m_timeStamp.flushLog();

	// different batch config
	switch (m_configMethod)
	{
	case GlobalDataHolder::CONFIG_BATCH_MESH2VOL:
		batch_mesh2vol();
		break;
	case GlobalDataHolder::CONFIG_BATCH_CONVERT_HDF5_FROM_0_1_TO_1_n1:
		batch_convertHdf5_from_0_1_to_1_n1();
		break;
	case GlobalDataHolder::CONFIG_BATCH_CONVERT_HDF5_FROM_FLOAT_TO_BIT1:
		batch_convertHdf5_from_float_to_bit1();
		break;
	default:
		break;
	}
}

inline std::string get_modelnet40_label(std::string meshName)
{
	std::string path, name, ext, path1, path2;
	ldp::fileparts(meshName, path, name, ext);
	path1 = ldp::parentpath(path);
	path2 = ldp::parentpath(path1);
	return path1.substr(path2.size(), path1.size() - 1 - path2.size());
}

void GlobalDataHolder::batch_mesh2vol()
{
	m_timeStamp.Prefix("mesh2vol");
	m_timeStamp.Stamp("from %s to %s", m_mesh2vol_inputDir.c_str(), m_mesh2vol_outputDir.c_str());
	m_timeStamp.flushLog();

	// gather all meshes
	std::vector<std::string> loadedMeshNames;
	getAllFilesInDir(m_mesh2vol_inputDir, loadedMeshNames, m_mesh2vol_meshExt);
	m_timeStamp.Stamp("%d <%s> meshes founded", loadedMeshNames.size(), m_mesh2vol_meshExt.c_str());
	m_timeStamp.flushLog();
	std::string outExt = ".hdf5";
	if (global_param::mesh2vol_occupy_1)
		outExt = ".hdf5_bool";

	// divide into train/test
	std::vector<std::string> trainMeshNames;
	std::vector<std::string> testMeshNames;
	std::vector<std::string> trainTestMeshNames;
	std::vector<int> trainMeshRandIds, testMeshRandIds, trainTestMeshRandIds;
	for (size_t i = 0; i < loadedMeshNames.size(); i++)
	{
		std::string meshName = loadedMeshNames[i];
		std::string path, name, ext;
		ldp::fileparts(meshName, path, name, ext);
		std::string parentPath = ldp::parentpath(path);
		std::string flag = path.substr(parentPath.size(), path.size()-1-parentPath.size());
		if (flag == "train")
		{
			trainMeshRandIds.push_back(trainMeshNames.size());
			trainMeshNames.push_back(meshName);
		}
		else if (flag == "test")
		{
			testMeshRandIds.push_back(testMeshNames.size());
			testMeshNames.push_back(meshName);
		}
	}
	for (size_t i = 0; i < testMeshRandIds.size(); i++)
		testMeshRandIds[i] += trainMeshNames.size();
	m_timeStamp.Stamp("%d training and %d testing meshes founded", 
		trainMeshNames.size(), testMeshNames.size());
	m_timeStamp.flushLog();
	trainTestMeshNames = trainMeshNames;
	trainTestMeshNames.insert(trainTestMeshNames.end(), testMeshNames.begin(), testMeshNames.end());
	srand(1234);
	std::random_shuffle(trainMeshRandIds.begin(), trainMeshRandIds.end());
	//std::random_shuffle(testMeshRandIds.begin(), testMeshRandIds.end()); // we do not need to shuffle test
	trainTestMeshRandIds = trainMeshRandIds;
	trainTestMeshRandIds.insert(trainTestMeshRandIds.end(), testMeshRandIds.begin(), testMeshRandIds.end());
	const int nTrainTestMeshes = trainTestMeshNames.size();
	const int nTrainMeshes = trainMeshNames.size();
	const int nTestMeshes = testMeshNames.size();

	// generate sampling views
	generateViews();

	// generating label mapping
	std::map<std::string, int> labelMaps;
	for (size_t i = 0; i < testMeshNames.size(); i++)
		labelMaps.insert(std::make_pair(get_modelnet40_label(testMeshNames[i]), (int)labelMaps.size()));
	for (auto it = labelMaps.begin(); it != labelMaps.end(); ++it)
		m_timeStamp.Stamp("label maps: %s -> %d", it->first, it->second);
	m_timeStamp.flushLog();

	// write hdf5 now
	ldp::Int3 res(global_param::volume_res[0], global_param::volume_res[1], global_param::volume_res[2]);
	const int nViewPerMesh = m_view_az_els.size();
	int nVoxelsPerView = res[0] * res[1] * res[2];
	if (global_param::mesh2vol_occupy_1 && (nVoxelsPerView % 8))
		throw std::exception("when outputing binary volume, the resolution and views must be even!");
	std::vector<float> tmpVolumeBuffer;
	std::vector<char> tmpVolumeBufferBool;
	if (global_param::mesh2vol_occupy_1)
		tmpVolumeBufferBool.resize(global_param::hdf5numMeshes*nVoxelsPerView*nViewPerMesh / 8);
	else
		tmpVolumeBuffer.resize(global_param::hdf5numMeshes*nVoxelsPerView*nViewPerMesh);
	std::vector<float> tmpLabelBuffer(global_param::hdf5numMeshes*nViewPerMesh);
	for (int iMeshAry = 0; iMeshAry < nTrainTestMeshes; iMeshAry++)
	{
		int meshId = trainTestMeshRandIds[iMeshAry];
		std::string meshName = trainTestMeshNames[meshId];
		auto labelIter = labelMaps.find(get_modelnet40_label(meshName));
		std::string trainTestFlag = "train_";
		if (iMeshAry >= nTrainMeshes)
			trainTestFlag = "test_";
		int batch_offset = iMeshAry % global_param::hdf5numMeshes;
		int batch_start = iMeshAry - batch_offset;
		if (iMeshAry >= nTrainMeshes)
		{
			batch_offset = (iMeshAry - nTrainMeshes) % global_param::hdf5numMeshes;
			batch_start = iMeshAry - nTrainMeshes - batch_offset;
		}

		// if there exists hdf5 file, just skip
		std::string hname = fullfile(m_mesh2vol_outputDir, trainTestFlag 
			+ std::to_string(batch_start) + outExt);
		if (ldp::file_exist(hname.c_str()))
		{
			m_timeStamp.Stamp("skipped: %s", meshName.c_str());
			continue;
		}

		// load current mesh
		ObjMesh meshNoR;
		if (m_mesh2vol_meshExt == "off")
			meshNoR.loadOff(meshName.c_str(), true);
		else if (m_mesh2vol_meshExt == "obj")
			meshNoR.loadObj(meshName.c_str(), true, true);
		else
			throw std::exception("error: non-supported mesh extension");

		// re-generate views if random required
		if (global_param::random_view_num > 0)
		{
			generateViews();
			if (m_view_az_els.size() != nViewPerMesh)
				throw std::exception("view size not matched after random generation!");
		}

		// write mesh info for each view
	#pragma omp parallel for
		for (int iView = 0; iView < nViewPerMesh; iView++)
		{
			ObjMesh mesh;
			VolumeData volume;
			applyView(meshNoR, mesh, iView);
			mesh2volume(volume, mesh);

			// convert to occupy volume
			if (global_param::mesh2vol_occupy_1)
			{
				std::vector<char> tmpBoolBuffer;
				convert_float_to_bool(nVoxelsPerView, volume.data(), tmpBoolBuffer);
				char* bufferPtr = tmpVolumeBufferBool.data() + (batch_offset*nViewPerMesh + iView)*nVoxelsPerView/8;
				for (size_t i = 0; i < tmpBoolBuffer.size(); i++)
					bufferPtr[i] = tmpBoolBuffer[i];
			}// end conversion
			else
			{
				float* bufferPtr = tmpVolumeBuffer.data() + (batch_offset*nViewPerMesh + iView)*nVoxelsPerView;
				for (int i = 0; i < nVoxelsPerView; i++)
					bufferPtr[i] = volume.data()[i];
			}
			tmpLabelBuffer[batch_offset*nViewPerMesh + iView] = labelIter->second;
		}// end for iView
		m_timeStamp.Stamp("mesh(%s) %d <- %d, label=%s, %d processed", meshName.c_str(),
			iMeshAry, meshId, labelIter->first.c_str(), labelIter->second);
		m_timeStamp.flushLog();

		// create H5File
		if (batch_offset + 1 == global_param::hdf5numMeshes
			|| iMeshAry + 1 == nTrainMeshes
			|| iMeshAry + 1 == nTrainTestMeshes)
		{
			hid_t file_id = H5Fcreate(hname.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
			if (file_id < 0)
				throw std::exception("H5Fcreate failed!");
			m_timeStamp.Stamp("hdf5 created: %s", hname.c_str());
			m_timeStamp.flushLog();
			herr_t err = 0;
			int meshesThisBatch = batch_offset + 1;
			if (global_param::mesh2vol_occupy_1)
			{
				hsize_t dims[5] = { meshesThisBatch*nViewPerMesh/8, 1, res[0], res[1], res[2] };
				int res = meshesThisBatch*nViewPerMesh - dims[0]*8;
				if (res)
					m_timeStamp.Stamp("warning: %d volumes dropped due to non-8-times!", res);
				err = H5LTmake_dataset_char(file_id, "volume", 5, dims, tmpVolumeBufferBool.data());
				if (err)
					throw std::exception("H5LTmake_dataset_float volume failed!");
				hsize_t dims2[2] = { dims[0]*8, 1 };
				err = H5LTmake_dataset_float(file_id, "label", 2, dims2, tmpLabelBuffer.data());
				if (err)
					throw std::exception("H5LTmake_dataset_float  label failed!");
			}
			else
			{
				hsize_t dims[5] = { meshesThisBatch*nViewPerMesh, 1, res[0], res[1], res[2] };
				err = H5LTmake_dataset_float(file_id, "volume", 5, dims, tmpVolumeBuffer.data());
				if (err)
					throw std::exception("H5LTmake_dataset_float volume failed!");
				hsize_t dims2[2] = { meshesThisBatch*nViewPerMesh, 1 };
				err = H5LTmake_dataset_float(file_id, "label", 2, dims2, tmpLabelBuffer.data());
				if (err)
					throw std::exception("H5LTmake_dataset_float  label failed!");
			}
			err = H5Fclose(file_id);
			if (err)
				throw std::exception("H5Fclose failed!");
			m_timeStamp.Stamp("hdf5 closed");
			m_timeStamp.flushLog();
		}
	}// end for iMesh
}

void GlobalDataHolder::batch_convertHdf5_from_0_1_to_1_n1()
{
	m_timeStamp.Prefix("mesh2vol");
	m_timeStamp.Stamp("convert all hdf5 0_1 to 1_n1 in: ", m_convertHdf5_from_0_1_to_1_n1_dir.c_str());
	m_timeStamp.flushLog();

	// gather all hdf5s
	std::vector<std::string> hdf5Names;
	getAllFilesInDir(m_convertHdf5_from_0_1_to_1_n1_dir, hdf5Names, "hdf5");
	m_timeStamp.Stamp("%d .hdf5 founded", hdf5Names.size());
	m_timeStamp.flushLog();

	// batch convert
	for (size_t iName = 0; iName < hdf5Names.size(); iName++)
	{
		try
		{
			loadHdf5(hdf5Names[iName].c_str());

			// write hdf5
			hid_t file_id = H5Fcreate(hdf5Names[iName].c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
			if (file_id < 0)
				throw std::exception("H5Fcreate failed!");
			m_timeStamp.Stamp("hdf5 created: %s", hdf5Names[iName].c_str());
			m_timeStamp.flushLog();
			herr_t err = 0;
			hsize_t dims[5] = { m_hdf5dims[0], m_hdf5dims[1], m_hdf5dims[2], m_hdf5dims[3], m_hdf5dims[4] };
			int cnt = m_hdf5dims[0] * m_hdf5dims[1] * m_hdf5dims[2] * m_hdf5dims[3] * m_hdf5dims[4];
			for (int i = 0; i < cnt; i++)
			{
				float val = m_hdf5volume[i];
				if (val >= 0)
					m_hdf5volume[i] = 1;
				else
					m_hdf5volume[i] = -1;
			}
			err = H5LTmake_dataset_float(file_id, "volume", 5, dims, m_hdf5volume.data());
			if (err)
				throw std::exception("H5LTmake_dataset_float volume failed!");
			hsize_t dims2[2] = { m_hdf5dims[0], 1 };
			err = H5LTmake_dataset_float(file_id, "label", 2, dims2, m_hdf5label.data());
			if (err)
				throw std::exception("H5LTmake_dataset_float  label failed!");
			err = H5Fclose(file_id);
			if (err)
				throw std::exception("H5Fclose failed!");
			m_timeStamp.Stamp("hdf5 closed");
			m_timeStamp.flushLog();
		}
		catch (std::exception e)
		{
			std::cout << e.what() << std::endl;
		}
	}// end for iName
}

void GlobalDataHolder::batch_convertHdf5_from_float_to_bit1()
{
	m_timeStamp.Prefix("convert");
	m_timeStamp.Stamp("convert all hdf5 float to bit1 in: ", m_convertHdf5_from_float_to_bit1_indir.c_str());
	m_timeStamp.flushLog();

	// gather all hdf5s
	std::vector<std::string> hdf5Names;
	getAllFilesInDir(m_convertHdf5_from_float_to_bit1_indir, hdf5Names, "hdf5");
	m_timeStamp.Stamp("%d .hdf5 founded", hdf5Names.size());
	m_timeStamp.flushLog();

	if (!directoryExists(m_convertHdf5_from_float_to_bit1_outdir))
	{
		mkdir(m_convertHdf5_from_float_to_bit1_outdir);
		m_timeStamp.Stamp("mkdir: ", m_convertHdf5_from_float_to_bit1_outdir.c_str());
		m_timeStamp.flushLog();
	}

	// batch convert
	std::vector<float> tmpFltVolume;
	std::vector<float> tmpFltLabel;
	std::vector<char> tmpVolume;
	std::vector<float> tmpLabel;
	for (size_t iName = 0; iName < hdf5Names.size(); iName++)
	{
		try
		{
			loadHdf5(hdf5Names[iName].c_str());
			std::string path, name, ext;
			ldp::fileparts(hdf5Names[iName], path, name, ext);
			std::string outname = fullfile(m_convertHdf5_from_float_to_bit1_outdir, name + ext + "_bool");

			// merge with previous
			const int voxelPer = m_hdf5dims[1] * m_hdf5dims[2] * m_hdf5dims[3] * m_hdf5dims[4];
			int prev_cnt = tmpFltVolume.size() / voxelPer;
			int cur_cnt = m_hdf5dims[0];
			int nxt_cnt = (prev_cnt + cur_cnt) % 8;
			tmpFltVolume.insert(tmpFltVolume.end(), m_hdf5volume.begin(), 
				m_hdf5volume.begin() + (cur_cnt-nxt_cnt)*voxelPer);
			tmpFltLabel.insert(tmpFltLabel.end(), m_hdf5label.begin(),
				m_hdf5label.begin() + cur_cnt - nxt_cnt);
			convert_float_to_bool(tmpFltVolume, tmpVolume);
			tmpLabel = tmpFltLabel;
			tmpFltVolume.assign(m_hdf5volume.begin() + (cur_cnt - nxt_cnt)*voxelPer, m_hdf5volume.end());
			tmpFltLabel.assign(m_hdf5label.begin() + cur_cnt - nxt_cnt, m_hdf5label.end());

			int wt_cnt = prev_cnt + cur_cnt - nxt_cnt;
			int wt_cnt_char = wt_cnt / 8;
			if (wt_cnt_char*8 != tmpLabel.size())
				throw std::exception("error, char cnt not proper");

			// write hdf5
			hid_t file_id = H5Fcreate(outname.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
			if (file_id < 0)
				throw std::exception("H5Fcreate failed!");
			m_timeStamp.Stamp("hdf5 created: %s, cnt = %d, charCnt=%d", outname.c_str(), wt_cnt, wt_cnt_char);
			m_timeStamp.flushLog();
			herr_t err = 0;
			hsize_t dims[5] = { wt_cnt_char, m_hdf5dims[1], m_hdf5dims[2], m_hdf5dims[3], m_hdf5dims[4] };
			
			err = H5LTmake_dataset_char(file_id, "volume", 5, dims, tmpVolume.data());
			if (err)
				throw std::exception("H5LTmake_dataset_char volume failed!");
			hsize_t dims2[2] = { tmpLabel.size(), 1 };
			err = H5LTmake_dataset_float(file_id, "label", 2, dims2, tmpLabel.data());
			if (err)
				throw std::exception("H5LTmake_dataset_float  label failed!");
			err = H5Fclose(file_id);
			if (err)
				throw std::exception("H5Fclose failed!");
			m_timeStamp.Stamp("hdf5 closed");
			m_timeStamp.flushLog();
		}
		catch (std::exception e)
		{
			std::cout << e.what() << std::endl;
		}
	}// end for iName
}

void GlobalDataHolder::loadHdf5(const char* filename)
{
	m_timeStamp.Stamp("begin loading hdf5 data");
	hid_t file_id = H5Fopen(filename, H5F_ACC_RDONLY, H5P_DEFAULT);
	herr_t status = 0;
	H5T_class_t class_ = H5T_NO_CLASS;
	if (file_id < 0)
		throw std::exception("failed to open fdf5 file");

	// load volume==============================================
	if (!H5LTfind_dataset(file_id, "volume"))
		throw std::exception("cannot find dataset volume");

	int ndims = 0;
	status = H5LTget_dataset_ndims(file_id, "volume", &ndims);
	if (ndims != 5)
		throw std::exception("volume must have dims of 5: num, channel, x, y, z");
	std::vector<hsize_t> dims(ndims, 0);
	status = H5LTget_dataset_info(
		file_id, "volume", dims.data(), &class_, NULL);
	if (class_ != H5T_FLOAT)
		throw std::exception("non supported data type, must be float");
	size_t cnt = 1;
	for (int i = 0; i < ndims; i++)
	{
		m_hdf5dims[i] = dims[i];
		cnt *= m_hdf5dims[i];
	}
	m_hdf5volume.clear();
	m_hdf5volume.resize(cnt, 0.f); 
	status = H5LTread_dataset_float(file_id, "volume", m_hdf5volume.data());
	if (status != 0)
		throw std::exception("load volume data failed");
	m_timeStamp.Stamp("volume loaded: %d %d %d %d %d", dims[0], dims[1], dims[2], dims[3], dims[4]);

	// load labels =============================================
	m_hdf5label.clear();
	m_hdf5label.resize(dims[0], 0.f);
	if (H5LTfind_dataset(file_id, "label"))
	{
		status = H5LTget_dataset_ndims(file_id, "label", &ndims);
		if (ndims != 2)
			throw std::exception("label must have dims of 5: num, label");
		std::vector<hsize_t> dims(ndims, 0);
		status = H5LTget_dataset_info(
			file_id, "label", dims.data(), &class_, NULL);
		if (class_ != H5T_FLOAT)
			throw std::exception("non supported label type, must be float");
		if (dims[0] != m_hdf5dims[0])
			throw std::exception("num not matched between volume and label");
		status = H5LTread_dataset_float(file_id, "label", m_hdf5label.data());
		if (status != 0)
			throw std::exception("load label data failed");
		m_timeStamp.Stamp("label loaded: %d %d", dims[0], dims[1]);
	}// has label data

	// finally close the file
	status = H5Fclose(file_id);
	if (status)
		throw std::exception("H5Fclose failed!");
}

void GlobalDataHolder::loadHdf5bool(const char* filename)
{
	m_timeStamp.Stamp("begin loading hdf5 data");
	hid_t file_id = H5Fopen(filename, H5F_ACC_RDONLY, H5P_DEFAULT);
	herr_t status = 0;
	H5T_class_t class_ = H5T_NO_CLASS;
	size_t class_size_ = 0;
	if (file_id < 0)
		throw std::exception("failed to open fdf5 file");

	// load volume==============================================
	if (!(status = H5LTfind_dataset(file_id, "volume")))
		throw std::exception("cannot find dataset volume");

	int ndims = 0;
	status = H5LTget_dataset_ndims(file_id, "volume", &ndims);
	if (ndims != 5)
		throw std::exception("volume must have dims of 5: num, channel, x, y, z");
	std::vector<hsize_t> dims(ndims, 0); 
	status = H5LTget_dataset_info(
		file_id, "volume", dims.data(), &class_, &class_size_);
	printf("volume class, id=%d, bytes=%d\n", class_, class_size_);
	size_t cnt = 1;
	for (int i = 0; i < ndims; i++)
	{
		m_hdf5dims[i] = dims[i];
		cnt *= m_hdf5dims[i];
	}
	m_hdf5dims[0] *= 8;
	std::vector<char> tmpVolume(cnt);
	status = H5LTread_dataset_char(file_id, "volume", tmpVolume.data());
	convert_bool_to_float(m_hdf5volume, tmpVolume);
	if (status != 0)
		throw std::exception("load volume data failed");
	m_timeStamp.Stamp("volume loaded: %d %d %d %d %d", dims[0]*8, dims[1], dims[2], dims[3], dims[4]);

	// load labels =============================================
	if (H5LTfind_dataset(file_id, "label"))
	{
		status = H5LTget_dataset_ndims(file_id, "label", &ndims);
		if (ndims != 2)
			throw std::exception("label must have dims of 5: num, label");
		std::vector<hsize_t> lbdims(ndims, 0);
		status = H5LTget_dataset_info(
			file_id, "label", lbdims.data(), &class_, NULL);
		if (lbdims[0] != m_hdf5dims[0])
			throw std::exception("num not matched between volume and label");
		m_hdf5label.clear();
		m_hdf5label.resize(lbdims[0]);
		status = H5LTread_dataset_float(file_id, "label", m_hdf5label.data());
		if (status != 0)
			throw std::exception("load label data failed");
		m_timeStamp.Stamp("label loaded: %d %d", lbdims[0], lbdims[1]);
	}// has label data

	// finally close the file
	status = H5Fclose(file_id);
	if (status)
		throw std::exception("H5Fclose failed!");
}

void GlobalDataHolder::fetchHdf5Index(int id)
{
	if (id >= m_hdf5dims[0])
		throw std::exception("index out of range");
	m_volume.resize(ldp::Int3(m_hdf5dims[2], m_hdf5dims[3], m_hdf5dims[4]), 1.f / m_hdf5dims[2]);
	kdtree::AABB box;
	box.min = ldp::Float3(m_volume.getResolution()) * (-0.5f) * m_volume.getVoxelSize();
	box.max = 0.f - box.min;
	m_volume.setBound(box);

	size_t cnt = 1;
	for (size_t i = 0; i < m_hdf5dims.size(); i++)
		cnt *= m_hdf5dims[i];
	int nVoxels = cnt / m_hdf5dims[0];
	memcpy(m_volume.data(), m_hdf5volume.data() + id * nVoxels, nVoxels * sizeof(float));
	int label = m_hdf5label[id];
	printf("id: %d, label: %d\n", id, label);

	//// a hack, reverse volume data for visualization
	//for (int i = 0; i < nVoxels; i++)
	//{
	//	if (m_volume.data()[i] == 0)
	//		m_volume.data()[i] = 0.5;
	//	else
	//		m_volume.data()[i] = -0.5;
	//}
}
