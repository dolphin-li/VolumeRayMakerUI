#include "global_data_holder.h"
#include <fstream>
#include "hdf5.h"
#include "hdf5_hl.h"
#include "embree\RayMeshSampling.h"

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
			bool occupy = (A[apos] > 0.f);
			data |= (occupy << k);
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
			bool occupy = (data >> k) & 0x1;
			if (occupy)
				A[apos] = 1.f;
			else
				A[apos] = 0.f;
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
	global_param::mesh2vol_export_bool_occupy = true;
	global_param::mesh2vol_export_float_normal = true;
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

void GlobalDataHolder::applyView(const ObjMesh& A, ObjMesh& B, int viewId)const
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
}

void GlobalDataHolder::mesh2volume(mpu::VolumeData& volume,
	mpu::VolumeData volumeNormals[3],
	const ObjMesh& mesh, ObjMesh& pointClound)
{
	// calculate volume bounding box 
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
	for (int k = 0; k < 3; k++)
	{
		volumeNormals[k].resize(res, vsz);
		volumeNormals[k].setBound(box);
	}

	// generate mesh kdtree 
	RayMeshSampling sampler;
	ldp::Float3 sbox[2] = { box.min, box.max };
	sampler.init(sbox);
	std::vector<ldp::Int3> tris;
	for (const auto& f : mesh.face_list)
		for (int k = 0; k < f.vertex_count - 2; k++)
			tris.push_back(ldp::Int3(f.vertex_index[0], f.vertex_index[k+1], f.vertex_index[k+2]));
	ldp::tic();
	sampler.sample(mesh.vertex_list.size(), mesh.vertex_list.data(),
		tris.size(), tris.data(), pointClound.vertex_list, pointClound.vertex_normal_list);
	ldp::toc();
	pointClound.updateBoundingBox();

	// put point clound into volume voxels
	volume.fill(0.f);
	for (int k = 0; k < 3; k++)
		volumeNormals[k].fill(0.f);
	for (size_t iPoint = 0; iPoint < pointClound.vertex_list.size(); iPoint++)
	{
		const ldp::Float3& v = pointClound.vertex_list[iPoint];
		const ldp::Float3& vn = pointClound.vertex_normal_list[iPoint];
		const ldp::Float3 idxf = volume.getVolumeIndexFromWorldPos(v);
		const ldp::Int3 idx(std::lroundf(idxf[0]), std::lroundf(idxf[1]), std::lroundf(idxf[2]));
		volume.data_XYZ(idx)[0] = 1.f;
		for (int k = 0; k < 3; k++)
			volumeNormals[k].data_XYZ(idx)[0] = vn[k];
	} // end for iPoint
	//pointClound.clear();
	for (int z = 0; z < volume.getResolution()[2]; z++)
	for (int y = 0; y < volume.getResolution()[1]; y++)
	for (int x = 0; x < volume.getResolution()[0]; x++)
	{
		ldp::Int3 idx(x, y, z);
		if (volume.data_XYZ(idx)[0] == 0.f)
			continue;

		ldp::Float3 vn;
		for (int k = 0; k < 3; k++)
			vn[k] = volumeNormals[k].data_XYZ(idx)[0];
		if (vn.length() != 0.f)
			vn.normalizeLocal();
		for (int k = 0; k < 3; k++)
			volumeNormals[k].data_XYZ(idx)[0] = vn[k];
		ldp::Float3 v = volume.getWorldPosFromVolumeIndex(idx);
		//pointClound.vertex_list.push_back(v);
		//pointClound.vertex_normal_list.push_back(vn);
	}
}

void GlobalDataHolder::showParams()
{
	m_timeStamp.Stamp("MESH_2_VOLUME_PADDING: %d", global_param::MESH_2_VOLUME_PADDING);
	m_timeStamp.Stamp("volume_res: %d %d %d", global_param::volume_res[0], global_param::volume_res[1],
		global_param::volume_res[2]);
	m_timeStamp.Stamp("hdf5numMeshes: %d", global_param::hdf5numMeshes);
	m_timeStamp.Stamp("mesh2vol_export_bool_occupy: %d", global_param::mesh2vol_export_bool_occupy);
	m_timeStamp.Stamp("mesh2vol_export_float_normal: %d", global_param::mesh2vol_export_float_normal);
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
		if (lineLabel == "param_volRes")
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
		else if (lineLabel == "mesh2vol_export_bool_occupy")
			global_param::mesh2vol_export_bool_occupy = atoi(lineBuffer.c_str());
		else if (lineLabel == "mesh2vol_export_float_normal")
			global_param::mesh2vol_export_float_normal = atoi(lineBuffer.c_str());
		else if (lineLabel == "mesh2vol_inputDir")
			m_mesh2vol_inputDir = lineBuffer;
		else if (lineLabel == "mesh2vol_outputDir")
			m_mesh2vol_outputDir = lineBuffer;
		else if (lineLabel == "mesh2vol_meshExt")
			m_mesh2vol_meshExt = lineBuffer;
	}
	stm.close();
	m_timeStamp.Stamp("config file parsed: ");
	showParams();
	m_timeStamp.flushLog();

	batch_mesh2vol();
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
	if (global_param::mesh2vol_export_bool_occupy && (nVoxelsPerView % 8))
		throw std::exception("when outputing binary volume, the resolution and views must be even!");
	std::vector<float> tmpVolumeBuffer;
	std::vector<char> tmpVolumeBufferBool;
	if (global_param::mesh2vol_export_bool_occupy)
		tmpVolumeBufferBool.resize(global_param::hdf5numMeshes*nVoxelsPerView*nViewPerMesh / 8);
	if (global_param::mesh2vol_export_float_normal)
		tmpVolumeBuffer.resize(global_param::hdf5numMeshes*nViewPerMesh*3*nVoxelsPerView);
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
		std::string hname_bool = fullfile(m_mesh2vol_outputDir, trainTestFlag 
			+ std::to_string(batch_start) + ".hdf5_bool");
		std::string hname_normal = fullfile(m_mesh2vol_outputDir, trainTestFlag
			+ std::to_string(batch_start) + ".hdf5_normal");
		if ((ldp::file_exist(hname_bool.c_str()) && !global_param::mesh2vol_export_float_normal)
			|| (ldp::file_exist(hname_normal.c_str()) && !global_param::mesh2vol_export_bool_occupy))
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
			ObjMesh mesh, pointCloud;
			mpu::VolumeData volume, volumeNormals[3];
			applyView(meshNoR, mesh, iView);
			mesh2volume(volume, volumeNormals, mesh, pointCloud);

			// convert to occupy volume
			if (global_param::mesh2vol_export_bool_occupy)
			{
				std::vector<char> tmpBoolBuffer;
				convert_float_to_bool(nVoxelsPerView, volume.data(), tmpBoolBuffer);
				char* bufferPtr = tmpVolumeBufferBool.data() + (batch_offset*nViewPerMesh + iView)*nVoxelsPerView/8;
				for (size_t i = 0; i < tmpBoolBuffer.size(); i++)
					bufferPtr[i] = tmpBoolBuffer[i];
			}// end conversion
			if (global_param::mesh2vol_export_float_normal)
			{
				float* bufferPtr = tmpVolumeBuffer.data() + (batch_offset*nViewPerMesh + iView)*3*nVoxelsPerView;
				for (int k = 0; k < 3; k++)
				{
					for (int i = 0; i < nVoxelsPerView; i++)
						bufferPtr[i] = volumeNormals[k].data()[i];
					bufferPtr += nVoxelsPerView;
				}
			}
			tmpLabelBuffer[batch_offset*nViewPerMesh + iView] = labelIter->second;
		}// end for iView
		m_timeStamp.Stamp("mesh(%s) %d <- %d, label=%s, %d processed", meshName.c_str(),
			iMeshAry, meshId, labelIter->first.c_str(), labelIter->second);
		m_timeStamp.flushLog();

		// create H5File
		if (batch_offset + 1 == global_param::hdf5numMeshes || iMeshAry + 1 == nTrainMeshes
			|| iMeshAry + 1 == nTrainTestMeshes)
		{
			herr_t err = 0;
			int meshesThisBatch = batch_offset + 1;
			if (global_param::mesh2vol_export_bool_occupy)
			{
				hid_t file_id = H5Fcreate(hname_bool.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
				if (file_id < 0)
					throw std::exception("H5Fcreate failed!");
				m_timeStamp.Stamp("hdf5 created: %s", hname_bool.c_str());
				m_timeStamp.flushLog();
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
				err = H5Fclose(file_id);
				if (err)
					throw std::exception("H5Fclose failed!");
				m_timeStamp.Stamp("hdf5 closed");
				m_timeStamp.flushLog();
			} // end if export bool hdf5
			if (global_param::mesh2vol_export_float_normal)
			{
				hid_t file_id = H5Fcreate(hname_normal.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
				if (file_id < 0)
					throw std::exception("H5Fcreate failed!");
				m_timeStamp.Stamp("hdf5 created: %s", hname_normal.c_str());
				m_timeStamp.flushLog();
				hsize_t dims[5] = { meshesThisBatch*nViewPerMesh, 3, res[0], res[1], res[2] };
				err = H5LTmake_dataset_float(file_id, "volume", 5, dims, tmpVolumeBuffer.data());
				if (err)
					throw std::exception("H5LTmake_dataset_float volume failed!");
				hsize_t dims2[2] = { meshesThisBatch*nViewPerMesh, 1 };
				err = H5LTmake_dataset_float(file_id, "label", 2, dims2, tmpLabelBuffer.data());
				if (err)
					throw std::exception("H5LTmake_dataset_float  label failed!");
				err = H5Fclose(file_id);
				if (err)
					throw std::exception("H5Fclose failed!");
				m_timeStamp.Stamp("hdf5 closed");
				m_timeStamp.flushLog();
			} // end if export normal hdf5
		} // end if create hdf5 files
	}// end for iMesh
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
	for (int k = 0; k < 3; k++)
	{
		m_volumeNormal[k].resize(m_volume.getResolution());
		m_volumeNormal[k].setBound(box);
	}

	size_t cnt = 1;
	for (size_t i = 0; i < m_hdf5dims.size(); i++)
		cnt *= m_hdf5dims[i];
	if (m_hdf5dims[1] == 1)
	{
		int nVoxels = cnt / m_hdf5dims[0];
		memcpy(m_volume.data(), m_hdf5volume.data() + id * nVoxels, nVoxels * sizeof(float));
		m_pointCloud.clear();
		for (int z = 0; z < m_volume.getResolution()[2]; z++)
			for (int y = 0; y < m_volume.getResolution()[1]; y++)
				for (int x = 0; x < m_volume.getResolution()[0]; x++)
				{
					ldp::Int3 idx(x, y, z);
					if (m_volume.data_XYZ(idx)[0] == 0.f)
						continue;
					ldp::Float3 v = m_volume.getWorldPosFromVolumeIndex(idx);
					m_pointCloud.vertex_list.push_back(v);
					m_pointCloud.vertex_normal_list.push_back(1.f);
				}
	} // end if depth data
	else if(m_hdf5dims[1] == 3)
	{
		int nVoxels = cnt / m_hdf5dims[0] / m_hdf5dims[1];
		for(int k=0; k<3; k++)
			memcpy(m_volumeNormal[k].data(), m_hdf5volume.data() + (id * 3 + k) * nVoxels, nVoxels * sizeof(float));
		m_pointCloud.clear();
		m_volume.fill(0.f);
		for (int z = 0; z < m_volume.getResolution()[2]; z++)
		for (int y = 0; y < m_volume.getResolution()[1]; y++)
		for (int x = 0; x < m_volume.getResolution()[0]; x++)
		{
			ldp::Int3 idx(x, y, z);
			ldp::Float3 vn;
			for (int k = 0; k < 3; k++)
				vn[k] = m_volumeNormal[k].data_XYZ(idx)[0];
			if (vn.length() == 0.f)
				continue;
			m_volume.data_XYZ(idx)[0] = 1.f;
			ldp::Float3 v = m_volume.getWorldPosFromVolumeIndex(idx);
			m_pointCloud.vertex_list.push_back(v);
			m_pointCloud.vertex_normal_list.push_back(vn);
		}
	} // end if normal data
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
