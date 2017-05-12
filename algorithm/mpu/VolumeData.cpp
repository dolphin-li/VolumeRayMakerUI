#include "VolumeData.h"
#include <iostream>
#include <fstream>
#include "ObjMesh.h"
#include <queue>
#include <stack>
namespace mpu
{
	void draw_or_split(VolumeData& Volume, const ldp::Float3& A,
		const ldp::Float3& B, const ldp::Float3& C);

	VolumeData::VolumeData()
	{
		clear();
	}

	VolumeData::~VolumeData()
	{
	}

	std::vector<std::string> VolumeData::getSupportedVolumeExts()
	{
		static const char* s_supported_volume_type[] =
		{
			".dvol",
			".kinect_volume",
			".hfdvol",
			".cvol"
		};
		static std::vector<std::string> exts(s_supported_volume_type, s_supported_volume_type + 4);
		return exts;
	}

	bool VolumeData::isSupportedVolumeExt(const char* filename)
	{
		const static int ns = 4;
		static const char* s_supported_volume_type[ns] =
		{
			".dvol",
			".kinect_volume",
			".hfdvol",
			".cvol"
		};

		std::string ext, dumy;
		fileparts(filename, dumy, dumy, ext);
		
		for (int i = 0; i < ns; i++)
		if (ext == s_supported_volume_type[i])
			return true;
		return false;
	}

	void VolumeData::clear()
	{
		m_data.clear();
		m_voxelSize = 0.f;
		m_resolution = 0;
		m_volumeType = VolumeTypeMpu;
	}

	void VolumeData::resize(ldp::UShort3 p, float voxelSize)
	{
		VolumeTemplate<float>::resize(p);
		m_voxelSize = voxelSize;
	}

	void VolumeData::save(const char* filename)const
	{
		std::string path, name, ext;
		fileparts(filename, path, name, ext);
		
		if (!isSupportedVolumeExt(ext.c_str()))
			throw std::exception(("not supported file extension: " + ext).c_str());

		FILE* pFile = fopen(filename, "wb");
		if (!pFile)
			throw std::exception((std::string("io error: saving file failed: ") + filename).c_str());

		if (ext == ".dvol")
			saveFloat(pFile);
		else if (ext == ".cvol")
			saveCompactHalf(pFile);
		else if (ext == ".hfdvol")
			saveHalf(pFile);
		else if (ext == ".kinect_volume")
			saveKinectFusionData_SDK(pFile);
		else if (ext == ".kvol")
			throw std::exception(("not supported saving format: " + ext).c_str());

		fclose(pFile);
	}

	void VolumeData::save(FILE* pFile)const
	{
		throw std::exception("not allowed calling");
	}

	void VolumeData::load(const char* filename)
	{
		std::string path, name, ext;
		fileparts(filename, path, name, ext);

		if (!isSupportedVolumeExt(ext.c_str()))
			throw std::exception(("not supported file extension: " + ext).c_str());

		FILE* pFile = fopen(filename, "rb");
		if (!pFile)
			throw std::exception((std::string("io error: saving file failed: ") + filename).c_str());

		if (ext == ".dvol")
			loadFloat(pFile);
		else if (ext == ".cvol")
			loadCompactHalf(pFile);
		else if (ext == ".hfdvol")
			loadHalf(pFile);
		else if (ext == ".kinect_volume")
			loadKinectFusionData_SDK(pFile);
		else if (ext == ".kvol")
			loadKinectFusionData(pFile);

		fclose(pFile);
	}

	void VolumeData::load(FILE* pFile)
	{
		throw std::exception("not allowed calling");
	}

	void VolumeData::saveFloat(const char* filename)const
	{
		FILE* pFile = fopen(filename, "wb");
		if (!pFile)
			throw std::exception((std::string("io error: saving file failed: ") + filename).c_str());

		saveFloat(pFile);
	}

	void VolumeData::saveFloat(FILE* pFile)const
	{
		fwrite(&m_resolution, sizeof(m_resolution), 1, pFile);
		fwrite(&m_voxelSize, sizeof(m_voxelSize), 1, pFile);
		fwrite(&m_boundingBox, sizeof(m_boundingBox), 1, pFile);
		fwrite(data(), sizeof(float), m_resolution[0] * m_resolution[1] * m_resolution[2], pFile);
		fwrite(&m_volumeType, sizeof(m_volumeType), 1, pFile);
		//printf("save volume, type %d, size %d, %d, %d\n", m_volumeType,
		//	m_resolution[0], m_resolution[1], m_resolution[2]);
	}

	void VolumeData::loadFloat(const char* filename)
	{
		FILE* pFile = fopen(filename, "rb");
		if (!pFile)
			throw std::exception((std::string("io error: loading file failed: ") + filename).c_str());

		loadFloat(pFile);

		fclose(pFile);
	}

	void VolumeData::loadFloat(FILE* pFile)
	{
		clear();

		fread(&m_resolution, sizeof(m_resolution), 1, pFile);
		VolumeTemplate<float>::resize(m_resolution);
		fread(&m_voxelSize, sizeof(m_voxelSize), 1, pFile);
		fread(&m_boundingBox, sizeof(m_boundingBox), 1, pFile);
		fread((char*)data(), sizeof(float), m_resolution[0] * m_resolution[1] * m_resolution[2], pFile);

		if (fread(&m_volumeType, sizeof(m_volumeType), 1, pFile) != 1)
		{
			printf("warning: no volume type provided, using the default: MPU\n");
			m_volumeType = VolumeTypeMpu;
		}

		printf("load volume, type %d, size %d, %d, %d, voxel size: %f\n", m_volumeType,
			m_resolution[0], m_resolution[1], m_resolution[2], m_voxelSize);
		printf("bounding box: %f %f %f, %f %f %f\n", m_boundingBox.min[0],
			m_boundingBox.min[1], m_boundingBox.min[2], m_boundingBox.max[0], 
			m_boundingBox.max[1], m_boundingBox.max[2]);
	}

	void VolumeData::saveHalf(const char* filename)const
	{
		FILE* pFile = fopen(filename, "wb");
		if (!pFile)
			throw std::exception((std::string("io error: saving file failed: ") + filename).c_str());

		saveHalf(pFile);

		fclose(pFile);
	}

	void VolumeData::saveHalf(FILE* pFile)const
	{
		fwrite(&m_resolution, sizeof(m_resolution), 1, pFile);
		fwrite(&m_voxelSize, sizeof(m_voxelSize), 1, pFile);
		fwrite(&m_boundingBox, sizeof(m_boundingBox), 1, pFile);
		std::vector<half_float::half> half_data(m_data.size());
		for (size_t i = 0; i < half_data.size(); i++)
			half_data[i] = m_data[i];
		fwrite(half_data.data(), sizeof(half_float::half), half_data.size(), pFile);
		fwrite(&m_volumeType, sizeof(m_volumeType), 1, pFile);
		//printf("save half volume, type %d, size %d, %d, %d\n", m_volumeType,
		//	m_resolution[0], m_resolution[1], m_resolution[2]);
	}

	void VolumeData::loadHalf(const char* filename)
	{
		FILE* pFile = fopen(filename, "rb");
		if (!pFile)
			throw std::exception((std::string("io error: loading file failed: ") + filename).c_str());

		loadHalf(pFile);

		fclose(pFile);
	}

	void VolumeData::loadHalf(FILE* pFile)
	{
		clear();

		fread(&m_resolution, sizeof(m_resolution), 1, pFile);
		VolumeTemplate<float>::resize(m_resolution);
		fread(&m_voxelSize, sizeof(m_voxelSize), 1, pFile);
		fread(&m_boundingBox, sizeof(m_boundingBox), 1, pFile);

		std::vector<half_float::half> half_data(m_data.size());
		int num = fread((char*)half_data.data(), sizeof(half_float::half), half_data.size(), pFile);
		if (num != half_data.size())
			throw std::exception(("data corruption: " + std::to_string(num) + " vs. "
			+ std::to_string(half_data.size())).c_str());
		for (size_t i = 0; i < half_data.size(); i++)
			m_data[i] = half_data[i];

		if (fread(&m_volumeType, sizeof(m_volumeType), 1, pFile) != 1)
		{
			printf("warning: no volume type provided, using the default: MPU\n");
			m_volumeType = VolumeTypeMpu;
		}

		printf("load half volume, type %d, size %d, %d, %d, voxel size: %f\n", m_volumeType,
			m_resolution[0], m_resolution[1], m_resolution[2], m_voxelSize);
		printf("bounding box: %f %f %f, %f %f %f\n", m_boundingBox.min[0],
			m_boundingBox.min[1], m_boundingBox.min[2], m_boundingBox.max[0],
			m_boundingBox.max[1], m_boundingBox.max[2]);
	}

	void VolumeData::saveCompactHalf(const char* filename)const
	{
		if (m_data.size() > UINT_MAX)
			throw std::exception("too large volume to be saved compactly!");

		FILE* pFile = fopen(filename, "wb");
		if (!pFile)
			throw std::exception((std::string("io error: saving file failed: ") + filename).c_str());

		saveCompactHalf(pFile);

		fclose(pFile);
	}

	void VolumeData::saveCompactHalf(FILE* pFile)const
	{
		fwrite(&m_resolution, sizeof(m_resolution), 1, pFile);
		fwrite(&m_voxelSize, sizeof(m_voxelSize), 1, pFile);
		fwrite(&m_boundingBox, sizeof(m_boundingBox), 1, pFile);

		float maxVal = -1e10f;
		for (int i = 0; i < m_data.size(); i++)
			maxVal = std::max(maxVal, m_data[i]);

		std::vector<CompactHalf> cdata;

		for (size_t i = 0; i < m_data.size(); i++)
		{
			float v = m_data[i];
			if (v < maxVal - m_voxelSize*0.1f)
			{
				CompactHalf ch;
				ch.idx = i;
				ch.val = v;
				cdata.push_back(ch);
			}
		}
		fwrite(&maxVal, sizeof(float), 1, pFile);
		int n = cdata.size();
		fwrite(&n, sizeof(int), 1, pFile);
		fwrite(cdata.data(), sizeof(CompactHalf), cdata.size(), pFile);
		fwrite(&m_volumeType, sizeof(m_volumeType), 1, pFile);
	}

	void VolumeData::loadCompactHalf(const char* filename)
	{
		FILE* pFile = fopen(filename, "rb");
		if (!pFile)
			throw std::exception((std::string("io error: loading file failed: ") + filename).c_str());

		loadCompactHalf(pFile);

		fclose(pFile);
	}

	void VolumeData::loadCompactHalf(FILE* pFile)
	{
		clear();

		fread(&m_resolution, sizeof(m_resolution), 1, pFile);
		VolumeTemplate<float>::resize(m_resolution);
		fread(&m_voxelSize, sizeof(m_voxelSize), 1, pFile);
		fread(&m_boundingBox, sizeof(m_boundingBox), 1, pFile);

		float maxVal = 0.f;
		fread(&maxVal, sizeof(float), 1, pFile);

		fill(maxVal);

		int n = 0;
		fread(&n, sizeof(int), 1, pFile);
		std::vector<CompactHalf> cdata(n);
		int num = fread((char*)cdata.data(), sizeof(CompactHalf), cdata.size(), pFile);
		if (num != cdata.size())
			throw std::exception(("data corruption: " + std::to_string(num) + " vs. "
			+ std::to_string(cdata.size())).c_str());

		for (size_t i = 0; i < cdata.size(); i++)
			m_data[cdata[i].idx] = cdata[i].val;

		if (fread(&m_volumeType, sizeof(m_volumeType), 1, pFile) != 1)
		{
			printf("warning: no volume type provided, using the default: MPU\n");
			m_volumeType = VolumeTypeMpu;
		}

		printf("load compact half volume, type %d, size %d, %d, %d, voxel size: %f\n", m_volumeType,
			m_resolution[0], m_resolution[1], m_resolution[2], m_voxelSize);
		printf("bounding box: %f %f %f, %f %f %f\n", m_boundingBox.min[0],
			m_boundingBox.min[1], m_boundingBox.min[2], m_boundingBox.max[0],
			m_boundingBox.max[1], m_boundingBox.max[2]);
	}

	void VolumeData::loadKinectFusionData(const char* filename, int weightThreshold)
	{
		FILE* pFile = fopen(filename, "rb");
		if (!pFile)
			throw std::exception((std::string("io error: loading file failed: ") + filename).c_str());

		loadKinectFusionData(pFile, weightThreshold);

		fclose(pFile);
	}
	
	void VolumeData::loadKinectFusionData(FILE* pFile, int weightThreshold)
	{
		clear();

		float voxelPerMeter;
		int nX, nY, nZ;
		fread(&m_voxelSize, sizeof(float), 1, pFile);
		fread(&nX, sizeof(int), 1, pFile);
		fread(&nY, sizeof(int), 1, pFile);
		fread(&nZ, sizeof(int), 1, pFile);
		std::vector<ldp::Short2> rawData;
		rawData.resize(nX*nY*nZ);

		if (nX*nY*nZ != fread(rawData.data(), sizeof(ldp::Short2), rawData.size(), pFile))
			throw std::exception("file corrupted");
		printf("kinect volume loaded: %d %d %d, %f\n", nX, nY, nZ, m_voxelSize);

		m_volumeType = VolumeTypeKinect;
		m_resolution = ldp::UShort3(nX, nY, nZ);
		VolumeTemplate<float>::resize(m_resolution);
		m_boundingBox.min = -m_voxelSize * ldp::Float3(m_resolution) / 2.f;
		m_boundingBox.max = m_voxelSize * ldp::Float3(m_resolution) / 2.f;

		const ldp::Short2* rawPtr = rawData.data();
		float* vPtr = data();
		for (int z = 0; z < m_resolution[2]; z++)
		{
			for (int y = 0; y < m_resolution[1]; y++)
			{
				for (int x = 0; x < m_resolution[0]; x++)
				{
					ldp::Short2 rawVal = *rawPtr++;
					float& vVar = *vPtr++;
					float dist = float(rawVal[0]) / float(MAXSHORT);

					if (rawVal[1] <= weightThreshold)
					{
						vVar = HOLE_CELL_VALUE;
						continue;
					}

					vVar = dist;
				}//x
			}//y
		}//z
	}

	void VolumeData::loadKinectFusionData_SDK(const char* filename, int weightThreshold)
	{
		FILE* pFile = fopen(filename, "rb");
		if (!pFile)
			throw std::exception((std::string("io error: loading file failed: ") + filename).c_str());

		loadKinectFusionData_SDK(pFile, weightThreshold);

		fclose(pFile);
	}

	void VolumeData::loadKinectFusionData_SDK(FILE* pFile, int weightThreshold)
	{
		clear();

		int nX, nY, nZ;
		fread(&m_voxelSize, sizeof(float), 1, pFile);
		m_voxelSize = 1.f / m_voxelSize;
		fread(&nX, sizeof(int), 1, pFile);
		fread(&nY, sizeof(int), 1, pFile);
		fread(&nZ, sizeof(int), 1, pFile);
		std::vector<short> rawData;
		rawData.resize(nX*nY*nZ);

		if (nX*nY*nZ != fread(rawData.data(), sizeof(short), rawData.size(), pFile))
			throw std::exception("file corrupted");
		printf("kinect volume loaded: %d %d %d, %f\n", nX, nY, nZ, m_voxelSize);

		m_volumeType = VolumeTypeKinect;
		m_resolution = ldp::UShort3(nX, nY, nZ);
		VolumeTemplate<float>::resize(m_resolution);
		m_boundingBox.min = -m_voxelSize * ldp::Float3(m_resolution) / 2.f;
		m_boundingBox.max = m_voxelSize * ldp::Float3(m_resolution) / 2.f;
		m_boundingBox.max[2] = -0.35f;
		m_boundingBox.min[2] = m_boundingBox.max[2] - float(m_resolution[2]) * m_voxelSize;

		const short* rawPtr = rawData.data();
		for (int z = 0; z < m_resolution[2]; z++)
		for (int y = 0; y < m_resolution[1]; y++)
		{
			float* vPtr = data_XYZ(0, m_resolution[1]-1-y, m_resolution[2]-1-z);
			for (int x = 0; x < m_resolution[0]; x++)
			{
				short rawVal = *rawPtr++;

				float& vVar = *vPtr++;
				int wRaw = (rawVal & 0xff);
				int depthRaw = -char((rawVal & 0xff00) >> 8);

				if (wRaw == weightThreshold)
				{
					vVar = HOLE_CELL_VALUE;
					continue;
				}

				vVar = std::min(1.f, std::max(-1.f, float(depthRaw) / float(0x7f)));
			}// x
		}//y,z
	}

	void VolumeData::saveKinectFusionData_SDK(const char* filename)const
	{
		FILE* pFile = fopen(filename, "wb");
		if (!pFile)
			throw std::exception((std::string("io error: creating file failed: ") + filename).c_str());

		saveKinectFusionData_SDK(pFile);

		fclose(pFile);
	}

	void VolumeData::saveKinectFusionData_SDK(FILE* pFile)const
	{
		float voxelPerMeter = 1.f / m_voxelSize;
		fwrite(&voxelPerMeter, sizeof(float), 1, pFile);
		ldp::Int3 res = m_resolution;
		fwrite(&res[0], sizeof(int), 1, pFile);
		fwrite(&res[1], sizeof(int), 1, pFile);
		fwrite(&res[2], sizeof(int), 1, pFile);
		std::vector<short> rawData;
		rawData.resize(m_resolution[0] * m_resolution[1]*m_resolution[2]);

		short* rawPtr = rawData.data();
		for (int z = 0; z < m_resolution[2]; z++)
		for (int y = 0; y < m_resolution[1]; y++)
		{
			const float* vPtr = data_XYZ(0, m_resolution[1] - 1 - y, m_resolution[2] - 1 - z);
			for (int x = 0; x < m_resolution[0]; x++)
			{
				short& rawVal = *rawPtr++;
				float vVar = std::min(1.f, std::max(-1.f, *vPtr++));

				if (is_hole_cell_value(vVar))
				{
					rawVal = 0x8000;
				}
				else
				{
					int depthRaw = -int(vVar * 0x7f);
					rawVal = ((depthRaw << 8) & 0xff00) + 0x7f;
				}
			}// x
		}//y,z
		fwrite(rawData.data(), sizeof(short), rawData.size(), pFile);
	}

	void VolumeData::makeHole(const VolumeMask& mask)
	{
		if (mask.getResolution() != getResolution())
			throw std::exception("VolumeData::makeHole(), size mis-matched");

		const unsigned char* mask_ptr = mask.data();
		float* data_ptr = data();

		for (int z = 0; z < m_resolution[2]; z++)
		{
			int z_plane_id_begin = z * stride_Z();
			for (int y = 0; y < m_resolution[1]; y++)
			{
				int y_row_id_begin = z_plane_id_begin + y*stride_Y();
				for (int x = 0; x < m_resolution[0]; x++)
				{
					int id = y_row_id_begin + x;
					if (mask_ptr[id])
						data_ptr[id] = HOLE_CELL_VALUE;
				}// end for x
			}// end for y
		}// end for z
	}

	void VolumeData::clipValues(float minVal, float maxVal)
	{
		for (int i = 0; i < m_data.size(); i++)
			m_data[i] = std::max(minVal, std::min(maxVal, m_data[i]));
	}

	float VolumeData::getValueScale()const
	{
		if (m_data.size() == 0)
			return 0.f;
		float minVal = 1e10;
		float maxVal = -1e10;
		for (int i = 0; i < m_data.size(); i++)
		{
			float v = m_data[i];
			if (is_hole_cell_value(v))
				continue;
			if (fabs(v) > INVALID_CELL_VALUE * 0.1f)
				continue;
			minVal = std::min(minVal, v);
			maxVal = std::max(maxVal, v);
		}
		return abs(maxVal - minVal);
	}

	void VolumeData::subVolumeTo(VolumeData& rhs, ldp::Int3 begin, ldp::Int3 end, bool updateBound)const
	{
		for (int k = 0; k < 3; k++)
		{
			begin[k] = std::max(begin[k], 0);
			end[k] = std::min(end[k], (int)m_resolution[k]);
			if (end[k] <= begin[k])
			{
				rhs.clear();
				return;
			}
		}
		VolumeTemplate<float>::subVolumeTo(rhs, begin, end);

		if (updateBound)
		{
			rhs.m_voxelSize = m_voxelSize;
			rhs.m_boundingBox.min = m_boundingBox.min + ldp::Float3(begin) * m_voxelSize;
			rhs.m_boundingBox.max = m_boundingBox.min + ldp::Float3(end) * m_voxelSize;
		}
		rhs.m_volumeType = m_volumeType;
	}

	void VolumeData::mirrorExtendTo(VolumeData& rhs, int radius)const
	{
		VolumeTemplate<float>::mirrorExtendTo(rhs, radius);

		rhs.m_voxelSize = m_voxelSize;
		rhs.m_boundingBox.min -= radius * m_voxelSize;
		rhs.m_boundingBox.max += radius * m_voxelSize;
		rhs.m_volumeType = m_volumeType;
	}

	static void draw_or_split(VolumeTemplate<int>& Volume,
		const ldp::Float3& A, const ldp::Float3& B, const ldp::Float3& C, int val)
	{
		ldp::Int3 res = Volume.getResolution();
		bool checkA, checkB, checkC;
		bool check1, check2, check3, check4, check5, check6;

		float dist1, dist2, dist3, maxdist;
		ldp::Float3 D;
		checkA = (A[0]<0) || (A[1]<0) || (A[2]<0) || (A[0]>(res[0] - 1)) 
			|| (A[1]>(res[1] - 1)) || (A[2]>(res[2] - 1));
		checkB = (B[0]<0) || (B[1]<0) || (B[2]<0) || (B[0]>(res[0] - 1))
			|| (B[1]>(res[1] - 1)) || (B[2]>(res[2] - 1));
		checkC = (C[0]<0) || (C[1]<0) || (C[2]<0) || (C[0]>(res[0] - 1))
			|| (C[1]>(res[1] - 1)) || (C[2]>(res[2] - 1));

		check1 = (A[0]<0) && (B[0]<0) && (C[0]<0);
		check2 = (A[1]<0) && (B[1]<0) && (C[1]<0);
		check3 = (A[2]<0) && (B[2]<0) && (C[2]<0);
		check4 = (A[0]>(res[0] - 1)) && (B[0]>(res[0] - 1)) && (C[0]>(res[0] - 1));
		check5 = (A[1]>(res[1] - 1)) && (B[1]>(res[1] - 1)) && (C[1]>(res[1] - 1));
		check6 = (A[2]>(res[2] - 1)) && (B[2]>(res[2] - 1)) && (C[2]>(res[2] - 1));

		/* Return if all vertices outside, on the same side */
		if (check1 || check2 || check3 || check4 || check5 || check6)
			return;

		dist1 = A.sqrDist(B);
		dist2 = C.sqrDist(B);
		dist3 = A.sqrDist(C);
		if (dist1>dist2)
		{
			if (dist1>dist3)
			{
				maxdist = dist1;
				if (maxdist>0.5)
				{
					D = (A + B)*0.5f;
					draw_or_split(Volume, D, B, C, val);
					draw_or_split(Volume, A, D, C, val);
				}
			}
			else
			{
				maxdist = dist3;
				if (maxdist>0.5)
				{
					D = (A + C)*0.5f;
					draw_or_split(Volume, D, B, C, val);
					draw_or_split(Volume, A, B, D, val);
				}

			}
		}
		else
		{
			if (dist2>dist3)
			{
				maxdist = dist2;
				D = (C + B)*0.5f;
				if (maxdist>0.5)
				{
					draw_or_split(Volume, A, D, C, val);
					draw_or_split(Volume, A, B, D, val);
				}
			}
			else
			{
				maxdist = dist3;
				if (maxdist>0.5)
				{
					D = (A + C)*0.5f;
					draw_or_split(Volume, D, B, C, val);
					draw_or_split(Volume, A, B, D, val);
				}

			}
		}

		if (checkA == false)
			Volume(A + 0.5f) = val;
		if (checkB == false)
			Volume(B + 0.5f) = val;
		if (checkC == false)
			Volume(C + 0.5f) = val;
	}

	static void flood_fill_replace_A_to_B(VolumeMask& mask, const ldp::Int3& seed, int A, int B)
	{
		ldp::Int3 res = mask.getResolution();
		std::stack<ldp::UShort3> Q;

		Q.push(seed);

		while (!Q.empty())
		{
			ldp::UShort3 p = Q.top();
			Q.pop();

			int l=p[0], r=p[0];
			while (l > 0) 
			{
				if (mask(l - 1, p[1], p[2]) != A)
					break;
				l--;
			}
			while (r < res[0]-1)
			{
				if (mask(r + 1, p[1], p[2]) != A)
					break;
				r++;
			}

			bool span_y0 = false, span_y1 = false, span_z0 = false, span_z1 = false;
			for (int k = l; k <= r; k++)
			{
				mask(k, p[1], p[2]) = B;

				if (p[1] > 0)
				{
					if (!span_y0 && mask(k, p[1] - 1, p[2]) == A)
					{
						Q.push(ldp::UShort3(k, p[1] - 1, p[2]));
						span_y0 = true;
					}
					else if (span_y0 &&  mask(k, p[1] - 1, p[2]) != A)
						span_y0 = false;
				}
				if (p[1] < res[1] - 1)
				{
					if (!span_y1 && mask(k, p[1] + 1, p[2]) == A)
					{
						Q.push(ldp::UShort3(k, p[1] + 1, p[2]));
						span_y1 = true;
					}
					else if (span_y1 && mask(k, p[1] + 1, p[2]) != A)
						span_y1 = false;
				}

				if (p[2] > 0)
				{
					if (!span_z0 && mask(k, p[1], p[2] - 1) == A)
					{
						Q.push(ldp::UShort3(k, p[1], p[2] - 1));
						span_z0 = true;
					}
					else if (span_z0 && mask(k, p[1], p[2] - 1) != A)
						span_z0 = false;
				}
				if (p[2] < res[2] - 1)
				{
					if (!span_z1 && mask(k, p[1], p[2] + 1) == A)
					{
						Q.push(ldp::UShort3(k, p[1], p[2] + 1));
						span_z1 = true;
					}
					else if (span_z1 && mask(k, p[1], p[2] + 1) != A)
						span_z1 = false;
				}
			}
		}// end while Q not empty
	}

	void VolumeData::binaryFromObjMesh(const ObjMesh& mesh, ldp::Int3 res, 
		float voxelSize, kdtree::AABB bound)
	{
		resize(res, voxelSize);
		setBound(bound);
		fill(0);

		for (int fid = 0; fid < mesh.face_list.size(); fid++)
		{
			const ObjMesh::obj_face& f = mesh.face_list[fid];
			for (int k = 0; k < f.vertex_count - 2; k++)
			{
				ldp::Float3 v[3];
				v[0] = mesh.vertex_list[f.vertex_index[0]];
				v[1] = mesh.vertex_list[f.vertex_index[k]];
				v[2] = mesh.vertex_list[f.vertex_index[k+1]];
				v[0] = getVolumeIndexFromWorldPos(v[0]);
				v[1] = getVolumeIndexFromWorldPos(v[1]);
				v[2] = getVolumeIndexFromWorldPos(v[2]);
				draw_or_split(*this, v[0], v[1], v[2]);
			}// k
		}// fid	
	}

	//==========================================================================
	/// bwdist: the same with matlab's
	//		using the linear-time Euclidean distance transform method

	////////// Functions F and Sep for the SDT labelling
	inline float bwdist_sqr(float u)
	{
		return u*u;
	}
	inline int bwdist_F(float u, float i, float gi2)
	{
		return (u - i)*(u - i) + gi2;
	}
	inline int bwdist_Sep(float i, float u, float gi2, float gu2)
	{
		return (u*u - i*i + gu2 - gi2) / (2 * (u - i));
	}
	/////////

	void bwdist(const VolumeMask& mask, VolumeData& distMap)
	{
		VolumeData tmpXVolume, tmpXYVolume;
		const ldp::Int3 resolution = mask.getResolution();
		const int inf = resolution[0] + resolution[1] + resolution[2];

		bool distMapPreReady = (distMap.getResolution() == mask.getResolution());

		// phase x-----------------------------------------------------
		tmpXVolume.resize(mask.getResolution());

#pragma omp parallel for
		for (int z = 0; z < resolution[2]; z++)
		for (int y = 0; y < resolution[1]; y++)
		{
			if (mask(0, y, z) == 0)
				tmpXVolume(0, y, z) = distMapPreReady ? distMap(0, y, z) : 0;
			else
				tmpXVolume(0, y, z) = inf;

			// Forward scan
			for (int x = 1; x < resolution[0]; x++)
			{
				if (mask(x, y, z) == 0)
					tmpXVolume(x, y, z) = distMapPreReady ? distMap(0, y, z) : 0;
				else
					tmpXVolume(x, y, z) = 1 + tmpXVolume(x - 1, y, z);
			}

			//Backward scan
			for (int x = resolution[0] - 2; x >= 0; x--)
			if (tmpXVolume(x + 1, y, z) < tmpXVolume(x, y, z))
				tmpXVolume(x, y, z) = 1 + tmpXVolume(x + 1, y, z);
		}// end for y,z

		// phase y-----------------------------------------------------
		tmpXYVolume.resize(mask.getResolution());
#pragma omp parallel for
		for (int z = 0; z < resolution[2]; z++)
		{
			std::vector<float> s(resolution[1]), t(resolution[1]);
			for (int x = 0; x < resolution[0]; x++)
			{
				float q = 0, w = 0;
				s[0] = 0;
				t[0] = 0;

				//Forward Scan
				for (int u = 1; u < resolution[1]; u++)
				{
					while (q >= 0 && (bwdist_F(t[q], s[q], bwdist_sqr(tmpXVolume(x, s[q], z))) >
						bwdist_F(t[q], u, bwdist_sqr(tmpXVolume(x, u, z)))))
						q--;

					if (q < 0)
					{
						q = 0;
						s[0] = u;
					}
					else
					{
						w = 1 + bwdist_Sep(s[q], u, bwdist_sqr(tmpXVolume(x, s[q], z)),
							bwdist_sqr(tmpXVolume(x, u, z)));
						if (w < resolution[1])
						{
							q++;
							s[q] = u;
							t[q] = w;
						}
					}
				}

				//Backward Scan
				for (int u = resolution[1] - 1; u >= 0; --u)
				{
					tmpXYVolume(x, u, z) = bwdist_F(u, s[q], bwdist_sqr(tmpXVolume(x, s[q], z)));
					if (u == t[q])
						q--;
				}
			}// end for x
		}// end for z

		// phase z-----------------------------------------------------
		tmpXVolume.clear();
		distMap.resize(resolution);
		distMap.fill(0);
#pragma omp parallel for
		for (int y = 0; y < resolution[1]; y++)
		{
			std::vector<float> s(resolution[2]), t(resolution[2]);
			for (int x = 0; x < resolution[0]; x++)
			{
				float q = 0, w = 0;
				s[0] = 0;
				t[0] = 0;

				//Forward Scan
				for (int u = 1; u < resolution[2]; u++)
				{
					while (q >= 0 && (bwdist_F(t[q], s[q], tmpXYVolume(x, y, s[q])) >
						bwdist_F(t[q], u, tmpXYVolume(x, y, u))))
						q--;

					if (q < 0)
					{
						q = 0;
						s[0] = u;
					}
					else
					{
						w = 1 + bwdist_Sep(s[q], u, tmpXYVolume(x, y, s[q]), tmpXYVolume(x, y, u));
						if (w < resolution[2])
						{
							q++;
							s[q] = u;
							t[q] = w;
						}
					}
				}

				//Backward Scan
				for (int u = resolution[2] - 1; u >= 0; --u)
				{
					distMap(x, y, u) = sqrtf((float)bwdist_F(u, s[q], tmpXYVolume(x, y, s[q])));
					if (u == t[q])
						q--;
				}
			}// end for x
		}// end for y
	}

	//=========================================================================
	void draw_or_split(VolumeData& Volume, const ldp::Float3& A, 
		const ldp::Float3& B, const ldp::Float3& C)
	{
		const ldp::Int3& R = Volume.getResolution();

		/* Check if vertices outside */
		bool checkA = (A[0]<0) || (A[1]<0) || (A[2]<0) || (A[0]>(R[0] - 1)) || (A[1]>(R[1] - 1)) || (A[2]>(R[2] - 1));
		bool checkB = (B[0]<0) || (B[1]<0) || (B[2]<0) || (B[0]>(R[0] - 1)) || (B[1]>(R[1] - 1)) || (B[2]>(R[2] - 1));
		bool checkC = (C[0]<0) || (C[1]<0) || (C[2]<0) || (C[0]>(R[0] - 1)) || (C[1]>(R[1] - 1)) || (C[2]>(R[2] - 1));

		bool check1 = (A[0]<0) && (B[0]<0) && (C[0]<0);
		bool check2 = (A[1]<0) && (B[1]<0) && (C[1]<0);
		bool check3 = (A[2]<0) && (B[2]<0) && (C[2]<0);
		bool check4 = (A[0]>(R[0] - 1)) && (B[0]>(R[0] - 1)) && (C[0]>(R[0] - 1));
		bool check5 = (A[1]>(R[1] - 1)) && (B[1]>(R[1] - 1)) && (C[1]>(R[1] - 1));
		bool check6 = (A[2]>(R[2] - 1)) && (B[2]>(R[2] - 1)) && (C[2]>(R[2] - 1));

		/* Return if all vertices outside, on the same side */
		if (check1 || check2 || check3 || check4 || check5 || check6)
			return;

		float dist1 = (A - B).sqrLength();
		float dist2 = (C - B).sqrLength();
		float dist3 = (A - C).sqrLength();
		float maxdist = 0.f;
		if (dist1>dist2)
		{
			if (dist1>dist3)
			{
				maxdist = dist1;
				if (maxdist>0.5*Volume.getVoxelSize())
				{
					ldp::Float3 D = (A + B)*0.5f;
					draw_or_split(Volume, D, B, C);
					draw_or_split(Volume, A, D, C);
				}
			}
			else
			{
				maxdist = dist3;
				if (maxdist>0.5*Volume.getVoxelSize())
				{
					ldp::Float3 D = (A + C)*0.5f;
					draw_or_split(Volume, D, B, C);
					draw_or_split(Volume, A, B, D);
				}

			}
		}
		else
		{
			if (dist2>dist3)
			{
				maxdist = dist2;
				if (maxdist>0.5*Volume.getVoxelSize())
				{
					ldp::Float3 D = (B + C)*0.5f;
					draw_or_split(Volume, A, D, C);
					draw_or_split(Volume, A, B, D);
				}
			}
			else
			{
				maxdist = dist3;
				if (maxdist>0.5*Volume.getVoxelSize())
				{
					ldp::Float3 D = (A + C)*0.5f;
					draw_or_split(Volume, D, B, C);
					draw_or_split(Volume, A, B, D);
				}

			}
		}

		if (checkA == false)
			Volume(ldp::Int3(A)) = 1;
		if (checkB == false)
			Volume(ldp::Int3(B)) = 1;
		if (checkC == false)
			Volume(ldp::Int3(C)) = 1;
	}
}// namespace mpu