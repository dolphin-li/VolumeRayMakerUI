#pragma once
#include "VolumeData.h"
namespace mpu
{
	class ConvolutionPyramid
	{
	public:
		ConvolutionPyramid();
		~ConvolutionPyramid();

	public:
		// boundary interpolation
		// assume the given volume has values at the boundary and 0 inside.
		// then this method smoothly interpolate those boundary values inward
		void convolve_boundary(VolumeData& srcDst);

		// solve a poisson equation with Neumann boundary conditions
		// assume the input is the negative divergence of a given volume
		// then the output is the reconstructed volume based on the input
		void solve_poisson(VolumeData& srcDiv_dstVolume);
	public:
		//imDst = imSrc conv Kernel (Kernel is implicitly defined by kernel5x5 (h1), kernel3x3 (g)), and kernel5x5up (h2);
		static void PyramidConvolve(VolumeData& srcDst, const float* kernel5x5, 
			const float* kernel3x3, const float* kernel5x5up);

		// dst = pad(src, 5)
		static void ZeroPadding5x5(VolumeData& dst, const VolumeData& src);

		// imDst(p) = imSrc(p*2)
		static void DownSamplex2(VolumeData& imDst, const VolumeData& imSrc);

		// imDst(p*2) = imSrc(p), imDst(p*2+1) = 0;
		// NOTE: imDst should be pre-allocated
		static void VolumeUpscalex2_ZeroHalf(VolumeData& imDst, const VolumeData& imSrc);

		// A = A + B
		static void AddImage(VolumeData& A, const VolumeData& B);

		// C = unpad(A+B)
		static void AddImageUnpad5x5(VolumeData& C, const VolumeData& A, const VolumeData& B);

		/// div  = -(Gx(Gx) + Gy(Gy) + Gz(Gz));
		static void ConvolutionPyramid::ComputeDiv(VolumeData& imDiv, const VolumeData& imGx,
			const VolumeData& imGy, const VolumeData& imGz);
	};
}