#ifndef VIENNACL_LINALG_KERNELS_SCALAR_SOURCE_HPP_
#define VIENNACL_LINALG_KERNELS_SCALAR_SOURCE_HPP_
//Automatically generated file from auxiliary-directory, do not edit manually!
/** @file scalar_source.h
 *  @brief OpenCL kernel source file, generated automatically. */
namespace viennacl
{
 namespace linalg
 {
  namespace kernels
  {
const char * const scalar_align1_asbs_gpu_cpu = 
"// generic kernel for the scalar operation s1 = alpha * s2 + beta * s3, where s1, s2, s3 are not necessarily distinct GPU scalars\n"
"__kernel void asbs_gpu_cpu(\n"
"          __global float * s1,\n"
"          \n"
"          __global const float * fac2,\n"
"          unsigned int options2,  // 0: no action, 1: flip sign, 2: take inverse, 3: flip sign and take inverse\n"
"          __global const float * s2,\n"
"          \n"
"          float fac3,\n"
"          unsigned int options3,  // 0: no action, 1: flip sign, 2: take inverse, 3: flip sign and take inverse\n"
"          __global const float * s3)\n"
"{ \n"
"  float alpha = fac2[0];\n"
"  if ((options2 >> 2) > 1)\n"
"  {\n"
"    for (unsigned int i=1; i<(options2 >> 2); ++i)\n"
"      alpha += fac2[i];\n"
"  }\n"
"  if (options2 & (1 << 0))\n"
"    alpha = -alpha;\n"
"  if (options2 & (1 << 1))\n"
"    alpha = ((float)(1)) / alpha;\n"
"  float beta = fac3;\n"
"  if (options3 & (1 << 0))\n"
"    beta = -beta;\n"
"  if (options3 & (1 << 1))\n"
"    beta = ((float)(1)) / beta;\n"
"    \n"
"  *s1 = *s2 * alpha + *s3 * beta;\n"
"}\n"
; //scalar_align1_asbs_gpu_cpu

const char * const scalar_align1_asbs_s_cpu_gpu = 
"// generic kernel for the scalar operation s1 += alpha * s2 + beta * s3, where s1, s2, s3 are not necessarily distinct GPU scalars\n"
"__kernel void asbs_s_cpu_gpu(\n"
"          __global float * s1,\n"
"          \n"
"          float fac2,\n"
"          unsigned int options2,\n"
"          __global const float * s2,\n"
"          \n"
"          __global const float * fac3,\n"
"          unsigned int options3,\n"
"          __global const float * s3)\n"
"{ \n"
"  float alpha = fac2;\n"
"  if (options2 & (1 << 0))\n"
"    alpha = -alpha;\n"
"  if (options2 & (1 << 1))\n"
"    alpha = ((float)(1)) / alpha;\n"
"  float beta = fac3[0];\n"
"  if ((options3 >> 2) > 1)\n"
"  {\n"
"    for (unsigned int i=1; i<(options3 >> 2); ++i)\n"
"      beta += fac3[i];\n"
"  }\n"
"  if (options3 & (1 << 0))\n"
"    beta = -beta;\n"
"  if (options3 & (1 << 1))\n"
"    beta = ((float)(1)) / beta;\n"
"  \n"
"  *s1 += *s2 * alpha + *s3 * beta;\n"
"}\n"
; //scalar_align1_asbs_s_cpu_gpu

const char * const scalar_align1_asbs_gpu_gpu = 
"// generic kernel for the scalar operation s1 = alpha * s2 + beta * s3, where s1, s2, s3 are not necessarily distinct GPU scalars\n"
"__kernel void asbs_gpu_gpu(\n"
"          __global float * s1,\n"
"          \n"
"          __global const float * fac2,\n"
"          unsigned int options2,  // 0: no action, 1: flip sign, 2: take inverse, 3: flip sign and take inverse\n"
"          __global const float * s2,\n"
"          \n"
"          __global const float * fac3,\n"
"          unsigned int options3,  // 0: no action, 1: flip sign, 2: take inverse, 3: flip sign and take inverse\n"
"          __global const float * s3)\n"
"{ \n"
"  float alpha = fac2[0];\n"
"  if ((options2 >> 2) > 1)\n"
"  {\n"
"    for (unsigned int i=1; i<(options2 >> 2); ++i)\n"
"      alpha += fac2[i];\n"
"  }\n"
"  if (options2 & (1 << 0))\n"
"    alpha = -alpha;\n"
"  if (options2 & (1 << 1))\n"
"    alpha = ((float)(1)) / alpha;\n"
"  float beta = fac3[0];\n"
"  if ((options3 >> 2) > 1)\n"
"  {\n"
"    for (unsigned int i=1; i<(options3 >> 2); ++i)\n"
"      beta += fac3[i];\n"
"  }\n"
"  if (options3 & (1 << 0))\n"
"    beta = -beta;\n"
"  if (options3 & (1 << 1))\n"
"    beta = ((float)(1)) / beta;\n"
"  \n"
"  *s1 = *s2 * alpha + *s3 * beta;\n"
"}\n"
; //scalar_align1_asbs_gpu_gpu

const char * const scalar_align1_asbs_s_cpu_cpu = 
"// generic kernel for the scalar operation s1 += alpha * s2 + beta * s3 + gamma * v4, where v1, v2, v3 are not necessarily distinct vectors\n"
"__kernel void asbs_s_cpu_cpu(\n"
"          __global float * s1,\n"
"          \n"
"          float fac2,\n"
"          unsigned int options2,  // 0: no action, 1: flip sign, 2: take inverse, 3: flip sign and take inverse\n"
"          __global const float * s2,\n"
"          \n"
"          float fac3,\n"
"          unsigned int options3,  // 0: no action, 1: flip sign, 2: take inverse, 3: flip sign and take inverse\n"
"          __global const float * s3)\n"
"{ \n"
"  float alpha = fac2;\n"
"  if (options2 & (1 << 0))\n"
"    alpha = -alpha;\n"
"  if (options2 & (1 << 1))\n"
"    alpha = ((float)(1)) / alpha;\n"
"  float beta = fac3;\n"
"  if (options3 & (1 << 0))\n"
"    beta = -beta;\n"
"  if (options3 & (1 << 1))\n"
"    beta = ((float)(1)) / beta;\n"
"  *s1 += *s2 * alpha + *s3 * beta;\n"
"}\n"
; //scalar_align1_asbs_s_cpu_cpu

const char * const scalar_align1_asbs_cpu_cpu = 
"// generic kernel for the scalar operation s1 = alpha * s2 + beta * s3, where s1, s2, s3 are not necessarily distinct GPU scalars\n"
"__kernel void asbs_cpu_cpu(\n"
"          __global float * s1,\n"
"          \n"
"          float fac2,\n"
"          unsigned int options2,  // 0: no action, 1: flip sign, 2: take inverse, 3: flip sign and take inverse\n"
"          __global const float * s2,\n"
"          \n"
"          float fac3,\n"
"          unsigned int options3,  // 0: no action, 1: flip sign, 2: take inverse, 3: flip sign and take inverse\n"
"          __global const float * s3)\n"
"{ \n"
"  float alpha = fac2;\n"
"  if (options2 & (1 << 0))\n"
"    alpha = -alpha;\n"
"  if (options2 & (1 << 1))\n"
"    alpha = ((float)(1)) / alpha;\n"
"  float beta = fac3;\n"
"  if (options3 & (1 << 0))\n"
"    beta = -beta;\n"
"  if (options3 & (1 << 1))\n"
"    beta = ((float)(1)) / beta;\n"
"  *s1 = *s2 * alpha + *s3 * beta;\n"
"}\n"
; //scalar_align1_asbs_cpu_cpu

const char * const scalar_align1_asbs_s_gpu_cpu = 
"// generic kernel for the scalar operation s1 += alpha * s2 + beta * s3, where s1, s2, s3 are not necessarily distinct GPU scalars\n"
"__kernel void asbs_s_gpu_cpu(\n"
"          __global float * s1,\n"
"          \n"
"          __global const float * fac2,\n"
"          unsigned int options2,  // 0: no action, 1: flip sign, 2: take inverse, 3: flip sign and take inverse\n"
"          __global const float * s2,\n"
"          \n"
"          float fac3,\n"
"          unsigned int options3,  // 0: no action, 1: flip sign, 2: take inverse, 3: flip sign and take inverse\n"
"          __global const float * s3)\n"
"{ \n"
"  float alpha = fac2[0];\n"
"  if ((options2 >> 2) > 1)\n"
"  {\n"
"    for (unsigned int i=1; i<(options2 >> 2); ++i)\n"
"      alpha += fac2[i];\n"
"  }\n"
"  if (options2 & (1 << 0))\n"
"    alpha = -alpha;\n"
"  if (options2 & (1 << 1))\n"
"    alpha = ((float)(1)) / alpha;\n"
"  float beta = fac3;\n"
"  if (options3 & (1 << 0))\n"
"    beta = -beta;\n"
"  if (options3 & (1 << 1))\n"
"    beta = ((float)(1)) / beta;\n"
"    \n"
"  *s1 += *s2 * alpha + *s3 * beta;\n"
"}\n"
; //scalar_align1_asbs_s_gpu_cpu

const char * const scalar_align1_as_gpu = 
"// generic kernel for the scalar operation s1 = alpha * s2, where s1, s2 are not necessarily distinct GPU scalars\n"
"__kernel void as_gpu(\n"
"          __global float * s1,\n"
"          __global const float * fac2,\n"
"          unsigned int options2,  // 0: no action, 1: flip sign, 2: take inverse, 3: flip sign and take inverse\n"
"          __global const float * s2)\n"
"{ \n"
"  float alpha = fac2[0];\n"
"  if ((options2 >> 2) > 1)\n"
"  {\n"
"    for (unsigned int i=1; i<(options2 >> 2); ++i)\n"
"      alpha += fac2[i];\n"
"  }\n"
"  if (options2 & (1 << 0))\n"
"    alpha = -alpha;\n"
"  if (options2 & (1 << 1))\n"
"    alpha = ((float)(1)) / alpha;\n"
"  *s1 = *s2 * alpha;\n"
"}\n"
; //scalar_align1_as_gpu

const char * const scalar_align1_as_cpu = 
"// generic kernel for the scalar operation s1 = alpha * s2, where s1, s2 are not necessarily distinct GPU scalars\n"
"__kernel void as_cpu(\n"
"          __global float * s1,\n"
"          float fac2,\n"
"          unsigned int options2,\n"
"          __global const float * s2)\n"
"{ \n"
"  float alpha = fac2;\n"
"  if (options2 & (1 << 0))\n"
"    alpha = -alpha;\n"
"  if (options2 & (1 << 1))\n"
"    alpha = ((float)(1)) / alpha;\n"
"  *s1 = *s2 * alpha;\n"
"}\n"
; //scalar_align1_as_cpu

const char * const scalar_align1_asbs_s_gpu_gpu = 
"// generic kernel for the scalar operation s1 += alpha * s2 + beta * s3, where s1, s2, s3 are not necessarily distinct GPU scalars\n"
"__kernel void avbv_v_gpu_gpu(\n"
"          __global float * s1,\n"
"          \n"
"          __global const float * fac2,\n"
"          unsigned int options2,  // 0: no action, 1: flip sign, 2: take inverse, 3: flip sign and take inverse\n"
"          __global const float * s2,\n"
"          \n"
"          __global const float * fac3,\n"
"          unsigned int options3,  // 0: no action, 1: flip sign, 2: take inverse, 3: flip sign and take inverse\n"
"          __global const float * s3)\n"
"{ \n"
"  float alpha = fac2[0];\n"
"  if ((options2 >> 2) > 1)\n"
"  {\n"
"    for (unsigned int i=1; i<(options2 >> 2); ++i)\n"
"      alpha += fac2[i];\n"
"  }\n"
"  if (options2 & (1 << 0))\n"
"    alpha = -alpha;\n"
"  if (options2 & (1 << 1))\n"
"    alpha = ((float)(1)) / alpha;\n"
"  float beta = fac3[0];\n"
"  if ((options3 >> 2) > 1)\n"
"  {\n"
"    for (unsigned int i=1; i<(options3 >> 2); ++i)\n"
"      beta += fac3[i];\n"
"  }\n"
"  if (options3 & (1 << 0))\n"
"    beta = -beta;\n"
"  if (options3 & (1 << 1))\n"
"    beta = ((float)(1)) / beta;\n"
"  \n"
"  *s1 += *s2 * alpha + *s3 * beta;\n"
"}\n"
; //scalar_align1_asbs_s_gpu_gpu

const char * const scalar_align1_asbs_cpu_gpu = 
"// generic kernel for the scalar operation s1 = alpha * s2 + beta * s3, where s1, s2, s3 are not necessarily distinct GPU scalars\n"
"__kernel void asbs_cpu_gpu(\n"
"          __global float * s1,\n"
"          \n"
"          float fac2,\n"
"          unsigned int options2,\n"
"          __global const float * s2,\n"
"          \n"
"          __global const float * fac3,\n"
"          unsigned int options3,\n"
"          __global const float * s3)\n"
"{ \n"
"  float alpha = fac2;\n"
"  if (options2 & (1 << 0))\n"
"    alpha = -alpha;\n"
"  if (options2 & (1 << 1))\n"
"    alpha = ((float)(1)) / alpha;\n"
"  float beta = fac3[0];\n"
"  if ((options3 >> 2) > 1)\n"
"  {\n"
"    for (unsigned int i=1; i<(options3 >> 2); ++i)\n"
"      beta += fac3[i];\n"
"  }\n"
"  if (options3 & (1 << 0))\n"
"    beta = -beta;\n"
"  if (options3 & (1 << 1))\n"
"    beta = ((float)(1)) / beta;\n"
"  \n"
"  *s1 = *s2 * alpha + *s3 * beta;\n"
"}\n"
; //scalar_align1_asbs_cpu_gpu

  }  //namespace kernels
 }  //namespace linalg
}  //namespace viennacl
#endif

