#if !defined(ZED_UTILS)
#define ZED_UTILS


#define MAX_DEPTH 0.999f


#if UNITY_REVERSED_Z
#define NEAR_DEPTH MAX_DEPTH
#define FAR_DEPTH 1 - MAX_DEPTH
#else
#define NEAR_DEPTH 1 - MAX_DEPTH
#define FAR_DEPTH MAX_DEPTH
#endif


#define MAX_ZED_DEPTH 20
#define MIN_ZED_DEPTH 0.1f

#define ZED_CLAMP(name) name = clamp(name,MIN_ZED_DEPTH, MAX_ZED_DEPTH);

#if UNITY_REVERSED_Z
#define ZED_DEPTH_CLAMP(name) clamp(name,FAR_DEPTH, NEAR_DEPTH)
#else
#define ZED_DEPTH_CLAMP(name) clamp(name,NEAR_DEPTH, FAR_DEPTH)
#endif

#define ZED_PI 3.14159265359
//Compute the depth of ZED to the Unity scale
float computeDepthXYZ(float3 colorXYZ) {
	
	if (isinf(colorXYZ.z) && colorXYZ.z > 0) return FAR_DEPTH;
	if (isinf(colorXYZ.z) && colorXYZ.z < 0) return NEAR_DEPTH;
	colorXYZ = clamp(colorXYZ, 0.01, 20);
	//reverse Y and Z axes
	colorXYZ.b = -colorXYZ.b;
	colorXYZ.g = -colorXYZ.g;

	float4 v = float4(colorXYZ, 1);
	//Project to unity's coordinate
	float4 depthXYZVector = mul(UNITY_MATRIX_P, v);

	if (depthXYZVector.w != depthXYZVector.w) return FAR_DEPTH;

	depthXYZVector.b /= (depthXYZVector.w);
	float depthReal = depthXYZVector.b;
	
	return ZED_DEPTH_CLAMP(depthReal);
}

float3 applyQuatToVec3(float4 q, float3 v)
{
	float3 t = 2 * cross(q.xyz, v);
	return v + q.w * t + cross(q.xyz, t);
}

//Compute the depth of ZED to the Unity scale
float computeDepthXYZ(float colorXYZ) {
	if (isinf(colorXYZ) && colorXYZ > 0) return FAR_DEPTH;
	if (isinf(colorXYZ) && colorXYZ < 0) return NEAR_DEPTH;

	if (colorXYZ.r != colorXYZ.r) return FAR_DEPTH;
	colorXYZ = clamp(colorXYZ, 0.01, 20);

	//reverse Y and Z axes
	colorXYZ = -colorXYZ;

	float4 v = float4(0,0, colorXYZ, 1);
	//Project to unity's coordinate
	float4 depthXYZVector = mul(UNITY_MATRIX_P, v);

	if (depthXYZVector.w != depthXYZVector.w) return FAR_DEPTH;

	depthXYZVector.b /= (depthXYZVector.w);
	float depthReal = depthXYZVector.b;

	return ZED_DEPTH_CLAMP(depthReal);
}

float4 GetPosWithoutOpticalCenter(float4 vertex) {
    float4x4 copy_projection = UNITY_MATRIX_P;
		copy_projection[0][2] = 0;
		copy_projection[1][2] = 0;
  return mul(mul(mul(copy_projection, UNITY_MATRIX_V), UNITY_MATRIX_M), vertex);
}


float3 RGBtoYUV(float3 rgb)
{
	float Y = .299 * rgb.x + .587 * rgb.y + .114 * rgb.z; // Luma
	float U = -.147 * rgb.x - .289 * rgb.y + .436 * rgb.z + 0.5; // Delta Blue
	float V = .615 * rgb.x - .515 * rgb.y - .100 * rgb.z + 0.5; // Delta Red
	return float3(Y, U, V);
}

//Algorithm to compute the alpha of a frag depending of the similarity of a color.
//ColorCamera is the color from a texture given by the camera
float computeAlphaYUVFromYUV(float3 colorCamera, in float3 keyColor) {
	return distance(float2(keyColor.y, keyColor.z), float2(colorCamera.y, colorCamera.z));
}

#endif
