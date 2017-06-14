
#if UNITY_VERSION >= 550
#define MAX_DEPTH 0.001f
#define MIN_DEPTH 0.999f
#else
#define MAX_DEPTH 0.999f
#define MIN_DEPTH 0.001f
#endif

//Compute the depth of ZED to the Unity scale
float computeDepthXYZ(float3 colorXYZ) {
if(isinf(colorXYZ.r)) return MAX_DEPTH;
if(colorXYZ.r != colorXYZ.r) return MIN_DEPTH;
	//reverse Y and Z axes
	colorXYZ.b = -colorXYZ.b;
	colorXYZ.g = -colorXYZ.g;

	float4 v = float4(colorXYZ, 1);
	//Project to unity's coordinate
	float4 depthXYZVector = mul(UNITY_MATRIX_P, v);

#if UNITY_VERSION >= 550
	if (depthXYZVector.w != depthXYZVector.w) return MAX_DEPTH;
#else
	if (depthXYZVector.w != depthXYZVector.w) return MIN_DEPTH;
#endif
	depthXYZVector.b /= (depthXYZVector.w);
	float depthReal = depthXYZVector.b;

	clamp(depthReal, MIN_DEPTH, MAX_DEPTH);

	return depthReal;
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
//Thresh is the degree of similarity
//Slope is the power of the alpha
float computeAlphaYUVFromYUV(float3 colorCamera, in float3 keyColor) {
	float3 mask = RGBtoYUV(keyColor);
	return distance(float2(mask.y, mask.z), float2(colorCamera.y, colorCamera.z));
}

