// Upgrade NOTE: replaced 'unity_World2Shadow' with 'unity_WorldToShadow'


Shader "ZED/ZED RGB-Depth"
{

  	Properties {
		_Color ("Color", Color) = (1,1,1,1)
		_MainTex ("Albedo (RGB)", 2D) = "white" {}
	}
	SubShader {

	//Tags { "RenderType"="Opaque" }


		ZWrite On
	Pass {
		Name "FORWARD"
		Tags { "LightMode" = "ForwardBase" "PassFlags" = "OnlyDirectional"  }

		Cull Off
CGPROGRAM
// compile directives
#pragma vertex vert_surf
#pragma fragment frag_surf

#pragma target 4.0
#pragma multi_compile_fwdbase
#pragma multi_compile_fwdadd_fullshadows
#pragma multi_compile __ ZED_XYZ
#include "HLSLSupport.cginc"
#include "UnityShaderVariables.cginc"

#define UNITY_PASS_FORWARDBASE
#include "UnityCG.cginc"

#include "AutoLight.cginc"
#include "S_ZED_Utils.cginc"
#include "S_ZED_Lighting.cginc"

sampler2D _MainTex;

struct Input {
	float2 uv_MainTex;
};

float4x4 _CameraMatrix;
sampler2D _DirectionalShadowMap;


struct v2f_surf {
  float4 pos : SV_POSITION;
  float2 pack0 : TEXCOORD0;
  SHADOW_COORDS(4)
};

float4 _MainTex_ST;
sampler2D _DepthXYZTex;
sampler2D _DepthXTex;

sampler2D _MaskTex;
int _HasShadows;

// vertex shader
v2f_surf vert_surf (appdata_full v) {

  v2f_surf o;
  UNITY_INITIALIZE_OUTPUT(v2f_surf,o);
  o.pos = GetPosWithoutOpticalCenter(v.vertex);

  o.pack0.xy = TRANSFORM_TEX(v.texcoord, _MainTex);
  o.pack0.y = 1 - o.pack0.y;

  TRANSFER_SHADOW(o); 

  return o;
}
// fragment shader
void frag_surf (v2f_surf IN, out fixed4 outColor : SV_Target, out float outDepth : SV_Depth) {

	float2 uv = IN.pack0.xy;

#if defined(ZED_XYZ)
	float3 zed_xyz = tex2D(_DepthXYZTex, uv).xyz;
	outDepth = computeDepthXYZ(zed_xyz.z);
#else
	float3 zed_xyz = tex2D (_DepthXYZTex, uv).xxx;
	outDepth = computeDepthXYZ(zed_xyz.z);
#endif
	fixed4 c = 0;
	float4 color =  tex2D (_MainTex, uv).bgra;
	float3 normals = tex2D (_NormalsTex, uv).rgb;

	if (_HasShadows == 1) {
		c = half4(color*saturate(tex2D(_DirectionalShadowMap, fixed2(uv.x, 1 - uv.y))));
	}
	else {
		c = half4(color);
	}
	c.a = 1;
	
    outColor = c;
}

ENDCG

}
	// ---- forward rendering additive lights pass:
	Pass {
		Name "FORWARD"
		Tags {"Lightmode"="Always" }
		ZWrite On
		Cull Off

		Blend One One
CGPROGRAM

#pragma vertex vert_surf
#pragma fragment frag_surf
#pragma target 4.0
#pragma multi_compile_fwdadd_fullshadows
#pragma multi_compile __ ZED_XYZ
#include "HLSLSupport.cginc"
#include "UnityShaderVariables.cginc"
#include "UnityCG.cginc"
#include "Lighting.cginc"
#include "UnityPBSLighting.cginc"
#include "AutoLight.cginc"
#include "S_ZED_Utils.cginc"

#define ZED_SPOT_LIGHT_DECLARATION
#define ZED_POINT_LIGHT_DECLARATION

#include "S_ZED_Lighting.cginc"

		sampler2D _MainTex;

		struct Input {
			float2 uv_MainTex;
		};
		


// vertex-to-fragment interpolation data
struct v2f_surf {
  float4 pos : SV_POSITION;
  float2 pack0 : TEXCOORD0; // _MainTex
  SHADOW_COORDS(3)
#if !defined(ZED_XYZ)
  ZED_WORLD_DIR(1)
#endif

};
float4 _MainTex_ST;
sampler2D _DepthXYZTex;
float4x4 _CameraMatrix;
float4x4 _ProjectionCameraInverse;

sampler2D _MaskTex;

sampler2D _ZEDShadowMap;

// vertex shader
v2f_surf vert_surf (appdata_full v) {
	v2f_surf o;
	UNITY_INITIALIZE_OUTPUT(v2f_surf,o);

	o.pos = GetPosWithoutOpticalCenter(v.vertex);
#if !defined(ZED_XYZ)
	ZED_TRANSFER_WORLD_DIR(o)
#endif
	o.pack0.xy = TRANSFORM_TEX(v.texcoord, _MainTex);
	o.pack0.y = 1 - o.pack0.y;

	TRANSFER_SHADOW(o); // pass shadow coordinates to pixel shader
	return o;
}

uniform int _isLinear;
// fragment shader
void frag_surf (v2f_surf IN, out fixed4 outColor : SV_Target, out float outDepth : SV_Depth){

#if defined(ZED_XYZ)
	float3 zed_xyz = tex2D(_DepthXYZTex, IN.pack0).xyz;
	outDepth = computeDepthXYZ(zed_xyz.z - 0.0001*zed_xyz.z); // Write the depth in the same position, causes flickering, so move the light a little backward

	outColor = computeLighting(IN.pack0.xy,
		mul(unity_CameraToWorld, float4(zed_xyz.r, zed_xyz.g, zed_xyz.b, 1)),
		1);
#else
	float3 zed_xyz = tex2D(_DepthXYZTex, IN.pack0).xxx;
	outDepth = computeDepthXYZ(zed_xyz.z - 0.0001*zed_xyz.z); // Write the depth in the same position, causes flickering, so move the light a little backward
	float3 worldspace;
	GET_XYZ(IN, zed_xyz.x, worldspace)

	outColor = saturate(computeLighting(IN.pack0.xy, worldspace, 1));

#endif


}

ENDCG

}

     }
	    Fallback Off

}
