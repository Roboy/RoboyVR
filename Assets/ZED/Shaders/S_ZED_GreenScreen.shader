﻿Shader "ZED/ZED Green Screen" {
	Properties
	{
		_DepthXYZTex("Depth texture", 2D) = "" {}
		_CameraTex("Texture from ZED", 2D) = "" {}
		_MaskTex("Texture mask from ZED", 2D) = "" {}
	}

		SubShader
	{
	Tags{
		"RenderType" = "Transparent"
		"Queue" = "Transparent-2"
	}

	Pass
	{
		/*To use as a garbage matte*/
		Stencil{
		Ref 2
		Comp[_ZEDStencilComp]
		Pass keep
	}

		
		ZWrite On
		Blend SrcAlpha OneMinusSrcAlpha
		CGPROGRAM
#pragma vertex vert
#pragma fragment frag
#pragma multi_compile FINAL FOREGROUND BACKGROUND ALPHA KEY
#pragma multi_compile __ ZED_XYZ
#pragma multi_compile_fwdbase
#pragma multi_compile_fwdadd_fullshadows
#pragma target 4.0
#include "UnityCG.cginc"
#include "S_ZED_Utils.cginc"
		struct appdata
	{
		float4 vertex : POSITION;
		float2 uv : TEXCOORD0;
	};

	struct v2f
	{
		float2 uv : TEXCOORD0;
		float4 vertex : SV_POSITION;
	};
	struct fragOut {
		float depth : SV_Depth;
		fixed4 color : SV_Target;
	};
	sampler2D _DepthXYZTex;
	sampler2D _CameraTex;
	float4 _CameraTex_ST;

	sampler2D _MaskTex;
	uniform float4x4 _ProjectionMatrix;

	v2f vert(appdata v)
	{
		v2f o;
		o.vertex =  GetPosWithoutOpticalCenter(v.vertex);

		o.uv = TRANSFORM_TEX(v.uv, _CameraTex);
		return o;
	}



	uint _numberColors;
	float4 _CameraTex_TexelSize;

	int _erosion;
	uniform float4 _keyColor;
	uniform float _smoothness;
	uniform float _range;
	uniform float _spill;
	float _whiteClip;
	float _blackClip;
	int _HasShadows;

	sampler2D _NormalsTex;
	sampler2D _DirectionalShadowMap;

	float4 _AmnbientLight;

	fragOut frag(v2f i)
	{
		//Get the depth in XYZ format
		float2 uv = i.uv;
#if defined(ZED_XYZ)
		float3 colorXYZ = tex2D(_DepthXYZTex,  fixed2(uv.x, 1 - uv.y)).rgb;
		//Compute the depth to work with Unity (1m real = 1m to Unity)
		float depthReal = computeDepthXYZ(colorXYZ.rgb);
#else
		float3 colorXYZ = tex2D(_DepthXYZTex,  fixed2(uv.x, 1 - uv.y)).rgb;
		//Compute the depth to work with Unity (1m real = 1m to Unity)
		float depthReal = computeDepthXYZ(colorXYZ.r);
#endif
		//Color from the camera
		float3 colorCamera = tex2D(_CameraTex, fixed2(uv.x, 1 - uv.y)).bgr;

		float alpha = tex2D(_MaskTex, uv).a;



	

		fragOut o;

		o.color.rgb = colorCamera.rgb;
		o.color.a = 1;

		float a = alpha <= 0.0 ? 0 : 1;
		o.depth = depthReal*a;



#ifndef FOREGROUND

		fixed4 c = 0;
		float4 color =  tex2D (_MaskTex, uv).rgba;
//#if defined(SHADOWS_SCREEN)
		if (_HasShadows == 1) {
			c = half4(color*(saturate(tex2D(_DirectionalShadowMap, fixed2(uv.x, uv.y)))));
		}
		else {
			c = half4(color);
		}
//#endif



		o.color.a = alpha;
		o.color.rgb = c.rgb;

#else

		o.depth = MAX_DEPTH;
		o.color.a = 1;
#endif

#ifdef ALPHA
		o.color.r = o.color.a;
		o.color.g = o.color.a;
		o.color.b = o.color.a;
		o.color.a = 1;
		o.depth = MAX_DEPTH;

#endif


#ifdef BACKGROUND
		o.color.a = 0;
#endif
#ifdef KEY
		o.depth = MAX_DEPTH;
		o.color.rgb =  tex2D (_MaskTex, uv).rgb;
		o.color.rgb *= alpha;
		o.color.rgb = clamp(o.color.rgb, float3(0., 0., 0.), float3(1, 1, 1));
		
		o.color.a = 1;
#endif

		return o;
	}
	ENDCG
	}

	// ---- forward rendering additive lights pass:
	Pass {

		/*To use as a garbage matte*/
		Stencil{
		Ref 2
		Comp[_ZEDStencilComp]
		Pass keep
	}
		//Offset 2000, 2000
		Name "FORWARD"
		Tags { "RenderType" = "Transparent"
		"Queue" = "Transparent-2"
		"Lightmode"="Always" 
	}
		ZWrite Off
		//Offset 0, –1
		Blend One One
CGPROGRAM

// compile directives
#pragma vertex vert_surf
#pragma fragment frag_surf
#pragma target 4.0
#pragma multi_compile_fwdadd_fullshadows
#pragma skip_variants INSTANCING_ON
#pragma multi_compile FINAL FOREGROUND BACKGROUND ALPHA KEY
#pragma multi_compile __ ZED_XYZ
#include "HLSLSupport.cginc"
#include "UnityShaderVariables.cginc"
#define UNITY_PASS_FORWARDADD
#include "UnityCG.cginc"
#include "Lighting.cginc"
#include "UnityPBSLighting.cginc"

#include "AutoLight.cginc"
#include "S_ZED_Utils.cginc"

#define ZED_SPOT_LIGHT_DECLARATION
#define ZED_POINT_LIGHT_DECLARATION

#include "S_ZED_Lighting.cginc"
#define INTERNAL_DATA
#define WorldReflectionVector(data,normal) data.worldRefl
#define WorldNormalVector(data,normal) normal


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


sampler2D _MaskTex;


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


// fragment shader
void frag_surf (v2f_surf IN, out fixed4 outColor : SV_Target, out float outDepth : SV_Depth){

#if FINAL
#if defined(ZED_XYZ)
	float3 zed_xyz = tex2D(_DepthXYZTex, IN.pack0).rgb;
	outColor = computeLighting(IN.pack0.xy,
		mul(unity_CameraToWorld, float4(zed_xyz.r, zed_xyz.g, zed_xyz.b, 1)),
		tex2D(_MaskTex, fixed2(IN.pack0.x, 1 - IN.pack0.y)).a);
	outDepth = computeDepthXYZ(float3(zed_xyz.x, zed_xyz.y, zed_xyz.z - 0.01*zed_xyz.z)); // Write the depth in the same position, causes flickering, so move the light a little backward
#else 
	float3 zed_xyz = tex2D(_DepthXYZTex, IN.pack0).xxx;
	float3 worldspace;
	float alpha = tex2D(_MaskTex, fixed2(IN.pack0.x, 1 - IN.pack0.y)).a;
	GET_XYZ(IN, zed_xyz.x, worldspace)
		outColor = computeLighting(IN.pack0.xy,
			worldspace,
			alpha);
	float a = alpha <= 0.0 ? 0 : 1;
	outDepth = computeDepthXYZ(zed_xyz.z - 0.01*zed_xyz.z)*a; // Write the depth in the same position, causes flickering, so move the light a little backward

#endif
#else
  outDepth = 0;
  outColor = 0;
  #endif
}

ENDCG

}
	}
	
}