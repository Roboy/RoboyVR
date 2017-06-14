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
		"Queue" = "Transparent"
	}

	Pass
	{

		Cull Off
		ZWrite Off
		Lighting Off
		Blend SrcAlpha OneMinusSrcAlpha
		CGPROGRAM
#pragma vertex vert
#pragma fragment frag
#pragma multi_compile FINAL FOREGROUND BACKGROUND ALPHA KEY
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
		float4x4 copy_projection = UNITY_MATRIX_P;
		copy_projection[0][2] = 0;
		copy_projection[1][2] = 0;

		o.vertex = mul(mul(mul(copy_projection, UNITY_MATRIX_V), UNITY_MATRIX_M), v.vertex);
		//o.vertex = float4(v.vertex.x*2.0, v.vertex.y*2.0, 1, 1); //To fill the screen directly

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

	fragOut frag(v2f i)
	{
		//Get the depth in XYZ format
		float2 uv = i.uv;
		float3 colorXYZ = tex2D(_DepthXYZTex, uv*fixed2(1,-1)).rgb;
		//Compute the depth to work with Unity (1m real = 1m to Unity)
		float depthReal = computeDepthXYZ(colorXYZ);
		//Color from the camera
		float3 colorCamera = tex2D(_CameraTex, uv*fixed2(1, -1)).bgr;

		float alpha = tex2D(_MaskTex, uv).a;

		fragOut o;

		o.color.rgb = colorCamera.rgb;
		o.color.a = 1;
		o.depth = depthReal;



#ifndef FOREGROUND
		o.color.a = alpha;
		o.color.rgb = tex2D(_MaskTex, uv).rgb;

#else

		o.depth = MIN_DEPTH;
		o.color.a = 1;
#endif

#ifdef ALPHA
		o.color.r = o.color.a;
		o.color.g = o.color.a;
		o.color.b = o.color.a;
		o.color.a = 1;
		o.depth = MIN_DEPTH;

#endif


#ifdef BACKGROUND
		o.color.a = 0;
#endif
#ifdef KEY
		o.depth = MIN_DEPTH;

		o.color.rgb *= o.color.a;
		o.color.rgb = clamp(o.color.rgb, float3(0., 0., 0.), float3(1, 1, 1));
		
		o.color.a = 1;
#endif

		return o;
	}
	ENDCG
	}



	}
}