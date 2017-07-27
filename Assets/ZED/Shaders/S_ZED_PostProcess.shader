// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

Shader "ZED/ZED Postprocess" {
	Properties
	{
		_MainTex("Texture", 2D) = "white" {}
	}
		SubShader
	{
		Tags{ "RenderType" = "Opaque" }
		LOD 100

		Pass
	{
		CGPROGRAM
#pragma vertex vert
#pragma fragment frag

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


	uniform sampler2D _MainTex;
	float4 _MainTex_ST;
	sampler2D _virtTex_ZED;
	sampler2D _PostprocessTex;
	float _time;
	float4 _MainTex_TexelSize;

	float rand(float2 co) {
		return frac(sin(dot(co.xy, float2(12.9898, 78.233))) * 43758.5453);
	}

	//Vertex Shader
	v2f vert(appdata v)
	{
		v2f o;
		o.vertex = GetPosWithoutOpticalCenter(v.vertex);
		o.uv = TRANSFORM_TEX(v.uv, _MainTex);
		//o.uv.y = 1 - o.uv.y;
		return o;
	}

	//Fragment Shader
	half4 frag(v2f i) : COLOR{
		float4 mask = tex2D(_virtTex_ZED, i.uv);
		float4 zed = tex2D(_MainTex, i.uv);
		float4 virt = tex2D(_PostprocessTex, i.uv);
		if (mask.r > 0)
		{
			float rnd = rand(_time*i.uv);
			float4 res = virt + (10 * float4(rnd, rnd, rnd, 1) - 5) / 255.;
			return res;
		}
		else
		{
			return zed;
		}
		//return virt;
	}
		ENDCG
	}
	}
}
