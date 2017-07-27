Shader "ZED/ZED Postprocess Blend"
{
	Properties
	{
		_MainTex ("Texture", 2D) = "white" {}
	}
	SubShader
	{
		Tags { "RenderType"="Opaque" }
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

			sampler2D _MainTex;
			sampler2D _ZEDMeshTex;
			float4 _ZEDMeshTex_ST;
			float4 _MainTex_ST;
			int _IsTextured;


			v2f vert (appdata v)
			{
				v2f o;
				o.vertex = GetPosWithoutOpticalCenter(v.vertex);

				o.uv = TRANSFORM_TEX(v.uv, _MainTex);
				return o;
			}
			
			fixed4 frag(v2f i) : SV_Target
			{
				float4 meshColor = tex2D(_ZEDMeshTex, i.uv);
				//return meshColor;
				if (_IsTextured == 0) {

					if (length(meshColor) < 0.1) {
						return tex2D(_MainTex, i.uv);
					}
					return  clamp(tex2D(_MainTex, i.uv)*meshColor, float4(0.35f,0.65f,0.95f,1), float4(0.4f, 0.7f, 1.0f, 1));
				}
				else {
					if (meshColor.r != 0) {
						return meshColor;
					}
				}
				return  tex2D(_MainTex, i.uv);
			}

			ENDCG
		}
	}
}
