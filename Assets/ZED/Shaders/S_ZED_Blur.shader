Shader "ZED/ZED Blur"
{
	Properties
	{
		_MainTex("Texture", 2D) = "white" {}
	}
		SubShader
	{
		Tags { "RenderType" = "Opaque" }
		LOD 100

		Pass
		{
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag

			#include "UnityCG.cginc"

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
			float4 _MainTex_ST;

			v2f vert(appdata v)
			{
				v2f o;
				o.vertex = UnityObjectToClipPos(v.vertex);
				o.uv = TRANSFORM_TEX(v.uv, _MainTex);
				return o;
			}
			float4 _MainTex_TexelSize;
			uniform int horizontal;

			uniform float weights[5];
			uniform float offset[5];
			float4 frag(v2f vi) : SV_Target
			{



			float2 uv = vi.uv;
			float result = tex2D(_MainTex, uv).a * weights[0]; // current fragment's contribution
			int i = 0;
			if (horizontal == 1)
			{
				for (i = 1; i < 3; ++i)
				{
					result += tex2D(_MainTex, uv + float2(offset[i] * _MainTex_TexelSize.x, 0.0)).a * weights[i];
					result += tex2D(_MainTex, uv - float2(offset[i] * _MainTex_TexelSize.x, 0.0)).a * weights[i];
				}
			}
			else
			{
				for (i = 1; i < 3; ++i)
				{
					result += tex2D(_MainTex, uv + float2(0.0, offset[i] * _MainTex_TexelSize.y)).a * weights[i];
					result += tex2D(_MainTex, uv - float2(0.0, offset[i] * _MainTex_TexelSize.y)).a * weights[i];
				}
			}

			return float4(tex2D(_MainTex, uv).rgb, result);
			}
			ENDCG
		}

		Pass
			{
				CGPROGRAM
#pragma vertex vert
#pragma fragment frag

#include "UnityCG.cginc"

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
			float4 _MainTex_ST;

			v2f vert(appdata v)
			{
				v2f o;
				o.vertex = UnityObjectToClipPos(v.vertex);
				o.uv = TRANSFORM_TEX(v.uv, _MainTex);
				return o;
			}
			float4 _MainTex_TexelSize;
			uniform int horizontal;

			uniform float weights2[5];
			uniform float offset2[5];
			float4 frag(v2f vi) : SV_Target
			{



				float2 uv = vi.uv;
				float3 result = tex2D(_MainTex, uv).rgb * weights2[0]; // current fragment's contribution
				result.r = tex2D(_MainTex, uv).r;
				int i = 0;
				if (horizontal == 1)
				{
					for (i = 1; i < 3; ++i)
					{
						result.gb += tex2D(_MainTex, uv + float2(offset2[i] * _MainTex_TexelSize.x, 0.0)).gb * weights2[i];
						result.gb += tex2D(_MainTex, uv - float2(offset2[i] * _MainTex_TexelSize.x, 0.0)).gb * weights2[i];
					}
				}
				else
				{
					for (i = 1; i < 3; ++i)
					{
						result.gb += tex2D(_MainTex, uv + float2(0.0, offset2[i] * _MainTex_TexelSize.y)).gb * weights2[i];
						result.gb += tex2D(_MainTex, uv - float2(0.0, offset2[i] * _MainTex_TexelSize.y)).gb * weights2[i];
					}
				}

				return float4(result.x, result.y, result.z, 1);
			}
				ENDCG
			}
	}
}
