Shader "ZED/ZED RGB-Depth"
{
	Properties
	{
		[HideInInspector] _MainTex("Base (RGB) Trans (A)", 2D) = "" {}
		_DepthXYZTex("Depth texture", 2D) = "" {}
		_CameraTex("Texture from ZED", 2D) = "" {}
	
	}
	SubShader
	{
		// No culling or depth
		Cull Off
		ZWrite On
		Tags{
		"RenderType" = "Opaque"
		"Queue" = "Geometry"
		}


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

			struct Input {
				float2 uv_MainTex;
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
			uniform float4x4 _ProjectionMatrix;
			sampler2D _DepthXYZTex;
			sampler2D _CameraTex;
			uniform float4 _MaskTex_ST;
			float4 _MainTex_ST;
			uniform int _isGrey;
		

			v2f vert(appdata v)
			{
				v2f o;
				//Remove the optical center of the frame -> recenter frame
				float4x4 copy_projection = UNITY_MATRIX_P;
				copy_projection[0][2] = 0;
				copy_projection[1][2] = 0;
				o.vertex = mul(mul(mul(copy_projection, UNITY_MATRIX_V), UNITY_MATRIX_M ), v.vertex);
				//o.vertex = float4(v.vertex.x*2.0, v.vertex.y*2.0, 1, 1); //To fill the screen directly

				o.uv = TRANSFORM_TEX(v.uv, _MainTex);
				return o;
			}


			fragOut frag(v2f i)
			{
				//Get the depth in XYZ format
				float2 uv = i.uv;
				uv.y = 1 - uv.y;
				float3 colorXYZ = tex2D(_DepthXYZTex, uv).rgb;
				//Compute the depth to work with Unity (1m real = 1m to Unity)
				float depthReal = computeDepthXYZ(colorXYZ);

				//Color from the camera
				fragOut o;
				if (_isGrey == 0) {
					float3 colorCamera = tex2D(_CameraTex, uv).bgr;
					o.color.rgb = colorCamera.rgb;
				}
				else {
					float colorCamera = tex2D(_CameraTex, uv).a;
					o.color.rgb = colorCamera;
				}
				o.color.a = 1;

				o.depth = depthReal;
				return o;
			}
			ENDCG
		}
	}

}
