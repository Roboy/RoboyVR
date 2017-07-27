Shader "ZED/ZED Deferred"
{
Properties
	{
		[HideInInspector] _MainTex("Base (RGB) Trans (A)", 2D) = "" {}
		_DepthXYZTex("Depth texture", 2D) = "" {}
		_CameraTex("Texture from ZED", 2D) = "" {}
	}
	SubShader
	{
		
		Pass
		{
		// Stencil is 2^8 bits, Unity uses 4 first bits 
		// 1 0 0 0 0 0 0 0 enables all lights
		// 1 1 0 0 0 0 0 0 enables all except the light may be not rendered if too far way
		// 1 1 1 0 0 0 0 0 enables all lights

		Stencil {
                Ref 128 
                Comp always
                Pass replace
            }
		
	//Tags { "RenderType"="Opaque" "Queue"="Geometry" "LightMode" = "Always" }
			ZWrite On
			Cull Front

			CGPROGRAM
			#pragma target 4.0
			#pragma vertex vert
			#pragma fragment frag
			
			#include "UnityCG.cginc"
			#include "S_ZED_Utils.cginc"
			#include "Lighting.cginc"
			#include "UnityCG.cginc"
			 #pragma multi_compile ___ UNITY_HDR_ON
			#pragma multi_compile __ ZED_XYZ
			struct v2f
			{
				float4 pos : POSITION;
				float4 screenUV : TEXCOORD0;
			};
			v2f vert (v2f v)
			{
				v2f o;
				o.pos = float4(v.pos.x*2.0,v.pos.y*2.0,1,1);
				o.screenUV = float4(v.pos.x-0.5,v.pos.y-0.5,0,1);
				return o;
			}
			sampler2D _MainTex;
			sampler2D _DepthXYZTex;
			float4 _DepthXYZTex_TexelSize;
			uniform float4 _CameraRotationQuat;
			sampler2D _MaskTex;
			sampler2D _NormalsTex;
			float _Exposure;
			uniform int _ZEDReferenceMeasure;
			void frag(v2f i, 
					  out half4 outColor : SV_Target0, 
					  out half4 outSpecRoughness : SV_Target1, 
					  out half4 outNormal : SV_Target2, 
					  out half4 outEmission : SV_Target3,
					  out float outDepth:DEPTH)
			{
				float2 uv = i.screenUV.xy / i.screenUV.w;
#if defined(ZED_XYZ)
				float4 dxyz = tex2D (_DepthXYZTex, uv).xyzw;
				float d = computeDepthXYZ(dxyz.xyz);
#else
				float4 dxyz = tex2D(_DepthXYZTex, uv).xxxx;
				float d = computeDepthXYZ(dxyz.x);
#endif
				float2 uvTex = float2(uv.x + 1.0f, uv.y + 1.0f);
				uvTex.y = 1 - uvTex.y;

				outSpecRoughness = half4(0,0,0,0);

				
				float3 normals = tex2D(_NormalsTex, uv).rgb;
				outColor = tex2D (_MainTex, uv).bgra;
				outDepth = d;
				#if UNITY_HDR_ON
				outEmission = half4(0,0,0,0);
				#else
				outEmission = half4(1,1,1,0);

				#endif
				outColor.a = 1;
				//if (_ZEDReferenceMeasure == 1) {
				normals = applyQuatToVec3(_CameraRotationQuat, float3(normals));
				//}
				//else {
				//
				//}
				normals = normalize(normals);
				
				outNormal.rgb = normals*0.5 + 0.5;

				outNormal.w = 0.33; // Used as mask
			}
			ENDCG
		}
	}
}