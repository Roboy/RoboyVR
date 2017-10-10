Shader "ZED/ZED Forward"
{
	Properties
	{
		[HideInInspector] _MainTex("Base (RGB) Trans (A)", 2D) = "" {}
		_DepthXYZTex("Depth texture", 2D) = "" {}
		_CameraTex("Texture from ZED", 2D) = "" {}
	}

	SubShader
	{
		Stencil{
		Ref 2
		Comp[_ZEDStencilComp]
		Pass keep
	}


		Pass
		{
		
	Tags { "RenderType"="Opaque" "Queue"="Geometry" "LightMode" = "Always" }
			ZWrite On
			ZTest Always
			Cull Front

			CGPROGRAM
			#pragma target 3.0
			#pragma vertex vert
			#pragma fragment frag
			#pragma exclude_renderers nomrt
#pragma multi_compile __ ZED_XYZ
			#include "UnityCG.cginc"
			#include "S_ZED_Utils.cginc"
			#include "Lighting.cginc"
			#include "UnityCG.cginc"
		sampler2D ZEDMaskTexGreenScreen;
	float4 _MaskTex_ST;
			struct v2f
			{
				float4 pos : SV_POSITION;
				float4 screenUV : TEXCOORD0;
			};
			v2f vert (float3 v : POSITION)
			{
				v2f o;
				o.pos = float4(v.x*2.0,v.y*2.0,0,1);
				o.screenUV = float4(v.x-0.5,v.y-0.5,0,1);
				return o;
			}

			sampler2D _MainTex;
			sampler2D _DepthXYZTex;
			uniform float4 _DepthXYZTex_TexelSize;
			uniform float4 _CameraRotationQuat;
			float4 _CameraPosition;
			int ZEDGreenScreenActivated;
			void frag(v2f i, 
					  out half4 outColor : COLOR0, 
					  out float outDepth:DEPTH)
			{
				float2 uv = i.screenUV.xy / i.screenUV.w;

				outColor = tex2D (_MainTex, uv).bgra;
				float2 uvTex = float2(uv.x + 1.0f, uv.y + 1.0f);
				uvTex.y = 1 - uvTex.y;
#if defined(ZED_XYZ)
				float4 dxyz = tex2D(_DepthXYZTex, uv).xyzw;
				float realDepth = computeDepthXYZ(dxyz.rgb);
#else
				float4 dxyz = tex2D(_DepthXYZTex, uv).xxxx;
				float realDepth = computeDepthXYZ(dxyz.r);
#endif
				if (ZEDGreenScreenActivated == 1) {
					float a = (tex2D(ZEDMaskTexGreenScreen, uvTex).a);
					
					a = a <= 0.5 ? 0 : 1;

					outDepth = realDepth*a;
				}
				else {
					outDepth = realDepth;
				}

			}

			ENDCG
		}
	}

}
