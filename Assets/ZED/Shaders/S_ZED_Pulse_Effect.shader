Shader "ZED/ZED Pulse Effect" {

	Properties{
		_MainTex("Base (RGB)", 2D) = "white" {}
		_GlowColor("Pulse Color", Color) = (1.0, 1.0, 1.0, 1.0)
		_BackColor("Background Color", Color) = (1.0, 1.0, 1.0, 1.0)

		_PulseIntensity("Pulse Intensity",  Float) = 1.0

		_BackGroundSpecular("Background Metallic",  Range(0, 1)) = 1.0
		_BackGroundGloss("Background Smoothness",  Range(0, 1)) = 1.0

		_Frequency("Glow Frequency", Float) = 1.0
		_MinPulseVal("Minimum Glow Multiplier", Range(0, 1)) = 0.5
		_Thickness("Thickness", Int) = 5
	}


		SubShader{
		Tags{"Queue" ="Transparent" "RenderType" = "Transparent" }


		CGPROGRAM
			// Physically based Standard lighting model, and enable shadows on all light types
#pragma surface surf Standard fullforwardshadows alpha:fade

			// Use shader model 3.0 target, to get nicer looking lighting
#pragma target 3.0

		sampler2D	_MainTex;
	fixed4		_GlowColor;
	fixed4 _BackColor;
	half		_Frequency;
	half		_MinPulseVal;
	float _BackGroundSpecular;
	float _PulseIntensity;
	float _BackGroundGloss;
	float _Thickness;
	float4 _MainTex_TexelSize;

	struct Input {
		float2 uv_MainTex;
	};

	void surf(Input IN, inout SurfaceOutputStandard o)
	{
		half thickness = _MainTex_TexelSize.x*_Thickness;

		half4 c = tex2D(_MainTex, IN.uv_MainTex);

		// Calculate distance from effect center, in pixels
		float2 vecFromCenter = float2(0, (IN.uv_MainTex.y));
		float radius = sqrt(dot(vecFromCenter, vecFromCenter));

		// Calculate current size of the pulse
		float outerRadius = 0.5*sin(_Frequency * _Time.x) + 0.5;
		float innerRadius = outerRadius - _Thickness;

		float middle = (outerRadius + innerRadius)*0.5;

		// Calculate a function that will be 1.0 inside the pulse, 0.0 outside,
		// with a 1px-wide falloff to antialias the edges
		float pulse = saturate(radius - innerRadius) * saturate(outerRadius - radius);
		pulse /= saturate(middle - innerRadius) * saturate(outerRadius - middle);

		// Lerp between the pulse color and background color based on this
		half3 pulseColor = _GlowColor.rgb;
		half3 backgroundColor = _BackColor.rgb;
		o.Albedo = (1 - pulse)*backgroundColor;
		o.Emission = _PulseIntensity*pulse*pulseColor;

		// Approximate linear-to-sRGB conversion (improves antialiasing quality)
		//o.Albedo = pulseColor;
		o.Metallic = (1 - pulse)*_BackGroundSpecular;
		o.Smoothness = (1 - pulse)*_BackGroundGloss;
		o.Alpha = _BackColor.a;

	}
	ENDCG
		}
			FallBack "Diffuse"
}