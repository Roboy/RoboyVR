#if !defined(ZED_LIGHTING)
#define ZED_LIGHTING

#include "UnityCG.cginc"
#include "Lighting.cginc"
#include "UnityPBSLighting.cginc"

#include "AutoLight.cginc"
/******************************************************************/
/************** Point lights information **************************/
/******************************************************************/
struct ZED_PointLight{
	float4 color;
	float range;
	float3 position;
};


/******************************************************************/
/************** Spot lights information **************************/
/******************************************************************/
struct ZED_SpotLight{
	float4 color;
	float3 position;
	float4 direction;
	float4 params;// angle, intensity, 1/range, cone interior
};

sampler2D _NormalsTex;
uniform float4 _CameraRotationQuat;

#if defined(ZED_SPOT_LIGHT_DECLARATION)
StructuredBuffer<ZED_SpotLight> spotLights;
int  numberSpotLights;
#endif

#if defined(ZED_POINT_LIGHT_DECLARATION)
StructuredBuffer<ZED_PointLight> pointLights;
int  numberPointLights;
#endif

float FallOff(float dist, float inverseRange, float coeff) {
	return lerp( 1.0, ( 1.0 - pow( dist * inverseRange * inverseRange, coeff ) ), 1 );
}
#define ZED_WORLD_DIR(index) float3 worldDirection : TEXCOORD##index;
#define ZED_TRANSFER_WORLD_DIR(o) o.worldDirection = mul(unity_ObjectToWorld, v.vertex).xyz - _WorldSpaceCameraPos;
#define GET_XYZ(o, z, world) world = (o.worldDirection/ o.pos.w) * z + _WorldSpaceCameraPos;


float4 LightCustom(SurfaceOutputStandard o, float3 worldViewDir, UnityGI gi) {
	float specularStrength = 0.5f;

	float diffuse = max(dot(o.Normal, gi.light.dir), 0.0);
	float3 reflectDir = reflect(-gi.light.dir, o.Normal);
	float spec = pow(max(dot(worldViewDir, reflectDir), 0.0), 32);
	float3 specular = specularStrength * spec * gi.light.color;
	float4 c = float4(gi.light.color*(diffuse + specular),1);
	return c;
}

half4 BRDF3_Unity_PBS2(half3 diffColor, half3 specColor, half oneMinusReflectivity, half smoothness,
	half3 normal, half3 viewDir,
	UnityLight light, UnityIndirect gi)
{
	//normal = float3(0, 1, 0);
	half3 reflDir = reflect(viewDir, normal);

	half nl = saturate(dot(normal, light.dir));
	half nv = saturate(dot(normal, viewDir));

	// Vectorize Pow4 to save instructions
	half2 rlPow4AndFresnelTerm = Pow4(half2(dot(reflDir, light.dir), 1 - nv));  // use R.L instead of N.H to save couple of instructions
	half rlPow4 = rlPow4AndFresnelTerm.x; // power exponent must match kHorizontalWarpExp in NHxRoughness() function in GeneratedTextures.cpp
	half fresnelTerm = rlPow4AndFresnelTerm.y;

	half grazingTerm = saturate(smoothness + (1 - oneMinusReflectivity));

	half3 color = BRDF3_Direct(diffColor, specColor, rlPow4, smoothness);

	color *= light.color*nl;

	color += BRDF3_Indirect(diffColor, specColor, gi, grazingTerm, fresnelTerm);
	//if (nl <= 0) return float4(1, 0, 0, 1);
	return half4(color, 1);
}
// Create a point light or spot light to be used per Unity
UnityLight CreateLight (float3 pos, float4 color, float3 worldPos) {

	UnityLight light;
	light.dir = normalize(pos);
	light.color = color;
	return light;
}
#if defined(ZED_SPOT_LIGHT_DECLARATION) || defined(ZED_POINT_LIGHT_DECLARATION)
half4 computeLighting(fixed2 uv, float3 worldPos, float alpha) {
	fixed3 worldViewDir = normalize(UnityWorldSpaceViewDir(worldPos));
  #ifdef UNITY_COMPILER_HLSL
  SurfaceOutputStandard o = (SurfaceOutputStandard)0;
  #else
  SurfaceOutputStandard o;
  #endif
  o.Albedo = 1.0;
  o.Emission = 0.0;
  o.Alpha = 1.0;
  o.Occlusion = 1.0;
  o.Metallic = 0.1;
  o.Normal = applyQuatToVec3(_CameraRotationQuat, float3(tex2D(_NormalsTex, uv).rgb));
  float4 c = 0;
 // Setup lighting environment
  UnityGI gi;
  UNITY_INITIALIZE_OUTPUT(UnityGI, gi);
  gi.indirect.diffuse = 0;
  gi.indirect.specular = 0;

  int indexPointLights = 0;
#if defined(ZED_POINT_LIGHT_DECLARATION)
  UNITY_LOOP
  for(indexPointLights = 0; indexPointLights < numberPointLights; indexPointLights++) {
		float3 lightVec = pointLights[indexPointLights].position - worldPos;
		float att = FallOff(dot(lightVec, lightVec), 1/pointLights[indexPointLights].range, 0.04);
		float v = dot(lightVec, float3(o.Normal.x, o.Normal.y, o.Normal.z));

		//Remove light from backward
		//UNITY_BRANCH
		if(v <= 0.0) {
			continue;
		}
		UnityLight p = CreateLight(lightVec, pointLights[indexPointLights].color*att*alpha, worldPos);
		//o.Normal = o.Normal*0.5 + 0.5;

		gi.light = p;
		c += LightingStandard(o, worldViewDir, gi);
		c.a = 1.0;	
  }
#endif
  c.a = 1.0;


#if defined(ZED_SPOT_LIGHT_DECLARATION)
   int indexSpotLights = 0;
   UNITY_LOOP
  for(indexSpotLights = 0; indexSpotLights < numberSpotLights; indexSpotLights++) {
		float3 lightVec = spotLights[indexSpotLights].position - worldPos;
		float att = FallOff(dot(lightVec, lightVec), spotLights[indexSpotLights].params.z, 0.03);
		
		
		float3 dirSpotToWorld = -lightVec;
		float dotDirectionWorld = dot(normalize(dirSpotToWorld), spotLights[indexSpotLights].direction.xyz);
		float angleWorld = degrees(acos(dotDirectionWorld));
		float angleMax = spotLights[indexSpotLights].params.x/2.0;

		UNITY_BRANCH
		if(dotDirectionWorld < 0 || dotDirectionWorld < spotLights[indexSpotLights].direction.w) {
			continue;

		}else {
		float angleP = angleMax*(1 -  spotLights[indexSpotLights].params.w);
		UNITY_BRANCH
		  if(angleP < angleWorld && angleWorld < angleMax) {
		  		 att *= (angleMax - angleWorld) /(angleMax - angleP);
		  }
		  
		}
		att = saturate(att);
		UnityLight p = CreateLight(lightVec, (spotLights[indexSpotLights].color)*att*alpha, worldPos);
		
		gi.light = p;
		c += LightingStandard (o, worldViewDir, gi);
		c.a = 1.0;
  }
#endif
  return c;
}
#endif

#endif