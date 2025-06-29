#version 130

uniform sampler2D uTexColor;
uniform sampler2D uTexNormal;

// lights in eye space
uniform vec3 uLight;
uniform vec3 uLight2;

in vec2 vTexCoord;
in mat3 vNTMat;
in vec3 vEyePos;

out vec4 fragColor;

void main() {
  // TODO: replace the following line with loading of normal from uTexNormal
  //       transforming to eye space, and normalizing
  vec4 normal4 = texture2D(uTexNormal, vTexCoord);
  vec3 normal = vNTMat * vec3(2*normal4.x - 1, 2*normal4.y - 1, 2*normal4.z - 1);
  normal = normalize(normal);

  vec3 viewDir = normalize(-vEyePos);
  vec3 lightDir = normalize(uLight - vEyePos);
  vec3 lightDir2 = normalize(uLight2 - vEyePos);

  float nDotL = dot(normal, lightDir);
  vec3 reflection = normalize( 2.0 * normal * nDotL - lightDir);
  float rDotV = max(0.0, dot(reflection, viewDir));
  float specular = pow(rDotV, 32.0);
  float diffuse = max(nDotL, 0.0);

  nDotL = dot(normal, lightDir2);
  reflection = normalize( 2.0 * normal * nDotL - lightDir2);
  rDotV = max(0.0, dot(reflection, viewDir));
  specular += pow(rDotV, 32.0);
  diffuse += max(nDotL, 0.0);

  vec3 color = texture(uTexColor, vTexCoord).xyz * diffuse + specular * vec3(0.6, 0.6, 0.6);

  fragColor = vec4(color, 1);
}
