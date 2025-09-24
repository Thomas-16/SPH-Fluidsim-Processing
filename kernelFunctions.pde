float densityKernel(float dst, float radius)
{
  return SpikyKernelPow2(dst, radius);
}

float nearDensityKernel(float dst, float radius)
{
  return SpikyKernelPow3(dst, radius);
}

float densityDerivative(float dst, float radius)
{
  return DerivativeSpikyPow2(dst, radius);
}

float nearDensityDerivative(float dst, float radius)
{
  return DerivativeSpikyPow3(dst, radius);
}

float viscosityKernel(float dst, float radius)
{
  return SmoothingKernelPoly6(dst, radius);
}


float Poly6ScalingFactor = 1;
float SpikyPow3ScalingFactor = 1;
float SpikyPow2ScalingFactor = 1;
float SpikyPow3DerivativeScalingFactor = 1;
float SpikyPow2DerivativeScalingFactor = 1;

float SmoothingKernelPoly6(float dst, float radius)
{
  if (dst < radius)
  {
    float v = radius * radius - dst * dst;
    return v * v * v * Poly6ScalingFactor;
  }
  return 0;
}

float SpikyKernelPow3(float dst, float radius)
{
  if (dst < radius)
  {
    float v = radius - dst;
    return v * v * v * SpikyPow3ScalingFactor;
  }
  return 0;
}

float SpikyKernelPow2(float dst, float radius)
{
  if (dst < radius)
  {
    float v = radius - dst;
    return v * v * SpikyPow2ScalingFactor;
  }
  return 0;
}

float DerivativeSpikyPow3(float dst, float radius)
{
  if (dst <= radius)
  {
    float v = radius - dst;
    return -v * v * SpikyPow3DerivativeScalingFactor;
  }
  return 0;
}

float DerivativeSpikyPow2(float dst, float radius)
{
  if (dst <= radius)
  {
    float v = radius - dst;
    return -v * SpikyPow2DerivativeScalingFactor;
  }
  return 0;
}
