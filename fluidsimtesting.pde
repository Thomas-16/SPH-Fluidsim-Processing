import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.stream.IntStream;

PVector[] positions;
PVector[] velocities;
float[] densities;
float[] nearDensities;
PVector[] predictedPositions;

int gridCellsX;
int gridCellsY;

Entry[] spatialLookup;
int[] startIndices;
int maxCellKey;

boolean frameBlending = true;
int bgTransparency = 10;

float particleSize = 0.017f;  // In simulation units
float particleSpacing = 0.013f;
int numParticles = 3200;

float collisionDamping = 0.1f;
float gravity = 10;
float smoothingRadius = 0.35f;  // In simulation units
float mass = 1;

float targetDensity = 160f;
float pressureMultiplier = 65f;
float nearPressureMultiplier = 14f;

float viscosityStrength = 0.8f;

float interactionRadius = 0.5f;
float interactionStrength = 130f;

float maxSpeed = 3f;
float minDistance = 0.01f;
float maxForce = 600f;

int lastTime = 0;


// Simulation space dimensions
float simWidth;
float simHeight;

void setup() {
  frameRate(60);
  
  // Experiemented with P2D renderer but it seems to be less efficient for some reason
  size(1280, 820);
  
  background(0);
  
  simWidth = 4.5f * (width/ (float) height);
  simHeight = 4.5f;
  
  // Calculate grid dimensions
  gridCellsX = (int)ceil(simWidth / smoothingRadius);
  gridCellsY = (int)ceil(simHeight / smoothingRadius);
  
  // Init kernel scaling factors
  Poly6ScalingFactor = PI * pow(smoothingRadius, 8);
  SpikyPow3ScalingFactor = 10f / (PI * pow(smoothingRadius, 5));
  SpikyPow2ScalingFactor = 6f / (PI * pow(smoothingRadius, 4));
  SpikyPow3DerivativeScalingFactor = 30f / (pow(smoothingRadius, 5) * PI);
  SpikyPow2DerivativeScalingFactor = 12f / (pow(smoothingRadius, 4) * PI);
  
  positions = new PVector[numParticles];
  velocities = new PVector[numParticles];
  densities = new float[numParticles];
  nearDensities = new float[numParticles];
  predictedPositions = new PVector[numParticles];
  
  for(int i = 0; i < numParticles; i++) {
    positions[i] = new PVector(random(simWidth), random(simHeight));
    velocities[i] = new PVector(0, 0);
    predictedPositions[i] = positions[i].copy();
  }
  updateSpatialLookup(positions, smoothingRadius);
  updateDensities(positions);
  
  setupParticles();
  
  lastTime = millis();
}

void setupParticles() {
  int particlesPerRow = (int) sqrt(numParticles);
  int particlesPerCol = (numParticles - 1) / particlesPerRow + 1;
  float spacing = particleSize * 2f + particleSpacing;

  for(int i = 0; i < numParticles; i++) {
    float x = simWidth/2f + (i % particlesPerRow - particlesPerRow / 2f + 0.5f) * spacing;
    float y = simHeight/2f + (i / particlesPerRow - particlesPerCol / 2f + 0.5f) * spacing;
    positions[i] = new PVector(x, y);
    velocities[i] = new PVector(0, 0);
    predictedPositions[i] = positions[i].copy();
  }
  
  updateSpatialLookup(positions, smoothingRadius);
  updateDensities(positions);
  
}

void draw() {
  float deltaTime = min((millis() - lastTime) / 1000.0f, 1.0f/30.0f);
  
  if(frameBlending) {
    fill(0, bgTransparency);
    noStroke();
    rect(0, 0, width, height);
  } else {
    background(0);
  }
  
  
  noStroke();
  fill(#00aaf2);
  
  // Get mouse position in simulation space
  float mouseSimX = mouseX * simWidth / width;
  float mouseSimY = mouseY * simHeight / height;
  boolean isInteracting = mousePressed;
  
  final float currentStrength = mousePressed ? 
    ((mouseButton == LEFT) ? interactionStrength : -interactionStrength) : 0;
  
  // Pre-calculate gravity acceleration
  float gravityDelta = gravity * deltaTime;
  
  // Apply forces and predict positions
  for(int i = 0; i < numParticles; i++) {
    boolean affectedByInteraction = false;
    if (isInteracting) {
      float dx = mouseSimX - positions[i].x;
      float dy = mouseSimY - positions[i].y;
      float distToMouseSq = dx*dx + dy*dy;
      affectedByInteraction = (distToMouseSq < interactionRadius * interactionRadius);
    }
    
    // Apply gravity directly without creating new PVector
    if (!affectedByInteraction) {
      velocities[i].y += gravityDelta;
    }
    
    // Apply interaction force if mouse is pressed
    if (isInteracting && affectedByInteraction) {
      PVector intForce = interactionForce(mouseSimX, mouseSimY, interactionRadius, currentStrength, i);
      velocities[i].x += intForce.x * deltaTime;
      velocities[i].y += intForce.y * deltaTime;
    }
    
    // Predict next positions without creating new objects
    predictedPositions[i].x = positions[i].x + velocities[i].x * (1.0f / 60.0f);
    predictedPositions[i].y = positions[i].y + velocities[i].y * (1.0f / 60.0f);
  }
  
  // Update the spatial grid lookups
  updateSpatialLookup(predictedPositions, smoothingRadius);
  
  // update densities
  updateDensities(predictedPositions);
  
  // Calculate forces in parallel
  IntStream.range(0, numParticles).parallel().forEach(i -> {
    PVector pressureForce = calculatePressureForce(predictedPositions, i);
    PVector viscosityForce = calculateViscosityForce(predictedPositions, i);
    
    PVector pressureAccel = PVector.div(pressureForce, densities[i]);
    velocities[i].add(PVector.mult(pressureAccel, deltaTime));
    velocities[i].add(PVector.mult(viscosityForce, deltaTime));
  });
  
  // Update positions
  for(int i = 0; i < numParticles; i++) {
    positions[i].add(PVector.mult(velocities[i], deltaTime));
    resolveCollisions(i);
  }
  
  
  // Draw interaction area indicator
  if (mousePressed) {
    noFill();
    stroke(255, 100);
    strokeWeight(2);
    float mouseScreenX = mouseSimX * width / simWidth;
    float mouseScreenY = mouseSimY * height / simHeight;
    float screenRadius = interactionRadius * (height / simHeight);
    circle(mouseScreenX, mouseScreenY, screenRadius * 2);
  }
  
  // Draw particles
  for (int i = 0; i < positions.length; i++) {
    float speed = velocities[i].mag();
    
    color particleColor = calculateParticleColor(speed);
    fill(particleColor);
    noStroke();
    
    PVector screenPos = simToScreen(positions[i]);
    float screenSize = particleSize * (height / simHeight);
    circle(screenPos.x, screenPos.y, screenSize * 2);
  }
  
  lastTime = millis();
  
  //println(frameRate);
}


PVector calculateViscosityForce(PVector[] posArr, int particleIndex) {
  float forceX = 0, forceY = 0;
  PVector particlePos = posArr[particleIndex];
  
  int centerX = (int)(particlePos.x / smoothingRadius);
  int centerY = (int)(particlePos.y / smoothingRadius);
  
  for (int[] offset : cellOffsets) {
    int cellX = centerX + offset[0];
    int cellY = centerY + offset[1];
    
    if (cellX < 0 || cellY < 0 || cellX >= gridCellsX || cellY >= gridCellsY) continue;
    
    long cellHash = hashCell(cellX, cellY);
    long key = getKeyFromHash(cellHash, numParticles);
    
    int cellStartIndex = startIndices[(int)key];
    if (cellStartIndex == -1) continue;
    
    for (int i = cellStartIndex; i < spatialLookup.length; i++) {
      if (spatialLookup[i].cellKey != key) break;
      
      int neighborIndex = spatialLookup[i].particleIndex;
      if (neighborIndex == particleIndex) continue;
      
      float dx = posArr[neighborIndex].x - particlePos.x;
      float dy = posArr[neighborIndex].y - particlePos.y;
      float dist = sqrt(dx*dx + dy*dy);
      
      if (dist > 0 && dist <= minDistance) {
        dist = minDistance;
      }
      
      if (dist < smoothingRadius && dist > 0) {
        float influence = viscosityKernel(dist, smoothingRadius);
        float factor = viscosityStrength * influence * mass / densities[neighborIndex];
        
        forceX += (velocities[neighborIndex].x - velocities[particleIndex].x) * factor;
        forceY += (velocities[neighborIndex].y - velocities[particleIndex].y) * factor;
      }
    }
  }
  
  return new PVector(forceX, forceY);
}

PVector calculatePressureForce(PVector[] posArr, int particleIndex) {
  float forceX = 0, forceY = 0;  // Use primitives instead of PVector
  PVector particlePos = posArr[particleIndex];
  
  int centerX = (int)(particlePos.x / smoothingRadius);
  int centerY = (int)(particlePos.y / smoothingRadius);
  
  for (int[] offset : cellOffsets) {
    int cellX = centerX + offset[0];
    int cellY = centerY + offset[1];
    
    if (cellX < 0 || cellY < 0 || cellX >= gridCellsX || cellY >= gridCellsY) continue;
    
    long cellHash = hashCell(cellX, cellY);
    long key = getKeyFromHash(cellHash, numParticles);
    
    int cellStartIndex = startIndices[(int)key];
    if (cellStartIndex == -1) continue;
    
    for (int i = cellStartIndex; i < spatialLookup.length; i++) {
      if (spatialLookup[i].cellKey != key) break;
      
      int neighborIndex = spatialLookup[i].particleIndex;
      if (neighborIndex == particleIndex) continue;
      
      float dx = posArr[neighborIndex].x - particlePos.x;
      float dy = posArr[neighborIndex].y - particlePos.y;
      float distSq = dx*dx + dy*dy;
      
      if (distSq < smoothingRadius * smoothingRadius && distSq > 0) {
        float dist = sqrt(distSq);
        dist = max(dist, minDistance);
        
        // Calculate direction without creating new PVector
        float dirX = dx / dist;
        float dirY = dy / dist;
        
        float slope = densityDerivative(dist, smoothingRadius);
        float nearDensitySlope = nearDensityDerivative(dist, smoothingRadius);
        slope = constrain(slope, -1000, 1000);
        
        float density = densities[neighborIndex];
        float nearDensity = nearDensities[neighborIndex];
        float sharedPressure = calculateSharedPressure(density, densities[particleIndex]);
        float sharedNearPressure = calculateSharedNearPressure(nearDensity, nearDensities[particleIndex]);
        
        float forceMagnitude = (sharedPressure * slope + sharedNearPressure * nearDensitySlope) * mass / density;
        forceX += dirX * forceMagnitude;
        forceY += dirY * forceMagnitude;
      }
    }
  }
  
  return new PVector(forceX, forceY);  // Only create one PVector at the end
}

float calculateSharedPressure(float densityA, float densityB) {
  float pressureA = densityToPressure(densityA);
  float pressureB = densityToPressure(densityB);
  return (pressureA + pressureB) / 2f;
}
float calculateSharedNearPressure(float nearDensityA, float nearDensityB) {
    float nearPressureA = densityToNearPressure(nearDensityA);
    float nearPressureB = densityToNearPressure(nearDensityB);
    return (nearPressureA + nearPressureB) / 2f;
}


void updateDensities(PVector[] posArr) {
  IntStream.range(0, numParticles).parallel().forEach(i -> {
    float density = 0f;
    float nearDensity = 0f;
    PVector samplePoint = posArr[i];
    
    int centerX = (int)(samplePoint.x / smoothingRadius);
    int centerY = (int)(samplePoint.y / smoothingRadius);
    
    for (int[] offset : cellOffsets) {
      int cellX = centerX + offset[0];
      int cellY = centerY + offset[1];
      
      if (cellX < 0 || cellY < 0 || cellX >= gridCellsX || cellY >= gridCellsY) continue;
      
      long cellHash = hashCell(cellX, cellY);  // Use int version
      long key = getKeyFromHash(cellHash, numParticles);
      
      int cellStartIndex = startIndices[(int)key];
      if (cellStartIndex == -1) continue;
      
      for (int j = cellStartIndex; j < spatialLookup.length; j++) {
        if (spatialLookup[j].cellKey != key) break;
        
        int particleIndex = spatialLookup[j].particleIndex;
        
        float dx = posArr[particleIndex].x - samplePoint.x;
        float dy = posArr[particleIndex].y - samplePoint.y;
        float distSq = dx*dx + dy*dy;
        
        if (distSq < smoothingRadius * smoothingRadius) {
          float dist = sqrt(distSq);
          float influence = densityKernel(dist, smoothingRadius);
          density += mass * influence;
          nearDensity += mass * nearDensityKernel(dist, smoothingRadius);
        }
      }
    }
    
    densities[i] = density;
    nearDensities[i] = nearDensity;
  });
}

PVector interactionForce(float inputX, float inputY, float radius, float strength, int particleIndex) {
  float offsetX = inputX - positions[particleIndex].x;
  float offsetY = inputY - positions[particleIndex].y;
  float sqrDist = offsetX*offsetX + offsetY*offsetY;
  
  if(sqrDist >= radius * radius) {
    return new PVector(0, 0);
  }
  
  float dist = sqrt(sqrDist);
  float dirX = dist > 0 ? offsetX / dist : 0;
  float dirY = dist > 0 ? offsetY / dist : 0;
  
  float forceX = 0, forceY = 0;
  
  if(strength > 0) {
    float centreT = 1 - dist / radius;
    forceX = (dirX * strength - velocities[particleIndex].x) * centreT;
    forceY = (dirY * strength - velocities[particleIndex].y) * centreT;
  } else {
    float edgeDistance = radius - dist;
    float edgeFalloff = 1 - (edgeDistance / radius);
    
    forceX = -velocities[particleIndex].x * 0.8f;
    forceY = -velocities[particleIndex].y * 0.8f;
    
    if(edgeFalloff > 0.5f) {
      forceX += dirX * strength * edgeFalloff;
      forceY += dirY * strength * edgeFalloff;
    }
  }
  
  return new PVector(forceX, forceY);
}

float densityToPressure(float density) {
  float densityError = density - targetDensity;
  float pressure = densityError * pressureMultiplier;
  return pressure;
}
float densityToNearPressure(float nearDensity) {
  return nearPressureMultiplier * nearDensity;
}

void resolveCollisions(int particleIndex) {
  PVector position = positions[particleIndex];
  PVector velocity = velocities[particleIndex];
  
  // Boundaries in simulation space
  if(position.x - particleSize < 0) {
    position.x = particleSize;
    velocity.x *= -collisionDamping;
  }
  if(position.x + particleSize > simWidth) {
    position.x = simWidth - particleSize;
    velocity.x *= -collisionDamping;
  }
  if(position.y - particleSize < 0) {
    position.y = particleSize;
    velocity.y *= -collisionDamping;
  }
  if(position.y + particleSize > simHeight) {
    position.y = simHeight - particleSize;
    velocity.y *= -collisionDamping;
  }
}

color calculateParticleColor(float speed) {
  float t = constrain(speed / maxSpeed, 0, 1);
  
  if (t < 0.25f) {
    // Blue to Cyan
    return lerpColor(color(0, 100, 255), color(0, 255, 255), t * 4);
  } else if (t < 0.5f) {
    // Cyan to Green
    return lerpColor(color(0, 255, 255), color(0, 255, 0), (t - 0.25) * 4);
  } else if (t < 0.75f) {
    // Green to Yellow
    return lerpColor(color(0, 255, 0), color(255, 255, 0), (t - 0.5) * 4);
  } else {
    // Yellow to Red
    return lerpColor(color(255, 255, 0), color(255, 50, 0), (t - 0.75) * 4);
  }
}

PVector indexTo2D(int index, int width) {
  return new PVector(index % width, index / width);
}

PVector screenToSim(PVector screenPos) {
  float x = screenPos.x * simWidth / width;
  float y = screenPos.y * simHeight / height;
  return new PVector(x, y);
}

PVector simToScreen(PVector simPos) {
  float x = simPos.x * width / simWidth;
  float y = simPos.y * height / simHeight;
  return new PVector(x, y);
}
PVector getRandomDir() {
  float angle = random(TWO_PI);
  return new PVector(cos(angle), sin(angle));
}
