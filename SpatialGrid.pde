int[][] cellOffsets = {
  {-1, -1},
  {0, -1},
  {1, -1},
  {-1, 0},
  {0, 0},
  {1, 0},
  {-1, 1},
  {0, 1},
  {1, 1}
};



void updateSpatialLookup(PVector[] points, float radius) {
  spatialLookup = new Entry[points.length];
  
  maxCellKey = points.length * 2;
  startIndices = new int[maxCellKey];
  Arrays.fill(startIndices, -1);
  
  // Create spatial lookup entries
  IntStream.range(0, numParticles).parallel().forEach(i -> {
    int cellX = (int)(points[i].x / radius);
    int cellY = (int)(points[i].y / radius);
    long cellHash = hashCell(cellX, cellY);  // Use the int version
    long cellKey = getKeyFromHash(cellHash, points.length);
    spatialLookup[i] = new Entry(i, cellKey);
  });
  
  // Sort by cell key
  Arrays.sort(spatialLookup, new Comparator<Entry>() {
    public int compare(Entry a, Entry b) {
      return Long.compare(a.cellKey, b.cellKey);
    }
  });
  
  // Calculate start indices for each unique cell key
  for (int i = 0; i < points.length; i++) {
    long key = spatialLookup[i].cellKey;
    long keyPrev = (i == 0) ? Long.MAX_VALUE : spatialLookup[i - 1].cellKey;
    
    if (key != keyPrev) {
      startIndices[(int)key] = i;  // Cast to int since we know it's within array bounds
    }
  }
}


// Converts a cell's coordinate to a single number
// Hash collisions are unavoidable but whatever
long hashCell(int cellX, int cellY) {
  long a = (long)cellX * 15823;
  long b = (long)cellY * 973733;
  return a + b;
}

long getKeyFromHash(long hash, int tableLength) {
  return hash % (tableLength * 2);
}


class Entry {
  int particleIndex;
  long cellKey;
  
  Entry(int particleIndex, long cellKey) {
    this.particleIndex = particleIndex;
    this.cellKey = cellKey;
  }
}
