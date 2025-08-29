# Nav3D - 3D Navigation Plugin for Unreal Engine 5

A comprehensive 3D navigation system for Unreal Engine 5, providing true volumetric pathfinding using Sparse Voxel Octrees. This plugin enables AI agents to navigate freely in 3D space with advanced pathfinding algorithms including A*, Theta*, and Lazy Theta*.

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Installation & Setup](#installation--setup)
- [Core Systems](#core-systems)
- [Usage Methods](#usage-methods)
- [Advanced Features](#advanced-features)
- [Performance & Optimization](#performance--optimization)
- [Debugging & Visualization](#debugging--visualization)
- [Roadmap & Limitations](#roadmap--limitations)

## Overview

Nav3D extends Unreal Engine's navigation system to support true 3D movement, perfect for:
- Flying units (aircraft, drones, birds)
- Swimming creatures
- Zero-gravity environments
- Complex 3D level geometry
- Multi-level architectural spaces

The system implements Daniel Brewer's "3D Flight Navigation Using Sparse Voxel Octrees" from Game AI Pro 3, providing both advanced pathfinding and sophisticated tactical reasoning capabilities for AI decision-making.

## Key Features

### ✈️ **True 3D Navigation**
- Full volumetric pathfinding without height restrictions
- Sparse Voxel Octree (SVO) data structure for efficient 3D space representation
- Multi-scale voxel resolution based on agent size

### 🧠 **Advanced Pathfinding Algorithms**
- **A-Star**: Fast, grid-aligned paths
- **Theta-Star**: Line-of-sight optimized paths with shortcuts
- **Lazy Theta-Star**: Performance-optimized variant (default)

### 🎯 **Advanced Tactical Reasoning System**
- **Region-based spatial analysis** with automatic free space identification
- **Visibility analysis** between regions using sample-based line-of-sight testing
- **Adjacency graphs** for tactical movement planning
- **Best position finding** with configurable visibility/distance/region size preferences
- **Future expansion ready** for heat maps, agent density analysis, and advanced tactical queries

### 🔧 **Advanced Cost & Smoothing Systems**
- Dynamic cost calculation (Distance vs Fixed cost models)
- Multiple heuristic functions (Euclidean, Manhattan)
- Node size compensation for hierarchical optimization
- Configurable path smoothing with CatmullRom interpolation

### ⚡ **Seamless UE5 Integration**
- Drop-in replacement for standard navigation
- Works with existing MoveTo behavior tree nodes
- Automatic pathfinding query routing
- No code changes required for basic usage

### 🚀 **Performance Optimized**
- Asynchronous path generation and data building
- Multi-threaded octree traversal
- Early line-of-sight optimization
- Dynamic navigation data updates

## Installation & Setup

### 1. Project Configuration

Add the plugin to your project and enable it in the Plugin Manager.

### 2. Navigation System Configuration

Open **Project Settings** → **Game** → **Navigation System**:

1. Expand the **Supported Agents** array
2. Add a new agent configuration:
   - **Nav Data Class**: `Nav3DData`
   - **Preferred Nav Data**: `Nav3DData`
   - **Agent Radius**: Critical setting - determines minimum voxel size (voxel = 2 × radius)
   - **Agent Height**: Used for collision detection

**Important**: You can create multiple agent types with different radii for varied unit scales (e.g., small drones vs large aircraft).

### 3. Level Setup

1. **Add Nav3D Bounds Volume**: Place a `Nav3DBoundsVolume` actor in your level
2. **Size the Volume**: Scale to encompass your navigation area
3. **Automatic Generation**: The system automatically creates `Nav3DData` actors

**Note**: The actual octree bounds may differ from your volume due to power-of-2 constraints required by the octree structure.

### 4. Character Setup

#### Movement Component Configuration
On your flying Character's Movement Component:
- **Nav Movement** → **Preferred Nav Data**: `Nav3DData`

#### AI Controller Setup
1. Create a **Navigation Query Filter** Blueprint based on `Nav3DQueryFilter`
2. Configure pathfinding options (see [Query Filter Configuration](#query-filter-configuration))
3. Set as **Default Nav Filter Class** in your AI Controller

### 5. Query Filter Configuration

Create a Blueprint from `Nav3DQueryFilter` to define pathfinding behavior:

#### Algorithm Selection
- **A-Star**: Fastest, jaggy paths along voxel centers
- **Theta-Star**: Most accurate with line-of-sight shortcuts
- **Lazy Theta-Star**: Recommended balance of speed and quality

#### Cost Calculation
- **Traversal Cost Calculator**:
  - `Distance`: Standard distance-based cost
  - `Fixed`: Equal cost regardless of voxel size (favors larger voxels)
- **Heuristic Calculator**:
  - `Euclidean`: Direct distance to goal
  - `Manhattan`: Taxicab geometry distance
- **Heuristic Scale**: Bias toward goal-oriented exploration
- **Use Node Size Compensation**: Makes larger voxels cheaper to traverse

#### Path Smoothing
- **Enable Smoothing**: Apply CatmullRom algorithm
- **Smoothing Subdivisions**: Control smoothness level

## Core Systems

### Sparse Voxel Octree (SVO)

The foundation of Nav3D is a hierarchical 3D grid system:

```
Layer 3: Largest voxels (coarse pathfinding)
Layer 2: Medium voxels  
Layer 1: Small voxels
Layer 0: Leaf nodes (finest detail)
```

**Benefits**:
- **Memory Efficient**: Only stores occupied space
- **Multi-Scale**: Large voxels for distance, small for precision  
- **Fast Queries**: Logarithmic search times
- **Dynamic Updates**: Efficient partial rebuilds

### Tactical Reasoning System

**The Major Innovation**: Beyond pathfinding, Nav3D provides extensive tactical analysis of 3D space:

#### Region Analysis
```cpp
// Automatic identification of navigable regions from free voxel space
// Converts raw octree data into meaningful tactical zones
struct FNav3DRegion {
    int32 Id;                           // Unique region identifier
    FBox Bounds;                        // 3D bounding box
    TArray<int32> AdjacentRegionIds;    // Connected regions
    TSet<int32> VisibilitySet;          // Regions visible from this region
    int32 LayerIndex;                   // Octree layer this region exists in
};
```

#### Visibility Analysis
```cpp
// Sample-based line-of-sight analysis between all region pairs
// Configurable sample density based on region size
// Multi-threaded visibility computation with occlusion testing
FNav3DTacticalData::CheckVisibility(ViewerRegionId, TargetRegionId, VisibilityType);
```

#### Best Position Finding
```cpp
// Advanced tactical position queries with multiple criteria
bool FindBestLocation(
    const FVector& StartPosition,
    const TArray<FVector>& ObserverPositions,
    ETacticalVisibility Visibility,      // TargetVisible, MutuallyVisible, TargetOccluded, MutuallyOccluded
    ETacticalDistance DistancePreference, // Closest, Median, Furthest  
    ETacticalRegion RegionPreference,    // Smallest, Medium, Largest
    bool bForceNewRegion,               // Must be different from start region
    TArray<FPositionCandidate>& OutCandidates
);
```

**Future Expansion Roadmap**:
- **Heat Map Analysis**: Agent density tracking per region
- **Traffic Analysis**: Popular paths and chokepoint identification  
- **Cover Analysis**: Advanced tactical positioning for combat scenarios
- **Dynamic Threat Assessment**: Real-time tactical value updates
- **Multi-Agent Coordination**: Region-based formation and tactical movement

### Pathfinding Algorithms

#### A-Star (Classic)
```cpp
// Fast grid-based pathfinding
// Pros: Very fast, predictable
// Cons: Jaggy paths, follows voxel centers
```

#### Theta-Star (Line-of-Sight Optimized) 
```cpp
// Adds line-of-sight checks to A* for direct paths
// Pros: Smooth, natural paths with shortcuts
// Cons: More expensive due to LoS calculations
```

#### Lazy Theta-Star (Recommended)
```cpp
// Deferred line-of-sight optimization
// Pros: Nearly as smooth as Theta*, much faster
// Cons: Slightly less optimal than full Theta*
```

### Octree Raycaster System

The raycaster performs line-of-sight checks crucial for:
1. **Pre-pathfinding optimization**: Direct line check before full pathfinding
2. **Theta*/Lazy Theta* algorithms**: Shortcut validation

#### Raycaster Types

**Octree Traversal (Recommended)**
- Pure mathematical algorithm based on academic research
- Fastest performance with exact voxel precision
- No physics engine overhead

**Physics Ray Cast**
- Uses UE5's built-in ray casting
- Good for complex collision scenarios
- Slower than octree traversal

**Physics Sphere Cast**
- Volumetric collision detection
- Best for agents needing clearance validation
- Slowest but most thorough

### Tactical Cost System

The plugin offers sophisticated cost calculation for strategic AI:

#### Distance-Based Costing
```cpp
// Standard: Cost = actual distance between points
// Use for: Realistic movement costs
float Cost = FVector::Dist(From, To);
```

#### Fixed Costing
```cpp  
// Strategic: Equal cost regardless of voxel size
// Use for: Encouraging use of highways/large open areas
float Cost = 1.0f; // Constant
```

#### Node Size Compensation
```cpp
// Hierarchical bias toward larger voxels
// Cheaper to traverse big open areas vs tight spaces
AdjustedCost = BaseCost * VoxelSizeMultiplier;
```

## Usage Methods

### Method 1: Behavior Tree Integration (Easiest)

**No code changes required** - use standard UE5 navigation nodes:

1. **Move To** task in Behavior Tree
2. Set **Filter Class** to your custom `Nav3DQueryFilter`
3. System automatically routes to 3D pathfinding

```cpp
// Your existing behavior tree setups work unchanged
// The plugin intercepts navigation queries automatically
```

### Method 2: Direct API Calls (Advanced)

For custom movement systems or direct control:

#### Synchronous Pathfinding
```cpp
#include "Pathfinding/Nav3DPathFinder.h"

// Get navigation components
UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
const FNavAgentProperties& AgentProps = MovementComponent->GetNavAgentPropertiesRef();
const ANavigationData* NavigationData = NavSys->GetNavDataForProps(AgentProps);
const ANav3DData* Nav3DData = Cast<ANav3DData>(NavigationData);

// Create query filter
FSharedConstNavQueryFilter QueryFilter;
if (QueryFilterClass) {
    QueryFilter = UNavigationQueryFilter::GetQueryFilter(*Nav3DData, nullptr, QueryFilterClass);
} else {
    QueryFilter = Nav3DData->GetDefaultQueryFilter();
}

// Execute pathfinding
FNav3DPath ResultPath;
ENavigationQueryResult::Type Result = FNav3DPathFinder::GetPath(
    ResultPath,
    *Nav3DData,
    StartLocation,
    EndLocation,
    AgentProps,
    QueryFilter
);

// Use results
if (Result == ENavigationQueryResult::Success) {
    const TArray<FNavPathPoint>& PathPoints = ResultPath.GetPathPoints();
    const TArray<float>& PathCosts = ResultPath.GetPathPointCosts();
    
    // Implement your movement logic here
    for (const FNavPathPoint& Point : PathPoints) {
        // Move to Point.Location
    }
}
```

#### Asynchronous Pathfinding
```cpp
// For non-blocking pathfinding in complex scenarios
// The plugin includes UNav3DAsyncPathfindingTask
// This executes FNav3DPathFinder::GetPath on background threads
```

### Method 3: Tactical Reasoning Integration

Use the advanced tactical analysis system for AI decision-making:

```cpp
// Find tactically optimal positions based on visibility and positioning criteria
bool bSuccess = Nav3DData->FindBestLocation(
    StartPosition,
    ObserverPositions,
    OutCandidates,
    ETacticalVisibility::TargetOccluded,  // Hide from observers
    ETacticalDistance::Furthest,          // Prefer distant positions
    ETacticalRegion::Largest,             // Prefer large open areas
    true,                                 // Force different region from start
    true                                  // Use raycasting validation
);

if (bSuccess) {
    // OutCandidates contains scored positions sorted by tactical value
    for (const FPositionCandidate& Candidate : OutCandidates) {
        // Use Candidate.Position, Candidate.Score, Candidate.RegionId
        // for tactical AI decision making
    }
}
```

### Method 4: Custom Movement Integration

Override key functions for specialized behavior:

```cpp
// In your Pawn class
FVector AMyFlyingPawn::GetNavAgentLocation() const override
{
    // Return appropriate location for pathfinding start
    // For flying units: center of mass, not feet
    return GetActorLocation() + FVector(0, 0, GetCapsuleComponent()->GetScaledCapsuleHalfHeight());
}

FVector AMyFlyingPawn::GetMoveGoalOffset(const AActor* MovingActor) const override  
{
    // Offset destination above ground for aerial navigation
    if (const ACharacter* Character = Cast<ACharacter>(MovingActor)) {
        if (Character->GetMovementComponent()->IsFlying()) {
            return FVector(0, 0, 200.0f); // Hover 200 units above target
        }
    }
    return FVector::ZeroVector;
}
```

## Advanced Features

### Multi-Volume Navigation

**Current Status**: ✅ **Fully Implemented** - Automatic cross-volume pathfinding

The plugin includes sophisticated multi-volume pathfinding that automatically handles navigation between separate navigation volumes:

```cpp
// Automatic cross-volume pathfinding with intelligent segment creation
// System finds intersection points and creates hybrid paths
// Combines 3D pathfinding within volumes with direct movement between volumes
```

**How Cross-Volume Pathfinding Works**:

1. **Path Sanitization**: `SanitizePath()` analyzes start/end points across all volumes
2. **Intersection Detection**: Uses `FNav3DUtils::RayBoxIntersection()` to find volume entry/exit points
3. **Segment Creation**: Builds composite paths with multiple segments:
   - **Navigation Segments**: 3D pathfinding within volumes using octree data
   - **Direct Movement Segments**: Straight-line movement between volumes
4. **Automatic Point Adjustment**: Finds closest valid points when start/end is outside volumes

**Supported Scenarios**:
- Start in Volume A, end in Volume B (automatic bridging)
- Start outside any volume, end in volume (finds intersection point)
- End outside any volume (finds closest valid endpoint)
- Complex multi-volume traversal with intermediate volumes

**Example Use Cases**:
- Building-to-building navigation in urban environments
- Cave system navigation with separate chambers
- Multi-floor buildings with distinct navigation zones
- Performance optimization for large worlds (separate high/low detail areas)

Force specific pathfinding settings within volumes:

```cpp
// On Nav3DBoundsVolume actor
VolumeNavigationQueryFilter = MySpecialQueryFilter;

// All pathfinding within this volume uses the override filter
// Useful for: No-fly zones, speed corridors, tactical areas
```

### Tactical Reasoning System

**Current Status**: ✅ **Fully Implemented** - Comprehensive tactical analysis system

Beyond pathfinding, Nav3D includes sophisticated tactical reasoning capabilities that analyze 3D space for AI decision-making:

**Key Capabilities**:
- **Region Identification**: Automatically converts octree voxel data into meaningful tactical regions
- **Adjacency Analysis**: Builds connectivity graphs between navigable regions
- **Visibility Computation**: Sample-based line-of-sight analysis between all region pairs
- **Tactical Queries**: Find optimal positions based on visibility, distance, and region size criteria
- **Multi-criteria Scoring**: Intelligent ranking of candidate positions for tactical decisions

**API Usage**:
```cpp
// Find tactically optimal positions
bool ANav3DData::FindBestLocation(
    const FVector& StartPosition,
    const TArray<FVector>& ObserverPositions,
    TArray<FPositionCandidate>& OutCandidates,
    ETacticalVisibility Visibility,      // TargetVisible, MutuallyVisible, TargetOccluded, MutuallyOccluded
    ETacticalDistance DistancePreference, // Closest, Median, Furthest
    ETacticalRegion RegionPreference,    // Smallest, Medium, Largest  
    bool bForceNewRegion,               // Must differ from start region
    bool bUseRaycasting                 // Validate with physics raycasts
);
```

**Region Data Structure**:
```cpp
struct FNav3DRegion {
    int32 Id;                           // Unique identifier
    FBox Bounds;                        // 3D bounding volume
    TArray<int32> AdjacentRegionIds;    // Connected regions for pathfinding
    TSet<int32> VisibilitySet;          // Regions visible from this region
    int32 LayerIndex;                   // Octree resolution level
};
```

**Future Tactical Expansions**:
- **Heat Map Analysis**: Track agent density and movement patterns per region
- **Chokepoint Detection**: Identify tactical bottlenecks and high-traffic areas
- **Cover Analysis**: Advanced positioning for combat and stealth scenarios
- **Multi-Agent Coordination**: Region-based formation and tactical movement planning
- **Dynamic Threat Assessment**: Real-time tactical value updates based on game state

### World Partition Support

**Current Status**: ⚠️ Experimental - not battle-tested

For large worlds using UE5's World Partition:

```cpp
// Theoretical support exists but needs extensive testing
// Custom commandlet required for proper nav data building
// See documentation's Level Streaming section for details
```

**Roadmap Priority**: High - essential for large-scale projects

### Custom Tri-Box Overlap System

**Current Status**: ✅ **Production Ready** - High-performance parallel occlusion detection

Nav3D implements a custom triangle-box overlap detection system that provides **massively parallel occlusion checking** during navigation data building, bypassing UE5's single-threaded physics system entirely:

#### Performance Architecture
```cpp
// Custom implementation similar to Recast's approach
// Triangle-box intersection using separating axis theorem
// Multi-threaded processing during octree generation
// No dependency on UE5's physics collision system
```

**Key Advantages**:
- **Parallel Processing**: Multi-threaded occlusion detection during build time
- **CPU-Only**: No physics engine overhead or single-threaded bottlenecks
- **Precision**: Exact triangle-voxel intersection mathematics
- **Scalability**: Performance scales with thread count, not physics system limitations

#### Technical Implementation
```cpp
namespace Nav3D::TriBoxOverlapUtils
{
    // Fast triangle-box intersection using separating axis theorem
    bool TriBoxOverlap(
        const FVector& BoxCenter, 
        const FVector& BoxHalfSize, 
        const FVector& TriVert0,
        const FVector& TriVert1, 
        const FVector& TriVert2
    );
    
    // Optimized cross product, dot product operations
    // Plane-box overlap testing
    // Min/max projection calculations
}
```

**Build Process Integration**:
1. **Mesh Extraction**: Gathers static mesh triangle data from overlapping components
2. **Parallel Voxelization**: Each thread processes different octree regions
3. **Triangle Testing**: `TriBoxOverlap()` performs exact intersection tests
4. **Voxel Classification**: Marks voxels as free/occupied based on triangle intersections

**Comparison to Alternatives**:

| Method | Threading | Performance | Accuracy | Memory |
|--------|-----------|-------------|----------|---------|
| **Nav3D Tri-Box** | ★★★★★ | ★★★★★ | ★★★★★ | ★★★★☆ |
| UE5 Physics Overlap | ★☆☆☆☆ | ★★☆☆☆ | ★★★★☆ | ★★★☆☆ |
| Recast Triangle Mesh | ★★★★☆ | ★★★★☆ | ★★★★★ | ★★☆☆☆ |

This approach mirrors Recast's triangle-based navigation mesh building but extends it to full 3D voxel space, providing the precision of triangle-level collision detection with the performance benefits of parallel processing.

## Performance & Optimization

### Voxel Size Strategy

```cpp
Agent Radius = 50  → Voxel Size = 100  // Fine detail, higher memory
Agent Radius = 100 → Voxel Size = 200  // Balanced
Agent Radius = 200 → Voxel Size = 400  // Coarse, fast, lower memory
```

### Algorithm Performance Comparison

| Algorithm | Speed | Path Quality | Memory | Use Case |
|-----------|-------|--------------|---------|----------|
| A-Star | ★★★★★ | ★★☆☆☆ | ★★★★☆ | Many simple agents |
| Theta-Star | ★★☆☆☆ | ★★★★★ | ★★★☆☆ | Hero units, cinematic |
| Lazy Theta-Star | ★★★★☆ | ★★★★☆ | ★★★★☆ | **Recommended default** |

### Memory Usage

Monitor navigation memory with console command:
```
CountNavMem
```

Typical memory usage:
- Small level (100m³): ~1-5 MB
- Medium level (500m³): ~10-50 MB  
- Large level (1000m³): ~50-200 MB

### Threading Architecture

```cpp
// Main Thread: Query management, result processing
// Background Thread: Pathfinding calculations, octree updates
// Async Safe: All navigation queries, dynamic updates
```

## Debugging & Visualization

### Enable Navigation Visualization

1. **Viewport** → **Show** → **Navigation** (checkbox)
2. Select `Nav3DData` actor in World Outliner
3. Check **Enable Drawing**

### Visualization Options

**Debug Draw Bounds**: White wireframe showing actual octree bounds
```cpp
// Shows the real navigation volume (may differ from your Nav3DBoundsVolume)
```

**Debug Draw Layers**: Hierarchical voxel display
```cpp
Layer 3: Large yellow cubes    // Coarse pathfinding
Layer 2: Medium yellow cubes   // Intermediate detail  
Layer 1: Small yellow cubes    // Fine detail
Layer 0: Tiny yellow cubes     // Leaf nodes
```

**Debug Draw Voxels**: Occupied vs free space
```cpp
Yellow cubes = Blocked/Occupied voxels
Green cubes  = Free/Navigable voxels  
```

**Debug Draw Active Paths**: Real-time path visualization
```cpp
// Shows all active agent paths during gameplay
// Different colors per agent
// Updates in real-time as agents move
```

### Pathfinding Test Actor

Use `Nav3DPathFinderTest` actors for algorithm testing:

1. Place two test actors in scene
2. Set one as "Other Actor" on the first
3. Configure pathfinding settings
4. Use test buttons:
   - **Auto Complete Instantly**: Immediate result
   - **Auto Complete Step by Step**: Animated algorithm visualization
   - **Auto Complete Until Next Node**: Single-step debugging

Results show performance metrics:
```cpp
Iterations: 45
Visited Nodes: 123  
Path Length: 1250.5
Path Segment Count: 8
```

### Tactical Reasoning Visualization

**Enable Tactical Debug Options**:
- **Debug Draw Regions**: Visualize identified tactical regions as colored boxes
- **Debug Draw Region IDs**: Display region identifiers for reference
- **Debug Draw Adjacency Graph**: Show connections between adjacent regions
- **Debug Draw Visibility**: Display line-of-sight relationships between regions
- **Debug Draw Best Cover**: Visualize optimal cover positions relative to observers

### Console Commands

```bash
CountNavMem          # Display navigation memory usage
ShowDebug Navigation # Toggle navigation debug overlay
```

## Roadmap & Current Status

### Current Limitations

**Single-Threaded Building**
- Octree generation uses one thread
- Could be parallelized into 8 sub-octrees
- **Priority**: Medium

**Level Streaming Support**
- Basic support exists but requires custom implementation
- No out-of-the-box World Partition integration
- Custom commandlet required for proper building
- **Priority**: High

### Roadmap Features

**Enhanced Tactical Analysis** (High Priority)
- Agent density heat maps per region
- Traffic flow analysis and chokepoint detection
- Advanced cover analysis for combat scenarios
- Multi-agent tactical coordination systems

**World Partition Integration** (High Priority)  
- Native UE5 World Partition support
- Automatic streaming volume management
- Large world optimization with tactical data preservation

### Implementation Status

**Production Ready**:
✅ Core pathfinding algorithms
✅ Single volume navigation  
✅ **Multi-volume cross-navigation with automatic bridging**
✅ **Advanced tactical reasoning with region analysis**
✅ Behavior tree integration
✅ Basic dynamic updates
✅ Ray-box intersection system for volume boundaries

**Experimental/In Development**:
⚠️ World Partition support
⚠️ Large-scale level streaming with custom commandlets
⚠️ Advanced threading optimizations (8-way parallel octree building)
⚠️ Heat map and traffic analysis features
