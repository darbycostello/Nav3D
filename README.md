# Nav3D - 3D Navigation Plugin for Unreal Engine 5

![Nav3D banner image](https://user-images.githubusercontent.com/891532/103788146-6cdfe280-5036-11eb-883f-c9bf174b4ec2.jpg)

A comprehensive 3D navigation system for Unreal Engine 5, providing true volumetric pathfinding using Sparse Voxel Octrees. This plugin enables AI agents to navigate freely in 3D space with advanced pathfinding algorithms including A*, Theta*, and Lazy Theta*.

## Overview

Nav3D extends Unreal Engine's navigation system to support true 3D movement, perfect for:
- Flying units (aircraft, drones, birds)
- Swimming creatures
- Zero-gravity environments
- Complex 3D level geometry
- Multi-level architectural spaces

The system implements Daniel Brewer's "3D Flight Navigation Using Sparse Voxel Octrees" from Game AI Pro 3, providing both advanced pathfinding and sophisticated tactical reasoning capabilities for AI decision-making.

![Nav3D debug image](https://raw.githubusercontent.com/darbycostello/Nav3D/refs/heads/v2.0/Resources/nav3d-debug.gif)

## Key Features

### ✈️ **True 3D Navigation**
- Full volumetric pathfinding without height restrictions
- Sparse Voxel Octree (SVO) data structure for efficient 3D space representation
- Multi-scale voxel resolution based on agent size
- Considers all static meshes and instanced static meshes for navigation obstacle detection

### 🧠 **Advanced Pathfinding Algorithms**
- **A-Star**: Fast, grid-aligned paths
- **Theta-Star**: Line-of-sight optimized paths with shortcuts
- **Lazy Theta-Star**: Performance-optimized variant (default)

### 🎯 **Tactical Reasoning System**
- **Region-based spatial analysis** with automatic free space identification
- **Visibility analysis** between regions using sample-based line-of-sight testing
- **Adjacency graphs** for tactical movement planning
- **Best position finding** with configurable visibility/distance/region size preferences

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
2. **Set Navigation Extents**: Use the brush settings in the volume's details panel to set the extents rather than scaling the volume actor
3. **Configure Build Settings**: 
   - **Automatic Generation** is configured in **Edit** → **Editor Preferences** → **Level Editor - Miscellaneous**
   - For large 3D levels, disable automatic generation and use the **Build** button in the `Nav3DData` actor inspector instead

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

The foundation of Nav3D is a hierarchical 3D grid system with dynamically determined layers. The system creates as many layers as needed to:
- Provide appropriately sized voxels at layer 0 to meet agent dimensions
- Fill the entire Nav3D bounds volume created by the user
- Maintain power-of-2 octree constraints

For example, a large volume might require 7+ layers, while a small volume might only need 3-4 layers.

```
Layer N: Largest voxels (coarse pathfinding, covers entire volume)
Layer N-1: Medium voxels  
...
Layer 1: Small voxels
Layer 0: Leaf nodes (finest detail, sized for agent radius)
```

**Benefits**:
- **Memory Efficient**: Only stores occupied space
- **Multi-Scale**: Large voxels for distance, small for precision  
- **Fast Queries**: Logarithmic search times
- **Dynamic Updates**: Real-time modifications without full rebuilds

### Multi-Volume Navigation

Nav3D supports seamless pathfinding across multiple navigation volumes:

1. **Path Sanitization**: Analyzes start/end points across all volumes
2. **Intersection Detection**: Uses ray-box intersection to find volume entry/exit points
3. **Segment Creation**: Builds composite paths with multiple segments:
   - **Navigation Segments**: 3D pathfinding within volumes using octree data
   - **Direct Movement Segments**: Straight-line movement between volumes
4. **Automatic Point Adjustment**: Finds closest valid points when start/end is outside volumes

### Tactical Reasoning System

Beyond pathfinding, Nav3D includes sophisticated tactical reasoning capabilities that analyze 3D space for AI decision-making:

**Key Capabilities**:
- **Region Identification**: Automatically converts octree voxel data into meaningful tactical regions
- **Adjacency Analysis**: Builds connectivity graphs between navigable regions
- **Visibility Computation**: Sample-based line-of-sight analysis between region pairs
- **Tactical Queries**: Find optimal positions based on visibility, distance, and region size criteria
- **Multi-criteria Scoring**: Intelligent ranking of candidate positions for tactical decisions

**Blueprint Integration**:
```cpp
FindBestLocation(StartPosition, ObserverPositions, OutCandidates, 
                 VisibilityPreference, DistancePreference, RegionPreference)
```

## Performance & Optimization

### Voxel Size Strategy

```cpp
Agent Radius = 50  → Voxel Size = 100  // Fine detail, higher memory
Agent Radius = 100 → Voxel Size = 200  // Balanced
Agent Radius = 200 → Voxel Size = 400  // Coarse, fast, lower memory
```

### Pathfinding Algorithms
- **A-Star**: Fast grid-aligned paths for simple scenarios
- **Theta-Star**: High-quality paths with line-of-sight shortcuts
- **Lazy Theta-Star**: Balanced performance and quality (recommended)

### Memory Usage

Monitor navigation memory with console command:
```
CountNavMem
```

Typical memory usage:
- Small level (100m³): ~1-5 MB
- Medium level (500m³): ~10-50 MB  
- Large level (1000m³): ~50-200 MB

## Debugging & Visualization

### Enable Navigation Visualization

1. **Viewport** → **Show** → **Navigation** (checkbox)
2. Select `Nav3DData` actor in World Outliner
3. Check **Enable Drawing**

### Debug Draw Options

**Debug Draw Bounds**: White wireframe showing actual octree bounds

**Debug Draw Layers**: Hierarchical voxel display by layer index

**Debug Draw Voxels**: 
- **Red cubes**: Occluded/blocked voxels
- **Green cubes**: Free/navigable voxels

**Debug Draw Active Paths**: Real-time path visualization during gameplay

### Build Analysis

Use the **Analyse** feature in the `Nav3DData` actor inspector to:
- See how many meshes will be included in voxelization
- Understand mesh types and collision settings
- Get an indication of build complexity before processing

### Pathfinding Test Actor

Use `Nav3DPathFinderTest` actors for algorithm testing:

1. Place two test actors in scene
2. Set one as "Other Actor" on the first
3. Configure pathfinding settings
4. Use test buttons:
   - **Auto Complete Instantly**: Immediate result
   - **Auto Complete Step by Step**: Animated algorithm visualization
   - **Auto Complete Until Next Node**: Single-step debugging

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

## Current Status

### Production Ready Features
✅ Core pathfinding algorithms  
✅ Single volume navigation  
✅ Multi-volume cross-navigation with automatic bridging  
✅ Tactical reasoning with region analysis  
✅ Behavior tree integration  
✅ Basic dynamic updates  
✅ Ray-box intersection system for volume boundaries  

### Known Limitations
⚠️ World Partition support requires custom implementation  
⚠️ Large-scale level streaming needs custom commandlets  
⚠️ Single-threaded octree building (could be parallelized)