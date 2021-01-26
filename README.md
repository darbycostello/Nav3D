Nav3D is a navigation and cover system plugin for UE4, using Sparse Voxel Octrees to provide pathfinding and associated queries within a fully 3D volume. It provides a number of core components that do **not** rely on UE4's other systems, such as Navmesh, AI or Behaviour Trees. This allows you greater flexibility in solving the 3D pathfinding problems for your particular project.

This is a work in progress and part of a wider project I'm working on. For professional projects I'm sure you're already aware of the commercially available SVO solution for UE4, which may better suit your needs. This project was put together as part of my own learning and project development. 

## Features Overview

- Nav3D Volume actor - set the size and minimum voxel scale. The software will calculate everything else to achieve the required level of detail.
- Navigation data can be built from any collision channel (e.g. World Static) within editor and then saved with the level.
- Nav3D actor component can be added to any actor, providing Blueprints-accessible pathfinding tasks that are executed as asynchronous tasks, away from the main game thread.
- Pathfinding (Greedy A*) supports Manhattan and Euclidean heuristics and includes a path-pruning method to remove extraneous path points. In addition you can apply Catmull-Rom smoothing for better-looking paths which don't deviate from their calculated locations.
- Pathfinding attempts to find the closest accessible locations to the requested start and end points, preventing the majority of pathfinding failures.
- Nav3D Occlusion component can be added to any actor allowing it to be used as a dynamic obstacle within the navigation system. When the actor moves its occlusion data is regenerated on the fly and again this is performed asynchronously, off the game thread.
- Generate a cover map along with the navigation, which maps the normals of each occluded surface voxel to one of 26 directions, (or if you like math, the faces of a [rhombicuboctahedron](https://en.wikipedia.org/wiki/Rhombicuboctahedron)). This is then used by the Nav3D component in fast async queries, to find the best hiding spots from a single opponent, or from multiple actors.
- Combine the cover system with the Nav3D Occlusion component to provide cover spots on moving actors.
- Nav3D Modifier Volume can be added to modify the path costs of a given region, for example to create less preferable areas for pathfinding. Modifiers can also invalidate any cover locations, to mark hazardous areas.
- Extensive debug drawing both in editor and in-game for octree layers, occluded voxels, traversable edges, Morton codes, navigation paths and cover map locations. This is all centralised and run from a single method in the Nav3d Volume.

## Limitations & Future Work
- While a Nav3D component will use whatever Nav3D Volume it finds itself within, there is currently no support for travelling between separate volumes, or how the pathfinding will be performed at each end. This is in the development roadmap.
- The system does not perform any interceptions or trajectory calculations in order to provide pathfinding to moving targets. This is also intended to be implemented in future versions and will likely tie into the Nav3D component owner's movement.
- The plugin does not take care of any steering or path-following mechanics. It is minimally designed as an environment query and plotting system. Likewise, there is no intention to integrate it with UE4's AI component, EQS or Blackboard (which I personally don't use). Instead I encourage you to integrate it with your own systems and steering behaviours, as required. I'm also working on a 3D RVO avoidance plugin and PID controllers for steering and path-following.

