# Nav3D for Unreal Engine

![Nav3D banner image](https://user-images.githubusercontent.com/891532/103788146-6cdfe280-5036-11eb-883f-c9bf174b4ec2.jpg)

Nav3D is a navigation and cover system plugin for UE4, using [Sparse Voxel Octrees](https://www.gdcvault.com/play/1022016/Getting-off-the-NavMesh-Navigating) to provide pathfinding solutions and associated queries within a full 3D volume. It comprises a number of modular components and does **not** rely on UE4's other systems, such as Navmesh, AI or Behaviour Trees. This allows you greater flexibility in solving the 3D pathfinding problems for your particular project.

This is a work in progress and part of a wider project I'm working on. For professional projects I'm sure you're already aware of the commercially available SVO solution for UE4, which may better suit your needs. This project was put together as part of my own learning and project development. 

## Features Overview

- Nav3D Volume actor - set the size and minimum voxel scale. The software will calculate everything else to achieve the required level of detail.
- Navigation data can be built from any collision channel (e.g. World Static) within editor and then saved with the level.
- Nav3D actor component can be added to any actor, providing Blueprints-accessible pathfinding tasks that are executed as asynchronous tasks, away from the main game thread.

![Nav3D Blueprints nodes](https://user-images.githubusercontent.com/891532/103788152-6f423c80-5036-11eb-914e-e16f14640a70.jpg)
- Pathfinding (Greedy A*) supports Manhattan and Euclidean heuristics and includes a path pruning method to remove extraneous path points. In addition you can apply Catmull-Rom smoothing for better-looking paths which don't deviate from their calculated locations.
- Pathfinding attempts to find the closest accessible locations to the requested start and end points, preventing the majority of pathfinding failures.
- Nav3D Occlusion component can be added to any actor allowing it to be used as a dynamic obstacle within the navigation system. When the actor moves, its occlusion data is regenerated on the fly and again this is performed asynchronously, away from the game thread.

![Occlusion component in action](https://user-images.githubusercontent.com/891532/103788164-71a49680-5036-11eb-883e-a50a30f06f90.gif)
- Generate a cover map along with the navigation, which maps the normals of each occluded surface voxel to one of 26 directions, (or if you like math, the faces of a [rhombicuboctahedron](https://en.wikipedia.org/wiki/Rhombicuboctahedron)). This is then used by the Nav3D component in fast async queries, to find the best hiding spots from a single opponent, or from multiple actors.

![Cover map generaton](https://user-images.githubusercontent.com/891532/103788163-710c0000-5036-11eb-9179-afc4be50d56a.jpg)
- Combine the cover system with the Nav3D Occlusion component to provide cover spots on moving actors.

![Cover map used with Nav3D Occlusion component](https://user-images.githubusercontent.com/891532/103788175-7406f080-5036-11eb-8e83-9e507c45a621.gif)
- Nav3D Modifier Volume can be added to modify the path costs of a given region, for example to create less preferable areas for pathfinding. Modifiers can also invalidate any cover locations, to mark hazardous areas.
- Extensive debug drawing both in editor and in-game for octree layers, occluded voxels, traversable edges, Morton codes, navigation paths and cover map locations. This is all centralised and run from a single method in the Nav3d Volume.

![Nav3D debug drawing with Morton codes](https://user-images.githubusercontent.com/891532/103788158-6fdad300-5036-11eb-98c5-ff642d7ad3c5.jpg)

## Limitations & Future Work
- While a Nav3D component will use whatever Nav3D Volume it finds itself within, there is currently no support for travelling between separate volumes, or how the pathfinding will be performed at each end. This is in the development roadmap.
- The system does not perform any interceptions or trajectory calculations in order to provide pathfinding to moving targets. This is also intended to be implemented in future versions and will likely tie into the Nav3D component owner's movement.
- The plugin does not take care of any steering or path-following mechanics. It is minimally designed as an environment query and plotting system. Likewise, there is no intention to integrate it with UE4's AI component, EQS or Blackboard (which I personally don't use). Instead I encourage you to integrate it with your own systems and steering behaviours, as required. I'm also working separately on a 3D RVO avoidance plugin and PID controllers for steering and path-following.

