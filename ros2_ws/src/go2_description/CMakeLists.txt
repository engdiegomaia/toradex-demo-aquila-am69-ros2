cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

# `launch` was in this list upstream and is deliberately gone: the upstream
# launch files bring up RViz2 inside the simulation launch, which is OGRE 2 and
# must not run on the module (CLAUDE.md rule 1). Our launch lives in
# demo_simulation. See README.md for the full list of removals.
install(
		DIRECTORY meshes xacro config urdf
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()