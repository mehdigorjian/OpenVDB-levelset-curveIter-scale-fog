# Using OpenVDB to process .stl file:
	- .stl to .obj subdivided
	- subdivisio to level-set
	- mean curve iteration
	- fog from level-set

## COMPILATION:

### Problem 1 is divided into two separate directories:
	1. loop-subdiv: which is a GUI-based app to subdivide the .stl file and save it as an  .obj file
	2. project-01: which performs:
		1. Subdiv_to_levelset
		2. Mean_curve_iter
		3. Z_scale
		4. fog_from_level_set


To compile and run 'Problem 1' do the following steps:

### STEP 1: converting .stl input to a loop subdivided .obj file
	1. open Terminal in ‘loop-subdiv-GUI' directory
		mkdir build
		cd build
		cmake ..
		make
	2. to run:
		./subdivision
	3.  it asks you to enter the .stl file path ../data/hw1.stl and outputs an .obj file subdiv_obj_hw1.obj
		# GUI keyboard shortcuts:
		[spacebar]    run subdivision 1X each time
  		2           run subdivision 2X
  		R,r         reset mesh to the original object
  		S,s         save subdivided .obj file

### STEP 2: copy the data from 'loop-subdiv-GUI' to 'project-01'
	- copy 'subdiv_obj_hw1.obj' file from the 'loop-subdiv-GUI/project-01/data' directory and paste it into the 'project-01/data' directory

### STEP 3:
	1. open Terminal in ‘project-01’ directory
	2. install OpenVDB library: https://github.com/AcademySoftwareFoundation/openvdb
		mkdir build
		cd build
		cmake ..
		make
	3. to run:
		./solveproblem

