# Surface-Reconstruction
Using C++ and the Point Cloud Library (PCL), a command line tool which loads a pointcloud file and performs a surface reconstruction on it, and then saves the constructed mesh to disk.
Name:	Umur Gogebakan
email:	umur.gogebakan@gmail.com
Ankara, Turkey

		COMPILATION
Example cmake command:
	$cmake -DCMAKE_BUILD_TYPE=None -DCMAKE_INSTALL_PREFIX=/usr           -DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_examples=ON
Then, typing make should simple yield the exacutable file called surface_reconstruct:
	$make
	
	Scanning dependencies of target surface_reconstruct
	[ 50%] Building CXX object CMakeFiles/surface_reconstruct.dir/surface_reconstruct.cpp.o
	[100%] Linking CXX executable surface_reconstruct
	[100%] Built target surface_reconstruct
	
	EXECUTION
Usage examples:
./surface_reconstruct bun000.ply output.obj

./surface_reconstruct bun000.ply output.obj -method poisson [poisson_options]
e.g.
	./surface_reconstruct bun000.ply output.obj -method poisson -depth 8

./surface_reconstruct bun000.ply output.obj -method grid_projection [grid_projection_options]
e.g.
	./surface_reconstruct bun000.ply output.obj -method grid_projection -resolution 0.005
	
Please type the following for detailed help option:
	./surface_reconstruct -h

Two ply files are also included that are used during testing.

* All the implementation and testing were done locally using NetBeans on Linux Ubuntu 18.04 .
* stanford_bunny.png shows the acquired result when the testing is done on example ply files on my local machine.


## [Stanford Bunny](http://graphics.stanford.edu/data/3Dscanrep/)
![Alt text](stanford_bunny.png?raw=true "Bun000")
