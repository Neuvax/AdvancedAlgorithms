all:
	cd Voronoi && /opt/homebrew/bin/cgal_create_CMakeLists -s main && cmake -DCMAKE_BUILD_TYPE=Release . && make

clean:
	cd Voronoi && rm -rf CMakeFiles && rm -rf cmake_install.cmake && rm -rf CMakeCache.txt && rm -rf Makefile && rm -rf main && rm -rf CMakeLists.txt && rm -rf CMakeLists.txt.bak

run:
	cd Voronoi && ./main