cmake -S . -B build/debug -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
cmake --build build/debug --parallel 3

# cmake -S . -B build/release -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON -DCMAKE_CXX_FLAGS="-march=native -O3"
# cmake --build build/release --parallel 3