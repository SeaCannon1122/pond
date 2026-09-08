cmake -S . -B build -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
cmake --build build --parallel 3