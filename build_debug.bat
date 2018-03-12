rem %EMSCRIPTEN%\emcc --clear-cache
rem --tracing
rem -D_LIBCPP_DEBUG=0  
rem --preload-file loads things late

%EMSCRIPTEN%\em++  -g3 -O0 --bind -s ASSERTIONS=2 -s SAFE_HEAP=1 -s DEMANGLE_SUPPORT=1 -s ALLOW_MEMORY_GROWTH=0 -D_DEBUG -D_LIBCPP_DEBUG=0  --memory-init-file 0 -Wno-switch -Isrc src/main.cpp src/Mesh.cpp --embed-file models/bunny.obj -o out/topo_js_main.html 

rem Pointer_stringify