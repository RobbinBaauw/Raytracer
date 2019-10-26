# CG Project
## Building
You can use CMake to build a release version, which makes everything much quicker as IO on Windows is single threaded, thus debugging makes everything much, much slower.

To do so, do the following:
```
cd build
cmake ../
cmake --build . --config Release
cd ../
build/Release/raytracing.exe
```

You can also run `buildAndRun.ps1` or `buildAndRunLog.ps1` from powershell.

Make sure glew32.dll is present in C:\Windows\System32!