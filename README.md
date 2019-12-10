# CG Project
## Building
You can use CMake to build a release version, which makes everything much quicker.

To do so, do the following:
```
cd build
cmake ../
cmake --build . --config Release
cd ../
build/Release/raytracing.exe
```

For Windows you can also run `buildAndRun.ps1` or `buildAndRunLog.ps1` from powershell.

Make sure glew32.dll is present in C:\Windows\System32!