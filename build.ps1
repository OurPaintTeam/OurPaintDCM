# Сборка OurPaintDCM с MinGW (MSYS2 UCRT64)
# Запускайте из PowerShell после добавления ucrt64 в PATH:
#   $env:Path += ";C:\msys64\ucrt64\bin"

$ErrorActionPreference = "Stop"

# Добавляем MinGW в PATH, если ещё не добавлен
if (-not (Get-Command g++ -ErrorAction SilentlyContinue)) {
    $env:Path += ";C:\msys64\ucrt64\bin"
}

# Удаляем старый build (очистка кэша CMake)
if (Test-Path build) {
    Remove-Item -Recurse -Force build
}

# Конфигурация с MinGW + Ninja
cmake -G "Ninja" `
    -DCMAKE_CXX_COMPILER="C:\msys64\ucrt64\bin\g++.exe" `
    -DCMAKE_C_COMPILER="C:\msys64\ucrt64\bin\gcc.exe" `
    -DCMAKE_MAKE_PROGRAM="C:\msys64\ucrt64\bin\ninja.exe" `
    -S . -B build

# Сборка
cmake --build build
