# Building Vox3D

## Prerequisites

- Git
- CMake 3.25 or newer
- Windows: Visual Studio 2022 with the "Desktop development with C++" workload
- Linux: `ninja-build` and gcc (`gcc-multilib` / `g++-multilib` for 32-bit)

## Get the code

```bash
git clone --recursive https://github.com/Asphaltian/VPhysics-Box3D.git
cd VPhysics-Box3D
```

If you already cloned without `--recursive`, pull the submodules:

```bash
git submodule update --init --recursive
```

## Build

> [!NOTE] 
> Only Garry's Mod x64 is well tested. Linux is experimental and some variants don't build yet.

Choose the preset for your game and architecture, configure, then build:

```bash
cmake --preset gmod-x64
cmake --build --preset gmod-x64-Release
```

| Preset | Game | Arch |
|:---|:---|:---:|
| `gmod-x86` | Garry's Mod | x86 |
| `gmod-x64` | Garry's Mod | x64 |
| `gmod-linux-x86` | Garry's Mod (Linux) | x86 |
| `gmod-linux-x64` | Garry's Mod (Linux) | x64 |
| `sdk2013-mp` | Source SDK 2013 Multiplayer | x86 |
| `sdk2013-sp` | Source SDK 2013 Singleplayer | x86 |
| `asw` | Alien Swarm | x86 |

Every preset has a `-Debug` and `-Release` build (e.g. `gmod-x64-Release`, `sdk2013-mp-Debug`). The result is `build/<preset>/bin/<config>/vphysics.dll` (`vphysics.so` on Linux).

If you have a Visual Studio other than 2022, skip the preset and name your generator:

```bash
cmake -S . -B build/gmod-x64 -G "Visual Studio 18 2026" -A x64 -D VOX_SDK=gmod
cmake --build build/gmod-x64 --config Release
```

## Install

Back up the game's own `vphysics.dll`, then drop the one you built in its place. For 64-bit Garry's Mod:

```
<Steam>/steamapps/common/GarrysMod/bin/win64/vphysics.dll
```

Other builds go in the matching game's binary directory (`bin/` for 32-bit Garry's Mod, the mod's `bin/` for SDK 2013).
