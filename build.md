# Building Vox3D

Vox3D builds with CMake. There is no Source tree and no VPC: the Source SDK it
targets is a submodule under `sdk/`, and a CMake preset selects which variant to
build against.

## Get the tree

Clone with submodules (Box3D and the mini-source-sdk both come in as submodules):

```bash
git clone --recursive https://github.com/Asphaltian/VPhysics-Box3D.git
cd VPhysics-Box3D
```

Already cloned without `--recursive`? Run:

```bash
git submodule update --init --recursive
```

## Build

Pick a preset (it selects the SDK variant and the target architecture), configure,
then build:

```bash
cmake --preset gmod-x64
cmake --build --preset gmod-x64-Release
```

The output lands in `build/gmod-x64/bin/Release/vphysics.dll`.

Available configure presets:

| Preset        | SDK variant   | Arch |
|:--------------|:--------------|:-----|
| `gmod-x64`    | gmod          | x64  |
| `gmod-x86`    | gmod          | x86  |
| `sdk2013-mp`  | Source SDK 2013 MP | x86 |
| `sdk2013-sp`  | Source SDK 2013 SP | x86 |
| `asw`         | Alien Swarm   | x86  |

Each has matching `-Debug` and `-Release` build presets (e.g. `sdk2013-mp-Debug`).

## SDK libraries

The shim links tier0, tier1, tier2, mathlib and vstdlib. tier0 and vstdlib are
always prebuilt import libs (they front game DLLs). tier1, tier2 and mathlib are
compiled from the `sdk/mini-source-sdk` submodule's own source when the variant
ships it (`gmod`, `sdk2013-*`), so nothing is committed and the build is fully
reproducible from the two submodules. The `asw` variant ships no source and links
its prebuilt tier1/mathlib/tier2/interfaces instead. This is all handled
automatically by `sdk/CMakeLists.txt`.

## Output

`vphysics.dll` is the loadable module. It implements `IPhysics` directly on top of
the Box3D engine (built as the `box3d` static lib) - there is no separate
CPU-dispatch loader.
