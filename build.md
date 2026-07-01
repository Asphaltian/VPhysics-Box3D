# Building Vox3D

Vox3D builds the same way as Volt: inside a Source SDK source tree, driven by Valve's VPC.
This guide covers the 32-bit Source SDK 2013 build on Windows. x64 and Garry's Mod are
outlined at the end but not walked through step by step.

## Prerequisites

- Visual Studio 2022 or newer, with the "Desktop development with C++" workload
- A recent Windows 10/11 SDK
- [mini-source-sdk](https://github.com/Joshua-Ashton/mini-source-sdk) -- a stripped Source SDK
  with VS2022 / Windows 11 SDK support. It has branches for `sdk2013-mp`, `sdk2013-sp`, and `asw`

## Get the code

Clone the mini-source-sdk and pick a branch. `sdk2013-mp` is used below.

```
git clone https://github.com/Joshua-Ashton/mini-source-sdk.git
cd mini-source-sdk/sdk2013-mp/src
```

Clone this repository into `src` as `vphysics_box3d`, with submodules (this pulls Box3D).

```
git clone --recursive https://github.com/Asphaltian/VPhysics-Box3D.git vphysics_box3d
```

If you already cloned without `--recursive`:

```
cd vphysics_box3d
git submodule update --init
cd ..
```

## Wire the VPC group

The mini-source-sdk ships pre-wired for Volt, not Vox3D. Edit `src/vpc_scripts/default.vgc`
and add an include for this repo's group and project scripts:

```
$Include "vphysics_box3d\vpc_scripts\vbox3d_groups.vgc"
$Include "vphysics_box3d\vpc_scripts\vbox3d_projects.vgc"
```

## Generate the projects

From `src`, run `fix_registry.bat` once as Administrator (VPC needs it to write the `.sln`),
then generate the solution. `+vox3d` is this repo's VPC group.

```
.\fix_registry.bat
devtools\bin\vpc.exe +vox3d /define:GAME_SDK2013 /mksln vox3d.sln
```

Branch defines:

| Branch | Define |
|:--|:--|
| `sdk2013-mp`, `sdk2013-sp` | `/define:GAME_SDK2013` |
| `asw` | `/define:GAME_ASW` |

## Build

Open `vox3d.sln` in Visual Studio and build, or build the generated projects with MSBuild.
The output is `game/bin/vphysics.dll` plus `vphysics_box3d_sse2.dll`, `_sse42.dll`, and
`_avx2.dll`. `vphysics.dll` is the loader; it picks the variant matching the CPU.

## Notes and common errors

- **Toolset.** VPC emits the v143 toolset. If your Visual Studio only has a newer one
  (e.g. v145), pass it to MSBuild:
  `/p:PlatformToolset=v145 /p:WindowsTargetPlatformVersion=10.0.26100.0`.
- **mathlib / tier1.** These are built from source in this tree, not prebuilt in `lib/public`.
  Build the `mathlib` and `tier1` projects before the Vox3D projects if the linker can't find them.
- **Box3D.** The Box3D submodule compiles as a C17 static library (`box3d_{sse2,sse42,avx2}.lib`),
  configured by the `box3d/*.vpc` scripts. No extra setup is needed.
- **`memoverride.cpp`.** If you build against the unmodified public SDK 2013 you may need the
  Windows 11 SDK for `_msize` symbols, same as Volt.

## x64 and Garry's Mod

The mini-source-sdk's VPC is too old to emit x64 projects. x64 builds are done inside the full
[source-sdk-2013](https://github.com/ValveSoftware/source-sdk-2013) tree, whose VPC does support
x64, with the extra define `/define:VBOX_FULL_SDK` (adds `/std:c++17` and adjusts a few SDK
compatibility guards). GMod additionally builds with `/define:GAME_GMOD`.

Garry's Mod uses the CS:GO VPhysics interface, not the public SDK's. Building for it needs
CS:GO/GMod VPhysics and appframework headers that are not redistributed here; supply your own.

## Alien Swarm

Alien Swarm builds from the `asw` branch with `/define:GAME_ASW`. The
`vphysics_box3d/compat/compat_asw.h` shim covers the interface differences.
