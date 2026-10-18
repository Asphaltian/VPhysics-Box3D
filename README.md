# VPhysics Box3D

This is a reimplementation of Source VPhysics (IVP/Havok) on [Box3D](https://github.com/erincatto/box3d).

For discussions, please join the Discord server [here](https://discord.gg/rbCJdZjewf).

## Features

| Feature | VPhysics | Volt | VBox3D |
|:--|:--:|:--:|:--:|
| Constraints (except pulleys) | ✔️ | ✔️ | ✔️ |
| Pulleys | ✔️ | ✔️ | ✔️ |
| Breakable constraints | ✔️ | ❌ | ✔️ |
| Motion controllers | ✔️ | ✔️ | ✔️ |
| Constraint motors | ✔️ | ✔️ | ✔️ |
| Ragdolls | ✔️ | ✔️ | ✔️ |
| Triggers | ✔️ | ✔️ | ✔️ |
| Object touch callbacks | ✔️ | ✔️ | ✔️ |
| Prop damage / breaking | ✔️ | ✔️ | ✔️ |
| Fluid events | ✔️ | ✔️ | ✔️ |
| Prop splashing effects | ✔️ | ✔️ | ✔️ |
| Wheeled vehicles | ✔️ | ✔️ | ✔️ |
| Raycast vehicles (airboat) | ✔️ | ❌ | ✔️ |
| Shadow controllers (NPCs, doors) | ✔️ | ✔️ | ✔️ |
| Save / restore | ✔️ | ✔️ | ✔️ |
| Per-object no-collide callbacks | ✔️ | ✔️ | ✔️ |
| Crash-resistant solver | ❌ | ✔️ | ✔️ |
| Thousands of objects without lag | ❌ | ✔️ | ✔️ |
| Multithreaded | ❌ | ✔️ | ✔️ |
| Player controller | ✔️ | ✔️ | ✔️ |

## Platforms

One build works for both Garry's Mod and Source SDK 2013 Multiplayer games, just pick the preset for your platform.

| Preset | Builds | Tested |
|:--|:--:|:--:|
| `windows-x64` | ✔️ | ✔️ |
| `windows-x86` | ✔️ | ✔️ |
| `linux-x64` | ✔️ |  |
| `linux-x86` | ✔️ |  |
| `dedicated-linux-x64` | ✔️ |  |
| `dedicated-linux-x86` | ✔️ | ✔️ |

Windows dedicated servers load the same `vphysics.dll` as the game, so they use the Windows presets.

## Building

You'll need:

* **CMake** 3.25 or newer
* **Windows:** Visual Studio 2026 with the **Desktop development with C++** workload, plus **C++ Clang tools for Windows** for `windows-x86`
* **Linux:** Podman or Docker

Clone the repository with its submodules:

```
git clone --recursive https://github.com/Asphaltian/VPhysics-Box3D.git
cd VPhysics-Box3D
```

On Windows, configure and build the preset for your platform:

```
cmake --preset windows-x64
cmake --build --preset windows-x64-release
```

Linux builds are made inside Valve's Steam Runtime SDK, the same environment the games themselves are built in:

```
podman run --rm -v "$PWD":/src -w /src registry.gitlab.steamos.cloud/steamrt/sniper/sdk:latest \
	sh -c "cmake --preset linux-x86 && cmake --build --preset linux-x86-release"
```

The library ends up in `build/<preset>/src/Release/`. Swap `-release` for `-debug` if you need a debug build.

## Installing

Replace the game's own `vphysics` library with the one you built:

| Game | Windows | Linux |
|:--|:--|:--|
| Garry's Mod | `bin/win64/vphysics.dll` (x64), `bin/vphysics.dll` (x86) | `bin/vphysics.so` |
| Garry's Mod Dedicated Server | `bin/win64/vphysics.dll` (x64), `bin/vphysics.dll` (x86) | `bin/vphysics_srv.so` |
| Source SDK Base 2013 Multiplayer | `bin/x64/vphysics.dll` (x64), `bin/vphysics.dll` (x86) | `bin/linux64/vphysics.so` (x64), `bin/vphysics.so` (x86) |
| Source SDK Base 2013 Dedicated Server | `bin/x64/vphysics.dll` (x64), `bin/vphysics.dll` (x86) | `bin/linux64/vphysics_srv.so` (x64), `bin/vphysics_srv.so` (x86) |

Keep in mind that Steam puts the original back whenever it verifies or updates the game files, so back it up and copy yours over again afterwards.

## Media

### IVP/Havok vs. Jolt vs. Box3D
[![IVP/Havok vs. Jolt vs. Box3D](https://i.ytimg.com/vi/cs8C8T1vg7g/hqdefault.jpg)](https://youtu.be/cs8C8T1vg7g "IVP/Havok vs. Jolt vs. Box3D")

### Watermelons
[![Watermelons](https://i.ytimg.com/vi/yTp4jTYFWJ8/hqdefault.jpg)](https://youtu.be/yTp4jTYFWJ8 "Watermelons")

### Buoyant Mossmans
[![Buoyant Mossmans](https://i.ytimg.com/vi/z4MaxT87Eqs/hqdefault.jpg)](https://youtu.be/z4MaxT87Eqs "Buoyant Mossmans")

### Battle for Flatgrass
[![Battle for Flatgrass](https://i.ytimg.com/vi/BVPiprozkNM/hqdefault.jpg)](https://youtu.be/BVPiprozkNM "Battle for Flatgrass")

### Vehicular Homicide
[![Vehicular Homicide](https://i.ytimg.com/vi/IOlv3qxC7Js/hqdefault.jpg)](https://youtu.be/IOlv3qxC7Js "Vehicular Homicide")

## Credits

* [Box3D](https://github.com/erincatto/box3d) by Erin Catto
* Inspired by [Volt](https://github.com/Joshua-Ashton/VPhysics-Jolt) by Joshua Ashton and Josh Dowell

## License

MIT, see [LICENSE](LICENSE). Box3D, qhull and Source SDK code retain their respective licenses.
