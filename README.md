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
| Portal support | ✔️ | ✔️ | ❌ |
| Per-object no-collide callbacks | ✔️ | ✔️ | ✔️ |
| Crash-resistant solver | ❌ | ✔️ | ✔️ |
| Thousands of objects without lag | ❌ | ✔️ | ✔️ |
| Multithreaded | ❌ | ✔️ | ✔️ |
| Player controller | ✔️ | ✔️ | ✔️ |

## Platforms

| Preset | Builds | Tested |
|:--|:--:|:--:|
| `gmod-windows-x64` | ✔️ | ✔️ |
| `gmod-linux-x64` | ✔️ |  |
| `sdk2013-windows-x64` | ✔️ |  |
| `sdk2013-linux-x64` | ✔️ |  |

## Building

You'll need:

* **CMake** 3.25 or newer
* **Windows:** Visual Studio 2026 with the **Desktop development with C++** workload
* **Linux:** Ninja and GCC 14

Clone the repository with its submodules:

```
git clone --recursive https://github.com/Asphaltian/VPhysics-Box3D.git
cd VPhysics-Box3D
```

Then configure and build the preset for your game, for example:

```
cmake --preset gmod-windows-x64
cmake --build --preset gmod-windows-x64-release
```

The library ends up in `build/<preset>/src/Release/`, as `vphysics.dll` on Windows and `vphysics.so` on Linux. Swap `-release` for `-debug` if you need a debug build.

## Installing

Replace the game's own `vphysics` library with the one you built:

* **Garry's Mod:** `GarrysMod/bin/win64` or `GarrysMod/bin/linux64`
* **Source SDK Base 2013 Multiplayer:** `bin/x64` or `bin/linux64`

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
