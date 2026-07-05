# VPhysics-Box3D

Source VPhysics (IVP/Havok) reimplemented on [Box3D](https://github.com/erincatto/box3d). Modelled on [VPhysics-Jolt](https://github.com/Joshua-Ashton/VPhysics-Jolt) (Volt).

Work in progress!

## Status

| Feature | VPhysics | Volt | Vox3D |
|:--|:--:|:--:|:--:|
| Constraints (except pulleys) | ✔️ | ✔️ | ✔️ |
| Pulleys | ✔️ | ✔️ | ❌ |
| Breakable constraints | ✔️ | ❌ | ❌ |
| Motion controllers | ✔️ | ✔️ | ✔️ |
| Constraint motors | ✔️ | ✔️ | ✔️ |
| Ragdolls | ✔️ | ✔️ | ✔️ |
| Triggers | ✔️ | ✔️ | ❌ |
| Object touch callbacks | ✔️ | ✔️ | ✔️ |
| Prop damage / breaking | ✔️ | ✔️ | ✔️ |
| Fluid events | ✔️ | ✔️ | ✔️ |
| Prop splashing effects | ✔️ | ✔️ | ✔️ |
| Wheeled vehicles | ✔️ | ✔️ | ❌ |
| Raycast vehicles (airboat) | ✔️ | ❌ | ❌ |
| Shadow controllers (NPCs, doors) | ✔️ | ✔️ | ✔️ |
| Save / restore | ✔️ | ✔️ | ❌ |
| Portal support | ✔️ | ✔️ | ❌ |
| Per-object no-collide callbacks | ✔️ | ✔️ | ✔️ |
| Crash-resistant solver | ❌ | ✔️ | ✔️ |
| Thousands of objects without lag | ❌ | ✔️ | ✔️ |
| Multithreaded | ❌ | ✔️ | ✔️ |
| Player controller | ✔️ | ✔️ | ✔️ |

## Platforms

Build status per CMake preset. Future support for macOS is unknown.

| Preset | Builds | Tested |
|:--|:--:|:--:|
| `gmod-x64` | ✔️ | ✔️ |
| `gmod-x86` | ✔️ |  |
| `gmod-linux-x86` | ✔️ |  |
| `gmod-linux-x64` | ✔️ |  |
| `sdk2013-mp` | ✔️ |  |
| `sdk2013-sp` | ❌ |  |
| `asw` | ❌ |  |

To build, see: [build.md](build.md)

## Media

### Watermelons
[![Watermelons](https://i.ytimg.com/vi/yTp4jTYFWJ8/hqdefault.jpg)](https://youtu.be/yTp4jTYFWJ8 "Watermelons")

### Buoyant Mossmans
[![Buoyant Mossmans](https://i.ytimg.com/vi/z4MaxT87Eqs/hqdefault.jpg)](https://youtu.be/z4MaxT87Eqs "Buoyant Mossmans")

### Battle for Flatgrass
[![Battle for Flatgrass](https://i.ytimg.com/vi/BVPiprozkNM/hqdefault.jpg)](https://youtu.be/BVPiprozkNM "Battle for Flatgrass")

## Credits

* [Box3D](https://github.com/erincatto/box3d) by Erin Catto
* [Volt](https://github.com/Joshua-Ashton/VPhysics-Jolt) by Joshua Ashton and Josh Dowell

## License

MIT, see [LICENSE](LICENSE). Box3D and Source SDK code retain their respective licenses.
