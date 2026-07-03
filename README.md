# VPhysics-Box3D

Source VPhysics (IVP/Havok) reimplemented on [Box3D](https://github.com/erincatto/box3d). Modelled on [VPhysics-Jolt](https://github.com/Joshua-Ashton/VPhysics-Jolt) (Volt).

Work in progress!

## Status

| Feature | Volt | Vox3D |
|:--|:--:|:--:|
| Constraints (except pulleys) | ✔️ | ✔️ |
| Pulleys | ✔️ | ❌ |
| Breakable constraints | ❌ | ❌ |
| Motion controllers | ✔️ | ✔️ |
| Constraint motors | ✔️ | ✔️ |
| Ragdolls | ✔️ | ✔️ |
| Triggers | ✔️ | ❌ |
| Object touch callbacks | ✔️ | ✔️ |
| Prop damage / breaking | ✔️ | ✔️ |
| Fluid events | ✔️ | ✔️ |
| Prop splashing effects | ✔️ | ✔️ |
| Wheeled vehicles | ✔️ | ❌ |
| Raycast vehicles (airboat) | ❌ | ❌ |
| Shadow controllers (NPCs, doors) | ✔️ | ✔️ |
| Save / restore | ✔️ | ❌ |
| Portal support | ✔️ | ❌ |
| Per-object no-collide callbacks | ✔️ | ❌ |
| Crash-resistant solver | ✔️ | ✔️ |
| Thousands of objects without lag | ✔️ | ✔️ |
| Multithreaded | ✔️ | ✔️ |
| Player controller | ✔️ | ✔️ |

## Platforms

> [!NOTE]
> These are Windows builds.
> Vox3D is failing to compile on Linux on the x64 MP SDK and x86 SP SDK, other branches have not been tested; macOS is unknown as of now.

| Branches | Builds | Tested |
|:--|:--:|:--:|
| SDK 2013 SP/MP x86 | ✔️ |  |
| SDK 2013 MP x64 | ✔️ |  |
| Alien Swarm x86 | ✔️ |  |
| Garry's Mod x86 | ✔️ |  |
| Garry's Mod x64 | ✔️ | ✔️ |

To build, see: [build.md](build.md)

## Media

### Watermelons
[![Watermelons](https://i.ytimg.com/vi/yTp4jTYFWJ8/hqdefault.jpg)](https://youtu.be/yTp4jTYFWJ8 "Watermelons")

### Tower of Barrels
[![Tower of Barrels](https://i.ytimg.com/vi/KAiDVJAWjMA/hqdefault.jpg)](https://youtu.be/KAiDVJAWjMA "Tower of Barrels")

### Buoyant Mossmans
[![Buoyant Mossmans](https://i.ytimg.com/vi/z4MaxT87Eqs/hqdefault.jpg)](https://youtu.be/z4MaxT87Eqs "Buoyant Mossmans")

## Credits

* [Box3D](https://github.com/erincatto/box3d) by Erin Catto
* [Volt](https://github.com/Joshua-Ashton/VPhysics-Jolt) by Joshua Ashton and Josh Dowell

## License

MIT, see [LICENSE](LICENSE). Box3D and Source SDK code retain their respective licences.
