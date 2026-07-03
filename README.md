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
> These are Windows builds. Linux and macOS are unknown as of now.

| Branches | Builds | Tested |
|:--|:--:|:--:|
| SDK 2013 SP/MP x86 | ✔️ | |
| SDK 2013 MP x64 | ✔️ | |
| Alien Swarm x86 | ✔️ | |
| Garry's Mod x86 | ✔️ | |
| Garry's Mod x64 | ✔️ | ✔️ |

To build, see: [build.md](build.md)

## Credits

* [Box3D](https://github.com/erincatto/box3d) by Erin Catto
* [Volt](https://github.com/Joshua-Ashton/VPhysics-Jolt) by Joshua Ashton and Josh Dowell

## License

MIT, see [LICENSE](LICENSE). Box3D and Source SDK code retain their respective licences.
