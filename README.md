# VPhysics-Box3D

Source VPhysics (IVP/Havok) reimplemented on [Box3D](https://github.com/erincatto/box3d). Modelled on [VPhysics-Jolt](https://github.com/Joshua-Ashton/VPhysics-Jolt) (Volt).

For discussions, please join the Discord server [here](https://discord.gg/rbCJdZjewf).

## Status

| Feature | VPhysics | Volt | Vox3D |
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

Build status per CMake preset. Future support for macOS is unknown.

| Preset | Builds | Tested |
|:--|:--:|:--:|
| `gmod-x64` | ✔️ | ✔️ |
| `gmod-x86` | ✔️ |  |
| `gmod-linux-x86` | ✔️ |  |
| `gmod-linux-x64` | ✔️ | ✔️ |
| `gmod-linux-x86-dedicated` | ✔️ |  |
| `gmod-linux-x64-dedicated` | ✔️ |  |
| `sdk2013-mp` | ✔️ |  |
| `sdk2013-mp-linux` | ✔️ |  |
| `sdk2013-sp` | ✔️ |  |
| `sdk2013-sp-linux` | ✔️ |  |
| `asw` | ✔️ |  |

To build, see: [build.md](build.md)

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

### Intrusive Thoughts
[![Intrusive Thoughts](https://i.ytimg.com/vi/Qr0fF_xtnmU/hqdefault.jpg)](https://youtu.be/Qr0fF_xtnmU "Intrusive Thoughts")

## Credits

* [Box3D](https://github.com/erincatto/box3d) by Erin Catto
* [Volt](https://github.com/Joshua-Ashton/VPhysics-Jolt) by Joshua Ashton and Josh Dowell

## License

MIT, see [LICENSE](LICENSE). Box3D and Source SDK code retain their respective licenses.
