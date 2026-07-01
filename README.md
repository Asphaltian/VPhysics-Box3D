# VPhysics-Box3D

Vox3D is a replacement for Source's VPhysics (originally IVP/Havok) built on
[Box3D](https://github.com/erincatto/box3d). It mirrors the structure of
[VPhysics-Jolt](https://github.com/Joshua-Ashton/VPhysics-Jolt) (Volt).

This is a work in progress. Rigid-body simulation, collision, materials, traces, and the
grab/shadow controllers work in-engine. Constraints, ragdoll joints, vehicles, fluids, and
save/restore are not implemented yet.

## Status

| Feature | Implemented |
|:--|:--:|
| Rigid bodies (fall, collide, rest) | yes |
| Collision geometry from `.phy` (convex hulls) and world | yes |
| Collision + touch callbacks, impact sounds | yes |
| Surface materials (friction, restitution, density) | yes |
| Traces against a collide (bullets, `+use`, movement) | yes |
| Shadow controllers (doors, `+use` pickup, physgun hold) | yes |
| Motion controllers (gravity gun) | yes |
| Player controller | no |
| Constraints (ragdoll, weld, hinge, ball-socket, sliding, pulley, length) | no |
| Wheeled / raycast vehicles | no |
| Fluids / buoyancy | no |
| Save / restore | no |

Traces cover convex hulls only; polysoup (concave) collision meshes are not traced yet.

## Platforms

| Target | Builds | Tested in-engine |
|:--|:--:|:--:|
| Source SDK 2013 SP/MP (x86) | yes | no |
| Alien Swarm (x86) | yes | no |
| Garry's Mod (x86) | yes | no |
| Source SDK 2013 (x64) | yes | no |
| Garry's Mod (x64) | yes | yes |

Garry's Mod uses the CS:GO VPhysics interface rather than the public SDK's; building for it
needs headers that aren't in this repo. See [build.md](build.md).

## Building

See [build.md](build.md).

## Credits

- [Box3D](https://github.com/erincatto/box3d) by Erin Catto
- Structure, conventions, and the IVP `.phy` / shadow-control logic follow
  [VPhysics-Jolt](https://github.com/Joshua-Ashton/VPhysics-Jolt) by Joshua Ashton and Josh Dowell

## License

MIT! See [LICENSE](LICENSE). Box3D and any Source SDK code retain their own licenses.
