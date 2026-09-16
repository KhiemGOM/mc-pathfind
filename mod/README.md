# Nether Pathfinder

A Fabric client mod for Minecraft 26.2 that computes a route through Nether terrain
(sprint/mine/bridge/climb/fall, weighted by real time cost) toward a target, and draws
it in-world for you to follow. Command-driven, Baritone-style: nothing runs
automatically, you tell it where to go.

Built on the same weighted-A* search engine documented in [`../README.md`](../README.md)
and [`../java/README.md`](../java/README.md) (this repo's own algorithm research) --
this project is just the missing piece: an actual playable mod wrapping that engine for
the current game version.

## Commands

- `/pathfind goto <x> <y> <z>` -- path to an explicit coordinate. Implemented.
- `/pathfind stop` -- clear the active route. Implemented.
- `/pathfind bastion` -- locate and path to the nearest bastion remnant in an
  integrated single-player Nether world. Multiplayer servers do not expose enough
  world-generation data to a client-only mod, so use `/pathfind goto` there.
- `/pathfind config` -- show saved search and movement settings.
- `/pathfind config <key> <value>` -- set and save one setting. Current settings cover
  `searchWeight`, `timeoutSeconds`, `maxExpansions`, `snapshotRadius`, `sprintSpeed`,
  `speedBridgeSpeed`, and `placeTime`.

## Scope

Computes and visualizes a route only -- it does not walk, mine, or place blocks for you.
No automatic target-switching or run-progression logic; every route starts from an
explicit command.

Before a search begins, `goto` snapshots the best pickaxe in the player inventory
(favoring durable tiers over a golden pickaxe) and the number of usable full blocks
available for bridging. It includes inventory, armor, and off-hand slots when the
game exposes them through the normal inventory container; bridge blocks are capped at
the engine's 255-block state limit.

## Rendering check

The route overlay uses Minecraft 26.2's render-state collection path
(`LevelRenderEvents.COLLECT_SUBMITS` and `submitCustomGeometry`) with the line render
type. The mod compiles against the current Fabric/Minecraft mappings; check it in a
dev client with `./gradlew runClient` before relying on it in a world.

## Building

```
./gradlew build
```

Output jar: `build/libs/nether-pathfinder-<version>.jar`.

## Running in a dev client

```
./gradlew runClient
```
