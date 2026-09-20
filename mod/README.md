# Nether Pathfinder (beta)

> **Beta, built for a demo.** This mod exists to show the planner in action. You can try it in
> game, but the integration is rough: expect bugs, missing polish, and behavior that changes
> between commits. It has only been exercised in single-player on Minecraft 26.2 with Fabric.

A Fabric client mod for Minecraft 26.2 that plans a route through Nether terrain (sprint, mine,
bridge, boat-crawl, parkour and boat jumps, climb, fall, each weighted by a real time cost) and
draws it in the world for you to follow. It only **plans and draws**: it never moves you, mines,
or places blocks. That is the difference from an automation bot such as Baritone. This is a
planner, and the aim is to push what pathfinding can do for Minecraft agents, not to replace one.

It is built on the weighted-A* engine documented in [`../README.md`](../README.md) and
[`../java/README.md`](../java/README.md), including the "Air Potential" mining-prune heuristic.

## What you see

- A colored line along the route: cyan sprint, gold dig, magenta bridge, green climb, blue fall,
  purple crawl, orange parkour. Dig and bridge blocks are merged into a single outlined volume.
- Icons at each action: a pickaxe for digging, a block for bridging, the Jump Boost effect for a
  jump, an oak boat for a boat jump or crawl, a gold block at the goal. Far away they float at eye
  height at the start of the action. Within a few blocks they move onto the work: lying flat on
  the face of the next blocks to mine, or on top of the next block to place.
- A reveal animation: the route drops in piece by piece.
- Optional see-through drawing, so the route (or just its falling pieces) shows through terrain.

## Commands

| Command | What it does |
| --- | --- |
| `/pathfind goto <x> <y> <z>` | Plan to an explicit coordinate. |
| `/pathfind stop` | Clear the active route. |
| `/pathfind bastion` | Plan to the nearest bastion remnant (integrated single-player only). |
| `/pathfind demo` | Plan to the nearest bastion, or replay the cached route for this portal. |
| `/pathfind demo replan` / `replay` / `clear` | Ignore the cache and search again / replay the reveal animation / forget this world's cached routes. |
| `/pathfind xray [off\|falling\|full]` | See-through mode. `falling` (default): pieces show through terrain while they drop in, then settle normally. `full`: always see-through. `off`: normal depth. No argument cycles. |
| `/pathfind config` | Show settings. `/pathfind config <key> <value>` sets and saves one. |

## The Nether demo

In a single-player world, arriving in the Nether runs the demo automatically (turn it off with
`/pathfind config demoAutoRun 0`). It finds the nearest bastion, generates the terrain between the
portal and the bastion on the integrated server (so the bastion does not need to be within render
distance), plans the route, and plays the reveal. The finished route is cached per world and per
portal, so going through the same portal again replays it without searching.

Long routes are planned in legs of at most about 150 blocks, each searched separately and joined,
because one search across hundreds of blocks becomes intractable. A blocked leg is retried with a
sideways detour, then split in half. This is a greedy decomposition, so the result is good but not
guaranteed optimal. The search also caps its own memory use to what the machine has free.

The demo assumes a loadout: an iron pickaxe and 32 bridge blocks (or your real inventory if it is
better). Change it with `demoPickaxeTier` and `demoBridgeBlocks`.

## Settings

Stored in `config/nether-pathfinder.properties`.

- Search (`goto`): `searchWeight`, `timeoutSeconds`, `maxExpansions`, `snapshotRadius`,
  `sprintSpeed`, `speedBridgeSpeed`, `placeTime`.
- Demo: `demoAutoRun`, `ladderStartWeight`, `demoMaxExpansions`, `demoTimeoutSeconds`,
  `demoBridgeBlocks`, `demoPickaxeTier` (0 none, 1 wooden, 2 golden, 3 stone, 4 iron, 5 diamond,
  6 netherite).
- Display: `routeThroughWalls` (0 off, 1 falling pieces only, 2 always), `iconThroughWalls`,
  `iconMinDistance`, `iconMaxDistance`.

Setting one value with `/pathfind config` writes every setting to the file, and the file then
overrides future default changes.

If a leg of the demo route cannot be found, the mod writes that leg's exact terrain, start and
goals to `config/nether-pathfinder-debug/` as a `.wbin` file (the format described in
[`../java/README.md`](../java/README.md)), so the failure can be replayed offline.

## Known limits

- Beta. Rough edges are expected, and it may crash or draw incorrectly on setups I have not tried.
- The demo and `/pathfind bastion` need a single-player world. On a server, use `goto`.
- Routes are planned once and drawn; they do not react to the world changing under you, apart
  from dug and placed blocks dropping out of the icons.
- Very long routes can fail to plan. Plans past about 1500 blocks are refused.
- Rendering is by design a custom overlay. Other rendering mods can change how it looks.

## Building

Needs JDK 25.

```
./gradlew build
```

Output jar: `build/libs/nether-pathfinder-<version>.jar`. Drop it in a Fabric 26.2 instance's `mods`
folder with Fabric API. To run a dev client, use `./gradlew runClient`.
