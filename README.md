[![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Mavesto-droid/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Mavesto-droid/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

# Sokoban Solver

A push-optimal Sokoban solver, built for [make.fun's "Build the best Sokoban solver, 21 November 2026" bounty](https://make.fun/bounty/sokoban-solver-01-run-20261121) (also at https://make.fun/c/sokoban-solver-01-run-20261121).

One program, `main.ts`, answers every instance of a drawn set in one run:
it reads `/input/<id>/level.txt` (XSB) and writes `/output/<id>/solution.txt`
(the player's moves in LURD).

## How it works

The drawn levels are a hallway with one to four small rooms off it (3–4
boxes each). A box pushed out of its room into the hallway can never come
back, so every box stays in its room and a level's push-optimal count P\* is
the sum of its rooms' optima. The solver therefore:

1. splits a level into rooms (cutting the hallway's dead squares out of the
   floor),
2. solves each room independently with a push-optimal A\* search over pushes
   — states are the boxes' squares plus the player's reachable region,
   pruned by dead squares and frozen 2×2 blocks, guided by an admissible
   minimum-cost box-to-goal assignment heuristic (push distances that ignore
   the other boxes), with re-opening so a state found again down a cheaper
   path keeps the cheaper path,
3. concatenates the rooms' move strings, the player walking between rooms
   through the hallway (walks are filled in with shortest-path BFS).

Fallbacks, all deterministic and budgeted by work rather than the clock
(two runs write the same bytes): greedy best-first search per room, a
whole-level push A\*, and finally a plain move A\* so an answer is still
produced rather than an illegal one.

## Results

On the kit's practice set it scores **1,000,000** — every instance solved in
exactly its push-optimal count (i001: 13, i002: 16, i003: 29, i004: 47,
i005: 60 pushes) in under a second for the whole set, well inside the 60 s
budget. It also scores 1,000,000 on seven further sets freshly drawn from
the published generator. The house baseline (plain-astar 1.0.0) scores
200,000 on the practice set, solving only i001.

## Run

```sh
node main.ts
```

Local run against the kit's practice set:

```sh
MAKEFUN_INPUT_DIR=<kit>/practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

Requires Node 22+ (the program is a single TypeScript file using erasable
syntax only; no dependencies, no network, no build step).

## Referral

Built with make.fun — referral code `j777ez47` (https://make.fun/r/j777ez47).
