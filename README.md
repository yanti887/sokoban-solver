# Sokoban Solver

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/yanti887/sokoban-solver/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/yanti887/sokoban-solver/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A Sokoban solver for make.fun's "Build the best Sokoban solver, 21 November 2026" bounty. It solves each level push-optimally and scores **1,000,000 / 1,000,000** on the practice set (all 5 levels at exactly P* pushes), far above the plain-astar baseline (200,000).

## Approach

The levels are a 1-wide hallway with 1–4 independent rooms off it; boxes never leave their room. So each room is solved push-optimally on its own, then the room solutions are stitched into one continuous legal playthrough.

Per-room search: push-based A* (state = box positions + player reachable region).
- **Hungarian heuristic** — brute-force permutations of box→goal assignment over shortest-path distances; admissible, so A* finds the true push-optimal count.
- **Deadlock pruning** — dead-square table (reverse push graph from the goals) + 2×2 wall/box blocks with no goal.
- **Deterministic** — fixed direction order, heap tie-break by (f, h, insertion order), node-expansion budget (never the wall clock).

## Run it

```sh
mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

Reads `/input/<id>/level.txt` for each instance, writes `/output/<id>/solution.txt` (LURD moves).

## Verification

- Practice set: **1,000,000 / 1,000,000** — all 5 levels at exactly P* (13, 16, 29, 47, 60 pushes). Baseline plain-astar: 200,000.
- 3 freshly generated sets (new seeds): all **1,000,000** too.
- Determinism: two runs produce byte-identical solution files.
- Wall time: 0.63s for the whole 5-level practice set (limit 60s).

---

Entered on [make.fun](https://make.fun/?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards
