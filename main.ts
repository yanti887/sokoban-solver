// Sokoban solver for make.fun "Build the best Sokoban solver, 21 November 2026".
// Strategy: the levels are a 1-wide hallway with 1-4 independent rooms off it
// (boxes never leave their room), so solve each room push-optimally with A*
// and stitch the room solutions into one continuous legal playthrough.
//
// Per-room search: push-based A* (state = box positions + player region).
//   - Heuristic: Hungarian assignment (brute-force permutations, <=4 boxes) over
//     shortest-path box-to-goal distances. Admissible: each push moves one box
//     one step, and every box must reach a distinct goal.
//   - Deadlock pruning: dead-square table (reverse push graph from the goals)
//     and 2x2 wall/box blocks with no goal.
//   - Deterministic: fixed direction order, heap tie-break by (f, h, insertion
//     order), node budget by expansions, never the wall clock.
// Local run: mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Node expansion cap per room (deterministic, by work, never the clock).
// The rooms are tiny (<=6x6, <=4 boxes) and the optima are <=17 pushes, so
// the search finishes far below this; the cap is only a safety valve.
const MAX_EXPANSIONS_PER_ROOM = 2_000_000;

type Dir = { dx: number; dy: number; ch: string };
const DIRS: Dir[] = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
];

type Level = {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  goalSet: Uint8Array;
  boxes: number[];
  player: number;
};

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const wall = new Uint8Array(w * h).fill(1);
  const goalSet = new Uint8Array(w * h);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = 0;
  rows.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') { goalSet[i] = 1; goals.push(i); }
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, goalSet, boxes, player };
}

/** BFS distances from `from` over cells where passable[i] is 1. -1 = unreachable. */
function bfsDist(from: number, passable: Uint8Array, w: number): Int32Array {
  const dist = new Int32Array(passable.length).fill(-1);
  dist[from] = 0;
  const q = [from];
  for (let head = 0; head < q.length; head++) {
    const c = q[head]!;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (const d of DIRS) {
      const nx = cx + d.dx;
      const ny = cy + d.dy;
      if (nx < 0 || ny < 0) continue;
      const n = ny * w + nx;
      if (n >= passable.length || !passable[n] || dist[n] !== -1) continue;
      dist[n] = dist[c]! + 1;
      q.push(n);
    }
  }
  return dist;
}

/**
 * BFS from `from` over passable cells with `blocked` cells treated as walls.
 * Returns { region, parent, dist } where region is the sorted reachable cells
 * and parent maps each reached cell to the cell we came from (-1 for `from`).
 */
function bfsRegion(
  from: number,
  passable: Uint8Array,
  blocked: Uint8Array,
  w: number,
): { region: number[]; parent: Int32Array } {
  const parent = new Int32Array(passable.length).fill(-2);
  parent[from] = -1;
  const q = [from];
  const region = [from];
  for (let head = 0; head < q.length; head++) {
    const c = q[head]!;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (const d of DIRS) {
      const nx = cx + d.dx;
      const ny = cy + d.dy;
      if (nx < 0 || ny < 0) continue;
      const n = ny * w + nx;
      if (n >= passable.length || !passable[n] || blocked[n] || parent[n] !== -2) continue;
      parent[n] = c;
      region.push(n);
      q.push(n);
    }
  }
  region.sort((a, b) => a - b);
  return { region, parent };
}

/** Reconstruct the walk (lowercase moves) from `from` to `to` using a BFS parent map. */
function walkPath(from: number, to: number, parent: Int32Array, w: number): string {
  if (from === to) return '';
  const steps: string[] = [];
  let c = to;
  while (c !== from) {
    const p = parent[c]!;
    const d = c - p;
    steps.push(d === 1 ? 'r' : d === -1 ? 'l' : d === w ? 'd' : 'u');
    c = p;
  }
  steps.reverse();
  return steps.join('');
}

type Room = {
  cells: number[]; // sorted room floor cells (door included, hallway excluded)
  cellSet: Uint8Array; // 1 on room cells
  boxes: number[];
  goals: number[];
  door: number; // room cell adjacent to the hallway
  entry: number; // hallway cell adjacent to the door (player's entry point)
};

/** Split a level into its rooms: the hallway is the row the player starts on. */
function decompose(level: Level): Room[] {
  const { w, h, wall, player } = level;
  const hallRow = (player / w) | 0;
  // Floor cells excluding the hallway row, grouped into connected components.
  const seen = new Uint8Array(w * h);
  const rooms: Room[] = [];
  for (let i = 0; i < w * h; i++) {
    if (wall[i] || seen[i] || (i / w | 0) === hallRow) continue;
    // Flood-fill this component (a room).
    const cells: number[] = [];
    const q = [i];
    seen[i] = 1;
    for (let head = 0; head < q.length; head++) {
      const c = q[head]!;
      cells.push(c);
      const cx = c % w;
      const cy = (c / w) | 0;
      for (const d of DIRS) {
        const nx = cx + d.dx;
        const ny = cy + d.dy;
        if (nx < 0 || ny < 0 || ny >= h) continue;
        const n = ny * w + nx;
        if (wall[n] || seen[n] || (n / w | 0) === hallRow) continue;
        seen[n] = 1;
        q.push(n);
      }
    }
    cells.sort((a, b) => a - b);
    const cellSet = new Uint8Array(w * h);
    for (const c of cells) cellSet[c] = 1;
    const boxes = level.boxes.filter((b) => cellSet[b]);
    const goals = level.goals.filter((g) => cellSet[g]);
    // The door is the room cell adjacent to a hallway floor cell.
    let door = -1;
    let entry = -1;
    for (const c of cells) {
      const cy = (c / w) | 0;
      if (cy === hallRow - 1 || cy === hallRow + 1) {
        const n = (cy === hallRow - 1 ? c + w : c - w);
        if (!wall[n] && (n / w | 0) === hallRow) { door = c; entry = n; break; }
      }
    }
    if (door < 0) continue; // a component with no door is not a room (should not happen)
    rooms.push({ cells, cellSet, boxes, goals, door, entry });
  }
  // Deterministic order: top rooms first, then left to right.
  rooms.sort((a, b) => a.door - b.door);
  return rooms;
}

/**
 * Dead-square table for a room: a room cell is dead when a box on it (not on a
 * goal) can never reach a goal, even if the player could teleport. Computed by
 * reverse pushes from the goals: a box at u can be pushed to v when the player
 * can stand at 2u - v, so u is live when v is live and 2u - v is floor.
 * Returns a Uint8Array, 1 = dead.
 */
function deadSquares(room: Room, w: number, h: number, wall: Uint8Array): Uint8Array {
  const { cells, cellSet, goals, entry } = room;
  // Floor the player can stand on: room cells + the entry (hallway) cell.
  const floor = new Uint8Array(w * h);
  for (const c of cells) floor[c] = 1;
  floor[entry] = 1;
  const live = new Uint8Array(w * h);
  const q: number[] = [];
  for (const g of goals) { live[g] = 1; q.push(g); }
  for (let head = 0; head < q.length; head++) {
    const v = q[head]!;
    const vx = v % w;
    const vy = (v / w) | 0;
    for (const d of DIRS) {
      // u is the cell the box is pushed FROM to reach v: u = v - d.
      const ux = vx - d.dx;
      const uy = vy - d.dy;
      if (ux < 0 || uy < 0 || ux >= w || uy >= h) continue;
      const u = uy * w + ux;
      if (!cellSet[u] || live[u]) continue;
      // The player must stand at w2 = 2u - v = u - d to push u -> v.
      const wx = ux - d.dx;
      const wy = uy - d.dy;
      if (wx < 0 || wy < 0 || wx >= w || wy >= h) continue;
      const w2 = wy * w + wx;
      if (!floor[w2]) continue;
      live[u] = 1;
      q.push(u);
    }
  }
  const dead = new Uint8Array(w * h);
  for (const c of cells) {
    if (!live[c] && !room.goals.includes(c)) dead[c] = 1;
  }
  return dead;
}

/** 2x2 deadlock: a 2x2 block of walls/boxes with >=1 box and no goal. */
function has2x2Deadlock(boxes: number[], boxSet: Uint8Array, goalSet: Uint8Array, wall: Uint8Array, w: number, h: number): boolean {
  for (const b of boxes) {
    const bx = b % w;
    const by = (b / w) | 0;
    for (let oy = -1; oy <= 0; oy++) {
      for (let ox = -1; ox <= 0; ox++) {
        const x = bx + ox;
        const y = by + oy;
        if (x < 0 || y < 0 || x + 1 >= w || y + 1 >= h) continue;
        let nBox = 0;
        let nGoal = 0;
        let blocked = 0;
        for (let dy = 0; dy < 2; dy++) {
          for (let dx = 0; dx < 2; dx++) {
            const c = (y + dy) * w + (x + dx);
            if (goalSet[c]) nGoal++;
            if (boxSet[c]) nBox++;
            if (wall[c] || boxSet[c]) blocked++;
          }
        }
        if (blocked === 4 && nBox >= 1 && nGoal === 0) return true;
      }
    }
  }
  return false;
}

/** Hungarian assignment by brute-force permutations (n <= 4): min total distance. */
function hungarian(cost: number[][]): number {
  const n = cost.length;
  if (n === 0) return 0;
  const perm = Array.from({ length: n }, (_, i) => i);
  let best = Infinity;
  const rec = (k: number, sum: number): void => {
    if (sum >= best) return;
    if (k === n) { best = sum; return; }
    for (let j = k; j < n; j++) {
      [perm[k], perm[j]] = [perm[j]!, perm[k]!];
      const c = cost[k]![perm[k]!]!;
      if (c === Infinity) { [perm[k], perm[j]] = [perm[j]!, perm[k]!]; continue; }
      rec(k + 1, sum + c);
      [perm[k], perm[j]] = [perm[j]!, perm[k]!];
    }
  };
  rec(0, 0);
  return best;
}

type RoomCtx = {
  room: Room;
  w: number;
  h: number;
  wall: Uint8Array;
  goalSet: Uint8Array;
  // floor the player can stand on: room cells + entry cell
  floor: Uint8Array;
  // dead[c] = 1 when a box on c can never reach a goal
  dead: Uint8Array;
  // distToGoal[g][cell]: shortest path (room cells only) from cell to goal g
  distToGoal: Int32Array[];
};

function makeRoomCtx(room: Room, level: Level): RoomCtx {
  const { w, h, wall, goalSet } = level;
  const floor = new Uint8Array(w * h);
  for (const c of room.cells) floor[c] = 1;
  floor[room.entry] = 1;
  const dead = deadSquares(room, w, h, wall);
  // Distances on room cells only (boxes never leave the room).
  const roomFloor = new Uint8Array(w * h);
  for (const c of room.cells) roomFloor[c] = 1;
  const distToGoal = room.goals.map((g) => bfsDist(g, roomFloor, w));
  return { room, w, h, wall, goalSet, floor, dead, distToGoal };
}

type Node = {
  boxes: number[];
  player: number; // cell the player stands on (box's old cell after a push)
  regionMin: number; // canonical player key: min cell of the reachable region
  g: number; // pushes so far
  parent: number; // node index, -1 for the start
  moves: string; // moves from parent to this node (walk + push)
};

function solveRoom(ctx: RoomCtx, requireExit: boolean): string | null {
  const { room, w, h, wall, goalSet, floor, dead, distToGoal } = ctx;
  const n = room.boxes.length;
  const goals = room.goals;
  // Cost matrix helper: dist from a box cell to each goal.
  const heuristic = (boxes: number[]): number => {
    const cost: number[][] = [];
    for (const b of boxes) {
      const row: number[] = [];
      for (let j = 0; j < goals.length; j++) {
        const d = distToGoal[j]![b]!;
        row.push(d < 0 ? Infinity : d);
      }
      cost.push(row);
    }
    return hungarian(cost);
  };

  const boxSetOf = (boxes: number[]): Uint8Array => {
    const s = new Uint8Array(w * h);
    for (const b of boxes) s[b] = 1;
    return s;
  };

  const isSolved = (boxes: number[]): boolean => boxes.every((b) => goalSet[b]);

  // Initial state: player at the entry cell.
  const startBoxes = room.boxes.slice().sort((a, b) => a - b);
  const startBoxSet = boxSetOf(startBoxes);
  const startRegion = bfsRegion(room.entry, floor, startBoxSet, w);
  const startNode: Node = {
    boxes: startBoxes,
    player: room.entry,
    regionMin: startRegion.region[0]!,
    g: 0,
    parent: -1,
    moves: '',
  };
  const h0 = heuristic(startBoxes);
  if (h0 === Infinity) return null;

  const nodes: Node[] = [startNode];
  // Heap entries: [f, h, seq, nodeIdx]. Deterministic tie-break by seq.
  const heap: [number, number, number, number][] = [[h0, h0, 0, 0]];
  let seq = 1;
  const less = (a: [number, number, number, number], b: [number, number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e: [number, number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const heapPop = (): [number, number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };

  const best = new Map<string, number>();
  const keyOf = (boxes: number[], regionMin: number): string =>
    boxes.join(',') + '|' + regionMin;
  best.set(keyOf(startBoxes, startRegion.region[0]!), 0);

  let expanded = 0;
  while (heap.length > 0) {
    const [, , , s] = heapPop();
    const node = nodes[s]!;
    if (best.get(keyOf(node.boxes, node.regionMin)) !== s) continue;
    // Goal test: all boxes on goals, and (when required) the player can reach the door.
    if (isSolved(node.boxes)) {
      const boxSet = boxSetOf(node.boxes);
      const region = bfsRegion(node.player, floor, boxSet, w).region;
      if (!requireExit || region.includes(room.door)) {
        let moves = '';
        for (let t = s; t > 0; t = nodes[t]!.parent) moves = nodes[t]!.moves + moves;
        return moves;
      }
    }
    if (expanded >= MAX_EXPANSIONS_PER_ROOM) return null;
    expanded++;
    const boxSet = boxSetOf(node.boxes);
    const { region, parent } = bfsRegion(node.player, floor, boxSet, w);
    const regionSet = new Uint8Array(w * h);
    for (const c of region) regionSet[c] = 1;
    for (let bi = 0; bi < n; bi++) {
      const b = node.boxes[bi]!;
      const bx = b % w;
      const by = (b / w) | 0;
      for (const d of DIRS) {
        const tx = bx + d.dx;
        const ty = by + d.dy;
        if (tx < 0 || ty < 0 || tx >= w || ty >= h) continue;
        const target = ty * w + tx;
        // The box must stay inside the room (a box in the hallway never comes back).
        if (!room.cellSet[target] || boxSet[target]) continue;
        const behindX = bx - d.dx;
        const behindY = by - d.dy;
        if (behindX < 0 || behindY < 0 || behindX >= w || behindY >= h) continue;
        const behind = behindY * w + behindX;
        if (!regionSet[behind]) continue; // player cannot reach the push position
        // Apply the push.
        const nb = node.boxes.slice();
        nb[bi] = target;
        nb.sort((a, b2) => a - b2);
        // Deadlock pruning.
        if (dead[target]) continue;
        const nbSet = boxSetOf(nb);
        if (has2x2Deadlock(nb, nbSet, goalSet, wall, w, h)) continue;
        const hh = heuristic(nb);
        if (hh === Infinity) continue;
        // New player region after the push (player ends up at the box's old cell).
        const newRegion = bfsRegion(b, floor, nbSet, w);
        const regionMin = newRegion.region[0]!;
        const k = keyOf(nb, regionMin);
        const g2 = node.g + 1;
        const seen = best.get(k);
        if (seen !== undefined && nodes[seen]!.g <= g2) continue;
        const walk = walkPath(node.player, behind, parent, w);
        const t = nodes.length;
        nodes.push({
          boxes: nb,
          player: b,
          regionMin,
          g: g2,
          parent: s,
          moves: walk + d.ch.toUpperCase(),
        });
        best.set(k, t);
        heapPush([g2 + hh, hh, seq++, t]);
      }
    }
  }
  return null;
}

/** Solve one whole level by solving its rooms in order and stitching the walks. */
function solveLevel(level: Level): string {
  const { w, h, wall, goalSet, boxes, player } = level;
  const rooms = decompose(level);
  if (rooms.length === 0) return '';
  // Walk helper on the full level: BFS treating all current boxes as obstacles.
  const walkFull = (from: number, to: number, boxSet: Uint8Array): string | null => {
    const passable = new Uint8Array(w * h);
    for (let i = 0; i < w * h; i++) passable[i] = wall[i] ? 0 : 1;
    const { parent } = bfsRegion(from, passable, boxSet, w);
    if (parent[to] === -2) return null;
    return walkPath(from, to, parent, w);
  };
  const boxSetOf = (bs: number[]): Uint8Array => {
    const s = new Uint8Array(w * h);
    for (const b of bs) s[b] = 1;
    return s;
  };

  // Solve each room (exit required for all but the last, in room order).
  const roomMoves: (string | null)[] = [];
  for (let ri = 0; ri < rooms.length; ri++) {
    const room = rooms[ri]!;
    const ctx = makeRoomCtx(room, level);
    const requireExit = ri < rooms.length - 1;
    let moves = solveRoom(ctx, requireExit);
    // If the exit constraint costs pushes, try without it (this room may be the
    // one that ends the playthrough); the caller reorders only when it must.
    if (moves === null && requireExit) moves = solveRoom(ctx, false);
    roomMoves.push(moves);
  }
  if (roomMoves.some((m) => m === null)) return '';

  // Stitch: walk to each room's entry, play its solution, walk out.
  let moves = '';
  let cur = player;
  const curBoxes = boxes.slice();
  for (let ri = 0; ri < rooms.length; ri++) {
    const room = rooms[ri]!;
    const boxSet = boxSetOf(curBoxes);
    const toEntry = walkFull(cur, room.entry, boxSet);
    if (toEntry === null) return '';
    moves += toEntry;
    cur = room.entry;
    // Play the room solution on the global state, tracking the player and boxes.
    const rm = roomMoves[ri]!;
    for (const ch of rm) {
      const d = DIRS.find((dd) => dd.ch === ch.toLowerCase())!;
      const next = cur + d.dy * w + d.dx;
      const bi = curBoxes.indexOf(next);
      if (bi >= 0) {
        curBoxes[bi] = next + d.dy * w + d.dx;
        curBoxes.sort((a, b) => a - b);
      }
      cur = next;
    }
    moves += rm;
    // After the room, the player is inside it; the next walk starts from there.
  }
  // Final check: every box on a goal.
  if (!curBoxes.every((b) => goalSet[b])) return '';
  return moves;
}

for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(parseLevel(readFileSync(join(I, id, 'level.txt'), 'utf8')));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
