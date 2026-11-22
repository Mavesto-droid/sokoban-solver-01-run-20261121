// Sokoban solver: push-optimal A* per room, with deadlock pruning.
//
// Strategy
// --------
// The levels this competition draws are a hallway with one to four small
// rooms off it (3-4 boxes each). A box pushed out of its room into the
// hallway can never come back, so every box stays in its room and a level's
// push-optimal count is the sum of its rooms' optima. This program therefore
// splits a level into rooms and solves each room independently with a
// push-optimal A* search, then concatenates the rooms' move strings (the
// player walks between rooms through the hallway).
//
// A search state is the boxes' squares plus the player's reachable region
// (named by its lowest square), exactly as in the classic push-based
// formulation: only pushes change the state, walks are filled in afterwards
// with a shortest-path BFS. The search prunes two deadlocks no solution can
// survive -- a box on a dead square (one from which no sequence of pushes,
// ignoring the other boxes, reaches any goal) and a frozen 2x2 block -- and
// is guided by an admissible heuristic: the minimum-cost assignment of
// boxes to goals under push distances that ignore the other boxes
// (enumerating the <= 24 assignments for <= 4 boxes). With an admissible
// heuristic, the first solved state popped from the open list is
// push-optimal, so each room is solved in its optimum and the whole level
// in P*.
//
// Fallbacks (all deterministic, budgeted by work, never by the clock, so two
// runs write the same bytes): if a room's A* exhausts its state budget, a
// greedy best-first search tries for any solution of that room; if the level
// does not split into rooms, a push A* runs over the whole level; and if
// that also gives up, a plain move A* (fewest moves) tries to at least
// produce a valid, solving answer. A level no fallback solves gets an empty
// answer rather than an illegal one.
//
// Usage: node main.ts, reading /input/<id>/level.txt and writing
// /output/<id>/solution.txt; MAKEFUN_INPUT_DIR / MAKEFUN_OUTPUT_DIR
// override the directories for local runs. Node 22+ (type stripping).
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Work budgets (state expansions), never the wall clock: the grader runs
// each entry twice and the two outputs must match byte for byte.
const ROOM_ASTAR_STATES = 300_000;
const ROOM_GREEDY_STATES = 300_000;
const LEVEL_ASTAR_STATES = 300_000;
const MOVE_ASTAR_STATES = 200_000;

type Dir = [number, number, string, string];
const DIRS: Dir[] = [
  [0, -1, 'u', 'U'],
  [0, 1, 'd', 'D'],
  [-1, 0, 'l', 'L'],
  [1, 0, 'r', 'R'],
];

interface Level {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x]!;
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
    }
  }
  return { w, h, wall, goals, boxes: boxes.sort((a, b) => a - b), player };
}

/** Squares from which a box alone can be pushed onto some goal. */
function liveSquares(level: Level): Uint8Array {
  const { w, wall, goals } = level;
  const live = new Uint8Array(wall.length);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi]!;
    for (const [dx, dy] of DIRS) {
      const d = dy * w + dx;
      const y = x - d;
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

/**
 * Push distances ignoring other boxes: dist[s * G + g] is the fewest pushes
 * that move a lone box from square s onto goal goals[g] (Infinity when the
 * box can never get there). A reverse BFS from each goal over pushes.
 */
function pushDistances(level: Level): { dist: Float64Array; nGoals: number } {
  const { w, wall, goals } = level;
  const size = wall.length;
  const nGoals = goals.length;
  const dist = new Float64Array(size * nGoals).fill(Infinity);
  for (let gi = 0; gi < nGoals; gi++) {
    const g = goals[gi]!;
    const off = gi;
    dist[g * nGoals + off] = 0;
    const queue = [g];
    for (let qi = 0; qi < queue.length; qi++) {
      const x = queue[qi]!;
      const dcur = dist[x * nGoals + off]!;
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx;
        const y = x - d; // square the box is pushed from
        if (wall[y] || wall[y - d]) continue; // player must stand at y - d
        if (dist[y * nGoals + off]! !== Infinity) continue;
        dist[y * nGoals + off] = dcur + 1;
        queue.push(y);
      }
    }
  }
  return { dist, nGoals };
}

/** The squares the player reaches from `from` with `occupied` in the way. */
function reach(
  level: Level,
  occupied: Uint8Array,
  from: number,
  seen: Int32Array,
  stamp: number,
  stack: Int32Array,
): number {
  const { w, wall } = level;
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n]!;
    if (p < low) low = p;
    const q0 = p - w;
    const q1 = p + w;
    const q2 = p - 1;
    const q3 = p + 1;
    if (wall[q0] === 0 && occupied[q0] === 0 && seen[q0] !== stamp) { seen[q0] = stamp; stack[n++] = q0; }
    if (wall[q1] === 0 && occupied[q1] === 0 && seen[q1] !== stamp) { seen[q1] = stamp; stack[n++] = q1; }
    if (wall[q2] === 0 && occupied[q2] === 0 && seen[q2] !== stamp) { seen[q2] = stamp; stack[n++] = q2; }
    if (wall[q3] === 0 && occupied[q3] === 0 && seen[q3] !== stamp) { seen[q3] = stamp; stack[n++] = q3; }
  }
  return low;
}

/** A box just pushed to `t` inside a frozen 2x2 block of walls/boxes. */
function frozenBlock(level: Level, occupied: Uint8Array, goal: Uint8Array, t: number): boolean {
  const { w, wall } = level;
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b]) || !(wall[c] || occupied[c]) || !(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goal[a]) || (occupied[b] && !goal[b]) || (occupied[c] && !goal[c]) || (occupied[d] && !goal[d])) return true;
  }
  return false;
}

/** Shortest walk from `from` to `to` around walls and boxes (lower LURD). */
function walkTo(level: Level, occupied: Uint8Array, from: number, to: number): string {
  const { w, wall } = level;
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how: string[] = new Array(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi]!;
    if (p === to) break;
    for (const [dx, dy, letter] of DIRS) {
      const q = p + dy * w + dx;
      if (wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = letter;
      queue.push(q);
    }
  }
  if (prev[to] === -1) throw new Error('walkTo: the push square is out of reach');
  let s = '';
  for (let p = to; p !== from; p = prev[p]!) s = how[p]! + s;
  return s;
}

function endOf(w: number, from: number, moves: string): number {
  let p = from;
  for (const ch of moves.toLowerCase()) p += ch === 'u' ? -w : ch === 'd' ? w : ch === 'l' ? -1 : 1;
  return p;
}

/** Minimum-cost assignment of boxes to goals under push distances. */
function assignmentHeuristic(
  boxes: number[],
  dist: Float64Array,
  nGoals: number,
): number {
  const n = boxes.length;
  if (n === 0) return 0;
  const perm: number[] = [];
  for (let i = 0; i < nGoals; i++) perm.push(i);
  let best = Infinity;
  // Enumerate injective assignments by recursion over boxes (n <= 4 here).
  const used = new Uint8Array(nGoals);
  const rec = (bi: number, cost: number): void => {
    if (cost >= best) return;
    if (bi === n) {
      best = cost;
      return;
    }
    const b = boxes[bi]!;
    for (let gi = 0; gi < nGoals; gi++) {
      if (used[gi]) continue;
      const d = dist[b * nGoals + gi]!;
      if (d === Infinity) continue;
      used[gi] = 1;
      rec(bi + 1, cost + d);
      used[gi] = 0;
    }
  };
  rec(0, 0);
  return best;
}

/** Sum over boxes of the distance to the nearest goal (cheap, admissible). */
function nearestHeuristic(boxes: number[], dist: Float64Array, nGoals: number): number {
  let sum = 0;
  for (const b of boxes) {
    let m = Infinity;
    for (let gi = 0; gi < nGoals; gi++) {
      const d = dist[b * nGoals + gi]!;
      if (d < m) m = d;
    }
    if (m === Infinity) return Infinity;
    sum += m;
  }
  return sum;
}

interface SearchResult {
  pushes: number;
  moves: string;
}

// A binary heap ordered by (f, h, -g, seq): lowest f first, then lowest h,
// then deepest, then earliest inserted -- a total order, so runs agree.
class Heap {
  // No `private` modifiers: the grader's Node 22 strips types, and plain
  // fields keep this file to erasable syntax only.
  a: [number, number, number, number, number][] = [];
  get size(): number {
    return this.a.length;
  }
  push(e: [number, number, number, number, number]): void {
    const a = this.a;
    let i = a.length;
    a.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!Heap.less(a[i]!, a[p]!)) break;
      const t = a[i]!; a[i] = a[p]!; a[p] = t;
      i = p;
    }
  }
  pop(): [number, number, number, number, number] {
    const a = this.a;
    const top = a[0]!;
    const last = a.pop()!;
    if (a.length > 0) {
      a[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= a.length) break;
        if (c + 1 < a.length && Heap.less(a[c + 1]!, a[c]!)) c++;
        if (!Heap.less(a[c]!, a[i]!)) break;
        const t = a[i]!; a[i] = a[c]!; a[c] = t;
        i = c;
      }
    }
    return top;
  }
  static less(x: [number, number, number, number, number], y: [number, number, number, number, number]): boolean {
    return x[0] < y[0]
      || (x[0] === y[0] && (x[1] < y[1]
        || (x[1] === y[1] && (x[2] > y[2]
          || (x[2] === y[2] && x[3] < y[3])))));
  }
}

/**
 * Push search over a level: A* when `optimal` (admissible heuristic, first
 * solved state popped is push-optimal), greedy best-first otherwise (h only,
 * finds a solution fast but not necessarily optimal). Returns null when the
 * state budget runs out or no solution exists.
 */
function solvePush(
  level: Level,
  maxStates: number,
  optimal: boolean,
): SearchResult | null {
  const { w, wall, goals } = level;
  const size = wall.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(level);
  const { dist, nGoals } = pushDistances(level);
  const useAssignment = level.boxes.length <= 4;
  const heuristic = (boxes: number[]): number =>
    useAssignment ? assignmentHeuristic(boxes, dist, nGoals) : nearestHeuristic(boxes, dist, nGoals);
  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;
  const solved = (boxes: number[]): boolean => boxes.every((b) => goal[b]);
  if (level.boxes.some((b) => !live[b])) return null;

  const keyOf = (boxes: number[], low: number): number => {
    let k = low;
    for (const b of boxes) k = k * size + b;
    return k;
  };

  const statesBoxes: number[][] = [level.boxes];
  const statesPlayer: number[] = [level.player];
  const parent: number[] = [-1];
  const gScore: number[] = [0];
  const index = new Map<number, number>();
  for (const b of level.boxes) occupied[b] = 1;
  const startLow = reach(level, occupied, level.player, seen, ++stamp, stack);
  for (const b of level.boxes) occupied[b] = 0;
  index.set(keyOf(level.boxes, startLow), 0);
  const h0 = heuristic(level.boxes);
  if (h0 === Infinity) return null;
  if (solved(level.boxes)) return { pushes: 0, moves: '' };

  const heap = new Heap();
  let seq = 0;
  heap.push([optimal ? h0 : h0, h0, 0, seq++, 0]);
  let expanded = 0;
  let found = -1;
  while (heap.size > 0 && expanded < maxStates) {
    const [, , g, , s] = heap.pop();
    if (gScore[s] !== g) continue; // stale entry
    const boxes = statesBoxes[s]!;
    if (solved(boxes)) {
      found = s;
      break;
    }
    expanded++;
    for (const b of boxes) occupied[b] = 1;
    reach(level, occupied, statesPlayer[s]!, seen, ++stamp, stack);
    const here = stamp;
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi]!;
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx;
        const t = b + d;
        // The player must reach the square behind the box.
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(level, occupied, goal, t);
        let key = -1;
        let next: number[] | null = null;
        let h = Infinity;
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((x, y) => x - y);
          h = heuristic(next);
          if (h !== Infinity) {
            key = keyOf(next, reach(level, occupied, b, mark, ++stamp, stack));
          }
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen || h === Infinity) continue;
        const ng = g + 1;
        // A state first reached is not always reached cheapest: A* with an
        // admissible heuristic stays optimal only when a state found again
        // down a cheaper path is re-opened (its parent and g updated, a
        // fresh heap entry pushed; stale entries are skipped on pop by the
        // gScore check). Closing a state on first generation instead can
        // keep a longer path and lose push-optimality.
        const existing = index.get(key);
        if (existing !== undefined) {
          if (gScore[existing]! <= ng) continue;
          parent[existing] = s;
          gScore[existing] = ng;
          heap.push([optimal ? ng + h : h, h, ng, seq++, existing]);
          continue;
        }
        const t2 = statesBoxes.length;
        index.set(key, t2);
        statesBoxes.push(next!);
        statesPlayer.push(b);
        parent.push(s);
        gScore.push(ng);
        const f = optimal ? ng + h : h;
        heap.push([f, h, ng, seq++, t2]);
      }
    }
    for (const x of boxes) occupied[x] = 0;
  }
  if (found < 0) return null;
  const path: number[] = [];
  for (let s = found; s > 0; s = parent[s]!) path.push(s);
  path.reverse();
  let moves = '';
  let player = level.player;
  let boxes = level.boxes;
  for (const s of path) {
    const nextBoxes = statesBoxes[s]!;
    const from = boxes.find((b) => !nextBoxes.includes(b))!;
    const to = nextBoxes.find((b) => !boxes.includes(b))!;
    const d = to - from;
    const dir = DIRS.find(([dx, dy]) => dy * w + dx === d)!;
    for (const b of boxes) occupied[b] = 1;
    moves += walkTo(level, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    moves += dir[3];
    player = from;
    boxes = nextBoxes;
  }
  return { pushes: path.length, moves };
}

interface Room {
  squares: number[];
  boxes: number[];
  goals: number[];
  done: boolean;
}

/**
 * The level's rooms: cut the dead squares of the player's row (the hallway:
 * a box there reaches no goal) out of the floor; each remaining part that
 * holds boxes is a room. Null when the level is no set of rooms.
 */
function roomsOf(level: Level): Room[] | null {
  const { w, wall, goals, boxes, player } = level;
  const live = liveSquares(level);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) if (!wall[i] && !live[i]) cut[i] = 1;
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms: Room[] = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (const [dx, dy] of DIRS) {
        const q = squares[i]! + dy * w + dx;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ squares, boxes: [], goals: [], done: false });
  }
  for (const b of boxes) rooms[part[b]!]!.boxes.push(b);
  for (const g of goals) if (part[g]! >= 0) rooms[part[g]!]!.goals.push(g);
  if (goals.some((g) => part[g]! < 0) || rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

/** Solve a level room by room; null when any room gives up. */
function solveByRooms(level: Level): SearchResult | null {
  const rooms = roomsOf(level);
  if (rooms === null) return null;
  let player = level.player;
  let pushes = 0;
  let moves = '';
  for (const room of rooms) {
    const wall = level.wall.slice();
    for (const other of rooms) {
      if (other !== room) for (const b of other.done ? other.goals : other.boxes) wall[b] = 1;
    }
    const sub: Level = {
      w: level.w,
      h: level.h,
      wall,
      goals: room.goals.slice().sort((a, b) => a - b),
      boxes: room.boxes.slice().sort((a, b) => a - b),
      player,
    };
    let solved = solvePush(sub, ROOM_ASTAR_STATES, true);
    if (solved === null) solved = solvePush(sub, ROOM_GREEDY_STATES, false);
    if (solved === null) return null;
    pushes += solved.pushes;
    moves += solved.moves;
    player = endOf(level.w, player, solved.moves);
    room.done = true;
  }
  return { pushes, moves };
}

/** Plain move A*: fewest moves (not pushes), a last-resort valid answer. */
function solveMoves(level: Level, maxStates: number): SearchResult | null {
  const { w, wall, goals } = level;
  const goal = new Set(goals);
  const dist = (b: number): number =>
    Math.min(...goals.map((g) => Math.abs((g % w) - (b % w)) + Math.abs(Math.floor(g / w) - Math.floor(b / w))));
  const est = (bs: number[]): number => bs.reduce((a, b) => a + dist(b), 0);
  const key = (p: number, bs: number[]): string => `${p}|${bs.join(',')}`;
  const sp: number[] = [level.player];
  const sb: number[][] = [level.boxes];
  const sg: number[] = [0];
  const parent: number[] = [-1];
  const step: string[] = [''];
  const best = new Map<string, number>([[key(level.player, level.boxes), 0]]);
  const heap = new Heap();
  let seq = 0;
  heap.push([est(level.boxes), est(level.boxes), 0, seq++, 0]);
  let expanded = 0;
  while (heap.size > 0 && expanded < maxStates) {
    const [, , g, , s] = heap.pop();
    if (sg[s] !== g) continue;
    const bs = sb[s]!;
    if (bs.every((b) => goal.has(b))) {
      let moves = '';
      let pushes = 0;
      for (let t = s; t > 0; t = parent[t]!) {
        moves = step[t]! + moves;
        if (step[t]! === step[t]!.toUpperCase()) pushes++;
      }
      return { pushes, moves };
    }
    expanded++;
    const p = sp[s]!;
    for (const [dx, dy, letter] of DIRS) {
      const d = dy * w + dx;
      const q = p + d;
      if (wall[q]) continue;
      let nb = bs;
      let move = letter;
      const at = bs.indexOf(q);
      if (at >= 0) {
        if (wall[q + d] || bs.includes(q + d)) continue;
        nb = bs.slice();
        nb[at] = q + d;
        nb.sort((a, b) => a - b);
        move = letter.toUpperCase();
      }
      const k = key(q, nb);
      const seen = best.get(k);
      if (seen !== undefined && sg[seen]! <= g + 1) continue;
      const t = sp.length;
      sp.push(q);
      sb.push(nb);
      sg.push(g + 1);
      parent.push(s);
      step.push(move);
      best.set(k, t);
      const h = est(nb);
      heap.push([g + 1 + h, h, g + 1, seq++, t]);
    }
  }
  return null;
}

function solve(text: string): string {
  const level = parseLevel(text);
  try {
    const byRooms = solveByRooms(level);
    if (byRooms !== null) return byRooms.moves;
  } catch {
    // A room's walk failed or similar: fall through to whole-level search.
  }
  const whole = solvePush(level, LEVEL_ASTAR_STATES, true) ?? solvePush(level, LEVEL_ASTAR_STATES, false);
  if (whole !== null) return whole.moves;
  const byMoves = solveMoves(level, MOVE_ASTAR_STATES);
  return byMoves !== null ? byMoves.moves : '';
}

for (const id of readdirSync(I).sort()) {
  const moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
