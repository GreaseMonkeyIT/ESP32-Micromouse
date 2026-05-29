# Micromouse — Improvement Guide
> Ordered by impact. All changes stay close to the existing code structure.

---

## Already Fixed (this session)

| File | Fix |
|------|-----|
| Algorithm_Integration.ino | `memset(visited, false, sizeof(visited))` at top of `floodFillUsingQueue()` |
| API.ino | Removed extra `}` closing brace in `moveForward()` — was a compile error |
| API.ino + Algorithm_Integration.ino | `Wire.begin()` / `mpu.begin()` / `mpu.calcOffsets()` moved to `setup()`, removed from `turnRight()` / `turnLeft()` |

---

## High Priority — Structural fixes that were needed on the robot

### 1. Reciprocal wall consistency in `isAccessible()`

**Problem:** `isAccessible(A, B)` only reads `cellsArray[A]` — it never checks `cellsArray[B]`. A wall recorded from B's perspective (e.g. B's south wall when checking B→A) is invisible. This can cause the flood fill to route through a wall that physically exists.

**Status:** Solved in the hardware-integration rewrite by replacing the old
integer state encoding with absolute 4-bit wall masks plus reciprocal wall
writes in `setWall()`.

```cpp
bool isAccessible(int cx, int cy, int tx, int ty) {
    // existing check on cellsArray[cx][cy] ...

    // Also check from the target cell's side (reciprocal wall):
    // if moving north into target, check target has no south wall, etc.
    // Simplest: call the same direction logic with (tx,ty) and the reversed target (cx,cy)
    // (swap arguments and flip the comparison axis)
}
```

The cleanest long-term fix was switching to bitmasks (see §3). The simulator
code still preserves the older layout for reference.

---

### 2. `prevAngle` drift across turns

**Problem:** `turnRight()` reads `rawAngle = abs(mpu.getAngleZ())` then sets `targetAngle = prevAngle + 85`. The MPU6050 accumulates angle from startup — so each turn adds 85° to the running total and in theory this is correct. **But** if the robot overshoots or the gyro drifts, the error compounds indefinitely: turn 5's target depends on turns 1–4 being accurate.

**Status:** Implemented. The turn functions now track angle *delta* per turn,
not absolute accumulation.

```cpp
void turnRight() {
    mpu.update();
    float startAngle = mpu.getAngleZ();   // absolute at start of THIS turn
    float target = startAngle + 85.0;     // +85° from now, not from all history

    while (mpu.getAngleZ() < target) {
        mpu.update();
        float correction = (target - mpu.getAngleZ()) * correctionFactor;
        Right();
        analogWrite(ENA, max((int)(baseSpeed + correction), 100));
        analogWrite(ENB, max((int)(baseSpeed - correction), 100));
    }
    Stop();
    // prevAngle no longer needed
}
```

---

### 3. Wall encoding — switch to 4-bit bitmasks

**Problem:** The current integer codes 1–14 are orientation-relative. `updateCells()` is a 60-line if-else tree. `isAccessible()` is another 40-line lookup. Adding a new wall condition or debugging a routing error means tracing through this by hand. It also doesn't store walls symmetrically (see §1).

**Status:** Implemented in the hardware-integration code. `cellsArray` was
replaced there with absolute 4-bit wall flags:

```cpp
#define WALL_N  0x01
#define WALL_E  0x02
#define WALL_S  0x04
#define WALL_W  0x08

uint8_t walls[16][16] = {0};  // replace cellsArray

// Setting a wall:
void setWall(int x, int y, int absoluteDir) {
    walls[x][y] |= absoluteDir;
    // also set reciprocal on neighbour:
    if (absoluteDir == WALL_N && y+1 < 16) walls[x][y+1] |= WALL_S;
    if (absoluteDir == WALL_S && y-1 >= 0) walls[x][y-1] |= WALL_N;
    if (absoluteDir == WALL_E && x+1 < 16) walls[x+1][y] |= WALL_W;
    if (absoluteDir == WALL_W && x-1 >= 0) walls[x-1][y] |= WALL_E;
}

// Checking access:
bool isAccessible(int cx, int cy, int tx, int ty) {
    if (tx == cx+1) return !(walls[cx][cy] & WALL_E);
    if (tx == cx-1) return !(walls[cx][cy] & WALL_W);
    if (ty == cy+1) return !(walls[cx][cy] & WALL_N);
    if (ty == cy-1) return !(walls[cx][cy] & WALL_S);
    return false;
}
```

`updateCells()` was replaced by `updateWalls()`, which converts sensor readings
plus orientation to absolute direction before calling `setWall()`. This cut a
large orientation-relative lookup tree down to a compact mapping.

---

## Medium Priority — Performance and robustness

### 4. Sensor averaging delay

**Problem:** `getDistance()` does 5 pings × 30ms delay × 3 sensors = up to 450ms blocked per loop iteration. At that rate the robot can only process ~2 cells/second.

**Status:** Quick fix implemented. `AVG_NUM` was dropped from 5 to 3 and
`delay(30)` to `delay(20)` → ~180ms per loop. Minimal accuracy loss since
walls don't move.

**Future upgrade:** Use `NewPing`'s non-blocking timer mode — fire all 3
sensors in parallel and read results on next loop iteration rather than
blocking.

---

### 5. Queue encoding

**Current:** x and y pushed separately — two `push()` and two `pop()` per cell.  
**Status:** Implemented in the hardware-integration code. Queue entries are now
encoded as a single int, which halves queue operations:

```cpp
// push
cellQueue.push(x * 16 + y);

// pop
int cell = cellQueue.front(); cellQueue.pop();
int current_X = cell / 16;
int current_Y = cell % 16;
```

---

### 6. `whereToMove` tie-breaking — prefer straight ahead

When multiple accessible neighbours have the same flood value, the current code
used to pick the last one found. The hardware-integration version now adds a
straight-ahead preference to avoid unnecessary turns and save time:

```cpp
// After finding minimum value, prefer the direction matching current orient:
if (accessibleNeighbors[orient] == minValue) {
    minCell = orient;  // go straight if it's equally good
}
```

---

## Low Priority — Nice to have

### 7. Return-to-start / speed run

Once the goal is reached, re-run flood fill with the **start cell** as the new goal (set `floodArray[0][0] = 0`). The mouse now knows the maze fully and can find the fastest return path. Standard competition strategy: explore → reach centre → sprint back on shortest known path.

### 8. Maze boundary walls

**Status:** Implemented in the hardware-integration code via
`initBoundaryWalls()` in `setup()`.

### 9. Serial.println in loop()

`Serial.println("Wall Left")` etc. inside `loop()` adds UART overhead every iteration. Remove or gate behind a `#define DEBUG` flag before competition.

---

## What's Already Good

- **MFFA over basic BFS** — you're using the right algorithm. Most beginner teams do a full re-flood every step; you only propagate inconsistencies.
- **Encoder-based `moveForward()`** — not relying on fixed delays, which would break as battery depletes.
- **`whereToMove` backtrack prevention** — the `previous` direction check stops the mouse from oscillating.
- **Proportional correction in turns** — `(targetAngle - angle) * correctionFactor` reduces overshoot as the target approaches. Simple and effective.

## Note On Code Versions

- `Algorithm Integration/` reflects the improved hardware-side implementation.
- `Algorithm Simulation/` still shows the earlier simulator-era architecture
  using `cellsArray` and orientation-relative wall encodings.
- This repo therefore captures both the original project structure and the
  later rewrite.
