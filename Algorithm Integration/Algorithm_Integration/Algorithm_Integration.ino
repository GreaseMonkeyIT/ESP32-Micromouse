#include <iostream>
#include <string>
#include <queue>
#include "API.h"
#include <stdio.h>


// ── Orientation constants ──────────────────────────────────────────────────
#define FORWARD   0   // facing North (+y)
#define RIGHT     1   // facing East  (+x)
#define BACKWARD  2   // facing South (-y)
#define LEFT      3   // facing West  (-x)

// ── Wall bitmask flags (absolute directions) ──────────────────────────────
// Improvement from original: 4-bit absolute bitmasks replace the integer code scheme.
// setWall() always writes both sides of the wall, so isAccessible() only needs to read one.
// This also eliminates the orientation-relative lookup tables in the old updateCells().
#define WALL_N  0x01   // north (+y)
#define WALL_E  0x02   // east  (+x)
#define WALL_S  0x04   // south (-y)
#define WALL_W  0x08   // west  (-x)

// walls[x][y] stores which sides of cell (x,y) have confirmed walls.
uint8_t walls[16][16] = {0};

int Orient = 0;

// Flood distances
int floodArray[16][16] = {
    {14,13,12,11,10, 9, 8, 7, 7, 8, 9,10,11,12,13,14},
    {13,12,11,10, 9, 8, 7, 6, 6, 7, 8, 9,10,11,12,13},
    {12,11,10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9,10,11,12},
    {11,10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9,10,11},
    {10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9,10},
    { 9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9},
    { 8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8},
    { 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7},
    { 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7},
    { 8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8},
    { 9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9},
    {10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9,10},
    {11,10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9,10,11},
    {12,11,10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9,10,11,12},
    {13,12,11,10, 9, 8, 7, 6, 6, 7, 8, 9,10,11,12,13},
    {14,13,12,11,10, 9, 8, 7, 7, 8, 9,10,11,12,13,14}
};


/**================================================================
 * @Fn      - setWall
 * @brief   - Records a wall bit on cell (x,y) and its reciprocal on the
 *            neighbour. Keeps both sides of every wall in sync automatically.
 */
void setWall(int x, int y, uint8_t dir) {
    walls[x][y] |= dir;
    if ((dir & WALL_N) && y + 1 < 16) walls[x][y + 1] |= WALL_S;
    if ((dir & WALL_S) && y - 1 >= 0) walls[x][y - 1] |= WALL_N;
    if ((dir & WALL_E) && x + 1 < 16) walls[x + 1][y] |= WALL_W;
    if ((dir & WALL_W) && x - 1 >= 0) walls[x - 1][y] |= WALL_E;
}


/**================================================================
 * @Fn      - updateWalls (replaces updateCells)
 * @brief   - Converts the three sensor booleans + current orientation into
 *            absolute wall directions and calls setWall(). No more integer
 *            code lookup table.
 *
 * Absolute direction lookup [orient][relative]:
 *   relative index 0=front, 1=right, 2=back, 3=left
 */
void updateWalls(int x, int y, int orient, bool leftWall, bool rightWall, bool frontWall) {
    // absDir[orient][relative] = absolute wall bit
    const uint8_t absDir[4][4] = {
        // front      right      back       left
        { WALL_N,   WALL_E,   WALL_S,   WALL_W },   // FORWARD (N)
        { WALL_E,   WALL_S,   WALL_W,   WALL_N },   // RIGHT   (E)
        { WALL_S,   WALL_W,   WALL_N,   WALL_E },   // BACKWARD(S)
        { WALL_W,   WALL_N,   WALL_E,   WALL_S },   // LEFT    (W)
    };

    if (frontWall) setWall(x, y, absDir[orient][0]);
    if (rightWall) setWall(x, y, absDir[orient][1]);
    if (leftWall)  setWall(x, y, absDir[orient][3]);
    // back wall sensing not required; since robot just came through there
}


/**================================================================
 * @Fn      - isAccessible
 * @brief   - Returns true if no wall blocks movement from (cx,cy) to (tx,ty).
 *            Single bitmask read, no orientation lookup.
 *            Because setWall() always writes both sides, reading one side is optimal.
 */
bool isAccessible(int cx, int cy, int tx, int ty) {
    if (tx == cx + 1) return !(walls[cx][cy] & WALL_E);
    if (tx == cx - 1) return !(walls[cx][cy] & WALL_W);
    if (ty == cy + 1) return !(walls[cx][cy] & WALL_N);
    if (ty == cy - 1) return !(walls[cx][cy] & WALL_S);
    return false;
}


/**================================================================
 * @Fn      - getSurroundings
 * @brief   - Computes coordinates of all four neighbours. Out of bounds
 *            neighbours are set to -1 (caught by >= 0 checks in callers).
 */
void getSurroundings(int cx, int cy,
                     int *nx, int *ny,
                     int *ex, int *ey,
                     int *sx, int *sy,
                     int *wx, int *wy) {
    *nx = cx;       *ny = (cy + 1 < 16) ? cy + 1 : -1;
    *ex = (cx + 1 < 16) ? cx + 1 : -1; *ey = cy;
    *sx = cx;       *sy = cy - 1;   // -1 caught by caller
    *wx = cx - 1;   *wy = cy;       // -1 caught by caller
}


/**================================================================
 * @Fn      - isIncrementConsistent
 * @brief   - True if at least one accessible neighbour has value == (current - 1).
 */
bool isIncrementConsistent(int cx, int cy) {
    int nx, ny, ex, ey, sx, sy, wx, wy;
    getSurroundings(cx, cy, &nx, &ny, &ex, &ey, &sx, &sy, &wx, &wy);

    int cur = floodArray[cx][cy];
    int vals[4] = {1000, 1000, 1000, 1000};

    if (nx >= 0 && ny >= 0 && isAccessible(cx, cy, nx, ny)) vals[FORWARD]  = floodArray[nx][ny];
    if (ex >= 0 && ey >= 0 && isAccessible(cx, cy, ex, ey)) vals[RIGHT]    = floodArray[ex][ey];
    if (sx >= 0 && sy >= 0 && isAccessible(cx, cy, sx, sy)) vals[BACKWARD] = floodArray[sx][sy];
    if (wx >= 0 && wy >= 0 && isAccessible(cx, cy, wx, wy)) vals[LEFT]     = floodArray[wx][wy];

    for (int i = 0; i < 4; i++)
        if (vals[i] == cur - 1) return true;
    return false;
}


/**================================================================
 * @Fn      - makeCellConsistent
 * @brief   - Sets cell value to min(accessible neighbours) + 1.
 */
void makeCellConsistent(int cx, int cy) {
    int nx, ny, ex, ey, sx, sy, wx, wy;
    getSurroundings(cx, cy, &nx, &ny, &ex, &ey, &sx, &sy, &wx, &wy);

    int vals[4] = {1000, 1000, 1000, 1000};

    if (nx >= 0 && ny >= 0 && isAccessible(cx, cy, nx, ny)) vals[FORWARD]  = floodArray[nx][ny];
    if (ex >= 0 && ey >= 0 && isAccessible(cx, cy, ex, ey)) vals[RIGHT]    = floodArray[ex][ey];
    if (sx >= 0 && sy >= 0 && isAccessible(cx, cy, sx, sy)) vals[BACKWARD] = floodArray[sx][sy];
    if (wx >= 0 && wy >= 0 && isAccessible(cx, cy, wx, wy)) vals[LEFT]     = floodArray[wx][wy];

    int minVal = 1000;
    for (int i = 0; i < 4; i++)
        if (vals[i] < minVal) minVal = vals[i];

    if (minVal != 1000)
        floodArray[cx][cy] = minVal + 1;
}


// visited: within-call loop guard prevents a cell being re-enqueued during
// one invocation. Reset at the top of every call. (Improvement: bugfix from session.)
bool visited[16][16] = {false};


/**================================================================
 * @Fn      - floodFillUsingQueue
 * @brief   - Propagates only inconsistent cells outward from (start_x, start_y).
 *            x*16+y queue encoding - halves push/pop operations.
 */
void floodFillUsingQueue(int start_x, int start_y, int previous_x, int previous_y) {
    int nx, ny, ex, ey, sx, sy, wx, wy;

    // Reset visited every call - it is a within-call guard, not persistent state
    memset(visited, false, sizeof(visited));

    std::queue<int> cellQueue;

    if (!isIncrementConsistent(start_x, start_y))
        floodArray[start_x][start_y]++;

    // Encode cell as single int (x*16+y)
    cellQueue.push(start_x * 16 + start_y);

    while (!cellQueue.empty()) {
        int cell     = cellQueue.front(); cellQueue.pop();
        int current_X = cell / 16;
        int current_Y = cell % 16;

        if (visited[current_X][current_Y]) continue;
        visited[current_X][current_Y] = true;

        if (!isIncrementConsistent(current_X, current_Y)) {
            makeCellConsistent(current_X, current_Y);

            getSurroundings(current_X, current_Y, &nx, &ny, &ex, &ey, &sx, &sy, &wx, &wy);

            int nX[4] = {nx, ex, sx, wx};
            int nY[4] = {ny, ey, sy, wy};

            for (int i = 0; i < 4; i++) {
                if (nX[i] >= 0 && nY[i] >= 0 &&
                    isAccessible(current_X, current_Y, nX[i], nY[i])) {
                    cellQueue.push(nX[i] * 16 + nY[i]);
                }
            }
        }
    }
}


/**================================================================
 * @Fn      - whereToMove
 * @brief   - Returns F, L, R, or B.
 *            When multiple neighbours share the minimum flood
 *            value, prefer the one matching current orientation (go straight)
 *            to avoid unnecessary turns.
 */
char whereToMove(int current_x, int current_y, int previous_x, int previous_y, int orient) {
    int nx, ny, ex, ey, sx, sy, wx, wy;
    getSurroundings(current_x, current_y, &nx, &ny, &ex, &ey, &sx, &sy, &wx, &wy);

    int accessibleNeighbors[4] = {1000, 1000, 1000, 1000};
    int AccessiblePathsNum = 0;
    int previous = -1;

    // Identify which global direction is our rear cell
    if      (sx == previous_x && sy == previous_y) previous = BACKWARD;
    else if (nx == previous_x && ny == previous_y) previous = FORWARD;
    else if (ex == previous_x && ey == previous_y) previous = RIGHT;
    else if (wx == previous_x && wy == previous_y) previous = LEFT;

    int nbrX[4] = {nx, ex, sx, wx};
    int nbrY[4] = {ny, ey, sy, wy};

    for (int i = 0; i < 4; i++) {
        if (nbrX[i] >= 0 && nbrY[i] >= 0 &&
            isAccessible(current_x, current_y, nbrX[i], nbrY[i])) {
            AccessiblePathsNum++;
            accessibleNeighbors[i] = floodArray[nbrX[i]][nbrY[i]];
        }
    }

    int minValue = 1000;
    int minCell  = 1000;

    for (int i = 0; i < 4; i++) {
        if (accessibleNeighbors[i] == 1000) continue;
        if (AccessiblePathsNum > 1 && i == previous) continue;  // don't backtrack unless forced
        if (accessibleNeighbors[i] < minValue) {
            minValue = accessibleNeighbors[i];
            minCell  = i;
        }
    }

    // Prefer straight ahead over turning
    if (accessibleNeighbors[orient] == minValue && AccessiblePathsNum > 1) {
        minCell = orient;
    }

    if (minCell == orient)                                        return 'F';
    if (minCell == (orient + 3) % 4 || minCell == (orient - 1 + 4) % 4) return 'L';
    if (minCell == (orient + 1) % 4 || minCell == (orient - 3 + 4) % 4) return 'R';
    return 'B';
}


/**================================================================
 * @Fn      - initBoundaryWalls
 * @brief   - Pre-load the outer maze walls into the walls array
 *            at startup. The robot knows these exist without needing to sense them.
 */
void initBoundaryWalls() {
    for (int i = 0; i < 16; i++) {
        setWall(i,  0, WALL_S);   // bottom row — south edge
        setWall(i, 15, WALL_N);   // top row    — north edge
        setWall( 0, i, WALL_W);   // left col   — west edge
        setWall(15, i, WALL_E);   // right col  — east edge
    }
}


// Global navigation state
int current_x = 0, current_y = 0;
int previous_x = 0, previous_y = 0;
int orient = FORWARD;
char Direction = 0;
bool leftW = false, rightW = false, frontW = false;


void setup() {
    Serial.begin(9600);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);

    pinMode(encoderLeft_A,  INPUT);
    pinMode(encoderLeft_B,  INPUT);
    pinMode(encoderRight_A, INPUT);
    pinMode(encoderRight_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(encoderLeft_A),  EncoderLeftHandler,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderLeft_B),  EncoderLeftHandler,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderRight_A), EncoderRightHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderRight_B), EncoderRightHandler, CHANGE);

    // MPU6050 - init once, not per turn (bugfix)
    Wire.begin();
    mpu.begin();
    mpu.calcOffsets();
    Serial.println("MPU6050 ready.");

    // Improvement #8: outer walls are always present - load them now
    initBoundaryWalls();
    Serial.println("Boundary walls loaded.");
}


void loop() {
    leftW  = wallLeft();
    rightW = wallRight();
    frontW = wallFront();

    if (leftW)  Serial.println("Wall Left");
    if (rightW) Serial.println("Wall Right");
    if (frontW) Serial.println("Wall Front");

    updateWalls(current_x, current_y, orient, leftW, rightW, frontW);

    if (floodArray[current_x][current_y] != 0) {
        floodFillUsingQueue(current_x, current_y, previous_x, previous_y);
    } else {
        Serial.println("Reached centre!");
        while (true);
    }

    Direction = whereToMove(current_x, current_y, previous_x, previous_y, orient);
    Serial.print("Direction: ");
    Serial.println(Direction);

    if (Direction == 'L') {
        turnLeft();
        orient = orientation(orient, 'L');
    } else if (Direction == 'R') {
        turnRight();
        orient = orientation(orient, 'R');
    } else if (Direction == 'B') {
        turnLeft();  orient = orientation(orient, 'L');
        turnLeft();  orient = orientation(orient, 'L');
    }

    Serial.println("moveForward");
    moveForward();

    previous_x = current_x;
    previous_y = current_y;
    updateCoordinates(orient, &current_x, &current_y);
}
