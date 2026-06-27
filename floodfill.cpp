#include "floodfill.h"
#include "sensors.h"
#include "motor_control.h"
#include "forward_pid.h"
#include "centering_pid.h"
#include "turning_pid.h"
#include <queue>


int walls[MAZE_SIZE+1][MAZE_SIZE+1];
int flood[MAZE_SIZE+1][MAZE_SIZE+1];

Heading robot_heading = NORTH;
Cell robot_pos = {1, 1};


void initMaze(){
    for(int i = 1; i<=MAZE_SIZE; i++){
      for(int j = 1; j<=MAZE_SIZE; j++){
        int dist1 = abs(i - GOAL_MIN) + abs(j - GOAL_MIN);
        int dist2 = abs(i - GOAL_MIN) + abs(j - GOAL_MAX);
        int dist3 = abs(i - GOAL_MAX) + abs(j - GOAL_MIN);
        int dist4 = abs(i - GOAL_MAX) + abs(j - GOAL_MAX);
        flood[i][j] = min(min(dist1, dist2), min(dist3, dist4));
      }
    }

    for (int i = 1; i<=MAZE_SIZE; i++) {
      walls[i][1]           |= WALL_SOUTH;
      walls[i][MAZE_SIZE] |= WALL_NORTH;
      walls[1][i]           |= WALL_WEST;
      walls[MAZE_SIZE][i] |= WALL_EAST;
    }

    for (int x = GOAL_MIN; x <= GOAL_MAX; x++) {
      for (int y = GOAL_MIN; y <= GOAL_MAX; y++) {
        flood[x][y] = 0;
      }
    }
}

bool isGoal() {
    return (robot_pos.x >= GOAL_MIN && robot_pos.x <= GOAL_MAX &&
            robot_pos.y >= GOAL_MIN && robot_pos.y <= GOAL_MAX);
}

void moveOneCell() {
    resetEncoders();
    long target_ticks = CELL_SIZE_MM * TICKS_PER_MM;
    current_target_speed = 80.0;

    

    while (true)
    {
        noInterrupts();
        long a = encoderA_count;
        long b = encoderB_count;
        interrupts();

        if (readFront() < FRONT_WALL_THRESHOLD) 
        break;

        if ((a + b) / 2 >= target_ticks)
            break;

        calculateSpeeds();
        updatePID(current_target_speed, speedA, speedB);

        int l = readLeft();
        int r = readRight();
        float centering = 0;
        if (l < 175 && r < 175)
        {
            centering = computeCenteringCorrection(l, r);
        }

        setMotors(
            constrain(pwmA_out + centering, 40, 200),
            constrain(pwmB_out - centering, 40, 200));
        delay(5);
    }
    resetPID();
    delay(30);
}

bool frontWall() { return readFront() < FRONT_WALL_THRESHOLD; }
bool leftWall()  { return readLeft()  < WALL_THRESHOLD; }
bool rightWall() { return readRight() < WALL_THRESHOLD; }

void propagateWalls(int x, int y) {
    if ((walls[x][y] & WALL_NORTH) && y+1 <= MAZE_SIZE)
        walls[x][y+1] |= WALL_SOUTH;
    if ((walls[x][y] & WALL_SOUTH) && y-1 >= 1)
        walls[x][y-1] |= WALL_NORTH;
    if ((walls[x][y] & WALL_EAST)  && x+1 <= MAZE_SIZE)
        walls[x+1][y] |= WALL_WEST;
    if ((walls[x][y] & WALL_WEST)  && x-1 >= 1)
        walls[x-1][y] |= WALL_EAST;
}

bool senseAndUpdateWalls() {
    int x = robot_pos.x, y = robot_pos.y;
    int before = walls[x][y];

    switch (robot_heading) {
    case NORTH:
        if (frontWall()) walls[x][y] |= WALL_NORTH;
        if (leftWall())  walls[x][y] |= WALL_WEST;
        if (rightWall()) walls[x][y] |= WALL_EAST;
        break;
    case EAST:
        if (frontWall()) walls[x][y] |= WALL_EAST;
        if (leftWall())  walls[x][y] |= WALL_NORTH;
        if (rightWall()) walls[x][y] |= WALL_SOUTH;
        break;
    case SOUTH:
        if (frontWall()) walls[x][y] |= WALL_SOUTH;
        if (leftWall())  walls[x][y] |= WALL_EAST;
        if (rightWall()) walls[x][y] |= WALL_WEST;
        break;
    case WEST:
        if (frontWall()) walls[x][y] |= WALL_WEST;
        if (leftWall())  walls[x][y] |= WALL_SOUTH;
        if (rightWall()) walls[x][y] |= WALL_NORTH;
        break;
    }
    propagateWalls(x,y);
    return walls[x][y] != before; // true if new walls discovered
}
Heading getBestNeighbour() {
    int x = robot_pos.x;
    int y = robot_pos.y;
    
    Heading best = NO_DIR;
    int bestVal = flood[x][y];  

    if (!(walls[x][y] & WALL_NORTH) && (y + 1 <= MAZE_SIZE)){
      if (flood[x][y+1] < bestVal){
        bestVal = flood[x][y+1];
        best = NORTH;
      }
    }

    if (!(walls[x][y] & WALL_SOUTH) && (y-1>=1)){
      if (flood[x][y-1] < bestVal){
        bestVal = flood[x][y-1];
        best = SOUTH;
      }
    }

    if (!(walls[x][y] & WALL_EAST) && (x + 1 <= MAZE_SIZE)){
      if (flood[x+1][y] < bestVal){
        bestVal = flood[x+1][y];
        best = EAST;
      }
    }

    if (!(walls[x][y] & WALL_WEST) && (x-1>=1)){
      if (flood[x-1][y] < bestVal){
        bestVal = flood[x-1][y];
        best = WEST;
      }
    }
    return best;
}

void moveTo(Heading to_move_at){
  int turns = (to_move_at - robot_heading + 4) % 4;
  if(turns !=0){
    stopMotors();
    delay(50);

    switch(turns) {
        case 0:
            break;
        case 1: {
            float current_yaw = getYaw();
            turnRight90(0,current_yaw);
            robot_heading = (Heading)((robot_heading + 1) % 4);
            break;
        }
        case 2: {
            float current_yaw = getYaw();
            turn180(0,current_yaw);
            robot_heading = (Heading)((robot_heading + 2) % 4);
            break;
        }
        case 3:{
            float current_yaw = getYaw();
            turnLeft90(0,current_yaw);
            robot_heading = (Heading)((robot_heading + 3) % 4);
            break;
        }

    }
  }
  
  
  moveOneCell();
  
  switch(to_move_at) {
        case NORTH: robot_pos.y++; break;
        case SOUTH: robot_pos.y--; break;
        case EAST:  robot_pos.x++; break;
        case WEST:  robot_pos.x--; break;
    }
  

}


void floodFill() {
    // Reset all cells to "infinity"
    for (int i = 1; i <= MAZE_SIZE; i++)
        for (int j = 1; j <= MAZE_SIZE; j++)
            flood[i][j] = 255;

    std::queue<Cell> q;

    // Seed from ALL goal cells
    for (int x = GOAL_MIN; x <= GOAL_MAX; x++) {
        for (int y = GOAL_MIN; y <= GOAL_MAX; y++) {
            flood[x][y] = 0;
            q.push({(uint8_t)x, (uint8_t)y});
        }
    }

    while (!q.empty()) {
        Cell c = q.front(); q.pop();
        int x = c.x, y = c.y;

        auto tryUpdate = [&](int nx, int ny) {
            if (nx >= 1 && nx <= MAZE_SIZE && ny >= 1 && ny <= MAZE_SIZE) {
                if (flood[nx][ny] > flood[x][y] + 1) {
                    flood[nx][ny] = flood[x][y] + 1;
                    q.push({(uint8_t)nx, (uint8_t)ny});
                }
            }
        };

        if (!(walls[x][y] & WALL_NORTH)) tryUpdate(x, y + 1);
        if (!(walls[x][y] & WALL_SOUTH)) tryUpdate(x, y - 1);
        if (!(walls[x][y] & WALL_EAST))  tryUpdate(x + 1, y);
        if (!(walls[x][y] & WALL_WEST))  tryUpdate(x - 1, y);
    }
}

void solve() {
    // Run initial flood fill from goal
    floodFill();

    while (!isGoal()) {
        bool wallsChanged = senseAndUpdateWalls(); // see note below
        
        if (wallsChanged) {
            floodFill(); // recompute distances with new wall info
        }

        Heading to_move_at = getBestNeighbour();
        if (to_move_at == NO_DIR) {
            floodFill();
            to_move_at = getBestNeighbour();
        }
        if (to_move_at != NO_DIR) {
            moveTo(to_move_at);
        } else {
            // Truly stuck — add error handling / stop
            stopMotors();
            while(1);
        }
    }
}




