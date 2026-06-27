#ifndef FLOODFILL_H
#define FLOODFILL_H

#include <Arduino.h>


#define MAZE_SIZE     16
#define CELL_SIZE_MM  200
#define TICKS_PER_MM  7.38
#define GOAL_MIN       7
#define GOAL_MAX       8

#define WALL_THRESHOLD  165
#define FRONT_WALL_THRESHOLD  100

#define WALL_NORTH    0x01
#define WALL_EAST     0x02
#define WALL_SOUTH    0x04
#define WALL_WEST     0x08
#define WALL_VISITED  0x10

enum Heading { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3, NO_DIR = 4};

struct Cell {
  uint8_t x;
  uint8_t y;
};

extern int  walls[MAZE_SIZE+1][MAZE_SIZE+1];
extern int  flood[MAZE_SIZE+1][MAZE_SIZE+1];
extern Heading  robot_heading;
extern Cell     robot_pos;

void initMaze();
void floodFill();
bool frontWall();
bool leftWall();
bool rightWall();
void solve();
void floodfill();
void moveOneCell();
Heading getBestNeighbour();


#endif