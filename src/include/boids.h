#include <SDL3/SDL.h>
#include <SDL3/SDL_events.h>
#include <iostream>
#include <array>
#include <vector>
#include <cmath>
#include <thread>
#include <atomic>
#include <algorithm>

#define TICK_RATE 60
#define SCREEN_WIDTH 1800
#define SCREEN_HEIGHT 1000
#define BOID_COUNT 300
#define MAX_VELOCITY 5
#define MIN_VELOCITY 2
#define BOID_SIZE 4

// in cells, (n + n + 1)^2 cell grid centered on boids current cell - size of 2 is a 5x5 cell grid
#define LOCAL_SIZE 1

// resolution of boid vectors
#define X_GRID_COUNT 20
#define Y_GRID_COUNT 10

// default rule parameters
#define AVOID_STRENGTH 0.1f
#define MIN_DISTANCE 10
#define CENTRE_STRENGTH 0.2f
#define ALIGN_STRENGTH 0.01f
#define EDGE_MARGIN 100
#define TURN_STRENGTH 1
#define MAX_TURN_RATE 0.1f
#define ACCEL_STRENGTH 2.0f
#define NUM_PARAMS 5

//  parameter indexes
#define AVOID 0
#define DISTANCE 1
#define ALIGN 2
#define CENTRE 3
#define EDGE 4


constexpr int cellWidth = SCREEN_WIDTH / X_GRID_COUNT;
constexpr int cellHeight = SCREEN_HEIGHT / Y_GRID_COUNT;

constexpr std::array<float, NUM_PARAMS> defaultParams = {AVOID_STRENGTH,
                                                        MIN_DISTANCE,
                                                        ALIGN_STRENGTH,
                                                        CENTRE_STRENGTH,
                                                        0};

constexpr std::array<std::pair<const char*, int>, 5> paramMap = {{{"avoid", AVOID},
                                                                {"distance", DISTANCE},
                                                                {"align", ALIGN},
                                                                {"centre", CENTRE},
                                                                {"edge", EDGE}}};

struct Boid {
    // x, y = pos | vx, vy = velocity | ax, ay = acceleration | gx, gy = grid pos
    float x, y;
    float vx, vy;
    float ax, ay;
    uint8_t gx, gy, id;

    Boid();
    Boid(int index);
};

typedef std::array<Boid, BOID_COUNT> boidarr;
    
// MAP -> 2D cell array [x][y] holding vectors of every boid in that cell
typedef std::array<std::array<std::vector<Boid*>, Y_GRID_COUNT>, X_GRID_COUNT> boidmap;

typedef std::array<std::atomic<float>, NUM_PARAMS> paramList;

// game functions
SDL_Window* initSDL();
SDL_Renderer* initRenderer(SDL_Window*);
void quit(SDL_Window*, SDL_Renderer*);
void drawToScreen(SDL_Renderer*, boidarr&, SDL_FRect&);

// boid functions
boidarr initBoids();
void reassignBoid(boidmap&, Boid&);
void updateBoids(boidmap&, boidarr&, paramList&);
void applyRules(boidmap&, Boid&, paramList&);
void getEdgeMode(std::string);

// input functions
void doInput(paramList&, std::string&);
void setDefaultParams(paramList&, int);