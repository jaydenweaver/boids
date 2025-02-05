#include <SDL3/SDL.h>
#include <SDL3/SDL_events.h>
#include <iostream>
#include <array>
#include <vector>
#include <cmath>

#define TICK_RATE 60
#define SCREEN_WIDTH 1600
#define SCREEN_HEIGHT 900
#define BOID_COUNT 100
#define MAX_VELOCITY 5
#define MIN_VELOCITY 2
#define BOID_SIZE 4

// in cells, (n + n + 1)^2 cell grid centered on boids current cell - size of 2 is a 5x5 cell grid
#define LOCAL_SIZE 3

// resolution of boid vectors
#define X_GRID_COUNT 20
#define Y_GRID_COUNT 10

// rule parameters
#define AVOID_STRENGTH 0.1f
#define MIN_DISTANCE 10
#define CENTRE_STRENGTH 0.1f
#define ALIGN_STRENGTH 0.1f
#define EDGE_MARGIN 100
#define TURN_STRENGTH 1
#define MAX_TURN_RATE 0.1f
#define ACCEL_STRENGTH 2.0f

constexpr int cellWidth = SCREEN_WIDTH / X_GRID_COUNT;
constexpr int cellHeight = SCREEN_HEIGHT / Y_GRID_COUNT;

constexpr float tickWait = 1 / TICK_RATE;

class Boid {
    public:
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
typedef std::array<std::array<std::vector<Boid*>, Y_GRID_COUNT + 1>, X_GRID_COUNT + 1> boidmap;

// game functions
SDL_Window* initSDL();
SDL_Renderer* initRenderer(SDL_Window* window);
void quit(SDL_Window* window, SDL_Renderer* renderer);
void drawToScreen(SDL_Renderer* renderer, boidarr& boids, SDL_FRect &brush);

// boid functions
boidarr initBoids();
void reassignBoid(boidmap &map, Boid &boid);
void updateBoids(boidmap &map, boidarr &arr);
void applyRules(boidmap &map, Boid &boid);
