#include <boids.h>

SDL_Window* initSDL() {
    if(SDL_Init(SDL_INIT_VIDEO) < 0) {  
        std::printf("failed to initialise SDL: %s\n", SDL_GetError());
        exit(1);
    }

    SDL_Window* window = SDL_CreateWindow("Boids", SCREEN_WIDTH, SCREEN_HEIGHT, 0);
    SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);

    if(window == nullptr) {
        std::printf("failed to initialise window: %s\n", SDL_GetError());
        exit(1);
    }

    return window;
}

SDL_Renderer* initRenderer(SDL_Window* window) {
    SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);
    if(renderer == nullptr) {
        std::printf("failed to initialise renderer: %s\n", SDL_GetError());
        exit(1);
    }
    return renderer;
}

void quit(SDL_Window* window, SDL_Renderer* renderer) {
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void drawToScreen(SDL_Renderer* renderer, boidarr& boids, SDL_FRect& brush) {
    brush.w = BOID_SIZE;
    brush.h = BOID_SIZE;

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

    for(int i = 0; i < BOID_COUNT; i++) {
        brush.x = boids[i].x;
        brush.y = boids[i].y;
        SDL_RenderRect(renderer, &brush);
    }
    
    SDL_RenderPresent(renderer);
}