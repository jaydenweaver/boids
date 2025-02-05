#include <boids.h>

int main(int argc, char* argv[]) {
    srand(time(0));
    boidmap map;
    boidarr boids = initBoids();

    SDL_Window* window = initSDL();
    SDL_Renderer* renderer = initRenderer(window);

    bool running = true;
    SDL_Event event;
    uint32_t tick;
    uint32_t lastTick = SDL_GetTicks();
    uint32_t timeSinceLastUpdate = 0;
    SDL_FRect brush;

    while(running) {
        tick = SDL_GetTicks();
        timeSinceLastUpdate += tick - lastTick;
        lastTick = tick;

        while(SDL_PollEvent(&event)) {
            if(event.type == SDL_EVENT_QUIT){
                running = false;
                break;
            }
        }

        if(timeSinceLastUpdate >= 1000 / TICK_RATE) {
            updateBoids(map, boids);
            timeSinceLastUpdate = 0;
        }

        drawToScreen(renderer, boids, brush);

        SDL_Delay(1);
    }

    quit(window, renderer);
    return 0;
}