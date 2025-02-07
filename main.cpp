#include <boids.h>

void userInputThread(std::atomic<bool> &running, std::array<std::atomic<float>, NUM_PARAMS> &params) {
    std::string input;
    while(running) {
        std::getline(std::cin, input);
        if(input == "quit") running = false;
    }
}

// setEdgeMode: -1 to keep current edge mode, 0 to set edge mode to default
void setDefaultParams(paramList &arr, int setEdgeMode) {
    for(int i = 0; i < NUM_PARAMS + setEdgeMode; i++){
        arr[i].store(defaultParams[i]);
    }
}

int main(int argc, char* argv[]) {
    srand(time(0));

    paramList params;
    setDefaultParams(params, 0);

    boidmap map;
    boidarr boids = initBoids();

    SDL_Window* window = initSDL();
    SDL_Renderer* renderer = initRenderer(window);

    std::atomic<bool> running = true;
    SDL_Event event;
    uint32_t tick;
    uint32_t lastTick = SDL_GetTicks();
    uint32_t timeSinceLastUpdate = 0;
    SDL_FRect brush;

    std::thread inputThread(userInputThread, std::ref(running), std::ref(params));

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
            updateBoids(map, boids, params);
            timeSinceLastUpdate = 0;
        }

        drawToScreen(renderer, boids, brush);

        SDL_Delay(1);
    }

    inputThread.join();
    quit(window, renderer);
    return 0;
}