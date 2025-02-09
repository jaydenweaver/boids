#include <boids.h>

void userInputThread(std::atomic<bool> &running, paramList &params) {
    std::string input;
    while(running) {
        std::cout << "enter command: " << std::flush;
        std::getline(std::cin, input);
        if(input == "quit") {
            running = false;
            continue;
        }
        doInput(params, input);
    }
}

int main(int argc, char* argv[]) {
    // new seed for initial boid locations / velocities
    srand(time(0));

    std::printf("boids simulation - refer to the README.md for usage guide\n");

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