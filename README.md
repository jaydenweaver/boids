# Boids simulation written in C++ using SDL3.

![](https://github.com/jaydenweaver/boids/blob/main/boids.gif)

To run, downloaded latest release and run main.exe.

## Commands:

set (parameter) (value) -> *sets the specified parameter to the specified value e.g. 'set align 0.5'*

set default -> *sets all parameters to their default values*

(parameter) -> *displays current value for specified parameter e.g. 'distance'*

## Parameters:

distance -> *minimum distance boids will try to maintain between each other*

avoid -> *how much boids will try to move away from other boids within its minimum distance*

centre -> *how much boids will try to move towards the centre of the boids within its range*

align -> *how much boids will try to match the average direction of the boids within its range*

edge -> 0 for boids to bounce off edges, otherwise loop around the screen
