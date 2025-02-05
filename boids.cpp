#include <boids.h>

/*

    boid array of size BOID_COUNT

    boid needs 4 total values: x, y, vx, vy

    randomise every boids position and velocity

    apply boid rules:

        -   separation: steer to avoid crowding local entities
        -   alignment:  steer toward the average heading of local entities
        -   cohesion:   steer to move toward the average position of local entities

        local:  within a certain vicinity of entity (defined as LOCAL_SIZE in boids.h)

*/
Boid::Boid(){}

Boid::Boid(int index) {
    id = index;
    
    // initialise with random velocity + position (excluding the edges of the screen)
    x = (rand() % (SCREEN_WIDTH - (SCREEN_WIDTH / 10))) + SCREEN_WIDTH / 20;
    y = (rand() % (SCREEN_HEIGHT - (SCREEN_HEIGHT / 10))) + SCREEN_HEIGHT / 20;
    vx = (rand() % (MAX_VELOCITY - MIN_VELOCITY)) + MIN_VELOCITY;
    vy = (rand() % (MAX_VELOCITY - MIN_VELOCITY)) + MIN_VELOCITY;

    // randomly assigns velocities to negatives so they dont just go forwards
    vx = rand() % 2 == 1 ? vx : -vx;
    vy = rand() % 2 == 1 ? vy : -vy;

    ax = 0.0;
    ay = 0.0;
    
    // place boid in cell
    gx = x / cellWidth;
    gy = y / cellHeight;
}

boidarr initBoids() {
    boidarr arr;
    for(int i = 0; i < BOID_COUNT; i++) {
        arr[i] = Boid(i+1);
    }
    return arr;
}

int distance(Boid &boid1, Boid* boid2) {
    return sqrt(((boid2->x - boid1.x) * (boid2->x - boid1.x)) + ((boid2->y - boid1.y) * (boid2->y - boid1.y)));
}

void reassignBoid(boidmap &map, Boid &boid) {
    std::vector<Boid*> &vec =  map[boid.gx][boid.gy];
    for(int i = 0; i < vec.size(); i++) {
        if(vec.at(i) -> id == boid.id) vec.erase(vec.begin() + i);
    }
    boid.gx = boid.x / cellWidth;
    boid.gy = boid.y / cellHeight;
    map[boid.gx][boid.gy].push_back(&boid);
}

void updateBoids(boidmap &map, boidarr &arr) {
    /*  move boids and bounce off edges
        application crashes if boid goes off screen due to reassignBoid call - cell doesnt exist (array index out of bounds)    */
    for(int i = 0; i < BOID_COUNT; i++){
        applyRules(map, arr[i]);
        if(0 >= arr[i].x + arr[i].vx || arr[i].x + arr[i].vx >= SCREEN_WIDTH) arr[i].vx = -arr[i].vx;
        if(0 >= arr[i].y + arr[i].vy || arr[i].y + arr[i].vy >= SCREEN_HEIGHT) arr[i].vy = -arr[i].vy;
        arr[i].x += arr[i].vx;
        arr[i].y += arr[i].vy;
        if(arr[i].x / cellWidth != arr[i].gx || arr[i].y / cellHeight != arr[i].gy) reassignBoid(map, arr[i]);
    }
}

void steerTo(Boid &boid, float targetVX, float targetVY) {
    float targetAngle = atan2(targetVY, targetVX);
    float currentAngle = atan2(boid.vy, boid.vx);
    float diff = targetAngle - currentAngle;

    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;

    if (diff < -MAX_TURN_RATE) diff = -MAX_TURN_RATE;
    if (diff > MAX_TURN_RATE) diff = MAX_TURN_RATE;

    float newAngle = currentAngle + diff;
    float velocity = sqrt(boid.vx * boid.vx + boid.vy * boid.vy);
    boid.ax = (cos(newAngle) * velocity);
    boid.ay = (sin(newAngle) * velocity);
}

void separation(Boid &boid, std::vector<Boid*> locals) {
    float strength = AVOID_STRENGTH;
    for(int i = 0; i < locals.size(); i++) {
        if (distance(boid, locals.at(i)) < MIN_DISTANCE) {
            boid.ax += ((boid.x - locals.at(i) -> x) * strength);
            boid.ay += ((boid.y - locals.at(i) -> y) * strength);
        }
    }
}

void alignment(Boid &boid, std::vector<Boid*> locals) {
    if(locals.empty()) return;

    float strength = ALIGN_STRENGTH;

    int16_t avgVX = 0;
    int16_t avgVY = 0;

    for(Boid* local : locals) {
        avgVX += local -> vx;
        avgVY += local -> vy; 
    }

    avgVX = avgVX / locals.size();
    avgVY = avgVY / locals.size();

    boid.ax += ((avgVX - boid.vx) * strength);
    boid.ay += ((avgVY - boid.vy) * strength);
}

// can lower complexity via calculating the centre per cell instead of per boid
void cohesion(Boid &boid, std::vector<Boid*> locals) {
    if(locals.empty()) return;
    
    float strength = CENTRE_STRENGTH;

    uint16_t centreX = 0;
    uint16_t centreY = 0;

    for(Boid* local : locals) {
        centreX += local -> x;
        centreY += local -> y;
    }

    centreX /= locals.size();
    centreY /= locals.size();

    boid.ax += ((centreX / boid.x) * strength);
    boid.ay += ((centreY / boid.y) * strength);
}

std::vector<Boid*> getLocals(boidmap &map, Boid &boid) {
    std::vector<Boid*> locals;
    int xCursor = -LOCAL_SIZE;
    int yCursor = -LOCAL_SIZE;
    int x, y;

    while(xCursor <= LOCAL_SIZE && xCursor + boid.gx <= X_GRID_COUNT) {
        if(boid.gx + xCursor < 0) {
            xCursor++;
            continue;
        }
        while(yCursor <= LOCAL_SIZE && yCursor + boid.gy <= Y_GRID_COUNT) {
            if(boid.gy + yCursor < 0) {
                yCursor++;
                continue;
            }
            x = boid.gx + xCursor;
            y = boid.gy + yCursor;
            locals.insert(locals.end(), map[x][y].begin(), map[x][y].end());

            yCursor++;
        }
        yCursor = -LOCAL_SIZE;
        xCursor++;
    }

    return locals;
}

void limitVelocity(Boid &boid) {
    float velocity = sqrt(boid.vx * boid.vx + boid.vy * boid.vy);
    if(velocity > MAX_VELOCITY) {
        boid.vx = (boid.vx / velocity) * MAX_VELOCITY;
        boid.vy = (boid.vy / velocity) * MAX_VELOCITY;
    } else if(velocity < MIN_VELOCITY) {
        if(rand() % 2 == 1) {
            boid.vx = rand() % 2 == 1 ? boid.vx + 1 : boid.vx - 1;
        } else {
            boid.vy = rand() % 2 == 1 ? boid.vy + 1 : boid.vy - 1;
        }
    }
}

void keepFromEdge(Boid &boid) {
    uint16_t margin = EDGE_MARGIN;
    uint8_t strength = TURN_STRENGTH;

    if (boid.x < margin) boid.ax += strength;
    if (boid.x > SCREEN_WIDTH - margin) boid.ax -= strength;
    if (boid.y < margin) boid.ay += strength;
    if (boid.y > SCREEN_HEIGHT - margin) boid.ay -= strength;
}

void applyRules(boidmap &map, Boid &boid) {
    std::vector<Boid*> locals = getLocals(map, boid);
    
    keepFromEdge(boid);

    cohesion(boid, locals);
    separation(boid, locals);
    alignment(boid, locals);

    steerTo(boid, boid.vx + boid.ax, boid.vy + boid.ay);

    boid.vx += boid.ax * ACCEL_STRENGTH;
    boid.vy += boid.ay * ACCEL_STRENGTH;

    boid.ax = 0.0;
    boid.ay = 0.0;

    limitVelocity(boid);
}