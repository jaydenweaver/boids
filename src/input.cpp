#include <boids.h>
#include <sstream>

// setEdgeMode: -1 to keep current edge mode, 0 to set edge mode to default
void setDefaultParams(paramList &arr, int setEdgeMode) {
    for(int i = 0; i < NUM_PARAMS + setEdgeMode; i++) {
        arr[i].store(defaultParams[i]);
    }
}

bool isSet(std::string &input) {
    return input.substr(0, 4) == "set ";
}

void displayValue(std::string &input, paramList &params) {
    auto isValid = std::find_if(paramMap.begin(), paramMap.end(), 
    [&](const auto& pair) { return input == pair.first; });

    if(isValid == paramMap.end()){
        std::printf("invalid command!\n");
    } else std::printf("%g\n", params[isValid -> second].load());
}

std::string getValString(std::string &input, int &index) {
    for(size_t i = 0; i < input.length() - 1; i++) {
        if(input[i] == ' ') {
            index = i + 1;
            break;
        }
    }
    return input.substr(index, input.length() - index);
}

void setParam(std::string &input, std::string &valString, paramList &params) {
    std::stringstream ss(valString);
    float val;

    if (!(ss >> val)) {
        std::printf("value input error!\n");
        return;
    }

    auto isValid = std::find_if(paramMap.begin(), paramMap.end(), 
                    [&](const auto& pair) { return input == pair.first; });

    if(isValid == paramMap.end()){
        std::printf("invalid command!: %s\n", input.c_str());
        return;
    }
    
    params[isValid -> second] = val;
    std::printf("%s set to %g\n", input.c_str(), val);
}

void doInput(paramList &params, std::string &input) {
    if(!isSet(input)) {
        displayValue(input, params);
        return;
    }

    input = input.substr(4, input.length() - 4);

    if(input == "default") {
        setDefaultParams(params, -1);
        return;
    }

    int valIndex = 0;
    std::string valString = getValString(input, valIndex);
    
    if(valIndex == 0) {
        std::printf("could not find value!\n");
        return;
    }

    input = input.substr(0, valIndex-1);
    setParam(input, valString, params);
}