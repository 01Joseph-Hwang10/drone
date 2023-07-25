#include "drone_interpreter.h"
#include "global.h"

#if defined(RC_FROM_INTERPRETER)

RabbitInterpreter interpreter(ROL, PIT, RUD, THR);

uint16_t sensor_getter1(){
    return 1;
}

void setup_interpreter()
{
    // uint8_t PROGRAM[88] = {
    //     0B00000000, 0B00000000, 0B00000000, 0B00010101,
    //     0B01101111, 0B10000000, 0B00000000, 0B00000101,
    //     0B00100000, 0B00000000, 0B00000000, 0B01111100,
    //     0B01101111, 0B10000000, 0B00000000, 0B00000001,
    //     0B00000000, 0B00011100, 0B01111110, 0B00000100,
    //     0B01101111, 0B10000000, 0B00000011, 0B11101000,
    //     0B01010000, 0B00000000, 0B00000000, 0B01111100,
    //     0B01101111, 0B10000000, 0B00000101, 0B01111000,
    //     0B01101111, 0B10100000, 0B00000110, 0B01000000,
    //     0B01101111, 0B11000000, 0B00000111, 0B11010000,
    //     0B01101111, 0B11100000, 0B00000111, 0B00001000,
    //     0B01001111, 0B10011111, 0B01111111, 0B01111111,
    //     0B00010000, 0B00000000, 0B00000000, 0B00000000,
    //     0B01101111, 0B10000000, 0B00000000, 0B00001100,
    //     0B00110000, 0B00000000, 0B00000000, 0B01111100,
    //     0B01100000, 0B00000000, 0B00000000, 0B00000000, 
    //     0B01101111, 0B10000000, 0B00000000, 0B00001010, 
    //     0B01101111, 0B10100000, 0B00000000, 0B00000011, 
    //     0B10000000, 0B01100000, 0B00111110, 0B01111101, 
    //     0B01101111, 0B11000000, 0B00000000, 0B00000001, 
    //     0B01110000, 0B00000000, 0B00000000, 0B01111110, 
    //     0B10010000, 0B00000000, 0B00000000, 0B01111101
    // };
    
    uint16_t rcData[4] = {0, 0, 0, 0};

    interpreter.setSensorGetter(1, sensor_getter1);
}

uint8_t * parse_instructions(const std::string source, uint8_t * dist) {
    for (int i = 0; i < source.length(); i++) {
        dist[i] = (uint8_t)source[i];
    }
    return dist;
}

bool load_instructions()
{
    std::string recieved = ble.getValue();
    if (recieved.length() > 0) {
        Serial.println(recieved.c_str());
        uint8_t instructions[recieved.length()];
        parse_instructions(recieved, instructions);
        interpreter.loadInstructions(instructions);
        return true;
    }

    return false;
}

bool buf_to_rc_from_interpreter()
{
    return interpreter.interpret(rcValue);
}

#endif