#ifndef DRONE_INTERPRETER_H
#define DRONE_INTERPRETER_H

#if defined(RC_FROM_INTERPRETER)
#include "rabbit_interpreter.h"
#include "global.h"

void setup_interpreter();
bool load_instructions();
bool buf_to_rc_from_interpreter();

#endif

#endif