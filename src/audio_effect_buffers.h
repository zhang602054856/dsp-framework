#ifndef _AUDIO_EFFECT_BUFFER_H_
#define _AUDIO_EFFECT_BUFFER_H_

#include "macros.h"

//assign the input buffer to modules need specific the index;
void* get_input_pp_buffer(uint8_t index);

//update pcm data from source to input pp buffer. finally use DMA.
void copy_to_pp_buffer(uint8_t index, void *source);
void copy_from_pp_buffer(uint8_t index, void *buffer);

//assign the output buffer to modules, also need specific the index;
void* get_output_pp_buffer(uint8_t index);

//get the module buffer address from here for each output node of modules.
float* get_available_module_buffer();

//each module allocate one coefficient data buffer, size may difference.
float* get_available_coefficent(int size);

//each module allocate one runtime coefficient data buffer, size may difference.
float* get_available_runtime_coefficent(int size);

float* get_available_input_buffer(uint8_t index);



void init_effect_buffers();


#endif