#ifndef INC_LIB_WAVETABLE_H_
#define INC_LIB_WAVETABLE_H_

// Reading the generated wavetable (wavetables.h): one bilinear lookup across
// the two axes it is stored on, phase and shape.
//
// This file used to also hold wave_fn(), a procedural morph between saw,
// triangle, sine and square with its own phase warp. Nothing ever called it -
// channel_compute() has always used the table - so it was two shape engines
// where only one ran, and the dead one is where the phase warp in helpers.h
// was first written.

// phase [0,1), shape [-1,1]. Returns the table's own units, INT16 scale.
float wavetable_lookup(float phase, float shape);

#endif /* INC_LIB_WAVETABLE_H_ */
