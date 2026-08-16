#ifndef INC_LIB_DRIFT_H_
#define INC_LIB_DRIFT_H_

// A slow random wander: octaves of a smoothed triangle, summed in the phase
// domain. The invisible hand rather than the sequencer.
//
// Nothing here is the stepped engine. There are no slots, no ties and no
// pattern - and, because every octave's rate is a whole number of cycles, no
// correction either. The waveform closes on itself by construction, its mean
// over a cycle is exactly zero at every setting, and its peak-to-peak is known
// in advance, so the level work the stepped modes need a per-channel
// measurement for is arithmetic here.
//
//   phase  [0,1)  position in the cycle, PLL-corrected by the caller
//   shape  [-1,1] how much detail, and which wander. Left is a single smooth
//                 swell per cycle - two turns of direction; right is three
//                 octaves of detail on top of it, and sixteen. Not periodic,
//                 unlike SHP in the stepped modes: detail is a ramp and its
//                 ends are opposite ends, the same way their MOD's density is.
//                 The wander itself does wrap - each octave advances a whole
//                 number of turns across the sweep - so it is only the detail
//                 that makes the two ends different.
//   mod    [-1,1] where it sits: leaning low with the peaks still reaching,
//                 through symmetric, to bunched against both rails.
//
// Returns [-1,1]. Stateless and deterministic, like every other shape here.
float drift_value(float phase, float shape, float mod);

#endif /* INC_LIB_DRIFT_H_ */
