// Nothing to see here.
//
// The wasm module is built with --no-entry and its whole surface is the
// bmcv_sim_* function list in sim/CMakeLists.txt, but Emscripten still wants a
// translation unit to link. Everything real lives in bmcv_sim.c and sim_rt.c.

typedef int bmcv_wasm_translation_unit_is_not_empty;
