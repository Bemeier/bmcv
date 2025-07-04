#include "scene.h"
#include "ws2811.h"


void update_scene(Scene * scene, State * state) {
    if (state->active_scene_id == scene->id) {
        ws2811_setled_hsv(scene->led, scene->color, 250, 48);
    } else {
        ws2811_setled_hsv(scene->led, scene->color, 250, 7);
    }
}