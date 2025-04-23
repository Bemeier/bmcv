#ifndef INC_HELPERS_H_
#define INC_HELPERS_H_

static inline int16_t smin(int16_t a, int16_t b) {
    return b ^ ((a ^ b) & -(a < b));
}

static inline int16_t smax(int16_t a, int16_t b) {
    return a ^ ((a ^ b) & -(a < b));
}

static inline int16_t sclamp(int16_t x, int16_t min_val, int16_t max_val) {
    x = x ^ ((x ^ min_val) & -(x < min_val));
    x = x ^ ((x ^ max_val) & -(x > max_val));
    return x;
}

#endif /* INC_HELPERS_H_ */
