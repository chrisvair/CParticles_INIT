#ifndef VEC2_H_
#define VEC2_H_

// ------------------------------------------------

typedef struct Vec2 {
  float x;
  float y;
} Vec2;

// ------------------------------------------------
Vec2 create_vec2(float x, float y);
Vec2 scalar_mult(Vec2 vect, float scalar);
Vec2 vect_sum(Vec2 vect1, Vec2 vect2);
Vec2 vect_sub(Vec2 vect1, Vec2 vect2);
float scalar_product(Vec2 vect1, Vec2 vect2);
Vec2 normalisation(Vec2 vect);
float norm(Vec2 vect);
Vec2 normal_plan(Vec2 start, Vec2 end);
Vec2 product(Vec2 vect1, Vec2 vect2);
// ------------------------------------------------

#endif