#include <stdlib.h>
#include <stdio.h>
#include "Vec2.h"
#include <math.h> 


Vec2 create_vec2(int x, int y){
    Vec2 vect;
    vect.x = x;
    vect.y = y;
    return vect;
}

Vec2 scalar_mult(Vec2 vect, float scalar){
    Vec2 multiplied_vect;
    multiplied_vect.x = scalar*vect.x;
    multiplied_vect.y = scalar*vect.y;
    return multiplied_vect;
}

Vec2 vect_sum(Vec2 vect1, Vec2 vect2){
    Vec2 summed_vect;
    summed_vect.x = vect1.x + vect2.x;
    summed_vect.y = vect1.y + vect2.y;
    return summed_vect;
}

Vec2 vect_sub(Vec2 vect1, Vec2 vect2){
    Vec2 sub_vect;
    sub_vect.x = vect1.x - vect2.x;
    sub_vect.y = vect1.y - vect2.y;
    return sub_vect;
}

float scalar_product(Vec2 vect1, Vec2 vect2){
    float result = (vect1.x * vect2.x) + (vect1.y * vect2.y);
    return result;
}

Vec2 normalisation(Vec2 vect){
    Vec2 norm_vect;
    float norm = sqrt(vect.x*vect.x + vect.y*vect.y);
    norm_vect.x = vect.x/norm;
    norm_vect.y = vect.y/norm;
    return norm_vect;
}

float norm(Vec2 vect){
    float norm = sqrt(vect.x*vect.x + vect.y*vect.y);
    return norm;
}

Vec2 normal_plan(Vec2 start, Vec2 end){
    Vec2 n = create_vec2(start.y - end.y, end.x - start.x);
    return normalisation(n);
}
