#include <stdio.h> 
#include <math.h> 

typedef struct {

    float x; //first slot
    float y; //second slot
} vec2; 

// if a variable's type is a plain number type (float, int), use it directly. If a variables type is a struct (like vec2),
// you must dot into a field to get a single number out of it


//Clarke: 3 phase currents into 2 axis (alpha and beta)
vec2 clarke(float ia, float ib) {
    float ic = -ia - ib;                // 3rd current is forced (sum to 0)
    vec2 out;                           // make an empty bundle 
    out.x = ia;                         // alpha 
    out.y = (ib - ic) / sqrtf(3.0f);    // beta  
    return out;                         // hand back the whole bundle 
}


//Park
vec2 park(vec2 ab, float theta) {
    vec2 out; 
    out.x = ab.x * cosf(theta) + ab.y * sinf(theta); // d
    out.y = -ab.x * sinf(theta) + ab.y * cosf(theta); // q
    return out; 
}


//invPark
vec2 invPark(vec2 dq, float theta) {
    vec2 out; 
    out.x = dq.x * cosf(theta) + -dq.y * sinf(theta); 
    out.y = dq.x * sinf(theta) + dq.y * cosf(theta);
    return out;
    }


int main(void) {
    vec2 ab = clarke(1.0f, -0.5f); //this gets alpha and beta 
    printf("alpha = %f, beta = %f\n", ab.x, ab.y);

    vec2 dq2 = park(ab, 1.5708f); //this rotates it forward by 90 degrees
    printf("d = %f, q = %f\n", dq2.x, dq2.y);

    vec2 ab2 = invPark(dq2, 1.5708f); //rotates it backward, last step, invpark 
    printf("alpha2 = %f, beta2 = %f\n", ab2.x, ab2.y);

    return 0;
    }




