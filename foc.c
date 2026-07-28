#include <stdio.h> 
#include <math.h> 

typedef struct {

    float x; //first slot
    float y; //second slot
} vec2; 

typedef struct {
    float x;
    float y;
    float z;
} vec3;

typedef struct {
    float Kp;
    float Ki;
    float integral;
} PIController;

float pi_update(PIController* pi, float error) {
    pi->integral += error; 
    float result = (pi->Kp * error) + (pi->Ki * pi->integral);
    return result;
}

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

//invClarke
vec3 invClarke(vec2 ab) {
    vec3 out; 
    out.x = ab.x; 
    out.y = (ab.y * sqrtf(3.0f) - ab.x) / 2;
    out.z = ((-ab.x) - ab.y * sqrtf(3.0f)) / 2;
    return out;
}

int main(void) {
    vec2 ab = clarke(1.0f, -0.5f); //this gets alpha and beta 
    printf("alpha = %f, beta = %f\n", ab.x, ab.y);

    vec2 dq2 = park(ab, 1.5708f); //this rotates it forward by 90 degrees
    printf("d = %f, q = %f\n", dq2.x, dq2.y);

    vec2 ab2 = invPark(dq2, 1.5708f); //rotates it backward, last step, invpark 
    printf("alpha2 = %f, beta2 = %f\n", ab2.x, ab2.y);

    vec3 abc = invClarke(ab);
    printf("ia = %f, ib = %f, ic = %f\n", abc.x, abc.y, abc.z);
    
    PIController myPI = {2.0f, 0.5f, 0.0f};

    float out1 = pi_update(&myPI, 1.0f);
    printf("out1 = %f, integral = %f\n", out1, myPI.integral);

    float out2 = pi_update(&myPI, 1.0f);
    printf("out2 = %f, integral = %f\n", out2, myPI.integral);
    
    return 0;
    }


