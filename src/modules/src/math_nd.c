#define DEBUG_MODULE "MATH_ND"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "debug.h"
#include "math_nd.h"

struct vecX mkvecX(int length, float vec[MAX_LENGTH]) {
	struct vecX v = {.length = length};
    memcpy(v.vec,vec, sizeof(v));
	return v;
}
// construct a vector with the same value repeated for x, y, and z.
struct vecX vrepeatX(int length, float x) {
    float vec[MAX_LENGTH];
    for(int i = 0; i < length; i++){
        vec[i] = x;
    }
	return mkvecX(length, vec);
}
// construct a zero-vector.
struct vecX vzeroX(int length) {
	return vrepeatX(length, 0.0f);
}
// construct the i'th basis vector, i.e. vbasis(0) == (1, 0, 0).
struct vecX vbasisX(int length, int i) {
	float vec[MAX_LENGTH];
	vec[i] = 1.0f;
	return mkvecX(length, vec);
}

struct vecX vneltX(int length, struct vecX v) {
    struct vecX vec = {.length = length};
    for(int i = 0; i < length; i++) {
        vec.vec[i] = v.vec[i];
    }
    return vec;
}

void vcpyX(struct vecX dest, struct vecX src) {
    if(dest.length == src.length) {
        memcpy(dest.vec, src.vec, sizeof(src.length));
    }
    else {
        DEBUG_PRINT("Dimension Error!");
        memcpy(dest.vec, vzeroX(dest.length).vec, sizeof(dest.length));
    }
}

// multiply a vector by a scalar.
struct vecX vXscl(float s, struct vecX v) {
    struct vecX vs = {.length = v.length};
    for(int i = 0; i < v.length; i++) {
        vs.vec[i] = v.vec[i] * s;
    }
	return vs;
}
// negate a vector.
struct vecX vXneg(struct vecX v) {
	return vXscl(-1.0f, v);
}
// divide a vector by a scalar.
// does not perform divide-by-zero check.
struct vecX vXdiv(struct vecX v, float s) {
	for(int i = 0; i < v.length; i++) {
        v.vec[i] /= s;
    }
	return v;
}
// add two vectors.
//if vectors are not with same length return 0
struct vecX vXadd(struct vecX a, struct vecX b) {
    if(a.length == b.length)
    {
        struct vecX v = {.length = a.length};
        for(int i = 0; i < v.length; i++) {
            v.vec[i] = a.vec[i] + b.vec[i];
        }
        return v;
    }
    else {
        DEBUG_PRINT("Dimension Error in Addtion!");
	    return vzeroX(a.length);
    }
}

// subtract a vector from another vector.
struct vecX vXsub(struct vecX a, struct vecX b) {
	return vXadd(a, vXneg(b));
}

// test if any element of a vector is NaN.
bool vXisnan(struct vecX v) {
    int nan = 0;
    for(int i = 0; i < v.length; i++) {
        nan += isnan(v.vec[i]);
    }
	return (nan > 0);
}

struct vecX vXadd3(struct vecX a, struct vecX b, struct vecX c) {
    if(a.length == b.length && a.length == c.length)
	    return vXadd(vXadd(a, b), c);
    else {
        DEBUG_PRINT("Dimension Error in Addtion!");
	    return vzeroX(a.length);
    }
}
// add 4 vectors.
struct vecX vXadd4(struct vecX a, struct vecX b, struct vecX c, struct vecX d) {
	if(a.length == b.length && a.length == c.length && a.length == d.length)
	    return vXadd(vXadd(a, b), vXadd(c, d));
    else {
        DEBUG_PRINT("Dimension Error in Addtion!");
	    return vzeroX(a.length);
    }   
}
// subtract b and c from a.
struct vecX vXsub2(struct vecX a, struct vecX b, struct vecX c) {
	if(a.length == b.length && a.length == c.length)
	    return vXadd3(a, vXneg(b), vXneg(c));
    else {
        DEBUG_PRINT("Dimension Error in Substraction!");
	    return vzeroX(a.length);
    }
}

struct matXX mzeroXX(int r, int c) {
	struct matXX m = {.colums = c, .rows = r};
	return m;
}

struct matXX mkmatXX(int r, int c, float mat[MAX_LENGTH][MAX_LENGTH]) {
    struct matXX m = {.colums = c, .rows = r};
	memcpy(m.mat, mat, sizeof(m));
	return m;
}

// matrix transpose.
struct matXX mXXtranspose(struct matXX m) {
	struct matXX mt = {.colums = m.rows, .rows = m.colums};
	for (int i = 0; i < MAX_LENGTH; ++i) {
		for (int j = 0; j < MAX_LENGTH; ++j) {
			mt.mat[i][j] = m.mat[j][i];
		}
	}
	return mt;
}
// multiply a matrix by a scalar.
struct matXX mXXscl(float s, struct matXX a) {
	struct matXX sa = {.colums = a.colums, .rows = a.rows};
	for (int i = 0; i < sa.rows; ++i) {
		for (int j = 0; j < sa.colums; ++j) {
			sa.mat[i][j] = s * a.mat[i][j];
		}
	}
	return sa;
}
// negate a matrix.
struct matXX mXXneg(struct matXX a) {
	return mXXscl(-1.0, a);
}

// multiply a matrix by a vector.
struct vecX mvXXmul(struct matXX a, struct vecX v) {
	if(a.colums == v.length) {
        struct vecX vec = {.length = a.rows};
        for(int i = 0; i < a.rows; i++) {
            float tmp = 0;
            for(int j = 0; j < a.colums; j++) {
                tmp += a.mat[i][j] * v.vec[j];
            }
            vec.vec[i] = tmp;
        }
        if(!vXisnan(vec))
            return vec;
        else {
            DEBUG_PRINT("Element in vector is nan!");
            return vzeroX(a.rows);
        }
    }
    else {
        DEBUG_PRINT("Dimension Error in Multiplication!");
        DEBUG_PRINT("Rows: %i * Length; %i", a.rows, v.length);
	    return vzeroX(a.rows);
    }
}
