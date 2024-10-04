#ifndef _MATH_ND_H_
#define _MATH_ND_H_

#pragma once

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define MAX_LENGTH 9

struct vecX {
    const int length;
    float vec[MAX_LENGTH];
};

struct matXX {
    const int rows;
    const int colums;
    float mat[MAX_LENGTH][MAX_LENGTH];
};

static inline struct vecX mkvecX(int length, float vec[MAX_LENGTH]) {
	struct vecX v = {.length length};
    memcpy(v.vec,vec,sizeof(vec));
	return v;
}
// construct a vector with the same value repeated for x, y, and z.
static inline struct vecX vrepeatX(int length, float x) {
    float vec[MAX_LENGTH];
    for(int i = 0; i < length; i++){
        vec[i] = x;
    }
	return mkvecX(length, vec);
}
// construct a zero-vector.
static inline struct vecX vzeroX(int length) {
	return vrepeatX(length, 0.0f);
}
// construct the i'th basis vector, i.e. vbasis(0) == (1, 0, 0).
static inline struct vecX vbasisX(int length, int i) {
	float vec[MAX_LENGTH];
	vec[i] = 1.0f;
	return mkvecX(length, vec);
}

static inline struct vecX vneltX(int length, struct vecX v) {
    struct vecX vec = {.length length};
    for(int i = 0; i < length; i++) {
        vec.vec[i] = v.vec[i];
    }
    return vec;
}

// multiply a vector by a scalar.
static inline struct vecX vscl(float s, struct vecX v) {
    for(int i = 0; i < v.length; i++) {
        v.vec[i] *= s;
    }
	return v;
}
// negate a vector.
static inline struct vecX vneg(struct vecX v) {
	return vscl(-1.0f, v);
}
// divide a vector by a scalar.
// does not perform divide-by-zero check.
static inline struct vecX vdiv(struct vecX v, float s) {
	for(int i = 0; i < v.length; i++) {
        v.vec[i] /= s;
    }
	return v;
}
// add two vectors.
//if vectors are not with same length return 0
static inline struct vecX vadd(struct vecX a, struct vecX b) {
    if(a.length == b.length)
    {
        vecX v = {.length a.length};
        for(int i = 0; i < v.length; i++) {
            v.vec[i] = a.vec[i] + b.vec[i];
        }
        return v;
    }
    else
	return vzeroX(a.length);
}

// subtract a vector from another vector.
static inline struct vecX vsub(struct vecX a, struct vecX b) {
	return vadd(a, vneg(b));
}

// test if any element of a vector is NaN.
static inline bool visnan(struct vecX v) {
    int nan = 0;
    for(int i = 0; i < v.length; i++) {
        nan += isnan(v.vec[i]);
    }
	return (nan > 0);
}

static inline struct vecX vadd3(struct vecX a, struct vecX b, struct vecX c) {
    if(a.length == b.length && a.length == c.length)
	    return vadd(vadd(a, b), c);
    else
        return vzeroX(a.length);
}
// add 4 vectors.
static inline struct vecX vadd4(struct vecX a, struct vecX b, struct vecX c, struct vecX d) {
	if(a.length == b.length && a.length == c.length && a.length == d.length)
	    return vadd(vadd(a, b), vadd(c, d));
    else
        return vzeroX(a.length);
}
// subtract b and c from a.
static inline struct vecX vsub2(struct vecX a, struct vecX b, struct vecX c) {
	if(a.length == b.length && a.length == c.length)
	    return vadd3(a, vneg(b), vneg(c));
    else
        return vzeroX(a.length);
}

static inline struct matXX mzeroXX(int r, int c) {
	struct matXX m;
    m.rows = r;
    m.colums = c;
	for (int i = 0; i < r; ++i) {
		for (int j = 0; j < c; ++j) {
			m.mat[i][j] = 0;
		}
	}
	return m;
}

static inline struct matXX mkmatXX(int r, int c, float mat[MAX_LENGTH][MAX_LENGTH]) {
    struct matXX m;
    m.rows = r;
    m.colums = c;
	memcpy(m.mat, mat, sizeof(m));
	return m;
}

// matrix transpose.
static inline struct matXX mtranspose(struct matXX m) {
	struct matXX mt = {.colums m.rows, .rows m.colums};
	for (int i = 0; i < MAX_LENGTH; ++i) {
		for (int j = 0; j < MAX_LENGTH; ++j) {
			mt.mat[i][j] = m.mat[j][i];
		}
	}
	return mt;
}
// multiply a matrix by a scalar.
static inline struct matXX mscl(float s, struct matXX a) {
	struct matXX sa = {.colums a.colums, .rows a.rows};
	for (int i = 0; i < sa.rows; ++i) {
		for (int j = 0; j < sa.colums; ++j) {
			sa.mat[i][j] = s * a.mat[i][j];
		}
	}
	return sa;
}
// negate a matrix.
static inline struct matXX mneg(struct matXX a) {
	return mscl(-1.0, a);
}

// multiply a matrix by a vector.
static inline struct vecX mvmul(struct matXX a, struct vecX v) {
	if(a.colums == v.length) {
        struct vecX vec = {.length a.rows};
        for(int i = 0; i < a.rows; i++) {
            float tmp = 0;
            for(int j = 0; j < a.colums; j++) {
                tmp += a.mat[i][j] * v.vec[j];
            }
            vec.vec[i] = tmp;
        }
        if(!visnan(vec))
            return vec;
        else
            return vzeroX(a.rows);
    }
    else
	    return vzeroX(a.rows);
}

#endif