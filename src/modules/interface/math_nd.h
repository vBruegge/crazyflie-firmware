#ifndef _MATH_ND_H_
#define _MATH_ND_H_

#pragma once

#define MAX_LENGTH 9

struct vecX {
    const uint8_t length;
    float vec[MAX_LENGTH];
};

struct matXX {
    const uint8_t rows;
    const uint8_t colums;
    float mat[MAX_LENGTH][MAX_LENGTH];
};

struct vecX mkvecX(int length, float vec[MAX_LENGTH]);

// construct a vector with the same value repeated for x, y, and z.
struct vecX vrepeatX(int length, float x);
// construct a zero-vector.
struct vecX vzeroX(int length);
// construct the i'th basis vector, i.e. vbasis(0) == (1, 0, 0).
struct vecX vbasisX(int length, int i);

struct vecX vneltX(int length, struct vecX v);

void vcpyX(struct vecX dest, struct vecX src);

// multiply a vector by a scalar.
struct vecX vXscl(float s, struct vecX v);
// negate a vector.
struct vecX vXneg(struct vecX v);
// divide a vector by a scalar.
// does not perform divide-by-zero check.
struct vecX vXdiv(struct vecX v, float s);
// add two vectors.
//if vectors are not with same length return 0
struct vecX vXadd(struct vecX a, struct vecX b);

// subtract a vector from another vector.
struct vecX vXsub(struct vecX a, struct vecX b);

// test if any element of a vector is NaN.
bool vXisnan(struct vecX v);

struct vecX vXadd3(struct vecX a, struct vecX b, struct vecX c);
// add 4 vectors.
struct vecX vXadd4(struct vecX a, struct vecX b, struct vecX c, struct vecX d);
// subtract b and c from a.
struct vecX vXsub2(struct vecX a, struct vecX b, struct vecX c);

struct matXX mzeroXX(int r, int c);

struct matXX mkmatXX(int r, int c, float mat[MAX_LENGTH][MAX_LENGTH]);

// matrix transpose.
struct matXX mXXtranspose(struct matXX m);
// multiply a matrix by a scalar.
struct matXX mXXscl(float s, struct matXX a);
// negate a matrix.
struct matXX mXXneg(struct matXX a);

// multiply a matrix by a vector.
struct vecX mvXXmul(struct matXX a, struct vecX v);

#endif