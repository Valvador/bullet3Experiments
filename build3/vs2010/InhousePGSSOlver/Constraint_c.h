#include "DMatrix.h"
#include "RigidBody_c.h"
#include <assert.h>

#ifndef CONSTRAINT_C_H
#define CONSTRAINT_C_H

public class Constraint_c
{
public:
	virtual ~Constraint_c(){}
	virtual DMatrix GetPenalty() { assert(0); return DMatrix(0, 0); };
	virtual DMatrix GetJacobian(const RigidBody_c* rb) { assert(0); return DMatrix(0, 0); };
	virtual int GetDimension() const { assert(0); return 0; };
};

#endif