#include "Math/Vector3.h"

namespace VSC
{
struct ContactPoint
{
	Vector3 position;
	Vector3 normal;
};

class CollisionAlgorithm
{
protected: 
	CollisionShape* shape0 = nullptr;
	CollisionShape* shape1 = nullptr;

public:
	void assignShape0(CollisionShape* s0) { shape0 = s0; }
	void assignShape1(CollisionShape* s1) { shape1 = s1; }

	virtual int getMaxContacts() = 0;
	virtual int getNumContacts() = 0;
	virtual const ContactPoint& getContactPoint(int i) = 0;
	virtual bool doCollide() = 0;
};
}