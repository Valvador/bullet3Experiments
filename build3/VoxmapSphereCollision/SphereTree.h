#include "Base/TreeNode.h"
#include "Math/Vector3.h"


namespace VSC
{
template <int numChildrenPerNode>
class SphereTreeNode : protected TreeNode<numChildrenPerNode>
{
private:
	int primaryNode;
	float radius;
	Vector3 position;

public:
	void addChild(SphereTreeNode* node)
	{
		TreeNode<numChildrenPerNode>::addChild(node);
	}

	SphereTreeNode<numChildrenPerNode>* getChild(int i)
	{
		return static_cast<SphereTreeNode<numChildrenPerNode>*>(TreeNode<numChildrenPerNode>::getChild(i));
	}

	void setPrimaryNode(int node)
	{
		assert(node < numChildNodes);
		primaryNode = node;
	}

	SphereTreeNode<numChildrenPerNode>(float _radius, const Vector3& _position)
		: position(_position)
		, radius(_radius)
	{};
};

};