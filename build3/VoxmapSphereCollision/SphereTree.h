#include "Base/TreeNode.h"
#include "Math/Vector3.h"


namespace VSC
{
template <int numChildrenPerNode>
class SphereTreeNode : public TreeNode<numChildrenPerNode>
{
private:
	int primaryNode = -1;
	float radius = 0.0f;
	Vector3 position = Vector3(0);

public:
	Vector3 getPosition() { return position; }
	float getRadius() { return radius; }

	void addChild(SphereTreeNode* node)
	{
		assert(node);
		int currentNumChildren = TreeNode<numChildrenPerNode>::getNumChildren();
		position = (position * currentNumChildren + node->position) * (1.0f / float(currentNumChildren + 1));
		TreeNode<numChildrenPerNode>::addChild(node);
	}

	SphereTreeNode<numChildrenPerNode>* getChild(int i)
	{
		return static_cast<SphereTreeNode<numChildrenPerNode>*>(TreeNode<numChildrenPerNode>::getChild(i));
	}

	void setPrimaryNode(int node)
	{
		assert(node < getNumChildren());
		primaryNode = node;
	}

	void computeRadius()
	{
		radius = 0;
		for (int i = 0; i < getNumChildren(); i++)
		{
			SphereTreeNode<numChildrenPerNode>* child = getChild(i);
			float distance = sqrtf((child->getPosition() - getPosition()).sqrMagnitude()) + child->getRadius();
			if (distance > radius)
			{
				radius = distance;
			}
		}
	}

	SphereTreeNode<numChildrenPerNode>(float _radius, const Vector3& _position)
		: position(_position)
		, radius(_radius)
	{};

	SphereTreeNode<numChildrenPerNode>()
		: position(Vector3(0.f))
		, radius(0.f)
	{};

	~SphereTreeNode<numChildrenPerNode>()
	{
		for (int i = 0; i < getNumChildren(); i++)
		{
			delete getChild(i);
		}
	}
};

class SphereTree
{
public:
	const static int SphereTreeNodeMax = 4;
private:
	SphereTreeNode<SphereTreeNodeMax>* rootNode;
public:
	void clear() { delete rootNode; rootNode = nullptr; }
	SphereTreeNode<SphereTreeNodeMax>* getRootNode() const { return rootNode; }

	SphereTree(SphereTreeNode<SphereTreeNodeMax>* root) : rootNode(root) {};
	SphereTree() : rootNode(nullptr) {};
	~SphereTree() { assert(rootNode == nullptr); }
};
};