#include <assert.h>
#include <vector>

namespace VSC
{
template <int numChildren>
class TreeNode
{
private:
	int numChildNodes;
	TreeNode* parent;
	TreeNode* children[numChildren];

public:
	int getNumChildren() { return numChildNodes; }

	TreeNode* getParent() { return parent; }

	void setParent(TreeNode* parentNode)
	{
		parent = parentNode;
	}

	TreeNode* getChild(int i)
	{
		assert(i < getNumChildren());
		return children[i];
	}

	void addChild(TreeNode* childNode)
	{
		assert(getNumChildren() < numChildren);
		children[numChildNodes] = childNode;
		childNode->setParent(this);
		numChildNodes++;
	}

	void resetChildren()
	{
		numChildNodes = 0;
	}

	TreeNode() : numChildNodes(0) {};
};
};