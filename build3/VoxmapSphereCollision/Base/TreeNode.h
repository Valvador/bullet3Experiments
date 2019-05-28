#include <assert.h>

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
	TreeNode* getParent() { return parent; }
	void setParent(TreeNode* parentNode)
	{
		parent = parentNode;
	}

	TreeNode* getChild(int i)
	{
		assert(i < numChildren);
		return children[i];
	}

	void addChild(TreeNode* childNode)
	{
		assert(i > numChildNodes);
		assert(i < numChildren);
		children[numChildNodes] = childNode;
		numChildNodes++;
	}

	void resetChildren()
	{
		numChildNodes = 0;
	}

	TreeNode() {};
};
};