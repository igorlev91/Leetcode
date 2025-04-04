#include "TreeNode.h"

#include "gtest/gtest.h"

using CommonDefs::TreeNode;

namespace
{

class Solution
{
public:
    bool isSymmetric(TreeNode* root) const
    {
        if (root == nullptr)
            return true;
        return isSymmetric(root->left, root->right);
    }

private:
    bool isSymmetric(TreeNode* left, TreeNode* right) const
    {
        if (left == nullptr)
            return right == nullptr;
        if (right == nullptr)
            return false;
        if (left->val != right->val)
            return false;
        if (!isSymmetric(left->left, right->right))
            return false;
        return isSymmetric(left->right, right->left);
    }
};

}

using CommonDefs::createTreeHolder;

namespace SymmetricTreeTask
{

TEST(SymmetricTreeTaskTests, Examples)
{
    const Solution solution;
    ASSERT_EQ(true, solution.isSymmetric(createTreeHolder(new TreeNode(1, new TreeNode(2, new TreeNode(3), new TreeNode(4)), new TreeNode(2, new TreeNode(4), new TreeNode(3)))).get()));
    ASSERT_EQ(false, solution.isSymmetric(createTreeHolder(new TreeNode(1, new TreeNode(2, nullptr, new TreeNode(3)), new TreeNode(2, nullptr, new TreeNode(3)))).get()));
}

TEST(SymmetricTreeTaskTests, FromWrongAnswers)
{
    const Solution solution;
    ASSERT_EQ(true, solution.isSymmetric(nullptr));
}

}