#include "TreeNode.h"
#include "TreeNodeUtils.h"

#include "gtest/gtest.h"

using CommonLib::TreeNode;

namespace
{

class Solution
{
public:
    TreeNode* lowestCommonAncestor(TreeNode* root, TreeNode* p, TreeNode* q) const
    {
        TreeNode* left = p->val < q->val ? p : q;
        TreeNode* right = q->val > p->val ? q : p;
        if (root->val > left->val && root->val < right->val)
            return root;
        if (root->val == left->val || root->val == right->val)
            return root;
        return lowestCommonAncestor(right->val < root->val ? root->left : root->right, left, right);
    }
};

}

using CommonLib::Codec;

namespace LowestCommonAncestorOfBinarySearchTreeTask
{

TEST(LowestCommonAncestorOfBinarySearchTreeTaskTests, Examples)
{
    const Solution solution;
    const std::shared_ptr<TreeNode> tree1 = Codec::createTree("[6,2,8,0,4,7,9,null,null,3,5]");
    ASSERT_EQ(6, solution.lowestCommonAncestor(tree1.get(), tree1.get()->left, tree1.get()->right)->val);
    ASSERT_EQ(2, solution.lowestCommonAncestor(tree1.get(), tree1.get()->left, tree1.get()->left->right)->val);
    const std::shared_ptr<TreeNode> tree2 = Codec::createTree("[2,1]");
    ASSERT_EQ(2, solution.lowestCommonAncestor(tree1.get(), tree2.get(), tree2.get()->left)->val);
}

}