#include <algorithm>
#include <vector>

#include "TreeNode.h"
#include "TreeNodeUtils.h"

#include "gtest/gtest.h"

using CommonLib::TreeNode;

namespace
{

class Solution
{
public:
    std::vector<std::vector<int>> levelOrderBottom(TreeNode* root) const
    {
        std::vector<std::vector<int>> dest;
        processTree(root, 0, dest);
        std::reverse(dest.begin(), dest.end());
        return dest;
    }

private:
    void processTree(TreeNode* root, size_t level, std::vector<std::vector<int>> &dest) const
    {
        if (root == nullptr)
            return;
        if (level == dest.size())
            dest.emplace_back();
        dest[level].push_back(root->val);
        processTree(root->left, level + 1, dest);
        processTree(root->right, level + 1, dest);
    }
};

}

using CommonLib::createTreeHolder;

namespace BinaryTreeLevelOrderTraversalIITask
{

TEST(BinaryTreeLevelOrderTraversalIITaskTests, Examples)
{
    const Solution solution;
    ASSERT_EQ(std::vector<std::vector<int>>({{15, 7}, {9, 20}, {3}}), solution.levelOrderBottom(createTreeHolder(new TreeNode(3, 9, new TreeNode(20, 15, 7))).get()));
}

}