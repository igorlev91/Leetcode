#include "TreeNode.h"

#include "gtest/gtest.h"

using CommonDefs::TreeNode;

namespace
{

class Solution
{
public:
    bool isValidBST(TreeNode* root) const
    {
        if (root == nullptr)
            return true;
        int minValue = 0;
        int maxValue = 0;
        return isValidBST(root, minValue, maxValue);
    }

private:
    bool isValidBST(TreeNode* root, int &minValue, int &maxValue) const
    {
        minValue = root->val;
        maxValue = root->val;
        if (root->left != nullptr)
        {
            int leftMinValue = 0;
            int leftMaxValue = 0;
            if (!isValidBST(root->left, leftMinValue, leftMaxValue))
                return false;
            if (leftMaxValue >= root->val)
                return false;
            minValue = leftMinValue;
        }
        if (root->right != nullptr)
        {
            int rightMinValue = 0;
            int rightMaxValue = 0;
            if (!isValidBST(root->right, rightMinValue, rightMaxValue))
                return false;
            if (rightMinValue <= root->val)
                return false;
            maxValue = rightMaxValue;
        }
        return true;
    }
};

}

using CommonDefs::createTreeHolder;

namespace ValidateBinarySearchTreeTask
{

TEST(ValidateBinarySearchTreeTaskTests, Examples)
{
    const Solution solution;
    ASSERT_EQ(true, solution.isValidBST(createTreeHolder(new TreeNode(2, new TreeNode(1), new TreeNode(3))).get()));
    ASSERT_EQ(false, solution.isValidBST(createTreeHolder(new TreeNode(5, new TreeNode(1), new TreeNode(4, new TreeNode(3), new TreeNode(6)))).get()));
}

TEST(ValidateBinarySearchTreeTaskTests, FromWrongAnswers)
{
    const Solution solution;
    ASSERT_EQ(true, solution.isValidBST(nullptr));
    ASSERT_EQ(false, solution.isValidBST(createTreeHolder(new TreeNode(10, new TreeNode(5), new TreeNode(15, new TreeNode(6), new TreeNode(20)))).get()));
}

}