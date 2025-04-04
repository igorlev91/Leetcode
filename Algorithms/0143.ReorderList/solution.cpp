#include <vector>

#include "ListNode.h"

#include "gtest/gtest.h"

using CommonDefs::ListNode;

namespace
{

class Solution
{
public:
    void reorderList(ListNode* head)
    {
        if (head == nullptr)
            return;
        std::vector<ListNode*> nodes;
        ListNode *current = head;
        while (current != nullptr)
        {
            nodes.push_back(current);
            current = current->next;
        }
        size_t front = 0;
        size_t back = nodes.size() - 1;
        while (front < back)
        {
            nodes[front++]->next = nodes[back];
            nodes[back--]->next = nodes[front];
        }
        nodes[front]->next = nullptr;
    }
};

}

using CommonDefs::createLinkedList;
using CommonDefs::checkAndDeleteLinkedList;

namespace ReorderListTask
{

TEST(ReorderListTaskTests, Examples)
{
    Solution solution;
    ListNode* head1 = createLinkedList({1, 2, 3, 4}, false).get();
    solution.reorderList(head1);
    checkAndDeleteLinkedList({1, 4, 2, 3}, head1);
    ListNode* head2 = createLinkedList({1, 2, 3, 4, 5}, false).get();
    solution.reorderList(head2);
    checkAndDeleteLinkedList({1, 5, 2, 4, 3}, head2);
}

TEST(ReorderListTaskTests, FromWrongAnswers)
{
    Solution solution;
    ASSERT_NO_THROW(solution.reorderList(nullptr));
}

}