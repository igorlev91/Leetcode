#include "ListNode.h"

#include "gtest/gtest.h"

using CommonDefs::ListNode;

namespace
{

class Solution
{
public:
    ListNode* reverseList(ListNode* head)
    {
        if (head == nullptr)
            return head;
        ListNode* current = head;
        ListNode* newHead = nullptr;
        while (current->next != nullptr)
        {
            ListNode* next = current->next;
            current->next = newHead;
            newHead = current;
            current = next;
        }
        current->next = newHead;
        return current;
    }
};

}

using CommonDefs::createLinkedList;
using CommonDefs::checkAndDeleteLinkedList;

namespace ReverseLinkedListTask
{

TEST(ReverseLinkedListTaskTests, Examples)
{
    Solution solution;
    checkAndDeleteLinkedList({5, 4, 3, 2, 1}, solution.reverseList(createLinkedList({1, 2, 3, 4, 5}, false).get()));
}

}