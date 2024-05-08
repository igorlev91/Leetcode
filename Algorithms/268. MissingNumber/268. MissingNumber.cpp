#include <iostream>
#include <unordered_set>
#include <vector>
using namespace std;

/*
 * Approaches:
 *
 * 1) Fill an unordered_set with all the numbers in the range of [0, n].
 * Then, loop over the input vector and remove each number from the unordered_set.
 * The missing number will be the only number left in the unordered_set.
 *
 * Time complexity: O(n)
 * Space complexity: O(n)
 *
 * 2) Sum up all the numbers in the range [0, n].
 * Then, loop through the input vector and subtract each number from the sum.
 * The value of the sum will be the missing number.
 *
 * Time complexity: O(n)
 * Space complexity: O(1)
 *
 * 3) Use bit manipulation, specifically, XOR, to single out the missing number.
 * Time complexity: O(n)
 * Space complexity: O(1)
 */

int missingNumber(vector<int>& nums)
{
    if(nums.empty())
    {
        return 0;
    }

    unordered_set<int> numbers;

    int n=int(nums.size());

    for(int number=0;number<=n;++number)
    {
        numbers.insert(number);
    }

    for(int number : nums)
    {
        numbers.erase(number);
    }

    return *numbers.begin();
}

int missingNumber(vector<int> & nums)
{
    if(nums.empty())
    {
        return 0;
    }

    int n=int(nums.size());

    int sum=0;

    for(int number=0;number<=n;++number)
    {
        sum+=number;
    }

    for(int number : nums)
    {
        sum-=number;
    }

    return sum;
}

int missingNumber(vector<int> & nums)
{
    if(nums.empty())
    {
        return true;
    }

    int n=int(nums.size());

    int result=n;

    int i=0;

    for(int number : nums)
    {
        result^=number;

        result^=i;

        i++;
    }

    return result;
}

class Solution{
public:
	int missingNumber(const vector<int>& a){
		int n = (int)a.size();
		long long sum = n * (n + 1LL) / 2;
		for(int v : a){
			sum -= v;
		}
		return (int)sum;
	}
};