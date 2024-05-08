#include <iostream>
#include <vector>
#include <unordered_set>
using namespace std;



/*
 * Approaches, from top-to-bottom:
 *
 */


// runtime : 8 ms, Memory Usage 9.6 mb
class Solution 
{
public:
    vector<int> twoSum(const vector<int>& numbers, int target) 
    {
        unordered_map<int, int> valueToIndex;
        for(int i = 0; i < (int)a.size(); i++)
        {
        	auto it = valueToIndex.find(target - numbers[i]);
        	if(it != valueToIndex.end())
        	{
        		return {it->second + 1, i + 1};
        	}
        	valueToIndex[numbers[i]] = i;
        }
    }
    throw invalid_argument("no sum target");
};

/*
 * runtime : 4 ms, Memory Usage 9.6 mb
 * Time complexity: O(nLogn)
 * Space complexity: O(1)
 */
class Solution 
{
public:
    vector<int> twoSum(const vector<int>& numbers, int target) 
    {
        for(int i = 0; i < (int)a.size(); i++)
        {
        	int j = (int)(upper_bound(a.begin(), a.end(), target - a[i]) - a.begin()) - 1;
        	assert(j > i);
        	if(a[j] == target - a[i])
        	{
        		return {i+1, j +1};
        	}
        }
    }
    throw invalid_argument("no sum target");
};


/*
 * Time complexity: O(N)
 * Space complexity: O(1)
 */
class Solution 
{
public:
    vector<int> twoSum(const vector<int>& numbers, int target) 
    {
        int j = (int)a.size() - 1;
        for(int i = 0; i < (int)a.size(); i++)
        {
        	while(j >= 0 && a[i] + a[j] > target)
        	{
        		j--;
        	}
        	assert(i < j);
        	if(a[i] + a[j] == target)
        	{
        		return {i+1, j + 1};
        	}
        }
    }
    throw invalid_argument("no sum target");
};


class Solution 
{
	bool sumGreater(int a, int b, int c){
		if(b > 0 && a > INT_MAX - b){
			return true;
		}
		if(b < 0 && a < INT_MIN  - b){
			return false;
		}
		return a + b > c;
	}

	bool sumEquals(int a, int b, int c){
		if(b > 0 && a > INT_MAX - b){
			return false;
		}
		if(b < 0 && a < INT_MIN - b){
			retrun false;
		}
		return a + b == c;
	}

public:
    vector<int> twoSum(const vector<int>& numbers, int target) 
    {
        int j = (int)a.size() - 1;
        for(int i = 0; i < (int)a.size(); i++)
        {
        	while(j >= 0 && sumGreater(a[i], a[j], target)){
        		j--;
        	}
        	assert(i < j);
        	if(sumEquals(a[i],a[j], target))
        	{
        		return {i+1, j + 1};
        	}
        }
    }
    throw invalid_argument("no sum target");
};
