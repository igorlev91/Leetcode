#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>
using namespace std;

/*
 * Approaches, from top-to-bottom:
 *
 * 1) https://en.wikipedia.org/wiki/Boyer%E2%80%93Moore_majority_vote_algorithm
 *
 * Time complexity: O(n)
 * Space complexity: O(1)
 *
 * 2) Sort the vector. Then, the majority element will be at the n/2 index (n=size of the input vector).
 *
 * Time complexity: O(n log n) [where n is the size of the input vector]
 * Space complexity: O(1)
 *
 * 3) Use a hash table to keep track of the frequency for each number. Return the number that has a frequency
 * greater than n/2 (n=number of elements in the input vector)
 *
 * Time complexity: O(n)
 * Space complexity: O(n)
 */


//runtime 20 ms, Memory Usage 11 Mb


class Solution 
{
public:
    int majorityElement(vector<int>& nums) 
    {
      int n = int(nums.size());
      int result = 0;
      int count = 0;
      for(int index = 0; index < n; ++index)
      {
        int current = nums[index];
        if(count == 0)
        {
          result = current;
          count = 1;
        }
        else
        {
          if(current == result)
          {
            count++;
          }
          else
          {
            count--;
          }
        }
      }
      return result;
    }
};

class Solution 
{
public:
    int majorityElement(vector<int>& nums) 
    {
      sort(nums.begin(), nums.end());
      int n=int(nums.size());
      return nums[n/2];
    }
};

//runtime 24 ms, Memory Usage 11.3 Mb
class Solution 
{
public:
    int majorityElement(vector<int>& a) 
    {
        unordered_map<int, size_t> count;
        for(int v : a)
        {
        	count[v]++;
        }
        size_t need = a.size() / 2 + 1;
        for(const auto &kv : count)
        {
        	if(kv.second >= need)
        	{
        		return kv.first;
        	}
        }
        throw invalid_argument("no majority element");
    }
};


//runtime 8 ms, Memory Usage 11.2 Mb
class Solution 
{
public:
    int majorityElement(vector<int>& a) 
    {
      	mt19937 gen(2125);
      	size_t need = a.size() / 2 + 1;
      	while(true){
      		int value = a[uniform_int_distrubation<size_t>(0, a.size()-1)(gen)];
      		size_t count = 0;
      		for(int v : a){
      			count += (v == value);
      		}
      		if(count >= need){
      			return value;
      		}
      	}
    }
};

class Solution 
{
public:
    int majorityElement(vector<int>& nums) 
    {
      	nth_element(a.begin(), a.begin() + a.size()/2, a.end());
      	return a[a.size()/2];
};

