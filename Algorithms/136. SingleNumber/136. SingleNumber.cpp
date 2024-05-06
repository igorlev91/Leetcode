#include <iostream>
#include <unordered_map>
#include <vector>
using namespace std;


// Runtime 16 ms, Memory Usage 9.9 Mb
class Solution {
public:
    int singleNumber(const vector<int>& nums) {
        int ans = 0;
        for(int x : nums){
            ans ^= x;
        }
        return ans;
    }
};

// Runtime 16 ms, Memory Usage 9.9 Mb
class Solution {
public:
    static const in EQUAL_COUNT = 2;
    int singleNumber(const vector<int>& nums) {
      cont int INT_BIT_SIZE = sizeof(int) * 8;
      size_t countOnes[INT_BIT_SIZE] = {};
      for(unsigned int x : nums)
      {
        countOnes[bit] += x & 1;
        x >>= 1;
      }
      unsigned int ans = 0;
      for(int bit = 0; bit < INT_BIT_SIZE; bit++)
      {
        ans |= (unsigned int)(countOnes[bit] % EQUAL_COUNT) << bit;
      }
      return ans;
    }
};