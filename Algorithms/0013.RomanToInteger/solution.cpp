#include <string>
#include <unordered_map>

#include "gtest/gtest.h"

namespace
{

class Solution
{
public:
    int romanToInt(std::string const &s)
    {
        int result = 0;
        for (size_t index = 0; index < s.size(); ++index)
        {
            result += (isSubtraction(s, index) ? -mRomanSymbols[s[index]] : mRomanSymbols[s[index]]);
        }
        return result;
    }

private:
    bool isSubtraction(std::string const &s, size_t index)
    {
        return index < (s.size() - 1) && mRomanSymbols[s[index]] < mRomanSymbols[s[index + 1]];
    }

    std::unordered_map<char, int> mRomanSymbols =
    {
        {'I', 1},
        {'V', 5},
        {'X', 10},
        {'L', 50},
        {'C', 100},
        {'D', 500},
        {'M', 1000}
    };
};

}

namespace RomanToIntegerTask
{

TEST(RomanToIntegerTaskTests, Examples)
{
    Solution solution;
    ASSERT_EQ(3, solution.romanToInt("III"));
    ASSERT_EQ(4, solution.romanToInt("IV"));
    ASSERT_EQ(9, solution.romanToInt("IX"));
    ASSERT_EQ(58, solution.romanToInt("LVIII"));
    ASSERT_EQ(1994, solution.romanToInt("MCMXCIV"));
}

}