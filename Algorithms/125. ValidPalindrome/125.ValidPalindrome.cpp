#include <iostream>
#include <string>
#include <locale>
using namespace std;

/*
 * Approach: Loop over the string using two pointers (integers indexers), lwo and high.
 * If the character pointed to by low is not alphanumeric, increment low.
 * Else if the character pointed to by high is not alphanumeric, decrement high.
 * Else, if the two characters are not the same, return false. Else, continue looping.
 *
 * Time complexity: O(n) [where n is the size of the input string]
 * Space complexity: O(1)
 */


class Solution 
{
public:
    bool isPalindrome(string s) 
    {
    	int low=0;
    	int high=int(s.size()-1);

    	while(low < high)
    	{
        	char left=tolower(s[low]);

        	char right=tolower(s[high]);

        	if(!isalnum(left))
        	{
            	low++;
        	}
        	else if(!isalnum(right))
        	{
            	high--;
        	}
        	else
        	{
            	if(left!=right)
            	{
                	return false;
            	}
            	else
            	{
                	low++;
                	high--;
            	}
        	}
        }
        return true;
    }

};


class Solution 
{
public:
	bool isAlphaNum(char c)
	{
		return ('A' <= c && c <= 'Z' || ('a' <= c && c <= 'z') ||
		'0' <= c && c <= '9');
	}


	char toLower(char c)
	{
		if('A' <= c && c <= 'Z')
		{
			return char(c - 'A' + 'a')
		}
		else
		{
			return c;
		}
	}

    bool isPalindrome(const string& s) 
    {
        int i = 0;
        int j = (int)s.size() - 1;
        while(true)
        {
        	while(i < j && !isAlphaNum(s[i]))
        	{
        		i++;
        	}
        	while(i < j && !isAlphaNum(s[j]))
        	{
        		j--;
        	}

        	if(i < j)
        	{
        		if(toLower(s[i]) != toLower(s[j]))
        		{
        			return false;
        		}
        		i++;
        		j--;
        	}
        	else
        	{
        		return true;
        	}
        }
    }
};