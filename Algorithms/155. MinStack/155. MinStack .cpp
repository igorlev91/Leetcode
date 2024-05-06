#include <iostream>
#include <stack>
using namespace std;


class MinStack
{
	struct Item{
		int value;
		int min;
	};
	stack<Item, list<Item>> st;
public:
	void push(int x){
		if(st.empty()){
			st.push(Item{x,x});
		} else{
			st.push(Item{x, min(x, st.top().min)});
		}
	}
	void pop(){
		if(st.empty()){
			throw logic_error("pop when empty stack");
		}
		return st.pop();
	}

	int top() const{
		if(st.empty()){
			throw logic_error("top when empty stack");
		}
		return st.top().value;
	}
	
	int getMin() const{
		if(st.empty()){
			throw logic_error("getMin when stack empty");
		}
		return st.top().min;
	}

 };


//solution 2 runtime 20 ms
template<typename T, size_t size = 16>
 class BigList{

 	struct Leaf{
 		T a[SIZE];

 		Leaf* next;

 		int filled;
 	};

 	Leaf* top;

public:
 	typedef T value_type;
 	typedef T& reference;
 	typedef const T& const_reference;
 	typedef size_t size_type;

 	BigList()
 		: top(nullpr)
 		{}

 	void push_back(const T& value){
 		if(top == nullptr || top->filled == SIZE){
 			Leaf* newTop = new Leaf();
 			newTop->next = top;
 		}

 	void pop_back(){
 		assert(top != nullptr);
 		assert(top->filled != 0);
 		top->filled--;
 		if(top->filled == 0){
 			Leaf* oldTop = top;
 			top = oldTop->next;
 			delete oldTop;
 		}
 	}

 	const T& back() const{
 		assert(top != nullptr);
 		assert(top->filled != 0);
 		return top->a[top->filled - 1];
 	}

 	T& back() {
 		assert(top != nullptr);
 		assert(top->filled != 0);
 		return top->a[top->filled - 1];
 	}

 	bool empty() const{
 		return top == nullptr;
 	}
 };

class MinStack
{
	struct Item{
		int value;
		int min;
	};
	stack<Item, BigList<Item>> st;
public:
	void push(int x){
		if(st.empty()){
			st.push(Item{x,x});
		} else{
			st.push(Item{x, min(x, st.top().min)});
		}
	}
	void pop(){
		if(st.empty()){
			throw logic_error("pop when empty stack");
		}
		return st.pop();
	}

	int top() const{
		if(st.empty()){
			throw logic_error("top when empty stack");
		}
		return st.top().value;
	}
	
	int getMin() const{
		if(st.empty()){
			throw logic_error("getMin when stack empty");
		}
		return st.top().min;
	}

 };


/*
 * Soltuion 3: use two stacks, one to keep track of all the numbers and another one to keep track of only
 * the minimum values we've seen so far. This way, we maintain the ordering and the number of minimum values.
 */

class MinStack
{
    public:
        stack<int> stk;

        stack<int> previousMinimumValues;

        void push(int x)
        {
            if(previousMinimumValues.empty() || x <= previousMinimumValues.top())
            {
                previousMinimumValues.push(x);
            }

            stk.push(x);
        }

        void pop()
        {
            int value=stk.top();

            if(value==getMin())
            {
                previousMinimumValues.pop();
            }

            stk.pop();
        }

        int top()
        {
            return stk.top();
        }

        int getMin()
        {
            return previousMinimumValues.top();
        }
};