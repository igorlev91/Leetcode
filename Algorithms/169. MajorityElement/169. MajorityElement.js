
// hashmap
var majorityElement = function(nums) {
    if(nums.length == 1){
    	return nums[0];
    }
    
    let half = nums.length / 2;

    let elementCount = new Map();

    for(let num of nums){
    	if(!elementCount.has(num)){
    		elementCount.set(num,1);
    	}
    	else{
    		elementCount.set(num, elementCount.get(num) + 1);
    	}

    	if(elementCount.get(num) > half){
    		return num;
    	}
    }

    return -1;
};

// moore vote
var majorityElement = function (nums) 
{
    let target = null, count = 0;
    for (let i = 0; i < nums.length; i++) {
        if (target == null || count === 0) {
            target = nums[i];
            count = 1;
            continue;
        } else if (target == nums[i]) {
            count++;
        } else if (target != nums[i]) {
            count--;
        }
    }
    return target;
}
