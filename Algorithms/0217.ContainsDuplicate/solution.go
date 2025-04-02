func containsDuplicate(nums []int) bool {
     seen := make(map[int]bool)
        for _, num := range nums {  
            if _, ok := seen[num]; ok { 
                return true
        }
        seen[num] = true
    }
    return false  
}

func main() {
    nums1 := []int{1, 2, 3, 1}
    fmt.Printf("Срез: %v, Contains Duplicate: %t\n", nums1, containsDuplicate(nums1)) 

    nums2 := []int{1, 2, 3, 4}
    fmt.Printf("Срез: %v, Contains Duplicate: %t\n", nums2, containsDuplicate(nums2))

    nums3 := []int{1, 1, 1, 3, 3, 4, 3, 2, 4, 2}
    fmt.Printf("Срез: %v, Contains Duplicate: %t\n", nums3, containsDuplicate(nums3))
}