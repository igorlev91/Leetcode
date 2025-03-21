impl Solution {
    pub fn reverse_words(s: String) -> String {
        let mut words = Vec::new();
        let mut s = s.chars();
        let mut word = String::new();
        while let Some(c) = s.next() {
            if c != ' ' {
                word.push(c);
                continue;
            }
            if word.len() != 0 {
                words.push(word);
                word = String::new();
            }
        }
        if word.len() != 0 {
            words.push(word);
        }
        words.reverse();
        words.join(" ")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    pub fn test() {
        assert_eq!(Solution::reverse_words("the sky is blue".into()), "blue is sky the".to_string());
        assert_eq!(Solution::reverse_words("  hello world  ".into()), "world hello".to_string());
        assert_eq!(Solution::reverse_words("a good   example".into()), "example good a".to_string());
    }
}