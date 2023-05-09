"""
20. 有效的括号
来源：力扣（LeetCode）
链接：https://leetcode-cn.com/problems/valid-parentheses

给定一个只包括 '('，')'，'{'，'}'，'['，']' 的字符串 s ，判断字符串是否有效。

有效字符串需满足：

左括号必须用相同类型的右括号闭合。
左括号必须以正确的顺序闭合。

栈
"""


class Solution:
    def isValid(self, s: str) -> bool:
        lis = []
        dic = {
            ')': '(', ']': '[', '}': '{'
        }

        for i in s:
            if lis != [] and lis[-1] == dic.get(i):
                lis.pop()
            else:
                lis.append(i)
        if lis == []:
            return True
        else:
            return False

def create_btree():
    print

if __name__ == '__main__':
    for s in ["()", "()[]{}", "(]", "([)]", "{[]}"]:
        ret = Solution().isValid()
        print(ret)
