#include "hello.h"

extern "C"
{
    // 暴露出去，给 Python 调用
    // 这里的示例使用了 C++ 的类，所以在这里用函数进行生成对象
    void func()
    {
        Hello obt;
        obt.output();
    }
}

void Hello::output()
{
    std::cout << "hello world" << std::endl;
}
