#include "dog.h"

extern "C"
{
    // 暴露出去，给 Python 调用
    // 这里的示例使用了 C++ 的类，所以在这里用函数进行生成对象
    u_int8_t func(uint8_t param)
    {
        Dog obt;
        return obt.age(param);
    }
}

u_int8_t Dog::age(uint8_t param)
{
    std::cout << "this dog is " << param << std::endl;

    return param;
}
