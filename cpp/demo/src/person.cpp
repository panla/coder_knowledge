#include <iostream>
#include <cstring>

#include "person.h"

extern "C"
{
    // 暴露出去，给 Python 调用
    // 这里的示例使用了 C++ 的类，所以在这里用函数进行生成对象
    void person_say()
    {
        Person person;
        person.say();
    }

    void person_run()
    {
        Person person;
        person.run();
    }

    void person_eat()
    {
        Person person;
        person.eat();
    }

    void person_all()
    {
        Person person;

        person.say();
        person.run();
        person.eat();
    }
}

void Person::say()
{
    std::cout << "Person say `how are you`" << std::endl;
}

void Person::run()
{
    std::string msg = "Person run `1000m`";
    std::cout << msg << std::endl;
}

void Person::eat()
{
    std::cout << "Person eat `a chicken`" << std::endl;
}

void Person::all()
{
    say();

    run();

    eat();
}
