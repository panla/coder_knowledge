#include <hello.h>
#include <person.h>

int main(int argc, char **argv)
{
    Hello obt;

    obt.output();

    Person person;
    person.all();

    return 0;
}