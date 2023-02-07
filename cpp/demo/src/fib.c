#include <stdio.h>

unsigned long fib(unsigned long n)
{
    unsigned long a = 1;
    unsigned long b = 1;

    unsigned long result;

    if (n == 1 || n == 2)
    {
        return 1;
    }
    else
    {
        return fib(n - 1) + fib(n - 2);
    }
}
