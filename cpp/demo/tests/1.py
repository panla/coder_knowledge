import ctypes

hello_so = ctypes.cdll.LoadLibrary('../lib/libhello.so')
hello_so.func()

person_so = ctypes.cdll.LoadLibrary('../lib/libperson.so')
person_so.person_all()
