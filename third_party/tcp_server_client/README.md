# TCP server client
A thin and simple C++ TCP client server

This code was written to run on linux machines, and to compile with C++11 and older versions of C++ (this is why I chose to use pthread instead of std::thread).

Both server and client are very easy and simple to use. You can find code examples of both server and client. 

The code is documented, so I hope you find it easy to change it to suit your needs if needed to.

The server class supports multiple clients.

### Compilation
1. Add pthread library flag
2. To enable client example or server example, refer to line 9 in the cmake list. The server example is enabled by default.
