#include <emscripten.h>  // note we added the emscripten header
#include <stdio.h>

extern "C" {
int EMSCRIPTEN_KEEPALIVE fib(int n) {
    if (n == 0 || n == 1)
        return 1;
    else
        return fib(n - 1) + fib(n - 2);
}

int EMSCRIPTEN_KEEPALIVE test(int n) {
    int i, j;
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
        }
    }
    return 1;
}

int EMSCRIPTEN_KEEPALIVE add(int n) {
    int i;
    int sum = 0;
    for (i = 1; i <= n; i++) {
        sum += i;
    }
    return sum;
}

int main() {
    printf("Hello world!\n");

    return 0;
}
}