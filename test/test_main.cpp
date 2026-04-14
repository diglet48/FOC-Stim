#include <unity.h>


extern void tests_4p();


void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}


int main(int argc, char **argv) {
    UNITY_BEGIN();
    tests_4p();
    UNITY_END();

    return 0;
}