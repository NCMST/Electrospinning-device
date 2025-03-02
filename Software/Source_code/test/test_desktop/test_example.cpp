#include <unity.h>

// Funcția pe care vrei să o testezi
int add(int a, int b) {
    return a + b;
}

// Testul tău
void test_add() {
    TEST_ASSERT_EQUAL(5, add(2, 3));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_add);
    UNITY_END();
    return 0;
}