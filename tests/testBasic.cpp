#include <gtest/gtest.h>
 
TEST (SquareRootTest, PositiveNos) { 
    EXPECT_EQ (18.0, 18.0);
    EXPECT_EQ (25.4, 25.4);
}
 
TEST (SquareRootTest, ZeroAndNegativeNos) { 
    ASSERT_EQ (0.0, 0.0);
    ASSERT_EQ (-1, -1.0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}