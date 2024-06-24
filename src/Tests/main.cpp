
#include <gtest/gtest.h>

#include "test_vecs.cc"
#include "test_geometry.cc"
#include "test_cdt.cc"

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}