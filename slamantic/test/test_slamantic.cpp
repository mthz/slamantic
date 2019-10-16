//
// copyright Matthias Schoerghuber (AIT)
//


#include <gtest/gtest.h>

#include <easylogging++.h>


using namespace std;

INITIALIZE_EASYLOGGINGPP

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
