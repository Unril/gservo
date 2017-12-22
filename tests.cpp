#include "catch.hpp"

#include <vector>

TEST_CASE( "Parser" )
{
std::vector<int> v(5);

REQUIRE(v.size() == 5);
REQUIRE(v.capacity() >= 5);
}
