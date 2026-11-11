#include <cassert>
#include <cstdio>
#include "../src/color_vote.h"

int main() {
  std::deque<int8_t> h;
  const std::size_t w = 6;

  // Stable red buoy, one green misread does not flip it.
  for (int i = 0; i < 5; ++i) assert(vote_color(h, 0, w) == 0);
  assert(vote_color(h, 1, w) == 0);

  // Buoy changes to green: flips when green reaches half of the window.
  assert(vote_color(h, 1, w) == 0);  // 4 red, 2 green
  assert(vote_color(h, 1, w) == 1);  // 3 red, 3 green, tie -> latest
  assert(vote_color(h, 1, w) == 1);
  assert(h.size() == w);

  std::puts("test_color_vote OK");
  return 0;
}
