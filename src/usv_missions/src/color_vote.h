#ifndef COLOR_VOTE_H
#define COLOR_VOTE_H

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>

// Push a new color reading, keep the last `window` readings and return the
// majority color. Ties go to the most recent reading, so a buoy that changes
// color flips once the new color reaches half of the window.
inline int8_t vote_color(std::deque<int8_t> &history, int8_t color, std::size_t window) {
  history.push_back(color);
  while (history.size() > window) history.pop_front();

  std::map<int8_t, int> counts;
  for (int8_t c : history) counts[c]++;

  int8_t best = history.back();
  int best_count = 0;
  for (auto it = history.rbegin(); it != history.rend(); ++it) {
    if (counts[*it] > best_count) {
      best = *it;
      best_count = counts[*it];
    }
  }
  return best;
}

#endif  // COLOR_VOTE_H
