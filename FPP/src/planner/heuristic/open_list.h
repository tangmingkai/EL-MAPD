#pragma once
#include <iostream>
#include <random>

#include "planner/heuristic/heuristic_state.h"

namespace planner {
namespace heuristic {

class OpenList {
 public:
  inline bool IsBetter(const State *s1, const State *s2) {
    return s1->g < s2->g;
  }

  explicit OpenList(int _max_size) : max_size_(_max_size), size_(0) {
    heap_ = new State *[max_size_];
  }

  ~OpenList() { delete[] heap_; }

  inline void Swap(State *s, State *p) {
    int i = s->heap_index;
    int j = p->heap_index;
    heap_[i] = p;
    heap_[j] = s;
    s->heap_index = j;
    p->heap_index = i;
  }

  inline void MoveUp(State *s) {
    int heap_index = s->heap_index;
    while (heap_index > 0) {
      int parent_heap_index = (heap_index - 1) / 2;
      if (IsBetter(heap_[parent_heap_index], heap_[heap_index])) {
        break;
      }
      Swap(s, heap_[parent_heap_index]);
      heap_index = parent_heap_index;
    }
  }

  inline void MoveDown(State *s) {
    int heap_index = s->heap_index;
    while (heap_index * 2 + 1 < size_) {
      int child_heap_index = heap_index * 2 + 1;
      if (child_heap_index + 1 < size_ &&
          IsBetter(heap_[child_heap_index + 1], heap_[child_heap_index])) {
        ++child_heap_index;
      }
      if (IsBetter(heap_[heap_index], heap_[child_heap_index])) {
        break;
      }
      Swap(s, heap_[child_heap_index]);
      heap_index = child_heap_index;
    }
  }

  void Push(State *s) {
    heap_[size_] = s;
    s->heap_index = size_;
    ++size_;
    MoveUp(s);
  }

  State *Pop() {
    State *ret = heap_[0];
    --size_;
    heap_[0] = heap_[size_];
    heap_[0]->heap_index = 0;
    MoveDown(heap_[0]);
    return ret;
  }

  State *Top() { return heap_[0]; }

  void Increase(State *s) { MoveUp(s); }

  inline bool Empty() { return size_ == 0; }

  inline bool Full() { return size_ == max_size_; }

  void Clear() { size_ = 0; }

 private:
  int max_size_;
  int size_;
  State **heap_;
};

}  // namespace heuristic
}  // namespace planner
