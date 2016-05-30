#include <boost/thread.hpp>
#include <boost/container/flat_map.hpp>

#include <iostream>
#include <cstdlib>
#include <vector>
#include <iostream>

#include <boost/atomic.hpp>

#include <xmmintrin.h>

static __inline__ unsigned long long rdtsc(void)
{
    unsigned hi, lo;
    __asm__ __volatile__ ("rdtsc" : "=a"(lo), "=d"(hi));
    return ( (unsigned long long)lo)|( ((unsigned long long)hi)<<32 );
}

struct spinlock0 {
  void lock()
  {
  }
  void unlock()
  {
  }
};

struct spinlock {
  boost::atomic_flag state_;

  spinlock()  {}

  void lock()
  {
    while (state_.test_and_set(boost::memory_order_acquire) != 0) {
      /* busy-wait */
//      asm("nop");
    }
  }
  void unlock()
  {
    state_.clear(boost::memory_order_release);
  }
};


struct spinlock1 {
  boost::atomic<size_t> state_;
  /*volatilei*/ size_t current_;

  spinlock1()
  : state_(0)
  , current_(0)  
  {}

  void lock()
  {
    size_t v = state_.fetch_add(1, boost::memory_order_relaxed);
    //size_t after_fetch = rdtsc();
    //size_t iters = 0;
    while (v != current_) {
      //std::cout << boost::this_thread::get_id() << std::dec <<  " waiting for: " << v << " current: " << current_ << std::endl;
      //_mm_pause();
      __asm volatile ("" ::: "memory");
     size_t seed = rdtsc() & 0x3f; //get random seed
     while (seed--) {
      __asm volatile ("" ::: "memory");
     }
 
//     asm volatile("": : :"memory");
     // ++iters;
    }
    //(void *)iters;
    //size_t after_loop = rdtsc();
    
    //std::cout << " for loop:  " << after_loop - after_fetch << " iters: " << iters <<  std::endl; 
  }
  void unlock()
  {
    ++current_;
  }
};

struct cpu
{
  size_t can_run_;
  cpu * next_;
};

struct queue
{
  boost::atomic<cpu*> cpu_;
  queue()
    :cpu_(0)
    {}
};


class spinlock2
{
private:
  queue queue_;
public:
  spinlock2()
  {}

  void lock(cpu& c)
  {
    c.can_run_ = 0;
    cpu * pred = queue_.cpu_.exchange(&c, boost::memory_order_seq_cst);
    if (pred != 0) {
      //atomic_thread_fence(boost::memory_order_seq_cst);
      pred->next_ = &c;

      //atomic_thread_fence(boost::memory_order_seq_cst);

    //std::cout <<  boost::this_thread::get_id() <<  " going to sleep"  << std::endl;
      while (c.can_run_ == 0) {
        asm volatile("": : :"memory");
        //std::cout <<  boost::this_thread::get_id() <<  " " << c.can_run_  << std::endl;
      } 
    //std::cout <<  boost::this_thread::get_id() <<  " woke up"  << std::endl;
    }
    c.can_run_ = 1;
  }

  void unlock(cpu& c)
  { 
    if (queue_.cpu_.load() == &c) {
      cpu * cc = &c;
      if (queue_.cpu_.compare_exchange_strong(cc, 0, boost::memory_order_relaxed)) {
      //  std::cout << "nothing to wake: " << boost::this_thread::get_id() << std::endl;
        return;
      }
     // std::cout << "hahahah" << std::endl;
    }
    
      //atomic_thread_fence(boost::memory_order_seq_cst);
    while (c.next_ == 0) {
      asm volatile("": : :"memory");
    }
      //atomic_thread_fence(boost::memory_order_seq_cst);
    c.next_->can_run_ = 0x123;
    //atomic_thread_fence(boost::memory_order_seq_cst);
    //std::cout << "waking: " << boost::this_thread::get_id() << std::endl;
  }
};

using boost::container::flat_map;


template <class Lock>
void counter_func1(Lock& mtx, size_t count, size_t & val)
{
  for (size_t i = 0; i<count; ++i) {
    unsigned long long before = rdtsc();
    mtx.lock();
    unsigned long long after  = rdtsc();
    //m[i] = i;
    //mtx.state_.test_and_set(boost::memory_order_acquire);
    ++val;
    std::cout << after - before << " " <<  boost::this_thread::get_id() << std::dec  << " "  << val << std::endl;
      //std::cout << boost::this_thread::get_id() << std::dec <<  " waiting for: " << v << " current: " << current_ << std::endl;
    //mtx.state_.clear(boost::memory_order_release);
    mtx.unlock();
  }
}

void counter_func2(spinlock2& mtx, size_t count, size_t & val)
{
  for (size_t i = 0; i<count; ++i) {
    cpu c;
    unsigned long long before = rdtsc();
    mtx.lock(c);
    unsigned long long after  = rdtsc();

    std::cout << after - before << " " <<  boost::this_thread::get_id() << std::dec  << " "  << val << std::endl;
    //std::cout <<  boost::this_thread::get_id() << std::dec  << " "  << val << std::endl;
    ++val;

    mtx.unlock(c);
  }
}
int main(int argc, char ** argv)
{
  if (argc != 2)
    return 1;
  char * end;

  size_t count = std::strtoul(argv[1], &end, 10);

  flat_map<int, int> map;
  boost::mutex mtx;
  spinlock lk;
  spinlock1 lk1;
  spinlock2 lk2;
//  spinlock2 lk2;
  size_t val=0;
//  boost::thread thr1(func2,  boost::ref(map), boost::ref(lk));
//  boost::thread thr2(func2,  boost::ref(map), boost::ref(lk));
  //
  //boost::thread thr1(func,  boost::ref(map), boost::ref(mtx), argc, boost::ref(val));
//  boost::thread thr2(func, boost::ref(map), boost::ref(mtx));


//  boost::thread thr1(counter_func1, boost::ref(mtx), count, boost::ref(val));
 // boost::thread thr2(counter_func1, boost::ref(mtx), count, boost::ref(val));
  size_t vvv = count / 4;

  //boost::thread thr1(counter_func2<boost::mutex>, boost::ref(mtx), vvv, boost::ref(val));
  //boost::thread thr2(counter_func2<boost::mutex>, boost::ref(mtx), vvv, boost::ref(val));
  //boost::thread thr3(counter_func2<boost::mutex>, boost::ref(mtx), vvv, boost::ref(val));
  //boost::thread thr4(counter_func2<boost::mutex>, boost::ref(mtx), vvv, boost::ref(val));
  //boost::thread thr5(counter_func2<boost::mutex>, boost::ref(mtx), vvv, boost::ref(val));
  //boost::thread thr6(counter_func2<boost::mutex>, boost::ref(mtx), vvv, boost::ref(val));
  //boost::thread thr7(counter_func2<boost::mutex>, boost::ref(mtx), vvv, boost::ref(val));
  //boost::thread thr8(counter_func2<boost::mutex>, boost::ref(mtx), vvv, boost::ref(val));

  //boost::thread thr1(counter_func2<spinlock1>, boost::ref(lk1), vvv, boost::ref(val));
  //boost::thread thr2(counter_func2<spinlock1>, boost::ref(lk1), vvv, boost::ref(val));
  //boost::thread thr3(counter_func2<spinlock1>, boost::ref(lk1), vvv, boost::ref(val));
  //boost::thread thr4(counter_func2<spinlock1>, boost::ref(lk1), vvv, boost::ref(val));
  //boost::thread thr5(counter_func2<spinlock1>, boost::ref(lk1), vvv, boost::ref(val));
  //boost::thread thr6(counter_func2<spinlock1>, boost::ref(lk1), vvv, boost::ref(val));
  //boost::thread thr7(counter_func2<spinlock1>, boost::ref(lk1), vvv, boost::ref(val));
  //boost::thread thr8(counter_func2<spinlock1>, boost::ref(lk1), vvv, boost::ref(val));


  boost::thread thr1(counter_func2, boost::ref(lk2), vvv, boost::ref(val));
  boost::thread thr2(counter_func2, boost::ref(lk2), vvv, boost::ref(val));
  boost::thread thr3(counter_func2, boost::ref(lk2), vvv, boost::ref(val));
  boost::thread thr4(counter_func2, boost::ref(lk2), vvv, boost::ref(val));
  boost::thread thr5(counter_func2, boost::ref(lk2), vvv, boost::ref(val));
  boost::thread thr6(counter_func2, boost::ref(lk2), vvv, boost::ref(val));
  boost::thread thr7(counter_func2, boost::ref(lk2), vvv, boost::ref(val));
  boost::thread thr8(counter_func2, boost::ref(lk2), vvv, boost::ref(val));

  thr1.join();
  thr2.join();
  thr3.join();
  thr4.join();
  thr5.join();
  thr6.join();
  thr7.join();
  thr8.join();

  std::cout << map.size() << " val: " << val << std::endl;

  return 0;
}
