//
// copyright Matthias Schoerghuber (AIT)
//

#ifndef SLAMANTIC_VECTOR_HPP
#define SLAMANTIC_VECTOR_HPP

#include <vector>

namespace slamantic
{
  namespace utils
  {
/**
 * erases all elements in master for which predicate pred = true, and additionally erases element in slave with same index
 * master and slave must have euqal size
 * @tparam T1
 * @tparam T2
 * @tparam Pred
 * @param master vector where predicate is evaluated
 * @param slave vector of equal size to master with index as correspondence
 * @param pred unary predicate which returns ​true if the element should be removed.
 */
    template<class T1, class T2, class UnaryPredicate>
    void multi_vector_erase_if(std::vector<T1>& master, std::vector<T2>& slave, UnaryPredicate pred)
    {
      assert(master.size() == slave.size());
      if(master.size() != slave.size())
      {
        return;
      }

      auto it_a  = master.begin();
      auto it_b  = slave.begin();
      auto eit_a = master.end();
      auto eit_b = slave.end();

      // iterate over vector, like std::remove_if swap erased element to the end of the vector
      while(it_a < eit_a)
      {
        if(pred(*it_a))
        {
          // swap erased element to end
          eit_a--;
          std::iter_swap(it_a, eit_a);
          eit_b--;
          std::iter_swap(it_b, eit_b);
        }
        else
        {
          it_a++;
          it_b++;
        }
      }
      // erase elements
      master.erase(eit_a, master.end());
      slave.erase(eit_b, slave.end());
    }
  }
}

#endif //SLAMANTIC_VECTOR_HPP
