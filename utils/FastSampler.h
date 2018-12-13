#ifndef UTILS_FAST_SAMPLER_H
#define UTILS_FAST_SAMPLER_H

/** @ingroup Utils
 * @brief Samples from a weighted discrete set in O(log(n)) time after an
 * O(n) initialization.
 */
struct FastSampler
{
  FastSampler(const std::vector<double>& probabilities) {
    sump.resize(probabilities.size());
    double sum=0;
    for(size_t i=0;i<probabilities.size();i++) {
      sum += probabilities[i];
      sump[i] = sum;
    }
  }

  template <class RNG>
  int Sample(RNG& rng) const {
    double val = rng.randDouble(sump.back());
    return Pick(val);
  }

  inline int Pick(double val) const {
    std::vector<double>::const_iterator i=std::upper_bound(sump.begin(),sump.end(),val);
    return (i-sump.begin())-1;
  }

  //partial sums over probabilities
  std::vector<double> sump;
};

#endif
