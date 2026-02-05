#pragma once
#include <cmath>
#include <iostream>
#include <random>
#include <chrono>
#include <vector>
#include <type_traits>
#include "mujoco/mjtnum.h"

namespace std_noise {

class Noise {
public:
  Noise() : gen(std::random_device()()) {
    seed = gen();
    // std::cout << "Using random seed: " << seed << std::endl;
  };
  
  virtual ~Noise() = default;
  
  void set_seed(unsigned int seed) {
    this->seed = seed;
    gen.seed(seed);
  }

  //produce_noise为一般噪声制造
  
  virtual void produce_noise(mjtNum &data) {
  }

  template <typename T>
  void produce_noise(std::vector<T> &data) {
    for (auto& value : data) {
      produce_noise(value);
    }
  }

  template <typename T, size_t N>
  void produce_noise(T (&data)[N]) {
    for (size_t i = 0; i < N; i++) {
      produce_noise(data[i]);
    }
  }

  template <typename T>
  void produce_noise(T *data, int len) {
    for (int i = 0; i < len; i++) {
      produce_noise(data[i]);
    }
  }

  // produce_united_noise是根据整个数组状态或者联合其他内容产生噪声，用于拓展噪声 
 virtual void produce_united_noise(){};

protected:
  std::mt19937 gen;
  unsigned int seed;
};

template <typename Distribution>
class DistributionNoise : public Noise {
public:
  template <typename... Args>
  DistributionNoise(Args&&... args) : dist(std::forward<Args>(args)...) {
    gen.seed(std::random_device()());
  }

  template <typename... Args>
  void set_distribution(Args&&... args) {
    dist = Distribution(std::forward<Args>(args)...);
  }


  void produce_noise(mjtNum &data) override {
    data += static_cast<mjtNum>(dist(gen));
  }

protected:
  Distribution dist;
};

// 高斯噪声
class GaussianNoise : public DistributionNoise<std::normal_distribution<mjtNum>> {
public:
  GaussianNoise(mjtNum mean = 0.0, mjtNum stddev = 1.0, unsigned int seed = 0) 
      : DistributionNoise(mean, stddev) {
    if (seed != 0) {
      set_seed(seed);
    }
  }

  mjtNum get_mean() const { return dist.mean(); }
  mjtNum get_stddev() const { return dist.stddev(); }
  void set_params(mjtNum mean, mjtNum stddev) {
    dist = std::normal_distribution<mjtNum>(mean, stddev);
  }
};

// 均匀分布噪声
class UniformNoise : public DistributionNoise<std::uniform_real_distribution<mjtNum>> {
public:
  UniformNoise(mjtNum low = 0.0, mjtNum high = 1.0, unsigned int seed = 0) 
      : DistributionNoise(low, high) {
    if (seed != 0) {
      set_seed(seed);
    }
  }

  mjtNum get_low() const { return dist.a(); }
  mjtNum get_high() const { return dist.b(); }
  void set_params(mjtNum low, mjtNum high) {
    dist = std::uniform_real_distribution<mjtNum>(low, high);
  }
};

// 泊松噪声
class PoissonNoise : public DistributionNoise<std::poisson_distribution<int>> {
public:
  PoissonNoise(mjtNum mean = 1.0, unsigned int seed = 0) 
      : DistributionNoise(mean) {
    if (seed != 0) {
      set_seed(seed);
    }
  }

  mjtNum get_mean() const { return dist.mean(); }
  void set_mean(mjtNum mean) {
    dist = std::poisson_distribution<int>(mean);
  }
};

} // namespace std_noise