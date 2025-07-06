#include <iostream>
#include <memory>
#include <algorithm>
#include <cmath>
#include <random>
#include <map>
#include <vector>
#include <immintrin.h>
#include "util.cpp"



// using namespace std::chrono_literals;
std::map<double, double> zscore_table;


class RiskCalculation{


public:
    RiskCalculation() {}
    ~RiskCalculation() {}
    // CALCULATION SUB-MODULE: Value at Risk

    
    /*
     Get the lower alpha-quantile returns
     @return x s.t. |{y| y < x, y in returns}| = alpha * |returns|
    */
    double var_historical(double* prices, double alpha=0.05) {
        auto returns = Util().get_returns(prices, sizeof(prices));
        int target_index = alpha * sizeof(returns);
        return Util().get_nth_smallest(returns, sizeof(returns), target_index);
    }
    
    double var_historical(std::vector<float> prices, double alpha=0.05) {
        auto returns = Util().get_returns(prices, sizeof(prices));
        int target_index = alpha * sizeof(returns);
        return Util().get_nth_smallest(returns, sizeof(returns), target_index);
    }

    /*
     Calculate VaR using Variance-Covariance Method
     @return x: z * sqrt(variance(returns) * time_horizon), 
      where Pr(Z = z) = alpha, Z being standard normal distribution
     */
    double var_variance(double* prices, double alpha=0.05) {
        auto returns = Util().get_returns(prices, sizeof(prices));
        int n = sizeof(returns);
        double std_dev = Util().variance(returns, n);
        double z_score = zscore_table[alpha];

        // Variance-Covariance formula
        return z_score * std::sqrt(std_dev * n);
    }


    /*
        Calculate VaR using Monte Carlo Simulation.
        Assume stock returns follow log-normal distribution.
    */
    double var_monte_carlo(double* prices, double alpha=0.05, int num_simulations=1000) {
        auto returns = Util().get_returns(prices, sizeof(prices));
        double* statistics = Util().get_statistics(returns, sizeof(returns));
        
        double* simulated_returns = new double[num_simulations];
        // define log-normal, gpt code
        std::random_device rd;
        std::mt19937 generator(rd());
        std::lognormal_distribution<double> lognormal_dist(statistics[0], statistics[2]);

        // simulations
        for (int i = 0; i < num_simulations; i ++) {
            simulated_returns[i] = lognormal_dist(generator);  
        }
        
        int index = alpha * num_simulations;
        return Util().get_nth_smallest(simulated_returns, sizeof(simulated_returns), index);
    }


    
    // CALCULATION SUB-MODULE: Liquidity

    double liquidity_volume(double* orderbook) {

    }



    // CALCULATION SUB-MODULE: Capital risks
    
    /*
    Pearson sample correlation:
    (n* sum{x*y} -sum(x)*sum(y) ) /
    sqrt(n*sum(x^2) - sum(x)^2) * sqrt(n*sum(y^2) - sum(y)^2)
    */
    double correlation(double* x, int size_x, double* y, int size_y) {
        double sum_xy = 0.0, sum_x = 0.0, sum_y = 0.0, sum_x_sq = 0.0, sum_y_sq = 0.0;
        int n = max(size_x, size_y);
        if (n == 0) return 0;
        for (int i = 0; i < n; i ++) {
            sum_x += x[i];
            sum_x_sq += x[i] * x[i];
        }
        
        for (int i = 0; i < n; i ++) {
            sum_y += y[i];
            sum_y_sq += y[i] * y[i];
        }
        
        for (int i = 0; i < n; i ++) {
            sum_xy += x[i] * y[i];
        }

        return (n * sum_xy - sum_x * sum_y) / 
        sqrt(n * sum_x_sq - sum_x * sum_x) / sqrt(n * sum_y_sq - sum_y * sum_y);

    }


    /*
    Calculate the covariance one by one using formula
    $ cov_i_j = \sum_i (x_i - x_bar)(y_i - y_bar) $
    Lower matrix is calculated.
    */
    std::vector<std::vector<float>> covariance_matrix_basic(std::vector<std::vector<int>> vecs) {        
        int n = vecs.size();
        int num_obser = vecs[0].size();

        float scaler = 1 / (num_obser - 1);
        std::vector<float> means;
        std::vector<std::vector<float>> covs;
        // calculate mean
        for (int i = 0; i < n; i++) {
            float tmp = 0.0;
            for (int j = 0; j < num_obser; j ++) {
                tmp += vecs[i][j];
            }
            means.push_back(tmp / vecs[i].size());
        }

        // calculate covariance$
        for (int x = 0; x < n; x++) {
            covs.push_back(std::vector<float>());
            for (int y = 0; y <= x; y++) {
                float tmp = 0.0;
                for (int i = 0; i < num_obser; i ++) {
                    tmp += (vecs[x][i] - means[x]) * (vecs[y][i] - means[y]);
                }
                covs[x].push_back(tmp * scaler);
            }
        }
        return covs;
    }

    std::vector<std::vector<float>> covariance_matrix_expanded(std::vector<std::vector<int>> vecs) {
        int n = vecs.size();
        int num_obser = vecs[0].size();
        
        float scaler = 1 / (num_obser - 1);
        
        std::vector<float> sums;
        std::vector<float> means;
        std::vector<std::vector<float>> covs;
        // calculate $\sum{x_i}$
        for (int i = 0; i < n; i++) {
            float tmp = 0.0;
            for (int j = 0; j < vecs[i].size(); j++) {
                tmp += vecs[i][j];
            }
            sums.push_back(tmp);
        }

        for (int x = 0; x < n; x++) {
            float tmp = 0.0;
            covs.push_back(std::vector<float>());
            for (int y = 0; y <= x; y++) {
                // calculate $\sum{x_i * y_i}$
                for (int i = 0; i < n; i ++) {
                    tmp += vecs[x][i] * vecs[y][i];
                }
                float cov = tmp - sums[x] * means[y] - sums[y] * means[x] + n * means[x] * means[y];
                covs[x].push_back(cov * scaler);
            }
        
        }

        return covs;
    }

    /*
    Using SIMD to calculate covariance.
    GPT writes better simd.
    */
    // std::vector<std::vector<float>> covariance_matrix_simd(std::initializer_list<std::vector<int>> vecs) {
    //     int n = vec.size();
    //     int num_obser = vecs[i].size();
    //     assert(num_obser > 1);
    //     float scaler = 1 / (num_obser - 1);
    //     std::vector<float> means;
    //     std::vector<std::vector<float>> covs;
        
    //     for (int x = 0; x < n; x ++) {
    //         float tmp = 0.0;
    //         for (int i = 0; i < num_obser; i ++) {
    //             tmp += vecs[x][i];
    //         }
    //         means.push_back(tmp);
    //     }

    //     for (int x = 0; x < n; x ++) {
    //         covs.push_back(std::vector<float>);
    //         for (int y = 0; y <= x; y++) {
    //             // calculate covariance of x and y
    //             __m256 sum_prod = _mm256_setzero_ps();  // To store the sum of (x_i - mean_x) * (y_i - mean_y)
    //             const size_t simd_size = 8;  // Number of floats AVX can process at once
                
    //             // Loop through data using AVX (process 8 elements at a time)
    //             for (size_t i = 0; i < n; i += simd_size) {
    //                 // Load 8 floats from x and y into SIMD registers
    //                 __m256 x_vals = _mm256_loadu_ps(&vecs[x][i]);
    //                 __m256 y_vals = _mm256_loadu_ps(&vecs[y][i]);
                    
    //                 // Subtract means from x and y
    //                 __m256 x_mean = _mm256_set1_ps(means[x]);
    //                 __m256 y_mean = _mm256_set1_ps(means[y]);
                    
    //                 __m256 x_diff = _mm256_sub_ps(x_vals, means[x]);
    //                 __m256 y_diff = _mm256_sub_ps(y_vals, means[y]);
                    
    //                 // Multiply differences element-wise
    //                 __m256 prod = _mm256_mul_ps(x_diff, y_diff);
                    
    //                 // Accumulate results
    //                 sum_prod = _mm256_add_ps(sum_prod, prod);
    //             }

    //             // Sum the elements of the SIMD register
    //             float result[simd_size];
    //             _mm256_storeu_ps(result, sum_prod);
    //             float covariance_sum = 0.0f;
    //             for (size_t i = 0; i < simd_size; ++i) {
    //                 covariance_sum += result[i];
    //             }
    //             covs[x].push_back(covariance_sum * scaler);
    //         }
    //     }
    //     return covs;
    // }   
};



