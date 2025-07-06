#include <cmath>
#include <vector>
using namespace std;

class Util {
public:

    Util() {}

    ~Util(){}


    //  S_{t}/S_{t-1} - 1
    double* get_returns(double* prices, int size) {
        double* results = new double[size - 1];
        for (int i = 0; i < size - 1; i++){
            results[i] = prices[i + 1] / prices[i] - 1;
        }
        return results;
    }

    std::vector<float> get_returns(std::vector<float> prices, int size) {
        std::vector<float> results;
        for (int i = 0; i < size - 1; i++){
            results.push_back(prices[i + 1] / prices[i] - 1);
        }
        return results;
    }

    float get_nth_smallest(std::vector<float> values, int size, int target) {
        // find the nth smallest value, but other values may be unsorted.
        _revised_quicksort(values, 0, size - 1, target);
        return values[target];
    }

    double get_nth_smallest(double* values, int size, int target) {
        // find the nth smallest value, but other values may be unsorted.
        _revised_quicksort(values, 0, size - 1, target);
        return values[target];
    }

    double mean(double* values, int size) {
        double sum = 0;
        for (int i = 0; i < size; i++) {
            sum += values[i];
        }
        return sum / size;
    }

    double variance(double* values, int size) {
        // this can be with SIMD, vector operations
        double m = mean(values, size);
        double diff = 0;
        for(int i = 0; i < size; i ++) {
            diff += (values[i] - m) * (values[i] - m);
        }
        return diff / size;
    }

    double std_dev(double* values, int size) {
        return std::sqrt(variance(values, size));
    }

    /*
        @return list of double for [mean, variance, standard_deviation]
    */
    double* get_statistics(double* values, int size) {
        double* results = new double[3];
        results[0] = mean(values, size);
        double diff = 0;
        for(int i = 0; i < size; i ++) {
            diff += (values[i] - results[0]) * (values[i] - results[0]);
        }
        results[1] = diff / size;
        results[2] = std::sqrt(results[1]);
        return results;
    }


private:

    void _swap(std::vector<float> array, int i, int j){
        if (i == j) return;
        float tmp = array[i];
        array[i] = array[j];
        array[j] = tmp;
    }


    void _swap(double* array, int i, int j){
        if (i == j) return;
        double tmp = array[i];
        array[i] = array[j];
        array[j] = tmp;
    }

    
    int _partition(std::vector<float> values, int low, int high) {
        // return an index of the pivot, where everything on the
        //  left of the pivot is smaller than the right ones.
        float pivot = values[high];
        int i = low;
        for (int j = low; j < high; j ++) {
            if (values[j] < pivot) {
                _swap(values, i, j);
                i ++;
            }
        }
        _swap(values, i, high);
        return i;
    }


    int _partition(double* values, int low, int high) {
        // return an index of the pivot, where everything on the
        //  left of the pivot is smaller than the right ones.
        double pivot = values[high];
        int i = low;
        for (int j = low; j < high; j ++) {
            if (values[j] < pivot) {
                _swap(values, i, j);
                i ++;
            }
        }
        _swap(values, i, high);
        return i;
    }


    int _revised_quicksort(std::vector<float> values, int low, int high, int target) {
    // high is the size of the array initially.
        
        if (low >= high) return low;  // base case for recursive call
        
        int p = _partition(values, low, high);  
        if (p == target) return target;   // found the target.

        // if the target value is to the left, no need to sort the right side
        if (p > target) return _revised_quicksort(values, low, p -1, target);
        
        // Similarly, if the target is larger, sort the right side.
        return _revised_quicksort(values, p + 1, high, target);   
    }


    int _revised_quicksort(double* values, int low, int high, int target) {
        // high is the size of the array initially.
        
        if (low >= high) return low;  // base case for recursive call
        
        int p = _partition(values, low, high);  
        if (p == target) return target;   // found the target.

        // if the target value is to the left, no need to sort the right side
        if (p > target) return _revised_quicksort(values, low, p -1, target);
        
        // Similarly, if the target is larger, sort the right side.
        return _revised_quicksort(values, p + 1, high, target);   
    }

    
};