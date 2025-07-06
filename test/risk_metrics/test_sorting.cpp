#include <iostream>
#include <cassert>
#include <cmath>
#include "../../src/risk_management/risk_metrics/src/util.cpp"



void assert_equal(double actual, double expected) {
    if(std::fabs(actual - expected) >= 1e-6) 
    throw std::runtime_error("Difference no smaller than 1e-6.");

}

void helper_test_array(double input[], int target, int size, double expected) {
    double actual = Util().get_nth_smallest(input, size, target);
    try {
        assert_equal(actual, expected); 
    }
    catch (const std::exception& e) {
        std::cout << "Inputs: ";
        for (int i = 0; i < size; i ++) {
            std::cout << input[i] << " ";
        }
        std::cout<< std::endl;
        
        std::cout << "Target: " << target << std::endl;
        std::cout << "Expected: " << expected << std::endl;
        std::cout << "Actual: " << actual << std::endl;
        assert(false && "Assertion of equality failed");
    }
}

void test_one_element() {
    int num_tests = 3;
    double inputs[] = {10, 0.1, -0.25};
    
    for (int i = 0; i < num_tests; i++) {
        double input[] =  {inputs[i]};
        helper_test_array(input, 0, 1, inputs[i]);
    }
    std::cout<< __func__ << ": "<< num_tests << " tests passed" << std::endl;
}


void test_short_array() {
    double inputs[][5] = {
        {1,2,3,4,5},
        {5,2,3,4,1},
        {5,2,4,3,1},
        {0.5, -0.5, 0.12, 0.4, -0.1},
        {0.5, -0.5, 0.12, 0.4, -0.1},
        {0.5, -0.5, 0.12, 0.4, -0.1},
        {0.5, -0.5, 0.12, 0.4, -0.1},
        {0.5, -0.5, 0.12, 0.4, -0.1},
        {0.5, -0.5, 0.12, 0.4, -0.0},
    };
    double targets[] = {
        0, 1, 1,

        0, 1, 2, 3, 4,

        1
    };
    double expected[] = {
        1,
        2,
        2,

        -0.5,
        -0.1,
        0.12,
        0.4,
        0.5,

        0.0
    };
    int num_tests = size(inputs);
    for (int i = 0; i < num_tests; i ++) {
        helper_test_array(inputs[i], targets[i], 5, expected[i]);
    }
    std::cout<< __func__ << ": "<< num_tests << " tests passed" << std::endl;
}


void test_repeated_element() {
    double inputs[][7] = {
        {-0.5, -0.5, 0.12, 0.12, -0.0, 0.6, 2},
        {-0.5, -0.5, 0.12, 0.12, -0.0, 0.6, 2},

        {-0.5, 2, 0.12, 0.12, -0.0, 0.6, -0.5},
        {-0.5, 2, 0.12, 0.12, -0.0, 0.6, -0.5},

        {-0.5, 2, 0.12, 0.0, -0.0, 0.6, -0.5},
        {-0.5, 2, 0.12, 0.0, -0.0, 0.6, -0.5},
        
        {2, 2, 0.12, -0.5, -0.0, 0.6, -0.5},
        {2, 2, 0.12, -0.5, -0.0, 0.6, -0.5},

        {2, 2, 0.15, 0.15, -0.0, 0.6, -0.5},
        {2, 2, 0.15, 0.15, -0.0, 0.6, -0.5},
    };
    double targets[] = {0, 3, 2, 6, 3, 4, 4, 6};  // zero indexing 
    double expected[] = {-0.5, 0.12, 0.0, 2, 0, 0.12, 0.6, 2};
    int num_tests = size(inputs);
    int i = 0;
    for (; i < num_tests; i ++) {
        helper_test_array(inputs[i], targets[i], 7, expected[i]);
    }
    std::cout<< __func__ << ": "<< i + 1 << " tests passed" << std::endl;
}

void test_reversed_array() {
    double inputs[][7] = {
        {5, 3, 2, 1, 0, -1, -2},
        {5, 3, 2, 1, 0, -1, -2},
        {5, 3, 2, 1, 0, -1, -2},
        {5, 3, 2, 1, 0, -1, -2},

        {0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0},
        {0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0},
        {0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0},
        {0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0}
    };
    double targets[] = {0, 2, 5, 6, 0, 1, 4, 6};  // zero indexing 
    double expected[] = {-2, 0, 3, 5, 0, 0.4, 0.7, 0.9};
    int num_tests = size(inputs);
    int i = 0;
    for (; i < num_tests; i ++) {
        helper_test_array(inputs[i], targets[i], 7, expected[i]);
    }
    std::cout<< __func__ << ": "<< i + 1 << " tests passed" << std::endl;
}


int main() {
    test_one_element();
    test_short_array();
    test_repeated_element();
    test_reversed_array();
    return 0;
}