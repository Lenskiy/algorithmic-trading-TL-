#include <iostream>
#include <chrono>
#include <vector>
#include <unordered_map>
#include "arrow/io/api.h"
#include "parquet/arrow/reader.h"
#include <arrow/table.h>
#include <boost/circular_buffer.hpp>
#include <memory>
#include <string>
using namespace std;

/*
Here we have three different ways about cache.
1. No cache
2. unordered_map
3. boost::circular_buffer

Between two different caches, 
the critical point of the performance gap is almost 500 cached data (roughly). 
It is faster to use unordered_map when it is less than 500, 
and it is faster to use boost::circular_buffer when it is greater than 500.

Of course, here we read the entire table, 
and different replacements can be made based on the specific scenario. 
But overall, unless there are only 10 to 20 cached data, 
there will not be an order of magnitude difference between the two methods.
*/

// Cache Implementation
unordered_map<string, shared_ptr<arrow::Table>> cache;

struct CacheEntry {
    string key;
    shared_ptr<arrow::Table> table;
};
boost::circular_buffer<CacheEntry> cache2(500);

// Function to clear cache
void clear_cache() {
    cache.clear();
}

// Function to read data from cache or disk
shared_ptr<arrow::Table> read_data(const string& file_path, bool use_cache) {
    string key = "unique_key_based_on_file_path";

    if (use_cache) {
        auto it = cache.find(key);
        if (it != cache.end()) {
            return it->second;  // Cache hit
        }
    }

    // Read from disk (Simulating reading from Parquet)
    shared_ptr<arrow::io::ReadableFile> infile;
    PARQUET_ASSIGN_OR_THROW(infile, arrow::io::ReadableFile::Open(file_path, arrow::default_memory_pool()));

    unique_ptr<parquet::arrow::FileReader> reader;
    parquet::arrow::FileReaderBuilder builder;
    PARQUET_THROW_NOT_OK(builder.Open(infile));
    PARQUET_THROW_NOT_OK(builder.Build(&reader));

    shared_ptr<arrow::Table> table;
    PARQUET_THROW_NOT_OK(reader->ReadTable(&table));

    if (use_cache) {
        cache[key] = table;  // Add to cache
    }
    return table;
}

// Function to read data from cache or disk
shared_ptr<arrow::Table> read_data2(const string& file_path, bool use_cache) {
    string key = "unique_key_based_on_file_path";

    if (use_cache) {
        for (const auto& entry : cache2) {
            if (entry.key == key) {
                return entry.table;  // Cache hit
            }
        }
    }

    // Read from disk (Simulating reading from Parquet)
    shared_ptr<arrow::io::ReadableFile> infile;
    PARQUET_ASSIGN_OR_THROW(infile, arrow::io::ReadableFile::Open(file_path, arrow::default_memory_pool()));

    unique_ptr<parquet::arrow::FileReader> reader;
    parquet::arrow::FileReaderBuilder builder;
    PARQUET_THROW_NOT_OK(builder.Open(infile));
    PARQUET_THROW_NOT_OK(builder.Build(&reader));

    shared_ptr<arrow::Table> table;
    PARQUET_THROW_NOT_OK(reader->ReadTable(&table));

    if (use_cache) {
        cache2.push_back({key, table});
    }
    return table;
}

// Testing Function
void performance_test(const string& file_path) {
    const int num_runs = 500;

    // Test without cache
    auto start_nc = chrono::high_resolution_clock::now();
    for (int i = 0; i < num_runs; ++i) {
        read_data(file_path, false);
    }
    auto end_nc = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed_nc = end_nc - start_nc;
    cout << "No Cache Total Time: " << elapsed_nc.count() << " seconds." << endl;

    // Clear cache
    clear_cache();

    // Test with cache
    auto start_c = chrono::high_resolution_clock::now();
    for (int i = 0; i < num_runs; ++i) {
        read_data(file_path, true);
    }
    auto end_c = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed_c = end_c - start_c;
    cout << "Cache Total Time: " << elapsed_c.count() << " seconds." << endl;

    auto start_uc = chrono::high_resolution_clock::now();
    for(int i=0; i< num_runs; ++i){
        read_data2(file_path,true);
    }
    auto end_uc = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed_uc = end_uc - start_uc;
    cout << "Cache2 Total Time: " << elapsed_uc.count() << " seconds." << endl;
}

int main() {
    string file_path = "large_test.parquet";
    performance_test(file_path);
    return 0;
}
