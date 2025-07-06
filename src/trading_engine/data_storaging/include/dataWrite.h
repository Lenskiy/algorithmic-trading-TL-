#include <arrow/io/api.h>
#include <parquet/arrow/schema.h>
#include <parquet/arrow/writer.h>
#include <parquet/arrow/reader.h>
#include <parquet/stream_writer.h>
#include <arrow/api.h>
#include <iostream>
#include <vector>
#include <string>
#include <mutex>

using namespace std;

struct Price {
    std::string stock_code;
    std::string market_code;
    double open;
    double close;
    double high;
    double low;
    int64_t time;  // Ensure that the time is int64_t
    double count;   // Number of trades
    double amount;  // Volume
    std::string frequency;

    Price(std::string stock_code_, std::string market_code_, double open_, double close_, double high_, double low_,
          int64_t time_, double count_, double amount_, std::string frequency_)
        : stock_code(stock_code_), market_code(market_code_), open(open_), close(close_), high(high_), low(low_),
          time(time_), count(count_), amount(amount_), frequency(frequency_) {}
};

class DataWriter {
public:
    DataWriter(const string& file_path) : file_path_(file_path) {}

    void writeData(const vector<Price>& prices) {
        lock_guard<mutex> lock(mutex_);

        // Read existing data (if any)
        vector<Price> existing_prices = readExistingData();

        // Append new prices to existing prices
        existing_prices.insert(existing_prices.end(), prices.begin(), prices.end());

        // Write combined prices to Parquet file
        writePricesToFile(existing_prices);
    }

private:
    string file_path_;
    mutex mutex_;

    vector<Price> readExistingData() {
        vector<Price> prices;
        shared_ptr<arrow::io::ReadableFile> infile;
        arrow::Status status = arrow::io::ReadableFile::Open(file_path_, arrow::default_memory_pool()).Value(&infile);
        if (!status.ok()) {
            cerr << "No existing file found. Creating a new one." << endl;
            return prices;
        }

        unique_ptr<parquet::arrow::FileReader> reader;
        parquet::arrow::FileReaderBuilder builder;
        PARQUET_THROW_NOT_OK(builder.Open(infile));
        PARQUET_THROW_NOT_OK(builder.Build(&reader));

        reader->set_use_threads(true);
        shared_ptr<arrow::Table> table;
        PARQUET_THROW_NOT_OK(reader->ReadTable(&table));

        // Assume the schema for the table matches the Price structure
        auto stock_code_array = static_pointer_cast<arrow::StringArray>(table->column(0)->chunk(0));
        auto market_code_array = static_pointer_cast<arrow::StringArray>(table->column(1)->chunk(0));
        auto open_array = static_pointer_cast<arrow::DoubleArray>(table->column(2)->chunk(0));
        auto close_array = static_pointer_cast<arrow::DoubleArray>(table->column(3)->chunk(0));
        auto high_array = static_pointer_cast<arrow::DoubleArray>(table->column(4)->chunk(0));
        auto low_array = static_pointer_cast<arrow::DoubleArray>(table->column(5)->chunk(0));
        auto time_array = static_pointer_cast<arrow::Int64Array>(table->column(6)->chunk(0));
        auto count_array = static_pointer_cast<arrow::DoubleArray>(table->column(7)->chunk(0));
        auto amount_array = static_pointer_cast<arrow::DoubleArray>(table->column(8)->chunk(0));
        auto frequency_array = static_pointer_cast<arrow::StringArray>(table->column(9)->chunk(0));

        for (int64_t i = 0; i < table->num_rows(); ++i) {
            prices.emplace_back(stock_code_array->GetString(i),
                                market_code_array->GetString(i),
                                open_array->Value(i),
                                close_array->Value(i),
                                high_array->Value(i),
                                low_array->Value(i),
                                time_array->Value(i),
                                count_array->Value(i),
                                amount_array->Value(i),
                                frequency_array->GetString(i));
        }

        return prices;
    }

    void writePricesToFile(const vector<Price>& prices) {
    try {
        // Define the Parquet schema for the Price structure
        shared_ptr<parquet::schema::GroupNode> schema = static_pointer_cast<parquet::schema::GroupNode>(
            parquet::schema::GroupNode::Make(
                "price_record",
                parquet::Repetition::REQUIRED, {
                    parquet::schema::PrimitiveNode::Make("stock_code", parquet::Repetition::REQUIRED, parquet::Type::BYTE_ARRAY, parquet::ConvertedType::UTF8),
                    parquet::schema::PrimitiveNode::Make("market_code", parquet::Repetition::REQUIRED, parquet::Type::BYTE_ARRAY, parquet::ConvertedType::UTF8),
                    parquet::schema::PrimitiveNode::Make("open", parquet::Repetition::REQUIRED, parquet::Type::DOUBLE, parquet::ConvertedType::NONE),
                    parquet::schema::PrimitiveNode::Make("close", parquet::Repetition::REQUIRED, parquet::Type::DOUBLE, parquet::ConvertedType::NONE),
                    parquet::schema::PrimitiveNode::Make("high", parquet::Repetition::REQUIRED, parquet::Type::DOUBLE, parquet::ConvertedType::NONE),
                    parquet::schema::PrimitiveNode::Make("low", parquet::Repetition::REQUIRED, parquet::Type::DOUBLE, parquet::ConvertedType::NONE),
                    parquet::schema::PrimitiveNode::Make("time", parquet::Repetition::REQUIRED, parquet::Type::INT64, parquet::ConvertedType::INT_64),  // Ensure INT64
                    parquet::schema::PrimitiveNode::Make("count", parquet::Repetition::REQUIRED, parquet::Type::DOUBLE, parquet::ConvertedType::NONE),
                    parquet::schema::PrimitiveNode::Make("amount", parquet::Repetition::REQUIRED, parquet::Type::DOUBLE, parquet::ConvertedType::NONE),
                    parquet::schema::PrimitiveNode::Make("frequency", parquet::Repetition::REQUIRED, parquet::Type::BYTE_ARRAY, parquet::ConvertedType::UTF8)
                }));

        // Create output file stream for writing Parquet data
        shared_ptr<arrow::io::FileOutputStream> outfile;
        PARQUET_THROW_NOT_OK(arrow::io::FileOutputStream::Open(file_path_).Value(&outfile));

        // Create Parquet writer and properties
        parquet::WriterProperties::Builder builder;
        unique_ptr<parquet::ParquetFileWriter> file_writer = parquet::ParquetFileWriter::Open(outfile, schema, builder.build());

        // StreamWriter to write rows to the file
        parquet::StreamWriter os(move(file_writer));

        // Write each price record to the Parquet file
        for (const auto& price : prices) {
            RCLCPP_INFO(rclcpp::get_logger("parquet_writer"), "Writing data: stock_code=%s, time=%lld", price.stock_code.c_str(), static_cast<long long>(price.time));

            os << price.stock_code << price.market_code << price.open << price.close
               << price.high << price.low << static_cast<int64_t>(price.time)  // Ensure time is written as INT64
               << price.count << price.amount << price.frequency << parquet::EndRow;
        }

        // Ensure the output file is properly closed and handle its return value
        PARQUET_THROW_NOT_OK(outfile->Close());  // Properly handle closing

    } catch (const std::exception& e) {
        // Catch and log any exceptions to help with debugging
        cerr << "Error writing Parquet file: " << e.what() << endl;
    }
    }

};
