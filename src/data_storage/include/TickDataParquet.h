#include <arrow/io/api.h>
#include <parquet/arrow/schema.h>
#include <parquet/arrow/writer.h>
#include <parquet/arrow/reader.h>
#include <parquet/stream_writer.h>
#include <arrow/api.h>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include "system_interface/msg/tick_data.hpp"
using namespace std;

/*
 * Notice if we reopen the parquet file and write it,
 * we will overwrite the initial version.
 * So we need to read the parquet file and append the new data,
 * then write it into new parquet file.
 */

class TickDataParquet
{
public:
    TickDataParquet(const string &file_path) : file_path_(file_path) {}

    void writeData(const vector<system_interface::msg::TickData> &trades)
    {
        lock_guard<mutex> lock(mutex_);

        // Read existing data
        vector<system_interface::msg::TickData> existing_trades = readExistingData();

        // Append new trades to existing trades
        existing_trades.insert(existing_trades.end(), trades.begin(), trades.end());

        // Write combined trades to Parquet file
        writeTradesToFile(existing_trades);
    }

    vector<system_interface::msg::TickData> readData()
    {
        return readExistingData();
    }

private:
    string file_path_;
    mutex mutex_;

    vector<system_interface::msg::TickData> readExistingData()
    {
        vector<system_interface::msg::TickData> trades;
        shared_ptr<arrow::io::ReadableFile> infile;
        arrow::Status status = arrow::io::ReadableFile::Open(file_path_, arrow::default_memory_pool()).Value(&infile);
        if (!status.ok())
        {
            cerr << "No existing file found. Creating a new one." << endl;
            return trades;
        }

        unique_ptr<parquet::arrow::FileReader> reader;
        parquet::arrow::FileReaderBuilder builder;
        PARQUET_THROW_NOT_OK(builder.Open(infile));
        PARQUET_THROW_NOT_OK(builder.Build(&reader));

        reader->set_use_threads(true);

        shared_ptr<arrow::Table> table;
        PARQUET_THROW_NOT_OK(reader->ReadTable(&table));

        auto symbol_array = static_pointer_cast<arrow::StringArray>(table->column(0)->chunk(0));
        auto exchange_array = static_pointer_cast<arrow::StringArray>(table->column(1)->chunk(0));
        auto is_buy_array = static_pointer_cast<arrow::BooleanArray>(table->column(2)->chunk(0));
        auto price_array = static_pointer_cast<arrow::FloatArray>(table->column(3)->chunk(0));
        auto size_array = static_pointer_cast<arrow::FloatArray>(table->column(4)->chunk(0));
        auto time_array = static_pointer_cast<arrow::Int64Array>(table->column(5)->chunk(0));

        for (int64_t i = 0; i < table->num_rows(); ++i)
        {
            system_interface::msg::TickData tick_data;
            tick_data.symbol = symbol_array->GetString(i);
            tick_data.exchange = exchange_array->GetString(i);
            tick_data.is_buy = is_buy_array->Value(i);
            tick_data.price = price_array->Value(i);
            tick_data.size = size_array->Value(i);
            tick_data.time = time_array->Value(i);
            trades.push_back(tick_data);
        }

        return trades;
    }

    void writeTradesToFile(const vector<system_interface::msg::TickData> &trades)
    {
        shared_ptr<parquet::schema::GroupNode> schema = static_pointer_cast<parquet::schema::GroupNode>(
            parquet::schema::GroupNode::Make(
                "trading_record",
                parquet::Repetition::REQUIRED, {
                    parquet::schema::PrimitiveNode::Make("symbol", parquet::Repetition::REQUIRED, parquet::Type::BYTE_ARRAY, parquet::ConvertedType::UTF8),
                    parquet::schema::PrimitiveNode::Make("exchange", parquet::Repetition::REQUIRED, parquet::Type::BYTE_ARRAY, parquet::ConvertedType::UTF8),
                    parquet::schema::PrimitiveNode::Make("is_buy", parquet::Repetition::REQUIRED, parquet::Type::BOOLEAN, parquet::ConvertedType::NONE),
                    parquet::schema::PrimitiveNode::Make("price", parquet::Repetition::REQUIRED, parquet::Type::FLOAT, parquet::ConvertedType::NONE),
                    parquet::schema::PrimitiveNode::Make("size", parquet::Repetition::REQUIRED, parquet::Type::FLOAT, parquet::ConvertedType::NONE),
                    parquet::schema::PrimitiveNode::Make("time", parquet::Repetition::REQUIRED, parquet::Type::INT64, parquet::ConvertedType::INT_64)
                }));

        shared_ptr<arrow::io::FileOutputStream> outfile;
        PARQUET_THROW_NOT_OK(arrow::io::FileOutputStream::Open(file_path_).Value(&outfile));
        parquet::WriterProperties::Builder builder;
        unique_ptr<parquet::ParquetFileWriter> file_writer = parquet::ParquetFileWriter::Open(outfile, schema, builder.build());
        parquet::StreamWriter os(move(file_writer));

        for (const auto &trade : trades)
        {
            os << trade.symbol << trade.exchange << trade.is_buy
               << trade.price << trade.size << trade.time << parquet::EndRow;
        }
    }
};
