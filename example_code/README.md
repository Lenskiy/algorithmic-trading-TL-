In ubuntu environment:

Those for compiling:

```bash
g++ webSocket.cpp -o webSocket -lssl -lcrypto
```

```bash
g++ -o sqlRead sqlRead.cpp -lpqxx -lpq
```

```bash
g++ parquetWrite.cpp -o parquetWrite -larrow -lparquet
```

```bash
g++ parquetRead.cpp -o parquetRead -larrow -lparquet
```

Then run:

```bash
./webSocket
```

```bash
./sqlRead
```

```bash
./parquetWrite
```

```bash
./parquetRead
```

Based on:

[BOOST] (https://www.boost.org/doc/libs/develop/libs/beast/doc/html/index.html)

[ARROW] (https://arrow.apache.org/docs/cpp/parquet.html#)

`Basic type in Arrow (For reference): `

`Understanding parquet::Type (Physical Types)`
These types describe the raw format in which data is stored in the Parquet file. Here are the primary physical types used in Parquet and typical scenarios where they are applicable:

`BOOLEAN`: Used for true/false values.
`INT32`: Used for 32-bit integers. Suitable for storing data that fits within this range, including dates and times (if additional logical type information is not required).
`INT64`: Used for 64-bit integers. Suitable for storing larger integers or timestamps.
`FLOAT`: Used for single-precision (32-bit) floating-point numbers.
`DOUBLE`: Used for double-precision (64-bit) floating-point numbers.
`BYTE_ARRAY`: Used for variable-length binary data, such as strings or arbitrary binary blobs.
FIXED_LEN_BYTE_ARRAY: Used for binary data of a specific, fixed length. Often used for encoding decimal values or other fixed-length binary data.

`Understanding parquet::ConvertedType (Logical Types)`
Logical types provide context on how the raw data stored as physical types should be interpreted. They enable more precise data handling and are essential for ensuring data compatibility and correctness:

`NONE`: Indicates no special logical interpretation. The data is to be interpreted exactly as defined by its physical type.
`UTF8`: Indicates that the data is stored as BYTE_ARRAY but should be interpreted as UTF-8 encoded text. This is commonly used for string data.
`DECIMAL`: Used for precise fixed-point numerical data that doesn't fit well into standard integer or floating-point types. Requires additional parameters such as precision and scale.
`DATE`: Indicates the data represents dates (number of days since a fixed epoch) and is typically stored as INT32.
`TIME_MILLIS`, `TIME_MICROS`: Represent time of day in milliseconds or microseconds since midnight. They provide more granularity than just storing time as INT32 or INT64.
`TIMESTAMP_MILLIS`, `TIMESTAMP_MICROS`: Used for timestamps in milliseconds or microseconds since the Unix epoch. This provides precise time representation, suitable for logs, event data, etc.
`UINT_8`, `UINT_16`, `UINT_32`, `UINT_64`: Used for unsigned integers, useful when handling data that should never be negative, such as quantities, counts, etc.

Let's see example in parquetWrite:

```
struct Trade {
    std::string trade_date;       
    std::string trade_time;          
    int64_t execution_timestamp;     
    int trader_id;
    std::string asset_symbol;
    int quantity;
    double price;

    // constructor 
    Trade(std::string date, std::string time, int64_t timestamp, int trader, std::string symbol, int qty, double pr) :
        trade_date(date), trade_time(time), execution_timestamp(timestamp),
        trader_id(trader), asset_symbol(symbol), quantity(qty), price(pr) {}
};

```
`struct `is similar with class(also class in Java), the only difference in C++ is that struct's member variables are public by default and `class` is private.

Then is the way we build constructor(remember also modify this if modify the member variables). Also need to modify:

```
    // Define schema using Parquet schema primitives directly
    std::shared_ptr<parquet::schema::GroupNode> schema = std::static_pointer_cast<parquet::schema::GroupNode>(
    parquet::schema::GroupNode::Make(
        "trading_record",
        parquet::Repetition::REQUIRED, {
            parquet::schema::PrimitiveNode::Make("trade_date", parquet::Repetition::REQUIRED, parquet::Type::BYTE_ARRAY, parquet::ConvertedType::UTF8),
            parquet::schema::PrimitiveNode::Make("trade_time", parquet::Repetition::REQUIRED, parquet::Type::BYTE_ARRAY, parquet::ConvertedType::UTF8),  // Modified to store time as a string
            parquet::schema::PrimitiveNode::Make("execution_timestamp", parquet::Repetition::REQUIRED, parquet::Type::INT64, parquet::ConvertedType::INT_64),
            parquet::schema::PrimitiveNode::Make("trader_id", parquet::Repetition::REQUIRED, parquet::Type::INT32, parquet::ConvertedType::INT_32),
            parquet::schema::PrimitiveNode::Make("asset_symbol", parquet::Repetition::REQUIRED, parquet::Type::BYTE_ARRAY, parquet::ConvertedType::UTF8),
            parquet::schema::PrimitiveNode::Make("quantity", parquet::Repetition::REQUIRED, parquet::Type::INT32, parquet::ConvertedType::INT_32),
            parquet::schema::PrimitiveNode::Make("price", parquet::Repetition::REQUIRED, parquet::Type::DOUBLE, parquet::ConvertedType::NONE)
        }));
```

For example:

```
struct Trade {
    std::string example
    std::string trade_date;       
    std::string trade_time;          
    int64_t execution_timestamp;     
    int trader_id;
    std::string asset_symbol;
    int quantity;
    double price;

    // constructor 
    Trade(std::exa, std::string date, std::string time, int64_t timestamp, int trader, std::string symbol, int qty, double pr) :
        example(exa),trade_date(date), trade_time(time), execution_timestamp(timestamp),
        trader_id(trader), asset_symbol(symbol), quantity(qty), price(pr) {}
};

```

And:


```
    // Define schema using Parquet schema primitives directly
    std::shared_ptr<parquet::schema::GroupNode> schema = std::static_pointer_cast<parquet::schema::GroupNode>(
    parquet::schema::GroupNode::Make(
        "trading_record",
        parquet::Repetition::REQUIRED, {
            parquet::schema::PrimitiveNode::Make("example", parquet::Repetition::REQUIRED, parquet::Type::BYTE_ARRAY, parquet::ConvertedType::UTF8),
            parquet::schema::PrimitiveNode::Make("trade_date", parquet::Repetition::REQUIRED, parquet::Type::BYTE_ARRAY, parquet::ConvertedType::UTF8),
            parquet::schema::PrimitiveNode::Make("trade_time", parquet::Repetition::REQUIRED, parquet::Type::BYTE_ARRAY, parquet::ConvertedType::UTF8), 
            parquet::schema::PrimitiveNode::Make("execution_timestamp", parquet::Repetition::REQUIRED, parquet::Type::INT64, parquet::ConvertedType::INT_64),
            parquet::schema::PrimitiveNode::Make("trader_id", parquet::Repetition::REQUIRED, parquet::Type::INT32, parquet::ConvertedType::INT_32),
            parquet::schema::PrimitiveNode::Make("asset_symbol", parquet::Repetition::REQUIRED, parquet::Type::BYTE_ARRAY, parquet::ConvertedType::UTF8),
            parquet::schema::PrimitiveNode::Make("quantity", parquet::Repetition::REQUIRED, parquet::Type::INT32, parquet::ConvertedType::INT_32),
            parquet::schema::PrimitiveNode::Make("price", parquet::Repetition::REQUIRED, parquet::Type::DOUBLE, parquet::ConvertedType::NONE)
        }));
```

And also remember:

```
// Function to simulate getting trade data
std::vector<Trade> getTrades() {
    return {
        {"2023-10-01", "09:00:00", 1664611200000, 1, "AAPL", 100, 150.25},
        {"2023-10-01", "10:00:00", 1664614800000, 2, "MSFT", 150, 250.75},
        // Additional trades can be added here
    };
}
```

And:

```
// Get data and write
for (const auto& trade : getTrades()) {
    os << trade.trade_date << trade.trade_time << trade.execution_timestamp
        << trade.trader_id << trade.asset_symbol << trade.quantity << trade.price << parquet::EndRow;
}
```

to:

```
// Function to simulate getting trade data
std::vector<Trade> getTrades() {
    return {
        {"example1" , "2023-10-01", "09:00:00", 1664611200000, 1, "AAPL", 100, 150.25},
        {"example2","2023-10-01", "10:00:00", 1664614800000, 2, "MSFT", 150, 250.75},
        // Additional trades can be added here
    };
}
```
And:

```
// Get data and write
for (const auto& trade : getTrades()) {
    os <<trade.example<< trade.trade_date << trade.trade_time << trade.execution_timestamp
        << trade.trader_id << trade.asset_symbol << trade.quantity << trade.price << parquet::EndRow;
}
```

Same changes in parquetRead:

Change

```
std::string trade_date;           
std::string trade_time;           
int64_t execution_timestamp;      
int trader_id;
std::string asset_symbol;
int quantity;
double price;

// Read data
while (!os.eof()) {
    os >> trade_date >> trade_time >> execution_timestamp
    >> trader_id >> asset_symbol >> quantity >> price >> parquet::EndRow;

    std::cout << "trade_date: " << trade_date << ", trade_time: " << trade_time << ", execution_timestamp: " 
    << execution_timestamp << ", trader_id:" <<trader_id<< ", asset_symbol:" << asset_symbol
    << ", quantity:" << quantity << ", price:"<<price<< std::endl;
}
```
to:

```
std::string example
std::string trade_date;           
std::string trade_time;           
int64_t execution_timestamp;      
int trader_id;
std::string asset_symbol;
int quantity;
double price;

// Read data
while (!os.eof()) {
    os >> example >> trade_date >> trade_time >> execution_timestamp
    >> trader_id >> asset_symbol >> quantity >> price >> parquet::EndRow;

    std::cout <<"example: " << example << "trade_date: " << trade_date << ", trade_time: " << trade_time << ", execution_timestamp: " << execution_timestamp << ", trader_id:" <<trader_id<< ", asset_symbol:" << asset_symbol << ", quantity:" << quantity << ", price:"<<price<< std::endl;
}
```