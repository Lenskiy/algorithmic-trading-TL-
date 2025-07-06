# Description
This C++ code provides functions to extract specific fields from a comma-separated string of data. It is designed to handle financial data related to options trading, extracting information such as instrument symbol, time, strike prices, premiums, expiration date, and stock price.

## Functionality
extractField(const std::string &data, const std::string &field):

This function takes in a string of data and a specific field to extract.
It searches for the field within the data string and returns the value associated with that field.
If the field is not found, it returns an empty string.
getSymbol(const std::string &data):

Returns the symbol of the instrument extracted from the data.
getTime(const std::string &data):

Returns the timestamp extracted from the data.
getPutStrike(const std::string &data):

Returns the strike price of the put option extracted from the data.
getPutPremium(const std::string &data):

Returns the premium of the put option extracted from the data.
getCallStrike(const std::string &data):

Returns the strike price of the call option extracted from the data.
getCallPremium(const std::string &data):

Returns the premium of the call option extracted from the data.
getExpirationDate(const std::string &data):

Returns the expiration date extracted from the data.
getStockPrice(const std::string &data):

Returns the stock price extracted from the data.

## How to Use
Compile the code using a C++ compiler. 

## Example Output

Stock Symbol: AAPL
Time: 2024-05-16 10:30:00
Put Strike: 200
Put Premium: 5.25
Call Strike: 210
Call Premium: 6.75
Expiration Date: 2024-06-15
Stock Price: 205.5

## Requirements
C++ environment and Internet connection

