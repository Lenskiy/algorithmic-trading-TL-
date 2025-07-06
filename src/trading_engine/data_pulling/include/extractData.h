#include <string>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>

using std::string;
string extractField(const string& data, const string& field) {
    size_t startPos = data.find(field + "=");
    if (startPos == string::npos) {
        return "";
    }
    startPos += field.length() + 1;
    size_t endPos = data.find(",", startPos);
    if (endPos == string::npos) {
        endPos = data.length();
    }
    return data.substr(startPos, endPos - startPos);
}

string getSymbol(const string& data) {
    string symbol = extractField(data, "instrument");
    if (symbol.empty()) {
        size_t pos = data.find("trade:");
        if (pos != string::npos) {
            size_t end = data.find("\"", pos);
            if (end != string::npos) {
                symbol = data.substr(pos + 6, end - pos - 6);
            }
        }
    }
    return symbol.empty() ? "XBTUSD" : symbol;
}

long long getTime(const std::string& data) {
    size_t timePos = data.find("time = ");
    if (timePos == std::string::npos) {
        throw std::runtime_error("Time not found in data string");
    }
    
    std::string timeStr = data.substr(timePos + 7, 27);
    std::tm tm = {};
    std::istringstream ss(timeStr);
    ss >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%S");
    
    if (ss.fail()) {
        throw std::runtime_error("Failed to parse time string");
    }
    
    auto tp = std::chrono::system_clock::from_time_t(std::mktime(&tm));
    
    int milliseconds = 0;
    size_t dotPos = timeStr.find('.');
    if (dotPos != std::string::npos) {
        std::string ms = timeStr.substr(dotPos + 1, 3);
        milliseconds = std::stoi(ms);
    }
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
    return duration.count() + milliseconds;
}

double getBidPrice(const string& data) {
    string price = extractField(data, "BID_PRICE");
    return !price.empty() ? stod(price) : 0.0;
}

double getBidSize(const string& data) {
    string size = extractField(data, "BID_SIZE");
    return !size.empty() ? stod(size) : 0.0;
}

double getAskPrice(const string& data) {
    string price = extractField(data, "ASK_PRICE");
    return !price.empty() ? stod(price) : 0.0;
}

double getAskSize(const string& data) {
    string size = extractField(data, "ASK_SIZE");
    return !size.empty() ? stod(size) : 0.0;
}

string getSymbolDepth(const string& data) {
    return extractField(data, "SYMBOL");
}

bool getIsBuy(const string& data) {
    string isBuyerMaker = extractField(data, "IS_BUYER_MAKER");
    return isBuyerMaker == "1";
}

float getPrice(const string& data) {
    string priceStr = extractField(data, "LAST_PRICE");
    return !priceStr.empty() ? stof(priceStr) : 0.0f;
}

float getSize(const string& data) {
    string sizeStr = extractField(data, "LAST_SIZE");
    return !sizeStr.empty() ? stof(sizeStr) : 0.0f;
}