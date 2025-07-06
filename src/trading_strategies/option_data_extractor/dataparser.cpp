#include <iostream>
#include <string>

std::string extractField(const std::string &data, const std::string &field)
{
    size_t startPos = data.find(field + "=");
    if (startPos == std::string::npos)
    {
        return "";
    }
    startPos += field.length() + 1;
    size_t endPos = data.find(",", startPos);
    if (endPos == std::string::npos)
    {
        endPos = data.length();
    }
    return data.substr(startPos, endPos - startPos);
}

std::string getSymbol(const std::string &data)
{
    return extractField(data, "instrument");
}

std::string getTime(const std::string &data)
{
    return extractField(data, "time");
}

double getPutStrike(const std::string &data)
{
    std::string strike = extractField(data, "PUT_STRIKE");
    return !strike.empty() ? std::stod(strike) : 0.0;
}

double getPutPremium(const std::string &data)
{
    std::string premium = extractField(data, "PUT_PREMIUM");
    return !premium.empty() ? std::stod(premium) : 0.0;
}

double getCallStrike(const std::string &data)
{
    std::string strike = extractField(data, "CALL_STRIKE");
    return !strike.empty() ? std::stod(strike) : 0.0;
}

double getCallPremium(const std::string &data)
{
    std::string premium = extractField(data, "CALL_PREMIUM");
    return !premium.empty() ? std::stod(premium) : 0.0;
}

std::string getExpirationDate(const std::string &data)
{
    return extractField(data, "EXPIRATION_DATE");
}

double getStockPrice(const std::string &data)
{
    std::string price = extractField(data, "STOCK_PRICE");
    return !price.empty() ? std::stod(price) : 0.0;
}

int main()
{
    std::string sampleData = "instrument= AAPL,time=2024-05-16 10:30:00,PUT_STRIKE=200.00,PUT_PREMIUM=5.25,CALL_STRIKE=210.00,CALL_PREMIUM=6.75,EXPIRATION_DATE=2024-06-15,STOCK_PRICE=205.50";

    std::cout << "Stock Symbol: " << getSymbol(sampleData) << std::endl;
    std::cout << "Time: " << getTime(sampleData) << std::endl;
    std::cout << "Put Strike: " << getPutStrike(sampleData) << std::endl;
    std::cout << "Put Premium: " << getPutPremium(sampleData) << std::endl;
    std::cout << "Call Strike: " << getCallStrike(sampleData) << std::endl;
    std::cout << "Call Premium: " << getCallPremium(sampleData) << std::endl;
    std::cout << "Expiration Date: " << getExpirationDate(sampleData) << std::endl;
    std::cout << "Stock Price: " << getStockPrice(sampleData) << std::endl;

    return 0;
}
