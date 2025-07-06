#include <iostream>
#include <cassert>
#include <string>
#include "../src/trading_engine/data_pulling/include/extractData.h"

using namespace std;

void testExtractField() {
    cout << "Testing extractField..." << endl;
    assert(extractField("", "field") == "");
    assert(extractField("key1=value1,key2=value2", "nonexistent") == "");
    assert(extractField("field=value1,key2=value2,key3=value3", "field") == "value1");
    assert(extractField("key1=value1,field=value2,key3=value3", "field") == "value2");
    assert(extractField("key1=value1,key2=value2,field=value3", "field") == "value3");
    cout << "All extractField tests passed." << endl;
}

void testGetSymbol() {
    cout << "Testing getSymbol..." << endl;
    assert(getSymbol("instrument=AAPL,BID_PRICE=123.45,ASK_PRICE=123.50") == "AAPL");
    assert(getSymbol("BID_PRICE=123.45,ASK_PRICE=123.50") == "");
    cout << "All getSymbol tests passed." << endl;
}

void testGetTime() {
    cout << "Testing getTime..." << endl;
    assert(getTime("time = 12345678,BID_PRICE=123.45,ASK_PRICE=123.50") == "12345678");
    assert(getTime("BID_PRICE=123.45,ASK_PRICE=123.50") == "");
    cout << "All getTime tests passed." << endl;
}

void testGetPrices() {
    cout << "Testing getBidPrice and getAskPrice..." << endl;
    string data = "BID_PRICE=123.45,ASK_PRICE=123.50";
    assert(getBidPrice(data) == 123.45);
    assert(getAskPrice(data) == 123.50);

    data = "BID_SIZE=10,ASK_SIZE=20";
    assert(getBidPrice(data) == 0.0);
    assert(getAskPrice(data) == 0.0);
    cout << "All getBidPrice and getAskPrice tests passed." << endl;
}

void testGetSizes() {
    cout << "Testing getBidSize and getAskSize..." << endl;
    string data = "BID_SIZE=10,ASK_SIZE=20";
    assert(getBidSize(data) == 10.0);
    assert(getAskSize(data) == 20.0);

    data = "BID_PRICE=123.45,ASK_PRICE=123.50";
    assert(getBidSize(data) == 0.0);
    assert(getAskSize(data) == 0.0);
    cout << "All getBidSize and getAskSize tests passed." << endl;
}

int main() {
    testExtractField();
    testGetSymbol();
    testGetTime();
    testGetPrices();
    testGetSizes();

    cout << "All tests passed!" << endl;
    return 0;
}