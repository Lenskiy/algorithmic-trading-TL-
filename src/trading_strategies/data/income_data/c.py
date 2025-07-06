import csv
import json
import requests

def get_financial_data(symbol):
    url = f'https://www.alphavantage.co/query?function=INCOME_STATEMENT&symbol={symbol}&apikey=DU1INHR9NU05Z2IC'
    response = requests.get(url)
    data = response.json()
    return data

def save_json(data, file_path):
    with open(file_path, "w") as json_file:
        json.dump(data, json_file)
    print("JSON file will save:", file_path)

def extract_symbols_from_csv(csv_file):
    symbols = []
    with open(csv_file, 'r', newline='') as file:
        reader = csv.reader(file)
        for _ in range(158):  
            next(reader)
        for _ in range(183):  
            row = next(reader)
            symbol = row[0].strip()  
            symbols.append(symbol)
    return symbols

api_key = 'DU1INHR9NU05Z2IC'
csv_file = "/Users/hankaorushou/Desktop/data.csv"  
companies = extract_symbols_from_csv(csv_file)

all_data = {}

for company in companies:
    data = get_financial_data(company)
    all_data[company] = data

save_json(all_data, "financial_data.json")

print("file will save 'financial_data.json'")