import csv
import json
import requests

def get_financial_data(symbol):
    url = f'https://www.alphavantage.co/query?function=BALANCE_SHEET&symbol={symbol}&apikey=NO7XJ7P7CJZ60LZ0'
    response = requests.get(url)
    data = response.json()
    return data

def extract_symbols_from_csv(csv_file, start_line, end_line):
    symbols = []
    with open(csv_file, 'r', newline='') as file:
        reader = csv.reader(file)
        for _ in range(start_line - 1):  
            next(reader)
        for _ in range(start_line, end_line + 1): 
            try:
                row = next(reader)
                symbol = row[0].strip() 
                symbols.append(symbol)
            except StopIteration:
                break
    return symbols

def merge_data(csv_file, start_line, end_line):
    api_key = 'NO7XJ7P7CJZ60LZ0'
    companies = extract_symbols_from_csv(csv_file, start_line, end_line)
    all_data = {}
    for company in companies:
        data = get_financial_data(company)
        all_data[company] = data
    return all_data

def save_json(data, file_path):
    with open(file_path, "w") as json_file:
        json.dump(data, json_file, indent=4)
    print("bs:", file_path)

csv_file = "/Users/hankaorushou/Desktop/data.csv"
start_line = 1
end_line = 25

merged_data = merge_data(csv_file, start_line, end_line)
save_json(merged_data, "balance_sheet.json")