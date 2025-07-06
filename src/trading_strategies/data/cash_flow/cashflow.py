import csv
import json
import requests

# Define a function to fetch financial data for a specific stock symbol from the Alpha Vantage API
def get_financial_data(symbol):
    url = f'https://www.alphavantage.co/query?function=CASH_FLOW&symbol={symbol}&apikey=T40Z69CAZY4R3N3G'
    response = requests.get(url)
    data = response.json()  # Parse the API response into JSON format
    return data

# Define a function to extract stock symbols from a CSV file
def extract_symbols_from_csv(csv_file, start_line, end_line):
    symbols = []
    with open(csv_file, 'r', newline='') as file:
        reader = csv.reader(file)
        for _ in range(start_line - 1):  # Skip lines before the start_line
            next(reader)
        for _ in range(start_line, end_line + 1):  # Extract lines from start_line to end_line
            try:
                row = next(reader)
                symbol = row[0].strip()  # Assuming stock symbols are in the first column
                symbols.append(symbol)
            except StopIteration:
                break
    return symbols

# Define a function to merge stock symbols extracted from the CSV file with financial data fetched from the API
def merge_data(csv_file, start_line, end_line):
    api_key = 'T40Z69CAZY4R3N3G'  # Alpha Vantage API key
    companies = extract_symbols_from_csv(csv_file, start_line, end_line)  # Extract stock symbols
    all_data = {}
    for company in companies:
        data = get_financial_data(company)  # Fetch financial data for each company
        all_data[company] = data  # Add financial data to the all_data dictionary, keyed by company symbol
    return all_data

# Define a function to save data as a JSON file
def save_json(data, file_path):
    with open(file_path, "w") as json_file:
        json.dump(data, json_file, indent=4)  # Write data to JSON file with indentation of 4 spaces
    print("Financial data for all companies has been saved to the file:", file_path)

# CSV file path
csv_file = "/Users/hankaorushou/Desktop/data.csv"
# Start and end lines
start_line = 482
end_line = 504

# Merge data
merged_data = merge_data(csv_file, start_line, end_line)
# Save the merged data as a JSON file
save_json(merged_data, "cashflow.json")
