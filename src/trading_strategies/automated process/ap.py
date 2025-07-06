import csv
import json
import requests
import time
from apscheduler.schedulers.background import BackgroundScheduler

def get_financial_data(symbol):
   
    api_key = '4A7OEUFM6CGTCV8O'  
    url = f'https://www.alphavantage.co/query?function=CASH_FLOW&symbol={symbol}&apikey=4A7OEUFM6CGTCV8O'
    response = requests.get(url)
    if response.status_code == 200:
        data = response.json()
        time.sleep(1)  
        return data
    else:
        print(f"Failed to retrieve data for {symbol}")
        return None

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
    
    companies = extract_symbols_from_csv(csv_file, start_line, end_line)
    all_data = {}
    for company in companies:
        data = get_financial_data(company)
        if data:
            all_data[company] = data
    return all_data

def save_json(data, file_path):
    
    with open(file_path, "w") as json_file:
        json.dump(data, json_file, indent=4)
    print("Financial data for all companies has been saved to the file:", file_path)

def automate_data_update(csv_file, start_line, end_line, output_file):
   
    merged_data = merge_data(csv_file, start_line, end_line)
    save_json(merged_data, output_file)

if __name__ == "__main__":
    csv_file = "/Users/hankaorushou/Desktop/data.csv"  
    start_line = 53 
    end_line = 57
    output_file = "companies.json"  

    scheduler = BackgroundScheduler()
    # scheduler.add_job(automate_data_update, 'cron', args=[csv_file, start_line, end_line, output_file], day_of_week='sat', hour=0, minute=0)
    # scheduler.add_job(automate_data_update, 'cron', month='3,6,9,12', day='last', hour=23, minute=59, second=59, args=[output_file])
    # scheduler.add_job(automate_data_update, 'cron', month=1, day=1, hour=0, minute=0, second=0, args=[output_file]) #annual report
    scheduler.add_job(automate_data_update, 'date', run_date='2024-05-16 10:32:20', args=[csv_file, start_line, end_line, output_file])
    scheduler.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        scheduler.shutdown()
