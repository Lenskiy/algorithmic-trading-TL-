# Automating Financial Data Updates

## Overview

Data collection and API design team aims to automate the updating process of financial data for companies. By dividing the list of companies into smaller batches and scheduling periodic execution of data update scripts, an efficient data updating workflow can be achieved. Additionally, parallel processing of multiple batches of data updates can be considered to further enhance efficiency.


## Implementation Steps

The purpose of this project is to automate the retrieval of financial data from the Alpha Vantage API and save it to JSON files. This automation process is implemented using Python and several libraries.

Tools Used and Implementation Steps
Python 3.x
Requests library: Used to make HTTP requests to fetch data.
APScheduler library: Used for scheduling task execution.
Alpha Vantage API: Used for fetching financial data.
CSV file: Contains the symbols of companies for which financial data needs to be updated.

Overview of Automation Process

### Fetching Financial Data: 

The get_financial_data(symbol) function retrieves financial data, such as cash flow statements, for a specified company from the Alpha Vantage API.

### Extracting Company Symbols from CSV: 

The extract_symbols_from_csv(csv_file, start_line, end_line) function extracts the symbols of companies from the provided CSV file to facilitate data retrieval.

### Merging Data: 

The merge_data(csv_file, start_line, end_line) function merges the retrieved financial data with company symbols to create a dictionary containing data for all companies.

### Saving Data to JSON File: 

The save_json(data, file_path) function saves the merged data to the specified JSON file.
Automating Updates: The automate_data_update(csv_file, start_line, end_line, output_file) function combines the above steps and provides an interface for automating data updates.

### Scheduling Tasks: 

Using the APScheduler library, tasks such as triggering data updates every Saturday at midnight, every first day of the first month of each quarter at midnight, and every first day of the first month of each year at midnight can be scheduled. (Actual scheduling may vary based on specific release dates of company data.) Scheduled tasks can only complete future automated updates.

## How to Use
Save the company symbols to a CSV file, ensuring the file path is correct.
Set the CSV file path (csv_file), start line (start_line), end line (end_line), and output JSON file path (output_file) in the main function.
Run the script, and it will automatically start updating data at the specified times.

## Notes
Ensure compliance with the terms and conditions of the Alpha Vantage API when using it.
Adjust the scheduling settings as needed to meet specific update requirements.

## Requirements
Python 3.x
Internet connection

