import json
import os
import pandas as pd

from src.trading_strategies.strategies.financial_asset.symbol import Symbol


def read_file(file_path):
    if get_file_type(file_path).lower() == 'csv':
        return read_csv(file_path)
    elif get_file_type(file_path).lower() == 'json':
        return read_json(file_path)
    else:
        raise ValueError("Unsupported file format")


def read_csv(file_path):
    return pd.read_csv(file_path)


def read_json(file_path):
    with open(file_path, 'r') as f:
        content = json.load(f)
    return json.dumps(content)


def get_file_type(file_path):
    _, file_extension = os.path.splitext(file_path)
    return file_extension[1:]


def get_historical_values(symbol: Symbol, file_path, start_date='2004-01-01', end_date='2024-01-01'):
    data = read_file(file_path)
    filtered_data = data[(data['Date'] <= end_date) & (data['Date'] >= start_date)]
    filtered_data.loc[:, 'Date'] = pd.to_datetime(filtered_data['Date'])
    return filtered_data[['Date', symbol.symbol]]
