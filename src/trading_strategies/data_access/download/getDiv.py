import yfinance as yf
import pandas as pd
import ssl

# Disable SSL certificate verification
ssl._create_default_https_context = ssl._create_unverified_context

# Get the list of tickers in the S&P 500 index
table = pd.read_html('https://en.wikipedia.org/wiki/List_of_S%26P_500_companies')
df = table[0]
tickers = df['Symbol'].tolist()

# Create an empty DataFrame to store the dividends
all_dividends = pd.DataFrame()

# Define date range
start_date = "2004-01-01"
end_date = "2024-01-01"

# Loop through each ticker and download the data
for ticker in tickers:
    try:
        # Download data for the current ticker
        GetFacebookInformation = yf.Ticker(ticker)
        data = GetFacebookInformation.dividends

        # Filter dividends within the specified date range
        data = data[(data.index >= start_date) & (data.index < end_date)]

        # Append to the DataFrame
        all_dividends = pd.concat([all_dividends, data.to_frame(name=ticker)], axis=1)
    except Exception as e:
        print(f"Error fetching data for {ticker}: {e}")

# Save the data to a CSV file
csv_filename = "sp500_dividends.csv"
all_dividends.to_csv(csv_filename)
