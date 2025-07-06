import yfinance as yf
import pandas as pd
import ssl

# Disable SSL certificate verification
ssl._create_default_https_context = ssl._create_unverified_context

# Get the list of tickers in the S&P 500 index
table = pd.read_html('https://en.wikipedia.org/wiki/List_of_S%26P_500_companies')
df = table[0]
tickers = df['Symbol'].tolist()

# Create an empty DataFrame to store the adjusted close prices
all_adj_close = pd.DataFrame()

# Loop through each ticker and download the data
for ticker in tickers:
    try:
        # Download data for the current ticker
        data = yf.download(ticker, start="2004-01-01", end="2024-01-01")

        # Extract adjusted close prices
        adj_close = data['Adj Close']

        # Handle Series object if returned instead of DataFrame
        if isinstance(adj_close, pd.Series):
            adj_close = pd.DataFrame(adj_close)

        # Rename column to ticker symbol
        adj_close.columns = [ticker]

        # Concatenate with all_adj_close DataFrame
        all_adj_close = pd.concat([all_adj_close, adj_close], axis=1)
    except Exception as e:
        print(f"Error fetching data for {ticker}: {e}")

# Save the data to a CSV file
csv_filename = "sp500_adj_close_prices.csv"
all_adj_close.to_csv(csv_filename)
