from statsmodels.tsa.arima.model import ARIMA
from src.trading_strategies.util.read_file import read_file

import matplotlib.pyplot as plt

def main():
    file_path = '../trading_strategies/data/sp500_adj_close_prices.csv'
    data = read_file(file_path)

    data = data[["Date", "AAPL"]]
    aapl = data[data['Date'] > '2020-01-01']

    train = aapl[0:int(len(aapl)*0.8)]
    test = aapl[int(len(aapl)*0.8):int(len(aapl))]

    arima = ARIMA(train["AAPL"], order=(10, 2, 3))
    arima_res = arima.fit()

    fig, ax = plt.subplots(figsize=(15, 8))
    ax.plot(aapl["AAPL"], label='Actual return')

    # plot the fitted values of model (in sample data predicted values)
    train_pred = arima_res.fittedvalues
    ax.plot(train.index, train_pred, color='green', label='fitted')

    # plot the forecast values of model (out of sample data predicted values)
    prediction_res = arima_res.get_forecast(len(test))
    conf_int = prediction_res.conf_int()
    # lower and upper limits of prediction
    lower, upper = conf_int[conf_int.columns[0]], conf_int[conf_int.columns[1]]
    forecast = prediction_res.predicted_mean
    ax.plot(test.index, forecast, label='forecast')
    ax.fill_between(test.index, lower, upper, color='red', alpha=0.3)
    ax.legend()

    plt.show()

if __name__ == "__main__":
    main()