import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

df = pd.read_csv('C:\\Users\\Terry\\Desktop\\TestData\\S&P500Constituents0428.csv')

# Print data
print("Price:")
print(df['Price'])

print("\nPE (TTM):")
print(df['PE (TTM)'])

# Calculate the correlation
correlation = df['Price'].corr(df['PE (TTM)'])

# Define
if correlation > 0:
    print("\n Positive correlated，coefficient:", correlation)
elif correlation < 0:
    print("\n Negative correlated，coefficient:", correlation)
else:
    print("\n no correlation")


df.dropna(inplace=True)

# collect the data
X = df['Price'].values.reshape(-1, 1)
y = df['PE (TTM)'].values

# Create the model
model = LinearRegression()

# Fit the data
model.fit(X, y)

# Get the gradient
slope = model.coef_[0]
print("the slope:", slope)

# Plot
plt.scatter(X, y, color='blue', label='Original data')
plt.plot(X, model.predict(X), color='red', linewidth=2, label='Fitted line')

# Plot
plt.title('Linear Regression')
plt.xlabel('Price')
plt.ylabel('PE (TTM)')
plt.legend()

# Show
plt.show()

# Polynomial Curve Fitting
A = df['Price'].values
B = df['PE (TTM)'].values

degree = 2
coefficients = np.polyfit(A, B, degree)
poly_function = np.poly1d(coefficients)

plt.scatter(A, B, color='blue', label='Original data')

# Fit the data
A_values = np.linspace(np.min(A), np.max(A), 100)
plt.plot(A_values, poly_function(A_values), color='red', label='Polynomial fit')

# Plot
plt.title('Polynomial Fit')
plt.xlabel('Price')
plt.ylabel('PE (TTM)')
plt.legend()

# Show
plt.show()
