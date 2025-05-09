import matplotlib.pyplot as plt

# Define points by color
black_points = [(60.96, -68.58), (124.46, 35.56), (109.22, 35.56)]
red_points   = [(115,-40), (89.69362, 61.05022), (33.74409, 27.97236)]
blue_points  = [(62.28551, -49.56379), (97.85015, 45.8111), (57.36403, 48.44079)]

# Manually define labels
labels = ['1', '2', '3', '1', '2', '3', '1', '2', '3']

# Plot setup
plt.figure(figsize=(8, 8))

# Plot black points as circles
for i, (x, y) in enumerate(black_points):
    plt.scatter(x, y, color='black', label='Ground truth' if i == 0 else "")
    plt.text(x, y + 3, labels[i], ha='center', fontsize=10)

# Plot red points as Xs
for i, (x, y) in enumerate(red_points):
    plt.scatter(x, y, color='red', marker='x', s=100, label='FK estimate' if i == 0 else "")
    plt.text(x, y + 3, labels[i + 3], ha='center', fontsize=10)

# Plot blue points as Xs
for i, (x, y) in enumerate(blue_points):
    plt.scatter(x, y, color='blue', marker='x', s=100, label='Kalman estimate' if i == 0 else "")
    plt.text(x, y + 3, labels[i + 6], ha='center', fontsize=10)

# Axis labels in cm
plt.xlabel('x position (cm)')
plt.ylabel('y position (cm)')

# Grid, title, legend
plt.grid(True)
plt.title('FK vs Kalman Estimates and Ground Truth')
plt.axis('equal')
plt.legend()

plt.show()
