import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec

# Dummy images (replace with your actual images)
img1 = np.full((300, 300, 3), (255, 200, 200), dtype=np.uint8)
img2 = np.full((300, 300, 3), (200, 255, 200), dtype=np.uint8)
img3 = np.full((300, 300, 3), (200, 200, 255), dtype=np.uint8)

# Convert to RGB for matplotlib
img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)
img3 = cv2.cvtColor(img3, cv2.COLOR_BGR2RGB)

# === Flexible layout using GridSpec ===
fig = plt.figure(figsize=(10, 6))
gs = gridspec.GridSpec(2, 3, width_ratios=[1, 0.05, 1])  # 2 rows, 3 columns (2 + spacing + 1)

# Left column: top and bottom images
ax1 = fig.add_subplot(gs[0, 0])  # Top left
ax2 = fig.add_subplot(gs[1, 0])  # Bottom left

# Right column: vertically centered by spanning rows
ax3 = fig.add_subplot(gs[:, 2])  # Entire right column (vertical center)

# Display images
for ax, img, title in zip([ax1, ax2, ax3], [img1, img2, img3], ["Top Left", "Bottom Left", "Right"]):
    ax.imshow(img)
    ax.set_title(title, fontsize=12, fontweight='bold', pad=10, color='darkblue')
    ax.axis('off')

# === Add flexible annotations ===
fig.text(0.25, 0.9, "Custom Annotation A", fontsize=12, color='black', style='italic')
fig.text(0.25, 0.05, "Label near Bottom Left", fontsize=12, color='green')
fig.text(0.75, 0.5, "Right Image Center", fontsize=14, color='red', ha='center', va='center')

plt.tight_layout()
plt.show()
