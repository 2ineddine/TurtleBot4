import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path


def compute_color_ranges(image: np.ndarray, lower_green=(40, 40, 40), upper_green=(90, 255, 255)):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower_green), np.array(upper_green))

    rgb_vals = image[mask > 0]
    hsv_vals = hsv[mask > 0]

    return rgb_vals, hsv_vals


def save_to_csv(rgb_vals, hsv_vals, csv_path):
    df = pd.DataFrame({
        'R': rgb_vals[:, 2], 'G': rgb_vals[:, 1], 'B': rgb_vals[:, 0],
        'H': hsv_vals[:, 0], 'S': hsv_vals[:, 1], 'V': hsv_vals[:, 2]
    })
    df.to_csv(csv_path, index=False)


def plot_3d_colors(rgb_vals, hsv_vals):
    fig = plt.figure(figsize=(12, 6))

    # RGB plot
    ax_rgb = fig.add_subplot(121, projection='3d')
    ax_rgb.scatter(rgb_vals[:, 2], rgb_vals[:, 1], rgb_vals[:, 0], c=rgb_vals / 255.0, s=1)
    ax_rgb.set_xlabel('Red')
    ax_rgb.set_ylabel('Green')
    ax_rgb.set_zlabel('Blue')
    ax_rgb.set_title('RGB Color Space')

    # HSV plot
    ax_hsv = fig.add_subplot(122, projection='3d')
    hsv_normalized = hsv_vals.astype(float)
    hsv_normalized[:, 0] /= 179.0
    hsv_normalized[:, 1:] /= 255.0
    rgb_for_hsv = cv2.cvtColor(hsv_vals[np.newaxis], cv2.COLOR_HSV2RGB).squeeze() / 255.0
    ax_hsv.scatter(hsv_vals[:, 0], hsv_vals[:, 1], hsv_vals[:, 2], c=rgb_for_hsv, s=1)
    ax_hsv.set_xlabel('Hue')
    ax_hsv.set_ylabel('Saturation')
    ax_hsv.set_zlabel('Value')
    ax_hsv.set_title('HSV Color Space')

    plt.tight_layout()
    plt.show()


def process_video_and_collect_colors(video_path: Path, csv_path: Path, frame_step: int = 5):
    cap = cv2.VideoCapture(str(video_path))
    all_rgb_vals = []
    all_hsv_vals = []
    frame_idx = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        if frame_idx % frame_step == 0:
            rgb_vals, hsv_vals = compute_color_ranges(frame)

            if len(rgb_vals) > 0:
                all_rgb_vals.append(rgb_vals)
                all_hsv_vals.append(hsv_vals)

            cv2.imshow("Video Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        frame_idx += 1

    cap.release()
    cv2.destroyAllWindows()

    if all_rgb_vals:
        all_rgb_vals = np.vstack(all_rgb_vals)
        all_hsv_vals = np.vstack(all_hsv_vals)

        save_to_csv(all_rgb_vals, all_hsv_vals, csv_path)
        plot_3d_colors(all_rgb_vals, all_hsv_vals)
    else:
        print("No green pixels detected in the video.")


# Example usage
if __name__ == "__main__":
    video_path = Path("./recorded_frames/green_segment_20250629_040105.avi")  # Replace with your actual path
    csv_output_path = Path("green_pixels.csv")

    process_video_and_collect_colors(video_path, csv_output_path)
