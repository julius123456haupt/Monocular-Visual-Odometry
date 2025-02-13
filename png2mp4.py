import cv2
import glob

# Set input folder path
image_folder = "/Volumes/Festplatte hauptmann/dataset/sequences/00/image_0"
output_video = "kitti00.mp4"
fps = 10  # Adjust FPS if needed

# Get all PNG images sorted in numerical order
images = sorted(glob.glob(f"{image_folder}/*.png"))

# Read the first image to get dimensions
frame = cv2.imread(images[0])
height, width, _ = frame.shape

# Define video writer
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec
video = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

# Write each image to video
for img in images:
    frame = cv2.imread(img)
    video.write(frame)

video.release()
cv2.destroyAllWindows()

print(f"Video saved as {output_video}")
