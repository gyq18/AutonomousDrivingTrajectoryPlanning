import cv2 
from tqdm import tqdm


def save_video(image_array, filename, frame_speed=24):
    writer = None
    for img in tqdm(image_array):
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        if writer is None:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(filename, fourcc, frame_speed,
                (img.shape[1], img.shape[0]), True)

        if writer is not None:
            writer.write(img)