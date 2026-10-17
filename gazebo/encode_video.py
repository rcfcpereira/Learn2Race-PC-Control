"""Encode the frames written by video_race.py into an H.264 MP4 (ffmpeg from the imageio-ffmpeg package).

    python gazebo/encode_video.py FRAME_DIR OUT.mp4 [FPS]
"""
import subprocess
import sys

import imageio_ffmpeg


def main():
    frames, out = sys.argv[1], sys.argv[2]
    fps = sys.argv[3] if len(sys.argv) > 3 else "20"
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", "-framerate", fps,
                    "-i", f"{frames}/frame_%05d.png", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
                    "-movflags", "+faststart", out], check=True)
    print(out)


if __name__ == "__main__":
    main()
