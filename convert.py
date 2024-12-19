from moviepy.editor import VideoFileClip
from glob import glob

def convert_to_gif():
    files = glob("logs/videos/*.mp4")
    for file in files:
        videoClip = VideoFileClip(file)
        to_save = file.replace("mp4", "gif")
        videoClip.write_gif(to_save)