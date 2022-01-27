# Update Readme

## Update Screenshot

### Record video of app

```bash
sudo add-apt-repository ppa:maarten-baert/simplescreenrecorder
sudo apt-get update
sudo apt-get install simplescreenrecorder

```

open app `SimpleScreenRecorder`

### Convert Video to Gif

Install

```bash
sudo apt install ffmpeg
```

Convert

```bash

FILE_NAME=~/Videos/simplescreenrecorder-2022-01-26_18.09.29.mkv

# resize video down
ffmpeg -y -i $FILE_NAME -filter_complex "scale=600:-1" video.mp4

# convert to gif

ffmpeg -y -i video.mp4 -filter_complex "fps=8,scale=600:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=32[p];[s1][p]paletteuse=dither=bayer" output.gif


```

This lead to a gif with a frame rate of 8fps, and about 6 MB file size.

## Reduce the size with gifsicle

```bash
gifsicle --optimize=3 output.gif -o output_small.gif

```
