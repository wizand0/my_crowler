fix jpegs
```
python fix_jpegs.py C:\REC_0
```

После восстановления кадры можно склеить как описано в README:

```
ffmpeg -framerate 5 -i REC_0/%05d.jpg -c:v libx264 -pix_fmt yuv420p output.mp4
```
Если ffmpeg жалуется на пропуски в нумерации (после удаления битых), используй с флагом -start_number:

```
ffmpeg -framerate 5 -start_number 1 -i REC_0/%05d.jpg \
       -vf "fps=5" -c:v libx264 -pix_fmt yuv420p \
       -vsync vfr output.mp4
````