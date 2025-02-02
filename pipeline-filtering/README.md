# Pipeline filtering
This package provides a filter performing binary segmentation on images.

## Usage
The package subscribes to an image feed. In order to create this download the `tacc2` bag from Teams and put it in a directory called `bags` inside of your workspace. Then you can play the bag using the following command:
```
ros2 bag play bags/tacc2/ -l
```

The package can be run using the following command:
```
colcon build --packages-select pipeline_filtering --symlink-install
```

Remember so source. Then you can launch the package using the following command:
```
ros2 launch pipeline_filtering pipeline_filtering_launch.py
```

Now binary images will be published to the topic `filtered_image`.

## Tuning
The filter can be tuned a little bit by adjusting the otsu-parameters inside the `params/pipeline_filtering_params.yaml`.

| Parameter | Type | Description |
| --- | --- | --- |
| gamma_auto_correction | bool | Will apply gamma threshold before segmentation when `true`. |
| gamma_auto_correction_weight | float | Weight to increase how 'hard' gamma correction to apply. Default value is 2.0. |
| otsu_segmentation | bool | Set to `true` for applying the binary segmentation. |
