# Velodyne

A python driver for Velodyne lidar.

## `Velodyne` Class

### `__init__` Method

Class constructor.

### `begin` Method

Starts a thread that continuously reads lidar readings and stores them. This method might be merged with the constructor in future.

### `close` Method

Closes the connection with the hardware.

### `scene` Property

Holds a float ndarray of raw readings scaled to match the environment.

### `scene_raw` Property

Holds a integer type ndarray of unscaled raw readings.

### `xyz` Property

Holds a tuple of (x,y,z) values of the readings.

### `plot_2d` Method

Shows the most recent readings in an `imshow` fashion.

### `plot_3d` Method

Shows the most recent readings in a 3-D `scattered` fashion.

### `live_2d` Method

Shows and continuously updates the readings in an `imshow` fashion.

#### Args
- `refresh_rate`: The update frequency of the figure.
- `fig_size`: The size of the figure, similar to matplotlib convention.

### `live_3d` Method

Shows and continuously updates the readings in a 3-D `scattered` fashion.

#### Args
- `refresh_rate`: The update frequency of the figure.
- `fig_size`: The size of the figure, similar to matplotlib convention.
