# mapserver

This is the mapserver package for storing evidence and raw data maps in an two dimensional grid structure.
Therefore, three different tools are distinguished:

1. `tile_publisher`: Which takes track of the current and past coordinate systems of the map. It sends the proper transforms and names of the current coordinate system.
2. `mapserver_raw`: An actual map stack, which stores any (for now laser and pointcloud messages are supported) data as raw format in a grid
3. `mapserver_stat`: A classical occupancy grid map approach, with the difference in its application approach that only occupancy grid map messages are received and fused.

## `tile_publisher`

The tile publisher publishes transformes from the parent coordinate system `tile_parent_tf` to the origin of the current map tile.
As the vehicle drives through the world, it will at some point preceed the current map dimension.
Therefore, if an inner boundary dimension (defined by `bounding_box_*`) is passed, a new transform for a new map tile is published to the tf tree.
To keep track of the vehicle and the resetting, `odometry_topic` needs to be specified.
Due to the fact, that the transform of the current tile cannot be simply resetted (discontinuities and jumps of frames are not handled by the tf API) a new transform with a new frame name is used.
This name is defined by `tile_origin_tf_prefix`, so that every tranform has a prefix plus and increasing number: `<tile_origin_tf_prefix>#`.
To ease work with the tiles and maps, additionally an tile origin for the current frame can be published.
The is defined by the prefix `tile_origin_tf_sufix_for_roi_origin` so that current tf tree can look like this (`tile_parent_tf`:=map, `tile_origin_tf_prefix`:=tile_, `tile_origin_tf_sufix_for_roi_origin`:= _roi):

* map
  * tile_0
    * tile_0_roi
  * tile_1
    * tile_1_roi
  * ...
  * tile_23
    * tile_23_roi

To inform all nodes what the current tile frame name is and if it has changed (in the upper example it is `tile_23_roi`, the name is advertised via `current_tf_name_topic`.
