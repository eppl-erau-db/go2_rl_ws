# Axis-aligned bounding box obstacle in the output frame (e.g. base_link).
# position = geometric center of the AABB (halfway between min and max corners).

geometry_msgs/Point position

# Full extents along x, y, z of the AABB (max - min per axis), meters.
float32 width
float32 height
float32 length

# Axis-aligned box volume (width * height * length), cubic meters.
float32 volume

# Surface normals under the obstacle XY footprint (dominant ground / plane).
# Typically one entry: binned-median ground normal, unit length when valid.
geometry_msgs/Vector3[] surface_normals

# YZ-binned robust estimate of the forward-facing (minimum-X) surface point in the
# same frame as position (e.g. base_link). NaN components when unavailable.
geometry_msgs/Point closest_surface_point
